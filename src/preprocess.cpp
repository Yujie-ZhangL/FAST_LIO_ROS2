#include "preprocess.h"
#include <pcl/common/common.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#define RETURN0 0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess() : feature_enabled(0), lidar_type(VELO16), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;
  N_SCANS = 16;
  SCAN_RATE = 10;
  group_size = 8;
  disA = 0.01;
  disB = 0.1;
  p2l_ratio = 225;
  limit_maxmid = 6.25;
  limit_midmin = 6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;
  given_offset_time = false;

  jump_up_limit = cos(jump_up_limit / 180 * M_PI);
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);
  cos160 = cos(cos160 / 180 * M_PI);
  smallp_intersect = cos(smallp_intersect / 180 * M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  pcl::fromROSMsg(*msg, *pcl_out);
}

void Preprocess::process(const sensor_msgs::msg::PointCloud2::UniquePtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)
  {
  case SEC:
    time_unit_scale = 1.e3f;
    break;
  case MS:
    time_unit_scale = 1.f;
    break;
  case US:
    time_unit_scale = 1.e-3f;
    break;
  case NS:
    time_unit_scale = 1.e-6f;
    break;
  default:
    time_unit_scale = 1.f;
    break;
  }

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;
  case VELO16:
    velodyne_handler(msg);
    break;
  case MID360:
    mid360_handler(msg);
    break;
  default:
    default_handler(msg);
    break;
  }
  *pcl_out = pl_surf;
}

void Preprocess::oust64_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<pcl::PointXYZI> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  if (plsize == 0)
    return;

  pl_surf.reserve(plsize);
  for (uint i = 0; i < plsize; i++)
  {
    if (i % point_filter_num != 0)
      continue;
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    pl_surf.push_back(added_pt);
  }
}

void Preprocess::velodyne_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<pcl::PointXYZI> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;

  pl_surf.reserve(plsize);
  for (int i = 0; i < plsize; i++)
  {
    if (i % point_filter_num != 0)
      continue;

    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = 0.0;
    if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
    {
      pl_surf.push_back(added_pt);
    }
  }
}

void Preprocess::mid360_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  // for simplicity, treat as generic point cloud
  default_handler(msg);
}

void Preprocess::default_handler(const sensor_msgs::msg::PointCloud2::UniquePtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();

  pcl::PointCloud<pcl::PointXYZI> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;

  pl_surf.reserve(plsize);
  for (uint i = 0; i < plsize; ++i)
  {
    PointType added_pt;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    added_pt.curvature = 0.0;

    if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
    {
      pl_surf.push_back(added_pt);
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "lidar";
  output.header.stamp = ct;
}
