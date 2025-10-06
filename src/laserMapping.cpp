#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <chrono>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "preprocess.h"

// ======================= 全局变量定义 =======================
#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)

std::mutex mtx_buffer;
std::condition_variable sig_buffer;

std::string lid_topic, imu_topic;
bool is_first_lidar = true;
bool lidar_pushed = false;
bool flg_exit = false;

double time_diff_lidar_to_imu = 0.0;

// 从原版 Fast-LIO 保留的主要对象
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());

// 全局缓存
std::deque<double> time_buffer;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

std::shared_ptr<Preprocess> p_pre(new Preprocess());
std::shared_ptr<ImuProcess> p_imu(new ImuProcess());

// ======================= 信号处理 =======================
void SigHandle(int sig)
{
    flg_exit = true;
    std::cout << "Caught signal " << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

// ======================= LiDAR 回调 =======================
void lidar_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
    std::unique_lock<std::mutex> lock(mtx_buffer);
    double cur_time = get_time_sec(msg->header.stamp);

    static double last_timestamp_lidar = -1.0;
    if (!is_first_lidar && cur_time < last_timestamp_lidar)
    {
        std::cerr << "Lidar timestamp decreased, clearing buffer..." << std::endl;
        lidar_buffer.clear();
    }

    if (is_first_lidar)
        is_first_lidar = false;

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);

    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;

    sig_buffer.notify_all();
}

// ======================= IMU 回调 =======================
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in)
{
    std::unique_lock<std::mutex> lock(mtx_buffer);
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);

    static double last_timestamp_imu = -1.0;
    double timestamp = get_time_sec(msg->header.stamp);

    if (timestamp < last_timestamp_imu)
    {
        std::cerr << "IMU timestamp decreased, clearing buffer..." << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;
    imu_buffer.push_back(msg);
    sig_buffer.notify_all();
}

// ======================= 数据同步 =======================
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty())
        return false;

    if (!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        lidar_pushed = true;
    }

    double lidar_end_time = meas.lidar_beg_time + 0.1;
    double imu_time = get_time_sec(imu_buffer.front()->header.stamp);

    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

// ======================= LaserMappingNode =======================
class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("laser_mapping", options)
    {
        declare_parameter<std::string>("common.lid_topic", "/velodyne_points");
        declare_parameter<std::string>("common.imu_topic", "/imu/data");
        declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);

        get_parameter("common.lid_topic", lid_topic);
        get_parameter("common.imu_topic", imu_topic);
        get_parameter("common.time_offset_lidar_to_imu", time_diff_lidar_to_imu);

        RCLCPP_INFO(this->get_logger(), "Subscribing to LiDAR: %s", lid_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to IMU: %s", imu_topic.c_str());

        sub_pcl_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lid_topic, rclcpp::SensorDataQoS(), lidar_cbk);
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 50, imu_cbk);

        pubLaserCloudFull_ = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 10);
        pubOdomAftMapped_ = create_publisher<nav_msgs::msg::Odometry>("/Odometry", 10);

        timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&LaserMappingNode::main_loop, this));
    }

private:
    void main_loop()
    {
        if (!sync_packages(Measures))
            return;

        p_imu->Process(Measures, kf, feats_undistort);
        state_point = kf.get_x();

        publish_odometry();
        publish_pointcloud();
    }

    void publish_odometry()
    {
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "map";
        odom.child_frame_id = "base_link";
        odom.header.stamp = get_ros_time(Measures.lidar_beg_time);

        odom.pose.pose.position.x = state_point.pos(0);
        odom.pose.pose.position.y = state_point.pos(1);
        odom.pose.pose.position.z = state_point.pos(2);

        Eigen::Quaterniond q(state_point.rot);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        pubOdomAftMapped_->publish(odom);
    }

    void publish_pointcloud()
    {
        if (!feats_undistort || feats_undistort->empty())
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*feats_undistort, cloud_msg);
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = get_ros_time(Measures.lidar_beg_time);
        pubLaserCloudFull_->publish(cloud_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ======================= main =======================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    signal(SIGINT, SigHandle);
    rclcpp::spin(std::make_shared<LaserMappingNode>());
    rclcpp::shutdown();
    return 0;
}
