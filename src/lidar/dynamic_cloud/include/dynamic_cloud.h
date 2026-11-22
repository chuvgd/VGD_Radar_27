#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include <pcl/impl/point_types.hpp>
#include <rclcpp/logging.hpp>
#include <vector>
#include <chrono>
#include <pcl/kdtree/kdtree_flann.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl/filters/passthrough.h>

namespace tdt_radar{
class DynamicCloud : public rclcpp::Node
{
    public:
    DynamicCloud(const rclcpp::NodeOptions& node_options);
    ~DynamicCloud(){}
    
    private:
    int accumulate_time = 3;
    int accumulate_count = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud; // RM2026.pcd 加载+体素降采样后的静态地图
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_; // 动态点（车）的 3 帧缓存队列
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> other_accumulated_clouds_; // 飞机区域点的 3 帧缓存队列
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;// 订阅 /livox/lidar（原始点云）
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;// 发布 /dynamic_cloud（动态点云）
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fly_pub_;// 发布 /fly_cloud（飞机区域点云）
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void GetDynamicCloud(pcl::PointCloud<pcl::PointXYZ> &input_cloud,pcl::PointCloud<pcl::PointXYZ> &output_cloud,float threshold,int thread_num);
    pcl::KdTreeFLANN<pcl::PointXYZ> kd_Tree; // 构造时用地图建索引，运行时做 nearestKSearch
    tf2_ros::Buffer tf_buffer_; 
    tf2_ros::TransformListener tf_listener_;
};
}//namespace tdt_radar
