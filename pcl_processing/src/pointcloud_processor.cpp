#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor() : Node("pointcloud_processor_node")
    {
        this->declare_parameter<std::string>("input_topic", "/point_cloud");
        this->declare_parameter<std::string>("output_topic", "/output_point_cloud");

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("output_topic", output_topic_);

        // Subscribe to input pointcloud
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
            input_topic_, 10,
            std::bind(&PointCloudProcessor::pointcloud_callback, this, std::placeholders::_1));

        // Publisher for output pointcloud
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud>(output_topic_, 10);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
        // Convert ROS PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *temp_cloud);

        // Create the filtering object
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(temp_cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*cloud_filtered);

        // Convert PCL PointCloud to ROS PointCloud2
        sensor_msgs::msg::PointCloud2::SharedPtr cloud_filtered_msg(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*cloud_filtered, *cloud_filtered_msg);

        // Publish the data.
        publisher_->publish(*cloud_filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    std::string input_topic_;
    std::string output_topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}

