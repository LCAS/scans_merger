#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <rclcpp/qos.hpp>

class CloudMergerNode : public rclcpp::Node
{
public:
    CloudMergerNode() : Node("cloud_merger_node")
    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("destination_frame", "base_link");
        this->declare_parameter<std::string>("input_cloud_1", "/front_lidar/points");
        this->declare_parameter<std::string>("input_cloud_2", "/back_lidar/points");
        this->declare_parameter<std::string>("merged_cloud", "/merged_cloud");

        // Get parameter values
        destination_frame_ = this->get_parameter("destination_frame").as_string();
        input_cloud_1_ = this->get_parameter("input_cloud_1").as_string();
        input_cloud_2_ = this->get_parameter("input_cloud_2").as_string();
        merged_cloud_ = this->get_parameter("merged_cloud").as_string();

        // Set up message filters for synchronization
        rclcpp::QoS qos_sub = rclcpp::QoS(rclcpp::SensorDataQoS());

        cloud_sub_1_.subscribe(this, input_cloud_1_, qos_sub.get_rmw_qos_profile());
        cloud_sub_2_.subscribe(this, input_cloud_2_, qos_sub.get_rmw_qos_profile());

        sync_.reset(new Sync(SyncPolicy(10), cloud_sub_1_, cloud_sub_2_));
        sync_->registerCallback(std::bind(&CloudMergerNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        // Publisher for the merged point cloud
        rclcpp::QoS qos = rclcpp::QoS(10);  // Adjust queue size based on your system's need
        qos.best_effort();  // Ensure it matches the reliability requirements
        merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_cloud_, qos);

        // Initialize the TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Cloud merger node started.");
    }

private:
    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud1,
                      const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud2)
    {
        sensor_msgs::msg::PointCloud2 transformed_cloud_1;
        sensor_msgs::msg::PointCloud2 transformed_cloud_2;

        RCLCPP_INFO(this->get_logger(), "Cloud merger node started.");

        auto start = this->get_clock()->now();

        if (transformCloudToTargetFrame(*cloud1, transformed_cloud_1) &&
            transformCloudToTargetFrame(*cloud2, transformed_cloud_2))
        {
            // Convert ROS PointCloud2 messages to PCL types
            pcl::PointCloud<pcl::PointXYZI> pcl_cloud_1;
            pcl::PointCloud<pcl::PointXYZI> pcl_cloud_2;
            pcl::fromROSMsg(transformed_cloud_1, pcl_cloud_1);
            pcl::fromROSMsg(transformed_cloud_2, pcl_cloud_2);

            // Merge the clouds
            pcl::PointCloud<pcl::PointXYZI> merged_cloud = pcl_cloud_1 + pcl_cloud_2;

            // Print the size of the merged cloud
            RCLCPP_INFO(this->get_logger(), "pcl_cloud_1 cloud size: %zu", pcl_cloud_1.size());
            RCLCPP_INFO(this->get_logger(), "pcl_cloud_2 cloud size: %zu", pcl_cloud_2.size());
            RCLCPP_INFO(this->get_logger(), "Merged point cloud size: %zu", merged_cloud.size());

            // Convert back to ROS PointCloud2
            sensor_msgs::msg::PointCloud2 output_cloud;
            pcl::toROSMsg(merged_cloud, output_cloud);
            output_cloud.header.frame_id = destination_frame_;
            output_cloud.header.stamp = transformed_cloud_1.header.stamp;


            // Publish the merged cloud
            merged_cloud_pub_->publish(std::move(output_cloud));

            // map_publisher_->publish(output_cloud);

        }

        auto end = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "Merging took: %f seconds", (end - start).seconds());


    }

    bool transformCloudToTargetFrame(const sensor_msgs::msg::PointCloud2 &input_cloud,
                                     sensor_msgs::msg::PointCloud2 &output_cloud)
    {

        if (!tf_buffer_->canTransform(destination_frame_, input_cloud.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.05))) {
            RCLCPP_WARN(this->get_logger(), "Transform not available yet...");
            return false;
        }

        try
        {
            // Get the transformation from the point cloud's frame to the destination_frame
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                destination_frame_, input_cloud.header.frame_id, tf2::TimePointZero);

            // Apply the transformation
            pcl_ros::transformPointCloud(destination_frame_, transform_stamped, input_cloud, output_cloud);
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform cloud: %s", ex.what());
            return false;
        }
    }

    // Message filter subscribers
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_1_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_2_;

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
    using Sync = message_filters::Synchronizer<SyncPolicy>;
    std::shared_ptr<Sync> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string destination_frame_;
    std::string input_cloud_1_;
    std::string input_cloud_2_;
    std::string merged_cloud_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CloudMergerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
