#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
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

        // Subscriber to the two point cloud topics with the matching QoS
        cloud_sub_1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_1_, rclcpp::SensorDataQoS(), std::bind(&CloudMergerNode::cloudCallback1, this, std::placeholders::_1));
        cloud_sub_2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_cloud_2_, rclcpp::SensorDataQoS(), std::bind(&CloudMergerNode::cloudCallback2, this, std::placeholders::_1));

        // Publisher for the merged point cloud
        rclcpp::QoS qos((rclcpp::SystemDefaultsQoS().keep_last(1).durability_volatile()));
        merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(merged_cloud_, qos);

        // Initialize the TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Cloud merger node started.");
    }

private:
    void cloudCallback1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_cloud_1_ = *msg;
        tryMerge();
    }

    void cloudCallback2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_cloud_2_ = *msg;
        tryMerge();
    }

    void tryMerge()
    {
        if (latest_cloud_1_.header.stamp.sec == 0 || latest_cloud_2_.header.stamp.sec == 0)
        {
            // If we haven't received both clouds, skip merging
            return;
        }

        sensor_msgs::msg::PointCloud2 transformed_cloud_1;
        sensor_msgs::msg::PointCloud2 transformed_cloud_2;

        if (transformCloudToTargetFrame(latest_cloud_1_, transformed_cloud_1) &&
            transformCloudToTargetFrame(latest_cloud_2_, transformed_cloud_2))
        {
            // Convert ROS PointCloud2 messages to PCL types
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud_1;
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud_2;
            pcl::fromROSMsg(transformed_cloud_1, pcl_cloud_1);
            pcl::fromROSMsg(transformed_cloud_2, pcl_cloud_2);

            // Merge the clouds
            pcl::PointCloud<pcl::PointXYZ> merged_cloud = pcl_cloud_1 + pcl_cloud_2;

            // Convert back to ROS PointCloud2
            sensor_msgs::msg::PointCloud2 output_cloud;
            pcl::toROSMsg(merged_cloud, output_cloud);
            output_cloud.header.frame_id = destination_frame_;
            output_cloud.header.stamp = this->get_clock()->now();

            // Publish the merged cloud
            merged_cloud_pub_->publish(output_cloud);
        }
    }

    bool transformCloudToTargetFrame(const sensor_msgs::msg::PointCloud2 &input_cloud,
                                     sensor_msgs::msg::PointCloud2 &output_cloud)
    {
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

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;

    sensor_msgs::msg::PointCloud2 latest_cloud_1_;
    sensor_msgs::msg::PointCloud2 latest_cloud_2_;

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
