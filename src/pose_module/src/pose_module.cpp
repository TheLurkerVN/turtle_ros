#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

const std::string POSE_TOPIC = "pose_topic";

class pose_publisher : public rclcpp::Node
{
    public:
        pose_publisher(): Node("posepub")
        {
            tf_buffer_= std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_= std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(POSE_TOPIC, 10);
            timer_ = this->create_wall_timer(1s, std::bind(&pose_publisher::onTimer, this));
        }

    private:
        void onTimer()
        {

            geometry_msgs::msg::TransformStamped transformBotToMap;

            try
            {
                transformBotToMap = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "map basefootprint %.3f %.3f", 
                transformBotToMap.transform.translation.x, transformBotToMap.transform.translation.x);

                /*
                transformBotToMap = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "map odom %.3f %.3f", 
                transformBotToMap.transform.translation.x, transformBotToMap.transform.translation.x);

                transformBotToMap = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
                RCLCPP_INFO(this->get_logger(), "odom basefootprint %.3f %.3f", 
                transformBotToMap.transform.translation.x, transformBotToMap.transform.translation.x);
                */
                /*
                transformBotToMap.position.x = 0;
                transformBotToMap.position.y = 0;
                transformBotToMap.position.z = 0;
                transformBotToMap.orientation.x = 0;
                transformBotToMap.orientation.y = 0;
                transformBotToMap.orientation.z = 0;
                transformBotToMap.orientation.w = 1;

                //geometry_msgs::msg::PoseStamped transform;
                auto transform = tf_buffer_->transform(transformBotToMap, "odom");
                */
                publisher_->publish(transformBotToMap);
            }
            catch(const tf2::TransformException & ex)
            {
                RCLCPP_INFO(this->get_logger(),"%s", ex.what());
            }
        }

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pose_publisher>());
    rclcpp::shutdown();
    return 0;
}
