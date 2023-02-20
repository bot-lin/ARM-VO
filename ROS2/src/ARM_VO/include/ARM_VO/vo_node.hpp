#ifndef ARM_VO_COMPONENT_HPP_
#define ARM_VO_COMPONENT_HPP_

#include "ARM_VO/visibility_control.h"
#include "../../../../include/ARM_VO.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <image_transport/image_transport.hpp>

void getQuaternion(const cv::Mat& R, float Q[]);

class ARM_VO_Node : public rclcpp::Node
{
public:
    ARM_VO_PUBLIC
    explicit ARM_VO_Node(const rclcpp::NodeOptions & options);
private:
    ARM_VO VO;
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    image_transport::Subscriber sub_;
    rmw_qos_profile_t custom_qos_;


};




#endif  