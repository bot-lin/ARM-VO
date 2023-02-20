#include "rclcpp/rclcpp.hpp"
#include "ARM_VO/vo_node.hpp"

ARM_VO_Node::ARM_VO_Node(const rclcpp::NodeOptions & options)
: Node("vo_node", options),
    node_(std::make_shared<rclcpp::Node>("arm_vo"))
{
    std::string filename =  this->declare_parameter("config_file", "/data/arm_vo.yaml");
    VO.loadSetting(filename);   
    sub_ = image_transport::create_subscription(
        node_.get(), "camera/image_raw", [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { callback(msg); }, "raw", custom_qos_);

}

void ARM_VO_Node::callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {

    cv::Mat curr_frame = cv_bridge::toCvShare(msg, "mono8")->image;
    if (!VO.initialized)
    {
        VO.init(curr_frame);
    }
    else
    {
        clock_t start = clock();

        VO.update(curr_frame);

        clock_t finish = clock();
        int FPS = 1000 / (1000*(finish-start)/CLOCKS_PER_SEC);

        float quat[4];
        getQuaternion(VO.R_f, quat);

        geometry_msgs::msg::Pose pose_msg;
        pose_msg.position.x = VO.t_f.at<float>(0);
        pose_msg.position.y = VO.t_f.at<float>(1);
        pose_msg.position.z = VO.t_f.at<float>(2);
        pose_msg.orientation.x = quat[0];
        pose_msg.orientation.y = quat[1];
        pose_msg.orientation.z = quat[2];
        pose_msg.orientation.w = quat[3];

        // posePublisher.publish(pose_msg);

        // Results.show(curr_frame, VO.prev_inliers, VO.curr_inliers, FPS, VO.t_f);
    }
    //////

    // if (camera_optical_frame_.empty()) {
    //     camera_optical_frame_ = msg->header.frame_id;
    // }
    // const rclcpp::Time tp_1 = node_->now();
    // const double timestamp = tp_1.seconds();

    // // input the current frame and estimate the camera pose
    // auto cam_pose_wc = slam_->feed_monocular_frame(cv_bridge::toCvShare(msg)->image, timestamp, mask_);

    // const rclcpp::Time tp_2 = node_->now();
    // const double track_time = (tp_2 - tp_1).seconds();

    // //track times in seconds
    // track_times_.push_back(track_time);

    // if (cam_pose_wc) {
    //     publish_pose(*cam_pose_wc, msg->header.stamp);
    // }
}

// -------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------
void getQuaternion(const cv::Mat& R, float Q[])
{
    float trace = R.at<float>(0,0) + R.at<float>(1,1) + R.at<float>(2,2);

    if (trace > 0.0)
    {
        float s = sqrtf(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<float>(2,1) - R.at<float>(1,2)) * s);
        Q[1] = ((R.at<float>(0,2) - R.at<float>(2,0)) * s);
        Q[2] = ((R.at<float>(1,0) - R.at<float>(0,1)) * s);
    }

    else
    {
        int i = R.at<float>(0,0) < R.at<float>(1,1) ? (R.at<float>(1,1) < R.at<float>(2,2) ? 2 : 1) : (R.at<float>(0,0) < R.at<float>(2,2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;

        float s = sqrtf(R.at<float>(i, i) - R.at<float>(j,j) - R.at<float>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<float>(k,j) - R.at<float>(j,k)) * s;
        Q[j] = (R.at<float>(j,i) + R.at<float>(i,j)) * s;
        Q[k] = (R.at<float>(k,i) + R.at<float>(i,k)) * s;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ARM_VO_Node)
