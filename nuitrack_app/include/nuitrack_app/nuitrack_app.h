#ifndef _nuitrack_app_
#define _nuitrack_app_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "nuitrack_app/parser.h"

#include "nuitrack_msgs/msg/faces.hpp"
#include "nuitrack_msgs/msg/face_info.hpp"

#include "nuitrack_msgs/msg/hands.hpp"
#include "nuitrack_msgs/msg/hand_info.hpp"

#include "nuitrack_msgs/msg/gestures.hpp"
#include "nuitrack_msgs/msg/gesture_info.hpp"

#include "nuitrack_msgs/msg/skeletons.hpp"
#include "nuitrack_msgs/msg/skeleton_info.hpp"
#include "nuitrack_msgs/msg/joint_info.hpp"

#include <nuitrack/Nuitrack.h>

class NuitrackApp : public rclcpp::Node

{
public:
    NuitrackApp();

    virtual ~NuitrackApp();

protected:
    void nuitrackTimerCallback();
    void onNewGestures(const tdv::nuitrack::GestureData::Ptr gesture_data);
    void onNewColorFrame(tdv::nuitrack::RGBFrame::Ptr color_frame);
    void onNewDepthFrame(tdv::nuitrack::DepthFrame::Ptr depth_frame);
    void onHandUpdate(tdv::nuitrack::HandTrackerData::Ptr handData);
    void onSkeletonUpdate(tdv::nuitrack::SkeletonData::Ptr skeletonData);
    void onNewFace();

private:
    std::string type2string(const tdv::nuitrack::GestureType gesture_type);

private:
    parser::JSON json;
    rclcpp::TimerBase::SharedPtr timer_;
    // nuitracker publishers
    rclcpp::Publisher<nuitrack_msgs::msg::Faces>::SharedPtr facePublisher_;
    rclcpp::Publisher<nuitrack_msgs::msg::Hands>::SharedPtr handPublisher_;
    rclcpp::Publisher<nuitrack_msgs::msg::Skeletons>::SharedPtr skeletonPublisher_;
    rclcpp::Publisher<nuitrack_msgs::msg::Gestures>::SharedPtr gesturePublisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorImagePublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthImagePublisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depthCloudPublisher_;

    tdv::nuitrack::OutputMode outputMode_;

    // Nuitrack objects that have associated callbacks
    tdv::nuitrack::HandTracker::Ptr handTracker_;
    tdv::nuitrack::SkeletonTracker::Ptr skeletonTracker_;
    tdv::nuitrack::GestureRecognizer::Ptr gestureRecognizer_;

    tdv::nuitrack::ColorSensor::Ptr colorSensor_;
    tdv::nuitrack::DepthSensor::Ptr depthSensor_;

    tdv::nuitrack::GestureData::Ptr gestureData;
    // face update is done in in nuitrackTimerCallback at a standard rate specified by face_frame_rate

    sensor_msgs::msg::PointCloud2 cloud_msg_;

    int frame_width_, frame_height_;
    // Params

    double face_frame_rate = 2;
    size_t count_;
};

#endif //_nuitrack_app_