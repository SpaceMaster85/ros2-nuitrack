#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <nuitrack/Nuitrack.h>

#include "nuitrack_app/parser.h"
#include "nuitrack_app/nuitrack_app.h"

#include "nuitrack_msgs/msg/faces.hpp"
#include "nuitrack_msgs/msg/face_info.hpp"

#include "nuitrack_msgs/msg/hands.hpp"
#include "nuitrack_msgs/msg/hand_info.hpp"

#include "nuitrack_msgs/msg/skeletons.hpp"
#include "nuitrack_msgs/msg/skeleton_info.hpp"
#include "nuitrack_msgs/msg/joint_info.hpp"

#include "nuitrack_msgs/msg/gestures.hpp"
#include "nuitrack_msgs/msg/gesture_info.hpp"

using namespace std::chrono_literals;
using namespace tdv::nuitrack;

NuitrackApp::NuitrackApp() : Node("nuitrack_app"), count_(0)
{
    try
    {
        Nuitrack::init("");
    }
    catch (const Exception &e)
    {
        std::cerr << "Can not initialize Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
    }


      // Realsense Depth Module - force to 848x480 @ 60 FPS
      Nuitrack::setConfigValue("Realsense2Module.Depth.Preset", "5");
      Nuitrack::setConfigValue("Realsense2Module.Depth.RawWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.Depth.RawHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.Depth.ProcessHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.Depth.FPS", "15");

      // Realsense RGB Module - force to 848x480 @ 60 FPS
      Nuitrack::setConfigValue("Realsense2Module.RGB.RawWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.RGB.RawHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.RGB.ProcessWidth", "848");
      Nuitrack::setConfigValue("Realsense2Module.RGB.ProcessHeight", "480");
      Nuitrack::setConfigValue("Realsense2Module.RGB.FPS", "15");

    // Enable Face Module
    tdv::nuitrack::Nuitrack::setConfigValue("Faces.ToUse", "true");
    tdv::nuitrack::Nuitrack::setConfigValue("DepthProvider.Depth2ColorRegistration", "true");



    // Create Hand
    handTracker_ = HandTracker::create();
    handTracker_->connectOnUpdate(std::bind(&NuitrackApp::onHandUpdate, this, std::placeholders::_1));

    // Create Skeleton
    skeletonTracker_ = tdv::nuitrack::SkeletonTracker::create();
    skeletonTracker_->connectOnUpdate(std::bind(&NuitrackApp::onSkeletonUpdate, this, std::placeholders::_1));

    // create Gesture
    gestureRecognizer_ = tdv::nuitrack::GestureRecognizer::create();
    gestureRecognizer_->connectOnNewGestures(std::bind(&NuitrackApp::onNewGestures, this, std::placeholders::_1));

    // Create color sensor
    colorSensor_ = tdv::nuitrack::ColorSensor::create();
    colorSensor_->connectOnNewFrame(std::bind(&NuitrackApp::onNewColorFrame, this, std::placeholders::_1));

    // Create color sensor
    depthSensor_ = tdv::nuitrack::DepthSensor::create();
    depthSensor_->connectOnNewFrame(std::bind(&NuitrackApp::onNewDepthFrame, this, std::placeholders::_1));

    // Start Nuitrack
    try
    {
        Nuitrack::run();
    }
    catch (const Exception &e)
    {
        std::cerr << "Can not start Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
    }

    facePublisher_ = this->create_publisher<nuitrack_msgs::msg::Faces>("nuitrack/faces", 1);
    handPublisher_ = this->create_publisher<nuitrack_msgs::msg::Hands>("nuitrack/hands", 1);
    skeletonPublisher_ = this->create_publisher<nuitrack_msgs::msg::Skeletons>("nuitrack/skeletons", 1);

    gesturePublisher_ = this->create_publisher<nuitrack_msgs::msg::Gestures>("nuitrack/gestures", 1);
    colorImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/color/image", 1);
    depthImagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image", 1);
    depthCloudPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("camera/depth_cloud", 1);


    outputMode_ = depthSensor_->getOutputMode();
    OutputMode colorOutputMode = colorSensor_->getOutputMode();


    // Use depth as the frame size
    frame_width_ = outputMode_.xres;
    frame_height_ = outputMode_.yres;
    int numpoints = frame_width_ * frame_height_;
    cloud_msg_.header.frame_id = "camera_depth_frame_";
    //cloud_msg_.header.stamp = ros::Time::now();
    cloud_msg_.width = numpoints;
    cloud_msg_.height = 1;
    cloud_msg_.is_bigendian = false;
    cloud_msg_.is_dense = false; // there may be invalid points

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(numpoints);

    timer_ = this->create_wall_timer(
        30ms, std::bind(&NuitrackApp::nuitrackTimerCallback, this));
}
NuitrackApp::~NuitrackApp()
{
    try
    {
        std::cerr << "Releasing Nuitrack..." << std::endl;
        Nuitrack::release();
    }
    catch (const Exception &e)
    {
        std::cerr << "Nuitrack release failed (ExceptionType: " << e.type() << ")" << std::endl;
    }
}

void NuitrackApp::nuitrackTimerCallback()
{
    try
    {
        tdv::nuitrack::Nuitrack::update();
    }
    catch (const tdv::nuitrack::LicenseNotAcquiredException &ex)
    {
        rclcpp::shutdown();
    }
    static std::chrono::time_point<std::chrono::system_clock> prevUpdate = std::chrono::system_clock::now();
    auto elapsedTime = std::chrono::system_clock::now() - prevUpdate;
    if (elapsedTime.count() > 1.0 / face_frame_rate)
    {
        onNewFace();
        prevUpdate = std::chrono::system_clock::now();
    }
}

void NuitrackApp::onNewColorFrame(tdv::nuitrack::RGBFrame::Ptr color_frame)
{

    int _width = color_frame->getCols();
    int _height = color_frame->getRows();
    //std::cout << "DBG COLOR:  Width = " << _width << " Height = " << _height << std::endl;

    // Point Cloud message for colorized depth cloud
    int numpoints = _width * _height;
    //cloud_msg_ = new(sensor_msgs::PointCloud2);

    sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud_msg_, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud_msg_, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud_msg_, "b");

    sensor_msgs::msg::Image color_msg;

    const tdv::nuitrack::Color3 *colorPtr = color_frame->getData();

    color_msg.header.stamp = this->get_clock()->now();
    color_msg.header.frame_id = "camera_color_frame_";
    color_msg.height = _height;
    color_msg.width = _width;
    color_msg.encoding = "rgb8"; //sensor_msgs::image_encodings::TYPE_16UC1;
    color_msg.is_bigendian = false;

    color_msg.step = 3 * _width; // sensor_msgs::ImagePtr row step size

    for (size_t row = 0; row < _height; ++row)
    {
        for (size_t col = 0; col < _width; ++col)
        {
            color_msg.data.push_back((colorPtr + col)->red);
            color_msg.data.push_back((colorPtr + col)->green);
            color_msg.data.push_back((colorPtr + col)->blue);

            *out_r = (colorPtr + col)->red; // pointcloud
            *out_g = (colorPtr + col)->green;
            *out_b = (colorPtr + col)->blue;
            ++out_r;
            ++out_g;
            ++out_b;
        }
        colorPtr += _width; // Next row
    }

    // Publish color frame
    colorImagePublisher_->publish(color_msg);
}

void NuitrackApp::onNewDepthFrame(tdv::nuitrack::DepthFrame::Ptr depth_frame)
{

    int width = depth_frame->getCols();
    int height = depth_frame->getRows();
    const uint16_t *depthPtr = depth_frame->getData();
    sensor_msgs::msg::Image depth_msg;
    depth_msg.header.stamp = this->get_clock()->now();
    depth_msg.header.frame_id = "nuitrack_depth_frame";
    depth_msg.height = height;
    depth_msg.width = width;
    depth_msg.encoding = "rgb8"; //sensor_msgs::image_encodings::TYPE_16UC1;
    depth_msg.is_bigendian = false;
    depth_msg.step = width; // sensor_msgs::ImagePtr row step size

    sensor_msgs::PointCloud2Iterator<float> out_x(cloud_msg_, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(cloud_msg_, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(cloud_msg_, "z");

    // std::cout << "=========================================================" << std::endl;
    //std::cout << "DEBUG: cloud x, y, z : world x, y, z " << std::endl;

    for (size_t row = 0; row < height; ++row)
    {
        for (size_t col = 0; col < width; ++col)
        {
            uint16_t fulldepthValue = *(depthPtr + col);
            uint16_t depthValue = *(depthPtr + col) >> 5;

            // RGB are all the same for depth (monochrome)
            depth_msg.data.push_back(depthValue);
            depth_msg.data.push_back(depthValue);
            depth_msg.data.push_back(depthValue);

            //store xyz in point cloud, transforming from image coordinates, (Z Forward to X Forward)
            Vector3 cloud_point = depthSensor_->convertProjToRealCoords(col, row, fulldepthValue);

            float X_World = cloud_point.x / 1000.0; // mm to meters
            float Y_World = cloud_point.y / 1000.0;
            float Z_World = cloud_point.z / 1000.0;

            *out_x = Z_World;
            *out_y = -X_World;
            *out_z = Y_World;
            ++out_x;
            ++out_y;
            ++out_z;
        }
        depthPtr += width; // Next row
    }

    // Publish color frame
    depthImagePublisher_->publish(depth_msg);

    cloud_msg_.header.stamp = this->get_clock()->now();
    depthCloudPublisher_->publish(cloud_msg_);
}

void NuitrackApp::onSkeletonUpdate(tdv::nuitrack::SkeletonData::Ptr skeletonData)
{
    nuitrack_msgs::msg::Skeletons msgSkeleton;
    const std::vector<tdv::nuitrack::Skeleton> skeletons = skeletonData->getSkeletons();
    for (const tdv::nuitrack::Skeleton &skeleton : skeletons)
    {

        nuitrack_msgs::msg::SkeletonInfo skeletonInfo;
        skeletonInfo.id = skeleton.id;
        const std::vector<tdv::nuitrack::Joint> joints = skeleton.joints;
        for (const tdv::nuitrack::Joint &joint : joints)
        {

            nuitrack_msgs::msg::JointInfo jointInfo;
            jointInfo.type = joint.type;
            jointInfo.confidence = joint.confidence;
            jointInfo.real.push_back(joint.real.x);
            jointInfo.real.push_back(joint.real.y);
            jointInfo.real.push_back(joint.real.z);
            jointInfo.projection.push_back(joint.proj.x);
            jointInfo.projection.push_back(joint.proj.y);
            jointInfo.projection.push_back(joint.proj.z);
            jointInfo.orientation.resize(9);
            std::copy(joint.orient.matrix, joint.orient.matrix + 9, jointInfo.orientation.begin());
            skeletonInfo.joints.push_back(jointInfo);
        }
        msgSkeleton.skeletons.push_back(skeletonInfo);
    }
    if (msgSkeleton.skeletons.size())
        skeletonPublisher_->publish(msgSkeleton);
}

void NuitrackApp::onHandUpdate(tdv::nuitrack::HandTrackerData::Ptr handData)
{

    auto msgHands = nuitrack_msgs::msg::Hands();
    const std::vector<UserHands> users_hands = handData->getUsersHands();

    for (const UserHands &user_hands : users_hands)
    {
        nuitrack_msgs::msg::HandInfo handInfo;
        const tdv::nuitrack::Hand::Ptr left_hand = user_hands.leftHand;
        const tdv::nuitrack::Hand::Ptr right_hand = user_hands.rightHand;
        handInfo.id = user_hands.userId;
        if (right_hand)
        {
            handInfo.right_projection.push_back(right_hand->x);
            handInfo.right_projection.push_back(right_hand->y);
            handInfo.right_real.push_back(right_hand->xReal);
            handInfo.right_real.push_back(right_hand->yReal);
            handInfo.right_real.push_back(right_hand->zReal);
            handInfo.right_click = right_hand->click;
            handInfo.right_pressure = right_hand->pressure;
        }
        if (left_hand)
        {
            handInfo.left_projection.push_back(left_hand->x);
            handInfo.left_projection.push_back(left_hand->y);
            handInfo.left_real.push_back(left_hand->xReal);
            handInfo.left_real.push_back(left_hand->yReal);
            handInfo.left_real.push_back(left_hand->zReal);
            handInfo.left_click = left_hand->click;
            handInfo.left_pressure = left_hand->pressure;
        }
        if (right_hand || left_hand)
            msgHands.hands.push_back(handInfo);
    }
    if (msgHands.hands.size())
        handPublisher_->publish(msgHands);
}

void NuitrackApp::onNewFace()
{
    json = parser::parse(tdv::nuitrack::Nuitrack::getInstancesJson());

    nuitrack_msgs::msg::Faces msgFace;
    for (const parser::Human &human : json.humans)
    {
        if (!human.face)
        {
            continue;
        }
        const parser::Face &face = human.face.get();
        nuitrack_msgs::msg::FaceInfo faceInfo;
        faceInfo.id = human.id;
        faceInfo.gender = face.gender;
        faceInfo.age_type = face.age.type;
        faceInfo.age_years = face.age.years;
        faceInfo.emotion_neutral = face.emotions.neutral;
        faceInfo.emotion_angry = face.emotions.angry;
        faceInfo.emotion_happy = face.emotions.happy;
        faceInfo.emotion_surprise = face.emotions.surprise;
        faceInfo.rectangle.push_back(face.rectangle.x);
        faceInfo.rectangle.push_back(face.rectangle.y);
        faceInfo.rectangle.push_back(face.rectangle.width);
        faceInfo.rectangle.push_back(face.rectangle.height);
        faceInfo.left_eye.push_back(face.eyes.left_eye.x);
        faceInfo.left_eye.push_back(face.eyes.left_eye.y);
        faceInfo.right_eye.push_back(face.eyes.right_eye.x);
        faceInfo.right_eye.push_back(face.eyes.right_eye.y);
        faceInfo.angles.push_back(face.angles.yaw);
        faceInfo.angles.push_back(face.angles.pitch);
        faceInfo.angles.push_back(face.angles.roll);
        msgFace.faces.push_back(faceInfo);
    }
    if (msgFace.faces.size())
        facePublisher_->publish(msgFace);
}

void NuitrackApp::onNewGestures(const tdv::nuitrack::GestureData::Ptr gesture_data)
{
    nuitrack_msgs::msg::Gestures msgGestures;
    const std::vector<tdv::nuitrack::Gesture> gestures = gesture_data->getGestures();
    for (const tdv::nuitrack::Gesture &gesture : gestures)
    {

        nuitrack_msgs::msg::GestureInfo gestureInfo;
        gestureInfo.name = type2string(gesture.type);
        if (gestureInfo.name.size())
        {
            gestureInfo.id = gesture.userId;
            msgGestures.gestures.push_back(gestureInfo);
        }
    }
    if (msgGestures.gestures.size())
        gesturePublisher_->publish(msgGestures);
}

// Convert Gesture Type to String
std::string NuitrackApp::type2string(const tdv::nuitrack::GestureType gesture_type)
{
    switch (gesture_type)
    {
    case tdv::nuitrack::GestureType::GESTURE_WAVING:
        return "WAVING";
    case tdv::nuitrack::GestureType::GESTURE_SWIPE_LEFT:
        return "SWIPE LEFT";
    case tdv::nuitrack::GestureType::GESTURE_SWIPE_RIGHT:
        return "SWIPE RIGHT";
    case tdv::nuitrack::GestureType::GESTURE_SWIPE_UP:
        return "SWIPE UP";
    case tdv::nuitrack::GestureType::GESTURE_SWIPE_DOWN:
        return "SWIPE DOWN";
    case tdv::nuitrack::GestureType::GESTURE_PUSH:
        return "PUSH";
    default:
        throw std::runtime_error("failed can't convert gesture type to string");
        return "";
    }
}