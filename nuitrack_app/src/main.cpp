#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nuitrack/Nuitrack.h>

#include "nuitrack_app/parser.h"

#include "nuitrack_msgs/msg/faces.hpp"
#include "nuitrack_msgs/msg/face_info.hpp"

#include "nuitrack_msgs/msg/hands.hpp"
#include "nuitrack_msgs/msg/hand_info.hpp"

#include "nuitrack_msgs/msg/skeletons.hpp"
#include "nuitrack_msgs/msg/skeleton_info.hpp"
#include "nuitrack_msgs/msg/joint_info.hpp"


using namespace std::chrono_literals;
using namespace tdv::nuitrack;


class MinimalPublisher : public rclcpp::Node
{
    public:
        MinimalPublisher() : Node("nuitrack_app"), count_(0)
    {
        try {
            Nuitrack::init("");
        }
        catch (const Exception& e) {
            std::cerr << "Can not initialize Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
        }

        // Enable Face Module
        tdv::nuitrack::Nuitrack::setConfigValue( "Faces.ToUse", "true" );
        tdv::nuitrack::Nuitrack::setConfigValue( "DepthProvider.Depth2ColorRegistration", "true" );

        // Create Hand
        handTracker_ = HandTracker::create();
        handTracker_->connectOnUpdate(std::bind(&MinimalPublisher::onHandUpdate, this, std::placeholders::_1));

        // Create Skeleton
        skeletonTracker_ = tdv::nuitrack::SkeletonTracker::create();
        skeletonTracker_->connectOnUpdate(std::bind(&MinimalPublisher::onSkeletonUpdate, this, std::placeholders::_1));

        // Start Nuitrack
        try {
            Nuitrack::run();
        } catch (const Exception& e) {
            std::cerr << "Can not start Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
        }

        facePublisher_ = this->create_publisher<nuitrack_msgs::msg::Faces>("nuitrack/faces");
        handPublisher_ = this->create_publisher<nuitrack_msgs::msg::Hands>("nuitrack/hands");
        skeletonPublisher_ = this->create_publisher<nuitrack_msgs::msg::Skeletons>("nuitrack/skeletons");
        timer_ = this->create_wall_timer(
                30ms, std::bind(&MinimalPublisher::nuitrackTimerCallback, this));
    }
        ~MinimalPublisher() {
            try {
                std::cerr << "Releasing Nuitrack..." << std::endl;
                Nuitrack::release();
            }
            catch (const Exception& e) {
                std::cerr << "Nuitrack release failed (ExceptionType: " << e.type() << ")" << std::endl;
            }
        }

    private:
        void nuitrackTimerCallback() {
            try{
                tdv::nuitrack::Nuitrack::update();
            }
            catch( const tdv::nuitrack::LicenseNotAcquiredException& ex ){
                rclcpp::shutdown();
            }
            static std::chrono::time_point<std::chrono::system_clock> prevUpdate = std::chrono::system_clock::now();
            auto elapsedTime = std::chrono::system_clock::now() - prevUpdate;
            if(elapsedTime.count() > 1.0/face_frame_rate) {
                onNewFace();
                prevUpdate = std::chrono::system_clock::now();
            }
        }

        void onSkeletonUpdate(tdv::nuitrack::SkeletonData::Ptr skeletonData) {
            nuitrack_msgs::msg::Skeletons msgSkeleton;
            const std::vector<tdv::nuitrack::Skeleton> skeletons = skeletonData->getSkeletons();
            for( const tdv::nuitrack::Skeleton& skeleton : skeletons ){

                nuitrack_msgs::msg::SkeletonInfo skeletonInfo;
                skeletonInfo.id = skeleton.id;
                const std::vector<tdv::nuitrack::Joint> joints = skeleton.joints;
                for( const tdv::nuitrack::Joint& joint : joints ){

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
                    std::copy(joint.orient.matrix, joint.orient.matrix+9, jointInfo.orientation.begin());
                    skeletonInfo.joints.push_back(jointInfo);
                }
                msgSkeleton.skeletons.push_back(skeletonInfo);
            }
            if(msgSkeleton.skeletons.size())
                skeletonPublisher_->publish(msgSkeleton);
        }

        void onHandUpdate(tdv::nuitrack::HandTrackerData::Ptr handData) {
            auto msgHands = nuitrack_msgs::msg::Hands();
            const std::vector<UserHands> users_hands = handData->getUsersHands();
            for(const UserHands& user_hands : users_hands ){
                nuitrack_msgs::msg::HandInfo handInfo;        
                const tdv::nuitrack::Hand::Ptr left_hand = user_hands.leftHand;
                const tdv::nuitrack::Hand::Ptr right_hand = user_hands.rightHand;
                handInfo.id = user_hands.userId;
                if(right_hand) {
                    handInfo.right_projection.push_back(right_hand->x);
                    handInfo.right_projection.push_back(right_hand->y);            
                    handInfo.right_real.push_back(right_hand->xReal);
                    handInfo.right_real.push_back(right_hand->yReal);
                    handInfo.right_real.push_back(right_hand->zReal);
                    handInfo.right_click = right_hand->click;
                    handInfo.right_pressure = right_hand->pressure;
                }
                if(left_hand) {
                    handInfo.left_projection.push_back(left_hand->x);
                    handInfo.left_projection.push_back(left_hand->y);
                    handInfo.left_real.push_back(left_hand->xReal);
                    handInfo.left_real.push_back(left_hand->yReal);
                    handInfo.left_real.push_back(left_hand->zReal);
                    handInfo.left_click = left_hand->click;
                    handInfo.left_pressure = left_hand->pressure;
                }
                if(right_hand || left_hand)
                    msgHands.hands.push_back(handInfo);
            }
            if(msgHands.hands.size())
                handPublisher_->publish(msgHands);
        }

        void onNewFace() {
            json = parser::parse( tdv::nuitrack::Nuitrack::getInstancesJson());

            nuitrack_msgs::msg::Faces msgFace;
            for( const parser::Human& human : json.humans ){
                if( !human.face ){
                    continue;
                }
                const parser::Face& face = human.face.get();
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
            if(msgFace.faces.size())
                facePublisher_->publish(msgFace);
        }

    private:
        parser::JSON json;
        rclcpp::TimerBase::SharedPtr timer_;

        // nuitracker publishers
        rclcpp::Publisher<nuitrack_msgs::msg::Faces>::SharedPtr facePublisher_;
        rclcpp::Publisher<nuitrack_msgs::msg::Hands>::SharedPtr handPublisher_;
        rclcpp::Publisher<nuitrack_msgs::msg::Skeletons>::SharedPtr skeletonPublisher_;

        // Nuitrack objects that have associated callbacks
        tdv::nuitrack::HandTracker::Ptr handTracker_;
        tdv::nuitrack::SkeletonTracker::Ptr skeletonTracker_;
        // face update is done in in nuitrackTimerCallback at a standard rate specified by face_frame_rate

        // Params
        double face_frame_rate = 2;
        size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
