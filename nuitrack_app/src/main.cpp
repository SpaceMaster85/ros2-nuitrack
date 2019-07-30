// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <iomanip>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <nuitrack/Nuitrack.h>
#include "nuitrack_msgs/msg/hands.hpp"

using namespace std::chrono_literals;

using namespace tdv::nuitrack;

void showHelpInfo()
{
	std::cout << "Usage: nuitrack_console_sample [path/to/nuitrack.config]" << std::endl;
}

// Callback for the hand data update event
void onHandUpdate(HandTrackerData::Ptr handData)
{
    if (!handData)
    {
        // No hand data
        std::cout << "No hand data" << std::endl;
        return;
    }

    auto userHands = handData->getUsersHands();
    if (userHands.empty())
    {
        // No user hands
        return;
    }

    auto rightHand = userHands[0].rightHand;
    if (!rightHand)
    {
        // No right hand
        std::cout << "Right hand of the first user is not found" << std::endl;
        return;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Right hand position: "
                 "x = " << rightHand->xReal << ", "
                 "y = " << rightHand->yReal << ", "
                 "z = " << rightHand->zReal << std::endl;
}

int main2()
{
    showHelpInfo();

    // Initialize Nuitrack
    try
    {
        Nuitrack::init("");
    }
    catch (const Exception& e)
    {
        std::cerr << "Can not initialize Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
        return EXIT_FAILURE;
    }
    
    auto handTracker = HandTracker::create();
    handTracker->connectOnUpdate(onHandUpdate);

    // Start Nuitrack
    try
    {
        Nuitrack::run();
    }
    catch (const Exception& e)
    {
        std::cerr << "Can not start Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
        return EXIT_FAILURE;
    }

    int errorCode = EXIT_SUCCESS;
    while (true)
    {
        try
        {
            // Wait for new hand tracking data
            Nuitrack::waitUpdate(handTracker);
        }
        catch (LicenseNotAcquiredException& e)
        {
            std::cerr << "LicenseNotAcquired exception (ExceptionType: " << e.type() << ")" << std::endl;
            errorCode = EXIT_FAILURE;
            break;
        }
        catch (const Exception& e)
        {
            std::cerr << "Nuitrack update failed (ExceptionType: " << e.type() << ")" << std::endl;
            errorCode = EXIT_FAILURE;
        }
    }

    // Release Nuitrack
    try
    {
        Nuitrack::release();
    }
    catch (const Exception& e)
    {
        std::cerr << "Nuitrack release failed (ExceptionType: " << e.type() << ")" << std::endl;
        errorCode = EXIT_FAILURE;
    }

    return errorCode;
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0)
  {

    try {
        Nuitrack::init("");
    }
    catch (const Exception& e) {
        std::cerr << "Can not initialize Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
    }
    handTracker = HandTracker::create();
    handTracker->connectOnUpdate(std::bind(&MinimalPublisher::onHandUpdate, this, std::placeholders::_1));
    // Start Nuitrack
    try {
        Nuitrack::run();
    } catch (const Exception& e) {
        std::cerr << "Can not start Nuitrack (ExceptionType: " << e.type() << ")" << std::endl;
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic");
    timer_ = this->create_wall_timer(
      30ms, std::bind(&MinimalPublisher::nuitrackTimerCallback, this));
  }

private:
  void nuitrackTimerCallback() {
      try{
          std::cerr << "running..." << std::endl;
          tdv::nuitrack::Nuitrack::update();
      }
      catch( const tdv::nuitrack::LicenseNotAcquiredException& ex ){
          rclcpp::shutdown();
      }
  }
  void onHandUpdate(tdv::nuitrack::HandTrackerData::Ptr handData) {
    if (!handData)
    {
        // No hand data
        std::cout << "No hand data" << std::endl;
        return;
    }

    auto userHands = handData->getUsersHands();
    if (userHands.empty())
    {
        // No user hands
        return;
    }

    auto rightHand = userHands[0].rightHand;
    if (!rightHand)
    {
        // No right hand
        std::cout << "Right hand of the first user is not found" << std::endl;
        return;
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Right hand position: "
                 "x = " << rightHand->xReal << ", "
                 "y = " << rightHand->yReal << ", "
                 "z = " << rightHand->zReal << std::endl;
    auto message = std_msgs::msg::String();
    message.data = "Hand seen! " + std::to_string(count_++);
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  tdv::nuitrack::HandTracker::Ptr handTracker;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
