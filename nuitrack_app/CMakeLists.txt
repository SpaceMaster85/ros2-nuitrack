cmake_minimum_required(VERSION 3.5)
project(nuitrack_app)

###
# Modify for your system
# Download the SDK from here: 
#   https://nuitrack.com/
set(NUITRACK_SDK_INCLUDE_DIR ~/nuitrack-sdk/Nuitrack/include)
###

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nuitrack_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  ./include
  ${NUITRACK_SDK_INCLUDE_DIR}
)
link_directories("/usr/local/lib/nuitrack")

add_executable(publisher src/main.cpp src/nuitrack_app.cpp)
ament_target_dependencies(publisher 
    rclcpp 
    std_msgs
    nuitrack_msgs
    sensor_msgs)
target_link_libraries(publisher nuitrack)

install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
