
#include "nuitrack_app/nuitrack_app.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuitrackApp>());
    rclcpp::shutdown();
    return 0;
}
