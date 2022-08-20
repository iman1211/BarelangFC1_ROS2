#include "rclcpp/rclcpp.hpp"

class player : public rclcpp::Node
{
public:
    player() : Node("robot_player")
    {
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<player>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}