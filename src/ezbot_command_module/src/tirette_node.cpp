#include "tirette.hpp"



void TiretteNode::readGPIO()
{
    auto message = std_msgs::msg::Bool();

    int value;


    value = gpiod_line_get_value(gpioline);
    printf("value = %d\n",value);

    message.data = value ? true : false;

    tirette_pub_->publish(message);

    gpiod_line_release(line);
    gpiod_chip_close(chip);
}
TiretteNode::TiretteNode() : Node("tirette_node")
{

   init_gpiod();
    tirette_pub_ = this->create_publisher<std_msgs::msg::Bool>("tirette_status", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TiretteNode::readGPIO, this));


}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TiretteNode>());
    rclcpp::shutdown();
    return 0;
}