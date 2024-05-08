#include "tirette.hpp"



void TiretteNode::readGPIO()
{
    auto message = std_msgs::msg::Bool();

    struct gpiod_chip *chip;
    struct gpiod_line *line;
    int value;

    chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        // handle error
    }

    line = gpiod_chip_get_line(chip, 11);
    if (!line) {
        // handle error
    }

    if (gpiod_line_request_input(line, "tirette_node") < 0) {
        // handle error
    }

    value = gpiod_line_get_value(line);
    if (value < 0) {
        // handle error
    }

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