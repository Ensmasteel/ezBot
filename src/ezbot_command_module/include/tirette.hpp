#ifndef TIRETTE_HPP
#define TIRETTE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gpiod.h>


class TiretteNode : public rclcpp::Node
{
public:
    TiretteNode();

private:

    int gpioTirette = 22;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tirette_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool init_gpiod(void);
    void readGPIO();
    
    struct gpiod_chip  *gpiochip;
    struct gpiod_line  *gpioline;
};




bool TiretteNode::init_gpiod(void)
{
  gpiochip = gpiod_chip_open_by_name("gpiochip4");
  if(gpiochip == NULL)
      gpiochip = gpiod_chip_open_by_name("gpiochip0");
  if(gpiochip == NULL)
    {
        printf("unable to open GPIO\n");
        return false;
    }
    printf("gpiochip successful = %d\n",gpiochip);

    gpioline = gpiod_chip_get_line(gpiochip,gpioTirette);
    if (gpioline == NULL)
    {
        printf("unable to get GPIO line\n");
        return false;
    }
    printf("gpioTirette successful = %d\n",gpioTirette);
    
    int temp;
    // temp = gpiod_line_request_input_flags(gpioline,"Tirette", GPIOD_LINE_BIAS_PULL_UP);
    
    temp = gpiod_line_request_input(gpioline,"Tirette");
    printf("gpiod_line_request_input_flags = %d\n",temp);
    return true;

}


#endif // TIRETTE_HPP