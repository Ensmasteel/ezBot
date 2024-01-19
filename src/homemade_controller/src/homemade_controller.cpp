
// rasperry pi pico communications
#include "homemade_controller/rppico_comms.hpp"

// ros2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


using namespace std::chrono_literals;

/*
This node is responsible for getting the cmd_vel (from any source) and converting it to speed orders for the rasoerry pi pico
and then sending it to the pico via serial.
Parameters (wheel radius, wheel base, max speed, max acceleration, max deceleration, max angular speed, max angular acceleration, max angular deceleration,
update rate)
 are stored in a yaml file. It is given in the launch file.

*/



class HomemadeController : public rclcpp::Node
{
    public:
        HomemadeController()
        : Node("HomemadeController"){
            // get parameters from the command line arguments or the yaml file
            this->declare_parameter("wheel_diameter", 0.06);
            this->declare_parameter("wheel_base", 0.25);
            this->declare_parameter("max_speed", 1.0);
            this->declare_parameter("max_acceleration", 1.0);
            this->declare_parameter("max_deceleration", 1.0);
            this->declare_parameter("max_angular_speed", 1.0); // rad.s-1
            this->declare_parameter("max_angular_acceleration", 1.0);
            this->declare_parameter("max_angular_deceleration", 1.0);
            this->declare_parameter("update_rate", 50); //Hz
            this->declare_parameter("serial_port", "/dev/ttyACM0");
            this->declare_parameter("serial_baudrate", "115200");
            
            
            timer_ = this->create_wall_timer(
                1000ms, std::bind(&HomemadeController::timer_callback, this));   
            
            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&HomemadeController::cmd_vel_callback, this, std::placeholders::_1));

            }

    void timer_callback(){
        //print the latests /cmd_vel topic values in the console for now
        RCLCPP_INFO(this->get_logger(), "Latest heard: '%f'", linear_x);
    }   

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        // convert the twist message to speed orders for the pico
        // send the speed orders to the pico
        // print the speed orders in the console for now
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear.x);
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->angular.z);
        linear_x = msg->linear.x;



    }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
        RpPicoComs rppico_comms_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        double linear_x;


};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HomemadeController>());
    rclcpp::shutdown();
    return 0;
}   