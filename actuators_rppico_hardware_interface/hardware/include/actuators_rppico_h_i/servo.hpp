#ifndef ACTUATOR_SERVO_HPP
#define ACTUATOR_SERVO_HPP

#include <string>
#include <cmath>


class Servo
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double angle = 0;

    Servo() = default;

    Servo(const std::string &servo_name, int servo_number)
    {
      setup(servo_name, servo_number);
    }

    
    void setup(const std::string &servo_name, int servo_number)
    {
      name = servo_name;
      servo_number = servo_number;
    }


    private:
    int servo_number = 0;
};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
