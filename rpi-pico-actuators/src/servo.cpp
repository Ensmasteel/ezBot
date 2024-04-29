#include "servo/servo_actuator.hpp"


ServoActuator::ServoActuator(int gpioNumber) {
    this->gpioNumber = gpioNumber;
    this->servo.attach(gpioNumber);
}


ServoActuator::ServoActuator(int gpioNumber, int min, int max, int safe) {
    this->gpioNumber = gpioNumber;
    this->min = min;
    this->max = max;
    this->safe = safe;
    this->servo.attach(gpioNumber);
    this->servo.write(safe);

}

int ServoActuator::limit(int angle) {
    if (angle > max) {
        return max;
    } else if (angle < min) {
        return min;
    } else {
        return angle;
    }
}


void ServoActuator::write(int angle) {
    this->servo.write(limit(angle));
}

