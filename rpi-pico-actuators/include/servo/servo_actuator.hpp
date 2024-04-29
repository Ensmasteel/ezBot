#ifndef SERVOACTUATOR_HPP
#define SERVOACTUATOR_HPP


#include <Arduino.h>
#include <Servo.h>

#include <array>


class ServoActuator {
    public:
        ServoActuator(int gpioNumber);
        ServoActuator(int gpoNumber, int min, int max, int safe);
        void write(int angle);
    private:
        int gpioNumber;
        int min;
        int max;
        int safe;
        Servo servo;
        int limit(int angle);



};


#endif