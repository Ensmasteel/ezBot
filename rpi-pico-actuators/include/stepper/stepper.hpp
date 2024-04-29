#ifndef STEPPER_HPP_
#define STEPPER_HPP_

#include <cstdint>
#include <Arduino.h>



/**
 * This class implements a stepper motor with speed control of the motor
 * and is able to limit acceleration and deceleration.
*/
class stepper{

    public:
        stepper();
        stepper(int stepPin, int dirPin, int enablePin, int cfg1Pin, int cfg2Pin, int maxSpeed, float acceleration);

        /**
         * set the target speed (in steps per second)
        */
        void setTargetSpeed(float targetSpeed);


        /**
         * make a step if it is time to do so
        */
        void run();
    

        int getPosition();

        float getSpeed();

        float getTargetSpeed();

        void setMaxSpeed(int maxSpeed);

        void setAcceleration(float acceleration);

        int getMaxSpeed();

        float getAcceleration();
    
        void log();


    private:
        int _stepPin;
        int _dirPin;
        int _enablePin;
        int _cfg1Pin;
        int _cfg2Pin;
        int _maxSpeed;
        double _acceleration;
        double _speed;
        double _targetSpeed;
        int _position;
        bool accelerating = true;
        long stepstostop;

        float epsilon =15.0;
        /// Arduino pin number assignments for the 2 or 4 pins required to interface to the
        /// stepper motor or driver
        uint8_t        _pin[4];

        /// Whether the _pins is inverted or not
        uint8_t        _pinInverted[4];
    


        typedef enum
        {
        DIRECTION_CCW = 0,  ///< Counter-Clockwise
            DIRECTION_CW  = 1   ///< Clockwise
        } Direction;

        virtual unsigned long  computeNewSpeed();


        virtual void step();

        bool _direction; //1 == CW, 0 == CCW

        unsigned int _minPulseWidth = 1; //in microseconds

        //the current interval time between steps
        unsigned long _stepInterval; //in microseconds

        unsigned long  _lastStepTime;

        /// The step counter for speed calculations
        long _n;

        /// Initial step size in microseconds
        double _c0;

        /// Last step size in microseconds
        double _cn;

        /// Min step size in microseconds based on maxSpeed
        double _cmin; // at max speed
        

};



#endif /* STEPPER_HPP_ */