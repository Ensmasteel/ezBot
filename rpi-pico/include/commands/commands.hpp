/* Define single-letter commands that will be sent by the raspberry over the
   serial link.
*/

#ifndef COMMANDS_HPP
#define COMMANDS_HPP

#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define LEFT            0
#define RIGHT           1

#endif


