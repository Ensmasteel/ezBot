#include <Arduino.h>

#include <hardware/uart.h>

#include <servo/servo_actuator.hpp>
#include "commands/commands.hpp"
#include "pins/pins.hpp"

UART Serial2(4,5, NC, NC);

ServoActuator servo1(servoGpioNumbers[1], 20, 180, 90);

ServoActuator servos[] = {servo1};


int arg = 0;
int i = 0;

char chr;
char cmd;

char argv1[16];
char argv2[16];
char argv3[16];
char argv4[16];

// converted arguments
long arg1;
long arg2;
long arg3;
long arg4;



void setup() {

  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.println("Hello World !");



}

void resetCommand(){
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  memset(argv4, 0, sizeof(argv4));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0;
  arg4 = 0;
  arg = 0;
  i = 0;
}

int runCommand(){

  char *p = argv1;
  char *str;

  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  arg3 = atoi(argv3);
  arg4 = atoi(argv4);

  switch(cmd){
    case SERVO_CMD:
      Serial.print("SERVO number : ");
      Serial.print(arg1);
      Serial.print(" angle : ");
      Serial.print(arg2);

      if (arg1 >= 1 && arg1 <= 12){
        servos[arg1-1].write(arg2);
      }
      else {
        Serial.print("Invalid servo number: ");
        Serial.println(arg1);
      }

      break;


    default:
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      break;
  }
  return 0;
}


void loop() {


  //run speed each motor

  while(Serial.available() > 0){
    chr = Serial.read();

    // if the chr is a carriage return
    if (chr == '\r'){
      if (arg == 1) argv1[i] = NULL;
      else if (arg == 2) argv2[i] = NULL;
      else if (arg == 3) argv3[i] = NULL;
      else if (arg == 4) argv4[i] = NULL;
      runCommand();
      resetCommand();
    }

    else if (chr == ' '){
      if (arg == 0) arg = 1;
      else if (arg == 1) arg = 2;
      else if (arg == 2) arg = 3;
      else if (arg == 3) arg = 4;
      else if (arg == 4){
      }
      i = 0;
    } 
    else {
      if (arg == 0){
        // the first argument is the single letter command
        cmd = chr;
      }
      else if (arg == 1){
        // subsequent arguemnts are numbers and thus can be multiple characters
        argv1[i] = chr;
        i++;
      }
      else if (arg == 2){
        argv2[i] = chr;
        i++;
      }
      else if (arg == 3){
        argv3[i] = chr;
        i++;
      }
      else if (arg == 4){
        argv4[i] = chr;
        i++;
      }
    }
  }
}
