#include <Arduino.h>

#include <hardware/uart.h>

#include <stepper/stepper.hpp>
#include "commands/commands.hpp"
#include "pins/pins.hpp"

UART Serial2(4,5, NC, NC);

// AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
// AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
// AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);
// AccelStepper stepper4(AccelStepper::DRIVER, STEP_PIN4, DIR_PIN4);


stepper  stepper1 = stepper(STEP_PIN1, DIR_PIN1, ENABLE_PIN, 0, 0, 2000, 1000);
stepper  stepper2 = stepper(STEP_PIN2, DIR_PIN2, ENABLE_PIN, 0, 0, 2000, 1000);
stepper  stepper3 = stepper(STEP_PIN3, DIR_PIN3, ENABLE_PIN, 0, 0, 2000, 1000);
stepper  stepper4 = stepper(STEP_PIN4, DIR_PIN4, ENABLE_PIN, 0, 0, 2000, 1000);

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


  stepper  stepper1 = stepper(STEP_PIN1, DIR_PIN1, ENABLE_PIN, 0, 0, 1000, 10000);
  stepper  stepper2 = stepper(STEP_PIN2, DIR_PIN2, ENABLE_PIN, 0, 0, 1000, 10000);
  stepper  stepper3 = stepper(STEP_PIN3, DIR_PIN3, ENABLE_PIN, 0, 0, 1000, 10000);
  stepper  stepper4 = stepper(STEP_PIN4, DIR_PIN4, ENABLE_PIN, 0, 0, 1000, 10000);


  // pin mode setup
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(CFG1_PIN, OUTPUT);
  pinMode(CFG2_PIN, OUTPUT);


  // enable the stepper drivers
  digitalWrite(ENABLE_PIN, LOW);

  // microstepping option : 
  // Currently 1/8th steps
  // https://wiki.fysetc.com/TMC2208/#motor-current-setting
  // MS1 <-> CFG1
  // MS2 <-> CFG2
  digitalWrite(CFG1_PIN, LOW);
  digitalWrite(CFG2_PIN, LOW);


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
    case MOTOR_SPEEDS:
      Serial.print("MOTOR_SPEEDS ");
      Serial.print(arg1);
      Serial.print(" ");
      Serial.print(arg2);
      Serial.print(" ");
      Serial.print(arg3);
      Serial.print(" ");
      Serial.println(arg4);

    
      //run the actual steppers
      stepper1.setTargetSpeed(arg1);
      stepper2.setTargetSpeed(arg2);
      stepper3.setTargetSpeed(arg3);
      stepper4.setTargetSpeed(arg4);



      break;
    case ACCELERATION:
      Serial.print("ACCELERATION ");
      Serial.print(arg1);

      stepper1.setAcceleration(arg1);
      stepper2.setAcceleration(arg1);
      stepper3.setAcceleration(arg1);
      stepper4.setAcceleration(arg1);
      break;
    case MAX_SPEED:
      Serial.print("MAX_SPEED ");
      Serial.print(arg1);

      stepper1.setMaxSpeed(arg1);
      stepper2.setMaxSpeed(arg1);
      stepper3.setMaxSpeed(arg1);
      stepper4.setMaxSpeed(arg1);
      break;
    // TODO : add a command for sending back the "encoder" values
    default:
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      break;
  }
  return 0;
}


void loop() {


  //run speed each motor

  stepper1.run();
  stepper2.run();
  stepper3.run();
  stepper4.run();

  stepper2.log();

  while(Serial2.available() > 0){
    chr = Serial2.read();

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
