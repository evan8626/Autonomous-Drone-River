#include <ArduinoRobot.h> // Want to change to use ROS in the future
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#define rtDelay(v) {vTaskDelay(v/15);}
#define portCHAR char

enum uint_8 {
  Motor0,     // Motor off
  Motor25,    // Motor 25%
  Motor50,    // Motor 50%
  Motor75,    // Motor 75%
  Motor100    // Motor 100%
};

const int pwm = 2;  // DC motor speed control
const int in1 = 22; // DC motor directional pin
const int in2 = 3;  // DC motor directional pin

// may be adding a volatile variable here for interrupts of motion

void setup() {
  // Initailization of communication hardware
  // Initialization of all motors (DC, Servo)
  // Initialization of object dectection hardware (1 LiDar sensor, and 2 ultrasonic senosrs)
  // Set up of tasks

}

void offLoad (){

  // function for offloading collected garbage
  
}

void motion(){

  // Function for when platform is in motion back to dock or to spot where it will collect
   
}

void GPS(){

  // Code for GPS location
  
}

void garbage() {
  // where code for garbage collection will be sitting. Monitoring of sensors/fullness level. Also allows for some small amount of motor control to make sure platform stays put.

}
