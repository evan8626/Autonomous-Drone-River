#include <Arduino.h>
#include "Headers/stateManagement.h"
#include "Headers/double_from_serial.h"
#include "Headers/UltraSonic.h"
#include "Headers/Garbage.h"
#include <Arduino_FreeRTOS.h>
#define rtDelay(v) {vTaskDelay(v/15);} // setting task delay time
#define portCHAR char // defining portCHAR, used for FreeRTOS task setup
#include <semphr.h> // semaphore header file

//function prototype for tasks. The 'void *pvParameters' may change.
void stateMgmt(void *pvParameters);

//state machine
enum robotState{
  INIT_POS,
  DESIGNATED_LOC,
  RETURN_TO_INIT,
  TRAVEL_TO_DESIGNATED
};
robotState currentState = INIT_POS;

//Full pins
const int fullPin1 = 15;
const int fullPin2 = 16;
//Full indication
bool fullInd = false;

//passed GPS coordinates from RPi
double initial_latitude = 0;
double initial_longitude = 0;
double designated_latitude = 0;
double designated_longitude = 0;
double current_latitude = 0;
double current_longitude = 0;

bool check_for_movement_conditions(uint32_t distance1, uint32_t distance2){
  //will add in logic that will check state of init pos, designated loc, as well as considering full level
}

void manager(){
  uint32_t distance1;
  uint32_t distance2;

  ultraSonic_sensor(&distance1, &distance2);
  // Read current GPS coordinates from Raspberry Pi
  if (Serial.available()) {
    current_latitude = read_double_from_serial();
    current_longitude = read_double_from_serial();
  }
  switch (currentState) {
    case INIT_POS:
      // Move robot to the designated location
      if(!fullPin1 && !fullPin2){
        control_servo(distance1, distance2);
        currentState = TRAVEL_TO_DESIGNATED;
      }
      break;

    case TRAVEL_TO_DESIGNATED:
      // Check if the robot has reached the designated location and update the state
      if ((current_latitude == designated_latitude) && (current_longitude == designated_longitude)) {
          currentState = DESIGNATED_LOC;
      } else {
        control_servo(distance1, distance2);
      }
      break;

    case DESIGNATED_LOC:
      // Check if there is an object too close to the robot or if it drifted away
      if ((!fullPin1 && !fullPin2) && check_for_movement_conditions(distance1, distance2)) {
        control_servo(distance1, distance2);
        // Check if the robot has reached the designated location and update the state
        if (fullPin1 && fullPin2) {
          control_servo(distance1, distance2);
          currentState = RETURN_TO_INIT;
        }
      }
      break;

    case RETURN_TO_INIT:
      // Move robot back to the initial position
      control_servo(distance1, distance2);
      // Check if the robot has reached the initial position and update the state
      if ((current_latitude == initial_latitude) && (current_longitude == initial_longitude)) {
        currentState = INIT_POS;
      }
      break;
  }
}

void stateMgmt(void *pvParameters){
  (void) pvParameters;

  manager();
  
  for(;;){
    vTaskDelay(pdMS_TO_TICKS(250)); //250 miliseconds
  }
}
