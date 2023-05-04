#include <Arduino.h>
#include <Arduino_FreeRTOS.h> // library for FreeRTOS to run on arduino
#include "Headers/motors.h"
#include "Headers/UltraSonic.h"
#include "Headers/stateManagement.h"
#include <semphr.h> // semaphore header file
#include <Servo.h> //servo header file
#include <PID_v1.h>
#define rtDelay(v) {vTaskDelay(v/15);} // setting task delay time
#define portCHAR char // defining portCHAR, used for FreeRTOS task setup

//function prototypes for tasks. The 'void *pvParameters' may change.
void motion(void *pvParameters);

//function to calculate the angle that the servo should turn
int calc_angle(int32_t distance1, int32_t distance2){
  //clamps the distance between the min and max distance values
  int clamped_distance = constrain(min(distance1, distance2), MIN_DIST, MAX_DIST);
  //Calculates the angle with linear interpolation
  int angle = map(clamped_distance, MIN_DIST, MAX_DIST, MIN_ANGLE, MAX_ANGLE);

  return angle;
}

//getting close to working. Just need to figure out why my servo motor will only turn to a fixed angle and then back to its basic position after the obstacle has been moved (basic pos = 90 degrees)
void control_servo(uint32_t distance1, uint32_t distance2){

  int dcSpeed = map(min(distance1, distance2), 0, 250, 255, 127);
  int servoAngle = calc_angle(distance1, distance2);
  servo1.write(servoAngle);
  input = servoAngle;
  setpoint = 90;

  myPID.Compute();
  // Set motor direction (forward)
  digitalWrite(dcMotor1, HIGH);
  digitalWrite(dcMotor2, LOW);

  // Set motor speed based on the PID output
  analogWrite(pwm, output);
  
  // Add debugging lines
  Serial.print("distance1: "); Serial.print(distance1);
  Serial.print(", distance2: "); Serial.print(distance2);
  Serial.print(", angle: "); Serial.println(servoAngle);
  //setServoAngle(angle);
}

void motion(void *pvParameters){

  // Task for when platform is in motion back to dock or to spot where it will collect
  (void) pvParameters;  // This is to avoid compiler warnings about unused parameters
  uint32_t distance[2];
  for(;;) {
    if (xQueueReceive(distanceQueue, &distance, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (check_for_movement_conditions(distance[0], distance[1])) {
        control_servo(distance[0], distance[1]);
      }
     }
     //Forgot to add, this will periodically udpate the servo motor even if no new distance data is received
     vTaskDelay(pdMS_TO_TICKS(50));
   }
 }
