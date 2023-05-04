#include <Arduino.h>
#include "Headers/UltraSonic.h"
#include "Headers/motors.h"
#include "Headers/Garbage.h"
#include <Arduino_FreeRTOS.h>
#define rtDelay(v) {vTaskDelay(v/15);} // setting task delay time
#define portCHAR char // defining portCHAR, used for FreeRTOS task setup
#include <semphr.h> // semaphore header file

// Queue to handle passing of distance measurements between objDetect task and motion task
QueueHandle_t distanceQueue;

//Full pins
const int fullPin1 = 15;
const int fullPin2 = 16;
//Full indication
bool fullInd = false;

//function prototype for tasks. The 'void *pvParameters' may change.
void objDetect(void *pvParameters);

// LiDAR sensor may be subject to change as a hardwired LiDar would be better than a standalone package
// LiDAR not included yet as I don't have any Arduino compatible LiDAR sensors
//Adafruit_VL53L0X lidar = Adafruit_VL53L0X();

void ultraSetup(){
  //Ultrasonic sensor setup
  pinMode(trigpin1, OUTPUT);
  pinMode(echopin1, INPUT);
  pinMode(trigpin2, OUTPUT);
  pinMode(echopin2, INPUT);

  //Full level sensor setup
  pinMode(fullPin1, INPUT);
  pinMode(fullPin2, INPUT);

  //Interrupt setups
  attachInterrupt(digitalPinToInterrupt(echopin1), echoInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echopin2), echoInterrupt2, CHANGE);
}

//Ultrasonic sensor function to determine distance
void ultraSonic_sensor(uint32_t *distance1, uint32_t *distance2){
  long dur1 = 0, dur2 = 0; //timers to determine how long it takes a signal to travel from the ultrasonic sensor and back again

  //ultra sonic 1
  digitalWrite(trigpin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin1, LOW);
  dur1 = pulseIn(echopin1, HIGH);

  //ultra sonic 2
  digitalWrite(trigpin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigpin2, LOW);
  dur2 = pulseIn(echopin2, HIGH);
  
  //calculation of distance in cm (this will change later as we will get longer range ultrasonic sensors that can measure in meters)
  *distance1 = (dur1 * 0.034)/2; // this equation is (time * speed of sound as cm/ms)/ round trip of signal (think of the distance there and the distance back. If we only multiply by 0.034 we're essentially doubling the distance even though the signal only traveled x feet)
  *distance2 = (dur2 * 0.034)/2;
}

void objDetect(void *pvParameters){
  (void) pvParameters;

  for(;;){
    //LiDAR sensors and ultrasonic sensors here
    //LiDAR not included yet as I don't have any Arduino compatible LiDAR sensors
    //...
    uint32_t distance[2];
    ultraSonic_sensor(&distance[0], &distance[1]);
    xQueueSend(distanceQueue, &distance, pdMS_TO_TICKS(500));
    distance[0] = (endTime1 - startTime1) / 58.2;
    distance[1] = (endTime2 - startTime2) / 58.2;
    vTaskDelay(pdMS_TO_TICKS(250)); //250 miliseconds
  }
}
