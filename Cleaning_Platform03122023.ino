//#include <Adafruit_VL53L0X.h> // library for LiDar sensor must download Adafruit_VL53L0X latest version
#include <Arduino.h>
#include "Headers/double_from_serial.h"
#include "Headers/UltraSonic.h"
#include "Headers/Garbage.h"
#include "Headers/offload.h"
#include "Headers/stateManagement.h"
#include "Headers/motors.h"
#include <Arduino_FreeRTOS.h> // library for FreeRTOS to run on arduino
#include <SPI.h> // SPI communication protocol header
#include <semphr.h> // semaphore header file
#include <Servo.h> //servo header file
#include <PID_v1.h>
#define rtDelay(v) {vTaskDelay(v/15);} // setting task delay time
#define portCHAR char // defining portCHAR, used for FreeRTOS task setup

//Full pins
const int fullPin1 = 15;
const int fullPin2 = 16;
//Full indication
bool fullInd = false;

//UltraSonic sensor 1 (position: right)
const int trigpin1 = 4;
const int echopin1 = 5;

//UltraSonic sensor 2 (position: left)
const int trigpin2 = 6;
const int echopin2 = 7;

volatile unsigned long startTime1, startTime2, endTime1, endTime2;
volatile bool echoState1, echoState2;

//Angle range
const int MIN_ANGLE = -180;
const int MAX_ANGLE = 180;
const int MAX_DIST = 425; //max distance an ultrasonic sensor can sense. Again, this will probably change because we will use longer range ultrasonic sensors.
const int MIN_DIST = 5;

//allowing interruption for simultaneous reading of ultrasonic sensors
void echoInterrupt1();
void echoInterrupt2();

//distance parameters for the ultrasonic header
const int MIN_ANGLE = -180;
const int MAX_ANGLE = 180;
const int MAX_DIST = 425; //max distance an ultrasonic sensor can sense. Again, this will probably change because we will use longer range ultrasonic sensors.
const int MIN_DIST = 5;

//DC Motor pins
const int pwm = 2;  // DC motor speed control
const int dcMotor1 = 22; // DC motor directional pin
const int dcMotor2 = 3;  // DC motor directional pin

//Servo pin
const int servo1Pin = 9;
Servo servo1; //creates servo1 object

//PID Controller
double setpoint;
double input;
double output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;

PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

//state machine
enum robotState{
  INIT_POS,
  DESIGNATED_LOC,
  RETURN_TO_INIT,
  TRAVEL_TO_DESIGNATED
};
robotState currentState = INIT_POS;

void setup() {
  // Initailization of communication hardware (subject to change, this is the RF24  
  Serial.begin(115200); // setting serial communication rate to 115200 meant for setup of serial comm between Arduino and RPi
  // Wait for initial position from Raspberry Pi
  while (!Serial.available()) {
    delay(100);
  }
  initial_latitude = read_double_from_serial();
  initial_longitude = read_double_from_serial();

  // Wait for designated position from Raspberry Pi
  while (!Serial.available()) {
    delay(100);
  }
  designated_latitude = read_double_from_serial();
  designated_longitude = read_double_from_serial();
  distanceQueue = xQueueCreate(1, sizeof(uint32_t) * 2); // Distance passing queue setup
  
  // Initialization of all motors (DC, Servo)
  pinMode(dcMotor1, OUTPUT); //driving motor
  pinMode(dcMotor2, OUTPUT); //driving motor
  servo1.attach(servo1Pin); //rudder control

  //set up of PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(100); // 100 ms sample time
  myPID.SetOutputLimits(0, 255);
  
  // Initialization of object dectection hardware (1 LiDar sensor, and 2 ultrasonic senosrs)
  //LiDar 
  // LiDAR not included yet as I don't have any Arduino compatible LiDAR sensors
//  if(!lidar.begin()){
//    Serial.println(F("failed to boot")); // if this lidar sensor is used this will become a syslog trace that is sent to the RPi
//    while (1);
//  }
//  Serial.println("lidar is online"); // if this lidar sensor is used this will become a syslog trace that is sent to the RPi

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
  
  // Set up of tasks
  xTaskCreate(objDetect, "Objection Detection Task", 1000, NULL, 1, NULL);
  xTaskCreate(motion, "Motion Task", 1000, NULL, 2, NULL);
  xTaskCreate(stateMgmt, "State Management", 1000, NULL, 3, NULL);

  // Starting the scheduler. Must be started here otherwise tasks will not run.
  vTaskStartScheduler();
}

void loop(){
}
