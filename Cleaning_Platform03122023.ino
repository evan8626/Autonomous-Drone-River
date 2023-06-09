//#include <Adafruit_VL53L0X.h> // library for LiDar sensor must download Adafruit_VL53L0X latest version
#include <Arduino.h>
#include <Arduino_FreeRTOS.h> // library for FreeRTOS to run on arduino
#include <queue.h>
#include <SPI.h> // SPI communication protocol header
#include <semphr.h> // semaphore header file
#include <Servo.h> //servo header file
#include <PID_v1.h>
#define rtDelay(v) {vTaskDelay(v/15);} // setting task delay time
#define portCHAR char // defining portCHAR, used for FreeRTOS task setup

QueueHandle_t distanceQueue;

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

//passed GPS coordinates from RPi
double initial_latitude = 0;
double initial_longitude = 0;
double designated_latitude = 0;
double designated_longitude = 0;
double current_latitude = 0;
double current_longitude = 0;

//function prototype for tasks. The 'void *pvParameters' may change.
void objDetect(void *pvParameters);

//function prototypes for tasks. The 'void *pvParameters' may change.
void motion(void *pvParameters);

//function prototype for tasks. The 'void *pvParameters' may change.
void stateMgmt(void *pvParameters);

void setup() {
  // Initailization of communication hardware (subject to change, this is the RF24  
  Serial.begin(115200); // setting serial communication rate to 115200 meant for setup of serial comm between Arduino and RPi
  // Wait for initial position from Raspberry Pi
  while (!Serial.available()) {
    delay(100);
  }
//  commenting out for now as I don't have the serial comm set up between the arduino and RPi yet. Nor do I have the GPS set up yet either.
  initial_latitude = read_double_from_serial();
  initial_longitude = read_double_from_serial();

  // Wait for designated position from Raspberry Pi
  while (!Serial.available()) {
    delay(100);
  }
//  commenting out for now as I don't have the serial comm set up between the arduino and RPi yet. Nor do I have the GPS set up yet either.
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

//Reads GPS coords from RPi
double read_double_from_serial() {
  double value;
  char buffer[32];
  int index = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer[index] = '\0';
      break;
    }
    buffer[index++] = c;
  }
  value = atof(buffer);
  return value;
}

//Sets a full indicator from not full to full or vice versa
bool full(){
  //sets a signal from false to true when garbage function detects full signal
  if((fullPin1 && fullPin2) == HIGH){
    fullInd = true;
  }
  else{
    fullInd = false;
  }
}

//Waste management code
void garbage(uint32_t distance1, uint32_t distance2) {
  // where code for garbage collection will be sitting. Monitoring of sensors/fullness level. Also allows for some small amount of motor control to make sure platform stays put.
  // create a while loop here that makes it so the garbage collection stays active so long as the full sensors have not been reached. Need to add in the pins and pinmode for the full level sensors
  while(!fullPin1 || !fullPin2){
    if((distance1 == 200) || (distance2 == 200)){
      control_servo(&distance1, &distance2);
    }
    else if((fullPin1 && fullPin2) == true){
      offLoad();
      break;
    }
  }
}

//function to calculate the angle that the servo should turn
int calc_angle(int32_t distance1, int32_t distance2){
  //clamps the distance between the min and max distance values
  int clamped_distance = constrain(min(distance1, distance2), MIN_DIST, MAX_DIST);
  //Calculates the angle with linear interpolation
  int angle = map(clamped_distance, MIN_DIST, MAX_DIST, MIN_ANGLE, MAX_ANGLE);

  return angle;
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


void offLoad (){

  // function for offloading collected garbage
  //once emptied, will call full() and set signal from true to false
  if (currentState == INIT_POS){
    // some action that has yet to be defined.
  }
}

//Function that will tell the robot it is no longer in its designated location with some range of tolerance
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

void objDetect(void *pvParameters){
  (void) pvParameters;

  for(;;){
    //LiDAR sensors and ultrasonic sensors here
    //LiDAR not included yet as I don't have any Arduino compatible LiDAR sensors
    //...
    uint32_t distance1, distance2;
    ultraSonic_sensor(&distance1, &distance2);
    xQueueSend(distanceQueue, &distance1, pdMS_TO_TICKS(500));
    xQueueSend(distanceQueue, &distance2, pdMS_TO_TICKS(500));
    distance1 = (endTime1 - startTime1) / 58.2;
    distance2 = (endTime2 - startTime2) / 58.2;
    vTaskDelay(pdMS_TO_TICKS(250)); //250 miliseconds
  }
}

void motion(void *pvParameters){

  // Task for when platform is in motion back to dock or to spot where it will collect
  (void) pvParameters;  // This is to avoid compiler warnings about unused parameters
  uint32_t distance1, distance2;
  for(;;) {
    if (xQueueReceive(distanceQueue, &distance1, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (check_for_movement_conditions(distance1, distance2)) {
        control_servo(distance1, distance2);
      }
     }
     else if (xQueueReceive(distanceQueue, &distance2, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (check_for_movement_conditions(distance1, distance2)) {
        control_servo(distance1, distance2);
      }
     }
     //Forgot to add, this will periodically udpate the servo motor even if no new distance data is received
     vTaskDelay(pdMS_TO_TICKS(50));
   }
 }

 void stateMgmt(void *pvParameters){
  (void) pvParameters;

  manager();
  
  for(;;){
    vTaskDelay(pdMS_TO_TICKS(250)); //250 miliseconds
  }
}


void loop(){
}
