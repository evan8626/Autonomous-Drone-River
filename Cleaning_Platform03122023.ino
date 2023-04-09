//#include <Adafruit_VL53L0X.h> // library for LiDar sensor must download Adafruit_VL53L0X latest version
#include <Arduino_FreeRTOS.h> // library for FreeRTOS to run on arduino
#include <SPI.h> // SPI communication protocol header
//#include <RF24.h> // Radio communication header must install RF24 by TMRh20 v 1.4.6 library for this to work
#include <semphr.h> // semaphore header file
#include <Servo.h> //servo header file
#define rtDelay(v) {vTaskDelay(v/15);} // setting task delay time
#define portCHAR char // defining portCHAR, used for FreeRTOS task setup

// Queue to hangle passing of distance measurements between objDetect task and motion task
QueueHandle_t distanceQueue;

//set up of DC motor speeds, motion function will reference this
enum uint_8 {
  Motor0,     // Motor off
  Motor25,    // Motor 25%
  Motor50,    // Motor 50%
  Motor75,    // Motor 75%
  Motor100    // Motor 100%
};

//function prototypes for tasks. The 'void *pvParameters' will change.
void objDetect(void *pvParameters);
void motion(void *pvParameters);
//void GPS(void *pvParameters);

//No Arduino compatible radio transceiver yet
// Create an RF24 object
//RF24 radio(10, 11); // CE, CSN pins

//const uint64_t pipe = 0xE8E8F0F0E1LL; // Address of the radio pipe. This address will more than likely change
//const char text[] = "Hello, world!";

//DC Motor pins
const int pwm = 2;  // DC motor speed control
const int dcMotor1 = 22; // DC motor directional pin
const int dcMotor2 = 3;  // DC motor directional pin

//Servo pin
const int servo1Pin = 9;
Servo servo1; //creates servo1 object
// may be adding a volatile variable here for interrupts of motion

// LiDAR sensor may be subject to change as a hardwired LiDar would be better than a standalone package
// LiDAR not included yet as I don't have any Arduino compatible LiDAR sensors
//Adafruit_VL53L0X lidar = Adafruit_VL53L0X();

//UltraSonic sensor 1 (position: right)
const int trigpin1 = 4;
const int echopin1 = 5;

//UltraSonic sensor 2 (position: left)
const int trigpin2 = 6;
const int echopin2 = 7;

//Angle range
const int MIN_ANGLE = -180;
const int MAX_ANGLE = 180;
const int MAX_DIST = 400; //max distance an ultrasonic sensor can sense. Again, this will probably change because we will use longer range ultrasonic sensors.

//Debugging of ultrasonic sensor. Change the pins based on which sensor to test
//NewPing sonar(trigpin1, echopin1, MAX_DIST);

//state machine
enum robotState{
  INIT_POS,
  DESIGNATED_LOC,
  RETURN_TO_INIT,
  WAIT_FOR_FULL
};

robotState currentState = INIT_POS;

void setup() {
  // Initailization of communication hardware (subject to change, this is the RF24  
  Serial.begin(115200); // setting serial communication rate to 115200 meant for setup of serial comm between Arduino and RPi
  distanceQueue = xQueueCreate(1, sizeof(uint32_t) * 2); // Distance passing queue setup
//  radio.begin(); //starts radio
//  radio.setPALevel(RF24_PA_MIN); //Sets pwr amplification level, subject to change
//  radio.setChannel(76); //Sets RF channel, subject to change
//  radio.openReadingPipe(1, pipe); //Sets addres of radio pipe
//  radio.startListening(); //Sets radio to receive mode
  
  // Initialization of all motors (DC, Servo)
  pinMode(dcMotor1, OUTPUT); //driving motor
  pinMode(dcMotor2, OUTPUT); //driving motor
  servo1.attach(servo1Pin); //rudder control
  
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
  
  // Set up of tasks
  xTaskCreate(objDetect, "Objection Detection Task", 1000, NULL, 1, NULL);
  xTaskCreate(motion, "Motion Task", 1000, NULL, 2, NULL);
  //xTaskCreate(GPS, "Location Task", 128, NULL, 3, NULL);

  // Starting the scheduler. Must be started here otherwise tasks will not run.
  vTaskStartScheduler();
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

//function for PWM control of the servo
void setServoAngle(int angle){
  int pulseWidth = map(angle, 0, 180, 1000, 2000);
  digitalWrite(servo1Pin, HIGH);
  delayMicroseconds(pulseWidth);
  //delayMicroseconds(250000);
  digitalWrite(servo1Pin, LOW);
}

//function to calculate the angle that the servo should turn
int calc_angle(int32_t distance){
  int angle;

  if(distance <= 75){
    angle = 90;
  }
  else if(distance > 75 && distance <= 200){
    angle = map(distance, 76, 200, 35, 45);
  }
  else if(distance > 200 && distance <= 300){
    angle = map(distance, 201, 300, 25, 35);
  }
  else if(distance > 300 && distance <= 350){
    angle = map(distance, 301, 350, 15, 25);
  }
  else if(distance > 350 && distance <= 400){
    angle = map(distance, 351, 400, 5, 15);
  }

  return angle;
}

bool designated_location_reached(){
  //This will be determined via GPS, once GPS module is here and coded a true false variable as well as a 
}

bool init_pos_reached(){
  //This will be determined via GPS, once GPS module is here and coded a true false variable as well as a 
}

bool full(){
  //sets a signal from false to true when garbage function detects full signal
}

bool check_for_movement_conditions(uint32_t distance1, uint32_t distance2){
  //will add in logic that will check state of init pos, designated loc, as well as considering full level
}

//getting close to working. Just need to figure out why my servo motor will only turn to a fixed angle and then back to its basic position after the obstacle has been moved (basic pos = 90 degrees)
void control_servo(uint32_t distance1, uint32_t distance2){
  //int32_t dist_diff;
  int angle;
  const int DEADBAND = 45; 
    if(abs((int32_t)distance1 - (int32_t)distance2) > DEADBAND){
      if(distance1 > distance2){ // So long as the difference between 1 and 2 favors 1, will write the angle to the servo depending on distance to ultrasonic sensor
        angle = 90 + calc_angle(distance1);
        angle = constrain(angle, 0 ,180);
        servo1.write(angle);
        delay(500);
      }
      else if(distance2 > distance1){ // So long as the difference between 1 and 2 favors 2, will write the angle to the servo depending on distance to ultrasonic sensor
        angle = 90 - calc_angle(distance2);
        angle = constrain(angle, 0 ,180);
        servo1.write(angle);
        delay(500);
      }
}
      else if (abs((int32_t)distance1 - (int32_t)distance2) < DEADBAND){
        angle = 90;
        servo1.write(angle);
        //code for DC motor to go in reverse
        delayMicroseconds(100);
      }
      else{
        angle = 90;
        servo1.write(angle);
        delayMicroseconds(100);
      }
  
      // Add debugging lines
      Serial.print("distance1: "); Serial.print(distance1);
      Serial.print(", distance2: "); Serial.print(distance2);
      Serial.print(", angle: "); Serial.println(angle);
      //setServoAngle(angle);
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
    //vTaskDelay to avoid deadlocks
    vTaskDelay(pdMS_TO_TICKS(250)); //250 miliseconds
  }
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

// Commenting this out for the time being as I have no GPS module to hardwire to the Ardiuno. Plus, this may work better on the RPi connection.
//void GPS(void *pvParameters){
//
//  // Code for GPS location
//  (void) pvParameters;
//
//  for(;;){
//    //GPS signaling 
//    //...
//
//    //vTaskDelay to avoid deadlocks
//    vTaskDelay(pdMS_TO_TICKS(100)); //100 miliseconds
//  }
//  
//}

void garbage(uint32_t distance1, uint32_t distance2) {
  // where code for garbage collection will be sitting. Monitoring of sensors/fullness level. Also allows for some small amount of motor control to make sure platform stays put.
  // create a while loop here that makes it so the garbage collection stays active so long as the full sensors have not been reached. Need to add in the pins and pinmode for the full level sensors
  if((distance1 == 75) || (distance2 == 75)){
    control_servo(&distance1, &distance2);
  }
  //add if loop that upon detecting full level sensor will call full() setting signal from false to true and break while loop
}

void offLoad (){

  // function for offloading collected garbage
  //once emptied, will call full() and set signal from true to false
  
}

void loop(){
  uint32_t distance1;
  uint32_t distance2;

  ultraSonic_sensor(&distance1, &distance2);

  switch (currentState) {
    case INIT_POS:
      // Move robot to the designated location
      control_servo(distance1, distance2);
      // Check if the robot has reached the designated location and update the state
      if (designated_location_reached()) {
        currentState = WAIT_FOR_FULL;
      }
      break;

    case DESIGNATED_LOC:
      // Check if there is an object too close to the robot or if it drifted away
      if (check_for_movement_conditions(distance1, distance2)) {
        control_servo(distance1, distance2);
        // Check if the robot has reached the designated location and update the state
        if (designated_location_reached()) {
          currentState = WAIT_FOR_FULL;
        }
      }
      break;

    case RETURN_TO_INIT:
      // Move robot back to the initial position
      control_servo(distance1, distance2);
      // Check if the robot has reached the initial position and update the state
      if (init_pos_reached()) {
        currentState = INIT_POS;
      }
      break;

    case WAIT_FOR_FULL:
      if (full()) {
        currentState = DESIGNATED_LOC;
      }
      break;
  }
}
