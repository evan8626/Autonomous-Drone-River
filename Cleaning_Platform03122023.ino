#include <Adafruit_VL53L0X.h> // library for LiDar sensor must download Adafruit_VL53L0X latest version
#include <Arduino_FreeRTOS.h> // library for FreeRTOS to run on arduino
#include <SPI.h> // SPI communication protocol header
#include <RF24.h> // Radio communication header must install RF24 by TMRh20 v 1.4.6 library for this to work
#include <semphr.h> // semaphore header file
#include <Servo.h> //servo header file
#define rtDelay(v) {vTaskDelay(v/15);} // setting task delay time
#define portCHAR char // defining portCHAR, used for FreeRTOS task setup

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
void GPS(void *pvParameters);

// Create an RF24 object
RF24 radio(10, 11); // CE, CSN pins

const uint64_t pipe = 0xE8E8F0F0E1LL; // Address of the radio pipe
const char text[] = "Hello, world!";

//DC Motor pins
const int pwm = 2;  // DC motor speed control
const int dcMotor1 = 22; // DC motor directional pin
const int dcMotor2 = 3;  // DC motor directional pin

//Servo pin
const int servo1Pin = 9;
Servo servo1; //creates servo1 object
// may be adding a volatile variable here for interrupts of motion

// LiDAR sensor may be subject to change as a hardwired LiDar would be better than a standalone package
Adafruit_VL53L0X lidar = Adafruit_VL53L0X();

//UltraSonic sensor 1 (position: right)
const int trigpin1 = 4;
const int echopin1 = 5;

//UltraSonic sensor 2 (position: left)
const int trigpin2 = 6;
const int echopin2 = 7;

void loop(){
  //leave empty required for compilation of Arduino sketch
}

void setup() {
  // Initailization of communication hardware (subject to change, this is the RF24 
  Serial.begin(115200); // setting serial communication rate to 115200 meant for setup of serial comm between Arduino and RPi
  radio.begin(); //starts radio
  radio.setPALevel(RF24_PA_MIN); //Sets pwr amplification level, subject to change
  radio.setChannel(76); //Sets RF channel, subject to change
  radio.openReadingPipe(1, pipe); //Sets addres of radio pipe
  radio.startListening(); //Sets radio to receive mode
  
  // Initialization of all motors (DC, Servo)
  pinMode(dcMotor1, OUTPUT); //driving motor
  pinMode(dcMotor2, OUTPUT); //driving motor
  servo1.attach(servo1Pin); //rudder control
  
  // Initialization of object dectection hardware (1 LiDar sensor, and 2 ultrasonic senosrs)
  //LiDar
  if(!lidar.begin()){
    Serial.println(F("failed to boot")); // if this lidar sensor is used this will become a syslog trace that is sent to the RPi
    while (1);
  }
  Serial.println("lidar is online"); // if this lidar sensor is used this will become a syslog trace that is sent to the RPi

  //Ultrasonic sensor setup
  pinMode(trigpin1, OUTPUT);
  pinMode(echopin1, INPUT);
  pinMode(trigpin2, OUTPUT);
  pinMode(echopin2, INPUT);
  
  // Set up of tasks
  xTaskCreate(objDetect, "Objection Detection Task", 128, NULL, 1, NULL);
  xTaskCreate(motion, "Motion Task", 128, NULL, 2, NULL);
  xTaskCreate(GPS, "Location Task", 128, NULL, 3, NULL);

  // Starting the scheduler
  vTaskStartScheduler();
}

void objDetect(void *pvParameters){
  (void) pvParameters;

  for(;;){
    //LiDAR sensors and ultrasonic sensors here
    //...

    //vTaskDelay to avoid deadlocks
    vTaskDelay(pdMS_TO_TICKS(100)); //100 miliseconds
  }
}

void motion(void *pvParameters){

  // Function for when platform is in motion back to dock or to spot where it will collect
  (void) pvParameters;

  for(;;){
    //Control of DC motors and servo motors here
    //May need to add in call to Objdect? Must look into preemption 
    //...
    servo1.write(0); //writing a 0 to the motor to start in basic position

    //vTaskDelay to avoid deadlocks
    vTaskDelay(pdMS_TO_TICKS(100)); //100 miliseconds
  }
   
}

void GPS(void *pvParameters){

  // Code for GPS location
  (void) pvParameters;

  for(;;){
    //GPS signaling 
    //...

    //vTaskDelay to avoid deadlocks
    vTaskDelay(pdMS_TO_TICKS(100)); //100 miliseconds
  }
  
}

void offLoad (){

  // function for offloading collected garbage
  
}



void garbage() {
  // where code for garbage collection will be sitting. Monitoring of sensors/fullness level. Also allows for some small amount of motor control to make sure platform stays put.

}
