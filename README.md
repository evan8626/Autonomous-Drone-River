# Autonomous-Drone-River
This project was inspired by the hope and desire for a cleaner and healthier future for our environment. Waste, especially plastic waste, is negatively effecting the great
lakes. This project will initially take place on the Maumee river near Toledo, Ohio which will impact Lake Erie. 

As the project is now, it's nothing more than a proof of concept and a way for me to learn more about project requirements for a robotics clean up project like this. 
No doubt, as I create the first working prototype there will be stumbling blocks I will have to overcome. I am expecting those to be learning how to use FreeRTOS, how to use
ROS, and getting comfortable with being uncomfortable. 

If I am able to make this a prototype that can prove this concept I hope to create a fleet of these robotic platforms in the Maumee to not only collect waste, but hopefully
clean the water to prevent major algae blooms from occurring on Lake Erie. Additionally, I'd be happy to take water data from the Maumee and give it to local water treatment
facilities (if this is something they'd be interested in). Otherwise, I'll publish water quality data of the Maumee online.

As of now, this project is being written in C++ on an Arduino mega which will act as my distributed IO controller. The Arduino is using FreeRTOS to control motors, and respond
to sensors in real-time. I will also be using a Raspberry Pi 3B running ROS to act as the main controller for my robot. This is subject to change as I learn more about best
practices and how I want my robot to function. 

Stay tuned for progress updates.

SIL (software-in-the-loop) testing plan: create a simulated environment to run the code in to see if the tasks of the robot are responding as they 
should. This will include GPS readings, reading from ultrasonic sensors, and readings from LiDAR sensors. SIL testing will also cover the states of the robot to be sure the 
states are changing as they should and when they should.

HIL (hardware-in-the-loop) testing plan: create a mock circuit with cheap off the shelf components for testing the functionality of the robot. If there resources are there 
create a small scale prototype that can be deployed and put into a body of water.

short term: 1 - 2 weeks
medium term: 4 - 5 weeks
long term: 5 - 7 weeks

4/27/2023
Next steps are to finish the offload function (short term), get and program a LiDAR sensor for Arduino (short term - medium term). write a CMake file, a test script, and a CI file to verify functionality (medium term). Write SIL testing env, and create circuit/hardware setup for HIL testing (long term).
