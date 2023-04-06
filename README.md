# Autonomous-Drone-River
A platform that is collects trash on a river and autonomously returns it to a specified location on land.

Skeleton code added today for microcontroller (03.12.2023)

Added intialization code and several new libraries (03.28.2023)

On master branch, have added tasks for object detection and motion control. Still need to fine tune the timings and add ability for the objDetect task to preempt the motion
task. Have tested the code on the MCU and run it with two ultrasonic sensors and a servo motor. For sometime with testing the servo motor would move based on how close or far 
an object was from one of the ultrasonic sensors but not the other. Additionally, after adding serial monitoring to the motion task to determine distances measured from both 
ultrasonic sensors it became apparent one wasn't reading anything at all, and the other was receiving some strange faulty readings. Will replace these sensors and continue
testing (04.06.2023)
