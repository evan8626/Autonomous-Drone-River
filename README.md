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
