#include <Arduino.h>
#include "Headers/Garbage.h"
#include "Headers/UltraSonic.h"
#include "Headers/offload.h"

//Full pins
const int fullPin1 = 15;
const int fullPin2 = 16;
//Full indication
bool fullInd = false;

bool full(){
  //sets a signal from false to true when garbage function detects full signal
  if((fullPin1 && fullPin2) == HIGH){
    fullInd = true;
  }
  else{
    fullInd = false;
  }
}

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
