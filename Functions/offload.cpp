#include <Arduino.h>
#include "Headers/offload.h"
#include "Headers/Garbage.h"
#include "Headers/States.h"

void offLoad (){

  // function for offloading collected garbage
  //once emptied, will call full() and set signal from true to false
  if (robotState currentState == INIT_POS && prevState == RETURN_TO_INIT){
    // some action that has yet to be defined.
  }
}
