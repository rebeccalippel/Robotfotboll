bool shouldAct(){
  if(millis()> 7000){
        return true;
    }
  return false;
  }

void waitRoutine(){
      movementState = idle;
      if(shouldAct()){
        planState = score;
        }
  }



