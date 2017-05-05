  #include "Motor.h"

  //Motor motor;
  int32_t pulses = 1920; // pulses per revolution
  double circumference = 30.9211; // the wheels circumference
  
  void idleMovement(){
  motor.steer(stop);
  }

  void turnRightMovement(){
    motor.steer(right,50);
  }

  void turnLeftMovement(){
    motor.steer(left,50);
  }

  void moveMovement(){
    motor.steer(forward,25);
  }

  void fastMovement(){
    motor.steer(forward,50);
    }

  

