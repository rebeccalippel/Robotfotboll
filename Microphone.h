/*
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

This class reads the input from the microphone and toggles the state
of the robot between on and paused.
Example of use:
  Include this file, #include Microphone.h
  Create an instance of the class, Microphone microphone{};

In the main loop of the Balanduino program:
  microphone.readMic();
  if(microphone.robotOn)
    'Calculate and give control input to the robot'
  else
    'Stop the robot so that it is balancing on the spot'
*/
#ifndef _microphone_h_
#define _microphone_h_

#include <stdint.h>

#include <Arduino.h>

class Microphone {
  
  uint32_t micTimer = 0;  //Timer that keeps track of when the mic input should be read.
  uint16_t micCounter = 0; //Counting number of times a loud sound is recorded.
  uint16_t micResetCounter = 0; //Counts number of times a quiet or no sound is recorded.
public:
  bool robotOn = false;
  void readMic() {
    if(millis()-micTimer>100){
      int val=analogRead(0);   //connect mic sensor to Analog 0 (A0)
      if(val>50) {
        micCounter++;
        if(micCounter>1){
          if(robotOn) {
            robotOn = false;
          }
          else {
            robotOn = true;
          }
          micCounter=0;
        }
        micResetCounter=0;
      }
      else {
        micResetCounter++;
        if(micResetCounter>10){
          micCounter=0;
        }
      }
      //Serial.println(val,DEC);//print the sound value to serial **For debugging**
      micTimer = millis();
    }
  }

};

#endif

