/* 
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

This is the main program for the soccer playing robot.
*/
#include <Arduino.h>
#include "Motor.h"
#include "Eeprom.h"
#include "BlockDisector.h"
#include "Bluetooth_slave.h"

#if ARDUINO < 156 // Make sure that at least Arduino IDE version 1.5.6 is used
  #error "Please update the Arduino IDE to version 1.5.6 or newer at the following website: http://arduino.cc/en/Main/Software"
#endif


Motor motor{};
Eeprom eeprom{&motor};

double timerComms = 0;

#include "StateMachine.h"

void setup() {

  /* Initialize UART */
  Serial.begin(115200);

  /* Setup buzzer pin */
  motor.initBuzzer();

  /* Read the PID values, target angle and other saved values in the EEPROM */
  if (!eeprom.checkInitializationFlags()) {
    eeprom.readEEPROMValues(); // Only read the EEPROM values if they have not been restored
  } else { // Indicate that the EEPROM values have been reset by turning on the buzzer
    motor.soundBuzzer(1000);
    delay(100); // Wait a little after the pin is cleared
  }

  motor.setupEncoders();
  
  motor.setupMotors();

  motor.setupIMU();


 // tools.printMenu();

  
  motor.calibrateAndReset();
  
  /* Beep to indicate that it is now ready */
  motor.soundBuzzer(100);

  motor.setupTiming();
}
int count1 = 0;
int timer2 = 0;
void loop() {
 // int timer1 = millis();
  motor.checkMotors();
 count1 = count1 +1;
  if(millis()- timerComms > 80){
   // Serial.println(timerComms);
    commsTimer = millis();
    runCommunication();
    timerComms = millis();
    //Serial.println("Count: "+String(count1));
    count1=0;
    }
  runStateMachine();
 
  motor.calculatePitch();

  motor.driveMotors();

  motor.updateEncoders();
//(Serial.print("Hela loopen: ");  
//Serial.println(timer1-timer2);

//timer2=timer1;
}

