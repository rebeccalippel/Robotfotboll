/*
Original code by 2013-2015 Kristian Lauszus, TKJ Electronics.

Modified by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").
*/

#include "Tools.h"

#include <Arduino.h>

#include "I2C.h"
#include "EEPROM.h"
#include "Motor.h"


#define VBAT A5 // Not broken out - used for battery voltage measurement

uint8_t batteryCounter = 0; // Counter used to check if it should check the battery level

void Tools::checkSerialData() {
  if (Serial.available()) {
    int input = Serial.read();
    if (input == 'm')
      printMenu();
    else {
      dataInput[0] = static_cast<char> (input); // Intentional cast
      delay(2); // Wait for rest of data
      uint8_t i = 1;
      while (1) {
        input = Serial.read();
        if (input == -1) // Error while reading the string
          return;
        dataInput[i] = static_cast<char> (input); // Intentional cast
        if (dataInput[i] == ';') // Keep reading until it reads a semicolon
          break;
        if (++i >= sizeof(dataInput) / sizeof(dataInput[0])) // String is too long
          return;
      }
      bluetoothData = false;
      setValues(dataInput);
    }
  }
}

void Tools::printMenu() {
  Serial.println(F("\r\n========================================== Menu ==========================================\r\n"));

  Serial.println(F("m\t\t\t\tSend to show this menu\r\n"));

  Serial.println(F("A;\t\t\t\tSend to abort. Send 'C' again to continue\r\n"));

  Serial.println(F("AC;\t\t\t\tSend to calibrate the accelerometer"));
  Serial.println(F("MC;\t\t\t\tSend to calibrate the motors\r\n"));

  Serial.println(F("GP;\t\t\t\tGet PID values"));
  Serial.println(F("GK;\t\t\t\tGet Kalman filter values"));
  Serial.println(F("GS;\t\t\t\tGet settings values"));
  Serial.println(F("GI;\t\t\t\tGet info values\r\n"));

  Serial.println(F("SP,Kp;\t\t\t\tUsed to set the Kp value"));
  Serial.println(F("SI,Ki;\t\t\t\tUsed to set the Ki value"));
  Serial.println(F("SD,Kd;\t\t\t\tUsed to set the Kd value"));
  Serial.println(F("ST,targetAngle;\t\t\tUsed to set the target angle"));
  Serial.println(F("SK,Qangle,Qbias,Rmeasure;\tUsed to set the Kalman filter values"));
  Serial.println(F("SA,angle;\t\t\tUsed to set the maximum controlling angle"));
  Serial.println(F("SU,value;\t\t\tUsed to set the maximum turning value"));
  Serial.println(F("SB,value;\t\t\tUsed to set the back to spot value (true = 1, false = 0)\r\n"));

  Serial.println(F("IB;\t\t\t\tStart sending IMU values"));
  Serial.println(F("IS;\t\t\t\tStop sending IMU values"));
  Serial.println(F("RB;\t\t\t\tStart sending report values"));
  Serial.println(F("RS;\t\t\t\tStop sending report values\r\n"));

  Serial.println(F("CS;\t\t\t\tSend stop command"));
  Serial.println(F("CJ,x,y;\t\t\t\tSteer robot using x,y-coordinates"));
  Serial.println(F("CM,pitch,roll;\t\t\tSteer robot using pitch and roll"));

  Serial.println(F("\r\n==========================================================================================\r\n"));
}

void Tools::calibrateAcc() {
  Serial.println(F("Please put the robot perfectly horizontal on its side and then send any character to start the calibration routine"));
  while (Serial.read() == -1);

  int16_t accYbuffer[25], accZbuffer[25];
  for (uint8_t i = 0; i < 25; i++) {
    while (i2cRead(0x3D, motor->i2cBuffer, 4));
    accYbuffer[i] = ((motor->i2cBuffer[0] << 8) | motor->i2cBuffer[1]);
    accZbuffer[i] = ((motor->i2cBuffer[2] << 8) | motor->i2cBuffer[3]);
    delay(10);
  }
  if (!motor->checkMinMax(accYbuffer, 25, 1000) || !motor->checkMinMax(accZbuffer, 25, 1000)) {
    Serial.print(F("Accelerometer calibration error"));
    motor->setBuzzer();
    while (1); // Halt
  }
  for (uint8_t i = 0; i < 25; i++) {
    motor->cfg.accYzero += accYbuffer[i];
    motor->cfg.accZzero += accZbuffer[i];
  }
  motor->cfg.accYzero /= 25.0f;
  motor->cfg.accZzero /= 25.0f;

  if (motor->cfg.accYzero < 0) // Check which side is laying down
    motor->cfg.accYzero += 16384.0f; // 16384.0 is equal to 1g while the full scale range is ±2g
  else
    motor->cfg.accYzero -= 16384.0f;

  Serial.print(F("New zero values (g's): ")); // Print the new values in g's
  Serial.print(motor->cfg.accYzero / 16384.0f);
  Serial.print(F(","));
  Serial.println(motor->cfg.accZzero / 16384.0f);

  eeprom->updateConfig(); // Store the new values in the EEPROM
  Serial.println(F("Calibration of the accelerometer is done"));
}

void Tools::calibrateMotor() {
  Serial.println(F("Put the robot so the wheels can move freely and then send any character to start the motor calibration routine"));
  while (Serial.read() == -1);

  Serial.println(F("Estimating minimum starting value. When the first two values do not change anymore, then send any character to continue\r\n"));
  delay(2000);
  float leftSpeed = 10.0f, rightSpeed = 10.0f;
  testMotorSpeed(&leftSpeed, &rightSpeed, 1.0f, 1.0f);

  Serial.print(F("\r\nThe speed values are (L/R): "));
  Serial.print(leftSpeed);
  Serial.print(F(","));
  Serial.println(rightSpeed);

  if (leftSpeed > rightSpeed) { // This means that the left motor needed a higher PWM signal before it rotated at the same speed
    motor->cfg.leftMotorScaler = 1.0f;
    motor->cfg.rightMotorScaler = rightSpeed / leftSpeed; // Therefore we will scale the right motor a bit down, so they match
  } else { // And the same goes for the right motor
    motor->cfg.leftMotorScaler = leftSpeed / rightSpeed;
    motor->cfg.rightMotorScaler = 1.0f;
  }

  Serial.print(F("The motor scalars are now (L/R): "));
  Serial.print(motor->cfg.leftMotorScaler);
  Serial.print(F(","));
  Serial.println(motor->cfg.rightMotorScaler);

  Serial.println(F("Now the motors will spin up again. Now the speed values should be almost equal. Send any character to exit\r\n"));
  delay(2000);
  leftSpeed = rightSpeed = 10.0f; // Reset speed values
  testMotorSpeed(&leftSpeed, &rightSpeed, motor->cfg.leftMotorScaler, motor->cfg.rightMotorScaler);

  float maxSpeed = max(leftSpeed, rightSpeed);
  float minSpeed = min(leftSpeed, rightSpeed);

  Serial.print(F("The difference is now: "));
  Serial.print((maxSpeed - minSpeed) / maxSpeed * 100.0f);
  Serial.println("%");

  eeprom->updateConfig(); // Store the new values in the EEPROM
  Serial.println(F("Calibration of the motors is done"));
}

void Tools::testMotorSpeed(float *leftSpeed, float *rightSpeed, float leftScaler, float rightScaler) {
  int32_t lastLeftPosition = motor->readLeftEncoder();
  int32_t lastRightPosition = motor->readRightEncoder();

  Serial.println(F("Velocity (L), Velocity (R), Speed value (L), Speed value (R)"));
  while (Serial.read() == -1) {
    motor->moveMotor(left, forward, (*leftSpeed)*leftScaler);
    motor->moveMotor(right, forward, (*rightSpeed)*rightScaler);

    int32_t leftPosition = motor->readLeftEncoder();
    int32_t leftVelocity = leftPosition - lastLeftPosition;
    lastLeftPosition = leftPosition;

    int32_t rightPosition = motor->readRightEncoder();
    int32_t rightVelocity = rightPosition - lastRightPosition;
    lastRightPosition = rightPosition;

    Serial.print(leftVelocity);
    Serial.print(F(","));
    Serial.print(rightVelocity);
    Serial.print(F(","));
    Serial.print(*leftSpeed);
    Serial.print(F(","));
    Serial.println(*rightSpeed);

    if (abs(leftVelocity) < 200)
      (*leftSpeed) += 0.1f;
    else if (abs(leftVelocity) > 203)
      (*leftSpeed) -= 0.1f;

    if (abs(rightVelocity) < 200)
      (*rightSpeed) += 0.1f;
    else if (abs(rightVelocity) > 203)
      (*rightSpeed) -= 0.1f;

    delay(100);
  }
  for (float i = *leftSpeed; i > 0; i--) { // Stop motors gently
    motor->moveMotor(left, forward, i);
    motor->moveMotor(right, forward, i);
    delay(50);
  }
  motor->stopMotor(left);
  motor->stopMotor(right);
}

void Tools::printValues() {
  Print *out; // This allows the robot to use either the hardware UART or the Bluetooth SPP connection dynamically
 
  if (sendPairConfirmation) {
    sendPairConfirmation = false;

    out->println(F("PC"));
  } else if (sendPIDValues) {
    sendPIDValues = false;

    out->print(F("P,"));
    out->print(motor->cfg.P);
    out->print(F(","));
    out->print(motor->cfg.I);
    out->print(F(","));
    out->print(motor->cfg.D);
    out->print(F(","));
    out->println(motor->cfg.targetAngle);
  } else if (sendSettings) {
    sendSettings = false;

    out->print(F("S,"));
    out->print(motor->cfg.backToSpot);
    out->print(F(","));
    out->print(motor->cfg.controlAngleLimit);
    out->print(F(","));
    out->println(motor->cfg.turningLimit);
  } else if (sendInfo) {
    sendInfo = false;

    out->print(F("I,"));
    out->print(eeprom->version);
    out->print(F(","));
    out->print(eeprom->eepromVersion);

#if defined(__AVR_ATmega644__)
    out->println(F(",ATmega644"));
#elif defined(__AVR_ATmega1284P__)
    out->println(F(",ATmega1284P"));
#else
    out->println(F(",Unknown"));
#endif
  } else if (sendKalmanValues) {
    sendKalmanValues = false;

    out->print(F("K,"));
    out->print(motor->kalman.getQangle(), 4);
    out->print(F(","));
    out->print(motor->kalman.getQbias(), 4);
    out->print(F(","));
    out->println(motor->kalman.getRmeasure(), 4);
  } else if (sendIMUValues && millis() - motor->imuTimer > 50) { // Only send data every 50ms
    motor->imuTimer = millis();

    out->print(F("V,"));
    out->print(motor->accAngle);
    out->print(F(","));
    out->print(motor->gyroAngle);
    out->print(F(","));
    out->println(motor->pitch);
  } else if (sendStatusReport && millis() - motor->reportTimer > 500) { // Send data every 500ms
    motor->reportTimer = millis();

    out->print(F("R,"));
    out->print(batteryVoltage);
    out->print(F(","));
    out->println((float)motor->reportTimer / 60000.0f);
  }
}

void Tools::setValues(char *input) {
  if (input[0] == 'A' && input[1] == ';') { // Abort
    motor->stopAndReset();
     
    
      while (Serial.read() != 'C');
    
  }

  else if (input[0] == 'A' && input[1] == 'C') // Accelerometer calibration
    calibrateAcc();
  else if (input[0] == 'M' && input[1] == 'C') // Motor calibration
    calibrateMotor();

  //For sending PID and IMU values
  else if (input[0] == 'G') { // The different application sends when it needs the PID, settings or info
    if (input[1] == 'P') // Get PID Values
      sendPIDValues = true;
    else if (input[1] == 'S') // Get settings
      sendSettings = true;
    else if (input[1] == 'I') // Get info
      sendInfo = true;
    else if (input[1] == 'K') // Get Kalman filter values
      sendKalmanValues = true;
  }

  else if (input[0] == 'S') { // Set different values
    //Set PID and target angle
    if (input[1] == 'P') {
      strtok(input, ","); // Ignore 'P'
      motor->cfg.P = atof(strtok(NULL, ";"));
    } else if (input[1] == 'I') {
      strtok(input, ","); // Ignore 'I'
      motor->cfg.I = atof(strtok(NULL, ";"));
    } else if (input[1] == 'D') {
      strtok(input, ","); // Ignore 'D'
      motor->cfg.D = atof(strtok(NULL, ";"));
    } else if (input[1] == 'T') { // Target Angle
      strtok(input, ","); // Ignore 'T'
      motor->cfg.targetAngle = atof(strtok(NULL, ";"));
    }
    else if (input[1] == 'K') { // Kalman values
      strtok(input, ","); // Ignore 'K'
      motor->cfg.Qangle = atof(strtok(NULL, ","));
      motor->cfg.Qbias = atof(strtok(NULL, ","));
      motor->cfg.Rmeasure = atof(strtok(NULL, ";"));
    }
    else if (input[1] == 'A') { // Controlling max angle
      strtok(input, ","); // Ignore 'A'
      motor->cfg.controlAngleLimit = atoi(strtok(NULL, ";"));
    } else if (input[1] == 'U') { // Turning max value
      strtok(input, ","); // Ignore 'U'
      motor->cfg.turningLimit = atoi(strtok(NULL, ";"));
    }
    else if (input[1] == 'B') { // Set Back To Spot
      if (input[3] == '1')
        motor->cfg.backToSpot = 1;
      else
        motor->cfg.backToSpot = 0;
    }

    eeprom->updateConfig();
  }

  else if (input[0] == 'I') { // IMU transmitting states
    if (input[1] == 'B') // Begin sending IMU values
      sendIMUValues = true; // Start sending output to application
    else if (input[1] == 'S') // Stop sending IMU values
      sendIMUValues = false; // Stop sending output to application
  }

  else if (input[0] == 'R') { // Report states
    if (input[1] == 'B') // Begin sending report values
      sendStatusReport = true; // Start sending output to application
    else if (input[1] == 'S') // Stop sending report values
      sendStatusReport = false; // Stop sending output to application
  }

  else if (input[0] == 'C') { // Commands
    if (input[1] == 'S') // Stop
      motor->steer(stop);
    else if (input[1] == 'J') { // Joystick
      receiveControlTimer = millis();
      strtok(input, ","); // Ignore 'J'
      sppData1 = atof(strtok(NULL, ",")); // x-axis
      sppData2 = atof(strtok(NULL, ";")); // y-axis
      motor->steer(joystick,sppData1,sppData2);
    }
    else if (input[1] == 'M') { // IMU
      receiveControlTimer = millis();
      strtok(input, ","); // Ignore 'M'
      sppData1 = atof(strtok(NULL, ",")); // Pitch
      sppData2 = atof(strtok(NULL, ";")); // Roll
      motor->steer(imu,sppData1,sppData2);
    }

    else if (input[1] == 'R') {
      eeprom->restoreEEPROMValues(); // Restore the default EEPROM values
      sendPIDValues = true;
      sendKalmanValues = true;
      sendSettings = true;
    }
  }
}

void Tools::checkBatteryVoltage(){
    batteryCounter++;
  if (batteryCounter >= 10) { // Measure battery every 1s
    batteryCounter = 0;
    batteryVoltage = (float)analogRead(VBAT) / 63.050847458f; // VBAT is connected to analog input 5 which is not broken out. This is then connected to a 47k-12k voltage divider - 1023.0/(3.3/(12.0/(12.0+47.0))) = 63.050847458
    if (batteryVoltage < 11.1 && batteryVoltage > 5) // Equal to 3.7V per cell - don't turn on if it's below 5V, this means that no battery is connected
      motor->setBuzzer();
    else
      motor->clearBuzzer();
  }
}


