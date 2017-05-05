/*
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

This file contains tools for communication via the arduino serial interface
and functions for calibration of the motors and the accelerometer.
*/

#ifndef _tools_h_
#define _tools_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

class Motor;
class Eeprom;
class Bluetooth;

class Tools {
private:
  Motor * motor;
  Eeprom * eeprom;
  Bluetooth * bluetooth;
public:
  bool sendIMUValues, sendSettings, sendInfo, sendStatusReport, sendPIDValues, sendPairConfirmation, sendKalmanValues; // Used to send out different values via Bluetooth
  float batteryVoltage; // Measured battery level

  char dataInput[30]; // Incoming data buffer
  bool bluetoothData; // True if data received is from the Bluetooth connection
  float sppData1, sppData2; // Data send via SPP connection
  uint32_t receiveControlTimer;

  Tools(Motor * m, Eeprom * e) { motor = m; eeprom = e; }
  void addBluetooth(Bluetooth * b) { bluetooth = b; }
  
  void checkSerialData();
  void printMenu();
  void calibrateMotor();
  void testMotorSpeed(float *leftSpeed, float *rightSpeed, float leftScaler, float rightScaler);
  void calibrateAcc();
  
  void printValues();
  void setValues(char *input);

  void checkBatteryVoltage();

};



#endif

