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

#include "Eeprom.h"

#include "EEPROMAnything.h"
#include "Motor.h"

const char* Eeprom::version = "1.1.0";
const uint8_t Eeprom::eepromVersion = 3; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed

/* EEPROM Address Definitions */
const uint8_t Eeprom::initFlagsAddr = 0; // Set the first byte to the EEPROM version
const uint8_t Eeprom::configAddr = 1; // Save the configuration starting from this location

bool Eeprom::checkInitializationFlags() {
  uint8_t initFlag;
  EEPROM_readAnything(initFlagsAddr, initFlag);
  if (initFlag != eepromVersion) { // Check if the EEPROM version matches the current one
    restoreEEPROMValues();
    EEPROM_updateAnything(initFlagsAddr, eepromVersion); // After the default values have been restored, set the flag
    return true; // Indicate that the values have been restored
  }
  return false;
}

void Eeprom::readEEPROMValues() {
  EEPROM_readAnything(configAddr, motor->cfg);

  motor->kalman.setQangle(motor->cfg.Qangle);
  motor->kalman.setQbias(motor->cfg.Qbias);
  motor->kalman.setRmeasure(motor->cfg.Rmeasure);
}

void Eeprom::updateConfig() {
  EEPROM_updateAnything(configAddr, motor->cfg);

  motor->kalman.setQangle(motor->cfg.Qangle);
  motor->kalman.setQbias(motor->cfg.Qbias);
  motor->kalman.setRmeasure(motor->cfg.Rmeasure);
}

void Eeprom::restoreEEPROMValues() {
  motor->cfg.P = 9.0f; //start value 9
  motor->cfg.I = 2.0f; //start value 2
  motor->cfg.D = 3.0f; //start value 3

  motor->cfg.targetAngle = 180.0f;
  motor->cfg.backToSpot = 1;
  motor->cfg.controlAngleLimit = 7;
  motor->cfg.turningLimit = 25;

  motor->cfg.Qangle = 0.001f;
  motor->cfg.Qbias = 0.003f;
  motor->cfg.Rmeasure = 0.03f;

  motor->cfg.accYzero = motor->cfg.accZzero = 0.0f;
  motor->cfg.leftMotorScaler = motor->cfg.rightMotorScaler = 1.0f;

  updateConfig();
}

