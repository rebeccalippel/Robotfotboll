/*
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").
*/

#ifndef _eeprom_h_
#define _eeprom_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

class Motor; //Forward declaration of Motor

class Eeprom {
private:
  Motor *motor;

  /* EEPROM Address Definitions */
  static const uint8_t initFlagsAddr; // Set the first byte to the EEPROM version
  static const uint8_t configAddr; // Save the configuration starting from this location
public:
  /* Firmware Version Information */
  static const char *version;
  static const uint8_t eepromVersion; // EEPROM version - used to restore the EEPROM values if the configuration struct have changed
  
  Eeprom(Motor * m) { motor = m; }
  
  bool checkInitializationFlags();
  void readEEPROMValues();
  void updateConfig();
  void restoreEEPROMValues();
};

#endif


