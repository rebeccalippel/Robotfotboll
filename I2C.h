#ifndef _i2c_h_
#define _i2c_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

#endif

