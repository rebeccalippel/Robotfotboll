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

#include "Motor.h"

#include <Arduino.h>
#include <Wire.h>
#include <Usb.h>

#include "I2C.h"


// Buzzer used for feedback, it can be disconnected using the jumper
#if BALANDUINO_REVISION < 13
  #define buzzer P5
#else
  #define buzzer P11 /* A4 */
#endif

#define LED MAKE_PIN(LED_BUILTIN) // LED_BUILTIN is defined in pins_arduino.h in the hardware add-on


/* Left motor */
#define leftA P23
#define leftB P24
#define leftPWM P18

/* Right motor */
#if BALANDUINO_REVISION < 13
  #define rightA P25
  #define rightB P26
#else
  #define rightA P15
  #define rightB P16
#endif
#define rightPWM P17

/* Pins connected to the motor drivers diagnostic pins */
#define leftDiag P21
#define rightDiag P22

/* Encoders */
#if BALANDUINO_REVISION < 13
  #define leftEncoder1Pin 15 // Used for attachInterrupt
  #define leftEncoder2Pin 30 // Used for pin change interrupt
  #define rightEncoder1Pin 16 // Used for attachInterrupt
  #define rightEncoder2Pin 31 // Used for pin change interrupt
#else
  #define leftEncoder1Pin 25 // Used for pin change interrupt
  #define leftEncoder2Pin 26 // Used for pin change interrupt
  #define rightEncoder1Pin 30 // Used for pin change interrupt
  #define rightEncoder2Pin 31 // Used for pin change interrupt
#endif

#define MAKE_PIN(pin) _MAKE_PIN(pin) // Puts a P in front of the pin number, e.g. 1 becomes P1
#define _MAKE_PIN(pin) P ## pin

#define leftEncoder1 MAKE_PIN(leftEncoder1Pin)
#define leftEncoder2 MAKE_PIN(leftEncoder2Pin)

#define rightEncoder1 MAKE_PIN(rightEncoder1Pin)
#define rightEncoder2 MAKE_PIN(rightEncoder2Pin)

// You should change these to match your pins
#if BALANDUINO_REVISION < 13
  #define PIN_CHANGE_INTERRUPT_VECTOR_LEFT PCINT0_vect
  #define PIN_CHANGE_INTERRUPT_VECTOR_RIGHT PCINT0_vect
#else
  #define PIN_CHANGE_INTERRUPT_VECTOR_LEFT PCINT1_vect
  #define PIN_CHANGE_INTERRUPT_VECTOR_RIGHT PCINT0_vect
#endif

// Encoder values
#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
  constexpr uint16_t zoneA = 8000 * 2;
  constexpr uint16_t zoneB = 4000 * 2;
  constexpr uint16_t zoneC = 1000 * 2;
  constexpr float positionScaleA = 600.0f * 2.0f; // One resolution is 1856 pulses per encoder
  constexpr float positionScaleB = 800.0f * 2.0f;
  constexpr float positionScaleC = 1000.0f * 2.0f;
  constexpr float positionScaleD = 500.0f * 2.0f;
  constexpr float velocityScaleMove = 70.0f * 5.0f;
  constexpr float velocityScaleStop = 60.0f * 2.0f;
  constexpr float velocityScaleTurning = 70.0f * 2.0f;
#else
  constexpr uint16_t zoneA = 8000;
  constexpr uint16_t zoneB = 4000;
  constexpr uint16_t zoneC = 1000;
  constexpr float positionScaleA = 600.0f; // One resolution is 928 pulses per encoder
  constexpr float positionScaleB = 800.0f;
  constexpr float positionScaleC = 1000.0f;
  constexpr float positionScaleD = 500.0f;
  constexpr float velocityScaleMove = 70.0f;
  constexpr float velocityScaleStop = 60.0f;
  constexpr float velocityScaleTurning = 70.0f;
#endif


/* Interrupt routine and encoder read functions */
// It uses gray code to detect if any pulses are missed. See: https://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino and http://en.wikipedia.org/wiki/Rotary_encoder#Incremental_rotary_encoder.

#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
const int8_t Motor::enc_states[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 }; // Encoder lookup table if it interrupts on every edge
#elif BALANDUINO_REVISION < 13
#warning "Only interrupting on every second edge!"
const int8_t Motor::enc_states[16] = { 0, 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, -1, 0, 0, 0 }; // Encoder lookup table if it only interrupts on every second edge - this only works on revision 1.2 and older
#endif

ISR(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) {
  Motor::leftEncoder();
#if BALANDUINO_REVISION >= 13
}
ISR(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT) {
#endif
  Motor::rightEncoder();
}

volatile int32_t Motor::leftCounter = 0;
volatile int32_t Motor::rightCounter = 0;

/*Definitions of static variables*/
const uint16_t Motor::PWM_FREQUENCY = 20000; // The motor driver can handle a PWM frequency up to 20kHz
const uint16_t Motor::PWMVALUE = F_CPU / PWM_FREQUENCY / 2; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2



void Motor::updatePID(float restAngle, float offset, float turning, float dt) {
  /* Brake */
  if (steerStop) {
    int32_t wheelPosition = getWheelsPosition();
    int32_t positionError = wheelPosition - targetPosition;
    if (cfg.backToSpot) {
      if (abs(positionError) > zoneA) // Inside zone A
        restAngle -= (float)positionError / positionScaleA;
      else if (abs(positionError) > zoneB) // Inside zone B
        restAngle -= (float)positionError / positionScaleB;
      else if (abs(positionError) > zoneC) // Inside zone C
        restAngle -= (float)positionError / positionScaleC;
      else // Inside zone D
        restAngle -= (float)positionError / positionScaleD;
    } else {
      if (abs(positionError) < zoneC)
        restAngle -= (float)positionError / positionScaleD;
      else
        targetPosition = wheelPosition;
    }
    restAngle -= (float)wheelVelocity / velocityScaleStop;

    restAngle = constrain(restAngle, cfg.targetAngle - 10, cfg.targetAngle + 10); // Limit rest Angle
  }
  /* Drive forward and backward */
  else {
    if ((offset > 0 && wheelVelocity < 0) || (offset < 0 && wheelVelocity > 0) || offset == 0) // Scale down offset at high speed - wheel velocity is negative when driving forward and positive when driving backward
      offset += (float)wheelVelocity / velocityScaleMove; // We will always compensate if the offset is 0, but steerStop is not set
    restAngle -= offset;
  }

  restAngle = constrain(restAngle, lastRestAngle - 1, lastRestAngle + 1); // Don't change restAngle with more than 1 degree in each loop
  lastRestAngle = restAngle;

  /* Update PID values */
  float error = restAngle - pitch;
  //scaled cfg.I with fator 1.2, cfg.P with factor 1.2, cfg.D with factor 1.4
  float pTerm = cfg.P * error;
  iTerm += cfg.I * 100.0f * error * dt; // Multiplication with Ki is done before integration limit, to make it independent from integration limit value
  iTerm = constrain(iTerm, -100.0f, 100.0f); // Limit the integrated error - prevents windup
  float dTerm = (cfg.D / 100.0f) * (error - lastError) / dt;
  lastError = error;
  float PIDValue = pTerm + iTerm + dTerm;

  /* Steer robot sideways */
  if (turning < 0) { // Left
    turning += abs((float)wheelVelocity / velocityScaleTurning); // Scale down at high speed
    if (turning > 0)
      turning = 0;
  }
  else if (turning > 0) { // Right
    turning -= abs((float)wheelVelocity / velocityScaleTurning); // Scale down at high speed
    if (turning < 0)
      turning = 0;
  }

  if (turningRadius > 0.5){
    radiusLeftScaler = (turningRadius + 10) / turningRadius;
    radiusRightScaler = (turningRadius - 10) / turningRadius;
  }
  else if (turningRadius < -0.5){
    radiusLeftScaler = (-turningRadius - 10) / -turningRadius;
    radiusRightScaler = (-turningRadius + 10) / -turningRadius;
  }
  else if (turningRadius <= 0.5 && turningRadius >= -0.5){
    radiusLeftScaler = 1;
    radiusRightScaler = 1;
  }
  /*if (turning > 0){ // Svänghöger
    PIDLeft = PIDValue * radiusLeftScaler  + turning;
    PIDRight = 0;
  }
  else if(turning < 0){ // Sväng vänster
      PIDRight = PIDValue * radiusRightScaler - turning;
      PIDLeft= 0;
  } 
  else{ /Kör rakt fram */
   float PIDLeft = PIDValue * radiusLeftScaler  + turning;
   float PIDRight = PIDValue * radiusRightScaler - turning;
 // }

  //float PIDLeft = PIDValue + turning;
  //float PIDRight = PIDValue - turning;
  
  PIDLeft *= cfg.leftMotorScaler; // Compensate for difference in some of the motors
  PIDRight *= cfg.rightMotorScaler;
 // Serial.print(PIDLeft);
  //Serial.print("\t");
  //Serial.println(PIDRight);

  /* Set PWM Values */
  if (PIDLeft >= 0)
    moveMotor(left, forward, PIDLeft);
  else
    moveMotor(left, backward, -PIDLeft);
  if (PIDRight >= 0)
    moveMotor(right, forward, PIDRight);
  else
    moveMotor(right, backward, -PIDRight);
}

void Motor::moveMotor(Command motor, Command direction, float speedRaw) { // Speed is a value in percentage 0-100%
  if (speedRaw > 100.0f)
    speedRaw = 100.0f;
  setPWM(motor, speedRaw * (float)PWMVALUE / 100.0f); // Scale from 0-100 to 0-PWMVALUE
  if (motor == left) {
    if (direction == forward) {
      leftA::Clear();
      leftB::Set();
    }
    else {
      leftA::Set();
      leftB::Clear();
    }
  }
  else {
    if (direction == forward) {
      rightA::Set();
      rightB::Clear();
    }
    else {
      rightA::Clear();
      rightB::Set();
    }
  }
}

void Motor::stopMotor(Command motor) {
  setPWM(motor, PWMVALUE); // Set high
  if (motor == left) {
    leftA::Set();
    leftB::Set();
  }
  else {
    rightA::Set();
    rightB::Set();
  }
}

void Motor::setPWM(Command motor, uint16_t dutyCycle) { // dutyCycle is a value between 0-ICR1
  if (motor == left)
    OCR1A = dutyCycle;
  else
    OCR1B = dutyCycle;
}

void Motor::stopAndReset() {
  stopMotor(left);
  stopMotor(right);
  lastError = 0;
  iTerm = 0;
  targetPosition = getWheelsPosition();
  lastRestAngle = cfg.targetAngle;
}

void Motor::leftEncoder() {
  static uint8_t old_AB = 0;
  old_AB <<= 2; // Remember previous state
  old_AB |= (leftEncoder2::IsSet() >> (leftEncoder2::Number - 1)) | (leftEncoder1::IsSet() >> leftEncoder1::Number);
  leftCounter -= enc_states[ old_AB & 0x0F ];
}

void Motor::rightEncoder() {
  static uint8_t old_AB = 0;
  old_AB <<= 2; // Remember previous state
  old_AB |= (rightEncoder2::IsSet() >> (rightEncoder2::Number - 1)) | (rightEncoder1::IsSet() >> rightEncoder1::Number);
  rightCounter += enc_states[ old_AB & 0x0F ];
}

int32_t Motor::readLeftEncoder() { // The encoders decrease when motors are traveling forward and increase when traveling backward
  return leftCounter;
}

int32_t Motor::readRightEncoder() {
  return rightCounter;
}

int32_t Motor::getWheelsPosition() {
  return leftCounter + rightCounter;
}

void Motor::checkMotors() {
    if (!leftDiag::IsSet() || !rightDiag::IsSet()) { // Motor driver will pull these low on error
      buzzer::Set();
      stopMotor(left);
      stopMotor(right);
      while (1);
  }
}

void Motor::calculatePitch(){
  /* Calculate pitch */
  while (i2cRead(0x3D, i2cBuffer, 8));
  int16_t accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  int16_t accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  int16_t gyroX = ((i2cBuffer[6] << 8) | i2cBuffer[7]);

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2((float)accY - cfg.accYzero, (float)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;

  uint32_t timer = micros();
  // This fixes the 0-360 transition problem when the accelerometer angle jumps between 0 and 360 degrees
  if ((accAngle < 90 && pitch > 270) || (accAngle > 270 && pitch < 90)) {
    kalman.setAngle(accAngle);
    pitch = accAngle;
    gyroAngle = accAngle;
  } else {
    float gyroRate = ((float)gyroX - gyroXzero) / 131.0f; // Convert to deg/s
    float dt = (float)(timer - kalmanTimer) / 1000000.0f;
    gyroAngle += gyroRate * dt; // Gyro angle is only used for debugging
    if (gyroAngle < 0 || gyroAngle > 360)
      gyroAngle = pitch; // Reset the gyro angle when it has drifted too much
    pitch = kalman.getAngle(accAngle, gyroRate, dt); // Calculate the angle using a Kalman filter
  }
  kalmanTimer = timer;
}

void Motor::driveMotors(){
  /* Drive motors */
  uint32_t timer = micros();
  // If the robot is laying down, it has to be put in a vertical position before it starts balancing
  // If it's already balancing it has to be ±45 degrees before it stops trying to balance
  if ((layingDown && (pitch < cfg.targetAngle - 10 || pitch > cfg.targetAngle + 10)) || (!layingDown && (pitch < cfg.targetAngle - 45 || pitch > cfg.targetAngle + 45))) {
    layingDown = true; // The robot is in a unsolvable position, so turn off both motors and wait until it's vertical again
    stopAndReset();
  } else {
    layingDown = false; // It's no longer laying down
    updatePID(cfg.targetAngle, targetOffset, turningOffset, (float)(timer - pidTimer) / 1000000.0f);
  }
  pidTimer = timer;
}

void Motor::updateEncoders(){
  /* Update encoders */
  uint32_t timer = millis();
  if (timer - encoderTimer >= 100) { // Update encoder values every 100ms
    encoderTimer = timer;
    int32_t wheelPosition = getWheelsPosition();
    wheelVelocity = wheelPosition - lastWheelPosition;
    lastWheelPosition = wheelPosition;
    //Serial.print(wheelPosition);Serial.print('\t');Serial.print(targetPosition);Serial.print('\t');Serial.println(wheelVelocity);
    if (abs(wheelVelocity) <= 40 && !stopped) { // Set new targetPosition if braking
      targetPosition = wheelPosition;
      stopped = true;
    }
  }
}

void Motor::setupEncoders(){
  /* Setup encoders */
  leftEncoder1::SetDirRead();
  leftEncoder2::SetDirRead();
  rightEncoder1::SetDirRead();
  rightEncoder2::SetDirRead();
  leftEncoder1::Set(); // Enable pull-ups
  leftEncoder2::Set();
  rightEncoder1::Set();
  rightEncoder2::Set();

#if BALANDUINO_REVISION < 13 // On the new revisions pin change interrupt is used for all pins
  attachInterrupt(digitalPinToInterrupt(leftEncoder1Pin), leftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rightEncoder1Pin), rightEncoder, CHANGE);
#endif

#if defined(PIN_CHANGE_INTERRUPT_VECTOR_LEFT) && defined(PIN_CHANGE_INTERRUPT_VECTOR_RIGHT)
  /* Enable encoder pins interrupt sources */
  #if BALANDUINO_REVISION >= 13
    *digitalPinToPCMSK(leftEncoder1Pin) |= (1 << digitalPinToPCMSKbit(leftEncoder1Pin));
    *digitalPinToPCMSK(rightEncoder1Pin) |= (1 << digitalPinToPCMSKbit(rightEncoder1Pin));
  #endif
  *digitalPinToPCMSK(leftEncoder2Pin) |= (1 << digitalPinToPCMSKbit(leftEncoder2Pin));
  *digitalPinToPCMSK(rightEncoder2Pin) |= (1 << digitalPinToPCMSKbit(rightEncoder2Pin));

  /* Enable pin change interrupts */
  #if BALANDUINO_REVISION >= 13
    *digitalPinToPCICR(leftEncoder1Pin) |= (1 << digitalPinToPCICRbit(leftEncoder1Pin));
    *digitalPinToPCICR(rightEncoder1Pin) |= (1 << digitalPinToPCICRbit(rightEncoder1Pin));
  #endif
  *digitalPinToPCICR(leftEncoder2Pin) |= (1 << digitalPinToPCICRbit(leftEncoder2Pin));
  *digitalPinToPCICR(rightEncoder2Pin) |= (1 << digitalPinToPCICRbit(rightEncoder2Pin));
#elif BALANDUINO_REVISION >= 13
  #error "Please define "PIN_CHANGE_INTERRUPT_VECTOR_LEFT" and "PIN_CHANGE_INTERRUPT_VECTOR_RIGHT""
#endif
}
void Motor::setupMotors(){
  /* Set the motordriver diagnostic pins to inputs */
  leftDiag::SetDirRead();
  rightDiag::SetDirRead();

  /* Setup motor pins to output */
  leftPWM::SetDirWrite();
  leftA::SetDirWrite();
  leftB::SetDirWrite();
  rightPWM::SetDirWrite();
  rightA::SetDirWrite();
  rightB::SetDirWrite();

  /* Set PWM frequency to 20kHz - see the datasheet http://www.atmel.com/Images/Atmel-8272-8-bit-AVR-microcontroller-ATmega164A_PA-324A_PA-644A_PA-1284_P_datasheet.pdf page 129-139 */
  // Set up PWM, Phase and Frequency Correct on pin 18 (OC1A) & pin 17 (OC1B) with ICR1 as TOP using Timer1
  TCCR1B = (1 << WGM13) | (1 << CS10); // Set PWM Phase and Frequency Correct with ICR1 as TOP and no prescaling
  ICR1 = PWMVALUE; // ICR1 is the TOP value - this is set so the frequency is equal to 20kHz

  /* Enable PWM on pin 18 (OC1A) & pin 17 (OC1B) */
  // Clear OC1A/OC1B on compare match when up-counting
  // Set OC1A/OC1B on compare match when down-counting
  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
}

void Motor::setupIMU(){
  /* Setup IMU */
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  while (i2cRead(0x75, i2cBuffer, 1));
  if (i2cBuffer[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    buzzer::Set();
    while (1); // Halt
  }

  while (i2cWrite(0x6B, 0x80, true)); // Reset device, this resets all internal registers to their default values
  do {
    while (i2cRead(0x6B, i2cBuffer, 1));
  } while (i2cBuffer[0] & 0x80); // Wait for the bit to clear
  delay(5);
  while (i2cWrite(0x6B, 0x09, true)); // PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode
#if 1
  i2cBuffer[0] = 1; // Set the sample rate to 500Hz - 1kHz/(1+1) = 500Hz
  i2cBuffer[1] = 0x03; // Disable FSYNC and set 44 Hz Acc filtering, 42 Hz Gyro filtering, 1 KHz sampling
#else
  i2cBuffer[0] = 15; // Set the sample rate to 500Hz - 8kHz/(15+1) = 500Hz
  i2cBuffer[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
#endif
  i2cBuffer[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cBuffer[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cBuffer, 4, true)); // Write to all four registers at once

  delay(100); // Wait for the sensor to get ready

  /* Set Kalman and gyro starting angle */
  while (i2cRead(0x3D, i2cBuffer, 4));
  int16_t accY = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
  int16_t accZ = ((i2cBuffer[2] << 8) | i2cBuffer[3]);
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // We then convert it to 0 to 2π and then from radians to degrees
  accAngle = (atan2((float)accY - cfg.accYzero, (float)accZ - cfg.accZzero) + PI) * RAD_TO_DEG;

  kalman.setAngle(accAngle); // Set starting angle
  pitch = accAngle;
  gyroAngle = accAngle;
}

void Motor::setupTiming(){
  /* Setup timing */
  kalmanTimer = micros();
  pidTimer = kalmanTimer;
  imuTimer = millis();
  encoderTimer = imuTimer;
  reportTimer = imuTimer;
  ledTimer = imuTimer;
  blinkTimer = imuTimer;
}

void Motor::calibrateAndReset() {
  /* Calibrate gyro zero value */
  while (calibrateGyro()); // Run again if the robot is moved while calibrating

  LED::SetDirWrite(); // Set LED pin to output
  stopAndReset(); // Turn off motors and reset different values
}

void Motor::initBuzzer(){
  buzzer::SetDirWrite();
}
void Motor::setBuzzer(){
  buzzer::Set();
}
void Motor::clearBuzzer(){
  buzzer::Clear();
}
void Motor::soundBuzzer(int msDelay){
  buzzer::Set();
  delay(msDelay);
  buzzer::Clear();
}

void Motor::steer(Command command) {
  steer(command, 0, 0);
}

void Motor::steer(Command command1, float amount1, Command command2, float amount2){
  steerStop = false;

  if(command1 == forward){
    targetOffset = scale(amount1, 0, 50, 0, cfg.controlAngleLimit);
  }
  else if(command1 == backward){
    targetOffset = -scale(amount1, 0, 50, 0, cfg.controlAngleLimit);
  }
  else if(command1 == right){
    turningOffset = scale(amount1, 0, 50, 0, cfg.turningLimit);
  }
  else if(command1 == left){
    turningOffset = -scale(amount1, 0, 50, 0, cfg.turningLimit);
  }
  else {
    steer(stop,0,0);
  }
  if(command2 == forward){
    targetOffset = scale(amount2, 0, 50, 0, cfg.controlAngleLimit);
  }
  else if(command2 == backward){
    targetOffset = -scale(amount2, 0, 50, 0, cfg.controlAngleLimit);
  }
  else if(command2 == right){
    turningOffset = scale(amount2, 0, 50, 0, cfg.turningLimit);
  }
  else if(command2 == left){
    turningOffset = -scale(amount2, 0, 50, 0, cfg.turningLimit);
  }
  else {
    steer(stop,0,0);
  }
  if(command1 == forward || command1 == backward){
    lastCommand = command1;
  }
  else{
    lastCommand = command2;
  }
}

void Motor::steer(Command command, float amount) {
  targetOffset = 0;
  turningOffset = 0;
  steerStop = false;
  
  if(command == forward){
    targetOffset = scale(amount, 0, 50, 0, cfg.controlAngleLimit);
  }
  else if(command == backward){
    targetOffset = -scale(amount, 0, 50, 0, cfg.controlAngleLimit);
  }
  else if(command == right){
    turningOffset = scale(amount, 0, 50, 0, cfg.turningLimit);
  }
  else if(command == left){
    turningOffset = -scale(amount, 0, 50, 0, cfg.turningLimit);
  }
  else {
    steer(stop,0,0);
  }
  if (command == left || command == right) {   
    if (lastCommand == forward || lastCommand == backward) { // Set new stop position
      targetPosition = getWheelsPosition();
      stopped = false;
      stopTimer = millis();
    }
    if(millis()-stopTimer > 500){
      steerStop = true;
      targetPosition = getWheelsPosition();
      stopped = false;
    }
  }
  lastCommand = command;  
}

void Motor::steer(Command command, float amountTurn, float amountForward) {
  commandSent = true; // Used to see if there has already been send a command or not

  steerStop = false;
  targetOffset = turningOffset = 0; // Set both to 0

  if (command == joystick) {
    if (amountForward > 0) // Forward
      targetOffset = scale(amountForward, 0, 1, 0, cfg.controlAngleLimit);
    else if (amountForward < 0) // Backward
      targetOffset = -scale(amountForward, 0, -1, 0, cfg.controlAngleLimit);
    if (amountTurn > 0) // Right
      turningOffset = scale(amountTurn, 0, 1, 0, cfg.turningLimit);
    else if (amountTurn < 0) // Left
      turningOffset = -scale(amountTurn, 0, -1, 0, cfg.turningLimit);
  } else if (command == imu) { //amountForward is turningOffset and amountTurn is targetOffset here
    if (amountTurn > 0) // Forward
      targetOffset = scale(amountTurn, 0, 36, 0, cfg.controlAngleLimit);
    else if (amountTurn < 0) // Backward
      targetOffset = -scale(amountTurn, 0, -36, 0, cfg.controlAngleLimit);
    if (amountForward > 0) // Right
      turningOffset = scale(amountForward, 0, 45, 0, cfg.turningLimit);
    else if (amountForward < 0) // Left
      turningOffset = -scale(amountForward, 0, -45, 0, cfg.turningLimit);
  }

  if (command == stop) {   
    if (lastCommand != stop) { // Set new stop position
      targetPosition = getWheelsPosition();
      stopped = false;
      stopTimer = millis();
    }
    if(millis()-stopTimer > 500){
      steerStop = true;
      targetPosition = getWheelsPosition();
      stopped = false;
    }
  }
  lastCommand = command;
}

float Motor::scale(float input, float inputMin, float inputMax, float outputMin, float outputMax) { // Like map() just returns a float
  float output;
  if (inputMin < inputMax)
    output = (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output = (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}

/*Private functions*/

bool Motor::calibrateGyro() {
  int16_t gyroXbuffer[25];
  for (uint8_t i = 0; i < 25; i++) {
    while (i2cRead(0x43, i2cBuffer, 2));
    gyroXbuffer[i] = ((i2cBuffer[0] << 8) | i2cBuffer[1]);
    delay(10);
  }
  if (!checkMinMax(gyroXbuffer, 25, 2000)) {
    Serial.println(F("Gyro calibration error"));
    buzzer::Set();
    return 1;
  }
  for (uint8_t i = 0; i < 25; i++)
    gyroXzero += gyroXbuffer[i];
  gyroXzero /= 25.0f;
  return 0;
}

bool Motor::checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference) { // Used to check that the robot is laying still while calibrating
  int16_t min = array[0], max = array[0];
  for (uint8_t i = 1; i < length; i++) {
    if (array[i] < min)
      min = array[i];
    else if (array[i] > max)
      max = array[i];
  }
  return max - min < maxDifference;
}

