/*
Developed by bachelor project SSYX02-1613.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

This file contains the core functions that makes the robot work,
including the regulator that is used for balancing the robot.
*/

#ifndef _motor_h_
#define _motor_h_

#include <stdint.h> // Needed for uint8_t, uint16_t etc.
#include <Kalman.h>

enum Command {
  stop,
  forward,
  backward,
  left,
  right,
  imu,
  joystick,
};

// This struct will store all the configuration values
typedef struct {
  float P, I, D; // PID variables
  float targetAngle; // Resting angle of the robot
  uint8_t backToSpot; // Set whenever the robot should stay in the same spot
  uint8_t controlAngleLimit; // Set the maximum tilting angle of the robot
  uint8_t turningLimit; // Set the maximum turning value
  float Qangle, Qbias, Rmeasure; // Kalman filter values
  float accYzero, accZzero; // Accelerometer zero values
  float leftMotorScaler, rightMotorScaler;
} cfg_t;

class Motor {
private:
  /* Counters used to count the pulses from the encoders */
  static volatile int32_t leftCounter;
  static volatile int32_t rightCounter;
  static const int8_t enc_states[16];
  
  static const uint16_t PWM_FREQUENCY; // The motor driver can handle a PWM frequency up to 20kHz
  static const uint16_t PWMVALUE; // The frequency is given by F_CPU/(2*N*ICR) - where N is the prescaler, prescaling is used so the frequency is given by F_CPU/(2*ICR) - ICR = F_CPU/PWM_FREQUENCY/2

  float lastRestAngle; // Used to limit the new restAngle if it's much larger than the previous one

/* IMU Data */
  float gyroXzero;

// Results
  float lastError; // Store last angle error
  float iTerm; // Store iTerm



  int32_t lastWheelPosition; // Used to calculate the wheel velocity
  int32_t wheelVelocity; // Wheel velocity based on encoder readings
  int32_t targetPosition; // The encoder position the robot should be at

  float radiusLeftScaler = 0;
  float radiusRightScaler = 0;
  
public:
  float turningRadius = 0;
  

  bool steerStop; // Stop by default
  bool stopped; // This is used to set a new target position after braking
  float targetOffset; // Offset for going forward and backward
  float turningOffset; // Offset for turning left and right
  
  Command lastCommand;
  cfg_t cfg;
  Kalman kalman;

  bool layingDown; // Use to indicate if the robot is laying down

  uint32_t ledTimer; // Used to update the LEDs to indicate battery level on the PS3, PS4, Wii and Xbox controllers
  uint32_t blinkTimer; // Used to blink the built in LED, starts blinking faster upon an incoming Bluetooth request
  bool commandSent; // This is used so multiple controller can be used at once

  uint8_t i2cBuffer[8]; // Buffer for I2C data
  float accAngle, gyroAngle; // Result from raw accelerometer and gyroscope readings
  float pitch; // Result from Kalman filter
 // float PIDLeft;
 // float PIDRight;

  /* Used for timing */
  uint32_t kalmanTimer; // Timer used for the Kalman filter
  uint32_t pidTimer; // Timer used for the PID loop
  uint32_t imuTimer; // This is used to set a delay between sending IMU values
  uint32_t encoderTimer; // Timer used used to determine when to update the encoder values
  uint32_t reportTimer; // This is used to set a delay between sending report values
  uint32_t stopTimer; //This is used for braking

  void updatePID(float restAngle, float offset, float turning, float dt);
  void moveMotor(Command motor, Command direction, float speedRaw);
  void stopMotor(Command motor);
  void setPWM(Command motor, uint16_t dutyCycle);
  void stopAndReset();
  
  static void leftEncoder();
  static void rightEncoder();
  int32_t readLeftEncoder();
  int32_t readRightEncoder();
  int32_t getWheelsPosition();

  void checkMotors();
  void calculatePitch();
  void driveMotors();
  void updateEncoders();
  
  void setupEncoders();
  void setupMotors();
  void setupIMU();
  void setupTiming();
  void calibrateAndReset();

  void initBuzzer();
  void setBuzzer();
  void clearBuzzer();
  void soundBuzzer(int delay);

  void steer(Command command);
  void steer(Command command1, float amount1, Command command2, float amount2);
  void steer(Command command, float amount);
  void steer(Command command, float amountTurn, float amountForward);
  float scale(float input, float inputMin, float inputMax, float outputMin, float outputMax);

  bool calibrateGyro();
  bool checkMinMax(int16_t *array, uint8_t length, int16_t maxDifference);
};


#endif

