
/*
 LilBot.h file v1.1
 Includes the contents of the Arduino Wire.h header
 to circumvent an Arduino idiosyncrasy

 Copyright 2014 Stellar Robotics, LLC and Chris Hakim
 Copyright 2015 Chris Hakim

 Robot balancing: John Sokol

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/

// Begin including contents of Wire.h

#ifndef TwoWire_h
#define TwoWire_h

#include "Arduino.h"
#include <inttypes.h>
#include <EEPROM.h>
#include "Stream.h"

#define BUFFER_LENGTH 32

class TwoWire : public Stream
{
  private:
    static uint8_t rxBuffer[];
    static uint8_t rxBufferIndex;
    static uint8_t rxBufferLength;

    static uint8_t txAddress;
    static uint8_t txBuffer[];
    static uint8_t txBufferIndex;
    static uint8_t txBufferLength;

    static uint8_t transmitting;
    static void (*user_onRequest)(void);
    static void (*user_onReceive)(int);
    static void onRequestService(void);
    static void onReceiveService(uint8_t*, int);
  public:
    TwoWire();
    void begin();
    void begin(uint8_t);
    void begin(int);
    void beginTransmission(uint8_t);
    void beginTransmission(int);
    uint8_t endTransmission(void);
    uint8_t endTransmission(uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t);
    uint8_t requestFrom(uint8_t, uint8_t, uint8_t);
    uint8_t requestFrom(int, int);
    uint8_t requestFrom(int, int, int);
    virtual size_t write(uint8_t);
    virtual size_t write(const uint8_t *, size_t);
    virtual int available(void);
    virtual int read(void);
    virtual int peek(void);
    virtual void flush(void);
    void onReceive( void (*)(int) );
    void onRequest( void (*)(void) );

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write;
};

extern TwoWire Wire;

#endif

// End contents of Wire.h

// Begin LilBot definitions

#ifndef _lilBot_h
#define _lilBot_h

class LilBot {
public:
  void begin(void);
  void balance(void);
  void wait(int delayMillis);
  void emote(short int emotion);
  void emoteByNumber(int emotionIndex);
  void go(long rawUnits);
  void stop(void);
  void rotate(double angle);
  short int blankFace, afraid, amused, angry, blissful, cool, crying, disappointed,
  embarrassed, happy, impatient, naughty, neutral, nonplussed, outraged, proud,
  resigned, sad, sarcastic, shocked, smiling, verySad, winking;
  void sound(int soundLine);
  void say(char *sentence);
private:
  static const char PROGMEM soundTable[][15];
  void checkBattery(bool reportBatteryVoltage);
  void initIMU(void);
  void writeIMU(char subAddress, char data);
  double controllerPID();
  void initProx(void);
  void proxGetResults(void);
  char proxParamRead(char param);
  void proxParamWrite(char param, char data);
  char proxRead(char regAddress);
  void proxReset(void);
  void proxStart(void);
  void proxWrite(char subAddress, char data);
  void readIMU(void);
  void wheels(double torque);
  void emoInit(void);
  short int emoState[23];
  void modulate(int startTone, char repeats, char increment);
  void sweep(int startTone, int endTone, int duration);
  void resetOdometry(void);
  enum motionStatus { MOTION_NONE, MOTION_FORWARD, MOTION_BACK, MOTION_RECOVER };
  enum proxStatus { PROX_NONE, PROX_RIGHT, PROX_LEFT, PROX_FRONT, PROX_EDGE };
  enum IMUGyroRanges { IMU_250_DPS = 0, IMU_500_DPS = 8, IMU_1000_DPS = 16, IMU_2000_DPS = 24 };
  enum IMUAccelRanges { IMU_2_G = 0, IMU_4_G = 8, IMU_8_G = 16, IMU_16_G = 24 };
  enum IMUSPS { IMU_200_SPS = 4, IMU_100_SPS = 9 };
  enum pinNumbers { RIGHT_FORWARD_PIN = 8, RIGHT_PWM_PIN = 10, LEFT_FORWARD_PIN = 7, LEFT_PWM_PIN = 9,
    BUZZER = 11, CAL_BUTTON = 12, LED = 13, IMU_INTERRUPT = 4 };
  enum IMUControls { IMU_LATCH_INT_EN_PULSE = 0, IMU_LATCH_INT_EN_HOLD = 0x20,
    IMU_INT_RD_CLEAR = 0x10, IMU_DATA_RDY_EN = 1 };
  struct {
    int right;
    int left;
    int edge;
    long rightBaseline;
    long leftBaseline;
    long edgeBaseline;
    long filteredRight;
    long filteredLeft;
    long filteredEdge;
    int event;
    bool enableEventHandlers;
  } botProx;
  double gyroScale;
  double acceleratometerScale;
  double botRotation;
  union {
    // IMUCalData[0] is gyroscope gain calibration (near 1.0)
    double IMUCalData[2];
    uint8_t IMUCalBytes[sizeof(double) * 2];
  } calVariables;
  double gyroY;
  double dtScaled;
  double accelerationScaleFactor, samplingPeriod;
  double targetAngle, accelYAngle, angle;
  double scaledXGyro, scaledYGyro, scaledXAccel, scaledZAccel;
  double  XGyro,  YGyro, ZGyro,  XAccel,  YAccel,  ZAccel;
  double XAngle,  YAngle,  ZAngle;
  double integratedError, lastError;
  int batteryVoltage;
  char gyroRange;
  char accelRange;
  char sampleDiv;
  double dTime;
  double gyroOffsetY;          // For balance
  double gyroOffsetX;          // To track robot rotation
  double accelerometerOffsetZ; // To track robot linear motion
  bool calibrate;
  double angleCombined;
  double adaptiveAngleBias, savedAdaptiveAngleBias;
  char motion;
  double tiltForMotion;        // Tilt for forward and backward motion
  double tiltFiltered;
  double motionSlowDownFactor; // To reduce speed near destination
  long wheelSpeed;
};

// C-style event handlers for proximity sensor
extern void (*frontObstacleHandler)(void);
extern void (*rightObstacleHandler)(void);
extern void (*leftObstacleHandler)(void);
extern void (*edgeHandler)(void);

#endif

// End LilBot definitions
