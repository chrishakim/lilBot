
/*
 LilBot.cpp library v1.0 (also runs with LilBot.h v1.1)

 Copyright 2014 Stellar Robotics, LLC and Chris Hakim

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

#include "Arduino.h"
#include "LilBot.h"
#include <math.h>
#include <inttypes.h>

/* Reserved keywords:
  Bot
  LilBot
  rightWheelPosition
  leftWheelPosition
  wheelPosition
  wheelLastPosition
  rightForward;
  leftForward;
  rightWheelEncoder
  leftWheelEncoder  
  frontObstacleHandler
  rightObstacleHandler
  leftObstacleHandler
  edgeHandler
  defaultFrontObstacleHandler
  defaultRightObstacleHandler;
  defaultLeftObstacleHandler;
  defaultEdgeHandler;
*/

// The wheel positions are updated by the encoder ISRs
// They are declared as volatile because ISRs access them
volatile long rightWheelPosition = 0;
volatile long leftWheelPosition = 0;
long wheelPosition = 0;
long wheelLastPosition = 0;
int rightForward;
int leftForward;

// ISR
void rightWheelEncoder(void) {
  rightWheelPosition += rightForward;
}

// ISR
void leftWheelEncoder(void) {
  leftWheelPosition += leftForward;
}

// C-style event handlers for proximity sensor
void defaultFrontObstacleHandler(void) {} // Default event handler, does nothing
void defaultRightObstacleHandler(void) {} // Default event handler, does nothing
void defaultLeftObstacleHandler(void) {}  // Default event handler, does nothing
void defaultEdgeHandler(void) {}          // Default event handler, does nothing
extern void (*frontObstacleHandler)(void) = defaultFrontObstacleHandler;
extern void (*rightObstacleHandler)(void) = defaultRightObstacleHandler;
extern void (*leftObstacleHandler)(void) = defaultLeftObstacleHandler;
extern void (*edgeHandler)(void) = defaultEdgeHandler;

// Delay function for balancing robot: it behaves as a blocking function, but does not block the balancing
void LilBot::wait(int delayMillis) {
  unsigned long endTime = millis() + delayMillis;
  while (endTime > millis())
    balance();
}

void LilBot::begin(void) {
  int samples;            // Sample counter for calibration
  unsigned long int timeToCalibrateGyro;
  bool robotDown = true;  // Start with robot down
  double gyroOffsetXAcc = 0.0;
  double gyroOffsetYAcc = 0.0;
  double accelAverageXAcc = 0.0;

  // Miscellaneous variable initialization
  gyroRange = IMU_2000_DPS;   // Only gyroscope range that can keep up with fast rotations
  accelRange = IMU_2_G;
  sampleDiv = IMU_100_SPS;
  dTime = 0.01;               // 100 Samples per second. 
  gyroOffsetY = 0.0;          // For balance
  gyroOffsetX = 0.0;          // To track robot rotation
  accelerometerOffsetZ = 0.0; // To track robot linear motion
  calibrate = false;
  rightForward = 0;
  leftForward = 0;
  angleCombined = 0;
  adaptiveAngleBias = 0.0;
  motion = MOTION_NONE;
  tiltForMotion = 0.0;  // Tilt for forward and backward motion
  tiltFiltered = 0.0;
  motionSlowDownFactor = 1.0; // To reduce speed near destination
  integratedError = 0.0;
  lastError = 0.0;
  wheelSpeed = 0;

  Serial.begin(115200);  // Debug through USB

  TWCR = 0;
  Wire.begin(); // I2C interface
  TWBR = 12;    // Set I2C interface clock to 400kHz

  emoInit();
  emote(happy);

  checkBattery(true);
  delay(1000);

  attachInterrupt(1, rightWheelEncoder, CHANGE);
  attachInterrupt(0, leftWheelEncoder, CHANGE);

  TCCR1B = TCCR1B & 0xf8 | 1;         // Set pins 9 and 10 PWM to 35kHz
  pinMode(4, INPUT);
  pinMode(CAL_BUTTON, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT); // Direction pin
  pinMode(RIGHT_PWM_PIN, OUTPUT);     // PWM speed-control pin
  pinMode(LEFT_FORWARD_PIN, OUTPUT);  // Direction pin
  pinMode(LEFT_PWM_PIN, OUTPUT);      // PWM speed-control pin
  wheels(-2.0);  // Stop

  initIMU();
  initProx();

  // Calibrate gyroscope offsets
  samples = 200;
  while (samples) {
    if (digitalRead(IMU_INTERRUPT) == HIGH) {
      readIMU();
      gyroOffsetXAcc += scaledXGyro;
      gyroOffsetYAcc += scaledYGyro;
      samples--;
    }
  }
  gyroOffsetX = gyroOffsetXAcc * 0.005;  // Scale to sample count
  gyroOffsetY = gyroOffsetYAcc * 0.005;  // Scale to sample count
  tone(BUZZER, 500, 100);
  if( digitalRead(CAL_BUTTON) == LOW) {
    delay(200);
    tone(BUZZER, 500, 100);
    calibrate = true;
    // Calibrate gyroscope scale
    timeToCalibrateGyro = millis() + 7000;
    while (millis() < timeToCalibrateGyro) {
      if (digitalRead(IMU_INTERRUPT) == HIGH) {
        readIMU();
        botRotation += dTime * scaledXGyro;
      }
    }
    tone(BUZZER, 500, 100);
    delay(200);
    tone(BUZZER, 500, 100);
    delay(200);
    tone(BUZZER, 500, 100);
    if (fabs(botRotation) > 4.0 && botRotation < 8.0)
      calVariables.IMUCalData[0] = fabs(2 * PI / botRotation);
    else
      calVariables.IMUCalData[0] = 1.0;  // Error in calibration, use default factor
  }

  // Detect robot lifted
  while (robotDown) {
    if (digitalRead(IMU_INTERRUPT) == HIGH) {
      readIMU();
      if (fabs(scaledXAccel) > fabs(scaledZAccel * 3.5)) {
        robotDown = false;
      }
    }
  }
  botRotation = 0.0;

  if (calibrate) {
    // Wait until user has lifted the robot and stabilized it
    delay(4000);
    tone(BUZZER, 1000, 100);
    delay(200);
    tone(BUZZER, 1000, 100);
    // Determine initial balance point
    samples = 512;
    while (samples) {
      if (digitalRead(IMU_INTERRUPT) == HIGH) {
        readIMU();
        accelAverageXAcc += scaledXAccel;
        accelerometerOffsetZ += scaledZAccel;
        samples--;
      }
    }
    // Scale sums to 512 sample count
    accelAverageXAcc *= 1.953125e-3;
    accelerometerOffsetZ *= 1.953125e-3;
    targetAngle = -atan2(accelAverageXAcc, accelerometerOffsetZ);
    tone(BUZZER, 1000, 100);
    delay(200);
    tone(BUZZER, 1000, 100);
    delay(200);
    tone(BUZZER, 1000, 100);
  }
  else {
    // Read EEPROM
    for (samples = 0; samples < sizeof(calVariables); samples++) {
      calVariables.IMUCalBytes[samples] = EEPROM.read(samples);
    }
    targetAngle = calVariables.IMUCalData[1];
  }

  // Let robot calibrate its own balance point and proximity sensor
  botProx.rightBaseline = 0;
  botProx.leftBaseline = 0;
  botProx.edgeBaseline = 0;
  wait(6000);
  botProx.rightBaseline = botProx.filteredRight >> 4;
  botProx.leftBaseline = botProx.filteredLeft >> 4;
  botProx.edgeBaseline = botProx.filteredEdge >> 4;
  // Prime filter with baseline subtracted, approximately zero
  botProx.filteredRight = 0;
  botProx.filteredLeft = 0;
  botProx.filteredEdge = 0;

  if (calibrate) {
    calVariables.IMUCalData[1] = targetAngle + adaptiveAngleBias;
    // Write EEPROM
    for (samples = 0; samples < sizeof(calVariables); samples++) {
      EEPROM.write(samples, calVariables.IMUCalBytes[samples]);
      wait(50);  // Space writes so as not to block balancing -- EEPROM.write() takes 3.3ms
    }
  }
  savedAdaptiveAngleBias = adaptiveAngleBias;
  motionSlowDownFactor = 1.0;

  tiltFiltered = 0.0;
  motion = MOTION_NONE;
  tone(BUZZER, 523, 100);
  wait(110);
  tone(BUZZER, 659, 100);
  wait(110);
  tone(BUZZER, 784, 100);
  wait(110);
  tone(BUZZER, 1047, 200);
  wait(250);
  botProx.enableEventHandlers = true;
}

void LilBot::go(long rawUnits) {
  // Motion proper
  resetOdometry();
  if (rawUnits > 0) {
    motion = MOTION_FORWARD;
    while (wheelPosition < rawUnits) {
      motionSlowDownFactor = 0.0005 * (double)constrain(rawUnits - wheelPosition, 200, 3333);
      balance();
    }
    // motionSlowDownFactor = 1.0;
  }
  else if (rawUnits < 0) {
    motion = MOTION_BACK;
    while (wheelPosition > rawUnits) {
      motionSlowDownFactor = 0.0005 * (double)constrain(wheelPosition - rawUnits, 200, 3333);
      balance();
    }
    // motionSlowDownFactor = 1.0;
  }
  else
    return; // rawUnits is zero
  stop();
}

void LilBot::stop(void) {
  resetOdometry();
  wait(50);  // Let momentum allow a bit of overshoot after resetOdometry()
  // Reverse motion to cancel momentum
  if (motion == MOTION_FORWARD) {
    motion = MOTION_BACK;
    while (wheelPosition > 0 && wheelSpeed > 0) {
      motionSlowDownFactor = constrain(wheelPosition * 0.0005, 0.0, 1.0);
      balance();
    }
  }
  else if (motion == MOTION_BACK) {
    motion = MOTION_FORWARD;
    while (wheelPosition < 0 && wheelSpeed < 0) {
      motionSlowDownFactor = constrain(-wheelPosition * 0.0005, 0.0, 1.0);
      balance();
    }
  }
  motionSlowDownFactor = 1.0;

  motion = MOTION_RECOVER;  // Stabilize before adaptive balancing
  adaptiveAngleBias = savedAdaptiveAngleBias;
  wait(3500);

  motion = MOTION_NONE;     // Resume adaptive balancing
  adaptiveAngleBias = savedAdaptiveAngleBias;
  wait(3500);
  resetOdometry();
  savedAdaptiveAngleBias = adaptiveAngleBias;
}

void LilBot::rotate(double angle) {
  motion = MOTION_NONE;
  botRotation = angle * PI * 5.55555555556e-3;  // Convert degrees to radians
  wait(3000);
}

// emoShield code

void LilBot::emoInit(void) {
  // Initialize emotions
  emoState[0] = blankFace = 0xffff;
  emoState[1] = afraid = 0xffe1 & 0x1ffe & 0xff7f & 0xffdf & 0xfbff & 0xfeff & 0xf7ff;
  emoState[2] = amused = 0xffeb & 0xbffe & 0xff7f & 0xfbff & 0xfeff & 0xf7ff;
  emoState[3] = angry = 0xffeb & 0xbffe & 0xff7f & 0xffdf & 0xfbff & 0xf7ff & 0xefff;
  emoState[4] = blissful = 0xffeb & 0xbffe & 0xff7f & 0xffbf & 0xfbff & 0xfeff & 0xf7ff;
  emoState[5] = cool = 0xffe1 & 0x1ffe & 0xff7f & 0xfbff & 0xfdff & 0xfeff;
  emoState[6] = crying = 0xffeb & 0xbffe & 0xffdf & 0xfbff & 0xfeff & 0xf7ff;
  emoState[7] = disappointed = 0xffe1 & 0x1ffe & 0xffbf & 0xfbff & 0xfeff & 0xf7ff;
  emoState[8] = embarrassed = 0xffe1 & 0x1ffe & 0xff7f & 0xffbf & 0xfbff & 0xfdff & 0xfeff;
  emoState[9] = happy = 0xffe1 & 0x1ffe & 0xff7f;
  emoState[10] = impatient = 0xffe1 & 0x1ffe & 0xffbf & 0xfbff & 0xf7ff & 0xefff;
  emoState[11] = naughty = 0xffe1 & 0x1ffe & 0xff7f & 0xfbff & 0xfdff & 0xefff;
  emoState[12] = neutral = 0xffe1 & 0x1ffe & 0xffbf;
  emoState[13] = nonplussed = 0xffe1 & 0x1ffe & 0xffbf & 0xfbff & 0xfdff & 0xfeff;
  emoState[14] = outraged = 0xffe1 & 0x1ffe & 0xff7f & 0xffdf & 0xfbff & 0xfdff & 0xefff;
  emoState[15] = proud = 0xffe1 & 0x1ffe & 0xff7f & 0xfbff & 0xf7ff & 0xefff;
  emoState[16] = resigned = 0xffe1 & 0x1ffe & 0xffdf & 0xfbff & 0xfdff & 0xfeff;
  emoState[17] = sad = 0xffe1 & 0x1ffe & 0xffdf;
  emoState[18] = sarcastic = 0xffe1 & 0x1ffe & 0xff7f & 0xffbf & 0xfbff & 0xfdff & 0xefff;
  emoState[19] = shocked = 0xffeb & 0xbffe & 0xff7f & 0xffdf & 0xfbff & 0xfeff & 0xf7ff;
  emoState[20] = smiling = 0xffe1 & 0x1ffe & 0xff7f & 0xffbf;
  emoState[21] = verySad = 0xffe1 & 0x1ffe & 0xffdf & 0xfbff & 0xfeff & 0xf7ff;
  emoState[22] = winking = 0xffe1 & 0xbffe & 0xff7f;
}

void LilBot::emote(short int emotion) {
  Wire.beginTransmission(0x27);
  Wire.write((uint8_t)2);  // Start from register 2
  Wire.write((uint8_t)0);
  Wire.write((uint8_t)0);
  Wire.endTransmission();
  Wire.beginTransmission(0x27);  // I/O expander (LED driver) I2C address
  Wire.write((uint8_t)6);  // Start from register 6
  Wire.write((uint8_t)(emotion & 0x00ff));
  Wire.write((uint8_t)((emotion >> 8) & 0x00ff));
  Wire.endTransmission();
}

void LilBot::emoteByNumber(int emotionIndex) {
  emote(emoState[emotionIndex % 22]);
}

// Sound code

// The astromech droid sound table is stored in flash memory to save RAM
// Each sound is stored as a line of 16 unsigned characters predicated on
// a frequency sweep defined by duration, starting frequency and ending frequency
// The 16 characters are divided into four groups, each of which encodes
// a sound sequence thus:
// Byte 1: duration in milliseconds, or zero to skip
// Byte 2: starting frequency in 10Hz increments (e.g. 2000Hz: 200)
// Byte 3: ending frequency
// Byte four: ending pause duration (in addition to tone duration) in milliseconds

const char LilBot::soundTable[][15] PROGMEM = {
  /* 0 */ { 50, 205, 225, 30, 90, 240, 240, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 1 */ { 50, 210, 225, 30, 247, 247, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 2 */ { 50, 200, 270, 10, 30, 305, 305, 0, 60, 305, 100, 0, 0, 0, 0 },
  /* 3 */ { 30, 170, 240, 0, 40, 240, 240, 30, 30, 215, 170, 0, 40, 170, 170 },
  /* 4 */ { 40, 240, 240, 0, 30, 240, 125, 40, 30, 70, 55, 0, 40, 55, 65 },
  /* 5 */ { 70, 50, 210, 40, 50, 240, 240, 0, 30, 240, 120, 0, 0, 0, 0 },
  /* 6 */ { 60, 500, 80, 60, 70, 50, 205, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 7 */ { 50, 175, 170, 30, 40, 200, 220, 30, 40, 235, 205, 0, 0, 0, 0 },
  /* 8 */ { 50, 225, 195, 30, 40, 170, 170, 30, 40, 195, 220, 0, 0, 0, 0 },
  /* 9 */ { 50, 205, 235, 30, 50, 242, 242, 30, 60, 220, 187, 0, 0, 0, 0 },
  /* 10 */ { 60, 230, 200, 20, 50, 190, 210, 40, 50, 250, 260, 0, 0, 0, 0 },
  /* 11 */ { 110, 145, 145, 0, 30, 245, 245, 0, 110, 172, 172, 0, 0, 0, 0 },
  /* 12 */ { 110, 170, 170, 0, 30, 243, 243, 0, 110, 143, 143, 0, 0, 0, 0 },
  /* 13 */ { 60, 60, 70, 30, 70, 175, 240, 30, 70, 220, 175, 0, 0, 0, 0 },
  /* 14 */ { 110, 320, 320, 0, 110, 148, 148, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 15 */ { 100, 145, 145, 0, 120, 3180, 3180, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 16 */ { 240, 265, 420, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 17 */ { 60, 500, 80, 50, 80, 50, 210, 30, 50, 235, 235, 0, 30, 235, 130 },
  /* 18 */ { 50, 175, 170, 30, 40, 197, 217, 30, 50, 227, 197, 30, 60, 187, 197 },
  /* 19 */ { 80, 265, 293, 0, 90, 293, 215, 60, 50, 130, 130, 0, 0, 0, 0 },
  /* 20 */ { 100, 260, 260, 70, 60, 148, 148, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 21 */ { 70, 140, 140, 60, 100, 260, 260, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 22 */ { 60, 60, 60, 40, 70, 175, 245, 30, 60, 215, 175, 30, 50, 200, 255 },
  /* 23 */ { 50, 192, 200, 50, 40, 250, 250, 30, 80, 265, 2940, 0, 100, 294, 222 },
  /* 24 */ { 40, 240, 255, 30, 80, 265, 290, 0, 100, 290, 215, 50, 50, 130, 130 },
  /* 25 */ { 130, 185, 313, 0, 150, 313, 205, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 26 */ { 80, 115, 245, 20, 60, 540, 510, 30, 60, 270, 180, 30, 130, 118, 114 },
  /* 27 */ { 110, 118, 118, 30, 60, 180, 270, 30, 60, 510, 540, 20, 90, 240, 115 },
  /* 28 */ { 60, 70, 340, 0, 110, 340, 340, 0, 90, 340, 115, 0, 80, 115, 115 },
  /* 29 */ { 100, 120, 120, 0, 100, 120, 340, 0, 100, 340, 340, 0, 70, 340, 70 },
  /* 30 */ { 140, 520, 520, 0, 60, 520, 150, 0, 180, 150, 57, 100, 50, 57, 57 },
  /* 31 */ { 50, 57, 57, 100, 200, 57, 150, 0, 50, 150, 520, 0, 130, 520, 520 },
  /* 32 */ { 50, 245, 245, 50, 50, 225, 195, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 33 */ { 30, 305, 305, 0, 70, 305, 100, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 34 */ { 60, 500, 85, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 35 */ { 50, 207, 235, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 36 */ { 200, 200, 415, 0, 100, 415, 250, 50, 100, 185, 185, 0, 100, 160, 160 },
  /* 37 */ { 100, 250, 415, 0, 200, 415, 200, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 38 */ { 100, 147, 147, 0, 100, 174, 174, 30, 110, 318, 318, 0, 180, 145, 145 },
  /* 39 */ { 50, 225, 205, 30, 50, 188, 205, 30, 50, 240, 255, 0, 0, 0, 0 },
  /* 40 */ { 50, 205, 225, 30, 50, 244, 244, 30, 50, 225, 190, 0, 0, 0, 0 },
  /* 41 */ { 50, 224, 197, 30, 50, 174, 174, 30, 50, 200, 222, 0, 0, 0, 0 },
  /* 42 */ { 50, 175, 175, 30, 50, 195, 212, 30, 50, 225, 200, 0, 0, 0, 0 },
  /* 43 */ { 70, 500, 75, 50, 70, 55, 205, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 44 */ { 70, 50, 205, 30, 50, 240, 240, 0, 30, 240, 120, 0, 0, 0, 0 },
  /* 45 */ { 70, 170, 245, 40, 70, 220, 180, 0, 0, 0, 0, 0, 0, 0, 0 },
  /* 46 */ { 50, 200, 275, 20, 40, 305, 305, 0, 60, 305, 105, 0, 0, 0, 0 },
  /* 47 */ { 50, 190, 270, 20, 20, 305, 305, 0, 60, 305, 100, 0, 0, 0, 0 },
  /* 48 */ { 50, 205, 225, 30, 50, 245, 245, 0, 0, 0, 0, 0, 0, 0, 0 } };

void LilBot::sweep(int duration, int startTone, int endTone) {
  // Works better with ms duration a multiple of five
  int steps = duration / 10;
  int toneIncrement = (endTone - startTone) * 10 / duration;

  while (steps--) {
    tone(11, endTone - toneIncrement * steps, 10);
    wait(10);
  }
}

void LilBot::sound(int soundLine) {
  int loopFourTimes = 0;
  int duration, startTone, endTone, pause;

  soundLine %= 49;  // Limit to actual table size

  while (loopFourTimes < 15) {
    duration  = pgm_read_byte(&(soundTable[soundLine][loopFourTimes++]));
    if (duration) {
      startTone = pgm_read_byte(&(soundTable[soundLine][loopFourTimes++])) * 10;
      endTone   = pgm_read_byte(&(soundTable[soundLine][loopFourTimes++])) * 10;
      if (loopFourTimes < 15)
        pause     = pgm_read_byte(&(soundTable[soundLine][loopFourTimes++]));
      else
        pause = 0;
      sweep(duration, startTone, endTone);
      if (pause)
        wait(pause);
    }
  }
  wait(150);
}

void LilBot::say(char *sentence) {
  int i, soundCode;
  int length = strlen(sentence);

  for (i = 0; i < length; i++) {
    if (sentence[i] == ' ') {
      wait(300);
      continue;
    }
    switch (sentence[i]) {
      case ',':
      case ';':
      case ':':
        soundCode = 38;
        break;
      case '@':
        soundCode = 39;
        break;
      case '.':
        soundCode = 40;
        break;
      case '?':
        soundCode = 41;
        break;
      case '!':
        soundCode = 42;
        break;
      case '+':
        soundCode = 43;
        break;
      case '-':
        soundCode = 44;
        break;
      case '*':
        soundCode = 45;
        break;
      case '/':
        soundCode = 46;
        break;
      case '=':
        soundCode = 47;
        break;
      default:
        if (sentence[i] >= 'A' && sentence[i] <= 'Z')
          soundCode = (int)(sentence[i] - 'A');
        else if (sentence[i] >= 'a' && sentence[i] <= 'z')
          soundCode = (int)(sentence[i] - 'a');
        else if (sentence[i] >= '0' && sentence[i] <= '9')
          soundCode = 26 + (int)(sentence[i] - '0');
        else
          soundCode = 48;
        break;
    }
    sound(soundCode);
  }
}

// Proximity-sensor code

char LilBot::proxRead(char regAddress) {
  Wire.beginTransmission(0x5a); // Si1143 address
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(0x5a, 1);
  while (!Wire.available())
    ;
  return Wire.read();
}

void LilBot::proxWrite(char subAddress, char data) {
  Wire.beginTransmission(0x5a); // Si1143 address
  Wire.write(subAddress);       // Register address
  Wire.write(data);
  Wire.endTransmission();
}

void LilBot::proxParamWrite(char param, char data) {
  Wire.beginTransmission(0x5a); // Si1143 address
  Wire.write(0x18);             // Command register address
  Wire.write(0);                // NOP to clear response register
  Wire.endTransmission();
  Wire.beginTransmission(0x5a); // Si1143 address
  Wire.write(0x17);             // PARAM register address
  Wire.write(data);             // Parameter value
  Wire.write(0xa0 | param);     // Set parameter command and parameter address
  Wire.endTransmission();
  while (proxRead(0x20) == 0)   // Poll response register to let
    ;                           // Si1143 update its parameter RAM
}

void LilBot::proxReset(void) {
  Wire.beginTransmission(0x5a); // Si1143 address
  Wire.write(0x18);             // Command register address
  Wire.write(1);                // Reset command
  Wire.endTransmission();
  wait(30);                    // Required reset time
}

void LilBot::proxStart(void) {
  Wire.beginTransmission(0x5a); // Si1143 address
  Wire.write(0x18);             // Command register address
  Wire.write(7);                // PSALS_FORCE
  Wire.endTransmission();
}

void LilBot::initProx(void) {
  pinMode(5, INPUT);          // Pin 5 is interrupt from Si1143, not used
  proxWrite(7, 0x17);         // Hardware key
  proxReset();
  proxWrite(7, 0x17);         // Hardware key
  proxWrite(3, 1);            // Interrupt pin mode
  proxWrite(4, 0x1d);         // Enable interrupts
  proxWrite(15, 0xff);        // LED1 and LED2 at maximum current
  proxWrite(16, 0x0f);        // LED3 at maximum current
  proxParamWrite(1, 0x7);     // Enable PS1, PS2, and PS3
  proxParamWrite(2, 0x21);    // LED1 for PS1 and LED2 for PS2
  proxParamWrite(3, 4);       // LE3 for PS3
  // proxParamWrite(7, 0);       // Small IR photodiode for PS1
  // proxParamWrite(8, 0);       // Small IR photodiode for PS2
  // proxParamWrite(9, 0);       // Small IR photodiode for PS3
  proxParamWrite(7, 3);       // Large IR photodiode for PS1
  proxParamWrite(8, 3);       // Large IR photodiode for PS2
  proxParamWrite(9, 3);       // Large IR photodiode for PS3
  proxParamWrite(0xa, 0x50);  // Proximity recovery time
  proxParamWrite(0xb, 2);     // Proximity gain
  proxParamWrite(0xc, 0x24);  // PS_ADC_MISC
  proxParamWrite(0xe, 0);     // Small IR photodiode for ALS
  proxParamWrite(0x10, 0x60); // Ambient-light recovery time
  proxParamWrite(0x11, 1);    // Ambient-light gain
  proxParamWrite(0x12, 0x20); // Ambient-light gain range
  botProx.event = PROX_NONE;
  botProx.enableEventHandlers = false;
}

char LilBot::proxParamRead(char param) {
  Wire.beginTransmission(0x5a); // Si1143 address
  Wire.write(0x18);             // Command register address
  Wire.write(0);                // NOP to clear response register
  Wire.endTransmission();
  Wire.beginTransmission(0x5a); // Si1143 address
  Wire.write(0x18);             // Command register address
  Wire.write(0x80 | param);     // Set parameter command and parameter address
  Wire.endTransmission();
  while (proxRead(0x20) == 0)   // Poll response register to let
    ;                           // Si1143 fetch parameter RAM data
  return proxRead(0x2e);
}

void LilBot::proxGetResults(void) {
  static bool firstMeasurement = true;

  Wire.beginTransmission(0x5a); // Si1143 address
  Wire.write(0x26);             // First register address
  Wire.endTransmission();
  Wire.requestFrom(0x5a, 6);    // Request 6 consecutive reads
  while (!Wire.available())     // Wait for first byte to be available
    ;
  botProx.right = (Wire.read() & 0x00ff);
  botProx.right |= ((Wire.read() << 8) & 0xff00);
  botProx.left = (Wire.read() & 0x00ff);
  botProx.left |= ((Wire.read() << 8) & 0xff00);
  botProx.edge = (Wire.read() & 0x00ff);
  botProx.edge |= ((Wire.read() << 8) & 0xff00);

  botProx.right -= (int)(botProx.rightBaseline);
  botProx.left -= (int)(botProx.leftBaseline);
  botProx.edge -= (int)(botProx.edgeBaseline);

  if (firstMeasurement) {
    // Prime filter with reasonable values
    botProx.filteredRight = botProx.right * 16;
    botProx.filteredLeft = botProx.left * 16;
    botProx.filteredEdge = botProx.edge * 16;
    firstMeasurement = false;
  }
  else {
    // Filter by a factor of sixteen
    botProx.filteredRight = botProx.filteredRight - (botProx.filteredRight >> 4) +
      botProx.right;
    botProx.filteredLeft = botProx.filteredLeft - (botProx.filteredLeft >> 4) +
      botProx.left;
    botProx.filteredEdge = botProx.filteredEdge - (botProx.filteredEdge >> 4) +
      botProx.edge;
  }
  
  Serial.println(botProx.filteredEdge);

  if (botProx.filteredRight > 60 && botProx.filteredLeft > 60)
    botProx.event = PROX_FRONT;
  else if (botProx.filteredRight > 400)
    botProx.event = PROX_RIGHT;
  else if (botProx.filteredLeft > 400)
    botProx.event = PROX_LEFT;
  else if (botProx.filteredEdge < -300)
    botProx.event = PROX_EDGE;
  else
    botProx.event = PROX_NONE;
  if (botProx.enableEventHandlers) {
    botProx.enableEventHandlers = false;
    switch (botProx.event) {
      case PROX_FRONT:
        (*frontObstacleHandler)();
        break;
      case PROX_RIGHT:
        (*rightObstacleHandler)();
        break;
      case PROX_LEFT:
        (*leftObstacleHandler)();
        break;
      case PROX_EDGE:
        (*edgeHandler)();
        break;
      default:  // No detection
        break;
    }
    botProx.enableEventHandlers = true;
  }
}

// Balancing

void LilBot::readIMU(void) {
  int IMURawAccelX;
  int IMURawAccelY;      // Not used, but part of sequential I2C read
  int IMURawAccelZ;
  int IMURawTemperature; // Not used, but part of sequential I2C read
  int IMURawGyroX, IMURawGyroY;
  int redo = 0;
  static double previousScaledXAccel = 0.0;
  static double previousScaledZAccel = 0.0;
  static double previousScaledXGyro = 0.0;
  static double previousScaledYGyro = 0.0;

  Wire.beginTransmission(0x68);
  Wire.write(59);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 12);

  if(!Wire.available())
    goto returnWithError;
  IMURawAccelX = (Wire.read() << 8);

  if(!Wire.available())
    goto returnWithError;
  IMURawAccelX |= (Wire.read() & 0x00ff);

  if(!Wire.available())
    goto returnWithError;
  IMURawAccelY = (Wire.read() << 8);

  if(!Wire.available())
    goto returnWithError;
  IMURawAccelY |= (Wire.read() & 0x00ff);

  if(!Wire.available())
    goto returnWithError;
  IMURawAccelZ = (Wire.read() << 8);

  if(!Wire.available())
    goto returnWithError;
  IMURawAccelZ |= (Wire.read() & 0x00ff);

  if(!Wire.available())
    goto returnWithError;
  IMURawTemperature = (Wire.read() << 8);

  if(!Wire.available())
    goto returnWithError;
  IMURawTemperature |= (Wire.read() & 0x00ff);

  if(!Wire.available())
    goto returnWithError;
  IMURawGyroX = (Wire.read() << 8);

  if(!Wire.available())
    goto returnWithError;
  IMURawGyroX |= (Wire.read() & 0x00ff);

  if(!Wire.available())
    goto returnWithError;
  IMURawGyroY = (Wire.read() << 8);

  if(!Wire.available())
    goto returnWithError;
  IMURawGyroY |= (Wire.read() & 0x00ff);

  scaledXAccel = (double)IMURawAccelX * acceleratometerScale;
  scaledZAccel = (double)IMURawAccelZ * acceleratometerScale;
  scaledXGyro = (double)IMURawGyroX * gyroScale - gyroOffsetX;
  scaledYGyro = (double)IMURawGyroY * gyroScale - gyroOffsetY;
  previousScaledXAccel = scaledXAccel;
  previousScaledZAccel = scaledZAccel;
  previousScaledXGyro = scaledXGyro;
  previousScaledYGyro = scaledYGyro;
  return;

  returnWithError:

  Serial.println("\nRestart");
  TWCR = 0;
  Wire.begin(); // I2C interface
  TWBR = 12;    // Set I2C interface clock to 400kHz

  scaledXAccel = previousScaledXAccel;
  scaledZAccel = previousScaledZAccel;
  scaledXGyro = previousScaledXGyro;
  scaledYGyro = previousScaledYGyro;
}

void LilBot::wheels(double torque) {
  double torqueR = torque;
  double torqueL = torque;
  double spinRight = 0.0;
  double spinLeft = 0.0;
  // Fuzzy estimation of wheel direction based on consistency of motor current sign
  // fuzzyHigh = 1 and fuzzyLow = -1: no fuzziness
  const int fuzzyHigh = 5;
  const int fuzzyLow = -5;
  double voltageCorrection = 410.0 / (double)batteryVoltage;

  spinRight = constrain(botRotation * 3.5, -0.25, 0.25);
  spinLeft = -spinRight;
  torqueR = constrain((torqueR + spinRight) * voltageCorrection, -1 , 1);
  torqueL = constrain((torqueL + spinLeft) * voltageCorrection, -1 , 1);

  if (torque == -2.0) {  // Stop
    analogWrite(RIGHT_PWM_PIN, 255);
    analogWrite(LEFT_PWM_PIN, 255);
    return;
  }

  if (torqueR > 0.0) {  // Forward
    if (rightForward > 0)
      rightForward = constrain(rightForward + 1, fuzzyLow, fuzzyHigh);
    else
      rightForward = 1;
    digitalWrite(RIGHT_FORWARD_PIN, HIGH);
  }
  else {                // Backward
    if (rightForward < 0)
      rightForward = constrain(rightForward - 1, fuzzyLow, fuzzyHigh);
    else
      rightForward = -1;
     digitalWrite(RIGHT_FORWARD_PIN, LOW);
  }
  analogWrite(RIGHT_PWM_PIN, 255 - abs((int)(torqueR * 255)));

  if (torqueL > 0.0) {  // Forward
    if (leftForward > 0)
      leftForward = constrain(leftForward + 1, fuzzyLow, fuzzyHigh);
    else
      leftForward = 1;
     digitalWrite(LEFT_FORWARD_PIN, HIGH);
  }
  else {                // Backward
    if (leftForward < 0)
      leftForward = constrain(leftForward - 1, fuzzyLow, fuzzyHigh);
    else
      leftForward = -1;
    digitalWrite(LEFT_FORWARD_PIN, LOW);
  }
  analogWrite(LEFT_PWM_PIN, 255 - abs((int)(torqueL * 255)));
}

void LilBot::writeIMU(char subAddress, char data) {
  Wire.beginTransmission(0x68);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

double LilBot::controllerPID() {
  static double currentAngle = accelYAngle;

  double K = 0.01;  // Overall torque gain factor
  double Kp = 8.0;
  double Kp2 = 16.0;
  double Ki = 3.0;
  double Kd = 15.0;
  double pTerm, p2Term, iTerm, dTerm, output, angleError;

  currentAngle = 0.98 * (currentAngle + dTime * scaledYGyro) + 0.02 * accelYAngle;
  botRotation += dTime * scaledXGyro * calVariables.IMUCalData[0];

  angleError = targetAngle + adaptiveAngleBias - currentAngle + tiltFiltered;

  angleError *= -57.29578; // Radians to degrees

  pTerm = Kp * angleError;

  // Error squared with sign preserved, great when falling (not used when Kp2 = 0.0)
  p2Term = Kp2 * angleError * fabs(angleError);

  integratedError = constrain(integratedError + angleError, -50, 50);
  iTerm = Ki * integratedError;
  dTerm = Kd * (angleError - lastError);
  lastError = angleError;

  output = K * (pTerm + p2Term + iTerm + dTerm);

  if (motion == MOTION_NONE)  // Do not adapt if there is a deliberate tilt
    adaptiveAngleBias -= output * 1.5e-4;

  return output ;
}

void LilBot::checkBattery(bool reportBatteryVoltage) {
  batteryVoltage = analogRead(1); // 14.648mV per LSB
  int blinkTime = 256 & (int)millis();

  if (reportBatteryVoltage) {
    Serial.print("\nBattery: ");
    Serial.print(0.014648 * (double)batteryVoltage, 3);
    Serial.println("V");
  }
  if (batteryVoltage < 454 && blinkTime == 256) // 6.65V (950mV per cell) warning
    digitalWrite(LED, HIGH);
  // LED blinks between 6V and 6.5V or is continuously on below 6.123V (875mV per cell)
  if (blinkTime == 0 && batteryVoltage > 418)
    digitalWrite(LED, LOW);
}

void LilBot::balance(void) {
  int balanceFormula;
  unsigned long startTime;  // To measure loop execution time
  static bool balancing = false;
  static int validAngle = 0;
  static bool firstProx = true;

  if (digitalRead(IMU_INTERRUPT) == HIGH) {
    startTime = micros();  // To time balancing loop

    readIMU();

    wheelPosition = rightWheelPosition + leftWheelPosition;
    wheelSpeed = wheelPosition - wheelLastPosition;
    wheelLastPosition = wheelPosition;

    switch (motion) {
      case MOTION_FORWARD:
        tiltForMotion = (0.028 - 1e-3 * (double)wheelSpeed) * motionSlowDownFactor;
        break;
      case MOTION_BACK:
        tiltForMotion = (-0.028 - 1e-3 * (double)wheelSpeed) * motionSlowDownFactor;
        break;
      default:
        // Stop
        tiltForMotion = 0.0;
        tiltFiltered = 0.0;
    }
    tiltFiltered = 0.99 * tiltFiltered + 0.01 * tiltForMotion;

    accelYAngle = -atan2(scaledXAccel, scaledZAccel);

    angle = 0.95 * angle + 0.05 * accelYAngle;  // Low pass filter

    if (fabs(accelYAngle - targetAngle) < (PI / 8))
    {
      if (!balancing && validAngle++ == 20) {
        balancing = true;
        Serial.println("Balancing");
        rightWheelPosition = 0;
        leftWheelPosition = 0;
        integratedError = 0.0;
        lastError = 0.0;
        tiltFiltered = 0.0;
        adaptiveAngleBias = savedAdaptiveAngleBias;
        emote(happy);
      }
    }
    else
      validAngle = 0;

    if (balancing) {
      if (fabs(angle - targetAngle) > (PI / 4)) {
        balancing = false;
        Serial.println("Not balancing");
        wheels(-2.0);  // Stop
        emote(sad);
      }
      else {
        wheels(controllerPID());
        if (firstProx) {
          proxStart();
          firstProx = false;
        }
        else {
          proxGetResults();
          proxStart();
        }
      }
    }
    else
      wheels(-2.0);  // Stop
    checkBattery(false);
    // Serial.print("Loop time: "); Serial.println(micros() - startTime);
  }
}

void LilBot::resetOdometry(void) {
  wheelLastPosition = wheelPosition = rightWheelPosition = leftWheelPosition = 0;
}

void LilBot::initIMU(void)
{
  // MPU-6050 initialization
  writeIMU(107, 0);         // MPU6050_RA_PWR_MGMT_1  0x6B   Get out of reset
  writeIMU(25, sampleDiv);  // MPU6050_RA_SMPLRT_DIV 0x19
  writeIMU(26, 1);          // MPU6050_RA_CONFIG 0x1A   186Hz low-pass filter, 2ms delay
  writeIMU(27, gyroRange);  // MPU6050_RA_GYRO_CONFIG 0x1B  
  writeIMU(28, accelRange); // MPU6050_RA_ACCEL_CONFIG 0x1C

  // For combining filter
  // First, apply radians/second/LSB conversion factor
  switch (gyroRange) {
    case IMU_250_DPS:
      dtScaled = 133.15805e-6;
      gyroScale = 1.0 / 131.072;
      break;
    case IMU_500_DPS:
      dtScaled = 266.31611e-6;
      gyroScale = 1.0 / 65.536;
      break;
    case IMU_1000_DPS:
      dtScaled = 532.63222e-6;
      gyroScale = 1.0 / 32.768;
      break;
    case IMU_2000_DPS:
    default:  // Error in gyroscope scale encoding
      dtScaled = 1.0652644e-3;
      gyroScale = 1.0 / 16.384;

  }
  gyroScale *= 17.45329252e-3;  // Degrees to radians 

  // Next, scale the conversion factor to the sampling rate
  samplingPeriod = 0.01 * (1.0 + (double) sampleDiv);
  dtScaled *= samplingPeriod;
  // Convert acceleration from g to m/s per sampling period
  // First, scale to m/s/s/LSB
  switch (accelRange) {
    case IMU_2_G:
      // 2g, or 19.6133 m/s/s, is 32768 counts
      accelerationScaleFactor = 19.6133 / 32768.0;
      acceleratometerScale = (1.0 / 16384.0);
      break;
    case IMU_4_G:
      // 4g, or 39.2266 m/s/s, is 32768 counts
      accelerationScaleFactor = 39.2266 / 32768.0;
      acceleratometerScale =(1.0 / 8192.0);
      break;
    case IMU_8_G:
      // 8g, or 78.4532 m/s/s, is 32768 counts
      accelerationScaleFactor = 78.4532 / 32768.0;
      acceleratometerScale =(1.0 / 4096.0);
      break;
    case IMU_16_G:

      // 16g, or 156.9064 m/s/s, is 32768 counts
      accelerationScaleFactor = 156.9064 / 32768.0;
      acceleratometerScale =(1.0 / 2048.0);
      break;
    default:
      break;
  }
  // Next, scale to sampling period
  accelerationScaleFactor *= samplingPeriod;

  writeIMU(55, IMU_LATCH_INT_EN_HOLD | IMU_INT_RD_CLEAR);
  writeIMU(56, IMU_DATA_RDY_EN);  // Interrupt on data ready
}
