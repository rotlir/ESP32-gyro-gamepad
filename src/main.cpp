#include <Arduino.h>
#include <BleGamepad.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <EncButton.h>

//ABXY BUTTONS
#define X_BUTTON 25         // A
#define CIRCLE_BUTTON 26    // B
#define TRIANGLE_BUTTON 33  // Y
#define SQUARE_BUTTON 32    // X

//TRIGGERS
#define R1_BUTTON 0
#define R2_BUTTON 0
#define L1_BUTTON 0
#define L2_BUTTON 0

//MENU BUTTONS
#define START_BUTTON 0
#define SELECT_BUTTON 0
#define PS_BUTTON 0

//JOYSTICKS BUTTONS
#define R3_BUTTON 12
#define L3_BUTTON 18

//JOYSTICK
#define LEFT_VRX_JOYSTICK 14
#define LEFT_VRY_JOYSTICK 34
#define RIGHT_VRX_JOYSTICK 4
#define RIGHT_VRY_JOYSTICK 15

//HAT
#define HAT_BTN_UP 23
#define HAT_BTN_DOWN 16
#define HAT_BTN_RIGHT 17
#define HAT_BTN_LEFT 13

#define NUM_BUTTONS 13

#define MPU_INT 27

#define MODE_BUTTON 19

#define LED 2 // built-in led

// if you experience any problems with the positioning of a joystick, try setting one of these values to true
const bool rJoystickCalNegative = false;
const bool lJoystickCalNegative = false;

int buttonsPins[NUM_BUTTONS] = {X_BUTTON, CIRCLE_BUTTON, TRIANGLE_BUTTON, SQUARE_BUTTON,
                          R1_BUTTON, R2_BUTTON, L1_BUTTON, L2_BUTTON,
                          START_BUTTON, SELECT_BUTTON, PS_BUTTON,
                          R3_BUTTON, L3_BUTTON};

int hatPins[4] = {HAT_BTN_UP, HAT_BTN_DOWN, HAT_BTN_RIGHT, HAT_BTN_LEFT};

byte androidGamepadButtons[NUM_BUTTONS] = {1, 2, 3, 4, 8, 10, 7, 9, 12, 11, 13, 15, 14};
byte PS1GamepadButtons[NUM_BUTTONS] = {2, 3, 4, 1, 6, 8, 5, 7, 10, 9, 13, 12, 11};
byte PCGamepadButtons[NUM_BUTTONS] = {1, 2, 4, 3, 6, 8, 5, 7, 10, 9, 0, 12, 11};

Quaternion q;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;
// calibration values for joysticks
uint16_t lJoystickXCalValue;
uint16_t lJoystickYCalValue;
uint16_t rJoystickXCalValue;
uint16_t rJoystickYCalValue;
bool gyroCurJoystick = true; // true is right joystick, false is left joystick

BleGamepad gamepad("rotlir gamepad", "rotlir");
MPU6050 mpu;
bool gyroMode = false;

Button btn(MODE_BUTTON);
typedef enum{ANDROID, PC} GamepadModes;
GamepadModes gamepadMode = ANDROID;

void joysticksHandlerForMobile(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry);
void joysticksHandlerForPC(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry);
void roundQuat(Quaternion *quat);
float mapFloat(float val, float oldMin, float oldMax, float newMin, float newMax);
void calibrateJoystick(const byte num, bool firstLaunch); // 0 is left joystick, 1 is right joystick
int checkHat();
void blink();

IRAM_ATTR void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  //Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(LEFT_VRX_JOYSTICK, INPUT);
  pinMode(LEFT_VRY_JOYSTICK, INPUT);
  pinMode(RIGHT_VRX_JOYSTICK, INPUT);
  pinMode(RIGHT_VRY_JOYSTICK, INPUT);

  // joysticks calibration
  Serial.println("calibrating left joystick");
  calibrateJoystick(0, true);
  long timer = millis() + 500;
  while(millis() <= timer) {
    calibrateJoystick(0, false);
  }
  Serial.println("calibrating right joystick");
  calibrateJoystick(1, true);
  timer = millis() + 500;
  while(millis() <= timer) {
    calibrateJoystick(1, false);
  }

  for(int i=0; i<NUM_BUTTONS; i++){
    pinMode(buttonsPins[i], INPUT_PULLUP);
    
  }
  for(int i = 0; i < 4; i++) {
    pinMode(hatPins[i], INPUT_PULLUP);
  }
  BleGamepadConfiguration conf;
  conf.setButtonCount(NUM_BUTTONS);
  gamepad.begin(&conf);
  Serial.println("waiting for bluetooth");
  while(!gamepad.isConnected()) {
    Serial.print(".");
    delay(1000);
  }
  Serial.print("\n");
  Serial.println("initializing mpu");
  mpu.initialize();
  pinMode(MPU_INT, INPUT);
  Serial.println("initializing dmp");
  devStatus = mpu.dmpInitialize();
  Serial.println("calibrating gyroscope");
  mpu.CalibrateGyro();
  Serial.println("calibrating accelerometer");
  mpu.CalibrateAccel();
  Serial.printf("lJoystickXCalValue: %i\n", lJoystickXCalValue);
  Serial.printf("lJoystickYCalValue: %i\n", lJoystickYCalValue);
  Serial.printf("rJoystickXCalValue: %i\n", rJoystickXCalValue);
  Serial.printf("rJoystickYCalValue: %i\n", rJoystickYCalValue);
  if(devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("init succesful");
  }
  else {
    Serial.println("Error!");
    ESP.restart();
  }
  digitalWrite(LED, HIGH);
}

void loop() {
  if(gamepad.isConnected()) {
    static uint16_t lJoystickValueX;
    static uint16_t lJoystickValueY;
    static uint16_t rJoystickValueX;
    static uint16_t rJoystickValueY;
    btn.tick();
    if(btn.hasClicks(3)) {
      gyroMode = !gyroMode;
      blink();
    }
    if(gyroMode && btn.hasClicks(1)) {
      digitalWrite(LED, LOW);
      Serial.println("recalibrating");
      mpu.CalibrateGyro();
      mpu.CalibrateAccel();
      digitalWrite(LED, HIGH);
    }
    if(gyroMode && btn.hasClicks(2)) {
      gyroCurJoystick = !gyroCurJoystick;
    }
    if(btn.releaseHold()) {
      switch(gamepadMode) {
        case ANDROID: {
          gamepadMode = PC;
          blink();
          break;
        }
        case PC: {
          gamepadMode = ANDROID;
          blink();
          break;
        }
      }
    }
    lJoystickValueX = analogRead(LEFT_VRX_JOYSTICK);
    lJoystickValueY = analogRead(LEFT_VRY_JOYSTICK);
    if(!gyroMode || (gyroMode && gyroCurJoystick)) {
    if(lJoystickValueX > 2047) {
      lJoystickValueX -= lJoystickXCalValue;
    } else {
      lJoystickValueX += lJoystickXCalValue;
    }
    if(lJoystickValueY > 2047) {
      lJoystickValueY -= lJoystickYCalValue;
    } else {
      lJoystickValueY += lJoystickYCalValue;
    }
    if(!gyroMode) {
      lJoystickValueX = map(lJoystickValueX, 4095 + lJoystickXCalValue, 0 - lJoystickXCalValue, 32737, 0);
      lJoystickValueY = map(lJoystickValueY, 0 - lJoystickYCalValue, 4095 + lJoystickYCalValue, 32737, 0);
      }
    else {
      uint16_t buf = lJoystickValueX;
      lJoystickValueX = map(lJoystickValueY, 4095 + lJoystickYCalValue, 0 - lJoystickYCalValue, 0, 32737);
      lJoystickValueY = map(buf, 0 - lJoystickXCalValue, 4095 + lJoystickXCalValue, 0, 32737);
      }
    }

    if(!gyroMode || (gyroMode && !gyroCurJoystick)) {
      rJoystickValueX = analogRead(RIGHT_VRX_JOYSTICK);
      rJoystickValueY = analogRead(RIGHT_VRY_JOYSTICK);
      if(!rJoystickCalNegative) {
      if(rJoystickValueX > 2047) {
      rJoystickValueX -= rJoystickXCalValue;
    } else {
      rJoystickValueX += rJoystickXCalValue;
    }
    if(rJoystickValueY > 2047) {
      rJoystickValueY -= rJoystickYCalValue;
    } else {
      rJoystickValueY += rJoystickYCalValue;
    }
      
    }
    } else {
      if(rJoystickValueX > 2047) {
      rJoystickValueX += rJoystickXCalValue;
    } else {
      rJoystickValueX -= rJoystickXCalValue;
    }
    if(rJoystickValueY > 2047) {
      rJoystickValueY += rJoystickYCalValue;
    } else {
      rJoystickValueY -= rJoystickYCalValue;
    }
    }
    if(!gyroMode) {
      rJoystickValueX = map(rJoystickValueX, 4095 + rJoystickXCalValue, 0 - rJoystickXCalValue, 0, 32737);
      rJoystickValueY = map(rJoystickValueY, 0 - rJoystickYCalValue, 4095 + rJoystickYCalValue, 0, 32737);
      }
    else {
      uint16_t buf = rJoystickValueX;
      rJoystickValueX = map(rJoystickValueY, 4095 + rJoystickYCalValue, 0 - rJoystickYCalValue, 32737, 0);
      rJoystickValueY = map(buf, 0 - rJoystickXCalValue, 4095 + rJoystickXCalValue, 32737, 0);
      }
    

    if(gyroMode && dmpDataReady) {
    while(!mpuInterrupt && fifoCount < packetSize) {
      if (mpuInterrupt && fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
      }
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          fifoCount = mpu.getFIFOCount();

      } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          fifoCount -= packetSize;

          mpu.dmpGetQuaternion(&q, fifoBuffer);
          roundQuat(&q);
          //Serial.printf("quat %f %f %f %f\n", q.w, q.x, q.y, q.z);
          if(gyroCurJoystick) {
            rJoystickValueX = (uint16_t)mapFloat(q.x, 0.75f, -0.75f, 0.0f, 32767.0f);
            rJoystickValueY = (uint16_t)mapFloat(q.y, 0.75f, -0.75f, 32767.0f, 0.0f);
            if(rJoystickValueX > 32767) rJoystickValueX = 32767;
            if(rJoystickValueY > 32767) rJoystickValueY = 32767;
          } else {
            lJoystickValueX = (uint16_t)mapFloat(q.y, 0.75f, -0.75f, 32767.0f, 0.0f);
            lJoystickValueY = (uint16_t)mapFloat(q.x, 0.75f, -0.75f, 0.0f, 32767.0f);
            if(lJoystickValueX > 32767) lJoystickValueX = 32767;
            if(lJoystickValueY > 32767) lJoystickValueY = 32767;
          }
  }
    }
    switch(gamepadMode){
        case ANDROID:
          for(int i=0; i<NUM_BUTTONS; i++){
            if(!digitalRead(buttonsPins[i])){
                gamepad.press(androidGamepadButtons[i]);   
            }
            else{
                gamepad.release(androidGamepadButtons[i]);     
            }
            joysticksHandlerForMobile(lJoystickValueX, lJoystickValueY, rJoystickValueX, rJoystickValueY);
            gamepad.setHat1(checkHat());
          } 
          break;

          case PC:
            for(int i=0; i<NUM_BUTTONS; i++){
              if(!digitalRead(buttonsPins[i])){
                gamepad.press(PCGamepadButtons[i]);
              }
              else{
                gamepad.release(PCGamepadButtons[i]);
              }
              joysticksHandlerForPC(lJoystickValueX, lJoystickValueY, rJoystickValueX, rJoystickValueY);
              gamepad.setHat1(checkHat());
            }
            break;
      }
  }
}

void joysticksHandlerForMobile(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry){
  if(gyroMode && gyroCurJoystick) {
    gamepad.setX(leftVrx);
    gamepad.setY(leftVry);
  }
  else
    gamepad.setLeftThumb(leftVry, leftVrx);
  if(gyroMode && !gyroCurJoystick)
    gamepad.setRightThumb(rightVry, rightVrx);
  else
    gamepad.setRightThumb(rightVrx, rightVry);
}

void joysticksHandlerForPC(uint16_t leftVrx, uint16_t leftVry, uint16_t rightVrx, uint16_t rightVry){
  if(gyroMode && gyroCurJoystick) {
    gamepad.setX(leftVrx);
    gamepad.setY(leftVry);
  }
  else {
    gamepad.setX(leftVry);
    gamepad.setY(leftVrx);
  }
  if(gyroMode && !gyroCurJoystick) {
    gamepad.setZ(rightVry);
    gamepad.setRZ(rightVrx);
  }
  else {
    gamepad.setZ(rightVrx);
    gamepad.setRZ(rightVry);
  }
}

void roundQuat(Quaternion *quat) {
  quat->w = std::ceil(quat->w * 100) / 100;
  quat->x = std::ceil(quat->x * 100) / 100;
  quat->y = std::ceil(quat->y * 100) / 100;
  quat->z = std::ceil(quat->z * 100) / 100;
}

float mapFloat(float val, float oldMin, float oldMax, float newMin, float newMax) {
  float out = (((val - oldMin) * (newMax - newMin)) / (oldMax - oldMin) + newMin);
  out = (out < 0) ? 0 : out;
  return out;
}

void calibrateJoystick(const byte num, bool firstLaunch) {
  static uint16_t lJoystickXMax;
  static uint16_t lJoystickXMin;
  static uint16_t lJoystickYMax;
  static uint16_t lJoystickYMin;

  static uint16_t rJoystickXMax;
  static uint16_t rJoystickXMin;
  static uint16_t rJoystickYMax;
  static uint16_t rJoystickYMin;

  uint16_t lJoystickX;
  uint16_t lJoystickY;
  uint16_t rJoystickX;
  uint16_t rJoystickY;

  if(num == 0 && firstLaunch) {
    lJoystickXMax = 2047;
    lJoystickXMin = 2047;
    lJoystickYMax = 2047;
    lJoystickYMin = 2047;
    return;
  }

  if(num == 1 && firstLaunch) {
    rJoystickXMax = 2047;
    rJoystickXMin = 2047;
    rJoystickYMax = 2047;
    rJoystickYMin = 2047;
    return;
  }

  if(num == 0) {
    lJoystickX = analogRead(LEFT_VRX_JOYSTICK);
    lJoystickY = analogRead(LEFT_VRY_JOYSTICK);
    if(lJoystickX > lJoystickXMax) {
      lJoystickXMax = lJoystickX;
    }
    if(lJoystickX < lJoystickXMin) {
      lJoystickXMin = lJoystickX;
    }
    if(lJoystickY > lJoystickYMax) {
      lJoystickYMax = lJoystickY;
    }
    if(lJoystickY < lJoystickYMin) {
      lJoystickYMin = lJoystickY;
    }
    int lJoystickXAverage = (lJoystickXMax + lJoystickXMin) / 2;
    lJoystickXCalValue = abs(2047 - lJoystickXAverage);
    int lJoystickYAverage = (lJoystickYMax + lJoystickXMin) / 2;
    lJoystickYCalValue = abs(2047 - lJoystickXAverage);
  }
  if(num == 1) {
    rJoystickX = analogRead(RIGHT_VRX_JOYSTICK);
    rJoystickY = analogRead(RIGHT_VRY_JOYSTICK);
    if(rJoystickX > rJoystickXMax) {
      rJoystickXMax = rJoystickX;
    }
    if(rJoystickX < rJoystickXMin) {
      rJoystickXMin = rJoystickX;
    }
    if(rJoystickY > rJoystickYMax) {
      rJoystickYMax = rJoystickY;
    }
    if(rJoystickY < rJoystickYMin) {
      rJoystickYMin = rJoystickY;
    }
    int rJoystickXAverage = (rJoystickXMax + rJoystickXMin) / 2;
    rJoystickXCalValue = abs(2047 - rJoystickXAverage);
    int rJoystickYAverage = (rJoystickYMax + rJoystickXMin) / 2;
    rJoystickYCalValue = abs(2047 - rJoystickY);
  }
}

int checkHat() {
  if((!digitalRead(HAT_BTN_UP) && !digitalRead(HAT_BTN_DOWN)) || (!digitalRead(HAT_BTN_LEFT) && !digitalRead(HAT_BTN_RIGHT))) 
    return HAT_CENTERED;
  else if (!digitalRead(HAT_BTN_UP) && !digitalRead(HAT_BTN_RIGHT))
    return HAT_UP_RIGHT;
  else if (!digitalRead(HAT_BTN_UP) && !digitalRead(HAT_BTN_LEFT)) 
    return HAT_UP_LEFT;
  else if (!digitalRead(HAT_BTN_UP))
    return HAT_UP;
  else if (!digitalRead(HAT_BTN_DOWN) && !digitalRead(HAT_BTN_RIGHT))
    return HAT_DOWN_RIGHT;
  else if (!digitalRead(HAT_BTN_DOWN) && !digitalRead(HAT_BTN_LEFT)) 
    return HAT_DOWN_LEFT;
  else if (!digitalRead(HAT_BTN_DOWN))
    return HAT_DOWN;
  else if (!digitalRead(HAT_BTN_LEFT))
    return HAT_LEFT;
  else if (!digitalRead(HAT_BTN_RIGHT))
    return HAT_RIGHT;
  else return HAT_CENTERED;
}

void blink() {
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
}
