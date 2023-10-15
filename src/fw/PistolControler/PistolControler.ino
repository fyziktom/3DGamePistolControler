#include <M5StickC.h>  // https://github.com/m5stack/M5StickC 
#include <BleMouse.h>  // https://github.com/T-vK/ESP32-BLE-Mouse

#define LEFT_BUTTON 33
#define RIGHT_BUTTON 32

#define ACC_BUFFER_SIZE 32
#define DIFF_BUFFER_SIZE 8

float accBufferX[ACC_BUFFER_SIZE] = {0};
float accBufferY[ACC_BUFFER_SIZE] = {0};
float accBufferZ[ACC_BUFFER_SIZE] = {0};

float diffBufferX[DIFF_BUFFER_SIZE] = {0};
float diffBufferY[DIFF_BUFFER_SIZE] = {0};
float diffBufferZ[DIFF_BUFFER_SIZE] = {0};

int accIndex = 0;
int diffIndex = 0;

float avgX = 0, avgY = 0, avgZ = 0;
float accX = 0, accY = 0, accZ = 0;
float avgDiffX = 0, avgDiffY = 0, avgDiffZ = 0;

boolean isActivated     = true;
boolean buttonIsPressed = false;
boolean tipped          = false;
boolean led             = false;

String  lcdText         = " ";

byte counter            = 0;

float sensitivityX = 0.3;
float sensitivityY = 0.2;

float thresholdX = 0.7;
float thresholdY = 1.3;

float maxMovementThresholdX = 50;
float maxMovementThresholdY = 50;

int left_button_last_state = 0;
int left_button_actual_state = 0;
int right_button_last_state = 0;
int right_button_actual_state = 0;

BleMouse bleMouse;

void writeText(String _Text, int _ColorText) {
  M5.Lcd.setTextColor(BLACK);
  M5.Lcd.setCursor(0, 80);
  M5.Lcd.println(lcdText);
  M5.Lcd.setTextColor(_ColorText);
  M5.Lcd.setCursor(0, 80);
  M5.Lcd.println(_Text);
  lcdText = _Text;
}

void setup() {
  pinMode(M5_LED, OUTPUT);
  digitalWrite(M5_LED, HIGH);
  bleMouse.begin();
  M5.begin();
  M5.Lcd.setRotation(2);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);
  writeText("3D Mouse", WHITE);

  pinMode(LEFT_BUTTON, INPUT);
  pinMode(RIGHT_BUTTON, INPUT);

  M5.MPU6886.Init();
}

void updateBuffersAndCalculateAverages(float accX, float accY, float accZ) {

    accBufferX[accIndex] = accX;
    accBufferY[accIndex] = accY;
    accBufferZ[accIndex] = accZ;

    avgX = 0;
    avgY = 0;
    avgZ = 0;
    for (int i = 0; i < ACC_BUFFER_SIZE; i++) {
        avgX += accBufferX[i];
        avgY += accBufferY[i];
        avgZ += accBufferZ[i];
    }
    avgX /= ACC_BUFFER_SIZE;
    avgY /= ACC_BUFFER_SIZE;
    avgZ /= ACC_BUFFER_SIZE;

    float diffX = abs(accX - avgX);
    float diffY = abs(accY - avgY);
    float diffZ = abs(accZ - avgZ);

    diffBufferX[diffIndex] = diffX;
    diffBufferY[diffIndex] = diffY;
    diffBufferZ[diffIndex] = diffZ;

    avgDiffX = 0;
    avgDiffY = 0;
    avgDiffZ = 0;
    for (int i = 0; i < DIFF_BUFFER_SIZE; i++) {
        avgDiffX += diffBufferX[i];
        avgDiffY += diffBufferY[i];
        avgDiffZ += diffBufferZ[i];
    }
    avgDiffX /= DIFF_BUFFER_SIZE;
    avgDiffY /= DIFF_BUFFER_SIZE;
    avgDiffZ /= DIFF_BUFFER_SIZE;

    accIndex = (accIndex + 1) % ACC_BUFFER_SIZE;
    diffIndex = (diffIndex + 1) % DIFF_BUFFER_SIZE;
}

float calculateLastThreeAvg(float buffer[], int currentIndex, int bufferSize) {
    float sum = 0;

    for (int offset = 0; offset < 3; offset++) {
        int index = (currentIndex - 1 - offset + bufferSize) % bufferSize;
        sum += buffer[index];
    }
    return sum / 3; 
}


void loop() {
  if (digitalRead(M5_BUTTON_HOME) == LOW) {
    if (!isActivated) {
      isActivated = true;
      writeText("3D Mouse", WHITE);
    } else {
      isActivated = false;
      writeText("3D Mouse", RED);
    }
    while (digitalRead(M5_BUTTON_HOME) == LOW) {
    }
  }

  if (bleMouse.isConnected() && isActivated) {

    if (M5.BtnB.wasReleased()) {
      bleMouse.press(MOUSE_MIDDLE);
      bleMouse.release(MOUSE_MIDDLE);
    }

    left_button_actual_state = digitalRead(LEFT_BUTTON);
    right_button_actual_state = digitalRead(RIGHT_BUTTON);

    if (left_button_actual_state != left_button_last_state && left_button_actual_state == HIGH) {
      left_button_last_state = left_button_actual_state;
      bleMouse.press(MOUSE_LEFT);
      bleMouse.release(MOUSE_LEFT);
    }

    if (left_button_actual_state != left_button_last_state && left_button_actual_state == LOW) {
      left_button_last_state = left_button_actual_state;
      bleMouse.release(MOUSE_LEFT);
    }

    if (right_button_actual_state != right_button_last_state && right_button_actual_state == HIGH) {
      right_button_last_state = right_button_actual_state;
      bleMouse.press(MOUSE_RIGHT);
      bleMouse.release(MOUSE_RIGHT);
    }

    if (right_button_actual_state != right_button_last_state && right_button_actual_state == LOW) {
      right_button_last_state = right_button_actual_state;
      bleMouse.release(MOUSE_RIGHT);
    }
    
    M5.MPU6886.getGyroData(&accX, &accY, &accZ);

    updateBuffersAndCalculateAverages(accX, accY, accZ);

    float latestAvgX = calculateLastThreeAvg(accBufferX, accIndex, ACC_BUFFER_SIZE);
    float latestAvgY = calculateLastThreeAvg(accBufferY, accIndex, ACC_BUFFER_SIZE);
    float latestAvgZ = calculateLastThreeAvg(accBufferZ, accIndex, ACC_BUFFER_SIZE);

    float moveX = 0, moveY = 0;
    
    if (avgDiffX > thresholdY && avgDiffX < maxMovementThresholdY) {
        moveY = latestAvgX * sensitivityY;
    }

    if (avgDiffZ > thresholdX && avgDiffZ < maxMovementThresholdX) {
        moveX = latestAvgZ * sensitivityX;
        if (abs(moveX) > 4) {
          moveY = 0;
        }
    }

    M5.Lcd.fillRect(15, 20, 40, 50, BLACK);
    M5.Lcd.setCursor(0, 20);
    M5.Lcd.print("X: ");
    M5.Lcd.println(moveX);
    M5.Lcd.print("Y: ");
    M5.Lcd.println(moveY);
    
    if (moveX != 0 || moveY != 0) {
      bleMouse.move(-int(moveX), -int(moveY));
    }
    
    delay(2);
  }
}

