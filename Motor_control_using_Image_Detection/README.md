# Project 1. Remote-Motor-Control-with-Image-Detection using  Arduino , OpenCV, MediaPipe and Pyserial

## PART 1: Servomotor control using Image Detection 
Control a servo motor in real-time using hand gestures detected by your webcam! This project uses MediaPipe for hand tracking, OpenCV for video capture and visualization, and an Arduino to control the servo motor.

## Overview

This project demonstrates a simple hand gesture-controlled servo motor system. The distance between your wrist and middle fingertip is tracked via webcam, and this distance is mapped to a servo angle (0–180°). Moving your hand closer or farther changes the servo's rotation in real-time.

## Hardware Requirements

Arduino Uno (or compatible board)

Servo Motor (e.g., SG90)

USB cable for Arduino

Jumper wires

Breadboard (optional)

Webcam

## Software Requirements

Arduino IDE

Python 3.x

Python Libraries:

opencv-python

mediapipe

pyserial

## Install Python dependencies using:

```

pip install opencv-python mediapipe pyserial

```

## Setup Instructions

### Arduino

Connect the servo signal pin to D9, VCC to 5V, and GND to GND on Arduino.

Connect Arduino to your computer via USB.

## Upload the following Arduino sketch. NOTE:This code does not support I2C OLED Display. Use this if you Don't want Display

```
#include <Servo.h>

Servo myServo;
int angle = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(9); // Servo signal pin to D9
  pinMode(13, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    angle = Serial.parseInt();  // Read angle from Python
    if (angle >= 0 && angle <= 180) {
      digitalWrite(13, HIGH);
      myServo.write(angle);
    }
  }
}

```

## This code supports I2C OLED Display. Use this if you want Display
```
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

Servo myServo;
int angle = 0;

void setup() {
  Serial.begin(9600);

  // Servo
  myServo.attach(9);
  pinMode(13, OUTPUT);

  // OLED initialization
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("Ready...");
  display.display();
}

void loop() {

  if (Serial.available() > 0) {

    angle = Serial.parseInt();

    if (angle >= 0 && angle <= 180) {
      digitalWrite(13, HIGH);
      myServo.write(angle);
    }

    // ---- OLED DISPLAY UPDATE ----
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 20);

    display.print("Angle: ");
    display.print(angle);

    display.display();

  }
}


```
## Below is the python that for OpenCV for Media processing & Mediapipe for Machine learning Image Detection.
## Arduino serial library to connect opencv & MediaPipe with arduino  

```

Python

Change the serial port to match your Arduino connection (e.g., "COM4" on Windows or "/dev/ttyUSB0" on Linux).

Run the Python script:

import cv2
import mediapipe as mp
import serial
import time

arduino = serial.Serial('COM4', 9600)
time.sleep(2)

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

with mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7, max_num_hands=1) as hands:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb_frame)

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                wrist = hand_landmarks.landmark[0]
                fingertip = hand_landmarks.landmark[12]

                distance = ((fingertip.x - wrist.x)**2 + (fingertip.y - wrist.y)**2)**0.5
                angle = int(((distance - 0.1) / (0.5 - 0.1)) * 180)
                angle = max(0, min(180, angle))

                cv2.putText(frame, f"Servo Angle: {angle}", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                arduino.write(f"{angle}\n".encode())
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        cv2.imshow("Palm Control", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

cap.release()
arduino.close()
cv2.destroyAllWindows()

```
## How It Works

Hand Detection: MediaPipe detects your hand landmarks in real-time.

Distance Calculation: The distance between the wrist and middle fingertip is computed.

Angle Mapping: Distance is mapped to a servo angle (0–180°).

Servo Control: Python sends the angle via serial to Arduino, which moves the servo.

Usage

Place your hand in front of the webcam.

Move your hand closer or farther away to control the servo.

Watch the live angle display on the OpenCV window.

Press ESC to exit.
