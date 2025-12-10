# Hand Gesture Controlled LED using OpenCV, MediaPipe & Arduino

Control LEDs connected to an Arduino using hand gestures detected through OpenCV and MediaPipe.
Each finger is recognized individually, and the states are sent to the Arduino through Serial Communication.

# Features

Real-time hand tracking using MediaPipe Hands

Detects Thumb, Index, Middle, Ring, and Pinky states (Up or Down)

Sends 5-digit binary data to Arduino
Example:

[1, 0, 1, 0, 1] → Thumb Up, Index Down, Middle Up, Ring Down, Pinky Up


Arduino turns LEDs ON/OFF based on finger positions

Works with Left and Right hand automatically

# Tech Stack
Component	Used For
Python	Computer vision & gesture detection
OpenCV	Camera capture & image processing
MediaPipe	Hand landmarks detection
Arduino (UNO / Nano etc.)	LED control via Serial
PySerial	Communication with Arduino

# How It Works

OpenCV captures webcam frames

MediaPipe identifies 21 hand landmarks

Python determines which fingers are up or down

Sends bytes([thumb, index, middle, ring, pinky]) to Arduino

Arduino turns LEDs ON/OFF accordingly

## Finger Detection Logic
Thumb

Right hand → Thumb is up if:

thumb_tip.x < thumb_joint.x


Left hand → Opposite logic:

thumb_tip.x > thumb_joint.x

Other Fingers

A finger is up if:

tip.y < dip.y

## Python Code (main.py)
```

import cv2
import mediapipe as mp
import serial
import time

# Initialize serial communication with Arduino
arduino = serial.Serial(port='COM4', baudrate=9600, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

# Initialize Mediapipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_drawing = mp.solutions.drawing_utils

# Function to detect individual fingers (1 for up, 0 for down)
def detect_fingers(image, hand_landmarks, handedness):
    finger_tips = [8, 12, 16, 20]  # Index, Middle, Ring, Pinky
    thumb_tip = 4
    finger_states = [0, 0, 0, 0, 0]  # Thumb, Index, Middle, Ring, Pinky

    # Thumb detection changes for left/right hand
    if handedness == "Right":
        if hand_landmarks.landmark[thumb_tip].x < hand_landmarks.landmark[thumb_tip - 1].x:
            finger_states[0] = 1  
    else:  # LEFT HAND
        if hand_landmarks.landmark[thumb_tip].x > hand_landmarks.landmark[thumb_tip - 1].x:
            finger_states[0] = 1  

    # Other fingers work same for both hands
    for idx, tip in enumerate(finger_tips):
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            finger_states[idx + 1] = 1

    return finger_states

# Start capturing video
cap = cv2.VideoCapture(0)

while cap.isOpened():
    success, image = cap.read()
    if not success:
        break

    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
    results = hands.process(image)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if results.multi_hand_landmarks:
        for hand_landmarks, hand_handedness in zip(results.multi_hand_landmarks, results.multi_handedness):

            hand_label = hand_handedness.classification[0].label  # "Left" or "Right"

            mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            fingers_state = detect_fingers(image, hand_landmarks, hand_label)

            arduino.write(bytes(fingers_state))
            print(hand_label, "Hand →", fingers_state)


    cv2.imshow('Hand Tracking', image)
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
arduino.close()


```

## Arduino Code (arduino.ino)

```
byte fingers[5];

void setup() {
  Serial.begin(9600);
  for (int pin = 2; pin <= 6; pin++) {
    pinMode(pin, OUTPUT);
  }
}

void loop() {
  if (Serial.available() >= 5) {
    Serial.readBytes(fingers, 5);

    for (int i = 0; i < 5; i++) {
      digitalWrite(2 + i, fingers[i] == 1 ? HIGH : LOW);
    }
  }
}

```

## Demo
  Finger Gesture -> LED Output
  
  Thumb Up	LED1 ON

 Index + Middle	LED1 & LED2 ON
 
 All Fingers Up	All LEDs ON
 
 ### Installation
 ### Install Dependencies
```
pip install opencv-python mediapipe pyserial

```

Connect Arduino

Connect 5 LEDs to pins 2, 3, 4, 5, 6

Common ground shared with USB

## Run the Program
python main.py

## Future Improvements

Add gesture-controlled servo motor

Add gesture-controlled robot

Add multi-hand support (Left + Right simultaneously)

Add GUI for visualization

# If you like this project…

Give the repo a Star star on GitHub!
It encourages me to build more open-source projects like this.
