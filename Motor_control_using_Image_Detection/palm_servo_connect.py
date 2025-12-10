
import cv2
import mediapipe as mp
import serial
import time

# Configure Arduino serial (change COM3 to your port, e.g., "COM5" or "/dev/ttyUSB0")
arduino = serial.Serial('COM4', 9600)
time.sleep(2)  # wait for serial connection

# Mediapipe setup
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

with mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7, max_num_hands=1) as hands:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Flip for mirror view
        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb_frame)

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                # Use wrist (0) and middle fingertip (12) for distance estimation
                wrist = hand_landmarks.landmark[0]
                fingertip = hand_landmarks.landmark[12]

                # Calculate distance in normalized coordinates
                distance = ((fingertip.x - wrist.x)**2 + (fingertip.y - wrist.y)**2)**0.5

                # Map distance (approx range 0.1â€“0.5) to servo angle 0â€“180
                angle = int(((distance - 0.1) / (0.5 - 0.1)) * 180)
                angle = max(0, min(180, angle))  # clamp

                cv2.putText(frame, f"Servo Angle: {angle}", (30, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Send to Arduino
                arduino.write(f"{angle}\n".encode())

                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        cv2.imshow("Palm Control", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

cap.release()
arduino.close()
cv2.destroyAllWindows()

