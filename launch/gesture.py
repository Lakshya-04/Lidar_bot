import cv2
import numpy as np
import mediapipe as mp
from collections import deque
import time

# Initialize MediaPipe Hands module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

# Initialize MediaPipe Face Detection module
mp_face_detection = mp.solutions.face_detection
face_detection = mp_face_detection.FaceDetection(min_detection_confidence=0.7)

# Deque to store previous gesture values for smoothing
gesture_history = deque(maxlen=10)
recent_frames = deque(maxlen=3)

# Function to count fingers
def count_fingers(hand_landmarks):
    fingers = []
    tips = [4, 8, 12, 16, 20]

    # Thumb
    if hand_landmarks.landmark[tips[0]].x < hand_landmarks.landmark[tips[0] - 1].x:
        fingers.append(1)
    else:
        fingers.append(0)

    # Other fingers
    for tip in tips[1:]:
        if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[tip - 2].y:
            fingers.append(1)
        else:
            fingers.append(0)

    return fingers.count(1)

# Initialize video capture
cap = cv2.VideoCapture(0)

# FPS counter variables
fps_start_time = time.time()
frame_count = 0

# Frame processing variables
frame_counter = 0

# Initialize `current_frame` to avoid undefined variable error
current_frame = None

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)

    recent_frames.append(frame)

    frame_counter += 1
    # Process every third frame
    if frame_counter % 3 == 0:
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame with MediaPipe Hands
        result_hands = hands.process(frame_rgb)
        # Process the frame with MediaPipe Face Detection
        result_faces = face_detection.process(frame_rgb)

        # Use the most recent frame for inference
        if recent_frames:
            current_frame = recent_frames[-1]
        else:
            current_frame = frame

        if result_hands.multi_hand_landmarks:
            for hand_landmarks in result_hands.multi_hand_landmarks:
                mp_draw.draw_landmarks(current_frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Count fingers
                num_fingers = count_fingers(hand_landmarks)

                # Append gesture to history for smoothing
                gesture_history.append(num_fingers)
                gesture = max(set(gesture_history), key=gesture_history.count)

                # Display the gesture
                font = cv2.FONT_HERSHEY_SIMPLEX
                if gesture == 0:
                    cv2.putText(current_frame, '0', (0, 50), font, 2, (0, 0, 255), 3, cv2.LINE_AA)
                elif gesture == 1:
                    cv2.putText(current_frame, '1', (0, 50), font, 2, (0, 0, 255), 3, cv2.LINE_AA)
                elif gesture == 2:
                    cv2.putText(current_frame, '2', (0, 50), font, 2, (0, 0, 255), 3, cv2.LINE_AA)
                elif gesture == 3:
                    cv2.putText(current_frame, '3', (0, 50), font, 2, (0, 0, 255), 3, cv2.LINE_AA)
                elif gesture == 4:
                    cv2.putText(current_frame, '4', (0, 50), font, 2, (0, 0, 255), 3, cv2.LINE_AA)
                elif gesture == 5:
                    cv2.putText(current_frame, '5', (0, 50), font, 2, (0, 0, 255), 3, cv2.LINE_AA)

        if result_faces.detections:
            for detection in result_faces.detections:
                mp_draw.draw_detection(current_frame, detection)

    # Calculate and display FPS
    frame_count += 1
    elapsed_time = time.time() - fps_start_time
    if elapsed_time > 1:
        fps = frame_count / elapsed_time
        frame_count = 0
        fps_start_time = time.time()
    else:
        fps = frame_count / elapsed_time

    # Display FPS on the frame
    if current_frame is not None:
        cv2.putText(current_frame, f'FPS: {fps:.2f}', (frame.shape[1] - 120, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Show the window with inference
        cv2.imshow('Frame', current_frame)

    # Break the loop on 'ESC' key press
    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
