import cv2
import torch
from yolov5 import YoLOv5

# Initialize the YOLOv5 model
model = YOLOv5('yolov5s.pt', device='cpu')  # Use 'cuda' for GPU

# Define the video source: 0 for webcam, or provide the path to a video file
video_source = 0  # For webcam
# video_source = 's2.mp4'  # For a video file

# Open the video source
cap = cv2.VideoCapture(video_source)

# Check if the video source opened successfully
if not cap.isOpened():
    print("Error: Could not open video source.")
    exit()

# Define the class names for YOLOv5 (COCO dataset)
class_names = model.names

# Define the target classes (dogs and cats) based on COCO dataset indices
target_classes = ['dog', 'cat']

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Perform detection
    results = model(frame)

    # Extract detection results
    detections = results.pred[0]

    for *box, conf, cls in detections:
        class_name = class_names[int(cls)]
        if class_name in target_classes:
            label = f'{class_name} {conf:.2f}'
            x1, y1, x2, y2 = map(int, box)
            # Draw bounding box and label on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame with detections
    cv2.imshow('Pet Detection', frame)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close windows
cap.release()
cv2.destroyAllWindows()
