import cv2
import numpy as np
import json
import serial
import time
import sys
from FaceRecognition2 import FaceRecognition

# Initialize serial connection
ser = serial.Serial(port='COM5', baudrate=9600, timeout=1)  # Adjust port as needed
time.sleep(2)  # Allow time for the serial connection to establish

def send_json(camera_on, servo_x, servo_y):
    try:
        data = {
            "camera_on": camera_on,
            "servo_x": servo_x,
            "servo_y": servo_y
        }
        json_data = json.dumps(data) + '\n'  # Ensure newline character for proper serial transmission
        ser.write(json_data.encode())
        print(f"Sent to Arduino: {json_data}")
        time.sleep(0.05)  # Delay to prevent buffer overflow
    except serial.SerialException as e:
        print(f"Error sending JSON to Arduino: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")

def map_servos(center_x, center_y, frame_width, frame_height):
    # Example mapping function, modify as needed
    servo_x = np.interp(center_x, [0, frame_width], [0, 180])
    servo_y = np.interp(center_y, [0, frame_height], [0, 180])
    return int(servo_x), int(servo_y)

# Initialize Face Recognition
currentDir = r'C:\Users\Noah Lee\OneDrive\Documents\GitHub\face_detection\faces'
# fr = FaceRecognition(currentDir)
URL = "http://192.168.1.121:81/stream"  # Change stream URL as needed
video_capture = cv2.VideoCapture(URL)

if not video_capture.isOpened():
    sys.exit('Video source not found')

face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Initialize FPS calculation
fps_start_time = time.time()
frame_count = 0

threshold_x = 30
threshold_y = 30
servo_pan_speed = 0.05
servo_tilt_speed = 0.05
pulse_pan_min = 0
pulse_pan_max = 180
pulse_tilt_min = 0
pulse_tilt_max = 180
servo_pos_x = 90  # Default starting positions
servo_pos_y = 90
dir_x = 1
dir_y = 1

while True:
    ret, frame = video_capture.read()
    if not ret:
        break

    # Update FPS calculation
    frame_count += 1
    elapsed_time = time.time() - fps_start_time
    if elapsed_time > 1.0:
        fps = frame_count / elapsed_time
        fps_start_time = time.time()
        frame_count = 0

    # Get frame dimensions
    frame_height, frame_width = frame.shape[:2]
    CENTER_X = frame_width // 2
    CENTER_Y = frame_height // 2

    # recognized, annotated_frame = fr.process_frame(frame)
    # frame = annotated_frame

    # Always use Haar cascade to get face coordinates
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

    largest_face_size = 0
    largest_face_bb = None

    for (x, y, w, h) in faces:
        face_size = w * h
        if face_size > largest_face_size:
            largest_face_size = face_size
            largest_face_bb = (x, y, w, h)

    if largest_face_bb is not None:
        x, y, w, h = largest_face_bb
        face_x = x + w // 2
        face_y = y + h // 2
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.line(frame, (CENTER_X, CENTER_Y), (face_x, face_y), (0, 255, 0), 2)

        diff_x = face_x - CENTER_X
        if abs(diff_x) <= threshold_x:
            diff_x = 0
        diff_y = face_y - CENTER_Y
        if abs(diff_y) <= threshold_y:
            diff_y = 0

        mov_x = dir_x * servo_pan_speed * diff_x
        mov_y = dir_y * servo_tilt_speed * diff_y

        servo_pos_x = servo_pos_x + mov_x
        servo_pos_y = servo_pos_y + mov_y

        servo_pos_x = max(pulse_pan_min, min(pulse_pan_max, servo_pos_x))
        servo_pos_y = max(pulse_tilt_min, min(pulse_tilt_max, servo_pos_y))

        print("Moving to X:", int(servo_pos_x), "Y:", int(servo_pos_y))
        send_json(True, int(servo_pos_x), int(servo_pos_y))

    else:
        print("No person detected")
        send_json(False, 90, 90)  # Default position when no face detected

    # Display FPS on the frame
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow('Face Recognition', frame)
    
    if cv2.waitKey(1) == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()
