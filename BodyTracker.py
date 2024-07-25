import cv2
import mediapipe as mp
import serial
import time
import numpy as np
import json
import threading

def send_servo_command(servo, angle):
    try:
        data = {"servo": servo, "angle": angle}
        command = json.dumps(data) + '\n'
        arduino.write(command.encode())
        time.sleep(0.05)  # Delay to prevent buffer overflow
    except Exception as e:
        print(f"Error sending servo command: {e}")

def control_servos(center_x, center_y, ws, hs):
    try:
        # Invert X axis to align with expected servo movement
        servoX = np.interp(center_x, [0, ws], [180, 0])
        servoY = np.interp(center_y, [0, hs], [0, 180])
        servoX = max(0, min(180, servoX))
        servoY = max(0, min(180, servoY))
        send_servo_command("x", int(servoX))
        send_servo_command("y", int(servoY))
        print(f"Servo X: {int(servoX)} deg, Servo Y: {int(servoY)} deg")
    except Exception as e:
        print(f"Error controlling servos: {e}")

# Initialize MediaPipe Holistic model
mp_holistic = mp.solutions.holistic
holistic = mp_holistic.Holistic(static_image_mode=False)

def process_frame(img, ws, hs):
    try:
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = holistic.process(img_rgb)
        if results.pose_landmarks:
            pose_landmarks = results.pose_landmarks.landmark
            center_x = int(pose_landmarks[0].x * ws)
            center_y = int(pose_landmarks[0].y * hs)
            pos = [center_x, center_y]
            cv2.putText(img, "Target Detected", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
            return center_x, center_y
        else:
            cv2.putText(img, "No target detected", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
            return None, None
    except Exception as e:
        print(f"Error processing frame: {e}")
        return None, None

def video_capture_thread(cap, frame_queue):
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        if len(frame_queue) > 1:  # Keep only the latest frame to reduce lag
            frame_queue.pop(0)
        frame_queue.append(frame)

def main():
    try:
        ws, hs = 320, 240

        # Initialize webcam
        URL = "http://192.168.1.121:81/stream"
        cap = cv2.VideoCapture(URL)

        if not cap.isOpened():
            print("Error: Could not open video stream.")
            return

        # Initialize Arduino
        port = "COM5"
        try:
            global arduino
            arduino = serial.Serial(port=port, baudrate=9600, timeout=.1)
        except Exception as e:
            print(f"Error initializing serial port: {e}")
            return

        cap.set(3, ws)
        cap.set(4, hs)

        frame_queue = []

        # Start the video capture thread
        capture_thread = threading.Thread(target=video_capture_thread, args=(cap, frame_queue))
        capture_thread.start()

        while True:
            if frame_queue:
                img = frame_queue.pop(0)
                center_x, center_y = process_frame(img, ws, hs)
                if center_x is not None and center_y is not None:
                    control_servos(center_x, center_y, ws, hs)
                cv2.imshow("Body Detection", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
        capture_thread.join()  # Ensure the capture thread is cleaned up

    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)  # Exit the script with error status

if __name__ == "__main__":
    main()
