import serial
import json
import cv2
import time
import numpy as np
import threading
import face_recognition
import os
import math

# Configuration
SERIAL_PORT = 'COM3'  # Change to your port
BAUD_RATE = 9600
TIMEOUT = 1  # Timeout in seconds

# Shared state
camera_state = threading.Event()
stop_event = threading.Event()
arduino_lock = threading.Lock()


def read_serial_data(ser):
    try:
        print(f"Connected to {ser.port} at {ser.baudrate} baud")

        while not stop_event.is_set():
            if ser.in_waiting > 0:  # Check if there's any data in the buffer
                try:
                    data = ser.readline().decode('utf-8').strip()  # Read a line of data
                    if data:
                        json_data = json.loads(data)  # Decode JSON data
                        print("Received JSON:", json_data)  # Print received JSON data

                        if json_data.get("camera") == "on":
                            camera_state.set()
                        else:
                            camera_state.clear()

                        json_string = json.dumps(json_data)  # Encode JSON data to string
                        with arduino_lock:
                            ser.write((json_string + "\n").encode('utf-8'))  # Send modified JSON string back to Arduino
                        print("Sent JSON:", json_string)
                except json.JSONDecodeError as e:
                    print(f"Error decoding JSON: {e}")
                except Exception as e:
                    print(f"Unexpected error: {e}")

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Interrupted by user")


def face_confidence(face_distance, face_match_threshold=0.6):
    range = 1.0 - face_match_threshold
    linear_val = (1.0 - face_distance) / (range * 2.0)

    if face_distance > face_match_threshold:
        return round(linear_val * 100, 2)
    else:
        value = (linear_val + ((1.0 - linear_val) * math.pow((linear_val - 0.5) * 2, 0.2))) * 100
        return round(value, 2)


class FaceRecognition:
    def __init__(self, current_dir):
        self.current_dir = current_dir
        self.known_face_encodings = []
        self.known_face_names = []
        self.encode_faces()

    def encode_faces(self):
        for image_file in os.listdir(self.current_dir):
            image_path = os.path.join(self.current_dir, image_file)
            try:
                print(f"Processing image at: {image_path}")
                face_image = face_recognition.load_image_file(image_path)
                encodings = face_recognition.face_encodings(face_image)

                if encodings:
                    face_encoding = encodings[0]
                    self.known_face_encodings.append(face_encoding)
                    name = os.path.splitext(image_file)[0]
                    self.known_face_names.append(name)
                else:
                    print(f"No faces found in image at {image_path}")

            except Exception as e:
                print(f"Error processing file {image_path}: {e}")
        print(self.known_face_names)

    def process_frame(self, frame):
        recognized = False
        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = 'Unknown'
            confidence = 'Unknown'
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)

            if matches[best_match_index]:
                confidence = face_confidence(face_distances[best_match_index])
                if confidence > 90:
                    name = self.known_face_names[best_match_index]
                    recognized = True

            face_names.append(f'{name} ({confidence}%)')

        for (top, right, bottom, left), name in zip(face_locations, face_names):
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), -1)
            cv2.putText(frame, name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255, 255), 1)

        return recognized, frame


def process_camera(fr, ser):
    video_capture = None

    while not stop_event.is_set():
        if camera_state.is_set():
            if video_capture is None or not video_capture.isOpened():
                video_capture = cv2.VideoCapture(0)
                if not video_capture.isOpened():
                    print('Video source not found')
                    stop_event.set()
                    break

            ret, frame = video_capture.read()
            if not ret:
                print("Failed to capture image")
                break

            recognized, annotated_frame = fr.process_frame(frame)
            cv2.imshow('Face Recognition', annotated_frame)

            if recognized:
                with arduino_lock:
                    ser.write("0".encode())
                print("Recognized person detected")
            else:
                face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
                gray = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

                if len(faces) > 0:
                    for (x, y, w, h) in faces:
                        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    with arduino_lock:
                        ser.write("0".encode())
                    print("Person detected")
                else:
                    with arduino_lock:
                        ser.write("1".encode())
                    print("No person detected")

            if cv2.waitKey(1) == ord('q'):
                stop_event.set()
                break
        else:
            if video_capture is not None and video_capture.isOpened():
                video_capture.release()
                cv2.destroyAllWindows()
                video_capture = None
            time.sleep(1)

    if video_capture is not None and video_capture.isOpened():
        video_capture.release()
    cv2.destroyAllWindows()


def main():
    current_dir = r'C:\Users\ewen2\Documents\GitHub\face_detection\faces'  # Change directory
    fr = FaceRecognition(current_dir)

    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

    serial_thread = threading.Thread(target=read_serial_data, args=(ser,))
    serial_thread.start()

    camera_thread = threading.Thread(target=process_camera, args=(fr, ser))
    camera_thread.start()

    serial_thread.join()
    camera_thread.join()


if __name__ == '__main__':
    main()