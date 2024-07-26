import face_recognition
import os
import sys
import cv2
import numpy as np
import math

def face_confidence(face_distance, face_match_threshold=0.6):
    range = 1.0 - face_match_threshold  
    linear_val = (1.0 - face_distance) / (range * 2.0)
    
    if face_distance > face_match_threshold:
        return round(linear_val * 100, 2)
    else: 
        value = (linear_val + ((1.0 - linear_val) * math.pow((linear_val - 0.5) * 2, 0.2))) * 100
        return round(value, 2)
    
class FaceRecognition():
    face_locations = []
    face_encodings = []
    face_names = []
    known_face_encodings = []
    known_face_names = []
    process_current_frame = True
    
    def __init__(self, currentDir):
        self.currentDir = currentDir
        self.encode_faces()
    
    def encode_faces(self):
        for image_file in os.listdir(self.currentDir):
            image_path = os.path.join(self.currentDir, image_file)
            try:
                print(f"Processing image at: {image_path}")
                face_image = face_recognition.load_image_file(image_path)
                encodings = face_recognition.face_encodings(face_image)
                
                if len(encodings) > 0:
                    # Use the first encoding if available
                    face_encoding = encodings[0]
                    self.known_face_encodings.append(face_encoding)
                    # Remove file extension
                    name = os.path.splitext(image_file)[0]
                    self.known_face_names.append(name)
                else:
                    print(f"No faces found in image at {image_path}")
                    
            except Exception as e:
                print(f"Error processing file {image_path}: {e}")
        print(self.known_face_names)
        
    def process_frame(self, frame):
        recognized = False
        if self.process_current_frame:
            small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
            rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
            
            self.face_locations = face_recognition.face_locations(rgb_small_frame)
            self.face_encodings = face_recognition.face_encodings(rgb_small_frame, self.face_locations)
            
            self.face_names = []
            for face_encoding in self.face_encodings:
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
                    else:
                        recognized = False
                    
                self.face_names.append(f'{name} ({confidence}%)')
                
        self.process_current_frame = not self.process_current_frame
        
        for (top, right, bottom, left), name in zip(self.face_locations, self.face_names):
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4
            
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), -1)
            cv2.putText(frame, name, (left + 6, bottom - 6), cv2.FONT_HERSHEY_DUPLEX, 0.8, (255, 255 , 255), 1)
            
        return recognized, frame
        


if __name__ == '__main__':
    currentDir = r'C:\Users\Noah Lee\OneDrive\Documents\GitHub\face_detection\faces'

    fr = FaceRecognition(currentDir)
    URL = "http://192.168.1.121:81/stream"
    video_capture = cv2.VideoCapture(0)
    
    if not video_capture.isOpened():
        sys.exit('Video source not found')
    
    frame_counter = 0
    
    while True:
        ret, frame = video_capture.read()
        if not ret:
            break

        frame_counter += 1

        if frame_counter % 1 == 0:
            recognized, annotated_frame = fr.process_frame(frame)
            frame =  annotated_frame

        cv2.imshow('Face Recognition', frame)
        print(recognized)
        if cv2.waitKey(1) == ord('q'):
            break

    video_capture.release()
    cv2.destroyAllWindows()
