
def main():
    currentDir = '/Users/joy/Desktop/face_detection/faces'  # Change directory
    fr = FaceRecognition(currentDir)

    video_capture = None

    serial_thread = threading.Thread(target=read_serial_data, args=(arduino,))
    serial_thread.start()
