import cv2
import mediapipe as mp
import serial
from flask import Flask, render_template, Response

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Initialize video capture
cap = cv2.VideoCapture(1)
cap.set(3, 320)
cap.set(4, 240)

# Initialize variables
_, frame = cap.read()
rows, cols, _ = frame.shape

# Initialize serial communication with the servo motor
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

# Initialize Mediapipe pose detection
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    app = Flask(__name__)

    @app.route('/')
    def index():
        return render_template('index.html')

    def gen():
        x_medium = int(cols / 2)
        center = int(cols / 2)
        while True:
            ret, frame = cap.read()

            # Convert the BGR image to RGB.
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False

            # Process pose estimation
            results = pose.process(image)

            # Draw the pose annotation on the image.
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

            # Calculate x_medium based on nose landmark
            if results.pose_landmarks is not None:
                x = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].x * cols
                x_medium = int(x)

            # Draw a vertical line on the frame at x_medium
            cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 155), 2)

            # Control servo motor based on x_medium position
            if x_medium < center - 45:
                ser.write(b'right\n')
                print("Servo bergerak ke kanan")
            elif x_medium > center + 45:
                ser.write(b'left\n')
                print("Servo bergerak ke kiri")

            # Encode the frame as JPEG
            ret, jpeg = cv2.imencode('.jpg', frame)

            # Yield the frame as bytes
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

    @app.route('/video_feed')
    def video_feed():
        return Response(gen(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')

    if __name__ == '__main__':
        app.run(debug=False, host='0.0.0.0', port=5003)

# Release video capture and close windows
cap.release()
cv2.destroyAllWindows()
