import cv2
import mediapipe as mp
import serial

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

cap = cv2.VideoCapture(0)
cap.set(3, 320)
cap.set(4, 240)

_, frame = cap.read()
rows, cols, _ = frame.shape
x_medium = int(cols / 2)
center = int(cols / 2)

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.flush()

with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while True:
        ret, frame = cap.read()

        # Convert the BGR image to RGB.
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False

        # To improve performance, optionally mark the image as not writeable to pass by reference.
        results = pose.process(image)

        # Draw the pose annotation on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        cv2.imshow('MediaPipe Pose', image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        if results.pose_landmarks is not None:
            x = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE].x * cols
            x_medium = int(x)

        cv2.line(frame, (x_medium, 0), (x_medium, 480), (0, 255, 0), 2)

        if x_medium < center - 25:
            ser.write(b'right\n')
            print("Servo bergerak ke kanan")
        elif x_medium > center + 25:
            ser.write(b'left\n')
            print("Servo bergerak ke kiri")
        else:
            ser.write(b'center\n')
            print("Servo berhenti")

cap.release()
cv2.destroyAllWindows()
