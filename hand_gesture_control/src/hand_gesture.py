import cv2
import mediapipe as mp
import rospy
from std_msgs.msg import String

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

# Initialize ROS node
rospy.init_node('hand_gesture_node')
pub = rospy.Publisher('gesture_topic', String, queue_size=10)

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        continue

    # Flip the frame horizontally for a later selfie-view display
    # and convert the BGR image to RGB.
    image = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)

    # Process the image and find hands
    results = hands.process(image)

    # Draw the hand annotations on the image.
    image.flags.writeable = True
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # Draw circles on landmarks 0, 4, 9, 12
            for id, lm in enumerate(hand_landmarks.landmark):
                if id in [0, 4, 9, 12]:
                    h, w, _ = image.shape
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    cv2.circle(image, (cx, cy), 10, (255, 0, 0), -1)

            # Get landmark positions
            landmarks = hand_landmarks.landmark
            y_9 = landmarks[9].y
            y_12 = landmarks[12].y
            x_4 = landmarks[4].x
            x_0 = landmarks[0].x
            x_20 = landmarks[20].x

            # Determine gestures
            if y_12 > y_9:
                pub.publish('S')
            elif y_12 < y_9:
                pub.publish('W')
            
            if x_4 > x_0:
                pub.publish('D')
            if x_20 < x_0:
                pub.publish('A')
    else:
        # Publish 'P' when no landmarks are detected
        pub.publish('P')

    # Show the image
    cv2.imshow('Hand Tracking', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
hands.close()



