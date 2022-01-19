import cv2
import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands


class HandDetector:
    def __init__(self) -> None:
        self.hands = mp_hands.Hands(
            # model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

    def detect(self, image):
        h, w, channel = image.shape
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)
        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        array = []
        if results.multi_hand_landmarks:
            for index, hand_landmarks in enumerate(results.multi_hand_landmarks):
                # print(hand_landmarks.landmark, len(hand_landmarks.landmark))
                points = [[p.x*w, p.y*h, p.z] for p in hand_landmarks.landmark]
                array.append({
                    "score": results.multi_handedness[index].classification[0].score,
                    "label": results.multi_handedness[index].classification[0].label,
                    "keypoints": points
                })
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
        return array, image


if __name__ == '__main__':
    hand = HandDetector()
