#! python3
from pi_ai import ObjectDetector
import cv2
import time

detector = ObjectDetector()
try:
    detector.load_model(use_TPU=True)
except Exception as e:
    print(e)
    detector.load_model()

cap = cv2.VideoCapture(0)

while True:
    # Grab a single frame of video
    ret, frame = cap.read()
    start = time.time()
    boxes, classes, scores = detector.detect(frame)
    end = time.time()
    # detector.label_tags(frame, tags)
    print("detect %d frame in %.2f ms" %
          (1, (end - start)*1000))
    image = detector.draw_labels(frame, boxes, classes, scores)
    cv2.imshow('Object detector', image)

    # Press any key to continue to next image, or press 'q' to quit
    if cv2.waitKey(2) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
