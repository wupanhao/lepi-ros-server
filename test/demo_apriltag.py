#! python2
from apriltag_detector import ApriltagDetector
import cv2
import time
detector = ApriltagDetector()
cap = cv2.VideoCapture(0)
while True:
    # Grab a single frame of video
    ret, frame = cap.read()
    start = time.time()
    detector.detect(frame)
    end = time.time()
    # detector.label_tags(frame, tags)
    print("detect %d frame in %.2f ms" %
            (1, (end - start)*1000))
    print(detector.detections)
    cv2.imshow("images", frame)
    c = cv2.waitKey(4)
    if c == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
