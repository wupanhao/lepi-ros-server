#! python3
from face_recognizer import UltraFaceInference
import cv2
import time

detector = UltraFaceInference()
cap = cv2.VideoCapture(0)
while True:
    # img_path = os.path.join(imgs_path, file_path)
    # img_ori = cv2.imread(img_path)
    ret, img_ori = cap.read()
    time_time = time.time()
    boxes, labels, probs = detector.detect(img_ori)
    img_ori = detector.drawBoxes(img_ori, boxes)
    print("inference time: {} s".format(round(time.time() - time_time, 4)))
    print(boxes)  # [left top right bottom]
    cv2.imshow("ultra_face_inference", img_ori)
    c = cv2.waitKey(2)
    if c == ord('q'):
        break
cv2.destroyAllWindows()
cap.release()
