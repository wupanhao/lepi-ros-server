import cv2
import time
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    # ret1, frame1 = cap1.read()
    if ret:
        cv2.imshow("cap0", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
