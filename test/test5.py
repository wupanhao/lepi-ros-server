# -*- coding: utf-8 -*-

import io
from time import sleep
import picamera
import numpy as np
import cv2

with picamera.PiCamera() as camera:
    camera.resolution = (320, 240)
    sleep(1)

    stream = io.BytesIO()
    for foo in camera.capture_continuous(stream, format='jpeg', use_video_port=True):
        data = np.fromstring(stream.getvalue(), dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)

        cv2.imshow("img", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Truncate the stream to the current position (in case
        # prior iterations output a longer image)
        stream.truncate()
        stream.seek(0)
