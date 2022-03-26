#!coding: utf-8
import os
import cv2
from .load_runtime import load_tflite_model
# from tflite_runtime.interpreter import Interpreter
import numpy as np
import enum

Key_Esc = 27


class KeypointType(enum.IntEnum):
    """Pose kepoints."""
    NOSE = 0
    LEFT_EYE = 1
    RIGHT_EYE = 2
    LEFT_EAR = 3
    RIGHT_EAR = 4
    LEFT_SHOULDER = 5
    RIGHT_SHOULDER = 6
    LEFT_ELBOW = 7
    RIGHT_ELBOW = 8
    LEFT_WRIST = 9
    RIGHT_WRIST = 10
    LEFT_HIP = 11
    RIGHT_HIP = 12
    LEFT_KNEE = 13
    RIGHT_KNEE = 14
    LEFT_ANKLE = 15
    RIGHT_ANKLE = 16


EDGES = (
    # (KeypointType.NOSE, KeypointType.LEFT_EYE),
    # (KeypointType.NOSE, KeypointType.RIGHT_EYE),
    # (KeypointType.NOSE, KeypointType.LEFT_EAR),
    # (KeypointType.NOSE, KeypointType.RIGHT_EAR),
    # (KeypointType.LEFT_EAR, KeypointType.LEFT_EYE),
    # (KeypointType.RIGHT_EAR, KeypointType.RIGHT_EYE),
    # (KeypointType.LEFT_EYE, KeypointType.RIGHT_EYE),
    (KeypointType.LEFT_SHOULDER, KeypointType.RIGHT_SHOULDER),
    (KeypointType.LEFT_SHOULDER, KeypointType.LEFT_ELBOW),
    (KeypointType.LEFT_SHOULDER, KeypointType.LEFT_HIP),
    (KeypointType.RIGHT_SHOULDER, KeypointType.RIGHT_ELBOW),
    (KeypointType.RIGHT_SHOULDER, KeypointType.RIGHT_HIP),
    (KeypointType.LEFT_ELBOW, KeypointType.LEFT_WRIST),
    (KeypointType.RIGHT_ELBOW, KeypointType.RIGHT_WRIST),
    (KeypointType.LEFT_HIP, KeypointType.RIGHT_HIP),
    (KeypointType.LEFT_HIP, KeypointType.LEFT_KNEE),
    (KeypointType.RIGHT_HIP, KeypointType.RIGHT_KNEE),
    (KeypointType.LEFT_KNEE, KeypointType.LEFT_ANKLE),
    (KeypointType.RIGHT_KNEE, KeypointType.RIGHT_ANKLE),
)


class MoveNetPose:
    def __init__(self):
        self.min_conf_threshold = 0.5

    def load_model(self, use_TPU=False):
        MODEL_PATH = os.path.expanduser(
            '~')+'/Lepi_Data/ros/pose_estimator/movenet_singlepose_lightning'
        GRAPH_NAME = 'model.tflite'

        # If using Edge TPU, assign filename for Edge TPU model
        if use_TPU:
            # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
            if (GRAPH_NAME == 'model.tflite'):
                GRAPH_NAME = 'model_edgetpu.tflite'

        # Path to .tflite file, which contains the model that is used for object detection
        PATH_TO_CKPT = os.path.join(MODEL_PATH, GRAPH_NAME)
        # self.interpreter = Interpreter(model_path=PATH_TO_CKPT)
        self.interpreter = load_tflite_model(
           model_path=PATH_TO_CKPT, use_TPU=use_TPU)
        self.interpreter.allocate_tensors()

        # Get model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

    # Loop over every image and perform detection
    def detect(self, image):
        # Load image and resize to expected shape [1xHxWx3]
        # image = cv2.imread(image_path)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (self.width, self.height))
        input_data = np.expand_dims(image_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.floating_model:
            input_mean = 127.5
            input_std = 127.5
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()

        # Output is a [1, 1, 17, 3] numpy array.
        keypoints_with_scores = self.interpreter.get_tensor(
            self.output_details[0]['index'])
        points = keypoints_with_scores[0][0]
        scores = [i[2] for i in points]
        score = np.sum(scores)/17.0
        return points.tolist(), score

    def set_threshold(self, threshold):
        if threshold < 100:
            self.min_conf_threshold = threshold/100.0

    def draw_pose(self, image, points):
        point_size = 1
        point_color = (0, 0, 255)  # BGR
        line_color = (0, 255, 0)
        thickness = 4
        size = image.shape
        # print(size)
        xys = {}
        for i, point in enumerate(points):
            x, y, score = point
            if score < 0.2:
                continue
            kp_x = int(y*size[1])
            kp_y = int(x*size[0])
            xys[KeypointType(i)] = (kp_x, kp_y)
            cv2.circle(image, (kp_x, kp_y), point_size, point_color, thickness)
        for a, b in EDGES:
            if a not in xys or b not in xys:
                continue
            cv2.line(image, xys[a], xys[b], line_color, thickness)


if __name__ == '__main__':
    import time
    detector = MoveNetPose()
    detector.load_model(use_TPU=True)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    while True:
        ret, image = cap.read()
        start = time.time()
        points, score = detector.detect(image)
        if score > 0.2:
            detector.draw_pose(image, points)
        print((time.time()-start)*1000)
        cv2.imshow('Object detector', image)
        # cv2.imshow('Pose',np.rot90(
        #    cv2.resize(cv2.flip(image, 1), (320, 240))))
        # 按Esc退出
        if cv2.waitKey(1) == Key_Esc:
            break

    # Clean up
    cap.release()
    cv2.destroyAllWindows()
