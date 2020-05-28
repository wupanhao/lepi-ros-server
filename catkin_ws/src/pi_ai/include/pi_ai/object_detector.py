#!coding: utf-8

# Import packages
import os
import cv2
import numpy as np
import sys
import importlib.util
from camera_utils import putText3
from .labels_map import detector_map
from .load_runtime import load_tflite_model


class ObjectDetector:
    def __init__(self):
        self.min_conf_threshold = 0.5

    def load_model(self,use_TPU = False):
        MODEL_PATH = os.path.expanduser('~')+'/Lepi_Data/ros/object_detector/coco_ssd_mobilenet_v1'
        GRAPH_NAME = 'model.tflite'
        LABELMAP_NAME = 'labelmap.txt'

        # If using Edge TPU, assign filename for Edge TPU model
        if use_TPU:
            # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
            if (GRAPH_NAME == 'model.tflite'):
                GRAPH_NAME = 'edgetpu.tflite'


        # Path to .tflite file, which contains the model that is used for object detection
        PATH_TO_CKPT = os.path.join(MODEL_PATH,GRAPH_NAME)

        # Path to label map file
        PATH_TO_LABELS = os.path.join(MODEL_PATH,LABELMAP_NAME)

        # Load the label map
        with open(PATH_TO_LABELS, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]

        # Have to do a weird fix for label map if using the COCO "starter model" from
        # https://www.tensorflow.org/lite/models/object_detection/overview
        # First label is '???', which has to be removed.
        if self.labels[0] == '???':
            del(self.labels[0])
        self.interpreter = load_tflite_model(model_path=PATH_TO_CKPT)

        # Get model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
        
        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

    # Loop over every image and perform detection
    def detect(self,image):

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
        self.interpreter.set_tensor(self.input_details[0]['index'],input_data)
        self.interpreter.invoke()

        # Retrieve detection results
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0] # Bounding box coordinates of detected objects
        classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0] # Class index of detected objects
        scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0] # Confidence of detected objects
        #num = interpreter.get_tensor(output_details[3]['index'])[0]  # Total number of detected objects (inaccurate and not needed)
        return boxes,classes,scores
    def set_threshold(self,threshold):
        if threshold < 100:
            self.min_conf_threshold = threshold/100.0
    def draw_labels(self,image,boxes,classes,scores):
        imH, imW, _ = image.shape
        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > self.min_conf_threshold) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin,xmin,ymax,xmax = self.getRealBox(boxes[i],(imW,imH))
                
                cv2.rectangle(image, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                # Draw label
                object_name = self.labels[int(classes[i])] # Look up object name from "labels" array using class index
                if object_name in detector_map:
                    object_name = detector_map[object_name]
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                # labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                # label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                # cv2.rectangle(image, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                # cv2.putText(image, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                image = putText3(image, label, (xmin, ymin+5),(0, 0, 0)) # Draw label text
        return image
    def getRealBox(self,box,size=(480,360)):
        imW,imH = size
        ymin = int(max(1,(box[0] * imH)))
        xmin = int(max(1,(box[1] * imW)))
        ymax = int(min(imH,(box[2] * imH)))
        xmax = int(min(imW,(box[3] * imW)))
        return [ymin,xmin,ymax,xmax]
if __name__ == '__main__':
    import sys
    image = cv2.imread(sys.argv[1])
    detector = ObjectDetector()
    detector.load_model()
    boxes,classes,scores = detector.detect(image)
    image = detector.draw_labels(image,boxes,classes,scores)
    cv2.imshow('Object detector', image)

    # Press any key to continue to next image, or press 'q' to quit
    if cv2.waitKey(0) == ord('q'):
        pass

    # Clean up
    cv2.destroyAllWindows()
