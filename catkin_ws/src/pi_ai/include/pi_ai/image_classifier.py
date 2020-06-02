#!coding: utf-8

# Import packages
import os
import cv2
import numpy as np
import sys
import importlib.util
from camera_utils import putText3
# from labels_map import detector_map
from .load_runtime import load_tflite_model

class ImageClassifier:
    def __init__(self):
        self.min_conf_threshold = 0.15

    def load_model(self,use_TPU = False):
        modal_name = 'mobilenet_v1_1.0_224_quant_and_labels'
        # modal_name = 'test_detector'
        MODEL_PATH = os.path.expanduser('~')+'/Lepi_Data/ros/image_classifier/'+modal_name
        default_model_name = 'model.tflite'
        GRAPH_NAME = default_model_name
        LABELMAP_NAME = 'labelmap.txt'

        # If using Edge TPU, assign filename for Edge TPU model
        if use_TPU:
            # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
            if (GRAPH_NAME == default_model_name):
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
        print('input_details: ',self.input_details)
        print('output_details: ',self.output_details)
        print('floating_model: ',self.floating_model)

    # Loop over every image and perform detection
    def detect(self,image,top_k=1):
        # Load image and resize to expected shape [1xHxWx3]
        # image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image, (self.width, self.height))
        input_data = np.expand_dims(image_resized, axis=0)
        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if self.floating_model:
            input_mean = 127.5
            input_std = 127.5
            input_data = (np.float32(input_data) - input_mean) / input_std
        # Perform the actual detection by running the model with the image as input
        self.interpreter.set_tensor(self.input_details[0]['index'],input_data)
        self.interpreter.invoke()
        output_details = self.output_details[0]
        output = np.squeeze(self.interpreter.get_tensor(output_details['index']))
        # If the model is quantized (uint8 data), then dequantize the results
        if output_details['dtype'] == np.uint8:
            scale, zero_point = output_details['quantization']
            output = scale * (output - zero_point)
        ordered = np.argpartition(-output, top_k)
        return [(self.labels[i], output[i]) for i in ordered[:top_k]]

    def draw_labels(self,image,class_,score):
        # Loop over all detections and draw detection box if confidence is above minimum threshold
        if ((score > self.min_conf_threshold) and (score <= 1.0)):
            # Get bounding box coordinates and draw box
            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
            # ymin,xmin,ymax,xmax = self.getRealBox(boxes[i],(imW,imH))
            # cv2.rectangle(image, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)
            # Draw label
            object_name = class_
            if object_name in detector_map:
                object_name = detector_map[object_name]
            label = '%s: %d%%' % (object_name, int(score*100)) # Example: 'person: 72%'
            # labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
            # label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            # cv2.rectangle(image, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
            # cv2.putText(image, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
            image = putText3(image, label, (25, 25),(0, 0, 0)) # Draw label text
        return image
    def set_threshold(self,threshold):
        if threshold < 100:
            self.min_conf_threshold = threshold/100.0
if __name__ == '__main__':
    import sys,time
    image = cv2.imread(sys.argv[1])
    detector = ImageClassifier()
    detector.load_model()
    start = time.time()
    class_,score = detector.detect(image)[0]
    end = time.time()
    print(class_,score,(end-start)*1000)
    image = detector.draw_labels(image,class_,score)
    cv2.imshow('Object detector', image)

    # Press any key to continue to next image, or press 'q' to quit
    if cv2.waitKey(0) == ord('q'):
        pass

    # Clean up
    cv2.destroyAllWindows()
