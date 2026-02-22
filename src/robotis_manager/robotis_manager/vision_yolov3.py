import sys
import os

import numpy as np
import cv2
import time


class Yolov3:
    def __init__(self):
        pass

    def __map(self, x, inMin, inMax, outMin, outMax):
        return (x - inMin) * (outMax - outMin) // (inMax - inMin) + outMin

    def load(self, weight_path, cfg, classes):
        self.classes = None
        with open(classes, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]
        self.colors = np.random.uniform(
            0, 255, size=(len(self.classes), 3))  # optional
        # self.net = cv2.dnn.readNet(weight_path, cfg)
        self.net = cv2.dnn.readNetFromDarknet(cfg, weight_path)
        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]

    def dettect(self, frame):
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        height, width, ch = frame.shape
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        class_ids = [np.argmax(detection[5:]) for out in outs for detection in out]
        confidences = [detection[np.argmax(detection[5:]) + 5] for out in outs for detection in out]
        boxes = [[int(center_x - w / 2), int(center_y - h / 2), int(detection[2] * width), int(detection[3] * height)]
                 for out in outs for detection in out
                 if detection[np.argmax(detection[5:]) + 5] > 0.5
                 for center_x, center_y, w, h in [((detection[0] * width), (detection[1] * height),
                                                   (detection[2] * width), (detection[3] * height))]]
        if len(boxes) > 0:
            indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)  # 0.4 changeable
            font = cv2.FONT_HERSHEY_PLAIN
            values = {}
            for i in indexes.flatten():
                label = str(self.classes[class_ids[i]])
                x, y, w, h = boxes[i]
                temp = {
                    "class": label,
                    "confidence": confidences[i],
                    "x": x,
                    "y": y,
                    "width": w,
                    "height": h,
                    "color": self.colors[class_ids[i]]
                }
                values.update(temp)
            return values
        else:
            return {}

    def draw(self, frame, detection):
        if detection is not None:
            color = detection["color"]
            cv2.rectangle(frame, (detection["x"], detection["y"]),
                          (detection["x"] + detection["width"], detection["y"] + detection["width"]), color, 2)
            # Plots one bounding box on image frame
            tl = round(0.002 * (frame.shape[0] + frame.shape[1]) / 2) + 1  # line/font thickness
            c1, c2 = (int(detection["x"]), int(detection["y"])), (int(detection["width"]), int(detection["height"]))

            tf = int(max(tl - 1, 1))  # font thickness
            t_size = cv2.getTextSize(detection["class"], 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3

            cv2.rectangle(frame, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(frame, detection["class"] + " " + str(int(detection["confidence"] * 100)) + "%",
                        (c1[0], c1[1] - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
            cv2.circle(frame, (
                int(detection["x"] + int(detection["width"] / 2)), int(detection["y"] + int(detection["height"] / 2))),
                       4, color, -1)
            cv2.putText(frame, str(int(detection["x"] + int(detection["width"] / 2))) + ", " + str(
                int(detection["y"] + int(detection["height"] / 2))), (
                            int(detection["x"] + int(detection["width"] / 2) + 10),
                            int(detection["y"] + int(detection["height"] / 2) + 10)), cv2.FONT_HERSHEY_PLAIN, tl / 2,
                        [255, 255, 255], thickness=tf, lineType=cv2.LINE_AA)
        return frame

    def _oldYoloAlgorithm(self, frame):
        blob = cv2.dnn.blobFromImage(frame, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        height, width, ch = frame.shape
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5:
                    # object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    # rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        value = {}
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)  # 0.4 changeable
        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                label = str(self.classes[class_ids[i]])
                x, y, w, h = boxes[i]
                temp = {
                    "class": label,
                    "confidence": confidences[i],
                    "x": x,
                    "y": y,
                    "width": w,
                    "height": h,
                    "color": self.colors[class_ids[i]]
                }
                value.update(temp)
                return value
