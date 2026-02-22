import sys
import os

import numpy as np
import cv2
import time


class Vision:
    def __init__(self, isUsingCam=False, addr=None):
        self.frame_count = 0
        self.filenames = None
        self.fourcc = None
        self.out = None
        # get address
        self.cap = None
        if isUsingCam:
            self.cap = cv2.VideoCapture(0)
        elif addr is not None:
            self.cap = cv2.VideoCapture(addr)
        # fps
        self._prev_time = 0
        self._new_time = 0

    def writeConfig(self, name):
        self.filenames = name
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')  # format video
        self.out = cv2.VideoWriter(self.filenames, self.fourcc, 15.0, (450, 337))  # filename, format, FPS, frame size

    def write(self, frame):
        self.out.write(frame)

    def imwrite(self, frame):
        cv2.imwrite("frame.png", frame)

    def resize(self, image, width=None, height=None,
               interpolation=cv2.INTER_AREA):
        dim = None
        w = image.shape[1]
        h = image.shape[0]
        if width is None and height is None:
            return image
        if width is None:
            r = height / float(h)
            dim = (int(w * r), height)
        else:
            r = width / float(w)
            dim = (width, int(h * r))
        resized = cv2.resize(image, dim, interpolation=interpolation)
        return resized

    def __get_fps(self):
        self._new_time = time.time()
        delta = self._new_time - self._prev_time
        self._prev_time = self._new_time
        fps = (1 / delta) if delta > 0 else 30
        fps = 30 if fps > 30 else fps
        return int(fps)

    def blur(self, frame=None, ksize=11):
        if ksize % 2 == 0:
            ksize += 1
        return cv2.GaussianBlur(frame, (ksize, ksize), 0)

    def setBrightness(self, frame, value):
        h, s, v = cv2.split(
            cv2.cvtColor(frame, cv2.COLOR_BGR2HSV))
        v = np.clip(v.astype(int) + value, 0, 255).astype(np.uint8)
        return cv2.cvtColor(
            cv2.merge((h, s, v)), cv2.COLOR_HSV2BGR)

    def setContrast(self, frame, value):
        alpha = float(131 * (value + 127)) / (127 * (131 - value))
        gamma = 127 * (1 - alpha)
        return cv2.addWeighted(
            frame, alpha, frame, 0, gamma)

    def setBrightnessNcontrast(self, frame, bright=0.0, contr=0.0, beta=0.0):
        return cv2.addWeighted(frame, 1 + float(contr)
                               / 100.0, frame, beta, float(bright))

    def read(self, frame_size=480, show_fps=False):
        try:
            success, frame = self.cap.read()
            if not success:
                return None
            if show_fps:
                try:
                    cv2.putText(frame, str(self.__get_fps()) + " fps", (20, 40), 0, 1,
                                [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)
                except RuntimeError as e:
                    print(e)
            frame = self.resize(frame, frame_size)
            return frame
        except Exception as e:
            print("[INFO] Failed to capture the Frame:", e)
            return None

    def show(self, frame, winName="frame", show_fps=False):
        if show_fps:
            try:  # put fps
                cv2.putText(frame, str(self.__get_fps()) + " fps", (20, 40), 0, 1,
                            [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)
            except RuntimeError as e:
                print(e)
        cv2.imshow(winName, frame)

    def wait(self, delay):
        cv2.waitKey(delay)
