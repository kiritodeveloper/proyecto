from multiprocessing import Process

from plot_robot import dibrobot
import matplotlib.pyplot as plt

import time

import cv2
import picamera
from picamera.array import PiRGBArray


class RobotFrameStealer(object):
    def __init__(self):
        self.frame = None

    def start(self, finished):
        p = Process(target=self.loop, args=finished)
        p.start()
        time.sleep(25)

    def loop(self, finished):
        cam = picamera.PiCamera()

        cam.resolution = (320, 240)
        # cam.resolution = (640, 480)
        cam.framerate = 32
        rawCapture = PiRGBArray(cam, size=(320, 240))
        # rawCapture = PiRGBArray(cam, size=(640, 480))

        # allow the camera to warmup
        time.sleep(0.1)

        for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            self.frame = img.array
            cv2.imshow('Captura', self.frame)
            # self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            rawCapture.truncate(0)
            cv2.waitKey(1)

            if finished.value:
                break
