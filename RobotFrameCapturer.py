import time

from multiprocessing import Process, Value, Array, Lock

from config_file import is_debug

import cv2

if is_debug:
    pass
else:
    #import picamera
    #from picamera.array import PiRGBArray
    pass


class RobotFrameCapturer(object):
    def __init__(self, params):
        # Create a detector with the parameters
        ver = cv2.__version__.split('.')
        if int(ver[0]) < 3:
            self.detector = cv2.SimpleBlobDetector(params)
        else:
            self.detector = cv2.SimpleBlobDetector_create(params)

        # Store X, Y and size
        self.x_object = Value('d', 0)
        self.y_object = Value('d', 0)
        self.size_object = Value('d', 0)

        # Check if finished
        self.finished = Value('b', False)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_frame_capturer = Lock()

    def start(self, finished, params):
        p = Process(target=self.loop, args=(finished, params))
        p.start()

    def stop(self):
        pass

    def getPosition(self):
        self.lock_frame_capturer.acquire()
        x = self.x_object.value
        y = self.y_object.value
        size = self.size_object.value
        self.lock_frame_capturer.release()
        return x, y, size

    def loop(self, finished, params):
        cam = picamera.PiCamera()

        cam.resolution = (320, 240)
        # cam.resolution = (640, 480)
        cam.framerate = 32
        rawCapture = PiRGBArray(cam, size=(320, 240))
        # rawCapture = PiRGBArray(cam, size=(640, 480))

        # allow the camera to warmup
        time.sleep(0.1)

        for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            frame = img.array
            # TODO: Update X, Y, Size
            if frame is None:
                print("Es none tambien")
            # cv2.imshow('Captura', self.frame)
            print("He sacado foto")
            # self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            rawCapture.truncate(0)
            cv2.waitKey(1)

            if finished.value:
                break
