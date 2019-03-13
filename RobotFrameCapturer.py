import time

from multiprocessing import Process, Value, Array, Lock

from config_file import is_debug

import cv2

if not is_debug:
    import picamera
    from picamera.array import PiRGBArray


class RobotFrameCapturer(object):
    def __init__(self, params):
        # Store X, Y and size
        self.x_object = Value('d', 0)
        self.y_object = Value('d', 0)
        self.size_object = Value('d', 0)

        # Check if finished
        self.finished = Value('b', False)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_frame_capturer = Lock()

        # Create a detector with the parameters
        ver = cv2.__version__.split('.')
        if int(ver[0]) < 3:
            self.detector = cv2.SimpleBlobDetector(params)
        else:
            self.detector = cv2.SimpleBlobDetector_create(params)

    def start(self, finished, minRange, maxRange):
        p = Process(target=self.loop, args=(finished, minRange, maxRange))
        p.start()

    def stop(self):
        self.finished.value = False

    def getPosition(self):
        self.lock_frame_capturer.acquire()
        x = self.x_object.value
        y = self.y_object.value
        size = self.size_object.value
        self.lock_frame_capturer.release()
        return x, y, size

    def obtainBlobPosition(self, imgBGR, minRange, maxRange):
        mask = cv2.inRange(imgBGR, minRange, maxRange)

        keypoints = self.detector.detect(255 - mask)

        # documentation of SimpleBlobDetector is not clear on what kp.size is exactly,
        # but it looks like the diameter of the blob.

        bestKP = None
        print(len(keypoints))
        if len(keypoints) != 0:
            kp = keypoints[0]
            for kpAux in keypoints:
                if kpAux.size > kp.size:
                    kp = kpAux

        # Update X, Y and size
        x = 0
        y = 0
        size = 0

        if bestKP is not None:
            x = bestKP.pt[0]
            y = bestKP.pt[1]
            size = bestKP.size

        return x, y, size

    def loop(self, finished, minRange, maxRange):
        if is_debug:
            cap = cv2.VideoCapture(0)
            while True:
                # Capture frame-by-frame
                ret, imgBGR = cap.read()

                x, y, size = self.obtainBlobPosition(imgBGR, minRange, maxRange)

                self.lock_frame_capturer.acquire()
                self.x_object = x
                self.y_object = y
                self.size_object = size
                self.lock_frame_capturer.release()

                cv2.waitKey(1)

                if finished.value:
                    break

        else:
            cam = picamera.PiCamera()

            cam.resolution = (320, 240)
            # cam.resolution = (640, 480)
            cam.framerate = 32
            rawCapture = PiRGBArray(cam, size=(320, 240))
            # rawCapture = PiRGBArray(cam, size=(640, 480))

            # allow the camera to warmup
            time.sleep(0.1)

            for img in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # Capture frame
                imgBGR = img.array

                x, y, size = self.obtainBlobPosition(imgBGR, minRange, maxRange)

                self.lock_frame_capturer.acquire()
                self.x_object = x
                self.y_object = y
                self.size_object = size
                self.lock_frame_capturer.release()

                rawCapture.truncate(0)
                cv2.waitKey(1)

                if finished.value:
                    break
