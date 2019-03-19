import time

from multiprocessing import Process, Value, Array, Lock

import numpy as np

from config_file import is_debug

import cv2

if not is_debug:
    import picamera
    from picamera.array import PiRGBArray


class RobotFrameCapturer(object):
    def __init__(self, minRange, maxRange):
        # Store X, Y and size
        self.x_object = Value('d', 0)
        self.y_object = Value('d', 0)
        self.size_object = Value('d', 0)

        # Check if finished
        self.finished = Value('b', False)

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_frame_capturer = Lock()

        # min and max range
        self.minRange = minRange
        self.maxRange = maxRange

    def getDetectorParamsConfiguration(self):
        # Setup default values for SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # These are just examples, tune your own if needed
        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200

        # Filter by Area
        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 10000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.1

        # Filter by Color
        params.filterByColor = False
        # not directly color, but intensity on the channel input
        # params.blobColor = 0
        params.filterByConvexity = False
        params.filterByInertia = False
        return params

    def getDetector(self):
        # Create a detector with the parameters
        ver = cv2.__version__.split('.')
        if int(ver[0]) < 3:
            return cv2.SimpleBlobDetector(self.getDetectorParamsConfiguration())
        else:
            return cv2.SimpleBlobDetector_create(self.getDetectorParamsConfiguration())

    def start(self):
        self.p = Process(target=self.loop_frame_capturer)
        self.p.start()

    def stop(self):
        self.finished.value = False
        self.p.close()

    def getPosition(self):
        self.lock_frame_capturer.acquire()
        x = self.x_object.value
        y = self.y_object.value
        size = self.size_object.value
        self.lock_frame_capturer.release()
        return x, y, size

    def obtainBlobPosition(self, detector, imgBGR, minRange, maxRange):
        """

        :param detector:
        :param imgBGR:
        :param minRange: in HSV
        :param maxRange: in HSV
        :return:
        """
        # Next lines copied from https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
        blurred = cv2.GaussianBlur(imgBGR, (11, 11), 0)
        #hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        hsv = blurred

        mask = cv2.inRange(hsv, minRange, maxRange)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        keypoints = detector.detect(255 - mask)

        # documentation of SimpleBlobDetector is not clear on what kp.size is exactly,
        # but it looks like the diameter of the blob.

        bestKP = None
        if len(keypoints) != 0:
            bestKP = keypoints[0]
            for actualKP in keypoints:
                if actualKP.size > bestKP.size:
                    bestKP = actualKP

        # Update X, Y and size
        x = 0
        y = 0
        size = 0

        bkpa = []

        if bestKP is not None:
            x = bestKP.pt[0]
            y = bestKP.pt[1]
            size = bestKP.size
            bkpa = [bestKP]

        # Show image for debug only
        im_with_keypoints = cv2.drawKeypoints(imgBGR, bkpa, np.array([]),
                                              (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        output = cv2.bitwise_and(imgBGR, imgBGR, mask=mask)
        cv2.imshow("images", np.hstack([im_with_keypoints, output]))

        # Take images every 100 ms
        cv2.waitKey(100)

        return x, y, size

    def loop_frame_capturer(self):
        detector = self.getDetector()

        print("Antes de nada")
        if is_debug:
            cap = cv2.VideoCapture(0)
            while True:
                # Capture frame-by-frame
                ret, imgBGR = cap.read()

                x, y, size = self.obtainBlobPosition(detector, imgBGR, self.minRange, self.maxRange)

                self.lock_frame_capturer.acquire()
                self.x_object.value = x
                self.y_object.value = y
                self.size_object.value = size
                self.lock_frame_capturer.release()
                print(size)

                if self.finished.value:
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

                x, y, size = self.obtainBlobPosition(detector, imgBGR, self.minRange, self.maxRange)

                self.lock_frame_capturer.acquire()
                self.x_object.value = x
                self.y_object.value = y
                self.size_object.value = size
                self.lock_frame_capturer.release()

                rawCapture.truncate(0)
                cv2.waitKey(1)

                if self.finished.value:
                    break
