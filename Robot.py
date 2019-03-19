#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division  # ''

import math
import time  # import the time library for the sleep function
import sys
from multiprocessing import Process, Value, Array, Lock
import numpy as np

from RobotFrameCapturer import RobotFrameCapturer
from config_file import *
from utils import delay_until

if not is_debug:
    import brickpi3  # import the BrickPi3 drivers
else:
    from FakeBlockPi import FakeBlockPi


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        # self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        # self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        # self.BP.offset_motor_encoder(self.BP.PORT_B,
        #    self.BP.get_motor_encoder(self.BP.PORT_B))
        # self.BP.offset_motor_encoder(self.BP.PORT_C,
        #    self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################
        # odometry shared memory values
        self.x = Value('d', init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])
        self.finished = Value('b', 1)  # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.015

        # Set robot physical parameters
        self.wheel_radius = 0.028  # m
        self.axis_length = 0.115  # m

        # Set initial speed
        self.v = Value('d', 0.0)
        self.w = Value('d', 0.0)

        if not is_debug:
            self.BP = brickpi3.BrickPi3()  # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        else:
            self.BP = FakeBlockPi()

        # Set motors ports
        self.motor_port_left = self.BP.PORT_C
        self.motor_port_right = self.BP.PORT_B

        # Encoder timer
        self.encoder_timer = 0

        # Previous values of encoders in rads
        self.r_prev_encoder_left = 0
        self.r_prev_encoder_right = 0


    def setSpeed(self, v, w):
        '''
        Set the speed of the robot
        :param v: lineal speed in m/s
        :param w: angular speed in rad/s
        '''

        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ..
        w_motors = np.array([[1 / self.wheel_radius, self.axis_length / (2 * self.wheel_radius)],
                             [1 / self.wheel_radius,
                              -self.axis_length / (2 * self.wheel_radius)]]).dot(np.array([v, w]))

        # Set motors speed
        speed_dps_left = math.degrees(w_motors[1])
        speed_dps_right = math.degrees(w_motors[0])

        self.BP.set_motor_dps(self.motor_port_left, speed_dps_left)
        self.BP.set_motor_dps(self.motor_port_right, speed_dps_right)

        self.lock_odometry.acquire()
        self.v.value = v
        self.w.value = w
        self.lock_odometry.release()

    def readSpeed(self):
        '''
        Read the robot speed
        :return: robot speed
        '''

        self.lock_odometry.acquire()
        if is_debug:
            v = self.v.value
            w = self.w.value
        else:
            [grad_izq, grad_der] = [self.BP.get_motor_encoder(self.motor_port_left),
                                    self.BP.get_motor_encoder(self.motor_port_right)]
            rad_izq = math.radians(grad_izq)
            rad_der = math.radians(grad_der)

            last_timer = self.encoder_timer
            self.encoder_timer = time.time()

            dt = self.encoder_timer - last_timer

            # Hacia delante es negativo, se cambia el signo
            w_izq = (rad_izq - self.r_prev_encoder_left) / dt
            w_der = (rad_der - self.r_prev_encoder_right) / dt

            self.r_prev_encoder_left = rad_izq
            self.r_prev_encoder_right = rad_der

            v_w = np.array([[self.wheel_radius / 2, self.wheel_radius / 2],
                            [self.wheel_radius / self.axis_length,
                             -self.wheel_radius / self.axis_length]]).dot(np.array([w_der, w_izq]))

            v = v_w[0]
            w = v_w[1]

        self.lock_odometry.release()

        return v, w

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        self.lock_odometry.acquire()
        x = self.x.value
        y = self.y.value
        th = self.th.value
        self.lock_odometry.release()

        return x, y, th

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False

        if not is_debug:
            self.BP.offset_motor_encoder(self.motor_port_left,
                                         self.BP.get_motor_encoder(self.motor_port_left))  # reset encoder B
            self.BP.offset_motor_encoder(self.motor_port_right,
                                         self.BP.get_motor_encoder(self.motor_port_right))  # reset encoder C

        self.encoder_timer = time.time()

        # Odometry update process
        self.p = Process(target=self.updateOdometry, args=(self.x, self.y, self.th, self.finished))

        self.p.start()
        print("PID: ", self.p.pid)
        # we don't really need to pass the shared params x, y, th, finished,
        # because they are part of the class, so visible within updateOdometry in any case,
        # but it's just to show an example of process receiving params

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self, x_odo, y_odo, th_odo, finished):

        # current processor time in a floating point value, in seconds
        t_next_period = time.time()

        while not finished.value:

            d_t = self.P

            [v, w] = self.readSpeed()

            x = x_odo.value

            y = y_odo.value

            th = th_odo.value

            if w == 0:
                # Straight movement
                x = x + d_t * v * math.cos(th)
                y = y + d_t * v * math.sin(th)
            else:
                # Curved movement
                x = x + (v / w) * (math.sin(th + w * d_t) - math.sin(th))
                y = y - (v / w) * (math.cos(th + w * d_t) - math.cos(th))

            th = th + d_t * w

            # update odometry
            self.lock_odometry.acquire()
            x_odo.value = x
            y_odo.value = y
            th_odo.value = self.normalizeAngle(th)

            self.lock_odometry.release()

            if not is_debug:
                self.logWrite(
                    "th: " + str(th) + ", v: " + str(v) + ", w: " + str(w) + ", x: " + str(x) + ", y: " + str(y)
                )
            else:
                pass

            # Periodic task
            t_next_period += self.P
            delay_until(t_next_period)

        sys.stdout.write("Stopping odometry ... X=  %d, \
                Y=  %d, th=  %d \n" % (x_odo.value, y_odo.value, th_odo.value))

    # Stop the odometry thread.
    def stopOdometry(self):
        self.finished.value = True
        self.BP.reset_all()

    # Write message in the log
    def logWrite(self, message):
        print(message)

    # Normalize angle between -pi and pi
    def normalizeAngle(self, angle):
        if angle < -math.pi:  # To positive
            angle = angle + 2 * math.pi
        elif angle > math.pi:
            angle = angle - 2 * math.pi
        return angle

    # ------------------- TRACKING -------------------
    def get_w(self, x):  # TODO OJO QUE LA X NO TIENE POR QUÉ SER EL CENTRO DE LA IMAGEN
        """
        Return the w speed
        :param x: horizontal position in the picture
        :return: w speed
        """
        """
        far_position = 200
        medium_position = 100
        near_position = 50

        far_w = 10
        medium_w = 5
        near_w = 2

        abs_value = abs(x)
        if (abs_value > far_position):
            w = np.sign(x) * far_w
        elif (medium_position < abs_value and abs_value <= far_position):
            w = np.sign(x) * medium_w
        else:
            w = np.sign(x) * near_w
        return w
        """
        if( x > 160):
            w = -0.8
        else:
            w = 0.8
        return w

    def get_v(self, A, targetSize):
        """
        Return the v speed
        :param A: area of the blob
        :param targetSize: area expected of the blob
        :return: v speed
        """

        # Area's difference threshold
        far_position = 200
        medium_position = 100
        near_position = 50

        far_v = 10
        medium_v = 5
        near_v = 2

        abs_value = abs(A - targetSize)
        if (abs_value > far_position):
            v = far_v
        elif (medium_position < abs_value and abs_value <= far_position):
            v = medium_v
        else:
            v = near_v

        return v

    def getRecognisedBlobOrientation(self, trackedObject):
        """
        Return recognised blob orientation based on actual position in range [pi, -pi]
        """
        # TODO:
        return - math.pi

    def getRecognisedBlobSize(self, trackedObject):
        """
        Return recognised blob size/ real size, return range [0, 1]. If 0 not blob is recognised, if 1 blob is in correct position to catch
        """
        # TODO:
        return 0.5

    def searchForPromisingBlob(self, frame_capturer, colorRangeMin, colorRangeMax):
        """
        Search promising blob and return an identification of it, None if not detected
        :param colorRangeMin:
        :param colorRangeMax:
        :return:
        """
        return frame_capturer.getPosition()

    def trackObject(self, colorRangeMin=[0, 0, 0], colorRangeMax=[255, 255, 255]):
        # Start the process who update the vision values
        frame_capturer = RobotFrameCapturer(colorRangeMin, colorRangeMax)
        print(colorRangeMin)
        frame_capturer.start()

        finished = False
        targetFound = False
        targetPositionReached = False

        trackObjectPeriod = 0.2

        recognition_w = 0.8  # TODO: Change

        recognition_v = 0  # TODO: Change

        recognition_sample_period = 0.2  # TODO: Change

        while not finished:
            print("No he acabado y busco cosas")
            x, y, size = frame_capturer.getPosition()
            print(x, y, size)

            # 1. search the most promising blob ..
            # Find promising blob
            self.setSpeed(recognition_v, recognition_w)
            while size == 0:
                # While not promising blob found
                print("Estoy buscando la pelota mientras giro")
                time.sleep(recognition_sample_period)
                x, y, size = frame_capturer.getPosition()
                print(x, y, size)

            # When promising blob is found, stop robot
            self.setSpeed(0, 0)
            print("Ya no es none")

            while not targetPositionReached:
                x, y, size = frame_capturer.getPosition()
                if(size == 0):
                    break
                print(x, y, size)

                next_w = self.get_w(x)

                """
                if abs(recognised_orientation) < 0.2:  # TODO: Change
                    next_w *= 0.1  # TODO: Change
                else:
                    next_w *= 0.4  # TODO: Change

                if recognised_orientation < 0:
                    next_w = -next_w

                # decide v
                if recognised_size == 0:
                    # Not blob recognised
                    next_w = recognition_w
                    next_v = 0
                elif recognised_size < 0.5:  # TODO: Change
                    next_v *= 0.1  # TODO: Change
                elif recognised_size < 0.9:  # TODO: Change
                    next_v *= 0.04  # TODO: Change
                else:
                    targetPositionReached = True
                    finished = True
                    next_v = 0
                    next_w = 0

                
                time.sleep(trackObjectPeriod)
                """
                self.setSpeed(0.1, next_w)
                print("ME PILLE EN EL BUCLE")

        frame_capturer.stop()
        return finished
