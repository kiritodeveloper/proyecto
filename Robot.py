#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division  # ''

import math
import time  # import the time library for the sleep function
import sys

# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Array, Lock

import numpy as np
from scipy import linalg
from plot_robot import dibrobot
import matplotlib.pyplot as plt

is_debug = True

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
        self.P = 0.05

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

        # Set motors ports
        motor_port_left = self.BP.PORT_B  # TODO: Change to correct value
        motor_port_right = self.BP.PORT_C  # TODO: Change to correct value

        # Set motors speed
        speed_dps_left = math.degrees(w_motors[0])
        speed_dps_right = math.degrees(w_motors[1])

        self.BP.set_motor_dps(motor_port_left, speed_dps_left)
        self.BP.set_motor_dps(motor_port_right, speed_dps_right)

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
        v = self.v.value
        w = self.w.value
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

        # Odometry update process
        self.p = Process(target=self.updateOdometry, args=(self.x, self.y, self.th, self.finished))



        self.p.start()
        print("PID: ", self.p.pid)
        # we don't really need to pass the shared params x, y, th, finished,
        # because they are part of the class, so visible within updateOdometry in any case,
        # but it's just to show an example of process receiving params

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self, x_odo, y_odo, th_odo, finished):

        while not finished.value:
            # current processor time in a floating point value, in seconds
            t_ini = time.clock()

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
                    "th: " + str(th) + ", dt: " + str(d_t) + ", w: " + str(w) + ", x: " + str(x) + ", y: " + str(y)
                )
            else:
                pass

            # save LOG
            t_end = time.clock()
            time.sleep(self.P - (t_end - t_ini))

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
        if(angle < -math.pi): #To positive
            angle = angle + 2*math.pi
        elif(angle > math.pi):
            angle = angle - 2*math.pi
        return angle
