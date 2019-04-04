#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division  # ''

import math
import time  # import the time library for the sleep function
import sys
from multiprocessing import Process, Value, Array, Lock
import numpy as np

from config_file import *

if not is_debug and not disable_open_cv:
    from RobotFrameCapturer import RobotFrameCapturer

from utils import delay_until

# Only import original drivers if it isn't in debug mode
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
        self.P = 0.02

        # Set robot physical parameters
        self.wheel_radius = 0.028  # m
        self.axis_length = 0.115  # m

        # Set initial speed
        self.v = Value('d', 0.0)
        self.w = Value('d', 0.0)

        if is_debug:
            self.BP = FakeBlockPi()

        else:
            self.BP = brickpi3.BrickPi3()  # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
            # Set motors ports
            self.motor_port_left = self.BP.PORT_C
            self.motor_port_right = self.BP.PORT_B
            self.motor_port_basket = self.BP.PORT_A

            # Sonar config
            self.motor_port_ultrasonic = self.BP.PORT_1
            self.BP.set_sensor_type(self.motor_port_ultrasonic, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
            self.min_distance_obstacle_detection = 30  # cm

        # Basket state
        self.basket_state = 'up'

        # Encoder timer
        self.encoder_timer = 0

        # Previous values of encoders in rads
        self.r_prev_encoder_left = 0
        self.r_prev_encoder_right = 0

    def setSpeed(self, v, w):
        """
        Set the speed of the robot
        :param v: lineal speed in m/s
        :param w: angular speed in rad/s
        """

        print("setting speed to %.2f %.2f" % (v, w))

        # compute the speed that should be set in each motor ..
        w_motors = np.array([[1 / self.wheel_radius, self.axis_length / (2 * self.wheel_radius)],
                             [1 / self.wheel_radius,
                              -self.axis_length / (2 * self.wheel_radius)]]).dot(np.array([v, w]))

        # Set motors speed
        if not is_debug:
            speed_dps_left = math.degrees(w_motors[1])
            speed_dps_right = math.degrees(w_motors[0])

            self.BP.set_motor_dps(self.motor_port_left, speed_dps_left)
            self.BP.set_motor_dps(self.motor_port_right, speed_dps_right)

        self.lock_odometry.acquire()
        self.v.value = v
        self.w.value = w
        self.lock_odometry.release()

    def readSpeed(self):
        """
        Read the robot speed
        :return: robot speed
        """

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
        """
        Update odometry every period
        :param x_odo: value where x coordinate must be stored
        :param y_odo: value where y coordinate must be stored
        :param th_odo: value where angle must be stored
        :param finished: if finish is true, odometry must stop updating
        """

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

    def stopOdometry(self):
        """
        Stop the odometry thread.
        """
        self.finished.value = True
        self.BP.reset_all()

    def logWrite(self, message):
        """
        Write message in the log (screen)
        :param message: message to write
        """
        # print(message)
        kk = 0

    def normalizeAngle(self, angle):
        """
        Normalize angle between -pi and pi
        :param angle: angle to normalize
        :return:
        """
        if angle < -math.pi:  # To positive
            angle = angle + 2 * math.pi
        elif angle > math.pi:
            angle = angle - 2 * math.pi
        return angle

    # ------------------- -------- -------------------
    # ------------------- TRACKING -------------------
    # ------------------- -------- -------------------
    def obtainTrackObjectSpeed(self, x, size):
        """
        Return the speed to track an object
        :param x:
        :param size:
        :return:
        """
        if x > 160:
            x = 320 - x
            w = -1
        else:
            w = 1

        # Split the screen in 7 parts, depending where it is the ball, the robot will turn with higher or lower speed
        if x < 60:
            w = 0.8 * w
        elif x < 100:
            w = 0.5 * w
        elif x < 140:
            w = 0.2 * w
        else:
            w = 0

        # Depending how far is the ball, the robot will follow it faster or slower
        if size < 40:
            v = 0.25
        elif size < 80:
            v = 0.15
        else:
            v = 0.08

        return v, w

    def trackObject(self, colorRangeMin=[0, 0, 0], colorRangeMax=[255, 255, 255]):
        """
        Track object in range (colorRangeMin, colorRangeMax)
        :param colorRangeMin:
        :param colorRangeMax:
        :return:
        """
        # Start the process who update the vision values
        frame_capturer = RobotFrameCapturer(colorRangeMin, colorRangeMax)
        frame_capturer.start()

        # Give camara time to wake up
        time.sleep(1)

        finished = False

        recognition_w = 0.8

        recognition_v = 0

        recognition_sample_period = 0.2

        last_x = 0

        while not finished:
            x, y, size = frame_capturer.getPosition()

            # 1. search the most promising blob ..
            # Find promising blob
            if last_x < 160:
                self.setSpeed(recognition_v, recognition_w)
            else:
                self.setSpeed(recognition_v, -recognition_w)

            while size == 0:
                # While not promising blob found
                time.sleep(recognition_sample_period)
                x, y, size = frame_capturer.getPosition()

            # When promising blob is found, stop robot
            self.setSpeed(0, 0)

            # Set ball not reached for now
            followBallRecognised = True

            while followBallRecognised:
                x, y, size = frame_capturer.getPosition()

                # Save last ball recognised position to guess by which side it slided
                if x > 10:
                    last_x = x

                # Only if ball is still in image, track it
                if size > 0:
                    next_v, next_w = self.obtainTrackObjectSpeed(x, size)

                    # If we think ball is in basket, catch it and rotate the robot to ensure the supposition
                    if size > 110:
                        self.setSpeed(0, 0)
                        self.catch('down')

                        self.setSpeed(0, 0.4)
                        time.sleep(5)

                        self.setSpeed(0, 0)
                        _, _, size = frame_capturer.getPosition()

                        # If ball is in the basket, finish, else lift the basket
                        if size > 70:
                            followBallRecognised = False
                            finished = True

                        else:
                            self.catch('up')
                            self.setSpeed(next_v, next_w)
                    else:
                        self.setSpeed(next_v, next_w)
                else:
                    followBallRecognised = False

        frame_capturer.stop()
        return finished

    def catch(self, movement):
        """
        Lift and low the basket
        :param movement:
        :return:
        """
        if movement != self.basket_state:
            if movement == 'up':
                self.BP.set_motor_dps(self.motor_port_basket, -85)
                time.sleep(1)
                self.BP.set_motor_dps(self.motor_port_basket, 0)
                self.basket_state = 'up'
            elif movement == 'down':
                self.BP.set_motor_dps(self.motor_port_basket, 85)
                time.sleep(1)
                self.BP.set_motor_dps(self.motor_port_basket, 0)
                self.basket_state = 'down'

    def detectObstacle(self):
        if is_debug:
            return False
        else:
            sensor_value = self.BP.get_sensor(self.motor_port_ultrasonic)
            print("Distancia: ", sensor_value)
            if sensor_value < self.min_distance_obstacle_detection:
                return True
            else:
                return False

    def go(self, x_goal, y_goal):
        def wait_for_position(x, y, robot, position_error_margin):
            """
            Wait until the robot reaches the position
            :param x: x position to be reached
            :param y: y position to be reached
            :param robot: robot configuration
            :param position_error_margin: error allowed in the position
            :param th_error_margin: error allowed in the orientation
            """
            [x_odo, y_odo, _] = robot.readOdometry()

            t_next_period = time.time()

            # Repeat while error decrease
            last_error = math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2)
            actual_error = last_error
            while position_error_margin < actual_error:
                last_error = actual_error
                while last_error >= actual_error:
                    [x_odo, y_odo, _] = robot.readOdometry()
                    last_error = actual_error
                    actual_error = math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2)
                    t_next_period += robot.P
                    delay_until(t_next_period)

        def wait_for_th(th, robot, th_error_margin):
            """
            Wait until the robot reaches the position
            :param robot: robot configuration
            :param th_error_margin: error allowed in the orientation
            """
            [_, _, th_odo] = robot.readOdometry()

            t_next_period = time.time()

            # Repeat while error decrease
            last_error = abs(self.normalizeAngle(th - th_odo))
            actual_error = last_error
            while th_error_margin < actual_error:
                last_error = actual_error
                while last_error >= actual_error:
                    [_, _, th_odo] = robot.readOdometry()
                    last_error = actual_error
                    actual_error = abs(self.normalizeAngle(th - th_odo))
                    t_next_period += robot.P
                    delay_until(t_next_period)

        [x_actual, y_actual, th_actual] = self.readOdometry()

        # Obtain positions
        final_x = x_goal
        final_y = y_goal
        aligned_angle = math.atan2(final_y - y_actual, final_x - x_actual)

        # Turn
        turn_speed = math.pi / 16

        if aligned_angle < th_actual:
            turn_speed = -turn_speed

        self.setSpeed(0, turn_speed)
        print('Estoy buscando th ', aligned_angle)
        wait_for_th(aligned_angle, self, 0.02)
        print("Ha encontrado th")

        # Stop robot
        self.setSpeed(0, 0)

        # Detect wall
        if self.detectObstacle():
            return False
        else:
            # Go forward
            self.setSpeed(0.15, 0)
            wait_for_position(final_x, final_y, self, 0.05)

            # Stop robot
            self.setSpeed(0, 0)

            return True
