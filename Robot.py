#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division  # ''

import collections
import math
import time  # import the time library for the sleep function
import sys
from multiprocessing import Process, Lock
import numpy as np

from SharedValue import SharedValue
from config_file import *

from RobotFrameCapturer import RobotFrameCapturer

from TimeUtils import delay_until

# Only import original drivers if it isn't in debug mode
import brickpi3


class Robot:
    def __init__(self, init_position=[0.0, 0.0, 0.0]):
        """
        Initialize basic robot params. \

        Initialize Motors and Sensors according to the set up in your robot
        """
        ######################################
        # Shared variables                   #
        ######################################

        # Odometry (shared memory values)
        self.x = SharedValue('d', init_position[0])
        self.y = SharedValue('d', init_position[1])
        self.th = SharedValue('d', init_position[2])
        self.finished = SharedValue('b', 1)  # boolean to show if odometry updates are finished
        self.lock_odometry = Lock()

        # Update period (shared constant values)
        self.odometry_update_period = 0.03
        self.speed_update_period = 0.03
        self.proximity_update_period = 0.03
        self.gyros_update_period = 0.03

        # Set robot physical parameters (shared constant values)
        self.wheel_radius = 0.028  # m
        self.axis_length = 0.112  # m

        # Sensors values history size
        self.history_max_size = 5

        # Actual speed (shared memory values)
        self.v = SharedValue('d', 0.0)
        self.w = SharedValue('d', 0.0)
        self.lock_actual_speed = Lock()

        # Gyro sensor offset and calibration
        self.gyro_1_offset = 2325
        self.gyro_2_offset = 2367

        self.gyro_1_correction_factor = 0.14 * 0.03
        self.gyro_2_correction_factor = 0.135 * 0.03

        # Sensors raw dara
        self.gyro_1_raw = SharedValue('d', self.gyro_1_offset)
        self.gyro_2_raw = ('d', self.gyro_2_offset)
        self.proximity_raw = SharedValue('d', 255)

        # Enable sensors
        self.enable_gyro_sensors = SharedValue('b', False)
        self.enable_proximity_sensor = SharedValue('b', False)

        # Odometry reseted
        self.odometry_reseted = SharedValue('b', False)

        # Robot sensors configuration
        self.BP = brickpi3.BrickPi3()
        self.BP.reset_all()

        # Set motors ports
        self.motor_port_left = self.BP.PORT_C
        self.motor_port_right = self.BP.PORT_B
        self.motor_port_basket = self.BP.PORT_A

        # Sonar config
        self.motor_port_ultrasonic = self.BP.PORT_1
        self.BP.set_sensor_type(self.motor_port_ultrasonic, self.BP.SENSOR_TYPE.NXT_ULTRASONIC)
        self.min_distance_obstacle_detection = 30  # cm

        # Gyroscopic sensors
        self.BP.set_sensor_type(self.BP.PORT_3, self.BP.SENSOR_TYPE.CUSTOM, [(self.BP.SENSOR_CUSTOM.PIN1_ADC)])
        self.BP.set_sensor_type(self.BP.PORT_4, self.BP.SENSOR_TYPE.CUSTOM, [(self.BP.SENSOR_CUSTOM.PIN1_ADC)])

        # Basket state
        self.basket_state = 'up'

    ####################################################################################################################
    # Start stop robot
    ####################################################################################################################

    def startRobot(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.set(False)

        self.BP.offset_motor_encoder(self.motor_port_left,
                                     self.BP.get_motor_encoder(self.motor_port_left))  # reset encoder B
        self.BP.offset_motor_encoder(self.motor_port_right,
                                     self.BP.get_motor_encoder(self.motor_port_right))  # reset encoder C

        # Odometry update process
        self.process_odometry = Process(target=self.updateOdometry)
        self.process_odometry.start()

        # Speed update process
        self.process_update = Process(target=self.updateSpeed)
        self.process_update.start()

        # Gyro update process
        self.process_gyros = Process(target=self.updateGyros)
        self.process_gyros.start()

        # Proximity update process
        self.process_proximity = Process(target=self.updateProximity)
        self.process_proximity.start()

    def stopRobot(self):
        """
        Stop the odometry thread.
        """
        self.finished.set(True)
        self.BP.reset_all()

    ####################################################################################################################
    # Speed control
    ####################################################################################################################

    def setSpeed(self, v, w):
        """
        Set the speed of the robot
        :param v: lineal speed in m/s
        :param w: angular speed in rad/s
        """
        # compute the speed that should be set in each motor ..
        w_motors = np.array([[1 / self.wheel_radius, self.axis_length / (2 * self.wheel_radius)],
                             [1 / self.wheel_radius,
                              -self.axis_length / (2 * self.wheel_radius)]]).dot(np.array([v, w]))

        # Set motors speed
        speed_dps_left = math.degrees(w_motors[1])
        speed_dps_right = math.degrees(w_motors[0])

        self.BP.set_motor_dps(self.motor_port_left, speed_dps_left)
        self.BP.set_motor_dps(self.motor_port_right, speed_dps_right)

    def readSpeed(self):
        """
        Read the robot speed
        :return: robot speed
        """
        self.lock_actual_speed.acquire()
        v = self.v.get()
        w = self.w.get()
        self.lock_actual_speed.release()
        return v, w

    def updateSpeed(self):
        """
        Update the variables self.w and self.v
        :return:
        """
        # Wheels encoders speed history
        history_dg_left = collections.deque(5 * [0], self.history_max_size)
        history_dg_right = collections.deque(5 * [0], self.history_max_size)

        # Previous values of encoders in rads
        r_prev_encoder_left = 0
        r_prev_encoder_right = 0

        # Encoder timer
        encoder_timer = 0

        # current processor time in a floating point value, in seconds
        t_next_period = time.time()

        while not self.finished.get():
            [grad_izq, grad_der] = [self.BP.get_motor_encoder(self.motor_port_left),
                                    self.BP.get_motor_encoder(self.motor_port_right)]

            history_dg_left.append(grad_izq)
            history_dg_right.append(grad_der)

            # Suaviza los valores con la media
            rad_izq = math.radians(sum(history_dg_left) / self.history_max_size)
            rad_der = math.radians(sum(history_dg_right) / self.history_max_size)

            last_timer = encoder_timer
            encoder_timer = time.time()

            dt = encoder_timer - last_timer

            # Hacia delante es negativo, se cambia el signo
            w_izq = (rad_izq - r_prev_encoder_left) / dt
            w_der = (rad_der - r_prev_encoder_right) / dt

            r_prev_encoder_left = rad_izq
            r_prev_encoder_right = rad_der

            v_w = np.array([[self.wheel_radius / 2, self.wheel_radius / 2],
                            [self.wheel_radius / self.axis_length,
                             -self.wheel_radius / self.axis_length]]).dot(np.array([w_der, w_izq]))

            v = v_w[0]
            w = v_w[1]

            if self.enable_gyro_sensors.get():
                # Only if is enabled, read gyro sensors
                gyro_1 = self.gyro_1_raw.get()
                gyro_2 = self.gyro_1_raw.get()

                # Sensor 1
                actual_value_gyro_1 = - (gyro_1 - self.gyro_1_offset) * self.gyro_1_correction_factor

                # Sensor 2
                actual_value_gyro_2 = - (gyro_2 - self.gyro_2_offset) * self.gyro_2_correction_factor

                # Final w
                w = (w + actual_value_gyro_1 + actual_value_gyro_2) / 3.0

            # Save speeds
            self.lock_actual_speed.acquire()
            self.v.set(v)
            self.w.set(w)
            self.lock_actual_speed.release()

            # Periodic task
            t_next_period += self.speed_update_period
            delay_until(t_next_period)

    ####################################################################################################################
    # Odometry
    ####################################################################################################################

    def setOdometry(self, x_new, y_new, th_new):
        """
        Set new odometry values
        :param x_new:
        :param y_new:
        :param th_new:
        :return:
        """
        x, y, th = self.readOdometry()

        if x_new is not None:
            x = x_new
        if y_new is not None:
            y = y_new
        if th_new is not None:
            th = th_new

        self.lock_odometry.acquire()
        self.x.set(x)
        self.y.set(y)
        self.th.set(th)
        self.odometry_reseted.set(True)
        self.lock_odometry.release()

    def readOdometry(self):
        """
        Returns current value of odometry estimation
        :return:
        """
        self.lock_odometry.acquire()
        x = self.x.get()
        y = self.y.get()
        th = self.th.get()
        self.lock_odometry.release()

        return x, y, th

    def updateOdometry(self):
        """
        Update odometry every period
        """

        # current processor time in a floating point value, in seconds
        t_next_period = time.time()

        while not self.finished.get():

            d_t = self.odometry_update_period

            [v, w] = self.readSpeed()

            x = self.x.get()

            y = self.y.get()

            th = self.th.get()

            if w == 0:
                # Straight movement
                x = x + d_t * v * math.cos(th)
                y = y + d_t * v * math.sin(th)
            else:
                # Curved movement
                x = x + (v / w) * (math.sin(th + w * d_t) - math.sin(th))
                y = y - (v / w) * (math.cos(th + w * d_t) - math.cos(th))

            # Update th
            th = th + d_t * w

            # Update odometry
            self.lock_odometry.acquire()
            if not self.odometry_reseted.get():
                self.x.set(x)
                self.y.set(y)
                self.th.set(self.normalizeAngle(th))

            else:
                # In case odometry is reseted, jump this updating period
                self.odometry_reseted.set(False)
            self.lock_odometry.release()

            # Periodic task
            t_next_period += self.odometry_update_period
            delay_until(t_next_period)

    ####################################################################################################################
    # Sensors
    ####################################################################################################################
    ####################################################################################################################
    # Gyro sensor
    ####################################################################################################################

    def enableGyroSensors(self, value):
        """
        Enable/Disable gyro sensors
        :param value:
        :return:
        """
        self.enable_gyro_sensors.set(value)

    def updateGyros(self):
        """
        Update the gyro read values periodically
        :return:
        """
        # Gyro sensors history
        history_gyro_1 = collections.deque(5 * [self.gyro_1_offset], self.history_max_size)
        history_gyro_2 = collections.deque(5 * [self.gyro_2_offset], self.history_max_size)

        # current processor time in a floating point value, in seconds
        t_next_period = time.time()

        while not self.finished.get():
            if self.enable_gyro_sensors.get():
                history_gyro_1.append(self.BP.get_sensor(self.BP.PORT_3)[0])
                history_gyro_2.append(self.BP.get_sensor(self.BP.PORT_4)[0])

            else:
                history_gyro_1.append(self.gyro_1_offset)
                history_gyro_2.append(self.gyro_2_offset)

            gyro_1, gyro_2 = sum(history_gyro_1) / self.history_max_size, sum(
                history_gyro_2) / self.history_max_size

            self.gyro_1_raw.set(gyro_1)
            self.gyro_1_raw.set(gyro_1)

        # Periodic task
        t_next_period += self.gyros_update_period
        delay_until(t_next_period)

    ####################################################################################################################
    # Proximity sensor
    ####################################################################################################################

    def enableProximitySensor(self, value):
        """
        Enable/Disable proximity sensor
        :param value:
        :return:
        """
        self.enable_proximity_sensor.set(value)

    def updateProximity(self):
        """
        Update the proximity read values periodically
        :return:
        """
        # Proximity sensors history
        history_proximity = collections.deque(5 * [255], self.history_max_size)

        # current processor time in a floating point value, in seconds
        t_next_period = time.time()

        while not self.finished.get():
            if self.enable_proximity_sensor.get():
                history_proximity.append(self.BP.get_sensor(self.motor_port_ultrasonic))
            else:
                history_proximity.append(255)

            proximity = sum(history_proximity) / self.history_max_size
            self.proximity_raw.set(proximity)

        # Periodic task
        t_next_period += self.proximity_update_period
        delay_until(t_next_period)

    ####################################################################################################################
    # Tracking
    ####################################################################################################################
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
            v = 0.15
        elif size < 80:
            v = 0.10
        else:
            v = 0.08

        return v, w

    def trackObject(self, salida, colorRangeMin=[0, 0, 0], colorRangeMax=[255, 255, 255]):
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

        if salida == 'A':
            recognition_w = -0.8
        else:
            recognition_w = 0.8

        recognition_v = 0

        recognition_sample_period = 0.2

        last_x = -10

        while not finished:
            x, y, size = frame_capturer.getPosition()

            # 1. search the most promising blob ..
            # Find promising blob
            print("last_x valor", last_x)
            if last_x == -10:
                self.setSpeed(recognition_v, recognition_w)
            elif last_x < 160:
                if salida == "A":
                    self.setSpeed(recognition_v, -recognition_w)
                else:
                    self.setSpeed(recognition_v, recognition_w)
            else:
                if salida == "A":
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
                        followBallRecognised = False
                        finished = True
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

    ####################################################################################################################
    # Navigating
    ####################################################################################################################

    def detectObstacle(self):
        """
        Return true if detect obstacle in the next cell
        :return:
        """
        sensor_value = self.proximity_raw.get()

        print("Distancia: ", sensor_value)
        return sensor_value < self.min_distance_obstacle_detection

    def wait_for_position(self, x, y, position_error_margin, minimize_error=True):
        """
        Wait until the robot reaches the position
        :param x: x position to be reached
        :param y: y position to be reached
        :param position_error_margin: error allowed in the position
        :param minimize_error: if true, the loop will keep control until the last error is minor to actual error
        """
        [x_odo, y_odo, _] = self.readOdometry()

        t_next_period = time.time()

        # Repeat while error decrease
        last_error = math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2)
        actual_error = last_error

        if minimize_error:
            # Minimize error
            while position_error_margin < actual_error:
                last_error = actual_error
                while last_error >= actual_error:
                    [x_odo, y_odo, _] = self.readOdometry()
                    last_error = actual_error
                    actual_error = math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2)
                    t_next_period += self.odometry_update_period
                    delay_until(t_next_period)
        else:
            # Stop when be in error margin
            while position_error_margin < math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2):
                [x_odo, y_odo, _] = self.readOdometry()
                t_next_period += self.odometry_update_period
                delay_until(t_next_period)

        print("He llegado a : ", x_odo, y_odo, " y busco: ", x, y)

    def wait_for_th(self, th, th_error_margin):
        """
        Wait until the robot reaches the position
        :param th_error_margin: error allowed in the orientation
        """
        [_, _, th_odo] = self.readOdometry()

        t_next_period = time.time()

        # Repeat while error decrease
        last_error = abs(self.normalizeAngle(th - th_odo))
        actual_error = last_error
        while th_error_margin < actual_error:
            last_error = actual_error
            while last_error >= actual_error:
                [_, _, th_odo] = self.readOdometry()
                # print("Tengo th: ", th_odo, " y busco: ", th)
                last_error = actual_error
                actual_error = abs(self.normalizeAngle(th - th_odo))
                t_next_period += self.odometry_update_period
                delay_until(t_next_period)
        # print("He llegado a : ", th_odo, " y busco: ", th)

    def orientate(self, aligned_angle):
        [_, _, th_actual] = self.readOdometry()

        # Turn
        turn_speed = math.pi / 6

        if aligned_angle > 5 * math.pi / 6 and th_actual < -math.pi / 4:
            turn_speed = -turn_speed
            aligned_angle = -aligned_angle
        elif aligned_angle < -5 * math.pi / 6 and th_actual > math.pi / 4:
            aligned_angle = -aligned_angle
        elif th_actual < -5 * math.pi / 6 and aligned_angle > math.pi / 4:
            turn_speed = -turn_speed
        elif aligned_angle < th_actual and not (th_actual > 5 * math.pi / 6 and aligned_angle < -math.pi / 4):
            turn_speed = -turn_speed

        print('Estoy buscando th ', aligned_angle)
        print('Velocidad ', turn_speed)
        self.setSpeed(0, turn_speed)
        self.wait_for_th(aligned_angle, 0.03)

        correction_speed = np.sign(turn_speed) * math.pi / 16

        self.setSpeed(0, -correction_speed)
        self.wait_for_th(aligned_angle, 0.02)
        print("Ha encontrado th")

        # Stop robot
        self.setSpeed(0, 0)

    def go(self, x_goal, y_goal):

        [x_actual, y_actual, th_actual] = self.readOdometry()

        print('Estoy en: ', x_actual, y_actual, th_actual)

        # Obtain positions
        final_x = x_goal
        final_y = y_goal
        aligned_angle = self.normalizeAngle(math.atan2(final_y - y_actual, final_x - x_actual))

        gyro_enabled = self.enable_gyro_sensors.get()

        # Turn
        self.enableGyroSensors(True)
        self.orientate(aligned_angle)
        self.enableGyroSensors(False)

        # Detect wall
        if self.detectObstacle():
            self.setSpeed(0, 0)
            self.enableGyroSensors(gyro_enabled)
            return False
        else:
            # Go forward
            self.setSpeed(0.15, 0)
            self.wait_for_position(final_x, final_y, 0.1)

            # Stop robot
            self.setSpeed(0, 0)
            self.enableGyroSensors(gyro_enabled)
            return True

    ####################################################################################################################
    # Utils
    ####################################################################################################################
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
