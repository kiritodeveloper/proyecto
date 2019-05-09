#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import print_function  # use python 3 syntax but make it compatible with python 2
from __future__ import division  # ''

import collections
import math
import time  # import the time library for the sleep function
import sys
from multiprocessing import Process, Value, Lock
import numpy as np

from config_file import *

from RobotFrameCapturer import RobotFrameCapturer

from utils import delay_until

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
        self.x = Value('d', init_position[0])
        self.y = Value('d', init_position[1])
        self.th = Value('d', init_position[2])
        self.finished = Value('b', 1)  # boolean to show if odometry updates are finished
        self.lock_odometry = Lock()

        # Update period (shared constant values)
        self.odometry_update_period = 0.03
        self.speed_update_period = 0.03
        self.proximity_update_period = 0.03
        self.gyros_update_period = 0.03

        # Set robot physical parameters (shared constant values)
        self.wheel_radius = 0.028  # m
        self.axis_length = 0.112  # m

        # Actual speed (shared memory values)
        self.v_actual = Value('d', 0.0)
        self.w_actual = Value('d', 0.0)
        self.lock_actual_speed = Lock()

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

        # Gyro sensor offset and calibration
        self.gyro_1_offset = 2325
        self.gyro_2_offset = 2367

        self.gyro_1_correction_factor = 0.14 * 0.03
        self.gyro_2_correction_factor = 0.135 * 0.03

        self.gyro_1_offset_correction_factor = 0
        self.gyro_2_offset_correction_factor = 0

        # Sensors raw dara
        self.gyro_1_raw = Value('d', self.gyro_1_offset)
        self.gyro_2_raw = Value('d', self.gyro_2_offset)
        self.proximity_raw = Value('d', 255)

        # Sensors values history
        self.history_max_size = 5
        self.history_dg_left = collections.deque(5 * [0], self.history_max_size)
        self.history_dg_right = collections.deque(5 * [0], self.history_max_size)
        self.history_gyro_1 = collections.deque(5 * [self.gyro_1_offset], self.history_max_size)
        self.history_gyro_2 = collections.deque(5 * [self.gyro_2_offset], self.history_max_size)
        self.history_proximity = collections.deque(5 * [1000], self.history_max_size)

        # Basket state
        self.basket_state = 'up'

        # Enable sensors
        self.enable_gyro_sensors = Value('b', False)
        self.enable_proximity_sensor = Value('b', False)

        # Odometry reseted
        self.odometry_reseted = Value('b', False)

        # Previous values of encoders in rads
        self.r_prev_encoder_left = 0
        self.r_prev_encoder_right = 0

    def enableGyroSensors(self, value):
        with self.enable_gyro_sensors.get_lock():
            self.enable_gyro_sensors.value = value

    def enableProximitySensor(self, value):
        with self.enable_proximity_sensor.get_lock():
            self.enable_proximity_sensor.value = value

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
        # Save speeds
        self.lock_actual_speed.acquire()
        v = self.v_actual.value
        w = self.w_actual.value
        self.lock_actual_speed.release()
        return v, w

    def updateSpeed(self):
        # Encoder timer
        encoder_timer = 0

        # current processor time in a floating point value, in seconds
        t_next_period = time.time()

        while not self.finished.value:
            [grad_izq, grad_der] = [self.BP.get_motor_encoder(self.motor_port_left),
                                    self.BP.get_motor_encoder(self.motor_port_right)]

            self.history_dg_left.append(grad_izq)
            self.history_dg_right.append(grad_der)

            # Suaviza los valores con la media
            rad_izq = math.radians(sum(self.history_dg_left) / self.history_max_size)
            rad_der = math.radians(sum(self.history_dg_right) / self.history_max_size)

            last_timer = encoder_timer
            encoder_timer = time.time()

            dt = encoder_timer - last_timer

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

            with self.gyro_1_raw.get_lock():
                gyro_1 = self.gyro_1_raw.value

            with self.gyro_2_raw.get_lock():
                gyro_2 = self.gyro_2_raw.value

            if self.enable_gyro_sensors.value:
                # Only if it is turning on read gyro sensors
                # Sensor 1
                actual_value_gyro_1 = - (gyro_1 - self.gyro_1_offset) * self.gyro_1_correction_factor

                # Sensor 2
                actual_value_gyro_2 = - (gyro_2 - self.gyro_2_offset) * self.gyro_2_correction_factor
                w = (w + actual_value_gyro_1 + actual_value_gyro_2) / 3.0
            else:
                self.history_gyro_1.append(self.gyro_1_offset)
                self.history_gyro_2.append(self.gyro_2_offset)

            # Save speeds
            self.lock_actual_speed.acquire()
            self.v_actual.value = v
            self.w_actual.value = w
            self.lock_actual_speed.release()

            # Periodic task
            t_next_period += self.speed_update_period
            delay_until(t_next_period)

    def updateGyros(self):
        # current processor time in a floating point value, in seconds
        t_next_period = time.time()

        while not self.finished.value:
            if not self.enable_gyro_sensors.value:
                self.history_gyro_1.append(self.BP.get_sensor(self.BP.PORT_3)[0])
                self.history_gyro_2.append(self.BP.get_sensor(self.BP.PORT_4)[0])

                gyro_1, gyro_2 = sum(self.history_gyro_1) / self.history_max_size, sum(
                    self.history_gyro_2) / self.history_max_size

                with self.gyro_1_raw.get_lock():
                    self.gyro_1_raw.value = gyro_1

                with self.gyro_2_raw.get_lock():
                    self.gyro_2_raw.value = gyro_2
            else:
                with self.gyro_1_raw.get_lock():
                    self.gyro_1_raw.value = 0

                with self.gyro_2_raw.get_lock():
                    self.gyro_2_raw.value = 0

        # Periodic task
        t_next_period += self.gyros_update_period
        delay_until(t_next_period)

    def updateProximity(self):
        # current processor time in a floating point value, in seconds
        t_next_period = time.time()

        while not self.finished.value:
            if not self.enable_proximity_sensor.value:
                self.history_proximity.append(self.BP.get_sensor(self.motor_port_ultrasonic))
                proximity = sum(self.history_proximity) / self.history_max_size

                with self.proximity_raw.get_lock():
                    self.proximity_raw.value = proximity
            else:
                with self.proximity_raw.get_lock():
                    self.proximity_raw.value = 255
        # Periodic task
        t_next_period += self.proximity_update_period
        delay_until(t_next_period)

    def readSensors(self):
        """
        Read both gyroscopes and proximity sensor
        :return: gyroscope_1, gyroscope_2, proximity
        """
        with self.proximity_raw.get_lock():
            proximity = self.proximity_raw.value

        with self.gyro_1_raw.get_lock():
            gyro_1 = self.gyro_1_raw.value

        with self.gyro_2_raw.get_lock():
            gyro_2 = self.gyro_2_raw.value

        return gyro_1, gyro_2, proximity

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

        self.BP.offset_motor_encoder(self.motor_port_left,
                                     self.BP.get_motor_encoder(self.motor_port_left))  # reset encoder B
        self.BP.offset_motor_encoder(self.motor_port_right,
                                     self.BP.get_motor_encoder(self.motor_port_right))  # reset encoder C

        # Odometry update process
        self.p = Process(target=self.updateOdometry)
        self.p.start()

        # Speed update process
        self.p_s = Process(target=self.updateSpeed)
        self.p_s.start()

        # Gyro update process
        self.p_g = Process(target=self.updateGyros)
        self.p_g.start()

        # Proximity update process
        self.p_p = Process(target=self.updateProximity)
        self.p_p.start()

    def resetOdometry(self, x_new, y_new, th_new):
        x, y, th = self.readOdometry()

        if x_new is not None:
            x = x_new
        if y_new is not None:
            y = y_new
        if th_new is not None:
            th = th_new

        self.lock_odometry.acquire()
        self.x.value = x
        self.y.value = y
        self.th.value = th
        self.odometry_reseted.value = True
        self.lock_odometry.release()

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self):
        """
        Update odometry every period
        :param x_odo: value where x coordinate must be stored
        :param y_odo: value where y coordinate must be stored
        :param th_odo: value where angle must be stored
        :param finished: if finish is true, odometry must stop updating
        """

        # current processor time in a floating point value, in seconds
        t_next_period = time.time()

        while not self.finished.value:

            d_t = self.odometry_update_period

            [v, w] = self.readSpeed()

            x = self.x.value

            y = self.y.value

            th = self.th.value

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

            if not self.odometry_reseted.value:
                self.lock_odometry.acquire()
                self.x.value = x
                self.y.value = y
                self.th.value = self.normalizeAngle(th)
                self.lock_odometry.release()
            else:
                # In case odometry is reseted, jump this updating period
                self.odometry_reseted.value = False

            # Periodic task
            t_next_period += self.odometry_update_period
            delay_until(t_next_period)
        sys.stdout.write("Stopping odometry ... X=  %d, \
                Y=  %d, th=  %d \n" % (self.x.value, self.y.value, self.th.value))

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
        print(message)

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

    def detectObstacle(self, number_of_cells=1):
        if is_debug:
            variable = False
            return variable
        else:
            self.lock_odometry.acquire()
            sensor_value = self.proximity_raw.value
            self.lock_odometry.release()
            print("Distancia: ", sensor_value)
            if sensor_value < number_of_cells * self.min_distance_obstacle_detection:
                return True
            else:
                return False

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

        with self.enable_gyro_sensors.get_lock():
            gyro_enabled = self.enable_gyro_sensors.value

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
