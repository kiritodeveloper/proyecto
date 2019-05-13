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
        self.proximity = Value('d', 600)
        self.finished = Value('b', 1)  # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()

        # odometry update period
        self.P = 0.03

        # Set robot physical parameters
        self.wheel_radius = 0.028  # m
        self.axis_length = 0.112  # m

        # Set initial speed
        self.v = Value('d', 0.0)
        self.w = Value('d', 0.0)

        if is_debug:
            self.BP = FakeBlockPi()

            # Gyro sensor offset and calibration
            self.gyro_1_offset = 2430
            self.gyro_2_offset = 2430

        else:
            self.BP = brickpi3.BrickPi3()  # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
            # Set motors ports

            self.BP.reset_all()

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
            time.sleep(0.5)
            self.gyro_1_offset = 2325
            self.gyro_2_offset = 2367

        """
            for i in range(10):
                self.gyro_1_offset += self.BP.get_sensor(self.BP.PORT_3)[0]
                self.gyro_2_offset += self.BP.get_sensor(self.BP.PORT_4)[0]
                time.sleep(0.1)

            self.gyro_1_offset /= 10
            self.gyro_2_offset /= 10
        """

        # print("Gyro 1 offset ", self.gyro_1_offset)
        # print("Gyro 2 offset ", self.gyro_2_offset)

        self.gyro_1_correction_factor = 0.14
        self.gyro_2_correction_factor = 0.135

        self.gyro_1_offset_correction_factor = 0
        self.gyro_2_offset_correction_factor = 0

        # Sensors values history
        self.history_max_size = 5
        self.history_dg_left = collections.deque(5 * [0], self.history_max_size)
        self.history_dg_right = collections.deque(5 * [0], self.history_max_size)
        self.history_gyro_1 = collections.deque(5 * [self.gyro_1_offset], self.history_max_size)
        self.history_gyro_2 = collections.deque(5 * [self.gyro_2_offset], self.history_max_size)
        self.history_proximity = collections.deque(5 * [1000], self.history_max_size)

        # Color config
        self.color_sensor = self.BP.PORT_2
        self.BP.set_sensor_type(self.color_sensor, self.BP.SENSOR_TYPE.NXT_LIGHT_ON)

        # Basket state
        self.basket_state = 'up'

        # Is spinning
        self.is_spinning = Value('b', False)

        # Enable sensors
        self.enable_gyro_sensors = Value('b', False)
        self.enable_proximity_sensor = Value('b', False)

        # Is spinning
        self.odometry_reseted = Value('b', False)

        # Encoder timer
        self.encoder_timer = 0

        # Previous values of encoders in rads
        self.r_prev_encoder_left = 0
        self.r_prev_encoder_right = 0

        # Color vector
        self.color = 10 * [None]
        self.color_vector_pos = 0
        self.actual_color = -1

        self.color_limit = 2800



    def enableGyroSensors(self, value):
        self.lock_odometry.acquire()
        self.enable_gyro_sensors.value = value
        self.lock_odometry.release()

    def enableProximitySensor(self, value):
        self.lock_odometry.acquire()
        self.enable_proximity_sensor.value = value
        self.lock_odometry.release()

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
        self.is_spinning.value = w != 0
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
            self.history_dg_left.append(grad_izq)
            self.history_dg_right.append(grad_der)

            # Suaviza los valores con la media
            rad_izq = math.radians(sum(self.history_dg_left) / self.history_max_size)
            rad_der = math.radians(sum(self.history_dg_right) / self.history_max_size)

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

    def readSensors(self, read_proximity=True, read_gyro=True):
        """
        Read both gyroscopes and proximity sensor
        :param read_proximity: If false return 0 for proximity
        :param read_gyro: If false return 255 for gyro
        :return: gyroscope_1, gyroscope_2, proximity
        """

        gyro_1 = 0
        gyro_2 = 0

        if read_gyro:
            self.history_gyro_1.append(self.BP.get_sensor(self.BP.PORT_3)[0])
            self.history_gyro_2.append(self.BP.get_sensor(self.BP.PORT_4)[0])

            gyro_1, gyro_2 = sum(self.history_gyro_1) / self.history_max_size, sum(
                self.history_gyro_2) / self.history_max_size

        proximity = 255
        if read_proximity:
            self.history_proximity.append(self.BP.get_sensor(self.motor_port_ultrasonic))
            proximity = sum(self.history_proximity) / self.history_max_size

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

            # Check if use sensors
            self.lock_odometry.acquire()
            is_spinning = self.is_spinning.value
            enable_gyro_sensors = self.enable_gyro_sensors.value
            enable_proximity_sensor = self.enable_proximity_sensor.value
            self.lock_odometry.release()

            # Get sensors data
            gyro_1, gyro_2, proximity = self.readSensors(read_proximity=enable_proximity_sensor,
                                                         read_gyro=enable_gyro_sensors)

            # Obtain precise th
            if is_spinning and enable_gyro_sensors:
                # Only if it is turning on read gyro sensors
                # Sensor 1
                actual_value_gyro_1 = - (gyro_1 - self.gyro_1_offset) * self.gyro_1_correction_factor * d_t

                # Sensor 2
                actual_value_gyro_2 = - (gyro_2 - self.gyro_2_offset) * self.gyro_2_correction_factor * d_t
                w = (w + actual_value_gyro_1 + actual_value_gyro_2) / 3.0
            else:
                self.history_gyro_1.append(self.gyro_1_offset)
                self.history_gyro_2.append(self.gyro_2_offset)

            # Update th
            th = th + d_t * w

            # Update odometry
            self.lock_odometry.acquire()
            if not self.odometry_reseted.value:
                x_odo.value = x
                y_odo.value = y
                th_odo.value = self.normalizeAngle(th)
            else:
                # In case odometry is reseted, jump this updating period
                self.odometry_reseted.value = False

            self.proximity.value = proximity
            self.lock_odometry.release()

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
            v = 0.25
        elif size < 80:
            v = 0.15
        else:
            v = 0.1

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
                        '''
                        if salida == 'A':
                            self.setSpeed(0, 0.4)
                        else:
                            self.setSpeed(0, -0.4)

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
                        '''
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
                self.BP.set_motor_dps(self.motor_port_basket, -270)
                time.sleep(0.27)
                self.BP.set_motor_dps(self.motor_port_basket, 0)
                self.basket_state = 'up'
            elif movement == 'down':
                self.BP.set_motor_dps(self.motor_port_basket, 270)
                time.sleep(0.27)
                self.BP.set_motor_dps(self.motor_port_basket, 0)
                self.basket_state = 'down'

    def detectObstacle(self, number_of_cells=1):
        if is_debug:
            variable = False
            return variable
        else:
            self.lock_odometry.acquire()
            sensor_value = self.proximity.value
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
                    #print(x_odo, y_odo, x, y)
                    last_error = actual_error
                    actual_error = math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2)
                    t_next_period += self.P
                    delay_until(t_next_period)
        else:
            # Stop when be in error margin
            while position_error_margin < math.sqrt((x_odo - x) ** 2 + (y_odo - y) ** 2):
                [x_odo, y_odo, _] = self.readOdometry()
                t_next_period += self.P
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
                t_next_period += self.P
                delay_until(t_next_period)
        # print("He llegado a : ", th_odo, " y busco: ", th)

    def orientate(self, aligned_angle):
        [_, _, th_actual] = self.readOdometry()

        # Turn
        turn_speed = math.pi / 3

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
        self.wait_for_th(aligned_angle, 0.1)

        correction_speed = np.sign(turn_speed) * math.pi / 8

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

        # Turn
        self.orientate(aligned_angle)

        # Detect wall
        if self.detectObstacle():
            self.setSpeed(0, 0)
            return False
        else:
            # Go forward
            self.setSpeed(0.25, 0)
            self.wait_for_position(final_x, final_y, 0.2)

            # Stop robot
            self.setSpeed(0, 0)

            return True

    def update_color(self):
        # Color = -1 is NONE
        # Color = 0 is WHITE
        # Color = 1 is BLACK
        # Color = 2 is LINE

        value = self.BP.get_sensor(self.color_sensor)

        #print(value)

        self.color[self.color_vector_pos] = value
        self.color_vector_pos = (self.color_vector_pos + 1) % 10

        if self.color[0] >= self.color_limit:
            actual_color = 1
        elif self.color[0] < self.color_limit:
            actual_color = 0

        for i in range(1, 10):
            if self.color[0] >= self.color_limit and actual_color == 0:
                actual_color = -1
                break
            elif self.color[0] < self.color_limit and actual_color == 1:
                actual_color = -1
                break

        return actual_color

    def detect_color(self):
        actual_color = self.update_color()
        while actual_color == -1:
            actual_color = self.update_color()

        return actual_color

    def turn_off_color(self):
        self.BP.set_sensor_type(self.color_sensor, self.BP.SENSOR_TYPE.NXT_LIGHT_OFF)




