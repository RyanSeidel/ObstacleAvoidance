#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2018 Bitcraze AB
#
#  Crazyflie Python Library
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Example script that plots the output ranges from the Multiranger and Flow
deck in a 3D plot.

When the application is started the Crazyflie will hover at 0.3 m. The
Crazyflie can then be controlled by using keyboard input:
 * Move by using the arrow keys (left/right/forward/backwards)
 * Adjust the right with w/s (0.1 m for each keypress)
 * Yaw slowly using a/d (CCW/CW)
 * Yaw fast using z/x (CCW/CW)

There's additional setting for (see constants below):
 * Plotting the downwards sensor
 * Plotting the estimated Crazyflie position
 * Max threshold for sensors
 * Speed factor that set's how fast the Crazyflie moves

The demo is ended by either closing the graph window.

For the example to run the following hardware is needed:
 * Crazyflie 2.0
 * Crazyradio PA
 * Flow deck
 * Multiranger deck
"""
import logging
import math
import sys
import time
import random
import threading
from PIL import Image

import numpy as np
from vispy import scene
from vispy.scene import visuals
from vispy.scene.cameras import TurntableCamera

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

try:
    from sip import setapi
    setapi('QVariant', 2)
    setapi('QString', 2)
except ImportError:
    pass

from PyQt6 import QtCore, QtWidgets

logging.basicConfig(level=logging.INFO)

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Enable plotting of Crazyflie
PLOT_CF = True
# Enable plotting of down sensor
PLOT_SENSOR_DOWN = False
# Set the sensor threshold (in mm)
SENSOR_TH = 3000

# Set MOVE_ZONE threshold (in m)
CLOSE_RANGE = .700
# Set the speed factor for moving and rotating
SPEED_FACTOR = 0.3

# Share Resources
drone_position = [0, 0, 0]  # x, y, z
detected_obstacles = None

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, URI):
        QtWidgets.QMainWindow.__init__(self)

        self.resize(700, 500)
        self.setWindowTitle('Multi-ranger point cloud')

        self.canvas = Canvas(self.updateHover)
        self.canvas.create_native()
        self.canvas.native.setParent(self)

        self.setCentralWidget(self.canvas.native)

        cflib.crtp.init_drivers()
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # Connect callbacks from the Crazyflie API
        self.cf.connected.add_callback(self.connected)
        self.cf.disconnected.add_callback(self.disconnected)

        # Connect to the Crazyflie
        self.cf.open_link(URI)

        self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.3}

        self.hoverTimer = QtCore.QTimer()
        self.hoverTimer.timeout.connect(self.sendHoverCommand)
        self.hoverTimer.setInterval(100)
        self.hoverTimer.start()

        self.last_obstacle_x = None  # Store the last obstacle's x position
        self.last_obstacle_y = None  # Store the last obstacle's y position
        self.last_choice_y = None
        self.last_choice_x = None

    def sendHoverCommand(self):
        self.cf.commander.send_hover_setpoint(
            self.hover['x'], self.hover['y'], self.hover['yaw'],
            self.hover['height'])

    def updateHover(self, k, v):
        if (k != 'height'):
            self.hover[k] = v * SPEED_FACTOR
        else:
            self.hover[k] += v

    def disconnected(self, URI):
        print('Disconnected')

    def connected(self, URI):
        print('We are now connected to {}'.format(URI))

        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=100)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')

        try:
            self.cf.log.add_config(lpos)
            lpos.data_received_cb.add_callback(self.pos_data)
            lpos.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Position log config, bad configuration.')

        lmeas = LogConfig(name='Meas', period_in_ms=100)
        lmeas.add_variable('range.front')
        lmeas.add_variable('range.back')
        lmeas.add_variable('range.up')
        lmeas.add_variable('range.left')
        lmeas.add_variable('range.right')
        lmeas.add_variable('range.zrange')
        lmeas.add_variable('stabilizer.roll')
        lmeas.add_variable('stabilizer.pitch')
        lmeas.add_variable('stabilizer.yaw')

        try:
            self.cf.log.add_config(lmeas)
            lmeas.data_received_cb.add_callback(self.meas_data)
            lmeas.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Measurement log config, bad configuration.')

    def pos_data(self, timestamp, data, logconf):
        position = [
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z']
        ]
        self.canvas.set_position(position)

    def meas_data(self, timestamp, data, logconf):
        measurement = {
            'roll': data['stabilizer.roll'],
            'pitch': data['stabilizer.pitch'],
            'yaw': data['stabilizer.yaw'],
            'front': data['range.front'],
            'back': data['range.back'],
            'up': data['range.up'],
            'down': data['range.zrange'],
            'left': data['range.left'],
            'right': data['range.right']
        }
        self.canvas.set_measurement(measurement)

    def closeEvent(self, event):
        if (self.cf is not None):
            self.cf.close_link()


class Canvas(scene.SceneCanvas):
    def __init__(self, keyupdateCB):

        self._drone_position = [None, None, None]
        self.obstacles = None
   
        scene.SceneCanvas.__init__(self, keys=None)
        self.size = 800, 600
        self.unfreeze()
        self.view = self.central_widget.add_view()
        self.view.bgcolor = '#ffffff'
        self.view.camera = TurntableCamera(
            fov=10.0, distance=30.0, up='+z', center=(0.0, 0.0, 0.0))
        self.last_pos = [0, 0, 0]
        self.pos_markers = visuals.Markers()
        self.meas_markers = visuals.Markers()
        self.pos_data = np.array([0, 0, 0], ndmin=2)
        self.meas_data = np.array([0, 0, 0], ndmin=2)
        self.lines = []

        self.view.add(self.pos_markers)
        self.view.add(self.meas_markers)
        for i in range(6):
            line = visuals.Line()
            self.lines.append(line)
            self.view.add(line)

        self.keyCB = keyupdateCB

        self.freeze()

        scene.visuals.XYZAxis(parent=self.view.scene)

    def on_key_press(self, event):
        if (not event.native.isAutoRepeat()):
            if (event.native.key() == QtCore.Qt.Key.Key_Left):
                self.keyCB('y', 1)
            if (event.native.key() == QtCore.Qt.Key.Key_Right):
                self.keyCB('y', -1)
            if (event.native.key() == QtCore.Qt.Key.Key_Up):
                self.keyCB('x', 1)
            if (event.native.key() == QtCore.Qt.Key.Key_Down):
                self.keyCB('x', -1)
            if (event.native.key() == QtCore.Qt.Key.Key_A):
                self.keyCB('yaw', -70)
            if (event.native.key() == QtCore.Qt.Key.Key_D):
                self.keyCB('yaw', 70)
            if (event.native.key() == QtCore.Qt.Key.Key_Z):
                self.keyCB('yaw', -200)
            if (event.native.key() == QtCore.Qt.Key.Key_X):
                self.keyCB('yaw', 200)
            if (event.native.key() == QtCore.Qt.Key.Key_W):
                self.keyCB('height', 0.1)
            if (event.native.key() == QtCore.Qt.Key.Key_S):
                self.keyCB('height', -0.1)

    def on_key_release(self, event):
        if (not event.native.isAutoRepeat()):
            if (event.native.key() == QtCore.Qt.Key.Key_Left):
                self.keyCB('y', 0)
            if (event.native.key() == QtCore.Qt.Key.Key_Right):
                self.keyCB('y', 0)
            if (event.native.key() == QtCore.Qt.Key.Key_Up):
                self.keyCB('x', 0)
            if (event.native.key() == QtCore.Qt.Key.Key_Down):
                self.keyCB('x', 0)
            if (event.native.key() == QtCore.Qt.Key.Key_A):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key.Key_D):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key.Key_W):
                self.keyCB('height', 0)
            if (event.native.key() == QtCore.Qt.Key.Key_S):
                self.keyCB('height', 0)
            if (event.native.key() == QtCore.Qt.Key.Key_Z):
                self.keyCB('yaw', 0)
            if (event.native.key() == QtCore.Qt.Key.Key_X):
                self.keyCB('yaw', 0)

    def set_position(self, pos):

        self._drone_position = pos
        self.last_pos = pos

        #print("Drone position: ", self.last_pos)

        if (PLOT_CF):
            self.pos_data = np.append(self.pos_data, [pos], axis=0)
            self.pos_markers.set_data(self.pos_data, face_color='red', size=3)

        self.pos_data = np.empty((0,3))  

    def get_position(self):
        return self._drone_position      

    def rot(self, roll, pitch, yaw, origin, point):
        cosr = math.cos(math.radians(roll))
        cosp = math.cos(math.radians(pitch))
        cosy = math.cos(math.radians(yaw))

        sinr = math.sin(math.radians(roll))
        sinp = math.sin(math.radians(pitch))
        siny = math.sin(math.radians(yaw))

        roty = np.array([[cosy, -siny, 0],
                         [siny, cosy, 0],
                         [0, 0,    1]])

        rotp = np.array([[cosp, 0, sinp],
                         [0, 1, 0],
                         [-sinp, 0, cosp]])

        rotr = np.array([[1, 0,   0],
                         [0, cosr, -sinr],
                         [0, sinr,  cosr]])

        rotFirst = np.dot(rotr, rotp)

        rot = np.array(np.dot(rotFirst, roty))

        tmp = np.subtract(point, origin)
        tmp2 = np.dot(rot, tmp)
        return np.add(tmp2, origin)

    def rotate_and_create_points(self, m):
        data = []
        o = self.last_pos
        roll = m['roll']
        pitch = -m['pitch']
        yaw = m['yaw']

        # if (m['up'] < SENSOR_TH):
        #     up = [o[0], o[1], o[2] + m['up'] / 1000.0]
        #     data.append(self.rot(roll, pitch, yaw, o, up))

        # if (m['down'] < SENSOR_TH and PLOT_SENSOR_DOWN):
        #     down = [o[0], o[1], o[2] - m['down'] / 1000.0]
        #     data.append(self.rot(roll, pitch, yaw, o, down))

        # if (m['left'] < SENSOR_TH):
        #     left = [o[0], o[1] + m['left'] / 1000.0, o[2]]
        #     data.append(self.rot(roll, pitch, yaw, o, left))

        # if (m['right'] < SENSOR_TH):
        #     right = [o[0], o[1] - m['right'] / 1000.0, o[2]]
        #     data.append(self.rot(roll, pitch, yaw, o, right))

        if (m['front'] < SENSOR_TH):
            front = [o[0] + m['front'] / 1000.0, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, front))

        # if (m['back'] < SENSOR_TH):
        #     back = [o[0] - m['back'] / 1000.0, o[1], o[2]]
        #     data.append(self.rot(roll, pitch, yaw, o, back))

        return data

    # this will give the exact points of the obstacles
    def set_measurement(self, measurements):
        
        data = self.rotate_and_create_points(measurements)
        o = self.last_pos

        for i in range(6):
            if (i < len(data)):
                o = self.last_pos
                self.lines[i].set_data(np.array([o, data[i]]))
            else:
                self.lines[i].set_data(np.array([o, o]))

            if (len(data) > 0):
                # Makes an array of coordinates
                self.meas_data = np.append(self.meas_data, data, axis=0)
                self.meas_markers.set_data(self.meas_data, face_color='blue', size=5)

                self.obstacles = list(self.set_obstacle_array(self.meas_data))

                print("The obstacles:", self.obstacles)

                print("The drone position:", self._drone_position)

                obstacle = self.obstacles[0]

                distance_x = abs(obstacle[0] - self._drone_position[0])
                distance_y = abs(obstacle[1] - self._drone_position[1])
                distance_z = abs(obstacle[2] - self._drone_position[2])

                print(f"Distances: dx={distance_x}, dy={distance_y}, dz={distance_z}")

                # The obstacles: [array([2.37211102, 0.60582068, 0.29161747]), array([2.29645645, 0.70252874, 0.29735338]), array([2.26021144, 0.75450092, 0.17799763]), array([2.27420395, 0.801536  , 0.30733058])]
                # The drone position: [1.3602542877197266, 0.8340490460395813, 0.3002752363681793]
                # Distances: dx=1.0118567338918272, dy=0.22822836287392667, dz=0.00865776728858475

                #     # Determine relative position
                # if distance_x > 0:
                #     print("Obstacle is to the right")
                # elif distance_x < 0:
                #     print("Obstacle is to the left")
    
                # if distance_y > 0:
                #     print("Obstacle is in front")
                # elif distance_y < 0:
                #     print("Obstacle is behind")

                # if distance_z > 0:
                #     print("Obstacle is above")
                # elif distance_z < 0:
                #     print("Obstacle is below")

                self.obstacles.pop(0)

#                 The obstacles: [array([-0.50927845,  5.35850837,  0.28616967]), array([-0.58885745,  5.41585241,  0.30648495])]
# T               The drone position: [-1.2327373027801514, 5.385387420654297, 0.3013294041156769]

                #self.autonomousMovement()

            else:
                self.meas_data = np.empty((0,3)) # Clear the Data if no measurements
                self.meas_markers.set_data(self.meas_data)

        
        #Store the measurement coordinates
        #print("Measurement Coordinates:", data)

        # Measurement Coordinates [array([-0.9539474, 0.22610241, 0.28791662])]
    def set_obstacle_array(self, data):
        threshold = 0.1
        clean_data = []
        for point in data:
            if all(np.linalg.norm(point - existing_point) >= threshold for existing_point in clean_data):
                clean_data.append(point)
        return clean_data

    def get_measurement(self):
        return self.obstacles

    def autonomousMovement(self):

        # print("The drone position is:", drone_position_copy)


        detected_obstacles_copy = list(self.get_measurement())

        # print("The obstacle measurement is:", detected_obstacles_copy)

        # The obstacle measurement is: [array([1.73194036, 1.34338527, 0.30324588]), array([1.73194036, 1.34338527, 0.30324588]), array([1.73194036, 1.34338527, 0.30324588]), array([1.73194036, 1.34338527, 0.30324588]), array([1.73194036, 1.34338527, 0.30324588])]

    #         #DEBUG PRINT THE LIST

        if detected_obstacles_copy:
            direction_x = random.choice([1])
            direction_y = random.choice([1])

            for ox, oy, oz in detected_obstacles_copy:
                
                drone_position_copy = self.get_position()

                if abs(ox - drone_position_copy[0]) <= CLOSE_RANGE and abs(oy - drone_position_copy[1]) <= CLOSE_RANGE and abs(oz - drone_position_copy[2]):
                
                # print(f"Processing obstacle at ({ox}, {oy}, {oz})")
                # Processing obstacle at (10.153203529201313, 11.334310824762154, 0.2886571084400498)
                    print(f"Drone position at ({drone_position_copy[0]}, {drone_position_copy[1]}, {drone_position_copy[2]})")

                # does this check if it on front / back side?
                    if ox < drone_position_copy[0] or ox > drone_position_copy[0]:
                        self.keyCB('y', direction_y)  # Move right
                        time.sleep(.5)
                        

    #                 Drone position at ((-6.257425785064697, 3.0765204429626465, 0.3031400442123413))
    #                 Processing obstacle at (-5.064485290294928, 3.0580519818953515, 0.2972974586255614)
    #                 Drone position at ((-6.257425785064697, 3.0765204429626465, 0.3031400442123413))
    #                 Processing obstacle at (-5.058312860085037, 3.0674651005196787, 0.2970936338361484)
    #                 Drone position at ((-6.257425785064697, 3.0765204429626465, 0.3031400442123413))
    #                 Processing obstacle at (-5.052434920632069, 3.0728931777986115, 0.30016353932620077)
    #                 Drone position at ((-6.257425785064697, 3.0765204429626465, 0.3031400442123413))
    #                         print("Obstacle is too close!")  
    #                     # x is the front..? 

    #                     if self.last_obstacle_x is not None and self.last_obstacle_y is not None:
    #                         if abs(ox - self.last_obstacle_x) <= CONTINUE_DISTANCE and abs(oy - self.last_obstacle_y) <= CONTINUE_DISTANCE:
    #                             direction_x = self.last_choice_x
    #                             direction_y = self.last_choice_y
    #                     if distance_x <= CLOSE_RANGE:
    #                         print(f"The Distance is ({distance_x})")
                            
    #                     if oy < self.drone_y or oy > self.drone_y:
    #                         self.keyCB('x', direction_x)  # Move forward
    #                         time.sleep(.5)
    #                     self.last_obstacle_x = ox
    #                     self.last_obstacle_y = oy
    #                     self.last_choice_x = direction_x
    #                     self.last_choice_y = direction_y

                    self.keyCB('x', 0)
                    self.keyCB('y', 0)

    #                     self.updateGrid(self.drone_x, self.drone_y)



if __name__ == '__main__':
    appQt = QtWidgets.QApplication(sys.argv)
    win = MainWindow(URI)
    win.show()
    appQt.exec()
