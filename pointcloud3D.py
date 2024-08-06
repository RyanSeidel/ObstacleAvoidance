import logging
import math
import pandas as pd
import os

import sys
import time
import random
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

        self.last_obstacle = None
        self.distance = None # comparing to new obstacle and last obstacle distance
        self.last_direction_x = None
        self.last_direction_y = None
        self.excel_data = []
        self.sensor_data = {
            'front': None,
            'back': None,
            'left': None,
            'right': None,
            'up': None
        }
        self.timer_start = time.time() 
        
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

        self.sensor_data['front'] = m['front']/1000
        self.sensor_data['back'] = m['back']/1000
        self.sensor_data['left'] = m['left']/1000
        self.sensor_data['right'] = m['right']/1000
        self.sensor_data['up'] = m['up']/1000
        self.sensor_data['down'] = m['down']/1000

        # reset it

        # if (m['up'] < SENSOR_TH):
        #     up = [o[0], o[1], o[2] + m['up'] / 1000.0]
        #     data.append(self.rot(roll, pitch, yaw, o, up) , 'up'))

        # if (m['down'] < SENSOR_TH and PLOT_SENSOR_DOWN):
        #     down = [o[0], o[1], o[2] - m['down'] / 1000.0]
        #     data.append(self.rot(roll, pitch, yaw, o, down) , 'down'))

        if (m['left'] < SENSOR_TH):
            left = [o[0], o[1] + m['left'] / 1000.0, o[2]]
            elapsed_time = time.time() - self.timer_start 
            data.append((self.rot(roll, pitch, yaw, o, left), 'left', elapsed_time))

        if (m['right'] < SENSOR_TH):
            right = [o[0], o[1] - m['right'] / 1000.0, o[2]]
            elapsed_time = time.time() - self.timer_start 
            data.append((self.rot(roll, pitch, yaw, o, right) , 'right', elapsed_time))

        if (m['front'] < SENSOR_TH):
            front = [o[0] + m['front'] / 1000.0, o[1], o[2]]
            elapsed_time = time.time() - self.timer_start 
            data.append((self.rot(roll, pitch, yaw, o, front), 'front', elapsed_time))

        if (m['back'] < SENSOR_TH):
            back = [o[0] - m['back'] / 1000.0, o[1], o[2]]
            elapsed_time = time.time() - self.timer_start 
            data.append((self.rot(roll, pitch, yaw, o, back), 'back', elapsed_time))

        return data

    # this will give the exact points of the obstacles
    def set_measurement(self, measurements):
        
        data_with_labels = self.rotate_and_create_points(measurements)
        data = [item[0] for item in data_with_labels]
        o = self.last_pos

        if (len(data) > 0):

            Move = 0

            # Makes an array of coordinates
            self.meas_data = np.append(self.meas_data, data, axis=0)
            self.meas_markers.set_data(self.meas_data, face_color='blue', size=5)

            print("The obstacles:", data_with_labels)

            #print("The drone position:", self._drone_position)

            obstacle = data_with_labels[0][0] # first element and # first coordinates
            label = data_with_labels[0][1]
            elapsed_time = data_with_labels[0][2]


            print("The obstacle ", obstacle)

            distance_x = abs(obstacle[0] - self._drone_position[0])
            distance_y = abs(obstacle[1] - self._drone_position[1])

            # distance_z = (obstacle[2] - self._drone_position[2])

            # distance_x = 2
            # distance_y = 2

            print(f"Distances: dx={distance_x}, dy={distance_y}")

            # Distances: dx=0.9316756574206697, dy=-0.022924507261234073, dz=-0.008884612349004672
            # The obstacles: [(array([1.13539016, 1.14460365, 0.27954656]), 'front')]
            # The obstacle  [1.13539016 1.14460365 0.27954656]
            
            action = 0

            if self.last_obstacle is not None:
                last_obstacle_coords = np.array(self.last_obstacle)
                current_obstacle_coords = np.array([obstacle[0], obstacle[1], obstacle[2]])
                self.distance = np.linalg.norm(current_obstacle_coords - last_obstacle_coords)

            self.last_obstacle = [obstacle[0], obstacle[1], obstacle[2]] 

            #Determine relative position
            if distance_x < .7 and data_with_labels[0][1] == "front":
                self.autonomousMovement(data_with_labels[0][1])
                action = time.time() - self.timer_start
                Move = 1
                self.meas_data = np.empty((0,3)) # Clear the Data if no measurements
                self.meas_markers.set_data(self.meas_data)
            
            elif distance_x < .7 and data_with_labels[0][1] == "back":
                self.autonomousMovement(data_with_labels[0][1])
                action = time.time() - self.timer_start
                Move = 1
                self.meas_data = np.empty((0,3)) # Clear the Data if no measurements
                self.meas_markers.set_data(self.meas_data)

            elif distance_y < .7 and data_with_labels[0][1] == "right":  
                self.autonomousMovement(data_with_labels[0][1])
                action = time.time() - self.timer_start
                Move = 1
                self.meas_data = np.empty((0,3)) # Clear the Data if no measurements
                self.meas_markers.set_data(self.meas_data)

            elif distance_y < .7 and data_with_labels[0][1] == "left":
                self.autonomousMovement(data_with_labels[0][1])
                action = time.time() - self.timer_start
                Move = 1
                self.meas_data = np.empty((0,3)) # Clear the Data if no measurements
                self.meas_markers.set_data(self.meas_data)

            self.excel_data.append({
                'object detected': elapsed_time,
                'action respond' : action,
                'drone_x': self._drone_position[0],
                'drone_y': self._drone_position[1],
                'drone_z': self._drone_position[2],
                'obstacle_x': obstacle[0],
                'obstacle_y': obstacle[1],
                'obstacle_z': obstacle[2],
                'label': label,
                'Move': Move
            })

            # needs to append a row in here in excel and move on to next row..
            # Append new data to Excel file
            self.append_to_excel('obstacle_gabetiery.xlsx')

            distance_x = 2

            distance_y = 2

            Move = 0

            action = 0

            elapsed_time = 0

            obstacle = [-9999,-9999,-9999]



    def append_to_excel(self, file_name):
        # Convert the list of dictionaries to a DataFrame
        new_data_df = pd.DataFrame(self.excel_data)

        if os.path.exists(file_name):
            try:
                # If file exists, load existing data into a DataFrame
                existing_data_df = pd.read_excel(file_name)
                # Append the new data to the existing data
                combined_data_df = pd.concat([existing_data_df, new_data_df], ignore_index=True)
            except Exception as e:
                print(f"Error reading {file_name}: {e}")
                combined_data_df = new_data_df
        else:
            # If file does not exist, the combined data is just the new data
            combined_data_df = new_data_df

        # Save the combined data to the Excel file
        combined_data_df.to_excel(file_name, index=False)

        # Clear the excel_data list after writing to the file to avoid duplicate entries
        self.excel_data = []



    def set_obstacle_array(self, data):
        threshold = 0.1
        clean_data = []
        for point in data:
            if all(np.linalg.norm(point - existing_point) >= threshold for existing_point in clean_data):
                clean_data.append(point)
        return clean_data

    def get_measurement(self):
        return self.obstacles

    def autonomousMovement(self, direction):
            
            direction_x = random.choice([1, -1])
            direction_y = random.choice([1, -1])

            if self.distance is not None and self.distance < .400: # .4m distance to keep going right
                direction_x = self.last_direction_x
                direction_y = self.last_direction_y

            # does this check if it on front / back side?
            if direction == "front" or direction == "back":
                self.keyCB('y', direction_y)  # Move right
                time.sleep(.5)

            # does this check if it on front / back side?
            if direction == "left" or direction == "right":
                self.keyCB('x', direction_x)  # Move right
                time.sleep(.5)

            self.keyCB('x', 0)
            self.keyCB('y', 0)

            self.last_direction_x = direction_x
            self.last_direction_y = direction_y

if __name__ == '__main__':
    appQt = QtWidgets.QApplication(sys.argv)
    win = MainWindow(URI)
    win.show()
    appQt.exec()
