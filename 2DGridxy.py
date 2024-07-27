"""
Example script that controls the Crazyflie drone using keyboard input.

When the application is started, the Crazyflie will hover at 0.3 m. The
Crazyflie can then be controlled by using keyboard input:
 * Move by using the arrow keys (left/right/forward/backwards)
 * Adjust the height with w/s (0.1 m for each keypress)
 * Yaw slowly using a/d (CCW/CW)
 * Yaw fast using z/x (CCW/CW)

The demo is ended by closing the application window.

For the example to run the following hardware is needed:
 * Crazyflie 2.0
 * Crazyradio PA
 * Multi-ranger deck
"""
import logging
import sys
import time
import random

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

try:
    from sip import setapi
    setapi('QVariant', 2)
    setapi('QString', 2)
except ImportError:
    pass

from PyQt6 import QtCore, QtWidgets, QtGui

logging.basicConfig(level=logging.INFO)

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Set the speed factor for moving and rotating
SPEED_FACTOR = 0.3

# Constants for the grid size and obstacle detection threshold
GRID_SIZE = 61
OBSTACLE_THRESHOLD = 3.0  # Distance threshold in meters
MOVE_DISTANCE = 3

def calculate_speed(distance1, time1, distance2, time2):
    if None not in (distance1, distance2) and time1 != time2:
        return (distance2 - distance1) / (time2 - time1)
    return 0

def print_distance_and_speed(sensor_name, prev_distance, prev_time, current_distance, current_time):
    speed = calculate_speed(prev_distance, prev_time, current_distance, current_time)
    if current_distance is not None:
        print(f"{time.strftime('%H:%M:%S', time.gmtime(current_time))} - Distance to object {sensor_name}: {current_distance:.3f} meters, Speed: {speed:.3f} m/s")
    else:
        print(f"{time.strftime('%H:%M:%S', time.gmtime(current_time))} - No object detected {sensor_name}.")

class CrazyflieController(QtWidgets.QMainWindow):

    def __init__(self, URI):
        super().__init__()

        self.resize(700, 700)
        self.setWindowTitle('Multi-ranger 2D GRID')

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

        # Add a QLabel widget for displaying the grid
        self.gridLabel = QtWidgets.QLabel(self)
        self.gridLabel.setGeometry(50, 50, 600, 600)
        self.obstacles = set()  # Set to store obstacle positions
        self.updateGrid(GRID_SIZE // 2, GRID_SIZE // 2)
        self.updateGrid(30, 30)  # Start with the drone at the center

        self.drone_x = 30
        self.drone_y = 30

        self.prev_time = time.time()
        self.prev_distances = {
            'front': None,
            'back': None,
            'left': None,
            'right': None,
            'up': None
        }

    def sendHoverCommand(self):
        self.cf.commander.send_hover_setpoint(
            self.hover['x'], self.hover['y'], self.hover['yaw'], self.hover['height'])

    def updateHover(self, k, v):
        if k != 'height':
            self.hover[k] = v * SPEED_FACTOR
        else:
            self.hover[k] += v

    def disconnected(self, URI):
        print('Disconnected')
        self.updateGrid(-1, -1)  # Clear the grid

    def connected(self, URI):
        print(f'We are now connected to {URI}')
        self.updateGrid(30, 30)  # Center the drone initially

        # The definition of the logconfig can be made before connecting
        lpos = LogConfig(name='Position', period_in_ms=100)
        lpos.add_variable('stateEstimate.x')
        lpos.add_variable('stateEstimate.y')
        lpos.add_variable('stateEstimate.z')

        lmulti = LogConfig(name='MultiRanger', period_in_ms=100)
        lmulti.add_variable('range.front')
        lmulti.add_variable('range.back')
        lmulti.add_variable('range.left')
        lmulti.add_variable('range.right')

        try:
            self.cf.log.add_config(lpos)
            lpos.data_received_cb.add_callback(self.pos_data)
            lpos.start()

            self.cf.log.add_config(lmulti)
            lmulti.data_received_cb.add_callback(self.multi_data)
            lmulti.start()
        except KeyError as e:
            print(f'Could not start log configuration, {str(e)} not found in TOC')
        except AttributeError:
            print('Could not add Position log config, bad configuration.')

# this function is in charge of keeping track of the Crazyflie drone position in the air
    def pos_data(self, timestamp, data, logconf):
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        # Transform the coordinates to fit the grid
        self.drone_x = int(x * 10) + 30  # Convert meters to grid units and center
        self.drone_y = int(y * 10) + 30  # Convert meters to grid units and center
        # Ensure the coordinates are within bounds
        self.drone_x = max(0, min(60, self.drone_x))
        self.drone_y = max(0, min(60, self.drone_y))
        # Update the grid with the new drone position
        self.updateGrid(self.drone_x, self.drone_y)


    # Define a method to move the drone left or right
    def move_drone(self, direction):
        if direction == 'left':
            self.updateHover('y', -1)
            time.sleep(1)
              
        elif direction == 'right':
            self.updateHover('y', 1)
            time.sleep(1)
                        

#this function is in charge of using the multi ranger to track the distances between the obstacles
    def multi_data(self, timestamp, data, logconf):
        front = data['range.front'] / 1000  # Convert mm to meters
        back = data['range.back'] / 1000  # Convert mm to meters
        left = data['range.left'] / 1000  # Convert mm to meters
        right = data['range.right'] / 1000  # Convert mm to meters

        print(f' front: {front} meters')
        print(f'back: {back} meters')
        print(f'left: {left} meters')
        print(f'right: {right} meters')

        # Add obstacles to the grid based on the sensor data
        self.obstacles.clear()

        if front < OBSTACLE_THRESHOLD:
            front_obstacle_x = self.drone_x
            front_obstacle_y = self.drone_y - int(front * 10)
            front_obstacle = (front_obstacle_x, front_obstacle_y)
            self.obstacles.add(front_obstacle)
            # Calculate and print the distance
            front_distance_x = front_obstacle_x - self.drone_x
            front_distance_y = front_obstacle_y - self.drone_y
            print(f'Drone position is currently at {self.drone_x}, {self.drone_y}')
            print(f'Obstacle detected at: {front_obstacle[0]},{front_obstacle[1]} with distance X: {front_distance_x}, Y: {front_distance_y}') 

            # if abs(front_distance_x) < 7 and abs(front_distance_x) > 0:
            #     # Randomly choose direction to move: left or right
            #     direction = random.choice(['left', 'right'])
            #     self.move_drone(direction, MOVE_DISTANCE)

            # Check for obstacle in the y direction and move accordingly
            if 0 < abs(front_distance_y) < 7:
                direction = random.choice(['left'])
                self.move_drone(direction)

            self.updateHover('y', 0)
        else:
            print('No obstacles nearby. Drone is hovering.')


        if back < OBSTACLE_THRESHOLD:
            back_obstacle_x = self.drone_x
            back_obstacle_y = self.drone_y + int(back * 10)
            back_obstacle = (back_obstacle_x, back_obstacle_y)
            self.obstacles.add(back_obstacle)
            # Calculate and print the distance
            back_distance_x = back_obstacle_x - self.drone_x
            back_distance_y = back_obstacle_y - self.drone_y
            print(f'Obstacle detected at: {back_obstacle[0]}, {back_obstacle[1]} with distance X: {back_distance_x}, Y: {back_distance_y}')

        if left < OBSTACLE_THRESHOLD:
            left_obstacle_x = self.drone_x - int(left * 10)
            left_obstacle_y = self.drone_y
            left_obstacle = (left_obstacle_x, left_obstacle_y)
            self.obstacles.add(left_obstacle)
            # Calculate and print the distance
            left_distance_x = left_obstacle_x - self.drone_x
            left_distance_y = left_obstacle_y - self.drone_y
            print(f'Obstacle detected at: {left_obstacle[0]}, {left_obstacle[1]} with distance X: {left_distance_x}, Y: {left_distance_y}')

        if right < OBSTACLE_THRESHOLD:
            right_obstacle_x = self.drone_x + int(right * 10)
            right_obstacle_y = self.drone_y
            right_obstacle = (right_obstacle_x, right_obstacle_y)
            self.obstacles.add(right_obstacle)
            # Calculate and print the distance
            right_distance_x = right_obstacle_x - self.drone_x
            right_distance_y = right_obstacle_y - self.drone_y
            print(f'Obstacle detected at: {right_obstacle[0]}, {right_obstacle[1]} with distance X: {right_distance_x}, Y: {right_distance_y}')

        self.updateGrid(self.drone_x, self.drone_y)

    def updateGrid(self, drone_x, drone_y):
        pixmap = QtGui.QPixmap(610, 610)
        pixmap.fill(QtGui.QColor('white'))

        painter = QtGui.QPainter(pixmap)
        pen = QtGui.QPen(QtCore.Qt.GlobalColor.black)
        pen.setWidth(1)
        painter.setPen(pen)

        # Draw the grid
        for i in range(0, 610, 10):
            painter.drawLine(i, 0, i, 610)
            painter.drawLine(0, i, 610, i)

        # Draw obstacles
        pen.setColor(QtCore.Qt.GlobalColor.black)
        painter.setPen(pen)
        for (ox, oy) in self.obstacles:
            if 0 <= ox < GRID_SIZE and 0 <= oy < GRID_SIZE:
                painter.drawText(ox * 10, oy * 10, 'X')

        # Draw the drone position
        if 0 <= drone_x < GRID_SIZE and 0 <= drone_y < GRID_SIZE:
            pen.setColor(QtCore.Qt.GlobalColor.red)
            painter.setPen(pen)
            painter.drawText(drone_x * 10, drone_y * 10, 'X')

        painter.end()

        self.gridLabel.setPixmap(pixmap)

    def closeEvent(self, event):
        if self.cf is not None:
            self.cf.close_link()

    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            if event.key() == QtCore.Qt.Key.Key_Left:
                self.updateHover('y', 1)
            if event.key() == QtCore.Qt.Key.Key_Right:
                self.updateHover('y', -1)
            if event.key() == QtCore.Qt.Key.Key_Up:
                self.updateHover('x', 1)
            if event.key() == QtCore.Qt.Key.Key_Down:
                self.updateHover('x', -1)
            if event.key() == QtCore.Qt.Key.Key_A:
                self.updateHover('yaw', -70)
            if event.key() == QtCore.Qt.Key.Key_D:
                self.updateHover('yaw', 70)
            if event.key() == QtCore.Qt.Key.Key_Z:
                self.updateHover('yaw', -200)
            if event.key() == QtCore.Qt.Key.Key_X:
                self.updateHover('yaw', 200)
            if event.key() == QtCore.Qt.Key.Key_W:
                self.updateHover('height', 0.1)
            if event.key() == QtCore.Qt.Key.Key_S:
                self.updateHover('height', -0.1)

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            if event.key() == QtCore.Qt.Key.Key_Left or event.key() == QtCore.Qt.Key.Key_Right:
                self.updateHover('y', 0)
            if event.key() == QtCore.Qt.Key.Key_Up or event.key() == QtCore.Qt.Key.Key_Down:
                self.updateHover('x', 0)
            if event.key() == QtCore.Qt.Key.Key_A or event.key() == QtCore.Qt.Key.Key_D:
                self.updateHover('yaw', 0)
            if event.key() == QtCore.Qt.Key.Key_Z or event.key() == QtCore.Qt.Key.Key_X:
                self.updateHover('yaw', 0)
            if event.key() == QtCore.Qt.Key.Key_W or event.key() == QtCore.Qt.Key.Key_S:
                self.updateHover('height', 0)

if __name__ == '__main__':
    appQt = QtWidgets.QApplication(sys.argv)
    mainWindow = CrazyflieController(URI)
   
    mainWindow.show()
    
    sys.exit(appQt.exec())
