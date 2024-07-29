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

from PyQt6 import QtCore, QtWidgets, QtGui

logging.basicConfig(level=logging.INFO)

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E703')

if len(sys.argv) > 1:
    URI = sys.argv[1]

# Set the speed factor for moving and rotating
SPEED_FACTOR = 0.3

# Constants for the grid size and obstacle detection threshold
GRID_SIZE = 61
OBSTACLE_THRESHOLD = 0.5  # Distance threshold in meters

class CrazyflieController(QtWidgets.QMainWindow):

    def __init__(self, URI):
        super().__init__() # Super constructor (Executes QMainWindow constructor)

        self.resize(700, 700) # Resize the window
        self.setWindowTitle('Multi-ranger 2D GRID') # Name the window

        # Initialize Crazyflie 
        cflib.crtp.init_drivers() 
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # Connect callbacks from the Crazyflie API
            # CrazyflieController class -> Crazyflie class -> Caller class
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
        self.updateGrid(30, 30)  # Start with the drone at the center

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

    def pos_data(self, timestamp, data, logconf):
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        # Transform the coordinates to fit the grid
        grid_x = int((x + 3) * 10)  # Example transformation
        grid_y = int((y + 3) * 10)  # Example transformation
        # Ensure the coordinates are within bounds
        grid_x = max(0, min(60, grid_x))
        grid_y = max(0, min(60, grid_y))
        # Update the grid with the new drone position
        self.updateGrid(grid_x, grid_y)

    def multi_data(self, timestamp, data, logconf):
        front = data['range.front']/1000
        back = data['range.back']/1000
        left = data['range.left']/1000
        right = data['range.right']/1000

        print(f' front: {front} meters')
        print(f'back: {back} meters')
        print(f'left: {left} meters')
        print(f'right: {right} meters')
      

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

        # Draw the drone position DRONE POSITIONS
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
