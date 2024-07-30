import logging
import sys
import time
import threading
import random

import gym
from gym import spaces
import numpy as np

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
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

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
CLOSE_RANGE = 10  # Distance at which the player makes a decision
CONTINUE_DISTANCE = 1 # Closest distance to last obstacle

class CrazyflieEnv(gym.Env):
    def __init__(self, controller):
        super(CrazyflieEnv, self).__init__()
        self.controller = controller

        # Define action and observation space
        # Actions: [x, y, z, yaw]
        self.action_space = spaces.Box(low=-1, high=1, shape=(4,), dtype=np.float32)

        # Observations: [front, back, left, right, x, y, z]
        self.observation_space = spaces.Box(low=0, high=GRID_SIZE, shape=(7,), dtype=np.float32)

    def reset(self):
        self.controller.reset_position()
        obs = self.controller.get_observation()
        return obs

    def step(self, action):
        self.controller.apply_action(action)
        time.sleep(0.1)  # Simulate a step duration
        obs = self.controller.get_observation()
        reward = self.controller.compute_reward()
        done = self.controller.is_done()
        return obs, reward, done, {}

    def render(self, mode='human'):
        self.controller.update_grid_display()

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
        
        self.updateGrid(30, 30)  # Start with the drone at the center

        self.last_obstacle_x = None  # Store the last obstacle's x position
        self.last_obstacle_y = None  # Store the last obstacle's y position
        self.last_choice_y = None
        self.last_choice_x = None

        self.drone_x = 30
        self.drone_y = 30

        self.sensor_data = {
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

    def pos_data(self, timestamp, data, logconf):
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        # Transform the coordinates to fit the grid
        self.drone_x = abs(round(x * 10) + 30)  # Convert meters to grid units and center
        self.drone_y = abs(round(y * 10) + 30)  # Convert meters to grid units and center
        # Ensure the coordinates are within bounds
        self.drone_x = max(0, min(60, self.drone_x))
        self.drone_y = max(0, min(60, self.drone_y))
        # Update the grid with the new drone position
        self.updateGrid(self.drone_x, self.drone_y)

    def multi_data(self, timestamp, data, logconf):
        self.sensor_data['front'] = data['range.front'] / 1000  # Convert mm to meters
        self.sensor_data['back'] = data['range.back'] / 1000  # Convert mm to meters
        self.sensor_data['left'] = data['range.left'] / 1000  # Convert mm to meters
        self.sensor_data['right'] = data['range.right'] / 1000  # Convert mm to meters

        self.detectObstacles()
        self.updateGrid(self.drone_x, self.drone_y)

    def detectObstacles(self):
        self.obstacles.clear()

        front = self.sensor_data['front']
        back = self.sensor_data['back']
        left = self.sensor_data['left']
        right = self.sensor_data['right']

        if front < OBSTACLE_THRESHOLD:
            front_obstacle_x = self.drone_x
            front_obstacle_y = self.drone_y - int(front * 10)
            self.obstacles.add((front_obstacle_x, front_obstacle_y))

        if back < OBSTACLE_THRESHOLD:
            back_obstacle_x = self.drone_x
            back_obstacle_y = self.drone_y + int(back * 10)
            self.obstacles.add((back_obstacle_x, back_obstacle_y))

        if left < OBSTACLE_THRESHOLD:
            left_obstacle_x = self.drone_x - int(left * 10)
            left_obstacle_y = self.drone_y
            self.obstacles.add((left_obstacle_x, left_obstacle_y))

        if right < OBSTACLE_THRESHOLD:
            right_obstacle_x = self.drone_x + int(right * 10)
            right_obstacle_y = self.drone_y
            self.obstacles.add((right_obstacle_x, right_obstacle_y))

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

    def reset_position(self):
        self.hover = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'height': 0.3}
        self.drone_x = 30
        self.drone_y = 30
        self.obstacles.clear()

    def apply_action(self, action):
        self.updateHover('x', action[0])
        self.updateHover('y', action[1])
        self.updateHover('z', action[2])
        self.updateHover('yaw', action[3])

    def get_observation(self):
        return np.array([
            self.sensor_data['front'] or 0.0,
            self.sensor_data['back'] or 0.0,
            self.sensor_data['left'] or 0.0,
            self.sensor_data['right'] or 0.0,
            self.drone_x,
            self.drone_y,
            self.hover['height']
        ])

    def compute_reward(self):
        # Example reward function: Negative reward for being close to obstacles
        reward = 0
        for obs in self.obstacles:
            distance = np.linalg.norm(np.array([self.drone_x, self.drone_y]) - np.array(obs))
            reward -= 1 / (distance + 1)
        return reward

    def is_done(self):
        # Example done condition: If the drone hits an obstacle
        for obs in self.obstacles:
            if np.linalg.norm(np.array([self.drone_x, self.drone_y]) - np.array(obs)) < 1:
                return True
        return False

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

    # Initialize the custom environment
    env = CrazyflieEnv(mainWindow)
    check_env(env)

    # Train the PPO agent
    model = PPO('MlpPolicy', env, verbose=1)
    model.learn(total_timesteps=10000)

    obs = env.reset()
    while True:
        action, _ = model.predict(obs)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
            obs = env.reset()

    sys.exit(appQt.exec())
