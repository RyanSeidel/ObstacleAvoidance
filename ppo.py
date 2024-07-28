import sys
from PyQt6 import QtCore, QtWidgets, QtGui
import threading
import time
import random
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import gymnasium as gym
from gymnasium import spaces

# Constants for the grid size and speed factor
GRID_SIZE = 61
SPEED_FACTOR = 1  # Adjust the speed factor as needed
DETECTION_RANGE = 30  # Detection range for the obstacle
CLOSE_RANGE = 7  # Distance at which the player makes a decision

# Define the custom environment
class GridEnvironment(gym.Env):
    def __init__(self, grid_size=61):
        super(GridEnvironment, self).__init__()
        self.grid_size = grid_size
        self.action_space = spaces.Discrete(4)  # 4 possible actions: up, down, left, right
        self.observation_space = spaces.Box(low=0, high=grid_size-1, shape=(2,), dtype=np.int32)
        self.seed()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.seed(seed)
        self.player = {'x': 30, 'y': 30}
        self.obstacles = [
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)}
]
        self.goal = {'x': 50, 'y': 50}  # Example goal position
        return np.array([self.player['x'], self.player['y']]), {}

    def step(self, action):
        if action == 0:  # Up
            new_y = max(0, self.player['y'] - 1)
            new_x = self.player['x']
        elif action == 1:  # Down
            new_y = min(self.grid_size - 1, self.player['y'] + 1)
            new_x = self.player['x']
        elif action == 2:  # Left
            new_x = max(0, self.player['x'] - 1)
            new_y = self.player['y']
        elif action == 3:  # Right
            new_x = min(self.grid_size - 1, self.player['x'] + 1)
            new_y = self.player['y']
        
        if not self.isObstacle(new_x, new_y):
            self.player['x'] = new_x
            self.player['y'] = new_y

        reward = self.calculate_reward()
        done = self.check_done()
        
        info = {}
        terminated = done
        truncated = False
        
        return np.array([self.player['x'], self.player['y']]), reward, terminated, truncated, info

    def calculate_reward(self):
        if self.isObstacle(self.player['x'], self.player['y']):
            return -10  # Penalize for collision
        if self.player['x'] == self.goal['x'] and self.player['y'] == self.goal['y']:
            return 10  # Reward for reaching the goal
        return 1  # Small reward for valid moves

    def check_done(self):
        if self.isObstacle(self.player['x'], self.player['y']):
            return True  # End episode if collision
        if self.player['x'] == self.goal['x'] and self.player['y'] == self.goal['y']:
            return True  # End episode if goal is reached
        return False

    def isObstacle(self, x, y):
        return any(obstacle['x'] == x and obstacle['y'] == y for obstacle in self.obstacles)

    def detect_obstacles(self):
        player_x = self.player['x']
        player_y = self.player['y']
        detected = []

        for obstacle in self.obstacles:
            obstacle_x = obstacle['x']
            obstacle_y = obstacle['y']

            if player_x == obstacle_x and abs(player_y - obstacle_y) <= DETECTION_RANGE:
                distance = abs(player_y - obstacle_y)
                if player_y < obstacle_y:
                    detected.append(('front', distance, 'DOWN'))
                elif player_y > obstacle_y:
                    detected.append(('back', distance, 'UP'))
            
            if player_y == obstacle_y and abs(player_x - obstacle_x) <= DETECTION_RANGE:
                distance = abs(player_x - obstacle_x)
                if player_x < obstacle_x:
                    detected.append(('right', distance, 'LEFT'))
                elif player_x > obstacle_x:
                    detected.append(('left', distance, 'RIGHT'))
        
        return detected

# Create the environment and check it
env = GridEnvironment()
check_env(env)

# Define the PPO model
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=10000)

# Define the PyQt6 application with PPO
class PlayerController(QtWidgets.QMainWindow):

    def __init__(self, model):
        super().__init__()
        self.model = model
        self.resize(700, 700)
        self.setWindowTitle('2D GRID')

        self.player = {'x': 30, 'y': 30}
        self.obstacles = [
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)},
    {'x': random.randint(0, GRID_SIZE-1), 'y': random.randint(0, GRID_SIZE-1), 'direction': random.choice(['UP', 'DOWN', 'LEFT', 'RIGHT']), 'start_time': time.time() + random.randint(0, 15)}
]

        self.goal = {'x': 50, 'y': 50}

        self.gridLabel = QtWidgets.QLabel(self)
        self.gridLabel.setGeometry(50, 50, 600, 600)
        self.updateGrid(self.player['x'], self.player['y'])

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.moveObstacles)
        self.timer.start(100)

        self.activeKeys = set()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.autonomousMovement)
        self.thread.start()

    def moveObstacles(self):
        current_time = time.time()
        for obstacle in self.obstacles:
            if 'start_time' in obstacle and current_time < obstacle['start_time']:
                continue  # Skip moving this obstacle until the start time is reached
            direction = obstacle['direction']
            if direction == 'DOWN':
                self.updateObstaclePosition(obstacle, 'y', SPEED_FACTOR)
            elif direction == 'UP':
                self.updateObstaclePosition(obstacle, 'y', -SPEED_FACTOR)
            elif direction == 'LEFT':
                self.updateObstaclePosition(obstacle, 'x', -SPEED_FACTOR)
            elif direction == 'RIGHT':
                self.updateObstaclePosition(obstacle, 'x', SPEED_FACTOR)

    def updatePlayerPosition(self, k, v):
        new_x = self.player['x']
        new_y = self.player['y']
        
        if k == 'x':
            new_x = max(0, min(GRID_SIZE - 1, self.player['x'] + v))
        elif k == 'y':
            new_y = max(0, min(GRID_SIZE - 1, self.player['y'] + v))
        
        if not self.isObstacle(new_x, new_y):
            self.player['x'] = new_x
            self.player['y'] = new_y
        
        self.updateGrid(self.player['x'], self.player['y'])

    def updateObstaclePosition(self, obstacle, k, v):
        new_x = obstacle['x']
        new_y = obstacle['y']
        
        if k == 'x':
            new_x = max(0, min(GRID_SIZE - 1, obstacle['x'] + v))
        elif k == 'y':
            new_y = max(0, min(GRID_SIZE - 1, obstacle['y'] + v))
        
        obstacle['x'] = new_x
        obstacle['y'] = new_y

        self.updateGrid(self.player['x'], self.player['y'])

    def detectObstacles(self):
        player_x = self.player['x']
        player_y = self.player['y']
        detected = []

        for obstacle in self.obstacles:
            obstacle_x = obstacle['x']
            obstacle_y = obstacle['y']

            if player_x == obstacle_x and abs(player_y - obstacle_y) <= DETECTION_RANGE:
                distance = abs(player_y - obstacle_y)
                if player_y < obstacle_y:
                    print(f"Obstacle detected in front! Distance: {distance}")
                    detected.append(('front', distance, 'DOWN'))
                elif player_y > obstacle_y:
                    print(f"Obstacle detected in back! Distance: {distance}")
                    detected.append(('back', distance, 'UP'))
            
            if player_y == obstacle_y and abs(player_x - obstacle_x) <= DETECTION_RANGE:
                distance = abs(player_x - obstacle_x)
                if player_x < obstacle_x:
                    print(f"Obstacle detected to the right! Distance: {distance}")
                    detected.append(('right', distance, 'LEFT'))
                elif player_x > obstacle_x:
                    print(f"Obstacle detected to the left! Distance: {distance}")
                    detected.append(('left', distance, 'RIGHT'))
        
        return detected

    def updateGrid(self, player_x, player_y):
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

        # Draw the obstacle positions
        pen.setColor(QtCore.Qt.GlobalColor.blue)
        painter.setPen(pen)
        for obstacle in self.obstacles:
            painter.drawText(obstacle['x'] * 10, obstacle['y'] * 10, 'O')

        # Draw the player position
        pen.setColor(QtCore.Qt.GlobalColor.red)
        painter.setPen(pen)
        painter.drawText(player_x * 10, player_y * 10, 'X')

        # Draw the goal position
        pen.setColor(QtCore.Qt.GlobalColor.green)
        painter.setPen(pen)
        painter.drawText(self.goal['x'] * 10, self.goal['y'] * 10, 'G')

        painter.end()

        self.gridLabel.setPixmap(pixmap)

    def autonomousMovement(self):
        while not self.stop_event.is_set():
            detected_obstacles = self.detectObstacles()
            if detected_obstacles:
                state = np.array([self.player['x'], self.player['y']])
                action, _states = self.model.predict(state)
                self.step(action)
            time.sleep(0.1)

    def step(self, action):
        if action == 0:  # Up
            self.player['y'] = max(0, self.player['y'] - 1)
        elif action == 1:  # Down
            self.player['y'] = min(GRID_SIZE - 1, self.player['y'] + 1)
        elif action == 2:  # Left
            self.player['x'] = max(0, self.player['x'] - 1)
        elif action == 3:  # Right
            self.player['x'] = min(GRID_SIZE - 1, self.player['x'] + 1)
        
        if self.isObstacle(self.player['x'], self.player['y']):
            print("Collision detected! Penalizing and stopping episode.")
            self.updateGrid(self.player['x'], self.player['y'])
            return  # End the step if collision happens
        
        self.updateGrid(self.player['x'], self.player['y'])

    def isObstacle(self, x, y):
        return any(obstacle['x'] == x and obstacle['y'] == y for obstacle in self.obstacles)

    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            self.activeKeys.add(event.key())

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            self.activeKeys.discard(event.key())

    def closeEvent(self, event):
        self.stop_event.set()
        self.thread.join()

if __name__ == '__main__':
    env = GridEnvironment()
    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=10000)
    
    appQt = QtWidgets.QApplication(sys.argv)
    mainWindow = PlayerController(model)
    mainWindow.show()
    sys.exit(appQt.exec())
