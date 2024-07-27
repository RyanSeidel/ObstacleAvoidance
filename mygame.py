import sys
from PyQt6 import QtCore, QtWidgets, QtGui
import threading
import time
import random

# Constants for the grid size and speed factor
GRID_SIZE = 61
SPEED_FACTOR = 1  # Adjust the speed factor as needed
DETECTION_RANGE = 30  # Detection range for the obstacle
CLOSE_RANGE = 7  # Distance at which the player makes a decision

class PlayerController(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self.resize(700, 700)
        self.setWindowTitle('2D GRID')

        self.player = {'x': 30, 'y': 30}
        self.obstacle = {'x': 30, 'y': 10}
        self.obstacle_direction = 'DOWN'

        # Add a QLabel widget for displaying the grid
        self.gridLabel = QtWidgets.QLabel(self)
        self.gridLabel.setGeometry(50, 50, 600, 600)
        self.updateGrid(self.player['x'], self.player['y'])

        # Timer for continuous movement
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.moveObstacle)
        self.timer.start(100)  # Adjust the interval as needed

        self.activeKeys = set()

        # Start a thread for obstacle detection and player movement
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.autonomousMovement)
        self.thread.start()

    def moveObstacle(self):
        # Move the obstacle straight down
        self.updateObstaclePosition('y', SPEED_FACTOR)
        self.obstacle_direction = 'DOWN'  # Since the obstacle always moves down

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

    def updateObstaclePosition(self, k, v):
        new_x = self.obstacle['x']
        new_y = self.obstacle['y']
        
        if k == 'x':
            new_x = max(0, min(GRID_SIZE - 1, self.obstacle['x'] + v))
        elif k == 'y':
            new_y = max(0, min(GRID_SIZE - 1, self.obstacle['y'] + v))
        
        self.obstacle['x'] = new_x
        self.obstacle['y'] = new_y

        self.updateGrid(self.player['x'], self.player['y'])

    def detectObstacles(self):
        player_x = self.player['x']
        player_y = self.player['y']
        obstacle_x = self.obstacle['x']
        obstacle_y = self.obstacle['y']

        if player_x == obstacle_x and abs(player_y - obstacle_y) <= DETECTION_RANGE:
            distance = abs(player_y - obstacle_y)
            if player_y < obstacle_y:
                print(f"Obstacle detected in front! Distance: {distance}")
                return 'front', distance
            elif player_y > obstacle_y:
                print(f"Obstacle detected in back! Distance: {distance}")
                return 'back', distance
        
        if player_y == obstacle_y and abs(player_x - obstacle_x) <= DETECTION_RANGE:
            distance = abs(player_x - obstacle_x)
            if player_x < obstacle_x:
                print(f"Obstacle detected to the right! Distance: {distance}")
                return 'right', distance
            elif player_x > obstacle_x:
                print(f"Obstacle detected to the left! Distance: {distance}")
                return 'left', distance
        
        return None, None

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

        # Draw the obstacle position
        pen.setColor(QtCore.Qt.GlobalColor.blue)
        painter.setPen(pen)
        painter.drawText(self.obstacle['x'] * 10, self.obstacle['y'] * 10, 'O')

        # Draw the player position
        pen.setColor(QtCore.Qt.GlobalColor.red)
        painter.setPen(pen)
        painter.drawText(player_x * 10, player_y * 10, 'X')

        painter.end()

        self.gridLabel.setPixmap(pixmap)

    def autonomousMovement(self):
        while not self.stop_event.is_set():
            direction, distance = self.detectObstacles()

            if direction and distance <= CLOSE_RANGE:
                if direction in ['left', 'right']:
                    if self.obstacle_direction == 'DOWN':
                        self.updatePlayerPosition('y', -SPEED_FACTOR)
                    elif self.obstacle_direction == 'UP':
                        self.updatePlayerPosition('y', SPEED_FACTOR)
                elif direction == 'front':
                    if random.choice(['left', 'right']) == 'left':
                        self.updatePlayerPosition('x', -SPEED_FACTOR)
                    else:
                        self.updatePlayerPosition('x', SPEED_FACTOR)
                elif direction == 'back':
                    if random.choice(['left', 'right']) == 'left':
                        self.updatePlayerPosition('x', -SPEED_FACTOR)
                    else:
                        self.updatePlayerPosition('x', SPEED_FACTOR)
            else:
                moving_direction = None

            time.sleep(0.1)

    def isObstacle(self, x, y):
        return x == self.obstacle['x'] and y == self.obstacle['y']

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
    appQt = QtWidgets.QApplication(sys.argv)
    mainWindow = PlayerController()
    mainWindow.show()
    sys.exit(appQt.exec())
