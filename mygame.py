import sys
from PyQt6 import QtCore, QtWidgets, QtGui

# Constants for the grid size and speed factor
GRID_SIZE = 61
SPEED_FACTOR = 1  # Adjust the speed factor as needed
DETECTION_RANGE = 30  # Detection range for the player

class PlayerController(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()

        self.resize(700, 700)
        self.setWindowTitle('2D GRID')

        self.player = {'x': 30, 'y': 30}
        self.obstacle = {'x': 30, 'y': 50}

        # Add a QLabel widget for displaying the grid
        self.gridLabel = QtWidgets.QLabel(self)
        self.gridLabel.setGeometry(50, 50, 600, 600)
        self.updateGrid(self.player['x'], self.player['y'])

        # Timer for continuous movement
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.movePlayer)
        self.timer.start(100)  # Adjust the interval as needed

        self.activeKeys = set()

    def movePlayer(self):
        for key in self.activeKeys:
            if key == QtCore.Qt.Key.Key_W:
                self.updatePlayerPosition('y', -SPEED_FACTOR)
            if key == QtCore.Qt.Key.Key_S:
                self.updatePlayerPosition('y', SPEED_FACTOR)
            if key == QtCore.Qt.Key.Key_A:
                self.updatePlayerPosition('x', -SPEED_FACTOR)
            if key == QtCore.Qt.Key.Key_D:
                self.updatePlayerPosition('x', SPEED_FACTOR)
        self.detectObstacles()

    def updatePlayerPosition(self, k, v):
        new_x = self.player['x']
        new_y = self.player['y']
        
        if k == 'x':
            new_x = max(0, min(GRID_SIZE - 1, self.player['x'] + v))
        elif k == 'y':
            new_y = max(0, min(GRID_SIZE - 1, self.player['y'] + v))
        
        if (new_x, new_y) != (self.obstacle['x'], self.obstacle['y']):
            self.player['x'] = new_x
            self.player['y'] = new_y
        
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
            elif player_y > obstacle_y:
                print(f"Obstacle detected in back! Distance: {distance}")
        
        if player_y == obstacle_y and abs(player_x - obstacle_x) <= DETECTION_RANGE:
            distance = abs(player_x - obstacle_x)
            if player_x < obstacle_x:
                print(f"Obstacle detected to the right! Distance: {distance}")
            elif player_x > obstacle_x:
                print(f"Obstacle detected to the left! Distance: {distance}")

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

    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            self.activeKeys.add(event.key())

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            self.activeKeys.discard(event.key())

if __name__ == '__main__':
    appQt = QtWidgets.QApplication(sys.argv)
    mainWindow = PlayerController()
    mainWindow.show()
    sys.exit(appQt.exec())
