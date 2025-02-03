import sys

from PySide6.QtWidgets import QMainWindow
from PySide6.QtWidgets import QApplication
import pyqtgraph as pg

import maxwellstudio_ui
import serial

class MaxwellStudio(maxwellstudio_ui.Ui_MainWindow, QMainWindow):
    def __init__(self, parent=None):
        super(MaxwellStudio, self).__init__(parent)
        self.setupUi(self)

        self.ser = serial.Serial('/dev/MAXWELL', 9600)

        # a qt timer
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(50)

    def update(self):
        # read the serial port
        line = self.ser.readline()
        line = line.decode('utf-8').strip()
        angle = float(line)
        # print(angle)
        self.rotorview.set_angle(angle)
        # self.rotorview.paintEvent(None)







def main():
    app = QApplication(sys.argv)
    window = MaxwellStudio()
    window.show()
    sys.exit(app.exec_())

main()
