from PySide6.QtWidgets import QGraphicsView
from PySide6.QtGui import QPainter
from PySide6.QtCore import Qt


class RotorView(QGraphicsView):
    def __init__(self, parent=None):
        self.parent = parent
        super(RotorView, self).__init__()

        # self.painter = QPainter(self.viewport())
        # self.paintEvent(None)
        self.angle = 0

    def paintEvent(self, event):
        self.painter = QPainter(self.viewport())
        angle = self.angle % 360
        # draw the rotor
        self.painter.setPen(Qt.black)
        self.painter.setBrush(Qt.white)
        centre = [250, 250]
        self.painter.translate(centre[0], centre[1])
        self.painter.rotate(angle)
        vector_length = 150
        self.painter.drawLine(0, 0, vector_length, 0)


        self.painter.end()

    def set_angle(self, angle):
        self.angle = angle
        self.update()
        # force a repaint
        self.repaint()










