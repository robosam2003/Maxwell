import sys
from collections import deque

from PySide6.QtCore import QObject
from PySide6.QtCore import QThread
from PySide6.QtWidgets import QMainWindow
from PySide6.QtWidgets import QApplication
# signals
# slots
# connect
from PySide6.QtCore import QTimer
from PySide6.QtCore import Signal
from PySide6.QtCore import Slot
from PySide6.QtCore import Qt

import pyqtgraph as pg

import maxwellstudio_ui
import serial
import time
from dataclasses import dataclass


@dataclass
class State:
    time:                float  = 0.0
    rotor_sector:        int    = 0
    electrical_velocity: float  = 0.0
    mechanical_velocity: float  = 0.0
    current_a:           float  = 0.0
    current_b:           float  = 0.0
    current_c:           float  = 0.0
    total_current:       float  = 0.0
    voltage_a:           float  = 0.0
    voltage_b:           float  = 0.0
    voltage_c:           float  = 0.0
    input_voltage:       float  = 0.0
    temperature:         float  = 0.0
    pwm_duty:            float  = 0.0
    pid_output:          float  = 0.0
    fault_1:             str    = ''
    fault_2:             str    = ''


class UpdatePlotsSignalEmmitter(QObject):
    signal = Signal()

    def __init__(self):
        super(UpdatePlotsSignalEmmitter, self).__init__()

    # def emit(self, *args, **kwargs):
    #     self.signal.emit()

class MaxwellStudio(maxwellstudio_ui.Ui_MainWindow, QMainWindow):
    def __init__(self, parent=None):
        super(MaxwellStudio, self).__init__(parent)
        self.setupUi(self)

        self.ser = serial.Serial('/dev/MAXWELL', 115200)

        self.state = State()
        self.start_time = time.time()
        self.setup_graphs()
        self.sample_no = 0

        self.update_plots_signal = UpdatePlotsSignalEmmitter()
        self.update_plots_signal.signal.connect(self.update_plots)
        # A q thread
        self.update_thread = QThread()
        self.update_thread.run = self.update
        self.update_thread.start()

        self.update_plots_timer = QTimer()
        self.update_plots_timer.timeout.connect(self.update_plots)
        self.update_plots_timer.start(1)

    def setup_graphs(self):
        BUFFER_SIZE = 1000
        self.current_plots.setLabel('left', 'Current')
        self.current_plots.setLabel('bottom', 'Sample')
        self.current_plots.setTitle('Current Plot')
        self.current_plots.showGrid(x=True, y=True)
        self.sample_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.current_a_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.current_b_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.current_c_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.total_current_circular_buffer = deque(maxlen=BUFFER_SIZE)


        self.voltage_plots.setLabel('left', 'Voltage', 'V')
        self.voltage_plots.setLabel('bottom', 'Sample')
        self.voltage_plots.setTitle('Voltage Plot')
        self.voltage_plots.showGrid(x=True, y=True)
        self.voltage_a_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.voltage_b_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.voltage_c_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.input_voltage_circular_buffer = deque(maxlen=BUFFER_SIZE)

    def update(self):
        while True:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                print(line)
                data = line.split('/')
                if len(data) != 11:
                    print(f'Invalid data: {data}')
                    continue
                if not self.check_checksum(data):
                    continue

                self.update_state(data)
            except serial.SerialException as e:
                print(f'Serial error: {e}')
                self.reconnect_serial()
            except Exception as e:
                print(f'Unexpected error: {e}')

    def reconnect_serial(self):
        while True:
            try:
                self.ser.close()
                self.ser.open()
                print('Reconnected to serial port')
                self.sample_circular_buffer.clear(); self.sample_no = 0
                self.current_a_circular_buffer.clear(); self.current_b_circular_buffer.clear(); self.current_c_circular_buffer.clear(); self.total_current_circular_buffer.clear()
                self.voltage_a_circular_buffer.clear(); self.voltage_b_circular_buffer.clear(); self.voltage_c_circular_buffer.clear(); self.input_voltage_circular_buffer.clear()
                break
            except serial.SerialException as e:
                print(f'Failed to reconnect: {e}')
                time.sleep(1)

    def check_checksum(self, data):
        line_without_checksum = '/'.join(data[:-1]) + '/'
        checksum = data[10]
        checksum2 = 0
        for char in line_without_checksum:
            checksum2 += ord(char)
        checksum2 = checksum2 % 256
        if checksum != str(checksum2):
            print(f'Checksum Error: {checksum} != {checksum2}')
            return False
        return True

    def update_state(self, data):
        self.state.time = time.time() - self.start_time
        # self.state.rotor_sector = int(data[0])
        # self.state.electrical_velocity = float(data[1])
        self.state.current_a = float(data[2])
        self.state.current_b = float(data[3])
        self.state.current_c = float(data[4])
        self.state.total_current = float(data[5])
        # self.state.pwm_duty = float(data[6])
        # self.state.pid_output = float(data[7])

        # self.state.fault_1 = data[8]
        # self.state.fault_2 = data[9]

        self.fault_codes_label.setText(f'Fault 1: {self.state.fault_1}\nFault 2: {self.state.fault_2}')

        self.sample_circular_buffer.append(self.sample_no); self.sample_no += 1
        self.current_a_circular_buffer.append(self.state.current_a)
        self.current_b_circular_buffer.append(self.state.current_b)
        self.current_c_circular_buffer.append(self.state.current_c)
        self.total_current_circular_buffer.append(self.state.total_current)

        self.voltage_a_circular_buffer.append(self.state.voltage_a)
        self.voltage_b_circular_buffer.append(self.state.voltage_b)
        self.voltage_c_circular_buffer.append(self.state.voltage_c)
        self.input_voltage_circular_buffer.append(self.state.input_voltage)

    def update_plots(self):
        self.current_plots.plot(self.sample_circular_buffer, self.current_a_circular_buffer,        pen=pg.mkPen(color=(255, 255, 0), width=2), clear=True, name='Current A')
        self.current_plots.plot(self.sample_circular_buffer, self.current_b_circular_buffer,        pen=pg.mkPen(color=(255, 0, 255), width=2), clear=False, name='Current B')
        self.current_plots.plot(self.sample_circular_buffer, self.current_c_circular_buffer,        pen=pg.mkPen(color=(0, 0, 255), width=2),   clear=False, name='Current C')
        # self.current_plots.plot(self.sample_circular_buffer, self.total_current_circular_buffer,    pen=pg.mkPen(color=(255, 0, 0), width=2),   clear=False, name='Total Current')

        self.voltage_plots.plot(self.sample_circular_buffer, self.voltage_a_circular_buffer,        pen=pg.mkPen(color=(255, 255, 0), width=2), clear=True, name='Voltage A')
        self.voltage_plots.plot(self.sample_circular_buffer, self.voltage_b_circular_buffer,        pen=pg.mkPen(color=(255, 0, 255), width=2), clear=False, name='Voltage B')
        self.voltage_plots.plot(self.sample_circular_buffer, self.voltage_c_circular_buffer,        pen=pg.mkPen(color=(0, 0, 255), width=2),   clear=False, name='Voltage C')
        # self.voltage_plots.plot(self.sample_circular_buffer, self.input_voltage_circular_buffer,    pen=pg.mkPen(color=(255, 0, 0), width=2),   clear=False, name='Input Voltage')

        # Legends
        self.current_plots.addLegend(offset=(0, 0))
        self.voltage_plots.addLegend(offset=(0, 0))






def main():
    app = QApplication(sys.argv)
    window = MaxwellStudio()
    window.show()
    sys.exit(app.exec_())

main()
