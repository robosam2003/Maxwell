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
    rotor_sector:        float  = 0.0
    electrical_velocity: float  = 0.0
    mechanical_velocity: float  = 0.0
    current_a:           float  = 0.0
    current_b:           float  = 0.0
    current_c:           float  = 0.0
    d_current:           float  = 0.0
    q_current:           float  = 0.0
    alpha_current:       float  = 0.0
    beta_current:        float  = 0.0
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



class MaxwellStudio(maxwellstudio_ui.Ui_MainWindow, QMainWindow):
    def __init__(self, parent=None):
        super(MaxwellStudio, self).__init__(parent)
        self.setupUi(self)

        self.ser = serial.Serial('/dev/MAXWELL', 921600)

        self.state = State()
        self.start_time = time.time()
        self.setup_graphs()
        self.sample_no = 0


        BUFFER_SIZE = 1000
        self.sample_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.current_a_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.current_b_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.current_c_circular_buffer = deque(maxlen=BUFFER_SIZE)

        self.d_current_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.q_current_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.alpha_current_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.beta_current_circular_buffer = deque(maxlen=BUFFER_SIZE)

        self.rotor_angle_circular_buffer = deque(maxlen=BUFFER_SIZE)

        self.voltage_a_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.voltage_b_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.voltage_c_circular_buffer = deque(maxlen=BUFFER_SIZE)
        self.input_voltage_circular_buffer = deque(maxlen=BUFFER_SIZE)

        self.buffers = [self.sample_circular_buffer,
                        self.current_a_circular_buffer, self.current_b_circular_buffer, self.current_c_circular_buffer,
                        self.d_current_circular_buffer, self.q_current_circular_buffer,
                        self.alpha_current_circular_buffer, self.beta_current_circular_buffer,
                        self.rotor_angle_circular_buffer,
                        self.voltage_a_circular_buffer, self.voltage_b_circular_buffer, self.voltage_c_circular_buffer, self.input_voltage_circular_buffer]

        # A q thread
        self.update_thread = QThread()
        self.update_thread.run = self.update
        self.update_thread.start()

        self.update_plots_timer = QTimer()
        self.update_plots_timer.timeout.connect(self.update_plots)
        self.update_plots_timer.start(10)

    def setup_graphs(self):
        self.current_plots.setLabel('left', 'Current')
        self.current_plots.setLabel('bottom', 'Sample')
        self.current_plots.setTitle('Current Plot')
        self.current_plots.showGrid(x=True, y=True)

        # self.total_current_circular_buffer = deque(maxlen=BUFFER_SIZE)

        self.dq_currents_plot.setLabel('left', 'Current')
        self.dq_currents_plot.setLabel('bottom', 'Sample')
        self.dq_currents_plot.setTitle('DQ Currents Plot')
        self.dq_currents_plot.showGrid(x=True, y=True)



        self.rotor_angle_plot.setLabel('left', 'Angle', 'degrees')
        self.rotor_angle_plot.setLabel('bottom', 'Sample')
        self.rotor_angle_plot.setTitle('Rotor Angle Plot')
        self.rotor_angle_plot.showGrid(x=True, y=True)


        self.voltage_plots.setLabel('left', 'Voltage')
        self.voltage_plots.setLabel('bottom', 'Sample')
        self.voltage_plots.setTitle('Voltage Plot')
        self.voltage_plots.showGrid(x=True, y=True)

    def update(self):
        while True:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                print(line)
                data = line.split('/')
                if len(data) != 16:
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

                for buffer in self.buffers:
                    buffer.clear()
                self.sample_no = 0
                break
            except serial.SerialException as e:
                print(f'Failed to reconnect: {e}')
                time.sleep(1)

    def check_checksum(self, data):
        line_without_checksum = '/'.join(data[:-1]) + '/'
        checksum = data[-1]
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
        self.state.rotor_sector = float(data[0])
        # self.state.electrical_velocity = float(data[1])
        self.state.current_a        = float(data[1])
        self.state.current_b        = float(data[2])
        self.state.current_c        = float(data[3])
        self.state.d_current        = float(data[4])
        self.state.q_current        = float(data[5])
        self.state.alpha_current    = float(data[6])
        self.state.beta_current     = float(data[7])
        self.state.pwm_duty         = float(data[8])
        self.state.voltage_a        = float(data[9])
        self.state.voltage_b        = float(data[10])
        self.state.voltage_c        = float(data[11])
        self.state.input_voltage    = float(data[12])

        self.state.fault_1 = data[13]
        self.state.fault_2 = data[14]

        self.fault_codes_label.setText(f'Fault 1: {self.state.fault_1}\nFault 2: {self.state.fault_2}')

        self.sample_circular_buffer.append(self.sample_no); self.sample_no += 1
        self.rotor_angle_circular_buffer.append(self.state.rotor_sector)

        self.current_a_circular_buffer.append(self.state.current_a)
        self.current_b_circular_buffer.append(self.state.current_b)
        self.current_c_circular_buffer.append(self.state.current_c)
        self.d_current_circular_buffer.append(self.state.d_current)
        self.q_current_circular_buffer.append(self.state.q_current)
        self.alpha_current_circular_buffer.append(self.state.alpha_current)
        self.beta_current_circular_buffer.append(self.state.beta_current)


        self.voltage_a_circular_buffer.append(self.state.voltage_a)
        self.voltage_b_circular_buffer.append(self.state.voltage_b)
        self.voltage_c_circular_buffer.append(self.state.voltage_c)
        self.input_voltage_circular_buffer.append(self.state.input_voltage)

    def update_plots(self):
        self.current_plots.plot(self.sample_circular_buffer, self.current_a_circular_buffer,        pen=pg.mkPen(color=(255, 255, 0), width=2), clear=True, name='A Current')
        self.current_plots.plot(self.sample_circular_buffer, self.current_b_circular_buffer,        pen=pg.mkPen(color=(255, 0, 255), width=2), clear=False, name='B Current')
        self.current_plots.plot(self.sample_circular_buffer, self.current_c_circular_buffer,        pen=pg.mkPen(color=(0, 0, 255), width=2),   clear=False, name='C Current')
        # self.current_plots.plot(self.sample_circular_buffer, self.total_current_circular_buffer,    pen=pg.mkPen(color=(255, 0, 0), width=2),   clear=False, name='Total Current')

        self.dq_currents_plot.plot(self.sample_circular_buffer, self.d_current_circular_buffer,        pen=pg.mkPen(color=(255, 255, 0), width=2), clear=True, name='D Current')
        self.dq_currents_plot.plot(self.sample_circular_buffer, self.q_current_circular_buffer,        pen=pg.mkPen(color=(255, 0, 255), width=2), clear=False, name='Q Current')
        # self.dq_currents_plot.plot(self.sample_circular_buffer, self.alpha_current_circular_buffer,    pen=pg.mkPen(color=(0, 0, 255), width=2),   clear=False, name='Alpha Current')
        # self.dq_currents_plot.plot(self.sample_circular_buffer, self.beta_current_circular_buffer,     pen=pg.mkPen(color=(255, 0, 0), width=2),   clear=False, name='Beta Current')


        self.voltage_plots.plot(self.sample_circular_buffer, self.voltage_a_circular_buffer,        pen=pg.mkPen(color=(255, 255, 0), width=2), clear=True,  name='Voltage A')
        self.voltage_plots.plot(self.sample_circular_buffer, self.voltage_b_circular_buffer,        pen=pg.mkPen(color=(255, 0, 255), width=2), clear=False, name='Voltage B')
        self.voltage_plots.plot(self.sample_circular_buffer, self.voltage_c_circular_buffer,        pen=pg.mkPen(color=(0, 0, 255), width=2),   clear=False, name='Voltage C')
        self.voltage_plots.plot(self.sample_circular_buffer, self.input_voltage_circular_buffer,    pen=pg.mkPen(color=(255, 0, 0), width=2),   clear=False, name='Input Voltage')

        self.rotor_angle_plot.plot(self.sample_circular_buffer, self.rotor_angle_circular_buffer,    pen=pg.mkPen(color=(255, 0, 0), width=2),   clear=True, name='Rotor Angle')


        # Legends
        self.current_plots.addLegend(offset=(0, 0))
        self.voltage_plots.addLegend(offset=(0, 0))
        self.dq_currents_plot.addLegend(offset=(0, 0))
        self.rotor_angle_plot.addLegend(offset=(0, 0))






def main():
    app = QApplication(sys.argv)
    window = MaxwellStudio()
    window.show()
    sys.exit(app.exec_())

main()
