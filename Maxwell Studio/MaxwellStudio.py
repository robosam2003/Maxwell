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
from typing import List

import maxwellstudio_ui
import serial
import struct
import time
from dataclasses import dataclass

BUFFER_SIZE = 500

# List of Pens
pens = [
    pg.mkPen('r', width=2),
    pg.mkPen('g', width=2),
    pg.mkPen('b', width=2),
    pg.mkPen('c', width=2),
    pg.mkPen('m', width=2),
    pg.mkPen('y', width=2),
    pg.mkPen('w', width=2)
    ]


TELEMETRY_PACKET_TYPE = {
    0: 'GENERAL',
    1: 'Rotor Position',
    2: 'Rotor Velocity',
    3: 'Phase Currents',
    4: 'A-B Currents',
    5: 'DQ Currents',
    6: 'Bus Voltage'
}




@dataclass
class Frame:
    name: str
    sample_queue:   deque[float]
    data_names:     List[str]
    data_queues:    List[deque[float]]

    def __init__(self, name: str, data: List[deque[float]]):
        self.name = name
        self.sample_queue = deque(maxlen=BUFFER_SIZE)
        self.data_queues = data
        self.data_names = [''] * len(data)



class MaxwellStudio(maxwellstudio_ui.Ui_MainWindow, QMainWindow):
    reset_signal = Signal()

    def __init__(self, parent=None):
        super(MaxwellStudio, self).__init__(parent)
        self.setupUi(self)

        self.ser = serial.Serial('COM6', 921600)

        self.frames = []
        self.plots = [self.plot1, self.plot2, self.plot3, self.plot4]
        self.comboboxes = [self.plot1_combobox, self.plot2_combobox, self.plot3_combobox, self.plot4_combobox]
        self.selected_frames = [None, None, None, None]

        for j, combobox in enumerate(self.comboboxes):
            combobox.addItem('')

        # Connect comboboxes to the function
        for i, combobox in enumerate(self.comboboxes):
            combobox.currentIndexChanged.connect(lambda x, index=i: self.combobox_changed(index))

        self.start_time = time.time()
        self.setup_graphs()

        self.reset_signal.connect(self.reset)

        # A q thread
        self.update_thread = QThread()
        self.update_thread.run = self.update
        self.update_thread.start()

        self.update_plots_timer = QTimer()
        self.update_plots_timer.timeout.connect(self.update_plots)
        self.update_plots_timer.start(1)

    def combobox_changed(self, i):
        # Get the selected frame
        selected_frame_name = self.comboboxes[i].currentText()
        if selected_frame_name == '':
            self.selected_frames[i] = None
            self.plots[i].clear()
            self.plots[i].setTitle(f'Plot {i+1}')
            return

        # Find the frame in the list
        for frame in self.frames:
            if frame.name == selected_frame_name:
                self.selected_frames[i] = frame
                print(f'Selected frame: {frame.name}')
                break

    def setup_graphs(self):
        for i, plot in enumerate(self.plots):
            plot.setTitle(f'Plot {i+1}')
            plot.setLabel('left', 'Value')
            plot.setLabel('bottom', 'Sample')
            plot.showGrid(x=True, y=True)

    @Slot()
    def reset(self):
        for frame in self.frames:
            frame.sample_queue.clear()
            for data_queue in frame.data_queues:
                data_queue.clear()

        self.frames = []
        self.plots = [self.plot1, self.plot2, self.plot3, self.plot4]
        self.comboboxes = [self.plot1_combobox, self.plot2_combobox, self.plot3_combobox, self.plot4_combobox]
        self.selected_frames = [None, None, None, None]

        for j, combobox in enumerate(self.comboboxes):
            combobox.clear()
            combobox.addItem('')

        # Connect comboboxes to the function
        for i, combobox in enumerate(self.comboboxes):
            combobox.currentIndexChanged.connect(lambda x, index=i: self.combobox_changed(index))

        # clear buffers
        self.start_time = time.time()

    def update(self):
        while True:
            QApplication.processEvents()
            try:
                line = self.ser.readline()
                # Print line in hex - bytearray
                # print(line.strip().hex())
                # interpret as uint8 values
                uint8_raw_values = [b for b in line.strip()]
                packet_type = uint8_raw_values[0]
                float_values = []
                for b in range(len(uint8_raw_values[1:])//4):
                    float_bytes = bytes(uint8_raw_values[1 + b*4: 1 + (b+1)*4])
                    float_values.append(struct.unpack('f', float_bytes)[0])
                name = TELEMETRY_PACKET_TYPE.get(packet_type, f'UNKNOWN_{packet_type}')

                # Set data names
                # if name.startswith('NAMESET'):
                #     name = name[7:]
                #     for frame in self.frames:
                #         if frame.name == name: # Frame already exists
                #             frame.data_names = data # Set new data names
                #             break
                #     else: # New frame
                #         data_names = data
                #         data = [deque(maxlen=BUFFER_SIZE) for _ in range(len(data_names))]
                #         self.frames.append(Frame(name, data))
                #         for combobox in self.comboboxes:
                #             combobox.addItem(name)

                data = [float(x) for x in float_values]
                for frame in self.frames:
                    if frame.name == name:
                        # Increment sample number
                        frame.sample_queue.append(time.time() - self.start_time)
                        for i, d in enumerate(data):
                            frame.data_queues[i].append(d)
                        break
                else: # New frame
                    new_frame = Frame(name, [deque(maxlen=BUFFER_SIZE) for _ in range(len(data))])
                    new_frame.sample_queue.append(time.time() - self.start_time)
                    for i, d in enumerate(data):
                        new_frame.data_queues[i].append(d)
                    self.frames.append(new_frame)
                    for combobox in self.comboboxes:
                        combobox.addItem(name)


            # Exception Handling
            except serial.SerialException as e:
                print(f'Serial error: {e}')
                self.reconnect_serial()
            except Exception as e:
                print(f'Unexpected error: {e}')
            # print(self.frames)

    def reconnect_serial(self):
        while True:
            try:
                self.ser.close()
                self.ser.open()
                print('Reconnected to serial port')

                # send Qsignal to reset
                self.reset_signal.emit()
                break
            except serial.SerialException as e:
                print(f'Failed to reconnect: {e}')
                time.sleep(1)


    def check_checksum(self, line):
        split = line.split('|')
        line_without_checksum = split[0]
        checksum = split[1]
        checksum2 = 0
        for char in line_without_checksum:
            checksum2 += ord(char)
        checksum2 = checksum2 % 256
        if checksum != str(checksum2):
            print(f'Checksum Error: {checksum} != {checksum2}')
            return False
        return True


    def update_plots(self):
        for i, frame in enumerate(self.selected_frames):
            if frame is not None:
                self.plots[i].clear()
                for j, data_stream in enumerate(frame.data_queues):
                    self.plots[i].plot(frame.sample_queue, data_stream, clear=False, name=frame.data_names[j], pen=pens[j%len(pens)])
                self.plots[i].setTitle(f'Plot {i+1} - {frame.name}')
                self.plots[i].addLegend(offset=(0, 0))









def main():
    app = QApplication(sys.argv)
    window = MaxwellStudio()
    window.showMaximized()
    sys.exit(app.exec())

main()
