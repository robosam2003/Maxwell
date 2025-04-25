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



@dataclass
class Frame:
    name: str
    sample_queue: deque[int]
    data_names: list[str]
    data_queues: list[deque[float]]

    def __init__(self, name: str, data: list[deque[float]]):
        self.name = name
        self.sample_queue = deque(maxlen=BUFFER_SIZE)
        self.data_queues = data
        self.data_names = [''] * len(data)



class MaxwellStudio(maxwellstudio_ui.Ui_MainWindow, QMainWindow):
    reset_signal = Signal()

    def __init__(self, parent=None):
        super(MaxwellStudio, self).__init__(parent)
        self.setupUi(self)

        self.ser = serial.Serial('/dev/MAXWELL', 921600)

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
        # self.update_serial_timer = QTimer()
        # self.update_serial_timer.timeout.connect(self.update)
        # self.update_serial_timer.start(10)

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

        self.start_time = time.time()

    def update(self):
        while True:
            try:
                # Example line: Measured Current/1.00/2.00/3.10/|178
                line = self.ser.readline().decode('utf-8').strip()
                # print(line)
                split1 = line.split('|')
                if len(split1) != 2:
                    print(f'Invalid line: {line}')
                    continue
                if not self.check_checksum(line):
                    continue
                # Split into name and data
                split2 = split1[0].split('/')[:-1]
                name = split2[0]
                data = split2[1:]

                # Set data names
                if name.startswith('NAMESET'):
                    name = name[7:]
                    for frame in self.frames:
                        if frame.name == name: # Frame already exists
                            frame.data_names = data # Set new data names
                            break
                    else: # New frame
                        data_names = data
                        data = [deque(maxlen=BUFFER_SIZE) for _ in range(len(data_names))]
                        self.frames.append(Frame(name, data))
                        for combobox in self.comboboxes:
                            combobox.addItem(name)
                else: # regular frame
                    data = [float(x) for x in data]
                    for frame in self.frames:
                        if frame.name == name:
                            # Increment sample number
                            frame.sample_queue.append(frame.sample_queue[-1] + 1)
                            for i, d in enumerate(data):
                                frame.data_queues[i].append(d)
                            break
                    else: # New frame
                        new_frame = Frame(name, [deque(maxlen=BUFFER_SIZE) for _ in range(len(data))])
                        new_frame.sample_queue.append(1)
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
    window.show()
    sys.exit(app.exec_())

main()
