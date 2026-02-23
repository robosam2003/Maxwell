import sys

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

import pyqtgraph as pg
from typing import List
import numpy as np

import maxwellstudio_ui
import serial
from dataclasses import dataclass

import can
import time
import struct

BUFFER_SIZE = 200

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
    6: 'Bus Voltage',
    7: 'Command Voltages',
    8: 'Command',
}

com_type = "SERIAL"



@dataclass
class Frame:
    name: str
    sample_buffer:   np.ndarray  # Circular buffer for time samples
    data_names:     List[str]
    data_buffers:    List[np.ndarray]  # List of circular buffers for each data stream
    index: int  # Current write position in circular buffer
    count: int  # Total number of samples written (capped at BUFFER_SIZE)

    def __init__(self, name: str, num_channels: int):
        self.name = name
        self.sample_buffer = np.zeros(BUFFER_SIZE, dtype=np.float32)
        self.data_buffers = [np.zeros(BUFFER_SIZE, dtype=np.float32) for _ in range(num_channels)]
        self.data_names = [''] * num_channels
        self.index = 0
        self.count = 0

    def append(self, timestamp: float, data: List[float]):
        """Append data to circular buffers - much faster than deque"""
        self.sample_buffer[self.index] = timestamp
        for i, value in enumerate(data):
            self.data_buffers[i][self.index] = value
        self.index = (self.index + 1) % BUFFER_SIZE
        self.count = min(self.count + 1, BUFFER_SIZE)

    def get_plot_data(self):
        """Return data in correct order for plotting"""
        if self.count < BUFFER_SIZE:
            # Buffer not full yet, return data from start
            return self.sample_buffer[:self.count], [buf[:self.count] for buf in self.data_buffers]
        else:
            # Buffer is full, need to reorder: [index:end] + [0:index]
            time_data = np.concatenate([self.sample_buffer[self.index:], self.sample_buffer[:self.index]])
            data_streams = [np.concatenate([buf[self.index:], buf[:self.index]]) for buf in self.data_buffers]
            return time_data, data_streams


# Serial worker that runs in its own thread and emits parsed packets
class SerialWorker(QObject):
    packet_received = Signal(str, list)   # name, list_of_floats
    error = Signal(str)

    def __init__(self, port: str = 'COM3', baud: int = 921600, timeout: float = 1.0):
        super().__init__()
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._running = False
        self.ser = None
        self.bus = None

    @Slot()
    def start(self):
        self._running = True
        try:
            if com_type == "SERIAL":
                self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            elif com_type == "CAN":
                self.bus = can.interface.Bus(interface="slcan", channel="COM4", bitrate=500000)
        except Exception as e:
            self.error.emit(f"Serial open error: {e}")
            # keep trying to open
            while self._running:
                try:
                    if com_type == "SERIAL":
                        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                    elif com_type == "CAN":
                        self.bus = can.interface.Bus(interface="slcan", channel="COM4", bitrate=500000)
                    break
                except Exception as e2:
                    self.error.emit(f"Retry open failed: {e2}")
                    time.sleep(1)

        while self._running:
            try:
                uint8_raw_values = []
                packet_type = 0
                if com_type == "SERIAL":
                    line = self.ser.readline()
                    if not line:
                        continue
                    uint8_raw_values = [b for b in line.strip()]
                    packet_type = uint8_raw_values[0]
                    checksum_packet = uint8_raw_values[-1]
                    checksum_calc = 0
                    for b in uint8_raw_values[:-1]:
                        checksum_calc ^= b
                    if checksum_packet != checksum_calc:
                        self.error.emit(f"Checksum error: packet {checksum_packet} != calc {checksum_calc}")
                        continue
                elif com_type == "CAN":
                    message = self.bus.recv(timeout=1)
                    if message is None:
                        continue
                    uint8_raw_values = [b for b in message.data] + [0] # Pad to avoid index error if data is empty
                    packet_type = uint8_raw_values[0]
                float_values = []
                for b in range(len(uint8_raw_values[1:-1]) // 4):
                    float_bytes = bytes(uint8_raw_values[1 + b*4: 1 + (b+1)*4])
                    float_values.append(struct.unpack('f', float_bytes)[0])

                name = TELEMETRY_PACKET_TYPE.get(packet_type)
                self.packet_received.emit(name, float_values)
            except serial.SerialException as e:
                self.error.emit(f"Serial error: {e}")
                # try to reconnect without blocking GUI thread
                try:
                    if self.ser:
                        self.ser.close()
                except:
                    pass
                time.sleep(1)
                while self._running:
                    try:
                        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                        break
                    except Exception as e2:
                        self.error.emit(f"Reconnect failed: {e2}")
                        time.sleep(1)
            except Exception as e:
                self.error.emit(f"Unexpected worker error: {e}")
                # time.sleep(0.1)

    @Slot()
    def stop(self):
        self._running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except:
            pass


class MaxwellStudio(maxwellstudio_ui.Ui_MainWindow, QMainWindow):
    reset_signal = Signal()

    def __init__(self, parent=None):
        super(MaxwellStudio, self).__init__(parent)
        self.setupUi(self)

        # self.ser = serial.Serial('COM6', 921600)

        self.frames = []
        self.plots = [self.plot1]
        self.comboboxes = [self.plot1_combobox]
        self.selected_frames = [None]

        # Store persistent plot curves to avoid recreating them every frame
        self.plot_curves = [[] for _ in range(1)]  # List of lists of PlotDataItem objects
        self.plot_legends = [None]  # Store legend widgets

        for j, combobox in enumerate(self.comboboxes):
            combobox.addItem('')

        # Connect comboboxes to the function
        for i, combobox in enumerate(self.comboboxes):
            combobox.currentIndexChanged.connect(lambda x, index=i: self.combobox_changed(index))

        self.start_time = time.time()
        self.setup_graphs()

        self.reset_signal.connect(self.reset)

        # Start SerialWorker in its own QThread
        self.reader_thread = QThread()
        self.reader = SerialWorker(port='COM3', baud=921600, timeout=1.0)
        self.reader.moveToThread(self.reader_thread)
        self.reader.packet_received.connect(self.handle_packet)
        self.reader.error.connect(self.handle_worker_error)
        self.reader_thread.started.connect(self.reader.start)
        self.reader_thread.start()

        self.update_plots_timer = QTimer()
        self.update_plots_timer.timeout.connect(self.update_plots)
        self.update_plots_timer.start(1) # Update every 10 ms


    @Slot(str, list)
    def handle_packet(self, name: str, data_list: list):
        if name == "GENERAL":
            self.outputLabel.setText(f"GENERAL: {data_list}")
        # Called in the main (GUI) thread via signal
        data = [float(x) for x in data_list]
        timestamp = time.time() - self.start_time

        for frame in self.frames:
            if frame.name == name:
                frame.append(timestamp, data)
                break
        else:
            new_frame = Frame(name, len(data))
            new_frame.append(timestamp, data)
            self.frames.append(new_frame)
            for combobox in self.comboboxes:
                combobox.addItem(name)

    @Slot(str)
    def handle_worker_error(self, msg: str):
        print(f"Worker: {msg}")

    def combobox_changed(self, i):
        # Get the selected frame
        selected_frame_name = self.comboboxes[i].currentText()
        if selected_frame_name == '':
            self.selected_frames[i] = None
            self.plots[i].clear()
            self.plot_curves[i] = []
            self.plot_legends[i] = None  # Just set to None, clear() already removed it
            self.plots[i].setTitle(f'Plot {i+1}')
            return

        # Find the frame in the list
        for frame in self.frames:
            if frame.name == selected_frame_name:
                self.selected_frames[i] = frame
                print(f'Selected frame: {frame.name}')

                # Clear old curves and legend
                self.plots[i].clear()
                self.plot_curves[i] = []
                self.plot_legends[i] = None  # clear() already removes all items including legend

                # Create persistent PlotDataItem objects for each data stream
                for j in range(len(frame.data_buffers)):
                    # Use a default name if data_names is empty
                    label = frame.data_names[j] if frame.data_names[j] else f'Channel {j}'
                    curve = self.plots[i].plot([], [], name=label, pen=pens[j % len(pens)])
                    self.plot_curves[i].append(curve)

                # Add legend once (only if we have curves)
                if len(self.plot_curves[i]) > 0:
                    self.plot_legends[i] = self.plots[i].addLegend(offset=(0, 0))

                self.plots[i].setTitle(f'Plot {i+1} - {frame.name}')
                break

    def setup_graphs(self):
        for i, plot in enumerate(self.plots):
            plot.setTitle(f'Plot {i+1}')
            plot.setLabel('left', 'Value', " ")
            plot.setLabel('bottom', 'Time', "s")
            plot.showGrid(x=True, y=True)

    @Slot()
    def reset(self):
        # Clear all frames
        self.frames = []
        self.plots = [self.plot1, self.plot2, self.plot3, self.plot4]
        self.comboboxes = [self.plot1_combobox, self.plot2_combobox, self.plot3_combobox, self.plot4_combobox]
        self.selected_frames = [None, None, None, None]
        self.plot_curves = [[] for _ in range(4)]
        self.plot_legends = [None, None, None, None]

        for j, combobox in enumerate(self.comboboxes):
            combobox.clear()
            combobox.addItem('')

        # Connect comboboxes to the function
        for i, combobox in enumerate(self.comboboxes):
            combobox.currentIndexChanged.connect(lambda x, index=i: self.combobox_changed(index))

        # clear buffers and reset start time
        self.start_time = time.time()


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
        """Update plot data without recreating plot items - MUCH faster!"""
        for i, frame in enumerate(self.selected_frames):
            if frame is not None and len(self.plot_curves[i]) > 0:
                time_data, data_streams = frame.get_plot_data()

                # Just update the data in existing curves - no clear() or recreate needed!
                for j, data_stream in enumerate(data_streams):
                    if j < len(self.plot_curves[i]):
                        self.plot_curves[i][j].setData(time_data, data_stream)

def main():
    app = QApplication(sys.argv)
    window = MaxwellStudio()
    window.showMaximized()
    sys.exit(app.exec())

main()
