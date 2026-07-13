import sys
import time
import struct
import queue
import signal
import importlib
import threading
from collections import deque
# pyrefly: ignore [missing-import]
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QPushButton, QLabel, QLineEdit, QComboBox, QGroupBox, QGridLayout, QMessageBox, QRadioButton, QButtonGroup)
# pyrefly: ignore [missing-import]
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QTimer, Qt
# pyrefly: ignore [missing-import]
import pyqtgraph as pg
import serial
import math
import os
import glob
import csv
from motors import MOTORS

try:
    PositionControlLoop = importlib.import_module("ak80-64_pos").PositionControlLoop
    VelocityControlLoop = importlib.import_module("ak80-64_vel").VelocityControlLoop
    CurrentControlLoop = importlib.import_module("ak80-64_current").CurrentControlLoop
except ImportError as e:
    print(f"Failed to import control loops: {e}")


# ==========================================
# SINUSOIDAL POSITION TRAJECTORY THREAD
# ==========================================
class SinusoidalPositionLoop(threading.Thread):
    """
    Generates a live sinusoidal position trajectory:
      target = offset + a*sin(2*pi*f*t) + b*cos(2*pi*f*t)
    Sends pos_direct commands to MotorThread at ~100 Hz.
    """
    DT = 0.01  # 100 Hz

    def __init__(self, send_cmd_func, a, b, freq, offset_deg, mode):
        """
        mode: 'sin'     -> offset + a*sin(...)
              'cos'     -> offset + b*cos(...)
              'sin+cos' -> offset + a*sin(...) + b*cos(...)
        """
        super().__init__()
        self.send_cmd_func = send_cmd_func
        self.a = a
        self.b = b
        self.freq = freq
        self.offset_deg = offset_deg
        self.mode = mode
        self.running = False
        self.daemon = True

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        t0 = time.perf_counter()
        print(f"\nSinusoidal position loop started: mode={self.mode}, a={self.a}, b={self.b}, "
              f"f={self.freq} Hz, offset={self.offset_deg} deg. Press Terminate to stop.")
        try:
            while self.running:
                loop_start = time.perf_counter()
                t = loop_start - t0
                omega_t = 2.0 * math.pi * self.freq * t

                if self.mode == 'sin':
                    target = self.offset_deg + self.a * math.sin(omega_t)
                elif self.mode == 'cos':
                    target = self.offset_deg + self.b * math.cos(omega_t)
                else:  # sin+cos
                    target = self.offset_deg + self.a * math.sin(omega_t) + self.b * math.cos(omega_t)

                self.send_cmd_func({'type': 'pos_direct', 'val': target})

                elapsed = time.perf_counter() - loop_start
                sleep_t = self.DT - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        except KeyboardInterrupt:
            pass
        finally:
            self.send_cmd_func({'type': 'stop'})
            print("\nSinusoidal loop stopped.")


# ==========================================
# 1. HARDWARE CONSTANTS & PROTOCOL
# ==========================================
DEFAULT_PORT = 'COM6'
BAUD_RATE = 921600
FRAME_HEAD = 0x02
FRAME_TAIL = 0x03

COMM_GET_VALUES = 4
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8
COMM_SET_POS = 9
COMM_SET_POS_SPD = 91
COMM_SET_POS_ORIGIN = 95

GEAR_RATIO = 64.0
TARGET_DT = 0.01  # 100 Hz Loop

def crc16_xmodem(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000: crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else: crc = (crc << 1) & 0xFFFF
    return crc

def build_frame(payload: bytes) -> bytes:
    crc = crc16_xmodem(payload)
    frame = bytearray([FRAME_HEAD, len(payload)])
    frame.extend(payload)
    frame.extend([(crc >> 8) & 0xFF, crc & 0xFF, FRAME_TAIL])
    return bytes(frame)

def parse_frame(raw: bytes):
    if len(raw) < 5 or raw[0] != FRAME_HEAD: return None
    length = raw[1]
    if len(raw) < length + 5: return None
    payload = raw[2:2 + length]
    crc_recv = (raw[2 + length] << 8) | raw[3 + length]
    if crc16_xmodem(payload) != crc_recv: return None
    return payload

# ==========================================
# 2. MOTOR BACKGROUND THREAD
# ==========================================
class LowPassFilter:
    """Simple Exponential Moving Average (EMA) Low Pass Filter"""
    def __init__(self, alpha):
        self.alpha = alpha
        self.value = None

    def update(self, new_value):
        if self.value is None:
            self.value = new_value
        else:
            self.value = self.alpha * new_value + (1.0 - self.alpha) * self.value
        return self.value


class MotorThread(QThread):
    telemetry_signal = pyqtSignal(dict)  # All telemetry data
    error_signal = pyqtSignal(str)
    connected_signal = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.port = DEFAULT_PORT
        self.baud_rate = BAUD_RATE
        self.gear_ratio = GEAR_RATIO
        self.running = False
        self.command_queue = queue.Queue()
        self.ser = None
        
        # Exponential Moving Average (EMA) Filters for telemetry data
        # alpha determines smoothing: lower alpha = more smoothing, higher alpha = faster response
        self.cur_filter = LowPassFilter(alpha=0.02)  # Very strong filtering for current
        self.erpm_filter = LowPassFilter(alpha=0.05) # Strong filtering for speed
        self.pos_filter = LowPassFilter(alpha=0.1)  # Stronger filtering for position
        self.current_target = 0.0
        self.current_target_type = None

    def connect_motor(self, port):
        self.port = port
        self.running = True
        self.start()

    def disconnect_motor(self):
        self.running = False
        self.wait()
        
    def send_cmd(self, cmd_dict):
        self.command_queue.put(cmd_dict)

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.01)
            time.sleep(0.2)
            self.connected_signal.emit(True)
        except Exception as e:
            self.error_signal.emit(f"Failed to connect: {e}")
            self.connected_signal.emit(False)
            return
        
        while self.running:
            start_loop = time.perf_counter()
            
            # 1. Process any UI commands
            while not self.command_queue.empty():
                try:
                    cmd = self.command_queue.get_nowait()
                    self._handle_command(cmd)
                except queue.Empty:
                    break

            # 2. Fetch Telemetry
            self._fetch_telemetry()
            
            # 3. Spin Wait for 100Hz loop
            elapsed = time.perf_counter() - start_loop
            if elapsed < TARGET_DT:
                time.sleep(TARGET_DT - elapsed)

        # Cleanup on exit
        if self.ser and self.ser.is_open:
            self._handle_command({'type': 'stop'})
            time.sleep(0.1)
            self.ser.close()
        self.connected_signal.emit(False)

    def _handle_command(self, cmd):
        if 'val' in cmd:
            self.current_target = cmd['val']
            self.current_target_type = cmd['type']
            
        if cmd['type'] == 'pos':
            # COMM_SET_POS_SPD (91) 13-byte to prevent overflow on 64:1 gear ratio
            target_rotor_deg = cmd['val'] * self.gear_ratio
            pos_int = int(target_rotor_deg * 1000.0)
            speed_erpm = 5000
            accel_erpm_s = 30000
            payload = bytes([COMM_SET_POS_SPD]) + struct.pack('>iii', pos_int, speed_erpm, accel_erpm_s)
            self.ser.write(build_frame(payload))
            
        elif cmd['type'] == 'pos_raw':
            target_rotor_deg = cmd['val']
            pos_int = int(target_rotor_deg * 1000.0)
            speed_erpm = 5000
            accel_erpm_s = 30000
            payload = bytes([COMM_SET_POS_SPD]) + struct.pack('>iii', pos_int, speed_erpm, accel_erpm_s)
            self.ser.write(build_frame(payload))
            
        elif cmd['type'] == 'vel':
            target_erpm = cmd['val']
            payload = bytes([COMM_SET_RPM]) + struct.pack('>i', int(target_erpm))
            self.ser.write(build_frame(payload))
            
        elif cmd['type'] == 'vel_raw':
            target_erpm = cmd['val']
            payload = bytes([COMM_SET_RPM]) + struct.pack('>i', int(target_erpm))
            self.ser.write(build_frame(payload))
            
        elif cmd['type'] == 'pos_direct':
            limb_deg = cmd['val']
            payload = bytes([COMM_SET_POS]) + struct.pack('>i', int(limb_deg * 1_000_000))
            self.ser.write(build_frame(payload))
            
        elif cmd['type'] == 'cur':
            amps = cmd['val']
            payload = bytes([COMM_SET_CURRENT]) + struct.pack('>i', int(amps * 1000))
            self.ser.write(build_frame(payload))
            
        elif cmd['type'] == 'origin':
            mode = cmd['val']
            payload = bytes([COMM_SET_POS_ORIGIN, mode])
            self.ser.write(build_frame(payload))
            
        elif cmd['type'] == 'stop':
            payload = bytes([COMM_SET_CURRENT]) + struct.pack('>i', 0)
            self.ser.write(build_frame(payload))

    def _fetch_telemetry(self):
        self.ser.reset_input_buffer()
        self.ser.write(build_frame(bytes([COMM_GET_VALUES])))
        
        head = self.ser.read(1)
        if not head or head[0] != FRAME_HEAD: return
        length_byte = self.ser.read(1)
        if not length_byte: return
        rest = self.ser.read(length_byte[0] + 3)
        
        reply = parse_frame(head + length_byte + rest)
        if reply and len(reply) >= 58:
            telemetry = {
                'temp_fet': struct.unpack('>h', reply[1:3])[0] / 10.0,
                'temp_motor': struct.unpack('>h', reply[3:5])[0] / 10.0,
                'motor_current': struct.unpack('>i', reply[5:9])[0] / 100.0,
                'current_in': struct.unpack('>i', reply[9:13])[0] / 100.0,
                'id': struct.unpack('>i', reply[13:17])[0] / 100.0,
                'iq': struct.unpack('>i', reply[17:21])[0] / 100.0,
                'duty_now': struct.unpack('>h', reply[21:23])[0] / 1000.0,
                'erpm': struct.unpack('>i', reply[23:27])[0],
                'v_in': struct.unpack('>h', reply[27:29])[0] / 10.0,
                'amp_hours': struct.unpack('>i', reply[29:33])[0] / 10000.0,
                'amp_hours_charged': struct.unpack('>i', reply[33:37])[0] / 10000.0,
                'watt_hours': struct.unpack('>i', reply[37:41])[0] / 10000.0,
                'watt_hours_charged': struct.unpack('>i', reply[41:45])[0] / 10000.0,
                'tachometer': struct.unpack('>i', reply[45:49])[0],
                'tachometer_abs': struct.unpack('>i', reply[49:53])[0],
                'fault_code': reply[53],
                'pos_deg': struct.unpack('>i', reply[54:58])[0] / 1_000_000.0,
            }

            # Apply low-pass filters for core UI variables
            telemetry['filtered_current'] = self.cur_filter.update(telemetry['motor_current'])
            telemetry['filtered_erpm'] = self.erpm_filter.update(telemetry['erpm'])
            telemetry['filtered_pos'] = self.pos_filter.update(telemetry['pos_deg'])
            
            self.telemetry_signal.emit(telemetry)


# ==========================================
# 3. MAIN GUI WINDOW
# ==========================================
class MotorControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AK80-64 High-Performance Motor Control")
        self.setGeometry(100, 100, 1200, 700)

        # Thread Setup
        self.motor_thread = MotorThread()
        self.motor_thread.connected_signal.connect(self.on_connected)
        self.motor_thread.telemetry_signal.connect(self.on_telemetry)
        self.motor_thread.error_signal.connect(self.show_error)

        # Data Logging — deque with maxlen auto-discards oldest entries in O(1)
        self.history_len = 500  # 5 seconds at 100Hz
        self.times = deque(maxlen=self.history_len)
        self.data_pos = deque(maxlen=self.history_len)
        self.data_cur = deque(maxlen=self.history_len)
        self.data_erpm = deque(maxlen=self.history_len)
        self.data_target = deque(maxlen=self.history_len)
        self.start_time = time.time()
        self.current_pos = 0.0
        self.active_loop = None
        self._csv_flush_counter = 0  # Batch CSV flushes

        # Setup CSV Logging
        if not os.path.exists("logs"):
            os.makedirs("logs")
            
        log_path = "logs/motor_telemetry.csv"
        file_exists = os.path.exists(log_path)
        
        self.session_num = 1
        if file_exists:
            try:
                with open(log_path, "r") as f:
                    for line in f:
                        if line.startswith("--- NEW SESSION"):
                            self.session_num += 1
            except Exception:
                pass
                
        self.log_file = open(log_path, "a")
        
        csv_headers = [
            "Time", "Session", "Target", "Filtered_Position", "Filtered_Current", "Filtered_ERPM",
            "Temp_FET", "Temp_Motor", "Motor_Current", "Current_In",
            "Id", "Iq", "Duty_Now", "ERPM", "V_in",
            "Amp_Hours", "Amp_Hours_Charged", "Watt_Hours", "Watt_Hours_Charged",
            "Tachometer", "Tachometer_Abs", "Fault_Code", "Raw_Pos_Deg"
        ]
        if not file_exists:
            self.log_file.write(",".join(csv_headers) + "\n")
            
        self.log_file.write(f"--- NEW SESSION {self.session_num} ---\n")
        self.csv_writer = csv.writer(self.log_file)

        self.setup_ui()
        self.kt = 0.136
        self.pole_pairs = 21
        self.on_motor_changed(self.motor_selector.currentText())

        # UI Update Timer (Running at 30 FPS to keep UI snappy)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(33)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)

        # Left Panel (Controls)
        left_panel = QVBoxLayout()
        main_layout.addLayout(left_panel, stretch=1)

        # --- Connection Group ---
        conn_group = QGroupBox("Connection")
        conn_layout = QGridLayout()

        # Motor Selection
        self.motor_selector = QComboBox()
        self.motor_selector.addItems(list(MOTORS.keys()))
        self.motor_selector.currentTextChanged.connect(self.on_motor_changed)

        self.port_input = QLineEdit(DEFAULT_PORT)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        
        self.stop_btn = QPushButton("EMERGENCY STOP (Zero Current)")
        self.stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 10px;")
        self.stop_btn.clicked.connect(self.cmd_stop)
        
        conn_layout.addWidget(QLabel("Motor:"), 0, 0)
        conn_layout.addWidget(self.motor_selector, 0, 1, 1, 2)
        conn_layout.addWidget(QLabel("COM Port:"), 1, 0)
        conn_layout.addWidget(self.port_input, 1, 1)
        conn_layout.addWidget(self.connect_btn, 1, 2)
        conn_layout.addWidget(self.stop_btn, 2, 0, 1, 3)
        conn_group.setLayout(conn_layout)
        left_panel.addWidget(conn_group)

        # --- Position Control Group ---
        pos_group = QGroupBox("Position Control")
        pos_layout = QGridLayout()

        # Mode selector
        pos_layout.addWidget(QLabel("Mode:"), 0, 0)
        self.pos_mode_combo = QComboBox()
        self.pos_mode_combo.addItems(["Point", "a·sin", "b·cos", "a·sin + b·cos"])
        self.pos_mode_combo.currentIndexChanged.connect(self._on_pos_mode_changed)
        pos_layout.addWidget(self.pos_mode_combo, 0, 1)

        # --- Point mode row ---
        self.pos_target_input = QLineEdit("0.0")
        self.pos_cw_btn = QRadioButton("Clockwise (CW)")
        self.pos_acw_btn = QRadioButton("Anti-Clockwise (ACW)")
        self.pos_cw_btn.setChecked(True)
        self.pos_target_label = QLabel("Target (deg):")
        pos_layout.addWidget(self.pos_target_label, 1, 0)
        pos_layout.addWidget(self.pos_target_input, 1, 1)
        pos_layout.addWidget(self.pos_cw_btn, 2, 0)
        pos_layout.addWidget(self.pos_acw_btn, 2, 1)

        # --- Waveform mode rows (hidden initially) ---
        self.pos_a_label = QLabel("a (sin amp, deg):")
        self.pos_a_input = QLineEdit("30.0")
        self.pos_b_label = QLabel("b (cos amp, deg):")
        self.pos_b_input = QLineEdit("30.0")
        self.pos_freq_label = QLabel("Frequency (Hz):")
        self.pos_freq_input = QLineEdit("0.5")
        self.pos_offset_label = QLabel("Offset (deg):")
        self.pos_offset_input = QLineEdit("0.0")
        pos_layout.addWidget(self.pos_a_label, 1, 0)
        pos_layout.addWidget(self.pos_a_input, 1, 1)
        pos_layout.addWidget(self.pos_b_label, 2, 0)
        pos_layout.addWidget(self.pos_b_input, 2, 1)
        pos_layout.addWidget(self.pos_freq_label, 3, 0)
        pos_layout.addWidget(self.pos_freq_input, 3, 1)
        pos_layout.addWidget(self.pos_offset_label, 4, 0)
        pos_layout.addWidget(self.pos_offset_input, 4, 1)

        # Hide waveform rows to start
        for w in [self.pos_a_label, self.pos_a_input,
                  self.pos_b_label, self.pos_b_input,
                  self.pos_freq_label, self.pos_freq_input,
                  self.pos_offset_label, self.pos_offset_input]:
            w.hide()

        self.pos_go_btn = QPushButton("Move to Position")
        self.pos_go_btn.clicked.connect(self.cmd_position)
        pos_layout.addWidget(self.pos_go_btn, 5, 0, 1, 2)
        pos_group.setLayout(pos_layout)
        left_panel.addWidget(pos_group)

        # --- Velocity Control Group ---
        vel_group = QGroupBox("Velocity Control (COMM_SET_RPM)")
        vel_layout = QGridLayout()
        
        self.vel_target_input = QLineEdit("50.0")
        
        self.vel_cw_btn = QRadioButton("Clockwise (CW)")
        self.vel_acw_btn = QRadioButton("Anti-Clockwise (ACW)")
        self.vel_acw_btn.setChecked(True)
        
        self.vel_go_btn = QPushButton("Spin Velocity")
        self.vel_go_btn.clicked.connect(self.cmd_velocity)

        vel_layout.addWidget(QLabel("Target (Limb RPM):"), 0, 0)
        vel_layout.addWidget(self.vel_target_input, 0, 1)
        vel_layout.addWidget(self.vel_cw_btn, 1, 0)
        vel_layout.addWidget(self.vel_acw_btn, 1, 1)
        vel_layout.addWidget(self.vel_go_btn, 2, 0, 1, 2)
        vel_group.setLayout(vel_layout)
        left_panel.addWidget(vel_group)

        # --- Current Control Group ---
        cur_group = QGroupBox("Torque Control (COMM_SET_CURRENT)")
        cur_layout = QGridLayout()
        self.cur_target_input = QLineEdit("0.0")
        
        self.cur_cw_btn = QRadioButton("CW")
        self.cur_cw_btn.setChecked(True)
        self.cur_ccw_btn = QRadioButton("CCW")
        self.cur_dir_group = QButtonGroup()
        self.cur_dir_group.addButton(self.cur_cw_btn)
        self.cur_dir_group.addButton(self.cur_ccw_btn)
        
        cur_dir_layout = QHBoxLayout()
        cur_dir_layout.addWidget(self.cur_cw_btn)
        cur_dir_layout.addWidget(self.cur_ccw_btn)
        
        self.cur_go_btn = QPushButton("Apply Torque")
        self.cur_go_btn.clicked.connect(self.cmd_current)

        # Live actual torque display
        self.actual_torque_label = QLabel("Actual Torque: -- Nm  (-- A)")
        self.actual_torque_label.setStyleSheet("font-weight: bold; color: #00bfff;")
        
        cur_layout.addWidget(QLabel("Target Torque (Nm):"), 0, 0)
        cur_layout.addWidget(self.cur_target_input, 0, 1)
        cur_layout.addWidget(QLabel("Direction:"), 1, 0)
        cur_layout.addLayout(cur_dir_layout, 1, 1)
        cur_layout.addWidget(self.cur_go_btn, 2, 0, 1, 2)
        cur_layout.addWidget(self.actual_torque_label, 3, 0, 1, 2)
        cur_group.setLayout(cur_layout)
        left_panel.addWidget(cur_group)

        # --- Origin Management Group ---
        org_group = QGroupBox("Origin Management")
        org_layout = QGridLayout()
        self.org_temp_btn = QPushButton("Set Temp Origin (Mode 0)")
        self.org_temp_btn.clicked.connect(lambda: self.motor_thread.send_cmd({'type': 'origin', 'val': 0}))
        
        self.org_perm_btn = QPushButton("Set Perm Origin (Mode 1)")
        self.org_perm_btn.clicked.connect(lambda: self.motor_thread.send_cmd({'type': 'origin', 'val': 1}))
        
        self.org_go_btn = QPushButton("Go to Origin (0.0 deg)")
        self.org_go_btn.clicked.connect(self.cmd_go_origin)

        org_layout.addWidget(self.org_temp_btn, 0, 0)
        org_layout.addWidget(self.org_perm_btn, 0, 1)
        org_layout.addWidget(self.org_go_btn, 1, 0, 1, 2)
        org_group.setLayout(org_layout)
        left_panel.addWidget(org_group)

        left_panel.addStretch()

        # Right Panel (Graphing)
        right_panel = QVBoxLayout()
        main_layout.addLayout(right_panel, stretch=3)

        self.graph_selector = QComboBox()
        self.graph_selector.addItems(["Position (Degrees)", "Torque (Nm)", "Speed (Limb RPM)"])
        right_panel.addWidget(QLabel("Select Data to Graph:"))
        right_panel.addWidget(self.graph_selector)

        # Real-time Telemetry Labels
        self.live_telemetry_label = QLabel("Live Telemetry: Waiting for connection...")
        self.live_telemetry_label.setStyleSheet("font-size: 16px; font-weight: bold; padding: 10px;")
        right_panel.addWidget(self.live_telemetry_label)

        # PyQtGraph Plot setup
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('k')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_curve = self.plot_widget.plot(pen=pg.mkPen(color='c', width=2))
        self.target_curve = self.plot_widget.plot(pen=pg.mkPen(color='r', width=2, style=Qt.DashLine))
        right_panel.addWidget(self.plot_widget)

    def _on_pos_mode_changed(self, idx):
        """Show/hide UI rows based on selected position mode."""
        is_point = (idx == 0)
        for w in [self.pos_target_label, self.pos_target_input,
                  self.pos_cw_btn, self.pos_acw_btn]:
            w.setVisible(is_point)

        has_a = idx in (1, 3)  # sin or sin+cos
        has_b = idx in (2, 3)  # cos or sin+cos
        self.pos_a_label.setVisible(has_a)
        self.pos_a_input.setVisible(has_a)
        self.pos_b_label.setVisible(has_b)
        self.pos_b_input.setVisible(has_b)
        for w in [self.pos_freq_label, self.pos_freq_input,
                  self.pos_offset_label, self.pos_offset_input]:
            w.setVisible(not is_point)

        if is_point:
            self.pos_go_btn.setText("Move to Position")
        else:
            self.pos_go_btn.setText("Start Trajectory")

    # --- Button Commands ---
    def toggle_connection(self):
        if self.motor_thread.running:
            self.stop_active_loop()
            self.motor_thread.disconnect_motor()
        else:
            self.motor_thread.connect_motor(self.port_input.text())
            self.connect_btn.setText("Connecting...")
            self.connect_btn.setEnabled(False)

    def on_motor_changed(self, motor_name):
        specs = MOTORS.get(motor_name)
        if specs:
            self.kt = specs.get("torque_constant", 0.136)
            self.pole_pairs = specs.get("pole_pairs", 21)
            self.motor_thread.gear_ratio = specs.get("gear_ratio", 64.0)
            self.motor_thread.baud_rate = specs.get("baud_rate", 921600)
            self.setWindowTitle(f"{motor_name} High-Performance Motor Control")

    def stop_active_loop(self):
        if self.active_loop and self.active_loop.is_alive():
            self.active_loop.stop()
            self.active_loop = None
            
        self.pos_go_btn.setText("Move to Position" if self.pos_mode_combo.currentIndex() == 0 else "Start Trajectory")
        self.pos_go_btn.setStyleSheet("")
        self.vel_go_btn.setText("Spin Velocity")
        self.vel_go_btn.setStyleSheet("")
        self.cur_go_btn.setText("Apply Torque")
        self.cur_go_btn.setStyleSheet("")
        self.motor_thread.send_cmd({'type': 'stop'})

    def cmd_stop(self):
        self.stop_active_loop()

    def cmd_position(self):
        if self.pos_go_btn.text() in ("Terminate", "Stop Trajectory"):
            self.stop_active_loop()
            return

        self.stop_active_loop()
        mode_idx = self.pos_mode_combo.currentIndex()

        if mode_idx == 0:
            # --- Point mode ---
            try:
                target_deg = float(self.pos_target_input.text())
            except ValueError:
                self.show_error("Invalid Position Input")
                return
            want_cw = self.pos_cw_btn.isChecked()
            self.active_loop = PositionControlLoop(self.current_pos, target_deg, want_cw, send_cmd_func=self.motor_thread.send_cmd)
            self.active_loop.start()
            self.pos_go_btn.setText("Terminate")
            self.pos_go_btn.setStyleSheet("background-color: orange; color: black; font-weight: bold;")
        else:
            # --- Waveform mode ---
            mode_map = {1: 'sin', 2: 'cos', 3: 'sin+cos'}
            mode = mode_map[mode_idx]
            try:
                a      = float(self.pos_a_input.text())      if mode_idx in (1, 3) else 0.0
                b      = float(self.pos_b_input.text())      if mode_idx in (2, 3) else 0.0
                freq   = float(self.pos_freq_input.text())
                offset = float(self.pos_offset_input.text())
            except ValueError:
                self.show_error("Invalid waveform parameters")
                return
            if freq <= 0:
                self.show_error("Frequency must be > 0 Hz")
                return
            self.active_loop = SinusoidalPositionLoop(self.motor_thread.send_cmd, a, b, freq, offset, mode)
            self.active_loop.start()
            self.pos_go_btn.setText("Stop Trajectory")
            self.pos_go_btn.setStyleSheet("background-color: orange; color: black; font-weight: bold;")

    def cmd_velocity(self):
        if self.vel_go_btn.text() == "Terminate":
            self.stop_active_loop()
            return
            
        self.stop_active_loop()
        try:
            target_rpm = float(self.vel_target_input.text())
        except ValueError:
            self.show_error("Invalid Velocity Input")
            return
            
        want_cw = self.vel_cw_btn.isChecked()
        target_rad_s = target_rpm * (2 * math.pi / 60.0)
        
        self.active_loop = VelocityControlLoop(target_rad_s, want_cw, send_cmd_func=self.motor_thread.send_cmd)
        self.active_loop.start()
        
        self.vel_go_btn.setText("Terminate")
        self.vel_go_btn.setStyleSheet("background-color: orange; color: black; font-weight: bold;")

    def cmd_current(self):
        if self.cur_go_btn.text() == "Terminate":
            self.stop_active_loop()
            return
            
        self.stop_active_loop()
        try:
            target_torque = float(self.cur_target_input.text())
        except ValueError:
            self.show_error("Invalid Input")
            return
            
        target_amps = target_torque / self.kt if self.kt != 0 else 0.0
        want_cw = self.cur_cw_btn.isChecked()
        self.active_loop = CurrentControlLoop(target_amps, want_cw, send_cmd_func=self.motor_thread.send_cmd)
        self.active_loop.start()
        
        self.cur_go_btn.setText("Terminate")
        self.cur_go_btn.setStyleSheet("background-color: orange; color: black; font-weight: bold;")

    def cmd_go_origin(self):
        # We assume they want the shortest path to 0
        target_circle = 0.0
        dist_cw = (self.current_pos - target_circle) % 360.0
        dist_ccw = (target_circle - self.current_pos) % 360.0
        
        if dist_cw < dist_ccw:
            final_target_deg = self.current_pos - dist_cw
        else:
            final_target_deg = self.current_pos + dist_ccw
            
        self.motor_thread.send_cmd({'type': 'pos', 'val': final_target_deg})

    # --- Signals & Callbacks ---
    @pyqtSlot(bool)
    def on_connected(self, connected):
        self.connect_btn.setEnabled(True)
        if connected:
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setStyleSheet("background-color: lightgreen;")
        else:
            self.connect_btn.setText("Connect")
            self.connect_btn.setStyleSheet("")
            self.live_telemetry_label.setText("Live Telemetry: Disconnected")

    @pyqtSlot(dict)
    def on_telemetry(self, t_data):
        self.current_pos = t_data['filtered_pos']
        t = time.time() - self.start_time
        
        current = t_data['filtered_current']
        erpm = t_data['filtered_erpm']
        pos_mod = self.current_pos % 360.0
        actual_torque_nm = abs(current) * self.kt
        
        self.times.append(t)
        self.data_pos.append(pos_mod)
        self.data_cur.append(actual_torque_nm)
        self.data_erpm.append(erpm)
        
        target_val = self.motor_thread.current_target
        if self.motor_thread.current_target_type in ('pos', 'pos_raw', 'pos_direct'):
            target_val = target_val % 360.0
        elif self.motor_thread.current_target_type == 'cur':
            target_val = abs(target_val) * self.kt
        self.data_target.append(target_val)
        # deque with maxlen auto-discards oldest — no manual pop needed
            
        self.live_telemetry_label.setText(f"Pos: {pos_mod:.2f} deg | Cur: {current:.2f} A | ERPM: {erpm:.0f}")
        self.actual_torque_label.setText(f"Actual Torque: {actual_torque_nm:.3f} Nm  ({current:.3f} A)")
        
        if hasattr(self, 'log_file') and not self.log_file.closed:
            target = self.motor_thread.current_target
            row = [
                f"{t:.4f}",
                str(self.session_num),
                f"{target:.4f}",
                f"{t_data['filtered_pos']:.4f}",
                f"{t_data['filtered_current']:.4f}",
                f"{t_data['filtered_erpm']:.0f}",
                f"{t_data['temp_fet']:.2f}",
                f"{t_data['temp_motor']:.2f}",
                f"{t_data['motor_current']:.4f}",
                f"{t_data['current_in']:.4f}",
                f"{t_data['id']:.4f}",
                f"{t_data['iq']:.4f}",
                f"{t_data['duty_now']:.4f}",
                f"{t_data['erpm']:.0f}",
                f"{t_data['v_in']:.2f}",
                f"{t_data['amp_hours']:.4f}",
                f"{t_data['amp_hours_charged']:.4f}",
                f"{t_data['watt_hours']:.4f}",
                f"{t_data['watt_hours_charged']:.4f}",
                f"{t_data['tachometer']}",
                f"{t_data['tachometer_abs']}",
                f"{t_data['fault_code']}",
                f"{t_data['pos_deg']:.4f}"
            ]
            self.csv_writer.writerow(row)
            # Flush to disk every ~1 second (100 ticks) instead of every tick
            self._csv_flush_counter += 1
            if self._csv_flush_counter >= 100:
                self.log_file.flush()
                self._csv_flush_counter = 0

    def update_plot(self):
        mode = self.graph_selector.currentIndex()
        # Convert deques to lists once for pyqtgraph (it needs sequences)
        t = list(self.times)
        tgt = list(self.data_target)
        
        if mode == 0:  # Position
            self.plot_curve.setData(t, list(self.data_pos))
            if self.motor_thread.current_target_type in ('pos', 'pos_raw', 'pos_direct'):
                self.target_curve.setData(t, tgt)
            else:
                self.target_curve.setData([], [])
                
        elif mode == 1:  # Current
            self.plot_curve.setData(t, list(self.data_cur))
            if self.motor_thread.current_target_type == 'cur':
                self.target_curve.setData(t, tgt)
            else:
                self.target_curve.setData([], [])
                
        elif mode == 2:  # Speed
            self.plot_curve.setData(t, list(self.data_erpm))
            if self.motor_thread.current_target_type in ('vel', 'vel_raw'):
                self.target_curve.setData(t, tgt)
            else:
                self.target_curve.setData([], [])

    def show_error(self, message):
        QMessageBox.critical(self, "Error", message)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MotorControlGUI()
    window.show()
    sys.exit(app.exec_())
