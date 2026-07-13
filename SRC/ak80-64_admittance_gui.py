import sys
import time
import struct
import queue
import signal
import importlib
import threading
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QLineEdit, QComboBox, QGroupBox, QGridLayout,
                             QMessageBox, QRadioButton, QButtonGroup, QTabWidget, QSlider, QDoubleSpinBox,
                             QFormLayout, QSplitter)
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QTimer, Qt
import pyqtgraph as pg
import serial
import math

try:
    PositionControlLoop = importlib.import_module("ak80-64_pos").PositionControlLoop
    VelocityControlLoop = importlib.import_module("ak80-64_vel").VelocityControlLoop
    CurrentControlLoop = importlib.import_module("ak80-64_current").CurrentControlLoop
except ImportError as e:
    print(f"Failed to import control loops: {e}")


# ==========================================
# ADMITTANCE CONTROLLER THREAD
# ==========================================
class AdmittanceModel:
    """Virtual mass-spring-damper: Mv*a + Bv*v + Kv*(x - r) = tau_ext"""
    def __init__(self, Mv, Bv, Kv, dt, pos_min_rad, pos_max_rad):
        self.Mv = Mv
        self.Bv = Bv
        self.Kv = Kv
        self.dt = dt
        self.pos_min = pos_min_rad
        self.pos_max = pos_max_rad
        self.theta_m = 0.0
        self.omega_m = 0.0

    def step(self, tau_ext, r_baseline_rad):
        alpha = (tau_ext - (self.Bv * self.omega_m) - (self.Kv * (self.theta_m - r_baseline_rad))) / self.Mv
        self.omega_m += alpha * self.dt
        self.theta_m += self.omega_m * self.dt
        if self.theta_m > self.pos_max:
            self.theta_m = self.pos_max
            if self.omega_m > 0: self.omega_m = 0.0
        elif self.theta_m < self.pos_min:
            self.theta_m = self.pos_min
            if self.omega_m < 0: self.omega_m = 0.0
        return self.theta_m, self.omega_m


class AdmittanceControlLoop(threading.Thread):
    POLE_PAIRS = 21
    GEAR_RATIO = 64.0
    KT_NM_PER_A = 0.136   # <-- Confirm from datasheet before running
    MAX_MISSED_READS = 10
    MAX_TORQUE_SLEW = 2.0
    DT = 0.01

    def __init__(self, send_cmd_func, params: dict):
        super().__init__()
        self.send_cmd_func = send_cmd_func
        self.params = params
        self.running = False
        self.daemon = True
        self.diag = {'tau_ext': 0.0, 'tau_cmd': 0.0, 'virtual_pos': 0.0, 'actual_pos': 0.0}

    def stop(self):
        self.running = False

    def run(self):
        self.running = True
        p = self.params

        adm = AdmittanceModel(
            p['Mv'], p['Bv'], p['Kv'], self.DT,
            math.radians(p['pos_min_deg']), math.radians(p['pos_max_deg'])
        )
        adm.theta_m = math.radians(p.get('current_pos', 0.0))

        cur_filter_lpf = LowPassFilterAdm(alpha=0.15)
        last_tau_cmd = 0.0
        missed = 0

        print("\nAdmittance controller running. Push on joint to test compliance.")
        print("Call stop() or press Ctrl+C to terminate.")

        try:
            while self.running:
                loop_start = time.perf_counter()

                Mv = max(0.01, p['Mv'])
                Bv = p['Bv']
                Kv = p['Kv']
                Kp_track = p['Kp_track']
                Kd_track = p['Kd_track']
                r_baseline_rad = math.radians(p['r_baseline_deg'])
                max_torque = p['max_torque']

                adm.Mv = Mv
                adm.Bv = Bv
                adm.Kv = Kv
                adm.pos_min = math.radians(p['pos_min_deg'])
                adm.pos_max = math.radians(p['pos_max_deg'])

                raw = p.get('_last_telemetry')

                if raw is not None:
                    missed = 0
                    actual_pos_deg, actual_current, actual_erpm = raw

                    motor_rpm = actual_erpm / self.POLE_PAIRS
                    actual_vel_rad_s = (motor_rpm * 2 * math.pi / 60.0) / self.GEAR_RATIO

                    filtered_cur = cur_filter_lpf.update(actual_current)
                    actual_torque = filtered_cur * self.KT_NM_PER_A

                    tau_friction = 0.05 * actual_vel_rad_s
                    tau_ext = -actual_torque - tau_friction

                    theta_m, omega_m = adm.step(tau_ext, r_baseline_rad)

                    e_pos = math.radians(actual_pos_deg) - theta_m
                    e_vel = actual_vel_rad_s - omega_m
                    tau_raw = -(Kp_track * e_pos) - (Kd_track * e_vel)

                    delta = tau_raw - last_tau_cmd
                    if delta > self.MAX_TORQUE_SLEW: tau_raw = last_tau_cmd + self.MAX_TORQUE_SLEW
                    elif delta < -self.MAX_TORQUE_SLEW: tau_raw = last_tau_cmd - self.MAX_TORQUE_SLEW

                    tau_cmd = max(-max_torque, min(max_torque, tau_raw))
                    last_tau_cmd = tau_cmd
                    current_cmd = tau_cmd / self.KT_NM_PER_A

                    self.send_cmd_func({'type': 'cur', 'val': current_cmd})

                    self.diag['tau_ext'] = tau_ext
                    self.diag['tau_cmd'] = tau_cmd
                    self.diag['virtual_pos'] = math.degrees(theta_m)
                    self.diag['actual_pos'] = actual_pos_deg
                else:
                    missed += 1
                    if missed >= self.MAX_MISSED_READS:
                        self.send_cmd_func({'type': 'stop'})
                        last_tau_cmd = 0.0
                        print(f"\rWARNING: {missed} missed reads -- current zeroed", end="")

                elapsed = time.perf_counter() - loop_start
                sleep_time = self.DT - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n\nCtrl+C (Admittance Thread).")
        finally:
            print("\nZeroing current (admittance).")
            self.send_cmd_func({'type': 'stop'})


class LowPassFilterAdm:
    """Standalone LPF for admittance thread"""
    def __init__(self, alpha):
        self.alpha = alpha
        self.val = 0.0
    def update(self, v):
        self.val = self.alpha * v + (1.0 - self.alpha) * self.val
        return self.val


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
    telemetry_signal = pyqtSignal(float, float, float)  # pos_deg, current_a, erpm
    error_signal = pyqtSignal(str)
    connected_signal = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.port = DEFAULT_PORT
        self.running = False
        self.command_queue = queue.Queue()
        self.ser = None
        self.cur_filter = LowPassFilter(alpha=0.02)
        self.erpm_filter = LowPassFilter(alpha=0.05)
        self.pos_filter = LowPassFilter(alpha=0.1)

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
            self.ser = serial.Serial(self.port, BAUD_RATE, timeout=0.01)
            time.sleep(0.2)
            self.connected_signal.emit(True)
        except Exception as e:
            self.error_signal.emit(f"Failed to connect: {e}")
            self.connected_signal.emit(False)
            return

        while self.running:
            start_loop = time.perf_counter()
            while not self.command_queue.empty():
                try:
                    cmd = self.command_queue.get_nowait()
                    self._handle_command(cmd)
                except queue.Empty:
                    break
            self._fetch_telemetry()
            elapsed = time.perf_counter() - start_loop
            if elapsed < TARGET_DT:
                time.sleep(TARGET_DT - elapsed)

        if self.ser and self.ser.is_open:
            self._handle_command({'type': 'stop'})
            time.sleep(0.1)
            self.ser.close()
        self.connected_signal.emit(False)

    def _handle_command(self, cmd):
        if cmd['type'] == 'pos':
            target_rotor_deg = cmd['val'] * GEAR_RATIO
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
            motor_current = struct.unpack('>i', reply[5:9])[0] / 100.0
            erpm = struct.unpack('>i', reply[23:27])[0]
            pos_deg = struct.unpack('>i', reply[54:58])[0] / 1_000_000.0
            filtered_current = self.cur_filter.update(motor_current)
            filtered_erpm = self.erpm_filter.update(erpm)
            filtered_pos = self.pos_filter.update(pos_deg)
            self.telemetry_signal.emit(filtered_pos, filtered_current, filtered_erpm)


# ==========================================
# 3. MAIN GUI WINDOW
# ==========================================
class MotorControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AK80-64 Motor Control — Admittance Edition")
        self.setGeometry(100, 100, 1400, 800)

        self.motor_thread = MotorThread()
        self.motor_thread.connected_signal.connect(self.on_connected)
        self.motor_thread.telemetry_signal.connect(self.on_telemetry)
        self.motor_thread.error_signal.connect(self.show_error)

        self.history_len = 500
        self.times = []
        self.data_pos = []
        self.data_cur = []
        self.data_erpm = []
        self.start_time = time.time()
        self.current_pos = 0.0
        self.active_loop = None

        self.admittance_params = {
            'Mv': 0.25, 'Bv': 2.5, 'Kv': 8.0,
            'Kp_track': 15.0, 'Kd_track': 1.0,
            'r_baseline_deg': 0.0, 'max_torque': 8.0,
            'pos_min_deg': -160.0, 'pos_max_deg': 0.0,
            'current_pos': 0.0, '_last_telemetry': None,
        }

        self.setup_ui()

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(33)

        self.adm_timer = QTimer()
        self.adm_timer.timeout.connect(self.update_adm_diag)
        self.adm_timer.start(100)

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        outer_layout = QVBoxLayout(central_widget)

        # --- Connection Bar (always visible) ---
        conn_group = QGroupBox("Connection")
        conn_layout = QGridLayout()
        self.port_input = QLineEdit(DEFAULT_PORT)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.stop_btn = QPushButton("EMERGENCY STOP (Zero Current)")
        self.stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold; padding: 8px;")
        self.stop_btn.clicked.connect(self.cmd_stop)
        conn_layout.addWidget(QLabel("COM Port:"), 0, 0)
        conn_layout.addWidget(self.port_input, 0, 1)
        conn_layout.addWidget(self.connect_btn, 0, 2)
        conn_layout.addWidget(self.stop_btn, 1, 0, 1, 3)
        conn_group.setLayout(conn_layout)
        outer_layout.addWidget(conn_group)

        self.tab_widget = QTabWidget()
        outer_layout.addWidget(self.tab_widget)

        # ============================
        # TAB 1: MOTOR CONTROL
        # ============================
        control_tab = QWidget()
        control_main = QHBoxLayout(control_tab)
        ctrl_left = QVBoxLayout()
        control_main.addLayout(ctrl_left, stretch=1)

        pos_group = QGroupBox("Position Control (COMM_SET_POS)")
        pos_layout = QGridLayout()
        self.pos_target_input = QLineEdit("0.0")
        self.pos_cw_btn = QRadioButton("Clockwise (CW)")
        self.pos_acw_btn = QRadioButton("Anti-Clockwise (ACW)")
        self.pos_cw_btn.setChecked(True)
        self.pos_go_btn = QPushButton("Move to Position")
        self.pos_go_btn.clicked.connect(self.cmd_position)
        pos_layout.addWidget(QLabel("Target (deg):"), 0, 0)
        pos_layout.addWidget(self.pos_target_input, 0, 1)
        pos_layout.addWidget(self.pos_cw_btn, 1, 0)
        pos_layout.addWidget(self.pos_acw_btn, 1, 1)
        pos_layout.addWidget(self.pos_go_btn, 2, 0, 1, 2)
        pos_group.setLayout(pos_layout)
        ctrl_left.addWidget(pos_group)

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
        ctrl_left.addWidget(vel_group)

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
        cur_layout.addWidget(QLabel("Target (Amps):"), 0, 0)
        cur_layout.addWidget(self.cur_target_input, 0, 1)
        cur_layout.addWidget(QLabel("Direction:"), 1, 0)
        cur_layout.addLayout(cur_dir_layout, 1, 1)
        cur_layout.addWidget(self.cur_go_btn, 2, 0, 1, 2)
        cur_group.setLayout(cur_layout)
        ctrl_left.addWidget(cur_group)

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
        ctrl_left.addWidget(org_group)
        ctrl_left.addStretch()

        ctrl_right = QVBoxLayout()
        control_main.addLayout(ctrl_right, stretch=3)
        self.graph_selector = QComboBox()
        self.graph_selector.addItems(["Position (Degrees)", "Current (Amps)", "Speed (ERPM)"])
        ctrl_right.addWidget(QLabel("Select Data to Graph:"))
        ctrl_right.addWidget(self.graph_selector)
        self.live_telemetry_label = QLabel("Live Telemetry: Waiting for connection...")
        self.live_telemetry_label.setStyleSheet("font-size: 16px; font-weight: bold; padding: 10px;")
        ctrl_right.addWidget(self.live_telemetry_label)
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('k')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_curve = self.plot_widget.plot(pen=pg.mkPen(color='c', width=2))
        ctrl_right.addWidget(self.plot_widget)
        self.tab_widget.addTab(control_tab, "Motor Control")

        # ============================
        # TAB 2: ADMITTANCE CONTROLLER
        # ============================
        adm_tab = QWidget()
        adm_main = QHBoxLayout(adm_tab)

        adm_left = QVBoxLayout()
        adm_main.addLayout(adm_left, stretch=1)

        def make_slider_row(label, key, lo, hi, default, scale=100):
            group = QGroupBox(label)
            layout = QHBoxLayout()
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(int(lo * scale))
            slider.setMaximum(int(hi * scale))
            slider.setValue(int(default * scale))
            val_label = QLabel(f"{default:.3f}")
            val_label.setMinimumWidth(60)
            def on_change(v, k=key, s=scale, lbl=val_label):
                real = v / s
                self.admittance_params[k] = real
                lbl.setText(f"{real:.3f}")
            slider.valueChanged.connect(on_change)
            layout.addWidget(slider)
            layout.addWidget(val_label)
            group.setLayout(layout)
            return group

        adm_left.addWidget(QLabel("--- Virtual Dynamics ---"))
        adm_left.addWidget(make_slider_row("Mv - Virtual Mass (kg.m^2)",        'Mv',             0.01,  2.0,   0.25,  100))
        adm_left.addWidget(make_slider_row("Bv - Virtual Damping (Nm.s/rad)",   'Bv',             0.0,   20.0,  2.5,   100))
        adm_left.addWidget(make_slider_row("Kv - Virtual Stiffness (Nm/rad)",   'Kv',             0.0,   50.0,  8.0,   100))
        adm_left.addWidget(QLabel("--- Inner Tracker Gains ---"))
        adm_left.addWidget(make_slider_row("Kp_track - Position Gain",          'Kp_track',       0.0,   60.0,  15.0,  10))
        adm_left.addWidget(make_slider_row("Kd_track - Velocity Gain",          'Kd_track',       0.0,   10.0,  1.0,   100))
        adm_left.addWidget(QLabel("--- Reference & Safety ---"))
        adm_left.addWidget(make_slider_row("Baseline Position (deg)",           'r_baseline_deg', -160.0, 0.0,  0.0,   10))
        adm_left.addWidget(make_slider_row("Max Torque (Nm)",                   'max_torque',     0.5,   12.0,  8.0,   10))
        adm_left.addWidget(make_slider_row("Joint Min (deg)",                   'pos_min_deg',    -180.0, 0.0, -160.0, 10))
        adm_left.addWidget(make_slider_row("Joint Max (deg)",                   'pos_max_deg',    -90.0,  10.0,  0.0,  10))

        self.adm_start_btn = QPushButton("Start Admittance Control")
        self.adm_start_btn.setStyleSheet("background-color: #2ecc71; color: black; font-weight: bold; padding: 10px; font-size: 14px;")
        self.adm_start_btn.clicked.connect(self.cmd_admittance_toggle)
        adm_left.addWidget(self.adm_start_btn)
        adm_left.addStretch()

        adm_right = QVBoxLayout()
        adm_main.addLayout(adm_right, stretch=2)

        self.adm_diag_label = QLabel("Admittance Diagnostics: Stopped")
        self.adm_diag_label.setStyleSheet("font-size: 14px; font-weight: bold; padding: 8px; background-color: #1a1a2e; color: #e0e0e0; border-radius: 6px;")
        self.adm_diag_label.setWordWrap(True)
        adm_right.addWidget(self.adm_diag_label)

        self.adm_plot_widget = pg.PlotWidget(title="Position: Actual vs Virtual")
        self.adm_plot_widget.setBackground('k')
        self.adm_plot_widget.showGrid(x=True, y=True)
        self.adm_plot_widget.addLegend()
        self.adm_curve_actual = self.adm_plot_widget.plot(pen=pg.mkPen(color='c', width=2), name="Actual Position")
        self.adm_curve_virtual = self.adm_plot_widget.plot(pen=pg.mkPen(color='y', width=2, style=Qt.DashLine), name="Virtual Position")
        adm_right.addWidget(self.adm_plot_widget)

        self.tau_plot_widget = pg.PlotWidget(title="Torque (Nm)")
        self.tau_plot_widget.setBackground('k')
        self.tau_plot_widget.showGrid(x=True, y=True)
        self.tau_plot_widget.addLegend()
        self.tau_curve_ext = self.tau_plot_widget.plot(pen=pg.mkPen(color='r', width=2), name="tau_ext")
        self.tau_curve_cmd = self.tau_plot_widget.plot(pen=pg.mkPen(color='g', width=2), name="tau_cmd")
        adm_right.addWidget(self.tau_plot_widget)

        self.adm_times = []
        self.adm_actual_pos = []
        self.adm_virtual_pos = []
        self.adm_tau_ext = []
        self.adm_tau_cmd = []

        self.tab_widget.addTab(adm_tab, "Admittance Control")

    # --- Button Commands ---
    def toggle_connection(self):
        if self.motor_thread.running:
            self.stop_active_loop()
            self.motor_thread.disconnect_motor()
        else:
            self.motor_thread.connect_motor(self.port_input.text())
            self.connect_btn.setText("Connecting...")
            self.connect_btn.setEnabled(False)

    def stop_active_loop(self):
        if self.active_loop and self.active_loop.is_alive():
            self.active_loop.stop()
            self.active_loop = None
        self.pos_go_btn.setText("Move to Position")
        self.pos_go_btn.setStyleSheet("")
        self.vel_go_btn.setText("Spin Velocity")
        self.vel_go_btn.setStyleSheet("")
        self.cur_go_btn.setText("Apply Torque")
        self.cur_go_btn.setStyleSheet("")
        if hasattr(self, 'adm_start_btn'):
            self.adm_start_btn.setText("Start Admittance Control")
            self.adm_start_btn.setStyleSheet("background-color: #2ecc71; color: black; font-weight: bold; padding: 10px; font-size: 14px;")
        self.motor_thread.send_cmd({'type': 'stop'})

    def cmd_stop(self):
        self.stop_active_loop()

    def cmd_position(self):
        if self.pos_go_btn.text() == "Terminate":
            self.stop_active_loop()
            return
        self.stop_active_loop()
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
            target_amps = float(self.cur_target_input.text())
        except ValueError:
            self.show_error("Invalid Input")
            return
        want_cw = self.cur_cw_btn.isChecked()
        self.active_loop = CurrentControlLoop(target_amps, want_cw, send_cmd_func=self.motor_thread.send_cmd)
        self.active_loop.start()
        self.cur_go_btn.setText("Terminate")
        self.cur_go_btn.setStyleSheet("background-color: orange; color: black; font-weight: bold;")

    def cmd_go_origin(self):
        target_circle = 0.0
        dist_cw = (self.current_pos - target_circle) % 360.0
        dist_ccw = (target_circle - self.current_pos) % 360.0
        if dist_cw < dist_ccw:
            final_target_deg = self.current_pos - dist_cw
        else:
            final_target_deg = self.current_pos + dist_ccw
        self.motor_thread.send_cmd({'type': 'pos', 'val': final_target_deg})

    def cmd_admittance_toggle(self):
        if self.active_loop and isinstance(self.active_loop, AdmittanceControlLoop) and self.active_loop.is_alive():
            self.active_loop.stop()
            self.active_loop = None
            self.adm_start_btn.setText("Start Admittance Control")
            self.adm_start_btn.setStyleSheet("background-color: #2ecc71; color: black; font-weight: bold; padding: 10px; font-size: 14px;")
            self.adm_diag_label.setText("Admittance Diagnostics: Stopped")
        else:
            self.stop_active_loop()
            self.admittance_params['current_pos'] = self.current_pos
            self.admittance_params['_last_telemetry'] = None
            self.active_loop = AdmittanceControlLoop(self.motor_thread.send_cmd, self.admittance_params)
            self.active_loop.start()
            self.adm_start_btn.setText("Stop Admittance Control")
            self.adm_start_btn.setStyleSheet("background-color: orange; color: black; font-weight: bold; padding: 10px; font-size: 14px;")

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

    @pyqtSlot(float, float, float)
    def on_telemetry(self, pos, current, erpm):
        self.current_pos = pos  # Must remain absolute for the control loops to work correctly
        t = time.time() - self.start_time
        
        pos_mod = pos % 360.0
        
        self.times.append(t)
        self.data_pos.append(pos_mod)
        self.data_cur.append(current)
        self.data_erpm.append(erpm)

        # Feed live telemetry to admittance thread via shared dict
        self.admittance_params['_last_telemetry'] = (pos, current, erpm)
        self.admittance_params['current_pos'] = pos

        if len(self.times) > self.history_len:
            self.times.pop(0)
            self.data_pos.pop(0)
            self.data_cur.pop(0)
            self.data_erpm.pop(0)
        self.live_telemetry_label.setText(f"Pos: {pos_mod:.2f} deg | Cur: {current:.2f} A | ERPM: {erpm:.0f}")

    def update_plot(self):
        if not self.times:
            return
        idx = self.graph_selector.currentIndex()
        view_box = self.plot_widget.getViewBox()
        if idx == 0:
            self.plot_curve.setData(self.times, self.data_pos)
            self.plot_widget.setLabel('left', 'Position', units='deg')
            view_box.setLimits(minYRange=1.0)
        elif idx == 1:
            self.plot_curve.setData(self.times, self.data_cur)
            self.plot_widget.setLabel('left', 'Current', units='A')
            view_box.setLimits(minYRange=0.5)
        elif idx == 2:
            self.plot_curve.setData(self.times, self.data_erpm)
            self.plot_widget.setLabel('left', 'Speed', units='ERPM')
            view_box.setLimits(minYRange=50.0)

    def update_adm_diag(self):
        if not (self.active_loop and isinstance(self.active_loop, AdmittanceControlLoop) and self.active_loop.is_alive()):
            return
        d = self.active_loop.diag
        t = time.time() - self.start_time

        self.adm_diag_label.setText(
            f"Actual Pos: {d['actual_pos']:.2f} deg  |  Virtual Pos: {d['virtual_pos']:.2f} deg\n"
            f"tau_ext: {d['tau_ext']:.3f} Nm  |  tau_cmd: {d['tau_cmd']:.3f} Nm"
        )

        self.adm_times.append(t)
        self.adm_actual_pos.append(d['actual_pos'])
        self.adm_virtual_pos.append(d['virtual_pos'])
        self.adm_tau_ext.append(d['tau_ext'])
        self.adm_tau_cmd.append(d['tau_cmd'])

        max_len = 300
        if len(self.adm_times) > max_len:
            self.adm_times = self.adm_times[-max_len:]
            self.adm_actual_pos = self.adm_actual_pos[-max_len:]
            self.adm_virtual_pos = self.adm_virtual_pos[-max_len:]
            self.adm_tau_ext = self.adm_tau_ext[-max_len:]
            self.adm_tau_cmd = self.adm_tau_cmd[-max_len:]

        self.adm_curve_actual.setData(self.adm_times, self.adm_actual_pos)
        self.adm_curve_virtual.setData(self.adm_times, self.adm_virtual_pos)
        self.tau_curve_ext.setData(self.adm_times, self.adm_tau_ext)
        self.tau_curve_cmd.setData(self.adm_times, self.adm_tau_cmd)

    @pyqtSlot(str)
    def show_error(self, message):
        QMessageBox.critical(self, "Error", message)

    def closeEvent(self, event):
        self.cleanup()
        event.accept()

    def cleanup(self):
        self.stop_active_loop()
        if self.motor_thread.running:
            self.motor_thread.disconnect_motor()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, lambda sig, frame: QApplication.quit())
    app = QApplication(sys.argv)
    window = MotorControlGUI()
    app.aboutToQuit.connect(window.cleanup)
    window.show()
    sys.exit(app.exec_())
