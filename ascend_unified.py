#!/usr/bin/env python3
"""
=============================================================================
ASCEND GCS — Complete Ground Control + Vision System
=============================================================================
Windows / Linux compatible. Single script does EVERYTHING:

  1. D435i depth camera → optical flow + altitude
  2. Sends VISION_POSITION_ESTIMATE to Pixhawk (position hold)
  3. Sends DISTANCE_SENSOR to Pixhawk (rangefinder)
  4. Reads ALL telemetry back from Pixhawk
  5. Live color camera feed via MJPEG stream
  6. Live optical flow visualization feed
  7. ARM / DISARM / MODE control via HTTP API
  8. Web dashboard at http://localhost:5000

Install (Windows):
  pip install pyrealsense2 pymavlink opencv-python numpy flask flask-cors

Install (Linux / Jetson / Pi):
  pip3 install pyrealsense2 pymavlink opencv-python-headless numpy flask flask-cors

Run with hardware:
  python ascend_gcs.py --serial COM5 --baud 57600         (Windows)
  python3 ascend_gcs.py --serial /dev/ttyACM0 --baud 57600 (Linux)

Run demo (no hardware):
  python ascend_gcs.py --demo

Dashboard: http://localhost:5000
Color feed: http://localhost:5000/feed/color
Flow feed:  http://localhost:5000/feed/flow

=============================================================================
Team Saarabai X — IRoC-U 2026 — Bennett University
=============================================================================
"""

import numpy as np
import cv2
import time
import threading
import collections
import argparse
import signal
import sys
import math
import os
from flask import Flask, Response, request, jsonify
from flask_cors import CORS

# =============================================================================
# CONFIGURATION
# =============================================================================
class Config:
    # Connection
    SERIAL_PORT = "COM13" if os.name == "nt" else "/dev/ttyACM0"
    BAUD_RATE = 57600
    WEB_PORT = 5000

    # D435i streams
    WIDTH = 640
    HEIGHT = 480
    FPS = 30

    # Optical flow
    LK_WIN = 25
    LK_LEVELS = 3
    FEAT_MAX = 500
    FEAT_QUALITY = 0.015
    FEAT_MIN_DIST = 8
    MIN_FEAT = 30
    MIN_FLOW = 15
    DENSE_FALLBACK = False
    OUTLIER_MAD = 2.5
    MAX_VEL = 5.0

    # Depth
    DEPTH_ROI_PCT = 15
    DEPTH_MIN = 0.1
    DEPTH_MAX = 10.0
    DEPTH_HIST = 5
    DEPTH_MAX_JUMP = 2.0

    # Kalman
    KF_Q = 0.001
    KF_R = 0.01

    # Camera yaw (degrees, 0 = USB connector forward)
    CAM_YAW = 0

    # Rates
    VISION_HZ = 30
    WEB_FEED_FPS = 12
    JPEG_QUALITY = 65

    # Safety / command gating
    HEARTBEAT_TIMEOUT = 3.0
    VISION_TIMEOUT = 2.0
    CAMERA_TIMEOUT = 2.0
    ARM_MIN_SOC = 0
    ARM_MIN_CELL_VOLTAGE = 3.5
    ARM_MIN_PACK_VOLTAGE = 10.5
    RC_MIN = 1000
    RC_MAX = 2000
    RC_SAFE_THROTTLE = 1200
    ALLOW_UNSAFE_MODES = False
    ALLOW_AUTO_MODE = False

    # Mode
    DEBUG = False
    DEMO = False


# =============================================================================
# KALMAN FILTER
# =============================================================================
class KalmanFilter2D:
    def __init__(self, q=0.01, r=0.05):
        self.x = np.zeros(4, dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64)
        self.Q = np.eye(4, dtype=np.float64) * q
        self.R = np.eye(2, dtype=np.float64) * r
        self.H = np.array([[0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)

    def predict(self, dt):
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt],
                       [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, vx, vy):
        z = np.array([vx, vy], dtype=np.float64)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def reset(self):
        self.x = np.zeros(4, dtype=np.float64)
        self.P = np.eye(4, dtype=np.float64)


# =============================================================================
# SHARED STATE — Thread-safe telemetry + video frames
# =============================================================================
class TelemetryState:
    HEARTBEAT_STALE_SEC = 3.0
    VISION_STALE_SEC = 2.0
    CAMERA_STALE_SEC = 2.0

    def __init__(self):
        self.lock = threading.Lock()
        self.telemetry = {
            "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0, "alt_rel": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "rollspeed": 0.0, "pitchspeed": 0.0, "yawspeed": 0.0,
            "imu_ax": 0.0, "imu_ay": 0.0, "imu_az": 0.0,
            "imu_gx": 0.0, "imu_gy": 0.0, "imu_gz": 0.0,
            "imu_temp": 0.0,
            "bat_voltage": 0.0, "bat_current": 0.0, "bat_soc": 0.0,
            "bat_temp": 0.0, "cell_voltages": [],
            "flow_quality": 0, "flow_features": 0, "flow_method": "IDLE",
            "rangefinder_dist": 0.0,
            "viso_x": 0.0, "viso_y": 0.0, "viso_z": 0.0,
            "armed": False, "mode": "UNKNOWN", "heading": 0,
            "throttle": 0, "climb": 0.0, "groundspeed": 0.0,
            "ekf_ok": False,
            "vibe_x": 0.0, "vibe_y": 0.0, "vibe_z": 0.0,
            "d435i_ok": False, "pixhawk_ok": False,
            "vision_fps": 0.0, "last_heartbeat": 0.0,
            "last_vision_frame": 0.0, "last_camera_frame": 0.0,
            "camera_feed_ok": False,
            "camera_source": "unavailable",
            "camera_width": 0, "camera_height": 0,
            "camera_snapshot_url": "/api/camera/latest.jpg",
            "identified_objects": [],
            "preflight_ready": False,
            "preflight_message": "Waiting for Pixhawk heartbeat",
        }
        self.color_jpeg = None
        self.flow_jpeg = None

    def update(self, **kw):
        with self.lock:
            if "identified_objects" in kw and kw["identified_objects"] is None:
                kw["identified_objects"] = []
            self.telemetry.update(kw)

    def get(self):
        with self.lock:
            snapshot = dict(self.telemetry)
            color_frame_available = self.color_jpeg is not None
        return self._sanitize_snapshot(snapshot, color_frame_available)

    def _sanitize_snapshot(self, snapshot, color_frame_available):
        now = time.time()
        heartbeat_age = now - float(snapshot.get("last_heartbeat") or 0.0)
        vision_age = now - float(snapshot.get("last_vision_frame") or 0.0)
        camera_age = now - float(snapshot.get("last_camera_frame") or 0.0)

        snapshot["pixhawk_ok"] = (
            bool(snapshot.get("pixhawk_ok")) and
            heartbeat_age <= self.HEARTBEAT_STALE_SEC
        )
        snapshot["d435i_ok"] = (
            bool(snapshot.get("d435i_ok")) and
            vision_age <= self.VISION_STALE_SEC
        )
        snapshot["camera_feed_ok"] = (
            color_frame_available and
            camera_age <= self.CAMERA_STALE_SEC
        )

        return {
            key: self._sanitize_value(value)
            for key, value in snapshot.items()
        }

    @staticmethod
    def _sanitize_value(value):
        if isinstance(value, bool) or value is None:
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else None
        if isinstance(value, list):
            return [TelemetryState._sanitize_value(item) for item in value]
        if isinstance(value, tuple):
            return [TelemetryState._sanitize_value(item) for item in value]
        if isinstance(value, dict):
            return {
                key: TelemetryState._sanitize_value(item)
                for key, item in value.items()
            }
        return value

    def set_camera_frame(self, jpeg_bytes, width=0, height=0, source="d435i-color"):
        ts = time.time() if jpeg_bytes is not None else 0.0
        with self.lock:
            self.color_jpeg = jpeg_bytes
            self.telemetry.update(
                camera_feed_ok=jpeg_bytes is not None,
                last_camera_frame=ts,
                camera_width=int(width or self.telemetry.get("camera_width") or 0),
                camera_height=int(height or self.telemetry.get("camera_height") or 0),
                camera_source=source or "unavailable",
                camera_snapshot_url="/api/camera/latest.jpg",
            )

    def set_color_frame(self, jpeg_bytes, width=0, height=0, source="d435i-color"):
        self.set_camera_frame(jpeg_bytes, width=width, height=height, source=source)

    def set_flow_frame(self, jpeg_bytes):
        with self.lock:
            self.flow_jpeg = jpeg_bytes

    def get_camera_frame(self):
        with self.lock:
            return self.color_jpeg

    def get_color_frame(self):
        return self.get_camera_frame()

    def get_flow_frame(self):
        with self.lock:
            return self.flow_jpeg

    update_telem = update
    get_telem = get


SharedState = TelemetryState


# =============================================================================
# MAVLINK HANDLER
# =============================================================================
class MAVLinkHandler:
    MODE_MAP = {
        0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO",
        4: "GUIDED", 5: "LOITER", 6: "RTL", 7: "CIRCLE",
        9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
        15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
        18: "THROW", 21: "SMART_RTL",
    }

    # Reverse map for setting modes
    MODE_NAME_TO_NUM = {v: k for k, v in MODE_MAP.items()}
    POSITION_MODES = {"LOITER", "POSHOLD", "GUIDED", "AUTO", "BRAKE", "RTL"}
    SAFE_ARM_MODES = {"STABILIZE", "ALT_HOLD", "LOITER", "POSHOLD", "BRAKE", "LAND"}
    BLOCKED_WEB_MODES = {"AUTOTUNE", "FLIP", "SPORT", "THROW"}

    def __init__(self, config, state):
        self.config = config
        self.state = state
        self.conn = None
        self.mavutil = None
        self.boot_time = time.time()
        self.last_hb_send = 0
        self.last_rc_target_send = 0
        self.connected = False
        self.target_z = None  # None or altitude in meters (> 0 = UP)
        self.target_x = 0.0
        self.target_y = 0.0
        self.active_rc_override = {}

    def connect(self):
        from pymavlink import mavutil
        self.mavutil = mavutil

        port = self.config.SERIAL_PORT
        baud = self.config.BAUD_RATE
        print(f"[MAVLink] Connecting to {port} @ {baud}...")

        try:
            self.conn = mavutil.mavlink_connection(port, baud=baud,
                                                    source_system=1,
                                                    source_component=197)
        except Exception as e:
            print(f"[MAVLink] Connection failed: {e}")
            if os.name == "nt":
                print("[HINT] On Windows, use COM port like: --serial COM13")
                print("[HINT] Check Device Manager → Ports (COM & LPT)")
            else:
                print("[HINT] Try: ls /dev/ttyACM* /dev/ttyUSB*")
            return False

        print("[MAVLink] Waiting for heartbeat (15s timeout)...")
        hb = self.conn.wait_heartbeat(timeout=15)
        if not hb:
            print("[MAVLink] TIMEOUT — no heartbeat received!")
            return False

        print(f"[MAVLink] Connected! System={self.conn.target_system} "
              f"Component={self.conn.target_component}")
        self.connected = True
        self.state.update_telem(
            pixhawk_ok=True,
            last_heartbeat=time.time(),
            preflight_ready=False,
            preflight_message="Connected to Pixhawk; waiting for live telemetry",
        )

        # Request data streams
        self.conn.mav.request_data_stream_send(
            self.conn.target_system, self.conn.target_component,
            self.mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1
        )

        # Request specific higher-rate messages
        for msg_id, rate in [
            (self.mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 15),
            (self.mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15),
            (self.mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 10),
            (self.mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5),
            (self.mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
            (self.mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 2),
            (self.mavutil.mavlink.MAVLINK_MSG_ID_VIBRATION, 2),
        ]:
            try:
                self.conn.mav.command_long_send(
                    self.conn.target_system, self.conn.target_component,
                    self.mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, msg_id, int(1e6 / rate), 0, 0, 0, 0, 0
                )
            except Exception:
                pass

        return True

    def _read_state(self):
        getter = getattr(self.state, "get", None) or self.state.get_telem
        return getter()

    def _wait_for_command_ack(self, command_id, timeout=3):
        deadline = time.time() + timeout
        while time.time() < deadline:
            remaining = max(0.0, deadline - time.time())
            ack = self.conn.recv_match(
                type="COMMAND_ACK",
                blocking=True,
                timeout=remaining,
            )
            if ack is None:
                return None
            ack_command = getattr(ack, "command", None)
            if ack_command in (None, command_id):
                return ack
        return None

    def _positioning_ready(self, snapshot):
        if not snapshot.get("ekf_ok"):
            return False, "EKF not ready"
        if snapshot.get("d435i_ok"):
            return True, ""
        if (snapshot.get("rangefinder_dist") or 0.0) > 0:
            return True, ""
        return False, "No fresh vision / range data for position-hold modes"

    def _preflight_checks(self, requested_mode=None):
        snapshot = self._read_state()

        if not snapshot.get("pixhawk_ok"):
            return False, "Pixhawk heartbeat is stale"
        if snapshot.get("armed") and requested_mode is None:
            return True, "Already armed"
        if requested_mode is None:
            current_mode = str(snapshot.get("mode") or "UNKNOWN").upper()
            if current_mode not in self.SAFE_ARM_MODES:
                return False, f"Refusing to arm from {current_mode}; switch to STABILIZE or ALT_HOLD first"
            bat_soc = snapshot.get("bat_soc")
            if bat_soc is not None and bat_soc < self.config.ARM_MIN_SOC:
                return False, f"Battery SOC too low to arm ({bat_soc:.0f}%)"
            cells = [cell for cell in (snapshot.get("cell_voltages") or []) if cell is not None]
            if cells and min(cells) < self.config.ARM_MIN_CELL_VOLTAGE:
                return False, f"Cell voltage too low ({min(cells):.2f}V)"
            bat_voltage = snapshot.get("bat_voltage")
            if bat_voltage is not None and bat_voltage > 0 and bat_voltage < self.config.ARM_MIN_PACK_VOLTAGE:
                return False, f"Pack voltage too low ({bat_voltage:.2f}V)"
        if requested_mode in self.BLOCKED_WEB_MODES and not self.config.ALLOW_UNSAFE_MODES:
            return False, f"{requested_mode} is blocked from the web API"
        if requested_mode == "AUTO" and not self.config.ALLOW_AUTO_MODE:
            return False, "AUTO mode is disabled for web commands; enable it explicitly if you need mission control"
        if requested_mode in self.POSITION_MODES:
            pos_ok, pos_msg = self._positioning_ready(snapshot)
            if not pos_ok:
                return False, pos_msg
        return True, "Preflight checks passed"

    def _update_preflight_status(self, ok, message):
        self.state.update_telem(preflight_ready=bool(ok), preflight_message=message)

    # ---- SEND TO PIXHAWK ----

    def send_vision_position(self, x, y, z):
        if not self.connected:
            return
        ts = int((time.time() - self.boot_time) * 1e6)
        # Tight covariance for position: trusting our optical flow deeply
        cov = [0.005, 0, 0, 0, 0, 0,
               0.005, 0, 0, 0, 0,
               0.01,  0, 0, 0,
               0.01,  0, 0,
               0.01,  0,
               0.02]
        try:
            self.conn.mav.vision_position_estimate_send(
                ts, float(x), float(y), float(z),
                0.0, 0.0, 0.0,
                cov, reset_counter=0
            )
        except Exception as e:
            if self.config.DEBUG:
                print(f"[MAV-TX] vision_position error: {e}")

    def send_vision_speed(self, vx, vy, vz):
        if not self.connected:
            return
        ts = int((time.time() - self.boot_time) * 1e6)
        # Tight covariance for velocity: extreme confidence in vector
        cov = [0.005, 0, 0,
               0, 0.005, 0,
               0, 0, 0.02]  # Row-major 3x3 covariance
        try:
            self.conn.mav.vision_speed_estimate_send(
                ts, float(vx), float(vy), float(vz),
                cov, reset_counter=0
            )
        except Exception as e:
            if self.config.DEBUG:
                print(f"[MAV-TX] vision_speed error: {e}")

    def send_distance(self, dist_cm):
        if not self.connected:
            return
        try:
            self.conn.mav.distance_sensor_send(
                int((time.time() - self.boot_time) * 1000),
                10,                     # min_distance cm
                1000,                   # max_distance cm
                int(max(0, dist_cm)),   # current_distance cm
                0, 0, 25, 0            # type=laser, id=0, orient=down, cov=0
            )
        except Exception as e:
            if self.config.DEBUG:
                print(f"[MAV-TX] distance error: {e}")

    def send_heartbeat(self):
        if not self.connected:
            return
        now = time.time()
        
        # 2Hz periodic target spoofing
        if now - getattr(self, "last_rc_target_send", 0) > 0.5:
            self.last_rc_target_send = now
            # Send continuous RC override if active
            if getattr(self, "active_rc_override", {}):
                try:
                    rc = [65535] * 18
                    for ch, val in self.active_rc_override.items():
                        idx = int(ch) - 1
                        if 0 <= idx < 18:
                            rc[idx] = int(val)
                    self.conn.mav.rc_channels_override_send(
                        self.conn.target_system, self.conn.target_component, *rc
                    )
                except Exception:
                    pass
            
            # Send continuous position target if active
            if getattr(self, "target_z", None) is not None:
                try:
                    self.conn.mav.set_position_target_local_ned_send(
                        0, self.conn.target_system, self.conn.target_component,
                        self.mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        int(0b110111111000), float(self.target_x), float(self.target_y), -float(self.target_z),
                        0, 0, 0, 0, 0, 0, 0, 0
                    )
                except Exception:
                    pass

        # 1Hz standard heartbeat
        if now - self.last_hb_send < 1.0:
            return
        try:
            self.conn.mav.heartbeat_send(
                self.mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                self.mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            self.last_hb_send = now
        except Exception:
            pass

    # ---- COMMANDS ----

    def arm(self):
        """Arm the drone."""
        if not self.connected:
            return False, "Not connected"
        ok, msg = self._preflight_checks()
        self._update_preflight_status(ok, msg)
        if not ok:
            return False, msg
        if "already armed" in msg.lower():
            return True, msg
        try:
            command_id = self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                command_id,
                0,      # confirmation
                1,      # param1: 1 = arm
                0,      # param2: 0 = normal arm
                0, 0, 0, 0, 0
            )
            ack = self._wait_for_command_ack(command_id, timeout=3)
            if ack and ack.result == 0:
                self._update_preflight_status(True, "Armed")
                return True, "Armed successfully"
            elif ack:
                return False, f"Arm rejected: result={ack.result}"
            return False, "No ACK received"
        except Exception as e:
            return False, str(e)

    def disarm(self):
        """Disarm the drone."""
        if not self.connected:
            return False, "Not connected"
        try:
            command_id = self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                command_id,
                0,      # confirmation
                0,      # param1: 0 = disarm
                0,      # param2: 0 = normal, 21196 = force
                0, 0, 0, 0, 0
            )
            ack = self._wait_for_command_ack(command_id, timeout=3)
            if ack and ack.result == 0:
                self._update_preflight_status(False, "Disarmed")
                self.target_z = None
                self.release_rc_override()
                return True, "Disarmed successfully"
            elif ack:
                return False, f"Disarm rejected: result={ack.result}"
            return False, "No ACK received"
        except Exception as e:
            return False, str(e)

    def force_disarm(self):
        """Force disarm (emergency). param2=21196 bypasses checks."""
        if not self.connected:
            return False, "Not connected"
        try:
            command_id = self.mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                command_id,
                0, 0, 21196.0, 0, 0, 0, 0, 0
            )
            self._update_preflight_status(False, "Force disarm sent")
            self.target_z = None
            self.release_rc_override()
            return True, "Force disarm sent"
        except Exception as e:
            return False, str(e)

    def takeoff(self, altitude=4.0):
        """Takeoff to specified altitude in meters."""
        if not self.connected:
            return False, "Not connected"
        try:
            # Force origin again just to be safe
            self.conn.mav.set_gps_global_origin_send(
                self.conn.target_system,
                int(28.38 * 1e7), int(77.12 * 1e7), 0
            )
            self.conn.mav.set_home_position_send(
                self.conn.target_system,
                int(28.38 * 1e7), int(77.12 * 1e7), 0, 0, 0, 0, 0, 0, 0, 0
            )

            # Ensure we are in GUIDED mode
            self.set_mode("GUIDED")
            import time
            time.sleep(0.5)

            # Set mid throttle on override so it doesn't fail-safe or auto-disarm instantly
            self.set_rc_override({"1": 1500, "2": 1500, "3": 1500, "4": 1500})
            
            # Now send TAKEOFF command
            command_id = self.mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                command_id,
                0, 0, 0, 0, 0, 0, 0, float(altitude)
            )
            
            # Snap current position for hover
            st = self.state.get_telem()
            self.target_x = st.get("viso_x", 0.0)
            self.target_y = st.get("viso_y", 0.0)
            
            ack = self._wait_for_command_ack(command_id, timeout=3)
            if ack and ack.result == 0:
                self.target_z = float(altitude)
                self._update_preflight_status(True, f"Takeoff commanded ({altitude}m)")
                return True, f"Takeoff commanded to {altitude}m"
            elif ack:
                # If traditional takeoff fails, spoof throttle in GUIDED
                self.target_z = float(altitude)
                return False, f"Takeoff rejected ({ack.result}), spoofing Z-target"
            
            self.target_z = float(altitude)
            return True, f"Takeoff {altitude}m sent (no ACK)"
        except Exception as e:
            return False, str(e)

    def set_mode(self, mode_name):
        """Set flight mode by name (STABILIZE, LOITER, AUTO, etc)."""
        if not self.connected:
            return False, "Not connected"

        mode_name = mode_name.upper().strip()
        ok, msg = self._preflight_checks(requested_mode=mode_name)
        self._update_preflight_status(ok, msg)
        if not ok:
            return False, msg
        mode_num = self.MODE_NAME_TO_NUM.get(mode_name)

        if mode_num is None:
            # Try mode_mapping from pymavlink
            mapping = self.conn.mode_mapping() or {}
            if mode_name in mapping:
                mode_num = mapping[mode_name]
            else:
                return False, f"Unknown mode: {mode_name}. Available: {list(self.MODE_MAP.values())}"

        try:
            command_id = self.mavutil.mavlink.MAV_CMD_DO_SET_MODE
            self.conn.mav.set_mode_send(
                self.conn.target_system,
                self.mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_num
            )
            ack = self._wait_for_command_ack(command_id, timeout=3)
            if ack and ack.result == 0:
                self._update_preflight_status(True, f"Mode set to {mode_name}")
                return True, f"Mode set to {mode_name}"
            elif ack:
                return False, f"Mode change rejected: result={ack.result}"
            return True, f"Mode {mode_name} sent (no ACK)"
        except Exception as e:
            return False, str(e)

    def set_rc_override(self, channels):
        """
        Override RC channels. channels = dict of {channel_num: pwm_value}
        Channel numbers are 1-indexed (1=roll, 2=pitch, 3=throttle, 4=yaw).
        Use 0 or 65535 to release a channel back to RC transmitter.
        """
        if not self.connected:
            return False, "Not connected"
        snapshot = self._read_state()
        if not snapshot.get("pixhawk_ok"):
            return False, "Pixhawk heartbeat is stale"
        if not isinstance(channels, dict):
            return False, "RC override payload must be a JSON object"
        try:
            rc = [65535] * 18  # 65535 = no override
            for ch, val in channels.items():
                idx = int(ch) - 1
                if 0 <= idx < 18:
                    pwm = int(val)
                    if pwm not in (0, 65535):
                        pwm = max(self.config.RC_MIN, min(self.config.RC_MAX, pwm))
                    if idx == 2 and not snapshot.get("armed") and pwm not in (0, 65535):
                        if pwm > self.config.RC_SAFE_THROTTLE:
                            return False, "Throttle override too high while disarmed"
                    rc[idx] = pwm
            self.active_rc_override = {ch: pwm for ch, pwm in channels.items()}
            self.conn.mav.rc_channels_override_send(
                self.conn.target_system,
                self.conn.target_component,
                *rc
            )
            return True, "RC override sent"
        except Exception as e:
            return False, str(e)

    def release_rc_override(self):
        """Release all RC channel overrides."""
        if not self.connected:
            return False, "Not connected"
        self.active_rc_override = {}
        try:
            rc = [0] * 18  # 0 = release override
            self.conn.mav.rc_channels_override_send(
                self.conn.target_system,
                self.conn.target_component,
                *rc
            )
            return True, "RC override released"
        except Exception as e:
            return False, str(e)

    # ---- READ FROM PIXHAWK (non-blocking) ----

    def read_messages(self):
        if not self.connected:
            return
        while True:
            msg = self.conn.recv_match(blocking=False)
            if msg is None:
                break
            self._process_message(msg)

    def _process_message(self, msg):
        t = msg.get_type()

        if t == "HEARTBEAT":
            armed = bool(msg.base_mode & self.mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            mode = self.MODE_MAP.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
            self.state.update_telem(
                armed=armed, mode=mode,
                last_heartbeat=time.time(), pixhawk_ok=True
            )

        elif t == "LOCAL_POSITION_NED":
            self.state.update_telem(
                pos_x=msg.x, pos_y=msg.y, pos_z=msg.z,
                alt_rel=max(0, -msg.z),
                vx=msg.vx, vy=msg.vy, vz=msg.vz,
            )

        elif t == "ATTITUDE":
            self.state.update_telem(
                roll=math.degrees(msg.roll),
                pitch=math.degrees(msg.pitch),
                yaw=math.degrees(msg.yaw) % 360,
                rollspeed=math.degrees(msg.rollspeed),
                pitchspeed=math.degrees(msg.pitchspeed),
                yawspeed=math.degrees(msg.yawspeed),
            )

        elif t == "RAW_IMU":
            self.state.update_telem(
                imu_ax=msg.xacc / 1000.0 * 9.81,
                imu_ay=msg.yacc / 1000.0 * 9.81,
                imu_az=msg.zacc / 1000.0 * 9.81,
                imu_gx=msg.xgyro / 1000.0 * 57.2958,
                imu_gy=msg.ygyro / 1000.0 * 57.2958,
                imu_gz=msg.zgyro / 1000.0 * 57.2958,
            )

        elif t in ("SCALED_IMU2", "SCALED_IMU"):
            if hasattr(msg, "temperature") and msg.temperature != 0:
                self.state.update_telem(imu_temp=msg.temperature / 100.0)

        elif t == "SYS_STATUS":
            updates = {}
            if msg.voltage_battery != -1 and msg.voltage_battery != 65535:
                updates["bat_voltage"] = msg.voltage_battery / 1000.0
            if msg.current_battery != -1:
                updates["bat_current"] = msg.current_battery / 100.0
            if msg.battery_remaining != -1:
                updates["bat_soc"] = msg.battery_remaining
            if updates:
                self.state.update_telem(**updates)

        elif t == "BATTERY_STATUS":
            cells = [v / 1000.0 for v in msg.voltages
                     if v != 65535 and v != 0 and v > 0]
            u = {}
            if cells:
                u["cell_voltages"] = cells
            if hasattr(msg, "temperature") and msg.temperature != 32767:
                u["bat_temp"] = msg.temperature / 100.0
            if u:
                self.state.update_telem(**u)

        elif t == "VFR_HUD":
            self.state.update_telem(
                heading=msg.heading,
                groundspeed=msg.groundspeed,
                throttle=msg.throttle,
                climb=msg.climb,
            )

        elif t == "OPTICAL_FLOW":
            self.state.update_telem(flow_quality=msg.quality)

        elif t == "DISTANCE_SENSOR":
            if msg.orientation == 25:
                self.state.update_telem(
                    rangefinder_dist=msg.current_distance / 100.0
                )

        elif t == "VISION_POSITION_ESTIMATE":
            self.state.update_telem(
                viso_x=msg.x, viso_y=msg.y, viso_z=msg.z
            )

        elif t == "GLOBAL_POSITION_INT":
            self.state.update_telem(
                alt_rel=msg.relative_alt / 1000.0
            )

        elif t == "EKF_STATUS_REPORT":
            f = msg.flags
            ok = bool((f & 0x01) and (f & 0x02) and (f & 0x04))
            self.state.update_telem(ekf_ok=ok)

        elif t == "VIBRATION":
            self.state.update_telem(
                vibe_x=msg.vibration_x,
                vibe_y=msg.vibration_y,
                vibe_z=msg.vibration_z,
            )

    _process = _process_message


# =============================================================================
# VISION ENGINE — D435i optical flow + altitude + video feeds
# =============================================================================
class VisionEngine:
    def __init__(self, config, state, mav):
        self.config = config
        self.state = state
        self.mav = mav
        self.kalman = KalmanFilter2D(config.KF_Q, config.KF_R)

        self.prev_gray = None
        self.prev_points = None
        self.prev_time = None
        self.depth_hist = collections.deque(maxlen=config.DEPTH_HIST)
        self.altitude = 0.0
        self.last_valid_alt = 0.0

        self.fx = 1.0
        self.fy = 1.0
        self.pipeline = None
        self.roi = (0, 0, 0, 0)

        yaw = np.radians(config.CAM_YAW)
        self.rot = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ], dtype=np.float64)

        self.lk_params = dict(
            winSize=(config.LK_WIN, config.LK_WIN),
            maxLevel=config.LK_LEVELS,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
        )
        self.feat_params = dict(
            maxCorners=config.FEAT_MAX,
            qualityLevel=config.FEAT_QUALITY,
            minDistance=config.FEAT_MIN_DIST,
            blockSize=7,
        )

        self.frame_count = 0
        self.fps_time = time.time()
        self.current_fps = 0.0
        self.last_web_frame = 0.0

    def start(self):
        import pyrealsense2 as rs
        self.rs = rs

        pipeline = rs.pipeline()
        rs_cfg = rs.config()

        # Enable all three streams
        rs_cfg.enable_stream(
            rs.stream.infrared, 1,
            self.config.WIDTH, self.config.HEIGHT,
            rs.format.y8, self.config.FPS
        )
        rs_cfg.enable_stream(
            rs.stream.color,
            self.config.WIDTH, self.config.HEIGHT,
            rs.format.bgr8, self.config.FPS
        )
        rs_cfg.enable_stream(
            rs.stream.depth,
            self.config.WIDTH, self.config.HEIGHT,
            rs.format.z16, self.config.FPS
        )

        profile = pipeline.start(rs_cfg)
        self.pipeline = pipeline
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        # Try high-accuracy preset
        try:
            profile.get_device().first_depth_sensor().set_option(
                rs.option.visual_preset, 3
            )
        except Exception:
            pass

        # Get intrinsics from IR stream
        ir_prof = profile.get_stream(rs.stream.infrared, 1)
        intr = ir_prof.as_video_stream_profile().get_intrinsics()
        self.fx = intr.fx
        self.fy = intr.fy

        # Depth ROI
        margin = int(self.config.DEPTH_ROI_PCT / 100.0 *
                      min(self.config.HEIGHT, self.config.WIDTH))
        cx = self.config.WIDTH // 2
        cy = self.config.HEIGHT // 2
        self.roi = (cy - margin, cy + margin, cx - margin, cx + margin)

        dev_name = profile.get_device().get_info(rs.camera_info.name)
        serial = profile.get_device().get_info(rs.camera_info.serial_number)
        print(f"[D435i] {dev_name} (S/N: {serial})")
        print(f"[D435i] Streams: IR + Color + Depth @ {self.config.WIDTH}x{self.config.HEIGHT}")
        print(f"[D435i] Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f}")
        print(f"[D435i] Depth scale: {self.depth_scale}")
        self.state.update_telem(
            d435i_ok=True,
            camera_source="d435i-color",
            preflight_ready=False,
            preflight_message="Vision pipeline online; waiting for EKF / heartbeat",
        )

        # Warm up — discard first frames
        for _ in range(30):
            try:
                pipeline.wait_for_frames(timeout_ms=1000)
            except Exception:
                pass

        return True

    def process_frame(self):
        """Process one frame. Returns True on success."""
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=300)
        except RuntimeError:
            return False

        ir_frame = frames.get_infrared_frame(1)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not ir_frame or not depth_frame:
            return False

        gray = np.asanyarray(ir_frame.get_data())
        depth_data = np.asanyarray(depth_frame.get_data())
        now = time.time()

        # ---- ALTITUDE from depth ----
        y1, y2, x1, x2 = self.roi
        roi = depth_data[y1:y2, x1:x2]
        valid_depths = roi[roi > 0].astype(np.float64)

        if len(valid_depths) > 50:
            raw_alt = float(np.median(valid_depths)) * self.depth_scale
            if self.config.DEPTH_MIN < raw_alt < self.config.DEPTH_MAX:
                # Reject impossible jumps
                if (self.last_valid_alt > 0 and
                        abs(raw_alt - self.last_valid_alt) > self.config.DEPTH_MAX_JUMP):
                    pass  # skip outlier
                else:
                    self.depth_hist.append(raw_alt)
                    if len(self.depth_hist) >= 3:
                        self.altitude = float(np.median(list(self.depth_hist)))
                    else:
                        self.altitude = raw_alt
                    self.last_valid_alt = self.altitude

        # ---- OPTICAL FLOW on IR ----
        vel_n = 0.0
        vel_e = 0.0
        dt = 0.0
        method = "IDLE"
        n_feat = 0
        flow_vis = None  # for visualization

        if self.prev_gray is not None:
            dt = now - self.prev_time
            if 0 < dt < 0.5:
                sparse_ok = False

                # --- Sparse Lucas-Kanade ---
                if (self.prev_points is not None and
                        len(self.prev_points) >= self.config.MIN_FLOW):
                    nxt, status, err = cv2.calcOpticalFlowPyrLK(
                        self.prev_gray, gray,
                        self.prev_points, None,
                        **self.lk_params
                    )
                    if nxt is not None:
                        st = status.flatten()
                        good_prev = self.prev_points[st == 1]
                        good_next = nxt[st == 1]

                        if len(good_prev) >= self.config.MIN_FLOW:
                            dx = good_next[:, 0, 0] - good_prev[:, 0, 0]
                            dy = good_next[:, 0, 1] - good_prev[:, 0, 1]

                            # MAD outlier rejection
                            mag = np.sqrt(dx ** 2 + dy ** 2)
                            median_mag = np.median(mag)
                            mad = np.median(np.abs(mag - median_mag))
                            if mad > 1e-6:
                                mask = np.abs(mag - median_mag) < self.config.OUTLIER_MAD * mad
                                dx = dx[mask]
                                dy = dy[mask]

                            if len(dx) >= 5:
                                med_dx = float(np.median(dx))
                                med_dy = float(np.median(dy))

                                flow_alt = self.altitude if self.altitude > 0.05 else 1.0
                                vx_cam = (med_dx * flow_alt) / self.fx / dt
                                vy_cam = (med_dy * flow_alt) / self.fy / dt

                                cam_vel = np.array([-vx_cam, -vy_cam])
                                body_vel = self.rot @ cam_vel
                                vel_n = float(body_vel[1])
                                vel_e = float(body_vel[0])
                                sparse_ok = True
                                method = "SPARSE"
                                n_feat = len(good_next)

                            # Update tracked points
                            if len(good_next) < self.config.MIN_FEAT:
                                self.prev_points = cv2.goodFeaturesToTrack(
                                    gray, **self.feat_params
                                )
                                if self.prev_points is not None:
                                    cv2.cornerSubPix(
                                        gray, self.prev_points, (5, 5), (-1, -1),
                                        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
                                    )
                            else:
                                self.prev_points = good_next.reshape(-1, 1, 2)

                # --- Dense Farneback fallback ---
                if not sparse_ok and self.config.DENSE_FALLBACK:
                    flow = cv2.calcOpticalFlowFarneback(
                        self.prev_gray, gray, None,
                        0.5, 3, 15, 3, 5, 1.2, 0
                    )
                    m = int(0.2 * min(gray.shape))
                    roi_flow = flow[m:gray.shape[0] - m, m:gray.shape[1] - m]
                    med_dx = float(np.median(roi_flow[:, :, 0]))
                    med_dy = float(np.median(roi_flow[:, :, 1]))

                    flow_alt = self.altitude if self.altitude > 0.05 else 1.0
                    vx_cam = (med_dx * flow_alt) / self.fx / dt
                    vy_cam = (med_dy * flow_alt) / self.fy / dt

                    cam_vel = np.array([-vx_cam, -vy_cam])
                    body_vel = self.rot @ cam_vel
                    vel_n = float(body_vel[1])
                    vel_e = float(body_vel[0])
                    method = "DENSE"
                    n_feat = -1

                    # Save dense flow for visualization
                    flow_vis = flow

        # Re-detect features if needed
        if (self.prev_points is None or
                (self.prev_points is not None and
                 len(self.prev_points) < self.config.MIN_FEAT)):
            self.prev_points = cv2.goodFeaturesToTrack(gray, **self.feat_params)
            if self.prev_points is not None:
                cv2.cornerSubPix(
                    gray, self.prev_points, (5, 5), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
                )

        self.prev_gray = gray.copy()
        self.prev_time = now

        # ---- VELOCITY CLAMP ----
        speed = math.sqrt(vel_n ** 2 + vel_e ** 2)
        if speed > self.config.MAX_VEL:
            scale = self.config.MAX_VEL / speed
            vel_n *= scale
            vel_e *= scale

        # ---- KALMAN FILTER ----
        if dt > 0:
            self.kalman.predict(dt)
            if method in ("SPARSE", "DENSE"):
                self.kalman.update(vel_n, vel_e)

        kx = float(self.kalman.x[0])
        ky = float(self.kalman.x[1])
        kvx = float(self.kalman.x[2])
        kvy = float(self.kalman.x[3])

        # ---- SEND TO PIXHAWK ----
        self.mav.send_vision_position(kx, ky, -self.altitude)
        self.mav.send_vision_speed(kvx, kvy, 0.0) # Z velocity (down) is hard to compute, keeping to 0 or derive from depth derivative
        if self.altitude > 0:
            self.mav.send_distance(int(self.altitude * 100))
        self.mav.send_heartbeat()

        # ---- READ FROM PIXHAWK ----
        self.mav.read_messages()

        # ---- FPS ----
        self.frame_count += 1
        elapsed_fps = now - self.fps_time
        if elapsed_fps > 1.0:
            self.current_fps = self.frame_count / elapsed_fps
            self.frame_count = 0
            self.fps_time = now

        # ---- UPDATE STATE ----
        fq = int(min(n_feat / 200.0, 1.0) * 100) if n_feat > 0 else 0
        self.state.update_telem(
            viso_x=kx, viso_y=ky, viso_z=-self.altitude,
            rangefinder_dist=self.altitude,
            flow_quality=fq,
            flow_features=max(0, n_feat),
            flow_method=method,
            vision_fps=self.current_fps,
            d435i_ok=True,
            last_vision_frame=now,
        )

        # ---- VIDEO FEEDS (rate-limited) ----
        if now - self.last_web_frame >= 1.0 / self.config.WEB_FEED_FPS:
            self.last_web_frame = now

            # Color feed with HUD
            if color_frame:
                color_img = np.asanyarray(color_frame.get_data())
                self._draw_color_hud(color_img, kx, ky, kvx, kvy, method, fq)
                _, jpeg = cv2.imencode(
                    ".jpg", color_img,
                    [cv2.IMWRITE_JPEG_QUALITY, self.config.JPEG_QUALITY]
                )
                self.state.set_color_frame(
                    jpeg.tobytes(),
                    width=color_img.shape[1],
                    height=color_img.shape[0],
                    source="d435i-color",
                )

            # Optical flow visualization
            flow_img = self._draw_flow_vis(
                gray, method, n_feat, flow_vis, kx, ky, kvx, kvy
            )
            _, jpeg = cv2.imencode(
                ".jpg", flow_img,
                [cv2.IMWRITE_JPEG_QUALITY, self.config.JPEG_QUALITY]
            )
            self.state.set_flow_frame(jpeg.tobytes())

        return True

    def _draw_color_hud(self, img, kx, ky, kvx, kvy, method, fq):
        """Draw heads-up display on color frame."""
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2

        # Crosshair
        cv2.line(img, (cx - 25, cy), (cx - 8, cy), (0, 255, 0), 1)
        cv2.line(img, (cx + 8, cy), (cx + 25, cy), (0, 255, 0), 1)
        cv2.line(img, (cx, cy - 25), (cx, cy - 8), (0, 255, 0), 1)
        cv2.line(img, (cx, cy + 8), (cx, cy + 25), (0, 255, 0), 1)

        # Corner brackets
        bsz = 30
        for bx, by, dx, dy in [(0, 0, 1, 1), (w, 0, -1, 1),
                                 (0, h, 1, -1), (w, h, -1, -1)]:
            cv2.line(img, (bx, by), (bx + dx * bsz, by), (0, 255, 0), 1)
            cv2.line(img, (bx, by), (bx, by + dy * bsz), (0, 255, 0), 1)

        # Velocity arrow
        arrow_scale = 80
        ax = int(cx + kvy * arrow_scale)
        ay = int(cy - kvx * arrow_scale)
        cv2.arrowedLine(img, (cx, cy), (ax, ay), (0, 140, 255), 2)

        # Altitude bar
        if self.altitude > 0:
            bar_max_h = h - 40
            bar_h = int(min(self.altitude / 10.0, 1.0) * bar_max_h)
            cv2.rectangle(img, (w - 22, h - 20 - bar_h), (w - 8, h - 20),
                          (0, 255, 255), -1)

        # Text overlay
        color_ok = (0, 255, 0) if fq > 60 else (0, 200, 255) if fq > 30 else (0, 0, 255)
        texts = [
            f"ALT {self.altitude:.2f}m",
            f"POS ({kx:.2f}, {ky:.2f})",
            f"{method} Q:{fq}%",
            f"FPS {self.current_fps:.0f}",
        ]
        for i, txt in enumerate(texts):
            cv2.putText(img, txt, (8, 20 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_ok, 1)

    def _draw_flow_vis(self, gray, method, n_feat, dense_flow, kx, ky, kvx, kvy):
        """Create optical flow visualization image."""
        vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        h, w = vis.shape[:2]

        # Draw tracked features
        if self.prev_points is not None and method == "SPARSE":
            for pt in self.prev_points:
                x, y = int(pt[0][0]), int(pt[0][1])
                cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)

        # Draw dense flow as color wheel
        if dense_flow is not None and method == "DENSE":
            mag, ang = cv2.cartToPolar(dense_flow[:, :, 0], dense_flow[:, :, 1])
            hsv = np.zeros((h, w, 3), dtype=np.uint8)
            hsv[:, :, 0] = ang * 180 / np.pi / 2
            hsv[:, :, 1] = 255
            hsv[:, :, 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            flow_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            vis = cv2.addWeighted(vis, 0.5, flow_bgr, 0.5, 0)

        # Depth ROI box
        y1, y2, x1, x2 = self.roi
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 255), 1)

        # Info
        cv2.putText(vis, f"OPTICAL FLOW: {method}", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.putText(vis, f"Features: {n_feat}", (8, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1)
        cv2.putText(vis, f"Alt: {self.altitude:.2f}m", (8, 54),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1)
        cv2.putText(vis, f"Pos: ({kx:.3f}, {ky:.3f})", (8, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1)
        cv2.putText(vis, f"Vel: ({kvx:.3f}, {kvy:.3f})", (8, 86),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 200), 1)

        return vis

    def stop(self):
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass


# =============================================================================
# VISION LOOP THREAD
# =============================================================================
def vision_thread(config, state, mav, stop_event):
    engine = VisionEngine(config, state, mav)

    try:
        if not engine.start():
            print("[D435i] Failed to start")
            state.update_telem(
                d435i_ok=False,
                last_vision_frame=0.0,
                camera_source="unavailable",
                preflight_ready=False,
                preflight_message="Vision pipeline unavailable",
            )
            # Still run MAVLink reads without camera
            while not stop_event.is_set():
                mav.read_messages()
                mav.send_heartbeat()
                time.sleep(0.05)
            return
    except Exception as e:
        print(f"[D435i] Error: {e}")
        print("[D435i] Running in MAVLink-only mode (no vision)")
        state.update_telem(
            d435i_ok=False,
            last_vision_frame=0.0,
            camera_source="unavailable",
            preflight_ready=False,
            preflight_message="Vision pipeline unavailable",
        )
        while not stop_event.is_set():
            mav.read_messages()
            mav.send_heartbeat()
            time.sleep(0.05)
        return

    target_dt = 1.0 / config.VISION_HZ
    print(f"[VISION] Running at {config.VISION_HZ}Hz")

    while not stop_event.is_set():
        t0 = time.time()
        try:
            engine.process_frame()
        except Exception as e:
            if config.DEBUG:
                print(f"[VISION] Frame error: {e}")
            time.sleep(0.01)
            continue

        elapsed = time.time() - t0
        sleep = target_dt - elapsed
        if sleep > 0:
            time.sleep(sleep)

    engine.stop()
    print("[VISION] Stopped")


# =============================================================================
# DEMO MODE
# =============================================================================
def demo_thread(state, stop_event):
    t = 0
    while not stop_event.is_set():
        t += 0.1
        ph = 0 if t < 4 else 1 if t < 11 else 2 if t < 28 else 3 if t < 36 else 4 if t < 43 else 5
        modes = ["STABILIZE", "AUTO", "LOITER", "LOITER", "AUTO", "LAND"]
        alt = [0.08, min((t - 4) * 0.72, 5), 5 + math.sin(t * 0.4) * 0.08,
               5 + math.sin(t * 0.6) * 0.12, max(5 - (t - 36) * 0.75, 0.05), 0.05][ph]

        state.update_telem(
            pos_x=math.sin(t * 0.15) * 0.25,
            pos_y=math.cos(t * 0.12) * 0.25,
            pos_z=-alt, alt_rel=alt,
            vx=math.sin(t * 0.2) * 0.08,
            vy=math.cos(t * 0.18) * 0.06,
            vz=[0, 0.72, math.sin(t * 0.5) * 0.03, math.sin(t * 0.5) * 0.03, -0.75, 0][ph],
            roll=math.sin(t * 0.7) * 3.5,
            pitch=math.cos(t * 0.6) * 2.8,
            yaw=(t * 4.5) % 360,
            heading=(t * 4.5) % 360,
            imu_ax=math.sin(t * 2) * 0.25,
            imu_ay=math.cos(t * 1.8) * 0.2,
            imu_az=-9.79 + math.sin(t * 3) * 0.1,
            imu_gx=math.sin(t * 1.5) * 1.5,
            imu_gy=math.cos(t * 1.3) * 1.2,
            imu_gz=math.sin(t * 0.8) * 0.8,
            imu_temp=41 + math.sin(t * 0.1) * 2,
            bat_voltage=12.58 - t * 0.012,
            bat_current=[3, 16, 10, 10, 8, 2][ph] + 0.5,
            bat_soc=max(98 - t * 0.55, 12),
            bat_temp=34 + t * 0.12,
            cell_voltages=[4.18 - t * 0.004, 4.16 - t * 0.004, 4.17 - t * 0.004],
            flow_quality=80 + int(math.sin(t) * 15) if ph in (1, 2, 3, 4) else 0,
            flow_features=180 if ph in (1, 2, 3, 4) else 0,
            flow_method="SPARSE" if ph in (1, 2, 3, 4) else "IDLE",
            rangefinder_dist=alt,
            viso_x=math.sin(t * 0.15) * 0.25,
            viso_y=math.cos(t * 0.12) * 0.25,
            viso_z=-alt,
            armed=ph in (1, 2, 3, 4),
            mode=modes[ph],
            throttle=[0, 65, 45, 45, 55, 0][ph],
            climb=[0, 0.72, 0, 0, -0.75, 0][ph],
            ekf_ok=True, d435i_ok=True, pixhawk_ok=True,
            vibe_x=15 + 5 * math.sin(t),
            vibe_y=14 + 4 * math.cos(t),
            vibe_z=20 + 6 * math.sin(t * 1.3),
            vision_fps=29.5 + math.sin(t) * 0.5,
            last_heartbeat=time.time(),
            last_vision_frame=time.time(),
            camera_source="demo",
            preflight_ready=True,
            preflight_message="Demo mode active",
        )
        time.sleep(0.1)


# =============================================================================
# FLASK WEB SERVER + API
# =============================================================================
app = Flask(__name__)
CORS(app)
telemetry_state = TelemetryState()
state = telemetry_state
mav_handler = None  # set in main


def _json_response(payload, status=200):
    response = jsonify(payload)
    response.status_code = status
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    return response


@app.route("/")
def index():
    return """<!DOCTYPE html>
<html><head><title>ASCEND GCS</title>
<style>
body{margin:0;padding:20px;background:#101214;color:#e6e8eb;font-family:'Consolas','Courier New',monospace}
h1{color:#10b981;letter-spacing:3px;font-size:18px}
h3{color:#6b7280;margin:16px 0 8px;font-size:12px;letter-spacing:1px}
a{color:#60a5fa;text-decoration:none}
pre{background:#16181c;padding:12px;border-radius:4px;font-size:11px;overflow-x:auto}
.feeds{display:flex;gap:12px;flex-wrap:wrap}
.feed{border:1px solid #1f2328;border-radius:4px;overflow:hidden}
.feed img{display:block;width:320px;height:240px;background:#000}
.feed p{padding:4px 8px;font-size:10px;color:#6b7280;margin:0}
.btn{display:inline-block;padding:6px 14px;margin:4px;border:1px solid #1f2328;border-radius:4px;
     background:#16181c;color:#e6e8eb;cursor:pointer;font-family:inherit;font-size:11px}
.btn:hover{background:#1f2328}
.btn.arm{border-color:#dc2626;color:#dc2626}
.btn.arm:hover{background:#dc262620}
.btn.disarm{border-color:#10b981;color:#10b981}
.btn.disarm:hover{background:#10b98120}
.btn.mode{border-color:#60a5fa;color:#60a5fa}
#log{background:#16181c;padding:8px;border-radius:4px;font-size:10px;max-height:200px;overflow-y:auto;color:#6b7280}
#telem{background:#16181c;padding:8px;border-radius:4px;font-size:10px;max-height:300px;overflow-y:auto}
</style></head><body>
<h1>ASCEND GCS SERVER</h1>
<p style="color:#6b7280">Team Saarabai X · IRoC-U 2026 · Bennett University</p>

<h3>LIVE FEEDS</h3>
<div class="feeds">
<div class="feed"><img src="/feed/color" onerror="this.style.opacity=0.3"><p>Color Camera + HUD</p></div>
<div class="feed"><img src="/feed/flow" onerror="this.style.opacity=0.3"><p>Optical Flow Visualization</p></div>
</div>

<h3>DRONE CONTROL</h3>
<button class="btn arm" onclick="cmd('/api/arm')">ARM</button>
<button class="btn disarm" onclick="cmd('/api/disarm')">DISARM</button>
<button class="btn arm" onclick="forceCmd('/api/force_disarm')">FORCE DISARM</button>
<br>
<button class="btn mode" onclick="cmd('/api/mode/STABILIZE')">STABILIZE</button>
<button class="btn mode" onclick="cmd('/api/mode/ALT_HOLD')">ALT_HOLD</button>
<button class="btn mode" onclick="cmd('/api/mode/LOITER')">LOITER</button>
<button class="btn mode" onclick="cmd('/api/mode/POSHOLD')">POSHOLD</button>
<button class="btn mode" onclick="cmd('/api/mode/LAND')">LAND</button>
<button class="btn mode" onclick="cmd('/api/mode/RTL')">RTL</button>
<button class="btn mode" onclick="cmd('/api/mode/GUIDED')">GUIDED</button>
<br>
<button class="btn" onclick="cmd('/api/rc_release')">RELEASE RC OVERRIDE</button>

<h3>COMMAND LOG</h3>
<div id="log"></div>

<h3>LIVE TELEMETRY</h3>
<div id="telem"></div>

<script>
async function cmd(url){
  try{
    const r=await fetch(url,{method:'POST'});
    const d=await r.json();
    const log=document.getElementById('log');
    const ts=new Date().toLocaleTimeString();
    log.innerHTML=`<div>[${ts}] ${url} → ${d.ok?'OK':'FAIL'}: ${d.msg}</div>`+log.innerHTML;
  }catch(e){
    document.getElementById('log').innerHTML=`<div style="color:#dc2626">Error: ${e}</div>`+document.getElementById('log').innerHTML;
  }
}
async function forceCmd(url){
  const confirmText=window.prompt('Type FORCE to send an emergency disarm');
  if(confirmText!=='FORCE')return;
  try{
    const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({confirm:'FORCE'})});
    const d=await r.json();
    const log=document.getElementById('log');
    const ts=new Date().toLocaleTimeString();
    log.innerHTML=`<div>[${ts}] ${url} -> ${d.ok?'OK':'FAIL'}: ${d.msg}</div>`+log.innerHTML;
  }catch(e){
    document.getElementById('log').innerHTML=`<div style="color:#dc2626">Error: ${e}</div>`+document.getElementById('log').innerHTML;
  }
}
setInterval(async()=>{
  try{
    const r=await fetch('/api/telemetry');
    const d=await r.json();
    const el=document.getElementById('telem');
    let s='';
    const important=['armed','mode','alt_rel','pos_x','pos_y','vx','vy','vz',
                     'bat_voltage','bat_soc','bat_current','flow_method','flow_quality',
                     'vision_fps','ekf_ok','d435i_ok','pixhawk_ok','preflight_message','heading','throttle',
                     'roll','pitch','yaw','rangefinder_dist','vibe_x','vibe_y','vibe_z'];
    for(const k of important){
      let v=d[k];
      if(typeof v==='number')v=v.toFixed(3);
      let c='#6b7280';
      if(k==='armed')c=v==='true'||v===true?'#10b981':'#dc2626';
      if(k==='mode')c='#60a5fa';
      if(k==='bat_soc')c=parseFloat(v)>50?'#10b981':parseFloat(v)>25?'#eab308':'#dc2626';
      s+=`<span style="color:${c}">${k}: ${v}</span>  `;
    }
    el.innerHTML=s;
  }catch{}
},200);
</script>
</body></html>"""


@app.route("/api/telemetry")
def api_telemetry():
    return _json_response(telemetry_state.get())


@app.route("/api/camera/latest.jpg")
def api_camera_latest():
    frame = telemetry_state.get_camera_frame()
    if frame is None:
        return Response(
            b"Camera frame unavailable",
            status=503,
            mimetype="text/plain",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0"},
        )
    return Response(
        frame,
        mimetype="image/jpeg",
        headers={"Cache-Control": "no-store, no-cache, must-revalidate, max-age=0"},
    )


@app.route("/api/arm", methods=["POST"])
def api_arm():
    if mav_handler:
        ok, msg = mav_handler.arm()
        return _json_response({"ok": ok, "msg": msg})
    return _json_response({"ok": False, "msg": "No MAVLink connection"})


@app.route("/api/disarm", methods=["POST"])
def api_disarm():
    if mav_handler:
        ok, msg = mav_handler.disarm()
        return _json_response({"ok": ok, "msg": msg})
    return _json_response({"ok": False, "msg": "No MAVLink connection"})


@app.route("/api/force_disarm", methods=["POST"])
def api_force_disarm():
    data = request.get_json(silent=True) or {}
    confirm = data.get("confirm") or request.args.get("confirm", "")
    if confirm != "FORCE":
        return _json_response(
            {"ok": False, "msg": "Confirmation required: send {'confirm': 'FORCE'}"},
            status=400,
        )
    if mav_handler:
        ok, msg = mav_handler.force_disarm()
        return _json_response({"ok": ok, "msg": msg})
    return _json_response({"ok": False, "msg": "No MAVLink connection"})


@app.route("/api/takeoff", methods=["POST"])
def api_takeoff():
    data = request.get_json(silent=True) or {}
    altitude = data.get("altitude", 4.0)
    if mav_handler:
        ok, msg = mav_handler.takeoff(altitude)
        return _json_response({"ok": ok, "msg": msg})
    return _json_response({"ok": False, "msg": "No MAVLink connection"})


@app.route("/api/mode/<mode_name>", methods=["POST"])
def api_set_mode(mode_name):
    if mav_handler:
        ok, msg = mav_handler.set_mode(mode_name)
        return _json_response({"ok": ok, "msg": msg})
    return _json_response({"ok": False, "msg": "No MAVLink connection"})


@app.route("/api/rc", methods=["POST"])
def api_rc_override():
    """POST JSON: {"1": 1500, "2": 1500, "3": 1200, "4": 1500}"""
    if mav_handler:
        data = request.get_json(silent=True) or {}
        ok, msg = mav_handler.set_rc_override(data)
        return _json_response({"ok": ok, "msg": msg})
    return _json_response({"ok": False, "msg": "No MAVLink connection"})


@app.route("/api/rc_release", methods=["POST"])
def api_rc_release():
    if mav_handler:
        ok, msg = mav_handler.release_rc_override()
        return _json_response({"ok": ok, "msg": msg})
    return _json_response({"ok": False, "msg": "No MAVLink connection"})


def _mjpeg_stream(frame_getter):
    """Generate MJPEG stream from a frame getter function."""
    while True:
        frame = frame_getter()
        if frame is not None:
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" +
                   frame + b"\r\n")
        time.sleep(1.0 / Config.WEB_FEED_FPS)


@app.route("/feed/color")
def feed_color():
    return Response(
        _mjpeg_stream(telemetry_state.get_color_frame),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/feed/flow")
def feed_flow():
    return Response(
        _mjpeg_stream(telemetry_state.get_flow_frame),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# =============================================================================
# MAIN
# =============================================================================
def main():
    global mav_handler, state, telemetry_state

    parser = argparse.ArgumentParser(description="ASCEND GCS — Complete Ground Control")
    parser.add_argument("--serial", default=Config.SERIAL_PORT,
                        help=f"Pixhawk serial port (default: {Config.SERIAL_PORT})")
    parser.add_argument("--baud", type=int, default=57600)
    parser.add_argument("--port", type=int, default=5000, help="Web server port")
    parser.add_argument("--yaw", type=float, default=0, help="Camera yaw degrees")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--demo", action="store_true", help="Run with simulated data")
    parser.add_argument("--dense-fallback", action="store_true",
                        help="Enable dense Farneback fallback only when sparse flow fails")
    parser.add_argument("--allow-auto-mode", action="store_true",
                        help="Allow AUTO mode changes from the web API")
    parser.add_argument("--allow-unsafe-modes", action="store_true",
                        help="Allow risky web mode changes such as AUTOTUNE / FLIP")
    args = parser.parse_args()

    config = Config()
    config.SERIAL_PORT = args.serial
    config.BAUD_RATE = args.baud
    config.WEB_PORT = args.port
    config.CAM_YAW = args.yaw
    config.DEBUG = args.debug
    config.DEMO = args.demo
    config.DENSE_FALLBACK = args.dense_fallback
    config.ALLOW_AUTO_MODE = args.allow_auto_mode
    config.ALLOW_UNSAFE_MODES = args.allow_unsafe_modes
    telemetry_state.HEARTBEAT_STALE_SEC = config.HEARTBEAT_TIMEOUT
    telemetry_state.VISION_STALE_SEC = config.VISION_TIMEOUT
    telemetry_state.CAMERA_STALE_SEC = config.CAMERA_TIMEOUT
    state = telemetry_state

    stop_event = threading.Event()

    def shutdown(sig, frame):
        print("\nShutting down...")
        stop_event.set()
        time.sleep(0.5)
        sys.exit(0)
    signal.signal(signal.SIGINT, shutdown)

    print()
    print("=" * 60)
    print("  ASCEND GCS — Ground Control System")
    print("  Team Saarabai X · IRoC-U 2026")
    print("  Bennett University · Technotix")
    print("=" * 60)
    print()

    if args.demo:
        print("[MODE] Demo — simulated telemetry, no hardware needed")
        print()
        threading.Thread(
            target=demo_thread, args=(state, stop_event), daemon=True
        ).start()
    else:
        # Connect to Pixhawk
        mav_handler = MAVLinkHandler(config, state)
        if not mav_handler.connect():
            print()
            print("[FATAL] Cannot connect to Pixhawk.")
            print(f"[FATAL] Tried: {config.SERIAL_PORT} @ {config.BAUD_RATE}")
            if os.name == "nt":
                print("[HINT] Windows: Check COM port in Device Manager")
                print("[HINT] Example: python ascend_gcs.py --serial COM13")
            else:
                print("[HINT] Linux: ls /dev/ttyACM* /dev/ttyUSB*")
            sys.exit(1)

        # Start vision + MAVLink thread
        threading.Thread(
            target=vision_thread,
            args=(config, state, mav_handler, stop_event),
            daemon=True,
        ).start()

    print(f"  Dashboard:   http://localhost:{config.WEB_PORT}")
    print(f"  Telemetry:   http://localhost:{config.WEB_PORT}/api/telemetry")
    print(f"  Color feed:  http://localhost:{config.WEB_PORT}/feed/color")
    print(f"  Flow feed:   http://localhost:{config.WEB_PORT}/feed/flow")
    print()
    print("  Controls: ARM, DISARM, MODE buttons on dashboard")
    print("  Web safety: AUTO disabled by default, FORCE DISARM requires confirmation")
    print("  Press Ctrl+C to stop")
    print()

    # Start Flask
    app.run(host="0.0.0.0", port=config.WEB_PORT,
            threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
