import json
import time
import unittest
import sys
from types import SimpleNamespace
from unittest import mock

sys.modules.setdefault("cv2", mock.MagicMock())

import ascend_unified


class TelemetryStateTests(unittest.TestCase):
    def test_get_sanitizes_non_finite_values_and_marks_stale_links(self):
        state = ascend_unified.TelemetryState()
        state.update(
            bat_voltage=float("nan"),
            last_heartbeat=time.time() - 10,
            last_vision_frame=time.time() - 5,
        )

        snapshot = state.get()

        self.assertIsNone(snapshot["bat_voltage"])
        self.assertFalse(snapshot["pixhawk_ok"])
        self.assertFalse(snapshot["d435i_ok"])
        self.assertFalse(snapshot["camera_feed_ok"])


class FlaskRouteTests(unittest.TestCase):
    def test_api_telemetry_returns_strict_json(self):
        state = ascend_unified.TelemetryState()
        state.update(pos_x=1.0, bat_voltage=float("nan"))

        with mock.patch.object(ascend_unified, "telemetry_state", state):
            client = ascend_unified.app.test_client()
            response = client.get("/api/telemetry")

        self.assertEqual(response.status_code, 200)
        self.assertIn("no-store", response.headers["Cache-Control"])
        payload = json.loads(response.data.decode("utf-8"))
        self.assertEqual(payload["pos_x"], 1.0)
        self.assertIsNone(payload["bat_voltage"])

    def test_camera_latest_reports_unavailable_until_frame_exists(self):
        state = ascend_unified.TelemetryState()

        with mock.patch.object(ascend_unified, "telemetry_state", state):
            client = ascend_unified.app.test_client()
            missing = client.get("/api/camera/latest.jpg")
            self.assertEqual(missing.status_code, 503)

            state.set_camera_frame(b"jpeg-bytes", 320, 240, source="demo")
            present = client.get("/api/camera/latest.jpg")

        self.assertEqual(present.status_code, 200)
        self.assertEqual(present.mimetype, "image/jpeg")
        self.assertEqual(present.data, b"jpeg-bytes")


class MAVLinkHandlerTests(unittest.TestCase):
    def setUp(self):
        self.state = ascend_unified.TelemetryState()
        self.handler = ascend_unified.MAVLinkHandler(ascend_unified.Config(), self.state)
        self.handler.mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_MODE_FLAG_SAFETY_ARMED=0b1000)
        )

    def test_process_message_updates_core_telemetry(self):
        heartbeat = SimpleNamespace(
            get_type=lambda: "HEARTBEAT",
            base_mode=0b1000,
            custom_mode=5,
        )
        local_position = SimpleNamespace(
            get_type=lambda: "LOCAL_POSITION_NED",
            x=1.2,
            y=-0.4,
            z=-2.0,
            vx=0.1,
            vy=0.2,
            vz=-0.3,
        )
        raw_imu = SimpleNamespace(
            get_type=lambda: "RAW_IMU",
            xacc=1000,
            yacc=0,
            zacc=-1000,
            xgyro=1000,
            ygyro=0,
            zgyro=-1000,
        )
        battery = SimpleNamespace(
            get_type=lambda: "BATTERY_STATUS",
            voltages=[4010, 4000, 3990, 65535],
            temperature=2550,
        )

        for message in (heartbeat, local_position, raw_imu, battery):
            self.handler._process_message(message)

        snapshot = self.state.get()
        self.assertTrue(snapshot["armed"])
        self.assertEqual(snapshot["mode"], "LOITER")
        self.assertAlmostEqual(snapshot["alt_rel"], 2.0, places=3)
        self.assertAlmostEqual(snapshot["imu_ax"], 9.81, places=3)
        self.assertAlmostEqual(snapshot["imu_gx"], 57.2958, places=3)
        self.assertEqual(snapshot["cell_voltages"], [4.01, 4.0, 3.99])
        self.assertAlmostEqual(snapshot["bat_temp"], 25.5, places=3)


if __name__ == "__main__":
    unittest.main()
