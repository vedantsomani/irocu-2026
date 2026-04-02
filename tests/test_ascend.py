import time
import unittest
import sys
from types import SimpleNamespace
from unittest import mock

class _DummyFlaskApp:
    def route(self, *_args, **_kwargs):
        def decorator(func):
            return func
        return decorator

    def run(self, *_args, **_kwargs):
        return None


class _DummyFlaskModule:
    Flask = staticmethod(lambda *_args, **_kwargs: _DummyFlaskApp())
    Response = object
    request = SimpleNamespace(get_json=lambda silent=True: {}, args={})
    jsonify = staticmethod(lambda *args, **kwargs: {"args": args, "kwargs": kwargs})


class _DummyCorsModule:
    CORS = staticmethod(lambda *_args, **_kwargs: None)


sys.modules.setdefault("cv2", mock.MagicMock())
sys.modules.setdefault("flask", _DummyFlaskModule())
sys.modules.setdefault("flask_cors", _DummyCorsModule())

import ascend


class FakeConnection:
    def __init__(self):
        self.target_system = 1
        self.target_component = 1
        self.mav = SimpleNamespace(
            rc_channels_override_send=mock.Mock(),
            set_position_target_local_ned_send=mock.Mock(),
            heartbeat_send=mock.Mock(),
            set_gps_global_origin_send=mock.Mock(),
            set_home_position_send=mock.Mock(),
            command_long_send=mock.Mock(),
            set_mode_send=mock.Mock(),
        )


class MAVLinkHandlerTakeoffTests(unittest.TestCase):
    def setUp(self):
        self.state = ascend.TelemetryState()
        self.handler = ascend.MAVLinkHandler(ascend.Config(), self.state)
        self.handler.connected = True
        self.handler.conn = FakeConnection()
        self.handler.mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(
                MAV_MODE_FLAG_SAFETY_ARMED=0b1000,
                MAV_TYPE_ONBOARD_CONTROLLER=1,
                MAV_AUTOPILOT_INVALID=0,
                MAV_FRAME_LOCAL_NED=1,
                MAV_CMD_NAV_TAKEOFF=22,
                MAV_CMD_DO_SET_MODE=176,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED=1,
            )
        )
        self.state.update(
            pixhawk_ok=True,
            armed=True,
            alt_rel=0.0,
            mode="GUIDED",
            last_heartbeat=time.time(),
        )

    def test_send_heartbeat_uses_takeoff_assist_until_altitude_is_reached(self):
        self.handler.active_rc_override = {"1": 1500, "2": 1500, "3": 1500, "4": 1500}
        self.handler._set_takeoff_assist(1650, 5.0, 0.75)
        self.handler.last_hb_send = time.time()

        self.handler.send_heartbeat()

        sent = self.handler.conn.mav.rc_channels_override_send.call_args_list[-1][0]
        self.assertEqual(sent[4], 1650)

        self.state.update(alt_rel=1.0)
        self.handler.last_rc_target_send = 0
        self.handler.send_heartbeat()

        sent = self.handler.conn.mav.rc_channels_override_send.call_args_list[-1][0]
        self.assertEqual(sent[4], 1500)
        self.assertIsNone(self.handler.takeoff_assist_pwm)

    def test_takeoff_aborts_if_guided_mode_change_fails(self):
        self.state.update(mode="STABILIZE")

        with mock.patch.object(self.handler, "set_mode", return_value=(False, "EKF not ready")):
            ok, msg = self.handler.takeoff(2.0)

        self.assertFalse(ok)
        self.assertIn("Takeoff aborted", msg)
        self.handler.conn.mav.command_long_send.assert_not_called()

    def test_takeoff_sets_target_and_assist_on_ack(self):
        self.state.update(viso_x=1.25, viso_y=-0.5)

        with mock.patch.object(self.handler, "_wait_for_command_ack", return_value=SimpleNamespace(result=0)):
            ok, msg = self.handler.takeoff(2.0)

        self.assertTrue(ok)
        self.assertIn("launch assist active", msg)
        self.assertEqual(self.handler.target_z, 2.0)
        self.assertEqual(self.handler.target_x, 1.25)
        self.assertEqual(self.handler.target_y, -0.5)
        self.assertEqual(self.handler.guidance_mode, "HOVER")
        self.assertEqual(
            self.handler.takeoff_assist_pwm,
            self.handler.config.RC_TAKEOFF_ASSIST_THROTTLE,
        )

    def test_set_rc_override_stores_sanitized_values_for_replay(self):
        ok, msg = self.handler.set_rc_override({"1": "1490", "3": "2500"})

        self.assertTrue(ok, msg)
        self.assertEqual(self.handler.active_rc_override["1"], 1490)
        self.assertEqual(self.handler.active_rc_override["3"], 2000)

    def test_hover_engages_guidance_using_realsense_altitude(self):
        self.state.update(
            d435i_ok=True,
            last_vision_frame=time.time(),
            rangefinder_dist=1.6,
            alt_rel=1.55,
            viso_x=0.4,
            viso_y=-0.2,
        )

        ok, msg = self.handler.hover()

        self.assertTrue(ok, msg)
        self.assertIn("Hover hold active", msg)
        self.assertEqual(self.handler.guidance_mode, "HOVER")
        self.assertAlmostEqual(self.handler.guidance_target_altitude, 1.6, places=3)
        self.assertAlmostEqual(self.handler.target_x, 0.4, places=3)
        self.assertAlmostEqual(self.handler.target_y, -0.2, places=3)

    def test_soft_land_gradually_reduces_guidance_target(self):
        self.state.update(d435i_ok=True, last_vision_frame=time.time(), rangefinder_dist=2.0, alt_rel=2.0)

        ok, msg = self.handler.land()

        self.assertTrue(ok, msg)
        self.assertEqual(self.handler.guidance_mode, "LAND")
        initial_target = self.handler.target_z
        self.handler.guidance_last_update = time.time() - 0.2
        self.handler.last_hb_send = time.time()
        self.handler.last_rc_target_send = 0

        self.handler.send_heartbeat()

        self.assertLess(self.handler.target_z, initial_target)
        sent = self.handler.conn.mav.set_position_target_local_ned_send.call_args_list[-1][0]
        self.assertAlmostEqual(sent[7], -self.handler.target_z, places=3)

    def test_set_mode_non_guided_clears_guidance_target(self):
        self.state.update(
            d435i_ok=True,
            ekf_ok=True,
            last_vision_frame=time.time(),
            rangefinder_dist=1.5,
            alt_rel=1.5,
        )
        self.handler._engage_guidance(self.state.get(), mode="HOVER")

        with mock.patch.object(self.handler, "_wait_for_command_ack", return_value=SimpleNamespace(result=0)):
            ok, msg = self.handler.set_mode("LOITER")

        self.assertTrue(ok, msg)
        self.assertEqual(self.handler.guidance_mode, "IDLE")
        self.assertIsNone(self.handler.target_z)

    def test_rc_override_falls_back_to_legacy_8_channel_signature(self):
        calls = []

        def legacy_only(*args):
            calls.append(args)
            if len(args) == 20:
                raise TypeError("legacy binding only supports 8 channels")
            return None

        self.handler.conn.mav.rc_channels_override_send = legacy_only

        ok, msg = self.handler.set_rc_override({"1": 1500, "2": 1500, "3": 1500, "4": 1500})

        self.assertTrue(ok, msg)
        self.assertEqual(self.handler.rc_override_channel_count, 8)
        self.assertEqual(len(calls), 2)
        self.assertEqual(len(calls[0]), 20)
        self.assertEqual(len(calls[1]), 10)


if __name__ == "__main__":
    unittest.main()
