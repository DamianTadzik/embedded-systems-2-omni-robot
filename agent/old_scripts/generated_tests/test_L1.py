import pytest
from unittest.mock import patch, MagicMock
from datetime import datetime
import numpy as np
def test_omni4_inverse_kinematics():
    # Normal case
    wheel_cmds, _ = omni4_inverse_kinematics(0.1, 0.2, 0.3, v_max=0.5, omega_max=25.0)
    assert len(wheel_cmds) == 4
    # Edge case: vx, vy, omega are zero
    wheel_cmds, _ = omni4_inverse_kinematics(0.0, 0.0, 0.0, v_max=0.5, omega_max=25.0)
    assert all(cmd[0] == 0 for cmd in wheel_cmds)
    # Edge case: max_omega_val exceeds omega_max
    wheel_cmds, _ = omni4_inverse_kinematics(0.1, 0.2, 100.0, v_max=0.5, omega_max=25.0)
    assert all(cmd[0] <= 255 for cmd in wheel_cmds)
def test_on_connect():
    client = MagicMock()
    userdata = None
    flags = None
    rc = 0
    on_connect(client, userdata, flags, rc)
    assert client.subscribe.called_once_with("robot/cmd_vel")
def test_on_message_valid_data():
    global vx, vy, omega
    msg = MagicMock()
    msg.topic = "robot/cmd_vel"
    msg.payload.decode.return_value = '{"vx": 0.1, "vy": 0.2, "omega": 0.3}'
    on_message(None, None, msg)
    assert vx == 0.1
    assert vy == 0.2
    assert omega == 0.3
def test_on_message_invalid_data():
    global vx, vy, omega
    msg = MagicMock()
    msg.topic = "robot/cmd_vel"
    msg.payload.decode.return_value = '{"vx": "a", "vy": 0.2, "omega": 0.3}'
    on_message(None, None, msg)
    assert vx == 0.0
    assert vy == 0.2
    assert omega == 0.3
def test_control_action():
    wheel_cmds = [(128, 1), (64, 0), (192, 1), (127, 0)]
    control_action(wheel_cmds)
    expected_command = [255, 1, 212, 100, 254, 100, 384, 100]
    assert ser.write.call_args[0][0] == bytearray(expected_command)
def test_main_loop():
    with patch('time.sleep') as mock_sleep:
        with patch('omni4_inverse_kinematics', return_value=([], [])):
            with patch('control_action'):
                main()
                mock_sleep.assert_called_with(1.0 / UPDATE_RATE_HZ)