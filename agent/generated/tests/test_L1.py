import pytest
from unittest.mock import patch, MagicMock
import numpy as np
# Mock the inverse kinematics function for testing
@patch('L1.omni4_inverse_kinematics')
def test_omni4_inverse_kinematics(mock_inverse_kinematics):
    # Normal case
    mock_inverse_kinematics.return_value = [(255, 1), (255, 1), (255, 1), (255, 1)]
    result = omni4_inverse_kinematics(0.1, 0.1, 0.1, 0.5, 25.0)
    assert result == [(255, 1), (255, 1), (255, 1), (255, 1)]
    # Edge case: max linear velocity
    mock_inverse_kinematics.return_value = [(255, 1), (255, 1), (255, 1), (255, 1)]
    result = omni4_inverse_kinematics(0.5, 0.5, 0.5, 0.5, 25.0)
    assert result == [(255, 1), (255, 1), (255, 1), (255, 1)]
    # Invalid input: negative linear velocity
    with pytest.raises(AssertionError):
        omni4_inverse_kinematics(-0.1, 0.1, 0.1, 0.5, 25.0)
# Mock the MQTT client for testing
@patch('L1.mqtt.Client')
def test_mqtt_subscription(mock_client):
    mock_client.return_value.on_connect = MagicMock()
    mock_client.return_value.on_message = MagicMock()
    # Normal case: receiving valid control command
    mock_client.return_value.loop_start()
    mock_client.return_value.on_message.reset_mock()
    mock_client.return_value.on_message.side_effect = lambda client, userdata, msg: setattr(userdata, 'vx', 0.1)
    client = mock_client.return_value
    client.subscribe("robot/cmd_vel")
    client.on_message(None, None, MagicMock(payload=json.dumps({"vx": 0.1})))
    assert vx == 0.1
    # Edge case: receiving invalid control command
    client.on_message(None, None, MagicMock(payload='invalid'))
    assert vx == 0.1
# Mock the serial communication for testing
@patch('L1.serial.Serial')
def test_control_action(mock_serial):
    mock_serial.return_value.write = MagicMock()
    # Normal case: sending valid control command
    wheel_cmds = [(255, 1), (255, 1), (255, 1), (255, 1)]
    control_action(wheel_cmds)
    assert mock_serial.return_value.write.call_args[0][0] == bytearray([255, 1, 255, 255, 255, 255])
    # Edge case: sending command with negative speeds
    wheel_cmds = [(-100, 1), (-100, 1), (-100, 1), (-100, 1)]
    control_action(wheel_cmds)
    assert mock_serial.return_value.write.call_args[0][0] == bytearray([255, 1, 0, 0, 0, 0])
# Mock the UART communication for testing
@patch('L1.serial.Serial')
def test_uart_communication(mock_serial):
    mock_serial.return_value.write = MagicMock()
    # Normal case: sending valid control command
    wheel_cmds = [(255, 1), (255, 1), (255, 1), (255, 1)]
    control_action(wheel_cmds)
    assert mock_serial.return_value.write.call_args[0][0] == bytearray([255, 1, 255, 255, 255, 255])
    # Edge case: sending command with negative speeds
    wheel_cmds = [(-100, 1), (-100, 1), (-100, 1), (-100, 1)]
    control_action(wheel_cmds)
    assert mock_serial.return_value.write.call_args[0][0] == bytearray([255, 1, 0, 0, 0, 0])