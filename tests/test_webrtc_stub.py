"""Tests for WebRTCStub — uses monkeypatching to avoid real sleeps."""
import pytest
from unittest.mock import patch
from xr_swarm_bridge.webrtc_stub import WebRTCStub


@pytest.fixture
def stub():
    """WebRTC stub with tiny latency for speed."""
    return WebRTCStub(latency_ms=0.001, packet_loss_rate=0.0)


def test_send_command_delivered(stub):
    with patch("time.sleep"):
        result = stub.send_command(1, {"action": "goto"})
    assert result["delivered"] is True
    assert result["robot_id"] == 1
    assert result["latency_ms"] == 0.001


def test_send_command_structure(stub):
    with patch("time.sleep"):
        result = stub.send_command(3, {"action": "stop"})
    assert "delivered" in result
    assert "latency_ms" in result
    assert "robot_id" in result


def test_send_command_calls_sleep():
    stub = WebRTCStub(latency_ms=5.0, packet_loss_rate=0.0)
    with patch("time.sleep") as mock_sleep:
        stub.send_command(0, {})
    mock_sleep.assert_called_once_with(0.005)


def test_send_command_packet_loss_always_drop():
    stub = WebRTCStub(latency_ms=0.001, packet_loss_rate=1.0)
    with patch("time.sleep"):
        result = stub.send_command(0, {})
    assert result["delivered"] is False


def test_broadcast_returns_list(stub):
    with patch("time.sleep"):
        results = stub.broadcast({"action": "stop"}, [0, 1, 2])
    assert isinstance(results, list)
    assert len(results) == 3


def test_broadcast_each_robot_id(stub):
    with patch("time.sleep"):
        results = stub.broadcast({"action": "stop"}, [4, 5, 6])
    ids = [r["robot_id"] for r in results]
    assert ids == [4, 5, 6]


def test_broadcast_empty_list(stub):
    with patch("time.sleep"):
        results = stub.broadcast({"action": "noop"}, [])
    assert results == []
