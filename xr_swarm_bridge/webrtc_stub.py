"""WebRTC stub — mock signaling layer for XR-Swarm-Bridge."""
from __future__ import annotations

import random
import time


class WebRTCStub:
    """Mock WebRTC signaling layer that simulates network conditions."""

    def __init__(self, latency_ms: float = 20.0, packet_loss_rate: float = 0.01):
        """Initialize the WebRTC stub.

        Args:
            latency_ms: Simulated one-way latency in milliseconds.
            packet_loss_rate: Probability [0, 1] that a packet is dropped.
        """
        self.latency_ms = latency_ms
        self.packet_loss_rate = packet_loss_rate

    def send_command(self, robot_id: int, command: dict) -> dict:
        """Simulate sending a command to a robot over WebRTC.

        Adds simulated latency and randomly drops packets based on loss rate.

        Args:
            robot_id: Target robot id.
            command: Command payload dict.

        Returns:
            Dict with keys:
                - delivered (bool): Whether the packet was delivered.
                - latency_ms (float): Simulated latency in milliseconds.
                - robot_id (int): Target robot id.
        """
        # Simulate network latency
        time.sleep(self.latency_ms / 1000.0)

        # Simulate packet loss
        delivered = random.random() >= self.packet_loss_rate

        return {
            "delivered": delivered,
            "latency_ms": self.latency_ms,
            "robot_id": robot_id,
        }

    def broadcast(self, command: dict, robot_ids: list) -> list:
        """Broadcast a command to multiple robots.

        Args:
            command: Command payload dict.
            robot_ids: List of robot ids to send command to.

        Returns:
            List of result dicts from send_command for each robot.
        """
        return [self.send_command(robot_id, command) for robot_id in robot_ids]
