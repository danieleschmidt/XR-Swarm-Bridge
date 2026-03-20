"""Causal trace recorder for XR-Swarm-Bridge command history."""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any


@dataclass
class TraceEntry:
    """A single entry in the causal trace log."""
    timestamp: float
    robot_id: int
    command: dict
    outcome: dict
    metadata: dict = field(default_factory=dict)


class CausalTraceRecorder:
    """Records and replays the causal history of robot commands."""

    def __init__(self):
        """Initialize an empty trace recorder."""
        self._entries: list[TraceEntry] = []

    def record(self, robot_id: int, command: dict, outcome: dict, metadata: dict = None) -> None:
        """Record a command and its outcome.

        Args:
            robot_id: The robot that received the command.
            command: The command payload dict.
            outcome: The result/outcome dict.
            metadata: Optional additional metadata dict.
        """
        entry = TraceEntry(
            timestamp=time.time(),
            robot_id=robot_id,
            command=command,
            outcome=outcome,
            metadata=metadata or {},
        )
        self._entries.append(entry)

    def rewind(self, steps: int) -> list:
        """Return the last N entries in reverse chronological order.

        Args:
            steps: Number of most recent entries to return.

        Returns:
            List of TraceEntry objects, newest first.
        """
        return list(reversed(self._entries[-steps:])) if steps > 0 else []

    def get_history(self, robot_id: int) -> list:
        """Return all entries for a specific robot.

        Args:
            robot_id: The robot whose history to retrieve.

        Returns:
            List of TraceEntry objects for that robot, in chronological order.
        """
        return [e for e in self._entries if e.robot_id == robot_id]

    def to_json(self) -> str:
        """Serialize the full trace to a JSON string.

        Returns:
            JSON-encoded string of all trace entries.
        """
        data = [
            {
                "timestamp": e.timestamp,
                "robot_id": e.robot_id,
                "command": e.command,
                "outcome": e.outcome,
                "metadata": e.metadata,
            }
            for e in self._entries
        ]
        return json.dumps(data, indent=2)

    def save(self, path: str) -> None:
        """Save the trace to a JSON file.

        Args:
            path: File path to write the JSON trace to.
        """
        with open(path, "w") as f:
            f.write(self.to_json())

    def clear(self) -> None:
        """Clear all recorded trace entries."""
        self._entries.clear()
