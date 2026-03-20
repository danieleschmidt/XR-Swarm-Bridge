"""Natural language command parser for XR-Swarm-Bridge."""
from __future__ import annotations

import re


class NLCommandParser:
    """Parses natural language robot commands using regex patterns."""

    # Pattern: "move robot {id} to {x},{y}" or
    #          "move robot {id} to position {x},{y}" or
    #          "move robot {id} to {x},{y},{z}"
    _MOVE_PATTERN = re.compile(
        r"move\s+robot\s+(\d+)\s+to\s+(?:position\s+)?"
        r"(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)(?:,\s*(-?\d+(?:\.\d+)?))?",
        re.IGNORECASE,
    )

    # Pattern: "stop robot {id}"
    _STOP_PATTERN = re.compile(
        r"stop\s+robot\s+(\d+)",
        re.IGNORECASE,
    )

    # Pattern: "assign robot {id} to {task}"
    _ASSIGN_PATTERN = re.compile(
        r"assign\s+robot\s+(\d+)\s+to\s+(.+)",
        re.IGNORECASE,
    )

    # Pattern: "status of robot {id}"
    _STATUS_PATTERN = re.compile(
        r"status\s+of\s+robot\s+(\d+)",
        re.IGNORECASE,
    )

    def parse(self, text: str) -> dict:
        """Parse a natural language command string.

        Args:
            text: Natural language command string.

        Returns:
            Dict with keys:
                - action (str): One of 'move', 'stop', 'assign', 'status'.
                - robot_id (int): Target robot id.
                - params (dict): Action-specific parameters.

        Raises:
            ValueError: If the command is not recognized.
        """
        text = text.strip()

        # Try move
        m = self._MOVE_PATTERN.search(text)
        if m:
            robot_id = int(m.group(1))
            x = float(m.group(2))
            y = float(m.group(3))
            z = float(m.group(4)) if m.group(4) is not None else 0.0
            return {
                "action": "move",
                "robot_id": robot_id,
                "params": {"position": (x, y, z)},
            }

        # Try stop
        m = self._STOP_PATTERN.search(text)
        if m:
            robot_id = int(m.group(1))
            return {
                "action": "stop",
                "robot_id": robot_id,
                "params": {},
            }

        # Try assign
        m = self._ASSIGN_PATTERN.search(text)
        if m:
            robot_id = int(m.group(1))
            task = m.group(2).strip()
            return {
                "action": "assign",
                "robot_id": robot_id,
                "params": {"task": task},
            }

        # Try status
        m = self._STATUS_PATTERN.search(text)
        if m:
            robot_id = int(m.group(1))
            return {
                "action": "status",
                "robot_id": robot_id,
                "params": {},
            }

        raise ValueError(f"Unrecognized command: {text!r}")
