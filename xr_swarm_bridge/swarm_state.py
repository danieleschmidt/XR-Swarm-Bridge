"""Swarm state management for XR-Swarm-Bridge."""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class Robot:
    """Represents a single robot in the swarm."""
    id: int
    position: tuple  # (x, y, z)
    velocity: tuple  # (vx, vy, vz)
    assignment: Optional[str] = None


class SwarmState:
    """Manages the collective state of all robots in the swarm."""

    def __init__(self, num_robots: int):
        """Initialize swarm with robots at random positions.

        Args:
            num_robots: Number of robots to initialize.
        """
        self._robots: dict[int, Robot] = {}
        for i in range(num_robots):
            position = (
                random.uniform(-10.0, 10.0),
                random.uniform(-10.0, 10.0),
                random.uniform(0.0, 5.0),
            )
            velocity = (0.0, 0.0, 0.0)
            self._robots[i] = Robot(id=i, position=position, velocity=velocity)

    def get_robot(self, robot_id: int) -> Robot:
        """Return the Robot with the given id.

        Args:
            robot_id: The robot's id.

        Returns:
            Robot object.

        Raises:
            KeyError: If robot_id does not exist.
        """
        if robot_id not in self._robots:
            raise KeyError(f"Robot {robot_id} not found")
        return self._robots[robot_id]

    def update_position(self, robot_id: int, position: tuple) -> None:
        """Update a robot's position.

        Args:
            robot_id: The robot's id.
            position: New (x, y, z) position tuple.
        """
        robot = self.get_robot(robot_id)
        self._robots[robot_id] = Robot(
            id=robot.id,
            position=position,
            velocity=robot.velocity,
            assignment=robot.assignment,
        )

    def update_velocity(self, robot_id: int, velocity: tuple) -> None:
        """Update a robot's velocity.

        Args:
            robot_id: The robot's id.
            velocity: New (vx, vy, vz) velocity tuple.
        """
        robot = self.get_robot(robot_id)
        self._robots[robot_id] = Robot(
            id=robot.id,
            position=robot.position,
            velocity=velocity,
            assignment=robot.assignment,
        )

    def assign_task(self, robot_id: int, task: str) -> None:
        """Assign a task to a robot.

        Args:
            robot_id: The robot's id.
            task: Task identifier string.
        """
        robot = self.get_robot(robot_id)
        self._robots[robot_id] = Robot(
            id=robot.id,
            position=robot.position,
            velocity=robot.velocity,
            assignment=task,
        )

    def to_dict(self) -> dict:
        """Serialize swarm state to a dictionary.

        Returns:
            Dict mapping robot_id to robot state dict.
        """
        return {
            robot_id: {
                "id": robot.id,
                "position": robot.position,
                "velocity": robot.velocity,
                "assignment": robot.assignment,
            }
            for robot_id, robot in self._robots.items()
        }

    def unassigned_robots(self) -> list:
        """Return list of robots without a current assignment.

        Returns:
            List of Robot objects with assignment == None.
        """
        return [r for r in self._robots.values() if r.assignment is None]
