"""Command planning for XR-Swarm-Bridge — greedy task assignment."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from xr_swarm_bridge.swarm_state import SwarmState


@dataclass
class Task:
    """Represents a task to be assigned to a robot."""
    id: str
    target_position: tuple  # (x, y, z)
    priority: int  # Higher value = higher priority


class CommandPlanner:
    """Plans task assignments for the swarm using greedy nearest-robot strategy."""

    def __init__(self, swarm_state: "SwarmState"):
        """Initialize the planner with a reference to swarm state.

        Args:
            swarm_state: The SwarmState instance to plan against.
        """
        self._swarm_state = swarm_state

    def plan(self, tasks: list) -> dict:
        """Assign tasks to robots greedily by priority.

        For each task (sorted by descending priority), assign the nearest
        unassigned robot. Updates swarm state assignments.

        Args:
            tasks: List of Task objects.

        Returns:
            Dict mapping robot_id (int) -> task_id (str).
        """
        # Sort tasks by priority descending (highest priority first)
        sorted_tasks = sorted(tasks, key=lambda t: t.priority, reverse=True)

        assignment: dict[int, str] = {}
        assigned_robot_ids: set[int] = set()

        for task in sorted_tasks:
            # Get currently unassigned robots not yet assigned in this plan
            candidates = [
                r for r in self._swarm_state.unassigned_robots()
                if r.id not in assigned_robot_ids
            ]
            if not candidates:
                break  # No robots left to assign

            # Find nearest robot to task target position
            nearest = min(
                candidates,
                key=lambda r: self.distance(r.position, task.target_position),
            )
            assignment[nearest.id] = task.id
            assigned_robot_ids.add(nearest.id)
            self._swarm_state.assign_task(nearest.id, task.id)

        return assignment

    def distance(self, pos1: tuple, pos2: tuple) -> float:
        """Compute Euclidean distance between two 3D positions.

        Args:
            pos1: First position (x, y, z).
            pos2: Second position (x, y, z).

        Returns:
            Euclidean distance as float.
        """
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))
