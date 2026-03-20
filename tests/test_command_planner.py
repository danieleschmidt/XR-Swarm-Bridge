"""Tests for CommandPlanner."""
import math
import pytest
from xr_swarm_bridge.swarm_state import SwarmState
from xr_swarm_bridge.command_planner import Task, CommandPlanner


def make_swarm_at_positions(positions):
    """Create a SwarmState and set robots at specific positions."""
    swarm = SwarmState(num_robots=len(positions))
    for i, pos in enumerate(positions):
        swarm.update_position(i, pos)
    return swarm


def test_distance_2d():
    swarm = SwarmState(num_robots=2)
    planner = CommandPlanner(swarm)
    d = planner.distance((0, 0, 0), (3, 4, 0))
    assert abs(d - 5.0) < 1e-9


def test_distance_3d():
    swarm = SwarmState(num_robots=2)
    planner = CommandPlanner(swarm)
    d = planner.distance((0, 0, 0), (1, 1, 1))
    assert abs(d - math.sqrt(3)) < 1e-9


def test_plan_assigns_nearest_robot():
    """Robot 0 at (0,0,0), Robot 1 at (10,10,10) — task at (0.1, 0.1, 0.1) => Robot 0"""
    swarm = make_swarm_at_positions([(0.0, 0.0, 0.0), (10.0, 10.0, 10.0)])
    planner = CommandPlanner(swarm)
    tasks = [Task(id="t1", target_position=(0.1, 0.1, 0.1), priority=1)]
    result = planner.plan(tasks)
    assert 0 in result
    assert result[0] == "t1"


def test_plan_priority_order():
    """High priority task should get first pick of robots."""
    swarm = make_swarm_at_positions([(0.0, 0.0, 0.0), (5.0, 5.0, 0.0)])
    planner = CommandPlanner(swarm)
    tasks = [
        Task(id="low", target_position=(0.0, 0.0, 0.0), priority=1),
        Task(id="high", target_position=(0.0, 0.0, 0.0), priority=10),
    ]
    result = planner.plan(tasks)
    # Both should be assigned
    assert len(result) == 2
    # Robot 0 (nearest to 0,0,0) should be assigned to "high"
    assert result[0] == "high"


def test_plan_returns_dict():
    swarm = SwarmState(num_robots=3)
    planner = CommandPlanner(swarm)
    tasks = [Task(id="t1", target_position=(1.0, 1.0, 1.0), priority=5)]
    result = planner.plan(tasks)
    assert isinstance(result, dict)
    assert len(result) == 1


def test_plan_updates_swarm_assignment():
    swarm = make_swarm_at_positions([(0.0, 0.0, 0.0)])
    planner = CommandPlanner(swarm)
    tasks = [Task(id="task_x", target_position=(0.0, 0.0, 0.0), priority=1)]
    result = planner.plan(tasks)
    robot_id = list(result.keys())[0]
    assert swarm.get_robot(robot_id).assignment == "task_x"


def test_plan_no_tasks_returns_empty():
    swarm = SwarmState(num_robots=3)
    planner = CommandPlanner(swarm)
    result = planner.plan([])
    assert result == {}


def test_plan_more_tasks_than_robots():
    swarm = SwarmState(num_robots=2)
    planner = CommandPlanner(swarm)
    tasks = [
        Task(id=f"t{i}", target_position=(float(i), 0.0, 0.0), priority=i)
        for i in range(5)
    ]
    result = planner.plan(tasks)
    # Only 2 robots, so at most 2 assignments
    assert len(result) <= 2
