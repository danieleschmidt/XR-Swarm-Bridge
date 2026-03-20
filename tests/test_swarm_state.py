"""Tests for SwarmState and Robot."""
import pytest
from xr_swarm_bridge.swarm_state import Robot, SwarmState


def test_swarm_state_initializes_correct_count():
    swarm = SwarmState(num_robots=5)
    assert len(swarm.to_dict()) == 5


def test_swarm_state_robot_ids():
    swarm = SwarmState(num_robots=3)
    assert set(swarm.to_dict().keys()) == {0, 1, 2}


def test_get_robot_returns_robot():
    swarm = SwarmState(num_robots=4)
    robot = swarm.get_robot(0)
    assert isinstance(robot, Robot)
    assert robot.id == 0


def test_get_robot_invalid_id_raises():
    swarm = SwarmState(num_robots=2)
    with pytest.raises(KeyError):
        swarm.get_robot(99)


def test_update_position():
    swarm = SwarmState(num_robots=3)
    new_pos = (1.0, 2.0, 3.0)
    swarm.update_position(0, new_pos)
    assert swarm.get_robot(0).position == new_pos


def test_update_velocity():
    swarm = SwarmState(num_robots=3)
    new_vel = (0.5, -0.5, 0.0)
    swarm.update_velocity(1, new_vel)
    assert swarm.get_robot(1).velocity == new_vel


def test_assign_task():
    swarm = SwarmState(num_robots=3)
    swarm.assign_task(0, "task_001")
    assert swarm.get_robot(0).assignment == "task_001"


def test_unassigned_robots_all_initially():
    swarm = SwarmState(num_robots=4)
    unassigned = swarm.unassigned_robots()
    assert len(unassigned) == 4


def test_unassigned_robots_after_assignment():
    swarm = SwarmState(num_robots=4)
    swarm.assign_task(0, "t1")
    swarm.assign_task(2, "t2")
    unassigned_ids = [r.id for r in swarm.unassigned_robots()]
    assert 0 not in unassigned_ids
    assert 2 not in unassigned_ids
    assert 1 in unassigned_ids
    assert 3 in unassigned_ids


def test_to_dict_structure():
    swarm = SwarmState(num_robots=2)
    d = swarm.to_dict()
    for robot_id, info in d.items():
        assert "id" in info
        assert "position" in info
        assert "velocity" in info
        assert "assignment" in info
        assert info["assignment"] is None
