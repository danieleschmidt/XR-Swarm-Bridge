"""Tests for NLCommandParser."""
import pytest
from xr_swarm_bridge.nl_parser import NLCommandParser


@pytest.fixture
def parser():
    return NLCommandParser()


def test_move_2d(parser):
    result = parser.parse("move robot 3 to 1,2")
    assert result["action"] == "move"
    assert result["robot_id"] == 3
    assert result["params"]["position"] == (1.0, 2.0, 0.0)


def test_move_with_position_keyword(parser):
    result = parser.parse("move robot 5 to position 4,6")
    assert result["action"] == "move"
    assert result["robot_id"] == 5
    assert result["params"]["position"] == (4.0, 6.0, 0.0)


def test_move_3d(parser):
    result = parser.parse("move robot 0 to 1,2,3")
    assert result["action"] == "move"
    assert result["robot_id"] == 0
    assert result["params"]["position"] == (1.0, 2.0, 3.0)


def test_move_float_coords(parser):
    result = parser.parse("move robot 1 to 1.5,2.5,0.5")
    assert result["params"]["position"] == (1.5, 2.5, 0.5)


def test_move_negative_coords(parser):
    result = parser.parse("move robot 2 to -3,-4,0")
    assert result["params"]["position"] == (-3.0, -4.0, 0.0)


def test_stop(parser):
    result = parser.parse("stop robot 7")
    assert result["action"] == "stop"
    assert result["robot_id"] == 7
    assert result["params"] == {}


def test_assign(parser):
    result = parser.parse("assign robot 2 to patrol_zone_A")
    assert result["action"] == "assign"
    assert result["robot_id"] == 2
    assert result["params"]["task"] == "patrol_zone_A"


def test_assign_multiword_task(parser):
    result = parser.parse("assign robot 4 to inspect building 3")
    assert result["action"] == "assign"
    assert result["params"]["task"] == "inspect building 3"


def test_status(parser):
    result = parser.parse("status of robot 9")
    assert result["action"] == "status"
    assert result["robot_id"] == 9
    assert result["params"] == {}


def test_unrecognized_raises_value_error(parser):
    with pytest.raises(ValueError):
        parser.parse("dance robot 5")


def test_case_insensitive_move(parser):
    result = parser.parse("MOVE ROBOT 1 TO 3,4")
    assert result["action"] == "move"
    assert result["robot_id"] == 1


def test_case_insensitive_stop(parser):
    result = parser.parse("Stop Robot 2")
    assert result["action"] == "stop"
    assert result["robot_id"] == 2
