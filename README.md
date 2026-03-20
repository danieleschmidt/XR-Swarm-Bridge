# XR-Swarm-Bridge

A Python package for controlling swarms of robots via an XR (Extended Reality) interface, featuring WebRTC-based signaling, causal command tracing, greedy task planning, and natural language command parsing.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    XR Interface (AR/VR)                      │
│          Natural language commands / gesture input           │
└───────────────────────┬─────────────────────────────────────┘
                        │
              ┌─────────▼──────────┐
              │   NLCommandParser  │  Parses "move robot 3 to 1,2"
              └─────────┬──────────┘
                        │
              ┌─────────▼──────────┐
              │  CommandPlanner    │  Greedy task → robot assignment
              └─────────┬──────────┘
                        │
              ┌─────────▼──────────┐
              │    SwarmState      │  Tracks all robot positions,
              │                    │  velocities, and assignments
              └─────────┬──────────┘
                        │
              ┌─────────▼──────────┐
              │    WebRTCStub      │  Simulates signaling layer
              │                    │  with latency & packet loss
              └─────────┬──────────┘
                        │
              ┌─────────▼──────────┐
              │ CausalTraceRecorder│  Records full command history
              │                    │  with rewind capability
              └────────────────────┘
```

## Components

### `SwarmState`
Manages the collective state of all robots in the swarm. Each robot has:
- `id`: Unique integer identifier
- `position`: 3D coordinates `(x, y, z)`
- `velocity`: 3D velocity vector `(vx, vy, vz)`
- `assignment`: Current task string or `None`

### `CommandPlanner`
Greedy task assignment planner. Given a list of `Task` objects with priorities, assigns each task (highest priority first) to the nearest unassigned robot using Euclidean distance.

### `WebRTCStub`
Mock WebRTC signaling layer that simulates:
- **Latency**: Configurable `latency_ms` delay per packet
- **Packet loss**: Configurable `packet_loss_rate` probability of drop
- **Broadcast**: Send commands to multiple robots simultaneously

### `CausalTraceRecorder`
Records the full causal history of robot commands with:
- **Timestamped entries** with robot_id, command, outcome, and metadata
- **Rewind**: Retrieve last N entries in reverse chronological order
- **Per-robot history**: Filter trace by robot id
- **JSON export**: Save full trace to disk

### `NLCommandParser`
Regex-based natural language parser supporting:
- `"move robot {id} to {x},{y}"` — 2D move (z defaults to 0)
- `"move robot {id} to position {x},{y}"` — 2D move with keyword
- `"move robot {id} to {x},{y},{z}"` — 3D move
- `"stop robot {id}"` — halt robot
- `"assign robot {id} to {task}"` — assign task by name
- `"status of robot {id}"` — query status

## Installation

```bash
pip install -e .
```

## Usage

```python
from xr_swarm_bridge import SwarmState, CommandPlanner, Task, WebRTCStub, CausalTraceRecorder, NLCommandParser

# Create 10 robots
swarm = SwarmState(num_robots=10)

# Plan task assignments
planner = CommandPlanner(swarm)
tasks = [
    Task(id="inspect_zone_A", target_position=(5.0, 5.0, 1.0), priority=10),
    Task(id="patrol_perimeter", target_position=(-5.0, 0.0, 0.5), priority=5),
]
assignments = planner.plan(tasks)

# Send commands via WebRTC
webrtc = WebRTCStub(latency_ms=20.0, packet_loss_rate=0.01)
recorder = CausalTraceRecorder()
for robot_id, task_id in assignments.items():
    result = webrtc.send_command(robot_id, {"action": "goto", "task": task_id})
    recorder.record(robot_id, {"action": "goto", "task": task_id}, result)

# Parse natural language
parser = NLCommandParser()
cmd = parser.parse("move robot 3 to 2.5,1.5,0")
print(cmd)  # {'action': 'move', 'robot_id': 3, 'params': {'position': (2.5, 1.5, 0.0)}}
```

## Running Tests

```bash
python -m pytest tests/ -v
```

## Demo

```bash
python demo.py
```

## Design Principles

- **Causal transparency**: Every command is recorded with its full context and outcome
- **Network realism**: WebRTC stub models real-world packet loss and latency
- **Priority-aware planning**: High-priority tasks always get first pick of nearby robots
- **Human-readable control**: Natural language interface bridges XR interaction to robot commands
- **Minimal dependencies**: Uses only Python standard library + optional numpy

## License

MIT
