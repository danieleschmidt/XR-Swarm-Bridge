"""Demo script for XR-Swarm-Bridge."""
import random

from xr_swarm_bridge import (
    SwarmState,
    CommandPlanner,
    Task,
    WebRTCStub,
    CausalTraceRecorder,
    NLCommandParser,
)


def main():
    print("=" * 60)
    print("XR-Swarm-Bridge Demo")
    print("=" * 60)

    # 1. Initialize swarm with 10 robots
    random.seed(42)
    swarm = SwarmState(num_robots=10)
    print(f"\n[SwarmState] Initialized {len(swarm.to_dict())} robots")
    for robot_id, info in swarm.to_dict().items():
        print(f"  Robot {robot_id}: pos={info['position']}")

    # 2. Define tasks
    tasks = [
        Task(id="task_alpha", target_position=(5.0, 5.0, 1.0), priority=10),
        Task(id="task_beta", target_position=(-5.0, 3.0, 0.5), priority=8),
        Task(id="task_gamma", target_position=(0.0, -4.0, 2.0), priority=6),
        Task(id="task_delta", target_position=(3.0, -3.0, 0.0), priority=4),
    ]

    # 3. Plan assignments
    planner = CommandPlanner(swarm)
    assignments = planner.plan(tasks)
    print(f"\n[CommandPlanner] Assignments:")
    for robot_id, task_id in assignments.items():
        print(f"  Robot {robot_id} -> {task_id}")

    # 4. Simulate WebRTC commands with low latency for demo
    webrtc = WebRTCStub(latency_ms=0.1, packet_loss_rate=0.0)
    recorder = CausalTraceRecorder()

    print(f"\n[WebRTCStub] Sending commands...")
    for robot_id, task_id in assignments.items():
        command = {"action": "goto", "task_id": task_id}
        result = webrtc.send_command(robot_id, command)
        recorder.record(robot_id, command, result)
        status = "✓ delivered" if result["delivered"] else "✗ dropped"
        print(f"  Robot {robot_id}: {status} (latency={result['latency_ms']}ms)")

    # 5. Broadcast stop to all assigned robots
    print(f"\n[WebRTCStub] Broadcasting stop to assigned robots...")
    stop_results = webrtc.broadcast({"action": "stop"}, list(assignments.keys()))
    for r in stop_results:
        recorder.record(r["robot_id"], {"action": "stop"}, r)
    print(f"  Broadcast to {len(stop_results)} robots complete")

    # 6. Causal trace summary
    print(f"\n[CausalTrace] Total entries: {len(recorder.rewind(999))}")
    print(f"  Last 3 entries (reversed):")
    for entry in recorder.rewind(3):
        print(f"  [{entry.timestamp:.2f}] Robot {entry.robot_id}: {entry.command} -> delivered={entry.outcome.get('delivered')}")

    # 7. NL Parser demo
    parser = NLCommandParser()
    nl_commands = [
        "move robot 3 to 1.5,2.5",
        "move robot 0 to position 3,4",
        "move robot 5 to 1,2,3",
        "stop robot 7",
        "assign robot 2 to patrol_zone_A",
        "status of robot 1",
    ]
    print(f"\n[NLParser] Parsing natural language commands:")
    for cmd in nl_commands:
        parsed = parser.parse(cmd)
        print(f"  '{cmd}'")
        print(f"    -> action={parsed['action']}, robot_id={parsed['robot_id']}, params={parsed['params']}")

    print("\n" + "=" * 60)
    print("Demo complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
