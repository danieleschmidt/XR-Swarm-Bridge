"""XR-Swarm-Bridge: Python package for XR-based swarm robot control."""

from xr_swarm_bridge.swarm_state import Robot, SwarmState
from xr_swarm_bridge.command_planner import Task, CommandPlanner
from xr_swarm_bridge.webrtc_stub import WebRTCStub
from xr_swarm_bridge.causal_trace import TraceEntry, CausalTraceRecorder
from xr_swarm_bridge.nl_parser import NLCommandParser

__all__ = [
    "Robot",
    "SwarmState",
    "Task",
    "CommandPlanner",
    "WebRTCStub",
    "CausalTraceRecorder",
    "TraceEntry",
    "NLCommandParser",
]

__version__ = "0.1.0"
