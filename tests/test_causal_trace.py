"""Tests for CausalTraceRecorder."""
import json
import os
import tempfile
import pytest
from xr_swarm_bridge.causal_trace import TraceEntry, CausalTraceRecorder


@pytest.fixture
def recorder():
    return CausalTraceRecorder()


def test_record_stores_entry(recorder):
    recorder.record(0, {"action": "move"}, {"delivered": True})
    assert len(recorder.rewind(10)) == 1


def test_record_entry_fields(recorder):
    recorder.record(2, {"action": "stop"}, {"delivered": False}, metadata={"note": "test"})
    entries = recorder.rewind(1)
    e = entries[0]
    assert e.robot_id == 2
    assert e.command == {"action": "stop"}
    assert e.outcome == {"delivered": False}
    assert e.metadata == {"note": "test"}
    assert e.timestamp > 0


def test_rewind_returns_reversed(recorder):
    for i in range(5):
        recorder.record(i, {"seq": i}, {"ok": True})
    entries = recorder.rewind(5)
    seqs = [e.command["seq"] for e in entries]
    assert seqs == [4, 3, 2, 1, 0]


def test_rewind_limited(recorder):
    for i in range(10):
        recorder.record(i % 3, {"seq": i}, {})
    entries = recorder.rewind(3)
    assert len(entries) == 3


def test_rewind_zero(recorder):
    recorder.record(0, {}, {})
    assert recorder.rewind(0) == []


def test_get_history_by_robot(recorder):
    recorder.record(0, {"a": 1}, {})
    recorder.record(1, {"a": 2}, {})
    recorder.record(0, {"a": 3}, {})
    history = recorder.get_history(0)
    assert len(history) == 2
    assert all(e.robot_id == 0 for e in history)


def test_get_history_empty(recorder):
    recorder.record(1, {}, {})
    assert recorder.get_history(99) == []


def test_to_json_valid(recorder):
    recorder.record(0, {"action": "move"}, {"delivered": True})
    data = json.loads(recorder.to_json())
    assert isinstance(data, list)
    assert len(data) == 1
    assert data[0]["robot_id"] == 0


def test_save_and_reload(recorder):
    recorder.record(3, {"action": "stop"}, {"delivered": True})
    with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
        path = f.name
    try:
        recorder.save(path)
        with open(path) as f:
            data = json.load(f)
        assert len(data) == 1
        assert data[0]["robot_id"] == 3
    finally:
        os.unlink(path)


def test_clear(recorder):
    recorder.record(0, {}, {})
    recorder.record(1, {}, {})
    recorder.clear()
    assert recorder.rewind(10) == []
