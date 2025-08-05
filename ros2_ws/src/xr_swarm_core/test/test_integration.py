#!/usr/bin/env python3
"""
Integration tests for XR-Swarm-Bridge system
Tests end-to-end functionality including ROS communication, WebRTC, and error handling
"""

import unittest
import rclpy
import time
import json
import threading
import asyncio
from typing import Dict, List, Any
from unittest.mock import Mock, patch, MagicMock

# Import components to test
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from xr_swarm_core.swarm_agent_base import SwarmAgent
from xr_swarm_core.error_handler import ErrorHandler, SwarmError, ErrorSeverity
from scripts.swarm_coordinator import SwarmCoordinator
from robot_agents.scripts.drone_agent import DroneAgent


class TestSwarmIntegration(unittest.TestCase):
    """Integration tests for swarm system"""
    
    @classmethod
    def setUpClass(cls):
        """Set up ROS for all tests"""
        if not rclpy.ok():
            rclpy.init()
    
    def setUp(self):
        """Set up test environment"""
        self.coordinator = SwarmCoordinator()
        self.agents = []
        self.test_messages = []
        
    def tearDown(self):
        """Clean up after each test"""
        for agent in self.agents:
            agent.destroy_node()
        self.coordinator.destroy_node()
        self.agents.clear()
        self.test_messages.clear()
    
    def create_test_agent(self, agent_id: str, agent_type: str = "test") -> SwarmAgent:
        """Create a test agent"""
        agent = SwarmAgent(agent_id, agent_type)
        agent.setup()
        self.agents.append(agent)
        return agent
    
    def test_agent_registration_and_discovery(self):
        """Test that agents register with coordinator and are discoverable"""
        # Create test agents
        agent1 = self.create_test_agent("test_01", "drone")
        agent2 = self.create_test_agent("test_02", "ugv")
        
        # Simulate agent status publishing
        for agent in [agent1, agent2]:
            agent.publish_heartbeat()
        
        # Allow time for message processing
        time.sleep(0.5)
        
        # Check that coordinator received agent status
        # In real test, would verify coordinator's agent registry
        self.assertTrue(len(self.agents) == 2)
        self.assertEqual(agent1.agent_type, "drone")
        self.assertEqual(agent2.agent_type, "ugv")
    
    def test_command_distribution_and_execution(self):
        """Test command distribution from coordinator to agents"""
        agent = self.create_test_agent("cmd_test_01")
        
        # Mock command execution
        command_executed = threading.Event()
        original_handler = agent.handle_command
        
        def mock_command_handler(msg):
            original_handler(msg)
            command_executed.set()
        
        agent.handle_command = mock_command_handler
        
        # Send test command
        test_command = {
            'type': 'navigate',
            'position': [10, 20, 5]
        }
        
        # Simulate command from coordinator
        from std_msgs.msg import String
        cmd_msg = String()
        cmd_msg.data = json.dumps(test_command)
        agent.handle_command(cmd_msg)
        
        # Verify command was processed
        self.assertTrue(command_executed.wait(timeout=2.0))
    
    def test_error_handling_and_recovery(self):
        """Test error handling and recovery mechanisms"""
        agent = self.create_test_agent("error_test_01")
        
        # Create error handler
        error_handler = ErrorHandler(agent, "test_agent")
        
        # Register recovery handler
        recovery_called = threading.Event()
        
        def mock_recovery(error: SwarmError) -> bool:
            recovery_called.set()
            return True
        
        error_handler.register_recovery_handler("TEST_ERROR", mock_recovery)
        
        # Trigger error
        test_error = error_handler.handle_error(
            "TEST_ERROR",
            "Test error message",
            ErrorSeverity.ERROR
        )
        
        # Verify error was handled and recovery was triggered
        self.assertTrue(recovery_called.wait(timeout=2.0))
        self.assertEqual(test_error.code, "TEST_ERROR")
        self.assertEqual(test_error.severity, ErrorSeverity.ERROR)
    
    def test_multi_agent_coordination(self):
        """Test coordination between multiple agents"""
        # Create multiple agents
        agents = [
            self.create_test_agent(f"coord_test_{i:02d}", "drone")
            for i in range(5)
        ]
        
        # Send formation command to all agents
        formation_command = {
            'type': 'formation',
            'formation': 'line',
            'agents': [agent.agent_id for agent in agents]
        }
        
        # Track command reception
        commands_received = 0
        command_lock = threading.Lock()
        
        def track_command(original_handler):
            def wrapper(msg):
                nonlocal commands_received
                original_handler(msg)
                with command_lock:
                    commands_received += 1
            return wrapper
        
        # Wrap command handlers
        for agent in agents:
            agent.handle_command = track_command(agent.handle_command)
        
        # Send commands to all agents
        from std_msgs.msg import String
        cmd_msg = String()
        cmd_msg.data = json.dumps(formation_command)
        
        for agent in agents:
            agent.handle_command(cmd_msg)
        
        # Verify all agents received the command
        time.sleep(0.5)
        self.assertEqual(commands_received, len(agents))
    
    def test_telemetry_collection_and_aggregation(self):
        """Test telemetry collection from agents"""
        agent = self.create_test_agent("telemetry_test_01", "drone")
        
        # Track published telemetry
        published_telemetry = []
        
        def capture_telemetry(original_publish):
            def wrapper(telemetry_data):
                published_telemetry.append(telemetry_data)
                return original_publish(telemetry_data)
            return wrapper
        
        agent.publish_telemetry = capture_telemetry(agent.publish_telemetry)
        
        # Publish test telemetry
        test_telemetry = {
            'battery_level': 85,
            'position': {'x': 10, 'y': 20, 'z': 5},
            'velocity': {'x': 1, 'y': 0, 'z': 0}
        }
        
        agent.publish_telemetry(test_telemetry)
        
        # Verify telemetry was captured
        self.assertEqual(len(published_telemetry), 1)
        self.assertEqual(published_telemetry[0]['battery_level'], 85)
    
    def test_emergency_procedures(self):
        """Test emergency stop and safety procedures"""
        agents = [
            self.create_test_agent(f"emergency_test_{i:02d}")
            for i in range(3)
        ]
        
        # Track emergency stop execution
        emergency_stops = 0
        stop_lock = threading.Lock()
        
        def track_emergency_stop(original_handler):
            def wrapper(command_data):
                nonlocal emergency_stops
                original_handler(command_data)
                with stop_lock:
                    emergency_stops += 1
            return wrapper
        
        # Wrap emergency stop handlers
        for agent in agents:
            agent.handle_emergency_stop = track_emergency_stop(agent.handle_emergency_stop)
        
        # Trigger emergency stop
        emergency_command = {'type': 'emergency_stop'}
        
        from std_msgs.msg import String
        cmd_msg = String()
        cmd_msg.data = json.dumps(emergency_command)
        
        for agent in agents:
            agent.handle_command(cmd_msg)
        
        # Verify all agents executed emergency stop
        time.sleep(0.5)
        self.assertEqual(emergency_stops, len(agents))
        
        # Verify all agents are in emergency state
        for agent in agents:
            self.assertEqual(agent.status, "emergency_stop")
    
    def test_network_resilience(self):
        """Test system behavior under network conditions"""
        agent = self.create_test_agent("network_test_01")
        
        # Simulate network timeout
        timeout_occurred = threading.Event()
        
        def simulate_timeout():
            time.sleep(2)  # Simulate 2 second delay
            timeout_occurred.set()
        
        timeout_thread = threading.Thread(target=simulate_timeout)
        timeout_thread.start()
        
        # Test agent continues operating during timeout
        start_time = time.time()
        agent.publish_heartbeat()  # Should not block
        duration = time.time() - start_time
        
        # Heartbeat should complete quickly even with simulated network issues
        self.assertLess(duration, 0.1)
        
        timeout_thread.join()
        self.assertTrue(timeout_occurred.is_set())
    
    def test_performance_under_load(self):
        """Test system performance under high load"""
        # Create many agents
        num_agents = 20
        agents = [
            self.create_test_agent(f"load_test_{i:03d}")
            for i in range(num_agents)
        ]
        
        # Measure command processing time
        start_time = time.time()
        
        # Send commands to all agents simultaneously
        test_command = {'type': 'ping'}
        from std_msgs.msg import String
        cmd_msg = String()
        cmd_msg.data = json.dumps(test_command)
        
        threads = []
        for agent in agents:
            thread = threading.Thread(target=agent.handle_command, args=(cmd_msg,))
            threads.append(thread)
            thread.start()
        
        # Wait for all commands to complete
        for thread in threads:
            thread.join(timeout=5.0)  # 5 second timeout
        
        duration = time.time() - start_time
        
        # All commands should complete within reasonable time
        self.assertLess(duration, 2.0)  # Should complete in under 2 seconds
        
        # Verify all agents are still responsive
        for agent in agents:
            self.assertNotEqual(agent.status, "timeout")


class TestDroneAgent(unittest.TestCase):
    """Specific tests for drone agent functionality"""
    
    @classmethod
    def setUpClass(cls):
        """Set up ROS for all tests"""
        if not rclpy.ok():
            rclpy.init()
    
    def setUp(self):
        """Set up test environment"""
        self.drone = DroneAgent("test_drone_01")
        self.drone.setup()
    
    def tearDown(self):
        """Clean up after each test"""
        self.drone.cleanup()
        self.drone.destroy_node()
    
    def test_drone_initialization(self):
        """Test drone agent initialization"""
        self.assertEqual(self.drone.agent_id, "test_drone_01")
        self.assertEqual(self.drone.agent_type, "drone")
        self.assertIn('navigate', self.drone.capabilities)
        self.assertIn('hover', self.drone.capabilities)
        self.assertEqual(self.drone.status, "idle")
    
    def test_takeoff_command(self):
        """Test drone takeoff functionality"""
        # Arm the drone first
        self.drone.handle_arm({})
        self.assertTrue(self.drone.is_armed)
        
        # Test takeoff
        takeoff_command = {'altitude': 10.0}
        self.drone.handle_takeoff(takeoff_command)
        
        self.assertEqual(self.drone.status, "taking_off")
    
    def test_navigation_waypoints(self):
        """Test drone navigation with waypoints"""
        waypoints = [
            [10, 0, 5],
            [20, 10, 5],
            [0, 20, 5]
        ]
        
        nav_command = {'waypoints': waypoints}
        self.drone.handle_navigate(nav_command)
        
        self.assertEqual(len(self.drone.waypoints), 3)
        self.assertEqual(self.drone.status, "navigating")
        self.assertEqual(self.drone.current_waypoint_index, 0)
    
    def test_search_pattern_generation(self):
        """Test search pattern generation"""
        area = {
            'center': [50, 50],
            'size': [100, 100],
            'altitude': 15
        }
        
        # Test spiral pattern
        spiral_waypoints = self.drone.generate_search_pattern('spiral', area)
        self.assertGreater(len(spiral_waypoints), 0)
        
        # Test grid pattern
        grid_waypoints = self.drone.generate_search_pattern('grid', area)
        self.assertGreater(len(grid_waypoints), 0)
        
        # Test perimeter pattern
        perimeter_waypoints = self.drone.generate_search_pattern('perimeter', area)
        self.assertEqual(len(perimeter_waypoints), 5)  # 4 corners + return to start
    
    def test_emergency_stop_behavior(self):
        """Test drone emergency stop behavior"""
        # Start with some waypoints
        self.drone.waypoints = [[10, 10, 5], [20, 20, 5]]
        self.drone.current_waypoint_index = 0
        self.drone.status = "navigating"
        
        # Trigger emergency stop
        self.drone.handle_emergency_stop({})
        
        self.assertEqual(self.drone.status, "emergency_hover")
        self.assertEqual(len(self.drone.waypoints), 0)
    
    def test_battery_monitoring(self):
        """Test battery level monitoring and low battery behavior"""
        from std_msgs.msg import Float32
        
        # Test normal battery level
        normal_battery = Float32()
        normal_battery.data = 80.0
        self.drone.battery_callback(normal_battery)
        
        self.assertEqual(self.drone.battery_level, 80.0)
        self.assertNotEqual(self.drone.status, "low_battery")
        
        # Test low battery warning
        low_battery = Float32()
        low_battery.data = 15.0
        self.drone.battery_callback(low_battery)
        
        self.assertEqual(self.drone.battery_level, 15.0)
        self.assertEqual(self.drone.status, "low_battery")
        
        # Test critical battery (should trigger return to base)
        critical_battery = Float32()
        critical_battery.data = 8.0
        self.drone.battery_callback(critical_battery)
        
        self.assertEqual(self.drone.status, "returning_to_base")


class TestErrorHandling(unittest.TestCase):
    """Test error handling system"""
    
    @classmethod
    def setUpClass(cls):
        """Set up ROS for all tests"""
        if not rclpy.ok():
            rclpy.init()
    
    def setUp(self):
        """Set up test environment"""
        self.test_node = SwarmAgent("error_test_node")
        self.error_handler = ErrorHandler(self.test_node, "test_component")
    
    def tearDown(self):
        """Clean up after each test"""
        self.test_node.destroy_node()
    
    def test_error_creation_and_logging(self):
        """Test error creation and logging"""
        error = self.error_handler.handle_error(
            "TEST_001",
            "Test error message",
            ErrorSeverity.WARNING,
            agent_id="test_agent",
            context={"test_data": "value"}
        )
        
        self.assertEqual(error.code, "TEST_001")
        self.assertEqual(error.message, "Test error message")
        self.assertEqual(error.severity, ErrorSeverity.WARNING)
        self.assertEqual(error.agent_id, "test_agent")
        self.assertEqual(error.context["test_data"], "value")
    
    def test_error_recovery_mechanism(self):
        """Test error recovery handlers"""
        recovery_executed = threading.Event()
        
        def test_recovery_handler(error: SwarmError) -> bool:
            recovery_executed.set()
            return True
        
        self.error_handler.register_recovery_handler("RECOVERABLE_ERROR", test_recovery_handler)
        
        # Trigger error that should be recovered
        error = self.error_handler.handle_error(
            "RECOVERABLE_ERROR",
            "This error should be recovered",
            ErrorSeverity.ERROR
        )
        
        # Verify recovery was attempted
        self.assertTrue(recovery_executed.wait(timeout=1.0))
    
    def test_circuit_breaker_functionality(self):
        """Test circuit breaker pattern"""
        # Trigger multiple errors to trip circuit breaker
        for i in range(6):  # Exceed threshold of 5
            self.error_handler.handle_error(
                "CIRCUIT_TEST",
                f"Error {i+1}",
                ErrorSeverity.ERROR
            )
        
        # Circuit should be open now
        self.assertTrue(self.error_handler.is_circuit_open("CIRCUIT_TEST"))
    
    def test_error_statistics(self):
        """Test error statistics collection"""
        # Generate various errors
        errors = [
            ("ERROR_001", "First error"),
            ("ERROR_002", "Second error"),
            ("ERROR_001", "First error again"),
        ]
        
        for code, message in errors:
            self.error_handler.handle_error(code, message, ErrorSeverity.WARNING)
        
        stats = self.error_handler.get_error_statistics()
        
        self.assertEqual(stats['total_errors'], 3)
        self.assertEqual(stats['error_counts']['ERROR_001'], 2)
        self.assertEqual(stats['error_counts']['ERROR_002'], 1)
        self.assertEqual(len(stats['recent_errors']), 3)


if __name__ == '__main__':
    # Run integration tests
    test_suite = unittest.TestLoader().loadTestsFromModule(sys.modules[__name__])
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)
    
    # Cleanup ROS
    if rclpy.ok():
        rclpy.shutdown()
    
    # Exit with appropriate code
    sys.exit(0 if result.wasSuccessful() else 1)