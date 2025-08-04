#!/usr/bin/env python3

import unittest
import rclpy
import json
import time
from unittest.mock import Mock, patch, MagicMock
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

# Import the class to test
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from xr_swarm_core.swarm_agent_base import SwarmAgent


class TestSwarmAgent(unittest.TestCase):
    """Test suite for SwarmAgent base class"""
    
    @classmethod
    def setUpClass(cls):
        """Set up ROS for all tests"""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Clean up ROS after all tests"""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up each test"""
        self.agent = SwarmAgent("test_agent", "test_type")
        self.agent.setup()
    
    def tearDown(self):
        """Clean up after each test"""
        self.agent.cleanup()
        self.agent.destroy_node()
    
    def test_initialization(self):
        """Test agent initialization"""
        self.assertEqual(self.agent.agent_id, "test_agent")
        self.assertEqual(self.agent.agent_type, "test_type")
        self.assertEqual(self.agent.status, "idle")
        self.assertIsInstance(self.agent.capabilities, list)
    
    def test_command_handler_registration(self):
        """Test command handler registration"""
        # Test default handlers are registered
        self.assertIn('ping', self.agent.command_handlers)
        self.assertIn('status', self.agent.command_handlers)
        self.assertIn('emergency_stop', self.agent.command_handlers)
        
        # Test custom handler registration
        def custom_handler(data):
            pass
        
        self.agent.register_command_handler('custom', custom_handler)
        self.assertIn('custom', self.agent.command_handlers)
        self.assertEqual(self.agent.command_handlers['custom'], custom_handler)
    
    def test_ping_command(self):
        """Test ping command handling"""
        with patch.object(self.agent, 'publish_response') as mock_publish:
            command_data = {'type': 'ping', 'timestamp': time.time()}
            self.agent.handle_ping(command_data)
            
            mock_publish.assert_called_once()
            call_args = mock_publish.call_args[0][0]
            self.assertEqual(call_args['type'], 'pong')
            self.assertEqual(call_args['agent_id'], 'test_agent')
    
    def test_status_request(self):
        """Test status request handling"""
        with patch.object(self.agent, 'publish_response') as mock_publish:
            command_data = {'type': 'status'}
            self.agent.handle_status_request(command_data)
            
            mock_publish.assert_called_once()
            call_args = mock_publish.call_args[0][0]
            self.assertEqual(call_args['agent_id'], 'test_agent')
            self.assertEqual(call_args['agent_type'], 'test_type')
            self.assertEqual(call_args['status'], 'idle')
    
    def test_emergency_stop(self):
        """Test emergency stop command"""
        command_data = {'type': 'emergency_stop'}
        self.agent.handle_emergency_stop(command_data)
        
        self.assertEqual(self.agent.status, "emergency_stop")
    
    def test_mode_change(self):
        """Test mode change command"""
        command_data = {'type': 'set_mode', 'mode': 'active'}
        self.agent.handle_set_mode(command_data)
        
        self.assertEqual(self.agent.status, "active")
    
    def test_json_command_parsing(self):
        """Test JSON command parsing"""
        with patch.object(self.agent, 'handle_ping') as mock_handler:
            msg = String()
            msg.data = json.dumps({'type': 'ping', 'timestamp': time.time()})
            
            self.agent.handle_command(msg)
            mock_handler.assert_called_once()
    
    def test_invalid_json_handling(self):
        """Test handling of invalid JSON commands"""
        with patch.object(self.agent.get_logger(), 'error') as mock_logger:
            msg = String()
            msg.data = "invalid json"
            
            self.agent.handle_command(msg)
            mock_logger.assert_called_once()
    
    def test_unknown_command_handling(self):
        """Test handling of unknown commands"""
        with patch.object(self.agent.get_logger(), 'warn') as mock_logger:
            msg = String()
            msg.data = json.dumps({'type': 'unknown_command'})
            
            self.agent.handle_command(msg)
            mock_logger.assert_called_once()
    
    def test_global_command_filtering(self):
        """Test global command filtering by type and ID"""
        with patch.object(self.agent, 'handle_command') as mock_handler:
            # Test type-based filtering
            msg = String()
            msg.data = json.dumps({
                'type': 'test_command',
                'target_types': ['test_type']
            })
            
            self.agent.handle_global_command(msg)
            mock_handler.assert_called_once()
            
            mock_handler.reset_mock()
            
            # Test ID-based filtering
            msg.data = json.dumps({
                'type': 'test_command',
                'target_ids': ['test_agent']
            })
            
            self.agent.handle_global_command(msg)
            mock_handler.assert_called_once()
    
    def test_webrtc_command_parsing(self):
        """Test WebRTC command parsing"""
        with patch.object(self.agent, 'handle_ping') as mock_handler:
            msg = String()
            msg.data = "ping:test_data"
            
            self.agent.handle_webrtc_command(msg)
            mock_handler.assert_called_once()
    
    def test_telemetry_publishing(self):
        """Test telemetry data publishing"""
        with patch.object(self.agent.telemetry_pub, 'publish') as mock_publish:
            telemetry_data = {'sensor': 'test', 'value': 42}
            self.agent.publish_telemetry(telemetry_data)
            
            mock_publish.assert_called_once()
            msg = mock_publish.call_args[0][0]
            published_data = json.loads(msg.data)
            
            self.assertEqual(published_data['agent_id'], 'test_agent')
            self.assertEqual(published_data['sensor'], 'test')
            self.assertEqual(published_data['value'], 42)
            self.assertIn('timestamp', published_data)
    
    def test_heartbeat_publishing(self):
        """Test heartbeat publishing"""
        with patch.object(self.agent.status_pub, 'publish') as mock_status_pub, \
             patch.object(self.agent.webrtc_pub, 'publish') as mock_webrtc_pub:
            
            self.agent.publish_heartbeat()
            
            # Verify both ROS and WebRTC heartbeats are published
            mock_status_pub.assert_called_once()
            mock_webrtc_pub.assert_called_once()
            
            # Check ROS heartbeat content
            ros_msg = mock_status_pub.call_args[0][0]
            ros_data = json.loads(ros_msg.data)
            self.assertEqual(ros_data['agent_id'], 'test_agent')
            self.assertEqual(ros_data['status'], 'idle')
            
            # Check WebRTC heartbeat content
            webrtc_msg = mock_webrtc_pub.call_args[0][0]
            self.assertIn('heartbeat:', webrtc_msg.data)


class TestSwarmAgentIntegration(unittest.TestCase):
    """Integration tests for SwarmAgent"""
    
    @classmethod
    def setUpClass(cls):
        """Set up ROS for all tests"""
        if not rclpy.ok():
            rclpy.init()
    
    def test_full_command_cycle(self):
        """Test complete command handling cycle"""
        agent = SwarmAgent("integration_test", "test")
        
        # Create a mock response handler
        responses = []
        
        def mock_publish_response(data):
            responses.append(data)
        
        agent.publish_response = mock_publish_response
        
        try:
            # Send ping command
            msg = String()
            msg.data = json.dumps({'type': 'ping', 'timestamp': time.time()})
            agent.handle_command(msg)
            
            # Verify response
            self.assertEqual(len(responses), 1)
            response = responses[0]
            self.assertEqual(response['type'], 'pong')
            self.assertEqual(response['agent_id'], 'integration_test')
            
        finally:
            agent.destroy_node()


class TestSwarmAgentPerformance(unittest.TestCase):
    """Performance tests for SwarmAgent"""
    
    @classmethod
    def setUpClass(cls):
        """Set up ROS for all tests"""
        if not rclpy.ok():
            rclpy.init()
    
    def test_command_processing_speed(self):
        """Test command processing performance"""
        agent = SwarmAgent("perf_test", "test")
        
        # Mock the publish methods to avoid actual network calls
        agent.publish_response = Mock()
        
        try:
            start_time = time.time()
            num_commands = 1000
            
            for i in range(num_commands):
                msg = String()
                msg.data = json.dumps({'type': 'ping', 'id': i})
                agent.handle_command(msg)
            
            end_time = time.time()
            processing_time = end_time - start_time
            
            # Should be able to process at least 100 commands per second
            self.assertLess(processing_time, num_commands / 100)
            
            print(f"Processed {num_commands} commands in {processing_time:.3f}s "
                  f"({num_commands/processing_time:.1f} cmd/s)")
            
        finally:
            agent.destroy_node()
    
    def test_memory_usage(self):
        """Test memory usage doesn't grow excessively"""
        import psutil
        import os
        
        process = psutil.Process(os.getpid())
        initial_memory = process.memory_info().rss
        
        agent = SwarmAgent("memory_test", "test")
        agent.publish_response = Mock()
        
        try:
            # Process many commands
            for i in range(10000):
                msg = String()
                msg.data = json.dumps({'type': 'ping', 'id': i})
                agent.handle_command(msg)
                
                # Force garbage collection periodically
                if i % 1000 == 0:
                    import gc
                    gc.collect()
            
            final_memory = process.memory_info().rss
            memory_growth = final_memory - initial_memory
            
            # Memory growth should be less than 50MB
            self.assertLess(memory_growth, 50 * 1024 * 1024)
            
            print(f"Memory growth: {memory_growth / 1024 / 1024:.1f} MB")
            
        finally:
            agent.destroy_node()


if __name__ == '__main__':
    # Run all tests
    unittest.main(verbosity=2)