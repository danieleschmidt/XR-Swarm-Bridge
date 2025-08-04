#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import PointCloud2, Image
import json
import time
from typing import Dict, List, Any, Optional, Callable


class SwarmAgent(Node):
    """Base class for all robots in the XR-Swarm-Bridge system"""
    
    def __init__(self, agent_id: str, agent_type: str = "generic"):
        super().__init__(f"swarm_agent_{agent_id}")
        
        self.agent_id = agent_id
        self.agent_type = agent_type
        self.capabilities = []
        self.status = "idle"
        self.last_heartbeat = time.time()
        
        # QoS profiles for different data types
        self.command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )
        
        # Core publishers
        self.status_pub = self.create_publisher(
            String, f'/swarm/{self.agent_id}/status', self.telemetry_qos
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, f'/swarm/{self.agent_id}/pose', self.telemetry_qos
        )
        self.telemetry_pub = self.create_publisher(
            String, f'/swarm/{self.agent_id}/telemetry', self.telemetry_qos
        )
        
        # Core subscribers
        self.command_sub = self.create_subscription(
            String, f'/swarm/{self.agent_id}/command', 
            self.handle_command, self.command_qos
        )
        self.global_command_sub = self.create_subscription(
            String, '/swarm/global/command',
            self.handle_global_command, self.command_qos
        )
        
        # WebRTC data channel for low-latency communication
        self.webrtc_pub = self.create_publisher(
            String, f'/webrtc/{self.agent_id}/data', self.telemetry_qos
        )
        self.webrtc_sub = self.create_subscription(
            String, f'/webrtc/{self.agent_id}/command',
            self.handle_webrtc_command, self.telemetry_qos
        )
        
        # Heartbeat timer
        self.heartbeat_timer = self.create_timer(1.0, self.publish_heartbeat)
        
        # Command handlers
        self.command_handlers = {}
        self.register_default_handlers()
        
        self.get_logger().info(f"SwarmAgent {self.agent_id} ({self.agent_type}) initialized")
    
    def register_default_handlers(self):
        """Register default command handlers"""
        self.command_handlers.update({
            'ping': self.handle_ping,
            'status': self.handle_status_request,
            'emergency_stop': self.handle_emergency_stop,
            'set_mode': self.handle_set_mode
        })
    
    def register_command_handler(self, command: str, handler: Callable):
        """Register a custom command handler"""
        self.command_handlers[command] = handler
    
    def handle_command(self, msg: String):
        """Handle direct commands to this agent"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('type')
            
            if command_type in self.command_handlers:
                self.command_handlers[command_type](command_data)
            else:
                self.get_logger().warn(f"Unknown command type: {command_type}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid command JSON: {msg.data}")
    
    def handle_global_command(self, msg: String):
        """Handle broadcast commands to all agents"""
        try:
            command_data = json.loads(msg.data)
            
            # Check if this agent should respond to this global command
            target_types = command_data.get('target_types', [])
            target_ids = command_data.get('target_ids', [])
            
            if (not target_types or self.agent_type in target_types) and \
               (not target_ids or self.agent_id in target_ids):
                self.handle_command(msg)
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid global command JSON: {msg.data}")
    
    def handle_webrtc_command(self, msg: String):
        """Handle low-latency WebRTC commands"""
        # WebRTC commands bypass JSON parsing for speed
        parts = msg.data.split(':', 2)
        if len(parts) >= 2:
            command_type = parts[0]
            if command_type in self.command_handlers:
                # Convert simple format to dict for handler compatibility
                command_data = {'type': command_type, 'data': parts[1] if len(parts) > 1 else ''}
                self.command_handlers[command_type](command_data)
    
    def handle_ping(self, command_data: Dict[str, Any]):
        """Handle ping command"""
        response = {
            'type': 'pong',
            'agent_id': self.agent_id,
            'timestamp': time.time(),
            'status': self.status
        }
        self.publish_response(response)
    
    def handle_status_request(self, command_data: Dict[str, Any]):
        """Handle status request"""
        status_data = {
            'agent_id': self.agent_id,
            'agent_type': self.agent_type,
            'status': self.status,
            'capabilities': self.capabilities,
            'timestamp': time.time()
        }
        self.publish_response(status_data)
    
    def handle_emergency_stop(self, command_data: Dict[str, Any]):
        """Handle emergency stop command"""
        self.status = "emergency_stop"
        self.get_logger().warn("EMERGENCY STOP activated")
        # Override in subclass for specific stop behavior
    
    def handle_set_mode(self, command_data: Dict[str, Any]):
        """Handle mode change command"""
        new_mode = command_data.get('mode', 'idle')
        old_status = self.status
        self.status = new_mode
        self.get_logger().info(f"Mode changed from {old_status} to {new_mode}")
    
    def publish_heartbeat(self):
        """Publish periodic heartbeat"""
        heartbeat = {
            'agent_id': self.agent_id,
            'timestamp': time.time(),
            'status': self.status,
            'uptime': time.time() - self.last_heartbeat
        }
        
        msg = String()
        msg.data = json.dumps(heartbeat)
        self.status_pub.publish(msg)
        
        # Also send via WebRTC for low latency
        webrtc_msg = String()
        webrtc_msg.data = f"heartbeat:{self.status}:{time.time()}"
        self.webrtc_pub.publish(webrtc_msg)
    
    def publish_response(self, response_data: Dict[str, Any]):
        """Publish response to telemetry topic"""
        msg = String()
        msg.data = json.dumps(response_data)
        self.telemetry_pub.publish(msg)
    
    def publish_telemetry(self, telemetry_data: Dict[str, Any]):
        """Publish telemetry data"""
        telemetry_data.update({
            'agent_id': self.agent_id,
            'timestamp': time.time()
        })
        
        msg = String()
        msg.data = json.dumps(telemetry_data)
        self.telemetry_pub.publish(msg)
    
    def publish_pose(self, pose: PoseStamped):
        """Publish agent pose"""
        self.pose_pub.publish(pose)
    
    # Abstract methods to be implemented by subclasses
    def setup(self):
        """Setup agent-specific initialization"""
        pass
    
    def cleanup(self):
        """Cleanup before shutdown"""
        pass


def main(args=None):
    rclpy.init(args=args)
    
    # Example usage - would normally be subclassed
    agent = SwarmAgent("test_01", "test")
    agent.setup()
    
    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.cleanup()
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()