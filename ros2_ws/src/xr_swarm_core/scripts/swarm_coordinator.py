#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import json
import time
from typing import Dict, List, Any, Optional
import threading
import asyncio
import websockets
import ssl


class SwarmCoordinator(Node):
    """Central coordinator for managing robot swarm and XR interface"""
    
    def __init__(self):
        super().__init__('swarm_coordinator')
        
        self.agents = {}  # agent_id -> agent_info
        self.last_seen = {}  # agent_id -> timestamp
        self.mission_state = "idle"
        self.active_mission = None
        
        # QoS profiles
        self.command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=100
        )
        
        # Publishers
        self.global_command_pub = self.create_publisher(
            String, '/swarm/global/command', self.command_qos
        )
        
        self.mission_status_pub = self.create_publisher(
            String, '/swarm/mission/status', self.telemetry_qos
        )
        
        # Subscribers - listen to all agent status updates
        self.status_sub = self.create_subscription(
            String, '/swarm/+/status',
            self.handle_agent_status, self.telemetry_qos
        )
        
        self.telemetry_sub = self.create_subscription(
            String, '/swarm/+/telemetry',
            self.handle_agent_telemetry, self.telemetry_qos
        )
        
        # WebRTC aggregation for XR interface
        self.webrtc_aggregator_pub = self.create_publisher(
            String, '/webrtc/swarm/aggregated', self.telemetry_qos
        )
        
        # XR interface command subscriber
        self.xr_command_sub = self.create_subscription(
            String, '/xr/command',
            self.handle_xr_command, self.command_qos
        )
        
        # Timers
        self.health_check_timer = self.create_timer(5.0, self.check_agent_health)
        self.status_broadcast_timer = self.create_timer(1.0, self.broadcast_swarm_status)
        
        # WebSocket server for XR interface
        self.websocket_port = 8443
        self.websocket_clients = set()
        self.start_websocket_server()
        
        self.get_logger().info("SwarmCoordinator initialized")
    
    def start_websocket_server(self):
        """Start WebSocket server for XR interface communication"""
        def run_server():
            async def handle_client(websocket, path):
                self.websocket_clients.add(websocket)
                self.get_logger().info(f"XR client connected: {websocket.remote_address}")
                
                try:
                    async for message in websocket:
                        await self.handle_websocket_message(websocket, message)
                except websockets.exceptions.ConnectionClosed:
                    pass
                finally:
                    self.websocket_clients.discard(websocket)
                    self.get_logger().info(f"XR client disconnected")
            
            # SSL context for secure WebSocket
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            # Note: In production, use proper certificates
            
            start_server = websockets.serve(
                handle_client, "0.0.0.0", self.websocket_port
            )
            
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(start_server)
            loop.run_forever()
        
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
    
    async def handle_websocket_message(self, websocket, message):
        """Handle incoming WebSocket message from XR interface"""
        try:
            data = json.loads(message)
            command_type = data.get('type')
            
            if command_type == 'swarm_command':
                self.handle_swarm_command(data)
            elif command_type == 'agent_command':
                self.handle_direct_agent_command(data)
            elif command_type == 'mission_start':
                self.start_mission(data.get('mission'))
            elif command_type == 'mission_stop':
                self.stop_mission()
            elif command_type == 'get_status':
                await self.send_swarm_status(websocket)
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid WebSocket message: {message}")
        except Exception as e:
            self.get_logger().error(f"Error handling WebSocket message: {e}")
    
    async def send_swarm_status(self, websocket):
        """Send current swarm status to XR interface"""
        status = {
            'type': 'swarm_status',
            'agents': self.agents,
            'mission_state': self.mission_state,
            'active_mission': self.active_mission,
            'timestamp': time.time()
        }
        
        try:
            await websocket.send(json.dumps(status))
        except websockets.exceptions.ConnectionClosed:
            pass
    
    async def broadcast_to_xr_clients(self, message):
        """Broadcast message to all connected XR clients"""
        if self.websocket_clients:
            # Remove closed connections
            disconnected = set()
            for client in self.websocket_clients:
                try:
                    await client.send(json.dumps(message))
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
            
            self.websocket_clients -= disconnected
    
    def handle_agent_status(self, msg: String):
        """Handle status updates from individual agents"""
        try:
            status_data = json.loads(msg.data)
            agent_id = status_data.get('agent_id')
            
            if agent_id:
                self.last_seen[agent_id] = time.time()
                
                # Update agent info
                if agent_id not in self.agents:
                    self.agents[agent_id] = {}
                
                self.agents[agent_id].update({
                    'status': status_data.get('status', 'unknown'),
                    'last_heartbeat': time.time(),
                    'uptime': status_data.get('uptime', 0)
                })
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid agent status JSON: {msg.data}")
    
    def handle_agent_telemetry(self, msg: String):
        """Handle telemetry data from agents"""
        try:
            telemetry_data = json.loads(msg.data)
            agent_id = telemetry_data.get('agent_id')
            
            if agent_id and agent_id in self.agents:
                self.agents[agent_id]['telemetry'] = telemetry_data
                
                # Forward to XR interface via WebSocket
                xr_message = {
                    'type': 'agent_telemetry',
                    'agent_id': agent_id,
                    'data': telemetry_data
                }
                
                # Note: This would need proper async handling in a real implementation
                # For now, we'll just log it
                self.get_logger().debug(f"Telemetry from {agent_id}: {telemetry_data}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid telemetry JSON: {msg.data}")
    
    def handle_xr_command(self, msg: String):
        """Handle commands from XR interface via ROS topic"""
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('type')
            
            if command_type == 'global_command':
                self.send_global_command(command_data.get('command'))
            elif command_type == 'agent_command':
                self.send_agent_command(
                    command_data.get('agent_id'),
                    command_data.get('command')
                )
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid XR command JSON: {msg.data}")
    
    def handle_swarm_command(self, command_data: Dict[str, Any]):
        """Handle swarm-level commands"""
        command = command_data.get('command', {})
        command_type = command.get('type')
        
        if command_type == 'formation':
            self.execute_formation_command(command)
        elif command_type == 'search_pattern':
            self.execute_search_pattern(command)
        elif command_type == 'emergency_stop':
            self.emergency_stop_all()
        elif command_type == 'return_to_base':
            self.return_all_to_base()
    
    def handle_direct_agent_command(self, command_data: Dict[str, Any]):
        """Handle direct commands to specific agents"""
        agent_id = command_data.get('agent_id')
        command = command_data.get('command')
        
        if agent_id and command:
            self.send_agent_command(agent_id, command)
    
    def send_global_command(self, command: Dict[str, Any]):
        """Send command to all agents"""
        msg = String()
        msg.data = json.dumps(command)
        self.global_command_pub.publish(msg)
        
        self.get_logger().info(f"Sent global command: {command.get('type', 'unknown')}")
    
    def send_agent_command(self, agent_id: str, command: Dict[str, Any]):
        """Send command to specific agent"""
        # Create agent-specific publisher if needed
        topic_name = f'/swarm/{agent_id}/command'
        
        # For simplicity, we'll use the global command with targeting
        targeted_command = command.copy()
        targeted_command['target_ids'] = [agent_id]
        
        self.send_global_command(targeted_command)
    
    def execute_formation_command(self, command: Dict[str, Any]):
        """Execute formation command"""
        formation_type = command.get('formation', 'line')
        target_agents = command.get('agents', list(self.agents.keys()))
        
        # Generate positions based on formation type
        positions = self.generate_formation_positions(formation_type, len(target_agents))
        
        # Send navigation commands to each agent
        for i, agent_id in enumerate(target_agents):
            if i < len(positions):
                nav_command = {
                    'type': 'navigate',
                    'position': positions[i],
                    'formation': formation_type
                }
                self.send_agent_command(agent_id, nav_command)
    
    def generate_formation_positions(self, formation_type: str, num_agents: int) -> List[List[float]]:
        """Generate positions for formation"""
        positions = []
        
        if formation_type == 'line':
            for i in range(num_agents):
                positions.append([i * 2.0, 0.0, 0.0])  # 2m spacing
        elif formation_type == 'grid':
            cols = int(num_agents ** 0.5) + 1
            for i in range(num_agents):
                row = i // cols
                col = i % cols
                positions.append([col * 2.0, row * 2.0, 0.0])
        elif formation_type == 'circle':
            import math
            radius = max(num_agents * 0.5, 5.0)
            for i in range(num_agents):
                angle = 2 * math.pi * i / num_agents
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                positions.append([x, y, 0.0])
        
        return positions
    
    def execute_search_pattern(self, command: Dict[str, Any]):
        """Execute search pattern command"""
        pattern_type = command.get('pattern', 'spiral')
        search_area = command.get('area', {'center': [0, 0], 'size': [100, 100]})
        
        search_command = {
            'type': 'search',
            'pattern': pattern_type,
            'area': search_area
        }
        
        self.send_global_command(search_command)
    
    def emergency_stop_all(self):
        """Emergency stop all agents"""
        emergency_command = {
            'type': 'emergency_stop',
            'timestamp': time.time()
        }
        
        self.send_global_command(emergency_command)
        self.mission_state = "emergency_stop"
        
        self.get_logger().warn("EMERGENCY STOP - All agents commanded to stop")
    
    def return_all_to_base(self):
        """Command all agents to return to base"""
        return_command = {
            'type': 'return_to_base',
            'timestamp': time.time()
        }
        
        self.send_global_command(return_command)
        self.mission_state = "returning"
    
    def start_mission(self, mission_data: Dict[str, Any]):
        """Start a new mission"""
        self.active_mission = mission_data
        self.mission_state = "active"
        
        # Send mission start command to all agents
        mission_command = {
            'type': 'mission_start',
            'mission': mission_data,
            'timestamp': time.time()
        }
        
        self.send_global_command(mission_command)
        self.get_logger().info(f"Mission started: {mission_data.get('name', 'Unnamed')}")
    
    def stop_mission(self):
        """Stop current mission"""
        if self.active_mission:
            mission_command = {
                'type': 'mission_stop',
                'timestamp': time.time()
            }
            
            self.send_global_command(mission_command)
            
            self.active_mission = None
            self.mission_state = "idle"
            
            self.get_logger().info("Mission stopped")
    
    def check_agent_health(self):
        """Check health of all agents"""
        current_time = time.time()
        timeout_threshold = 10.0  # 10 seconds
        
        unhealthy_agents = []
        
        for agent_id in list(self.agents.keys()):
            last_seen = self.last_seen.get(agent_id, 0)
            if current_time - last_seen > timeout_threshold:
                unhealthy_agents.append(agent_id)
                self.agents[agent_id]['status'] = 'timeout'
        
        if unhealthy_agents:
            self.get_logger().warn(f"Unhealthy agents detected: {unhealthy_agents}")
    
    def broadcast_swarm_status(self):
        """Broadcast current swarm status"""
        status = {
            'type': 'swarm_status_update',
            'total_agents': len(self.agents),
            'active_agents': len([a for a in self.agents.values() if a.get('status') != 'timeout']),
            'mission_state': self.mission_state,
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.mission_status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    coordinator = SwarmCoordinator()
    
    try:
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()