#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String, Float32
import sys
import os

# Add the xr_swarm_core package to the path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'xr_swarm_core'))
from xr_swarm_core.swarm_agent_base import SwarmAgent

import json
import time
import math
import numpy as np
import threading
from typing import Dict, Any, List


class DroneAgent(SwarmAgent):
    """Drone agent implementation for XR-Swarm-Bridge"""
    
    def __init__(self, agent_id: str):
        super().__init__(agent_id, "drone")
        
        # Drone-specific capabilities
        self.capabilities = [
            'navigate', 'hover', 'follow', 'search', 'stream_video',
            'altitude_control', 'gimbal_control', 'landing', 'takeoff'
        ]
        
        # Drone state
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.battery_level = 100.0
        self.altitude = 0.0
        self.is_armed = False
        self.flight_mode = "MANUAL"
        
        # Simulation physics
        self.position = np.array([
            np.random.uniform(-50, 50),
            np.random.uniform(-50, 50), 
            0.0
        ])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.target_velocity = np.array([0.0, 0.0, 0.0])
        self.max_speed = 10.0
        self.acceleration = 3.0
        self.home_position = self.position.copy()
        
        # Publishers for drone control
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/drone/{self.agent_id}/cmd_vel', 10
        )
        
        self.takeoff_pub = self.create_publisher(
            String, f'/drone/{self.agent_id}/takeoff', 10
        )
        
        self.land_pub = self.create_publisher(
            String, f'/drone/{self.agent_id}/land', 10
        )
        
        # Subscribers for drone feedback
        self.pose_sub = self.create_subscription(
            PoseStamped, f'/drone/{self.agent_id}/pose',
            self.pose_callback, 10
        )
        
        self.battery_sub = self.create_subscription(
            Float32, f'/drone/{self.agent_id}/battery',
            self.battery_callback, 10
        )
        
        # Video streaming (simulated)
        self.video_pub = self.create_publisher(
            Image, f'/drone/{self.agent_id}/camera/image', 10
        )
        
        # Register drone-specific command handlers
        self.register_drone_handlers()
        
        # Navigation state
        self.waypoints = []
        self.current_waypoint_index = 0
        self.navigation_tolerance = 1.0  # meters
        
        # Simulation and telemetry timers
        self.sim_timer = self.create_timer(0.1, self.simulation_step)
        self.telemetry_timer = self.create_timer(0.5, self.publish_drone_telemetry)
        
        self.get_logger().info(f"DroneAgent {self.agent_id} initialized")
    
    def simulation_step(self):
        """Simulate drone physics and movement"""
        dt = 0.1  # 100ms timestep
        
        # Apply simple physics
        vel_error = self.target_velocity - self.velocity
        max_accel = self.acceleration * dt
        
        for i in range(3):
            if abs(vel_error[i]) > max_accel:
                self.velocity[i] += np.sign(vel_error[i]) * max_accel
            else:
                self.velocity[i] = self.target_velocity[i]
        
        # Enforce maximum speed
        speed = np.linalg.norm(self.velocity)
        if speed > self.max_speed:
            self.velocity = self.velocity / speed * self.max_speed
        
        # Update position
        self.position += self.velocity * dt
        
        # Enforce ground constraint
        if self.position[2] < 0:
            self.position[2] = 0
            self.velocity[2] = max(0, self.velocity[2])
            if self.altitude <= 0.5:
                self.velocity *= 0.9  # Ground friction
        
        # Update internal state
        self.altitude = self.position[2]
        
        # Update pose message
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.header.frame_id = "map"
        self.current_pose.pose.position.x = float(self.position[0])
        self.current_pose.pose.position.y = float(self.position[1])
        self.current_pose.pose.position.z = float(self.position[2])
        
        # Publish pose for swarm coordination
        self.publish_pose(self.current_pose)
        
        # Update navigation if active
        if self.waypoints and self.current_waypoint_index < len(self.waypoints):
            self.check_waypoint_reached()
        
        # Simulate battery drain
        if self.altitude > 0.1:  # Flying
            self.battery_level = max(0.0, self.battery_level - 0.002)
        
        # Publish WebRTC telemetry for low-latency updates
        webrtc_data = {
            'pos': {'x': float(self.position[0]), 'y': float(self.position[1]), 'z': float(self.position[2])},
            'bat': self.battery_level,
            'stat': self.status
        }
        webrtc_msg = String()
        webrtc_msg.data = json.dumps(webrtc_data)
        self.webrtc_pub.publish(webrtc_msg)

    def register_drone_handlers(self):
        """Register drone-specific command handlers"""
        self.command_handlers.update({
            'takeoff': self.handle_takeoff,
            'land': self.handle_land,
            'navigate': self.handle_navigate,
            'hover': self.handle_hover,
            'follow': self.handle_follow,
            'search': self.handle_search,
            'set_altitude': self.handle_set_altitude,
            'arm': self.handle_arm,
            'disarm': self.handle_disarm,
            'return_to_base': self.handle_return_to_base,
            'formation': self.handle_formation
        })
    
    def pose_callback(self, msg: PoseStamped):
        """Handle pose updates from flight controller"""
        self.current_pose = msg
        self.altitude = msg.pose.position.z
        
        # Update pose publisher for swarm coordination
        self.publish_pose(msg)
        
        # Check if we've reached current waypoint
        if self.waypoints and self.current_waypoint_index < len(self.waypoints):
            self.check_waypoint_reached()
    
    def battery_callback(self, msg: Float32):
        """Handle battery level updates"""
        self.battery_level = msg.data
        
        # Check for low battery
        if self.battery_level < 20.0 and self.status != "low_battery":
            self.status = "low_battery"
            self.get_logger().warn(f"Low battery warning: {self.battery_level}%")
            
            # Auto return to base if critically low
            if self.battery_level < 10.0:
                self.handle_return_to_base({})
    
    def handle_takeoff(self, command_data: Dict[str, Any]):
        """Handle takeoff command"""
        altitude = command_data.get('altitude', 10.0)
        
        if self.altitude < 0.5:  # On ground
            self.is_armed = True
            self.status = "taking_off"
            
            # Set upward velocity for takeoff
            takeoff_target = self.position.copy()
            takeoff_target[2] = altitude
            self.navigate_to_position(takeoff_target.tolist())
            
            self.get_logger().info(f"Taking off to {altitude}m altitude")
        else:
            self.get_logger().warn("Already airborne!")
    
    def handle_land(self, command_data: Dict[str, Any]):
        """Handle landing command"""
        self.status = "landing"
        self.waypoints.clear()
        
        # Navigate to ground level
        land_target = self.position.copy()
        land_target[2] = 0.0
        self.navigate_to_position(land_target.tolist())
        
        self.get_logger().info("Landing initiated")
    
    def handle_navigate(self, command_data: Dict[str, Any]):
        """Handle navigation command"""
        position = command_data.get('position')
        waypoints = command_data.get('waypoints')
        
        if waypoints:
            self.waypoints = waypoints
            self.current_waypoint_index = 0
            self.status = "navigating"
            self.navigate_to_waypoint(0)
        elif position:
            self.waypoints = [position]
            self.current_waypoint_index = 0
            self.status = "navigating" 
            self.navigate_to_position(position)
        else:
            self.get_logger().error("Navigate command missing position or waypoints")
    
    def navigate_to_position(self, position: List[float]):
        """Navigate to a specific position using physics simulation"""
        if len(position) < 3:
            position.append(max(5.0, self.altitude))  # Default altitude
        
        target_pos = np.array(position, dtype=float)
        distance_vec = target_pos - self.position
        distance = np.linalg.norm(distance_vec)
        
        if distance > self.navigation_tolerance:
            # Proportional navigation
            direction = distance_vec / distance
            desired_speed = min(self.max_speed * 0.5, distance * 0.8)
            self.target_velocity = direction * desired_speed
        else:
            # Reached target - hover
            self.target_velocity = np.array([0.0, 0.0, 0.0])
            self.status = "hovering"
    
    def navigate_to_waypoint(self, waypoint_index: int):
        """Navigate to specific waypoint"""
        if waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[waypoint_index]
            self.navigate_to_position(waypoint)
    
    def check_waypoint_reached(self):
        """Check if current waypoint has been reached"""
        if self.current_waypoint_index >= len(self.waypoints):
            return
        
        waypoint = self.waypoints[self.current_waypoint_index]
        if len(waypoint) < 3:
            waypoint.append(max(5.0, self.altitude))
        
        target_pos = np.array(waypoint, dtype=float)
        distance = np.linalg.norm(target_pos - self.position)
        
        if distance <= self.navigation_tolerance:
            self.current_waypoint_index += 1
            
            if self.current_waypoint_index >= len(self.waypoints):
                # All waypoints reached
                self.target_velocity = np.array([0.0, 0.0, 0.0])
                self.status = "hovering"
                self.waypoints.clear()
                self.get_logger().info("All waypoints reached")
            else:
                # Move to next waypoint
                self.navigate_to_waypoint(self.current_waypoint_index)
    
    def handle_hover(self, command_data: Dict[str, Any]):
        """Handle hover command"""
        self.status = "hovering"
        self.waypoints.clear()
        self.target_velocity = np.array([0.0, 0.0, 0.0])
        
        self.get_logger().info("Hovering in place")
    
    def handle_follow(self, command_data: Dict[str, Any]):
        """Handle follow command"""
        target_id = command_data.get('target_id')
        offset = command_data.get('offset', [0, 0, 0])
        
        self.status = f"following_{target_id}"
        self.get_logger().info(f"Following target {target_id} with offset {offset}")
        
        # Follow logic would be implemented here
        # For now, just acknowledge the command
    
    def handle_search(self, command_data: Dict[str, Any]):
        """Handle search pattern command"""
        pattern = command_data.get('pattern', 'spiral')
        area = command_data.get('area', {'center': [0, 0], 'size': [50, 50]})
        
        self.status = f"searching_{pattern}"
        
        # Generate search waypoints based on pattern
        search_waypoints = self.generate_search_pattern(pattern, area)
        self.waypoints = search_waypoints
        self.current_waypoint_index = 0
        
        if search_waypoints:
            self.navigate_to_waypoint(0)
        
        self.get_logger().info(f"Starting {pattern} search pattern over area {area}")
    
    def generate_search_pattern(self, pattern: str, area: Dict[str, Any]) -> List[List[float]]:
        """Generate waypoints for search pattern"""
        center = area.get('center', [0, 0])
        size = area.get('size', [50, 50])
        altitude = area.get('altitude', 10.0)
        
        waypoints = []
        
        if pattern == 'spiral':
            # Generate spiral pattern
            radius_step = 2.0
            angle_step = math.pi / 6  # 30 degrees
            max_radius = min(size[0], size[1]) / 2
            
            radius = radius_step
            angle = 0
            
            while radius <= max_radius:
                x = center[0] + radius * math.cos(angle)
                y = center[1] + radius * math.sin(angle)
                waypoints.append([x, y, altitude])
                
                angle += angle_step
                if angle >= 2 * math.pi:
                    angle = 0
                    radius += radius_step
        
        elif pattern == 'grid':
            # Generate grid pattern
            spacing = 5.0
            x_points = int(size[0] / spacing) + 1
            y_points = int(size[1] / spacing) + 1
            
            start_x = center[0] - size[0] / 2
            start_y = center[1] - size[1] / 2
            
            for i in range(x_points):
                for j in range(y_points):
                    x = start_x + i * spacing
                    y = start_y + j * spacing
                    waypoints.append([x, y, altitude])
        
        elif pattern == 'perimeter':
            # Generate perimeter pattern
            corners = [
                [center[0] - size[0]/2, center[1] - size[1]/2, altitude],
                [center[0] + size[0]/2, center[1] - size[1]/2, altitude],
                [center[0] + size[0]/2, center[1] + size[1]/2, altitude],
                [center[0] - size[0]/2, center[1] + size[1]/2, altitude],
                [center[0] - size[0]/2, center[1] - size[1]/2, altitude]  # Return to start
            ]
            waypoints = corners
        
        return waypoints
    
    def handle_set_altitude(self, command_data: Dict[str, Any]):
        """Handle altitude change command"""
        new_altitude = command_data.get('altitude', self.altitude)
        
        current_pos = [
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            new_altitude
        ]
        
        self.navigate_to_position(current_pos)
        self.get_logger().info(f"Changing altitude to {new_altitude}m")
    
    def handle_arm(self, command_data: Dict[str, Any]):
        """Handle arm command"""
        self.is_armed = True
        self.status = "armed"
        self.get_logger().info("Drone armed")
    
    def handle_disarm(self, command_data: Dict[str, Any]):
        """Handle disarm command"""
        self.is_armed = False
        self.status = "disarmed"
        self.get_logger().info("Drone disarmed")
    
    def handle_return_to_base(self, command_data: Dict[str, Any]):
        """Handle return to base command"""
        base_position = command_data.get('base_position', [0, 0, 0])
        
        self.status = "returning_to_base"
        self.waypoints = [base_position]
        self.current_waypoint_index = 0
        self.navigate_to_position(base_position)
        
        self.get_logger().info(f"Returning to base at {base_position}")
    
    def handle_formation(self, command_data: Dict[str, Any]):
        """Handle formation flying command"""
        formation_pos = command_data.get('position', [0, 0, 10])
        formation_type = command_data.get('formation', 'line')
        
        self.navigate_to_position(formation_pos)
        self.status = f"formation_{formation_type}"
        self.get_logger().info(f"Moving to formation position {formation_pos}")
    
    def handle_emergency_stop(self, command_data: Dict[str, Any]):
        """Override emergency stop for drone-specific behavior"""
        super().handle_emergency_stop(command_data)
        
        # Stop all movement immediately
        self.target_velocity = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        
        # Clear waypoints
        self.waypoints.clear()
        
        # Hover in place
        self.status = "emergency_hover"
        self.get_logger().warn("EMERGENCY STOP - Hovering in place")
    
    def publish_drone_telemetry(self):
        """Publish drone-specific telemetry"""
        telemetry = {
            'agent_id': self.agent_id,
            'agent_type': self.agent_type,
            'battery_level': self.battery_level,
            'altitude': self.altitude,
            'is_armed': self.is_armed,
            'flight_mode': self.flight_mode,
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'target_velocity': self.target_velocity.tolist(),
            'waypoints_remaining': len(self.waypoints) - self.current_waypoint_index,
            'status': self.status,
            'timestamp': time.time()
        }
        
        self.publish_telemetry(telemetry)
    
    def setup(self):
        """Setup drone-specific initialization"""
        self.get_logger().info(f"Setting up drone {self.agent_id}")
        # Initialize drone systems, calibrate sensors, etc.
        
    def cleanup(self):
        """Cleanup before shutdown"""
        self.get_logger().info(f"Cleaning up drone {self.agent_id}")
        # Land if in air, disarm, cleanup resources
        if self.is_armed and self.altitude > 0.5:
            self.handle_land({})


def main(args=None):
    rclpy.init(args=args)
    
    # Get agent ID from command line arguments or environment
    import sys
    agent_id = sys.argv[1] if len(sys.argv) > 1 else "drone_01"
    
    drone = DroneAgent(agent_id)
    drone.setup()
    
    try:
        rclpy.spin(drone)
    except KeyboardInterrupt:
        pass
    finally:
        drone.cleanup()
        drone.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()