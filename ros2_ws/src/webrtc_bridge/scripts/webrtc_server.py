#!/usr/bin/env python3

import asyncio
import json
import logging
import ssl
import time
from typing import Dict, Set, Optional, Any
import websockets
from websockets.server import WebSocketServerProtocol
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from threading import Thread
import signal

# Try to import aiortc for WebRTC support
try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, RTCDataChannel
    from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRelay
    WEBRTC_AVAILABLE = True
except ImportError:
    print("aiortc not available - WebRTC functionality will be limited")
    WEBRTC_AVAILABLE = False


class WebRTCBridge(Node):
    """WebRTC bridge for low-latency communication between XR interface and ROS"""
    
    def __init__(self):
        super().__init__('webrtc_bridge')
        
        # WebSocket server for signaling
        self.websocket_clients: Set[WebSocketServerProtocol] = set()
        self.webrtc_connections: Dict[str, RTCPeerConnection] = {}
        self.data_channels: Dict[str, RTCDataChannel] = {}
        
        # ROS Publishers and Subscribers
        self.setup_ros_communication()
        
        # Configuration
        self.server_port = 8443
        self.server_host = '0.0.0.0'
        
        # Statistics
        self.stats = {
            'connections': 0,
            'messages_sent': 0,
            'messages_received': 0,
            'bytes_transferred': 0,
            'start_time': time.time()
        }
        
        # Media relay for efficient streaming
        if WEBRTC_AVAILABLE:
            self.media_relay = MediaRelay()
        
        self.get_logger().info("WebRTC Bridge initialized")
    
    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers"""
        # QoS profiles
        self.telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribe to all agent telemetry for forwarding to XR
        self.telemetry_sub = self.create_subscription(
            String, '/swarm/+/telemetry',
            self.handle_agent_telemetry, self.telemetry_qos
        )
        
        # Subscribe to video streams
        self.video_sub = self.create_subscription(
            Image, '/swarm/+/camera/image',
            self.handle_video_stream, self.telemetry_qos
        )
        
        # Subscribe to point cloud data
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/swarm/+/pointcloud',
            self.handle_pointcloud, self.telemetry_qos
        )
        
        # Publisher for commands from XR interface
        self.xr_command_pub = self.create_publisher(
            String, '/xr/command', 10
        )
        
        # Statistics publisher
        self.stats_pub = self.create_publisher(
            String, '/webrtc/stats', self.telemetry_qos
        )
        
        # Stats timer
        self.stats_timer = self.create_timer(5.0, self.publish_stats)
    
    def handle_agent_telemetry(self, msg: String):
        """Forward agent telemetry to XR clients via WebSocket/WebRTC"""
        try:
            telemetry_data = json.loads(msg.data)
            
            # Forward via WebSocket for non-critical data
            websocket_message = {
                'type': 'agent_telemetry',
                'data': telemetry_data,
                'timestamp': time.time()
            }
            
            asyncio.run_coroutine_threadsafe(
                self.broadcast_websocket(websocket_message),
                self.event_loop
            )
            
            # Send critical data via WebRTC data channel for low latency
            if telemetry_data.get('agent_id') in self.data_channels:
                critical_data = {
                    'pos': telemetry_data.get('position', {}),
                    'bat': telemetry_data.get('battery_level', 100),
                    'stat': telemetry_data.get('status', 'unknown')
                }
                
                data_channel = self.data_channels[telemetry_data['agent_id']]
                if data_channel.readyState == 'open':
                    data_channel.send(json.dumps(critical_data))
            
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Error handling telemetry: {e}")
    
    def handle_video_stream(self, msg: Image):
        """Handle video stream from robots"""
        try:
            # Extract agent ID from topic
            agent_id = msg.header.frame_id
            
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Compress for WebRTC transmission
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 70])
            
            # Send via WebRTC if available
            if agent_id in self.data_channels:
                data_channel = self.data_channels[agent_id]
                if data_channel.readyState == 'open':
                    # Send compressed image data
                    video_data = {
                        'type': 'video_frame',
                        'agent_id': agent_id,
                        'data': buffer.tobytes().hex(),  # Convert to hex string
                        'width': cv_image.shape[1],
                        'height': cv_image.shape[0],
                        'timestamp': time.time()
                    }
                    data_channel.send(json.dumps(video_data))
            
        except Exception as e:
            self.get_logger().error(f"Error handling video stream: {e}")
    
    def handle_pointcloud(self, msg: PointCloud2):
        """Handle point cloud data from robots"""
        try:
            # Extract agent ID from topic
            agent_id = msg.header.frame_id
            
            # Simplified point cloud forwarding
            pointcloud_data = {
                'type': 'pointcloud',
                'agent_id': agent_id,
                'width': msg.width,
                'height': msg.height,
                'timestamp': time.time()
            }
            
            # Send via WebSocket (point clouds are typically not as time-critical)
            asyncio.run_coroutine_threadsafe(
                self.broadcast_websocket(pointcloud_data),
                self.event_loop
            )
            
        except Exception as e:
            self.get_logger().error(f"Error handling point cloud: {e}")
    
    async def handle_websocket_client(self, websocket: WebSocketServerProtocol, path: str):
        """Handle WebSocket client connection"""
        self.websocket_clients.add(websocket)
        client_id = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        self.get_logger().info(f"WebSocket client connected: {client_id}")
        
        try:
            async for message in websocket:
                await self.handle_websocket_message(websocket, message, client_id)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")
        finally:
            self.websocket_clients.discard(websocket)
            # Clean up associated WebRTC connection
            if client_id in self.webrtc_connections:
                await self.webrtc_connections[client_id].close()
                del self.webrtc_connections[client_id]
            self.get_logger().info(f"WebSocket client disconnected: {client_id}")
    
    async def handle_websocket_message(self, websocket: WebSocketServerProtocol, message: str, client_id: str):
        """Handle incoming WebSocket message"""
        try:
            data = json.loads(message)
            message_type = data.get('type')
            
            if message_type == 'webrtc_offer':
                await self.handle_webrtc_offer(websocket, data, client_id)
            elif message_type == 'webrtc_answer':
                await self.handle_webrtc_answer(websocket, data, client_id)
            elif message_type == 'ice_candidate':
                await self.handle_ice_candidate(websocket, data, client_id)
            elif message_type == 'swarm_command':
                self.forward_to_ros(data)
            else:
                self.get_logger().warn(f"Unknown message type: {message_type}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON from {client_id}: {message}")
        except Exception as e:
            self.get_logger().error(f"Error handling message from {client_id}: {e}")
    
    async def handle_webrtc_offer(self, websocket: WebSocketServerProtocol, data: Dict[str, Any], client_id: str):
        """Handle WebRTC offer from client"""
        if not WEBRTC_AVAILABLE:
            await websocket.send(json.dumps({
                'type': 'error',
                'message': 'WebRTC not available on server'
            }))
            return
        
        try:
            # Create peer connection
            pc = RTCPeerConnection()
            self.webrtc_connections[client_id] = pc
            
            @pc.on("datachannel")
            def on_datachannel(channel):
                self.get_logger().info(f"Data channel established: {channel.label}")
                self.data_channels[channel.label] = channel
                
                @channel.on("message")
                def on_message(message):
                    # Handle incoming data channel messages
                    try:
                        data = json.loads(message)
                        self.handle_webrtc_data(data)
                    except json.JSONDecodeError:
                        self.get_logger().error(f"Invalid WebRTC data: {message}")
            
            # Set remote description
            await pc.setRemoteDescription(RTCSessionDescription(
                sdp=data['sdp'],
                type=data['type']
            ))
            
            # Create answer
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            
            # Send answer back to client
            await websocket.send(json.dumps({
                'type': 'webrtc_answer',
                'sdp': pc.localDescription.sdp,
                'type': pc.localDescription.type
            }))
            
        except Exception as e:
            self.get_logger().error(f"WebRTC offer error: {e}")
            await websocket.send(json.dumps({
                'type': 'error',
                'message': f'WebRTC setup failed: {str(e)}'
            }))
    
    async def handle_webrtc_answer(self, websocket: WebSocketServerProtocol, data: Dict[str, Any], client_id: str):
        """Handle WebRTC answer from client"""
        if client_id in self.webrtc_connections:
            try:
                pc = self.webrtc_connections[client_id]
                await pc.setRemoteDescription(RTCSessionDescription(
                    sdp=data['sdp'],
                    type=data['type']
                ))
            except Exception as e:
                self.get_logger().error(f"WebRTC answer error: {e}")
    
    async def handle_ice_candidate(self, websocket: WebSocketServerProtocol, data: Dict[str, Any], client_id: str):
        """Handle ICE candidate from client"""
        if client_id in self.webrtc_connections:
            try:
                pc = self.webrtc_connections[client_id]
                candidate = data.get('candidate')
                if candidate:
                    await pc.addIceCandidate(candidate)
            except Exception as e:
                self.get_logger().error(f"ICE candidate error: {e}")
    
    def handle_webrtc_data(self, data: Dict[str, Any]):
        """Handle data received via WebRTC data channel"""
        try:
            # Forward to ROS if it's a command
            if data.get('type') == 'command':
                self.forward_to_ros(data)
            
        except Exception as e:
            self.get_logger().error(f"Error handling WebRTC data: {e}")
    
    def forward_to_ros(self, data: Dict[str, Any]):
        """Forward command from XR interface to ROS"""
        try:
            msg = String()
            msg.data = json.dumps(data)
            self.xr_command_pub.publish(msg)
            
            self.stats['messages_received'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Error forwarding to ROS: {e}")
    
    async def broadcast_websocket(self, message: Dict[str, Any]):
        """Broadcast message to all WebSocket clients"""
        if self.websocket_clients:
            message_str = json.dumps(message)
            disconnected = set()
            
            for client in self.websocket_clients:
                try:
                    await client.send(message_str)
                    self.stats['messages_sent'] += 1
                    self.stats['bytes_transferred'] += len(message_str)
                except websockets.exceptions.ConnectionClosed:
                    disconnected.add(client)
                except Exception as e:
                    self.get_logger().error(f"Error broadcasting to client: {e}")
                    disconnected.add(client)
            
            # Clean up disconnected clients
            self.websocket_clients -= disconnected
    
    def publish_stats(self):
        """Publish WebRTC bridge statistics"""
        uptime = time.time() - self.stats['start_time']
        stats_data = {
            **self.stats,
            'uptime': uptime,
            'connections': len(self.websocket_clients),
            'webrtc_connections': len(self.webrtc_connections),
            'data_channels': len(self.data_channels),
            'messages_per_second': self.stats['messages_sent'] / max(uptime, 1),
            'timestamp': time.time()
        }
        
        msg = String()
        msg.data = json.dumps(stats_data)
        self.stats_pub.publish(msg)
    
    async def start_server(self):
        """Start the WebSocket server"""
        self.get_logger().info(f"Starting WebRTC bridge server on {self.server_host}:{self.server_port}")
        
        # SSL context for secure WebSocket (required for WebRTC)
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        # Note: In production, use proper certificates
        # ssl_context.load_cert_chain("path/to/cert.pem", "path/to/key.pem")
        
        try:
            server = await websockets.serve(
                self.handle_websocket_client,
                self.server_host,
                self.server_port,
                ssl=None  # Disable SSL for development - enable in production
            )
            
            self.get_logger().info("WebRTC bridge server started successfully")
            return server
        
        except Exception as e:
            self.get_logger().error(f"Failed to start server: {e}")
            raise
    
    def run_async_server(self):
        """Run the async server in a separate thread"""
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        
        try:
            server = self.event_loop.run_until_complete(self.start_server())
            self.event_loop.run_forever()
        except KeyboardInterrupt:
            pass
        finally:
            self.event_loop.close()


def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create WebRTC bridge
    bridge = WebRTCBridge()
    
    # Start WebSocket server in separate thread
    server_thread = Thread(target=bridge.run_async_server, daemon=True)
    server_thread.start()
    
    # Handle shutdown gracefully
    def signal_handler(signum, frame):
        bridge.get_logger().info("Shutting down WebRTC bridge...")
        bridge.destroy_node()
        rclpy.shutdown()
        exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # Spin ROS node
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()