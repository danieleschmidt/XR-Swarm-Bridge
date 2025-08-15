#!/usr/bin/env python3

"""
Enhanced WebRTC Bridge with Production-Ready Features
Includes advanced monitoring, security, resilience, and performance optimization
"""

import asyncio
import json
import logging
import ssl
import time
import hashlib
import secrets
import struct
from typing import Dict, Set, Optional, Any, List, Tuple
from dataclasses import dataclass, asdict
from enum import Enum
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
from threading import Thread, Lock
import signal
import queue
import statistics
from collections import defaultdict, deque
from contextlib import asynccontextmanager

# Import custom resilience engine
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'xr_swarm_core'))
from xr_swarm_core.advanced_resilience import AdvancedResilienceEngine

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

# Try to import aiortc for WebRTC support
try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, RTCDataChannel, RTCIceCandidate
    from aiortc.contrib.media import MediaBlackhole, MediaPlayer, MediaRelay
    WEBRTC_AVAILABLE = True
except ImportError:
    print("aiortc not available - WebRTC functionality will be limited")
    WEBRTC_AVAILABLE = False


class ConnectionState(Enum):
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RECONNECTING = "reconnecting"
    DISCONNECTED = "disconnected"
    FAILED = "failed"


class MessageType(Enum):
    WEBRTC_OFFER = "webrtc_offer"
    WEBRTC_ANSWER = "webrtc_answer"
    ICE_CANDIDATE = "ice_candidate"
    SWARM_COMMAND = "swarm_command"
    TELEMETRY_DATA = "telemetry_data"
    VIDEO_FRAME = "video_frame"
    HEARTBEAT = "heartbeat"
    AUTHENTICATION = "authentication"
    ERROR = "error"


@dataclass
class ConnectionMetrics:
    client_id: str
    connect_time: float
    last_activity: float
    bytes_sent: int
    bytes_received: int
    messages_sent: int
    messages_received: int
    latency_samples: List[float]
    packet_loss_rate: float
    connection_state: ConnectionState
    authentication_status: bool
    error_count: int
    bandwidth_usage: float


@dataclass
class SecurityToken:
    token: str
    client_id: str
    issued_at: float
    expires_at: float
    permissions: List[str]
    session_id: str


class RateLimiter:
    """Token bucket rate limiter for API protection"""
    
    def __init__(self, max_tokens: int, refill_rate: float):
        self.max_tokens = max_tokens
        self.refill_rate = refill_rate
        self.tokens = max_tokens
        self.last_refill = time.time()
        self.lock = Lock()
    
    def consume(self, tokens: int = 1) -> bool:
        """Attempt to consume tokens, return True if successful"""
        with self.lock:
            current_time = time.time()
            
            # Refill tokens based on elapsed time
            elapsed = current_time - self.last_refill
            self.tokens = min(self.max_tokens, self.tokens + elapsed * self.refill_rate)
            self.last_refill = current_time
            
            if self.tokens >= tokens:
                self.tokens -= tokens
                return True
            
            return False


class EnhancedWebRTCBridge(Node):
    """Enhanced WebRTC bridge with production-ready features"""
    
    def __init__(self):
        super().__init__('enhanced_webrtc_bridge')
        
        # Core WebRTC components
        self.websocket_clients: Set[WebSocketServerProtocol] = set()
        self.webrtc_connections: Dict[str, RTCPeerConnection] = {}
        self.data_channels: Dict[str, RTCDataChannel] = {}
        
        # Enhanced monitoring and metrics
        self.connection_metrics: Dict[str, ConnectionMetrics] = {}
        self.global_metrics = {
            'total_connections': 0,
            'active_connections': 0,
            'total_bytes_transferred': 0,
            'average_latency': 0.0,
            'error_rate': 0.0,
            'uptime': 0.0,
            'start_time': time.time()
        }
        
        # Security and authentication
        self.security_tokens: Dict[str, SecurityToken] = {}
        self.rate_limiters: Dict[str, RateLimiter] = {}
        self.blocked_ips: Set[str] = set()
        self.failed_auth_attempts: Dict[str, List[float]] = defaultdict(list)
        
        # Performance optimization
        self.message_queues: Dict[str, queue.Queue] = {}
        self.compression_enabled = True
        self.adaptive_quality = True
        self.bandwidth_monitors: Dict[str, deque] = defaultdict(lambda: deque(maxlen=100))
        
        # Resilience and self-healing
        self.resilience_engine = AdvancedResilienceEngine()
        self.health_check_interval = 30.0
        self.reconnection_strategies = {}
        
        # Configuration
        self.server_port = 8443
        self.server_host = '0.0.0.0'
        self.max_connections = 1000
        self.heartbeat_interval = 30.0
        self.max_message_size = 1024 * 1024  # 1MB
        
        # Initialize components
        self.setup_ros_communication()
        self.setup_security()
        self.setup_performance_monitoring()
        
        if CV_BRIDGE_AVAILABLE:
            self.bridge = CvBridge()
        
        if WEBRTC_AVAILABLE:
            self.media_relay = MediaRelay()
        
        self.get_logger().info("Enhanced WebRTC Bridge initialized with advanced features")
    
    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers with enhanced QoS"""
        
        # High-performance QoS for critical data
        self.critical_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=50,
            history_depth=50
        )
        
        # Best-effort QoS for high-frequency telemetry
        self.telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Enhanced subscribers with error handling
        self.telemetry_sub = self.create_subscription(
            String, '/swarm/+/telemetry',
            self.handle_agent_telemetry_enhanced, self.telemetry_qos
        )
        
        self.video_sub = self.create_subscription(
            Image, '/swarm/+/camera/image',
            self.handle_video_stream_enhanced, self.telemetry_qos
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/swarm/+/pointcloud',
            self.handle_pointcloud_enhanced, self.telemetry_qos
        )
        
        # Enhanced publishers
        self.xr_command_pub = self.create_publisher(
            String, '/xr/command', self.critical_qos
        )
        
        self.stats_pub = self.create_publisher(
            String, '/webrtc/enhanced_stats', self.telemetry_qos
        )
        
        self.health_pub = self.create_publisher(
            String, '/webrtc/health', self.telemetry_qos
        )
        
        # Enhanced timers
        self.stats_timer = self.create_timer(5.0, self.publish_enhanced_stats)
        self.health_timer = self.create_timer(self.health_check_interval, self.perform_health_check)
        self.cleanup_timer = self.create_timer(60.0, self.cleanup_stale_connections)
    
    def setup_security(self):
        """Setup security components"""
        
        # Generate server key for token signing
        self.server_secret = secrets.token_urlsafe(32)
        
        # Default rate limiter for general API access
        self.default_rate_limiter = RateLimiter(max_tokens=100, refill_rate=10.0)
        
        # Authentication rate limiter (stricter)
        self.auth_rate_limiter = RateLimiter(max_tokens=5, refill_rate=0.5)
        
        self.get_logger().info("Security components initialized")
    
    def setup_performance_monitoring(self):
        """Setup performance monitoring components"""
        
        # Performance metrics collection
        self.performance_metrics = {
            'message_processing_times': deque(maxlen=1000),
            'websocket_latencies': deque(maxlen=1000),
            'webrtc_latencies': deque(maxlen=1000),
            'bandwidth_utilization': deque(maxlen=100),
            'cpu_usage': deque(maxlen=100),
            'memory_usage': deque(maxlen=100)
        }
        
        # Start performance monitoring thread
        self.performance_thread = Thread(target=self.monitor_performance, daemon=True)
        self.performance_thread.start()
        
        self.get_logger().info("Performance monitoring initialized")
    
    def monitor_performance(self):
        """Monitor system performance in background thread"""
        
        while True:
            try:
                # Collect system metrics
                cpu_usage = self.get_cpu_usage()
                memory_usage = self.get_memory_usage()
                
                self.performance_metrics['cpu_usage'].append(cpu_usage)
                self.performance_metrics['memory_usage'].append(memory_usage)
                
                # Update resilience engine with health data
                health_data = {
                    'cpu_usage': cpu_usage,
                    'memory_usage': memory_usage,
                    'network_latency': self.calculate_average_latency(),
                    'communication_quality': self.calculate_communication_quality(),
                    'active_connections': len(self.websocket_clients),
                    'error_rate': self.global_metrics['error_rate']
                }
                
                self.resilience_engine.update_agent_health('webrtc_bridge', health_data)
                
                time.sleep(10.0)  # Monitor every 10 seconds
                
            except Exception as e:
                self.get_logger().error(f"Error in performance monitoring: {e}")
                time.sleep(30.0)  # Longer sleep on error
    
    def get_cpu_usage(self) -> float:
        """Get current CPU usage percentage"""
        try:
            import psutil
            return psutil.cpu_percent(interval=1)
        except ImportError:
            return 0.0  # Fallback if psutil not available
    
    def get_memory_usage(self) -> float:
        """Get current memory usage percentage"""
        try:
            import psutil
            return psutil.virtual_memory().percent
        except ImportError:
            return 0.0  # Fallback if psutil not available
    
    async def authenticate_client(self, websocket: WebSocketServerProtocol, 
                                auth_data: Dict[str, Any]) -> Optional[SecurityToken]:
        """Authenticate client and return security token"""
        
        client_ip = websocket.remote_address[0]
        
        # Check if IP is blocked
        if client_ip in self.blocked_ips:
            raise Exception("IP address is blocked")
        
        # Rate limit authentication attempts
        if not self.auth_rate_limiter.consume():
            raise Exception("Authentication rate limit exceeded")
        
        # Validate authentication data
        username = auth_data.get('username')
        password = auth_data.get('password')
        client_id = auth_data.get('client_id', f"client_{int(time.time())}")
        
        if not username or not password:
            self.record_failed_auth_attempt(client_ip)
            raise Exception("Username and password required")
        
        # Simple authentication (in production, use proper auth service)
        if self.validate_credentials(username, password):
            # Generate security token
            token = SecurityToken(
                token=secrets.token_urlsafe(32),
                client_id=client_id,
                issued_at=time.time(),
                expires_at=time.time() + 3600,  # 1 hour expiry
                permissions=['read', 'write', 'control'],
                session_id=secrets.token_urlsafe(16)
            )
            
            self.security_tokens[token.token] = token
            
            # Create rate limiter for this client
            self.rate_limiters[client_id] = RateLimiter(max_tokens=200, refill_rate=20.0)
            
            self.get_logger().info(f"Client {client_id} authenticated successfully")
            return token
        else:
            self.record_failed_auth_attempt(client_ip)
            raise Exception("Invalid credentials")
    
    def validate_credentials(self, username: str, password: str) -> bool:
        """Validate user credentials (placeholder implementation)"""
        # In production, this would check against proper auth service
        valid_users = {
            'admin': 'secure_password_123',
            'operator': 'operator_pass_456',
            'viewer': 'viewer_pass_789'
        }
        
        return valid_users.get(username) == password
    
    def record_failed_auth_attempt(self, client_ip: str):
        """Record failed authentication attempt and handle blocking"""
        
        current_time = time.time()
        self.failed_auth_attempts[client_ip].append(current_time)
        
        # Clean up old attempts (older than 1 hour)
        self.failed_auth_attempts[client_ip] = [
            t for t in self.failed_auth_attempts[client_ip] 
            if current_time - t < 3600
        ]
        
        # Block IP if too many failed attempts
        if len(self.failed_auth_attempts[client_ip]) >= 5:
            self.blocked_ips.add(client_ip)
            self.get_logger().warning(f"Blocked IP {client_ip} due to repeated auth failures")
    
    def verify_token(self, token: str) -> Optional[SecurityToken]:
        """Verify security token and return token data if valid"""
        
        security_token = self.security_tokens.get(token)
        if not security_token:
            return None
        
        # Check if token is expired
        if time.time() > security_token.expires_at:
            del self.security_tokens[token]
            return None
        
        return security_token
    
    async def handle_websocket_client_enhanced(self, websocket: WebSocketServerProtocol, path: str):
        """Enhanced WebSocket client handler with security and monitoring"""
        
        client_id = None
        
        try:
            client_ip = websocket.remote_address[0]
            
            # Check connection limits
            if len(self.websocket_clients) >= self.max_connections:
                await websocket.close(code=1013, reason="Server overloaded")
                return
            
            # Initial connection setup
            self.websocket_clients.add(websocket)
            
            # Initialize connection metrics
            temp_client_id = f"temp_{client_ip}_{int(time.time())}"
            self.connection_metrics[temp_client_id] = ConnectionMetrics(
                client_id=temp_client_id,
                connect_time=time.time(),
                last_activity=time.time(),
                bytes_sent=0,
                bytes_received=0,
                messages_sent=0,
                messages_received=0,
                latency_samples=[],
                packet_loss_rate=0.0,
                connection_state=ConnectionState.CONNECTING,
                authentication_status=False,
                error_count=0,
                bandwidth_usage=0.0
            )
            
            self.get_logger().info(f"WebSocket client connecting from {client_ip}")
            
            # Handle messages
            async for message in websocket:
                try:
                    # Rate limiting
                    if client_id and not self.rate_limiters.get(client_id, self.default_rate_limiter).consume():
                        await self.send_error(websocket, "Rate limit exceeded")
                        continue
                    
                    # Message size check
                    if len(message) > self.max_message_size:
                        await self.send_error(websocket, "Message too large")
                        continue
                    
                    # Process message
                    processed_client_id = await self.handle_websocket_message_enhanced(
                        websocket, message, temp_client_id
                    )
                    
                    if processed_client_id and not client_id:
                        client_id = processed_client_id
                        # Update metrics with real client ID
                        if temp_client_id in self.connection_metrics:
                            self.connection_metrics[client_id] = self.connection_metrics[temp_client_id]
                            self.connection_metrics[client_id].client_id = client_id
                            del self.connection_metrics[temp_client_id]
                    
                    # Update activity timestamp
                    metrics_key = client_id or temp_client_id
                    if metrics_key in self.connection_metrics:
                        self.connection_metrics[metrics_key].last_activity = time.time()
                        self.connection_metrics[metrics_key].messages_received += 1
                        self.connection_metrics[metrics_key].bytes_received += len(message)
                    
                except json.JSONDecodeError:
                    await self.send_error(websocket, "Invalid JSON format")
                except Exception as e:
                    self.get_logger().error(f"Error processing message: {e}")
                    await self.send_error(websocket, "Message processing error")
                    
                    # Update error count
                    metrics_key = client_id or temp_client_id
                    if metrics_key in self.connection_metrics:
                        self.connection_metrics[metrics_key].error_count += 1
        
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")
        finally:
            # Cleanup
            self.websocket_clients.discard(websocket)
            
            if client_id:
                # Update connection state
                if client_id in self.connection_metrics:
                    self.connection_metrics[client_id].connection_state = ConnectionState.DISCONNECTED
                
                # Clean up associated WebRTC connection
                if client_id in self.webrtc_connections:
                    try:
                        await self.webrtc_connections[client_id].close()
                    except:
                        pass
                    del self.webrtc_connections[client_id]
                
                # Clean up data channels
                for channel_id in list(self.data_channels.keys()):
                    if channel_id.startswith(client_id):
                        del self.data_channels[channel_id]
                
                self.get_logger().info(f"WebSocket client {client_id} disconnected")
            
            # Clean up temporary metrics
            if temp_client_id in self.connection_metrics:
                del self.connection_metrics[temp_client_id]
    
    async def handle_websocket_message_enhanced(self, websocket: WebSocketServerProtocol, 
                                              message: str, client_id: str) -> Optional[str]:
        """Enhanced WebSocket message handler with security and monitoring"""
        
        start_time = time.time()
        
        try:
            data = json.loads(message)
            message_type = MessageType(data.get('type', 'unknown'))
            
            # Authentication handling
            if message_type == MessageType.AUTHENTICATION:
                auth_data = data.get('auth', {})
                token = await self.authenticate_client(websocket, auth_data)
                
                if token:
                    # Update connection metrics
                    self.connection_metrics[client_id].authentication_status = True
                    self.connection_metrics[client_id].connection_state = ConnectionState.CONNECTED
                    self.connection_metrics[client_id].client_id = token.client_id
                    
                    response = {
                        'type': 'authentication_success',
                        'token': token.token,
                        'client_id': token.client_id,
                        'session_id': token.session_id,
                        'expires_at': token.expires_at
                    }
                    
                    await websocket.send(json.dumps(response))
                    return token.client_id
                else:
                    await self.send_error(websocket, "Authentication failed")
                    return None
            
            # Verify authentication for other message types
            token_str = data.get('token')
            if not token_str:
                await self.send_error(websocket, "Authentication token required")
                return None
            
            security_token = self.verify_token(token_str)
            if not security_token:
                await self.send_error(websocket, "Invalid or expired token")
                return None
            
            # Update client ID from token
            client_id = security_token.client_id
            
            # Process authenticated messages
            if message_type == MessageType.WEBRTC_OFFER:
                await self.handle_webrtc_offer_enhanced(websocket, data, client_id)
            elif message_type == MessageType.WEBRTC_ANSWER:
                await self.handle_webrtc_answer_enhanced(websocket, data, client_id)
            elif message_type == MessageType.ICE_CANDIDATE:
                await self.handle_ice_candidate_enhanced(websocket, data, client_id)
            elif message_type == MessageType.SWARM_COMMAND:
                await self.handle_swarm_command_enhanced(websocket, data, client_id, security_token)
            elif message_type == MessageType.HEARTBEAT:
                await self.handle_heartbeat(websocket, data, client_id)
            else:
                await self.send_error(websocket, f"Unknown message type: {message_type.value}")
            
            # Record processing time
            processing_time = time.time() - start_time
            self.performance_metrics['message_processing_times'].append(processing_time)
            
            return client_id
            
        except ValueError as e:
            await self.send_error(websocket, f"Invalid message type: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error handling message: {e}")
            await self.send_error(websocket, "Internal server error")
            return None
    
    async def handle_swarm_command_enhanced(self, websocket: WebSocketServerProtocol,
                                          data: Dict[str, Any], client_id: str,
                                          security_token: SecurityToken):
        """Enhanced swarm command handler with permission checking"""
        
        # Check permissions
        if 'control' not in security_token.permissions:
            await self.send_error(websocket, "Insufficient permissions for control commands")
            return
        
        # Validate command structure
        command = data.get('command', {})
        if not command:
            await self.send_error(websocket, "Missing command data")
            return
        
        # Add metadata
        enhanced_command = {
            **command,
            'client_id': client_id,
            'session_id': security_token.session_id,
            'timestamp': time.time(),
            'source': 'webrtc_bridge'
        }
        
        # Forward to ROS with enhanced error handling
        try:
            await self.resilience_engine.execute_with_circuit_breaker(
                'command_service',
                self.forward_to_ros_enhanced,
                enhanced_command
            )
            
            # Send acknowledgment
            response = {
                'type': 'command_acknowledged',
                'command_id': command.get('id', 'unknown'),
                'timestamp': time.time()
            }
            await websocket.send(json.dumps(response))
            
        except Exception as e:
            self.get_logger().error(f"Error forwarding command: {e}")
            await self.send_error(websocket, f"Command execution failed: {str(e)}")
    
    async def forward_to_ros_enhanced(self, command_data: Dict[str, Any]):
        """Enhanced ROS message forwarding with retry logic"""
        
        max_retries = 3
        retry_delay = 1.0
        
        for attempt in range(max_retries):
            try:
                msg = String()
                msg.data = json.dumps(command_data)
                self.xr_command_pub.publish(msg)
                
                self.global_metrics['total_bytes_transferred'] += len(msg.data)
                return
                
            except Exception as e:
                if attempt < max_retries - 1:
                    self.get_logger().warning(f"ROS publish attempt {attempt + 1} failed: {e}, retrying...")
                    await asyncio.sleep(retry_delay * (attempt + 1))
                else:
                    self.get_logger().error(f"Failed to publish to ROS after {max_retries} attempts: {e}")
                    raise
    
    async def handle_heartbeat(self, websocket: WebSocketServerProtocol,
                             data: Dict[str, Any], client_id: str):
        """Handle heartbeat messages for connection monitoring"""
        
        timestamp = data.get('timestamp', time.time())
        
        # Calculate latency if timestamp provided
        if timestamp:
            latency = (time.time() - timestamp) * 1000  # Convert to ms
            
            if client_id in self.connection_metrics:
                self.connection_metrics[client_id].latency_samples.append(latency)
                
                # Keep only recent samples
                if len(self.connection_metrics[client_id].latency_samples) > 100:
                    self.connection_metrics[client_id].latency_samples = \
                        self.connection_metrics[client_id].latency_samples[-100:]
        
        # Send heartbeat response
        response = {
            'type': 'heartbeat_response',
            'timestamp': time.time(),
            'server_time': time.time()
        }
        
        await websocket.send(json.dumps(response))
    
    async def handle_webrtc_offer_enhanced(self, websocket: WebSocketServerProtocol,
                                         data: Dict[str, Any], client_id: str):
        """Enhanced WebRTC offer handler with monitoring"""
        
        if not WEBRTC_AVAILABLE:
            await self.send_error(websocket, "WebRTC not available on server")
            return
        
        try:
            # Create peer connection with enhanced configuration
            ice_servers = [
                {"urls": "stun:stun.l.google.com:19302"},
                {"urls": "stun:stun1.l.google.com:19302"}
            ]
            
            pc = RTCPeerConnection(configuration={"iceServers": ice_servers})
            self.webrtc_connections[client_id] = pc
            
            # Enhanced data channel handling
            @pc.on("datachannel")
            def on_datachannel(channel):
                self.get_logger().info(f"Data channel established: {channel.label} for client {client_id}")
                channel_id = f"{client_id}_{channel.label}"
                self.data_channels[channel_id] = channel
                
                @channel.on("message")
                def on_message(message):
                    self.handle_webrtc_data_enhanced(message, client_id, channel.label)
                
                @channel.on("close")
                def on_close():
                    self.get_logger().info(f"Data channel closed: {channel.label} for client {client_id}")
                    if channel_id in self.data_channels:
                        del self.data_channels[channel_id]
            
            # Connection state monitoring
            @pc.on("connectionstatechange")
            async def on_connectionstatechange():
                state = pc.connectionState
                self.get_logger().info(f"WebRTC connection state changed to {state} for client {client_id}")
                
                if client_id in self.connection_metrics:
                    if state == "connected":
                        self.connection_metrics[client_id].connection_state = ConnectionState.CONNECTED
                    elif state == "failed":
                        self.connection_metrics[client_id].connection_state = ConnectionState.FAILED
                    elif state == "disconnected":
                        self.connection_metrics[client_id].connection_state = ConnectionState.DISCONNECTED
            
            # Set remote description
            await pc.setRemoteDescription(RTCSessionDescription(
                sdp=data['sdp'],
                type=data['type']
            ))
            
            # Create answer
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            
            # Send answer
            response = {
                'type': 'webrtc_answer',
                'sdp': pc.localDescription.sdp,
                'type': pc.localDescription.type,
                'timestamp': time.time()
            }
            
            await websocket.send(json.dumps(response))
            
        except Exception as e:
            self.get_logger().error(f"WebRTC offer error for client {client_id}: {e}")
            await self.send_error(websocket, f"WebRTC setup failed: {str(e)}")
    
    async def handle_webrtc_answer_enhanced(self, websocket: WebSocketServerProtocol,
                                          data: Dict[str, Any], client_id: str):
        """Enhanced WebRTC answer handler"""
        
        if client_id in self.webrtc_connections:
            try:
                pc = self.webrtc_connections[client_id]
                await pc.setRemoteDescription(RTCSessionDescription(
                    sdp=data['sdp'],
                    type=data['type']
                ))
                
                self.get_logger().info(f"WebRTC answer processed for client {client_id}")
                
            except Exception as e:
                self.get_logger().error(f"WebRTC answer error for client {client_id}: {e}")
                await self.send_error(websocket, f"WebRTC answer processing failed: {str(e)}")
    
    async def handle_ice_candidate_enhanced(self, websocket: WebSocketServerProtocol,
                                          data: Dict[str, Any], client_id: str):
        """Enhanced ICE candidate handler"""
        
        if client_id in self.webrtc_connections:
            try:
                pc = self.webrtc_connections[client_id]
                candidate_data = data.get('candidate')
                
                if candidate_data:
                    candidate = RTCIceCandidate(
                        component=candidate_data.get('component', 1),
                        foundation=candidate_data.get('foundation', ''),
                        ip=candidate_data.get('ip', ''),
                        port=candidate_data.get('port', 0),
                        priority=candidate_data.get('priority', 0),
                        protocol=candidate_data.get('protocol', 'udp'),
                        type=candidate_data.get('type', 'host')
                    )
                    
                    await pc.addIceCandidate(candidate)
                    
            except Exception as e:
                self.get_logger().error(f"ICE candidate error for client {client_id}: {e}")
    
    def handle_webrtc_data_enhanced(self, message: str, client_id: str, channel_label: str):
        """Enhanced WebRTC data channel message handler"""
        
        try:
            # Update bandwidth monitoring
            self.update_bandwidth_usage(client_id, len(message))
            
            # Parse and process message
            if isinstance(message, str):
                data = json.loads(message)
            else:
                # Handle binary data
                data = {'type': 'binary_data', 'size': len(message)}
            
            # Forward to ROS if it's a command
            if data.get('type') == 'command':
                enhanced_data = {
                    **data,
                    'client_id': client_id,
                    'channel': channel_label,
                    'timestamp': time.time(),
                    'source': 'webrtc_data_channel'
                }
                
                # Use async wrapper for ROS publishing
                asyncio.create_task(self.forward_to_ros_enhanced(enhanced_data))
            
        except Exception as e:
            self.get_logger().error(f"Error handling WebRTC data from {client_id}: {e}")
    
    def update_bandwidth_usage(self, client_id: str, bytes_transferred: int):
        """Update bandwidth usage monitoring"""
        
        current_time = time.time()
        
        # Add to bandwidth monitor
        self.bandwidth_monitors[client_id].append((current_time, bytes_transferred))
        
        # Calculate bandwidth usage (bytes per second)
        if len(self.bandwidth_monitors[client_id]) > 1:
            time_window = 10.0  # 10 second window
            cutoff_time = current_time - time_window
            
            # Filter recent data
            recent_data = [(t, b) for t, b in self.bandwidth_monitors[client_id] if t > cutoff_time]
            
            if len(recent_data) > 1:
                total_bytes = sum(b for _, b in recent_data)
                time_span = recent_data[-1][0] - recent_data[0][0]
                
                if time_span > 0:
                    bandwidth = total_bytes / time_span
                    
                    if client_id in self.connection_metrics:
                        self.connection_metrics[client_id].bandwidth_usage = bandwidth
    
    async def handle_agent_telemetry_enhanced(self, msg: String):
        """Enhanced agent telemetry handler with adaptive compression"""
        
        try:
            telemetry_data = json.loads(msg.data)
            
            # Add server-side timestamp
            telemetry_data['server_timestamp'] = time.time()
            
            # Adaptive quality based on connection load
            if self.adaptive_quality:
                telemetry_data = self.apply_adaptive_quality(telemetry_data)
            
            # Compress if enabled
            if self.compression_enabled:
                telemetry_data = self.compress_telemetry_data(telemetry_data)
            
            # Broadcast via WebSocket with load balancing
            await self.broadcast_websocket_enhanced({
                'type': 'agent_telemetry',
                'data': telemetry_data,
                'timestamp': time.time()
            })
            
            # Send critical data via WebRTC for low latency
            agent_id = telemetry_data.get('agent_id')
            if agent_id:
                critical_data = {
                    'type': 'critical_telemetry',
                    'agent_id': agent_id,
                    'pos': telemetry_data.get('position', {}),
                    'bat': telemetry_data.get('battery_level', 100),
                    'stat': telemetry_data.get('status', 'unknown'),
                    'timestamp': time.time()
                }
                
                await self.send_via_webrtc_data_channels(critical_data)
            
        except Exception as e:
            self.get_logger().error(f"Error handling enhanced telemetry: {e}")
    
    def apply_adaptive_quality(self, telemetry_data: Dict[str, Any]) -> Dict[str, Any]:
        """Apply adaptive quality based on system load"""
        
        # Calculate system load
        active_connections = len(self.websocket_clients)
        avg_latency = self.calculate_average_latency()
        
        # Reduce quality if system is under load
        if active_connections > 50 or avg_latency > 200:
            # Reduce precision of position data
            if 'position' in telemetry_data and isinstance(telemetry_data['position'], list):
                telemetry_data['position'] = [round(x, 2) for x in telemetry_data['position']]
            
            # Remove non-critical fields
            non_critical_fields = ['debug_info', 'extended_telemetry', 'raw_sensor_data']
            for field in non_critical_fields:
                telemetry_data.pop(field, None)
        
        return telemetry_data
    
    def compress_telemetry_data(self, telemetry_data: Dict[str, Any]) -> Dict[str, Any]:
        """Compress telemetry data for bandwidth efficiency"""
        
        # Simple compression - remove redundant fields, use abbreviations
        compressed = {}
        
        field_mapping = {
            'agent_id': 'id',
            'position': 'pos',
            'battery_level': 'bat',
            'status': 'stat',
            'timestamp': 'ts',
            'velocity': 'vel',
            'orientation': 'ori'
        }
        
        for key, value in telemetry_data.items():
            compressed_key = field_mapping.get(key, key)
            compressed[compressed_key] = value
        
        return compressed
    
    async def send_via_webrtc_data_channels(self, data: Dict[str, Any]):
        """Send data via all active WebRTC data channels"""
        
        message = json.dumps(data)
        
        for channel_id, channel in list(self.data_channels.items()):
            try:
                if channel.readyState == 'open':
                    channel.send(message)
                else:
                    # Clean up closed channels
                    del self.data_channels[channel_id]
            except Exception as e:
                self.get_logger().error(f"Error sending via data channel {channel_id}: {e}")
    
    async def broadcast_websocket_enhanced(self, message: Dict[str, Any]):
        """Enhanced WebSocket broadcasting with load balancing"""
        
        if not self.websocket_clients:
            return
        
        message_str = json.dumps(message)
        message_size = len(message_str)
        
        # Track bandwidth
        self.global_metrics['total_bytes_transferred'] += message_size * len(self.websocket_clients)
        
        # Send to clients in batches to prevent blocking
        batch_size = 50
        client_list = list(self.websocket_clients)
        
        for i in range(0, len(client_list), batch_size):
            batch = client_list[i:i + batch_size]
            await self.send_to_client_batch(batch, message_str)
    
    async def send_to_client_batch(self, clients: List[WebSocketServerProtocol], message: str):
        """Send message to a batch of clients concurrently"""
        
        async def send_to_client(client):
            try:
                await client.send(message)
                
                # Update metrics if we can identify the client
                client_id = self.get_client_id_from_websocket(client)
                if client_id and client_id in self.connection_metrics:
                    self.connection_metrics[client_id].messages_sent += 1
                    self.connection_metrics[client_id].bytes_sent += len(message)
                
            except websockets.exceptions.ConnectionClosed:
                self.websocket_clients.discard(client)
            except Exception as e:
                self.get_logger().error(f"Error sending to client: {e}")
                self.websocket_clients.discard(client)
        
        # Send to all clients in batch concurrently
        await asyncio.gather(*[send_to_client(client) for client in clients], return_exceptions=True)
    
    def get_client_id_from_websocket(self, websocket: WebSocketServerProtocol) -> Optional[str]:
        """Get client ID from WebSocket connection"""
        
        # This would require maintaining a reverse mapping in a real implementation
        for client_id, metrics in self.connection_metrics.items():
            # This is a simplified approach - in production, maintain proper mapping
            if metrics.connection_state == ConnectionState.CONNECTED:
                return client_id
        
        return None
    
    async def handle_video_stream_enhanced(self, msg: Image):
        """Enhanced video stream handler with adaptive quality"""
        
        if not CV_BRIDGE_AVAILABLE:
            return
        
        try:
            agent_id = msg.header.frame_id
            
            # Convert and compress based on system load
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Adaptive compression
            quality = self.calculate_video_quality()
            compression_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
            
            # Resize if needed for bandwidth conservation
            if quality < 70:
                height, width = cv_image.shape[:2]
                new_width = int(width * 0.75)
                new_height = int(height * 0.75)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
            
            _, buffer = cv2.imencode('.jpg', cv_image, compression_params)
            
            # Send via appropriate channels based on priority
            video_data = {
                'type': 'video_frame',
                'agent_id': agent_id,
                'data': buffer.tobytes().hex(),
                'width': cv_image.shape[1],
                'height': cv_image.shape[0],
                'quality': quality,
                'timestamp': time.time()
            }
            
            # High-priority agents get WebRTC, others get WebSocket
            if self.is_high_priority_agent(agent_id):
                await self.send_via_webrtc_data_channels(video_data)
            else:
                await self.broadcast_websocket_enhanced(video_data)
            
        except Exception as e:
            self.get_logger().error(f"Error handling enhanced video stream: {e}")
    
    def calculate_video_quality(self) -> int:
        """Calculate optimal video quality based on system load"""
        
        active_connections = len(self.websocket_clients)
        avg_latency = self.calculate_average_latency()
        
        # Base quality
        quality = 85
        
        # Reduce quality based on load
        if active_connections > 100:
            quality -= 20
        elif active_connections > 50:
            quality -= 10
        
        if avg_latency > 500:
            quality -= 15
        elif avg_latency > 200:
            quality -= 10
        
        return max(30, min(95, quality))
    
    def is_high_priority_agent(self, agent_id: str) -> bool:
        """Determine if agent is high priority for video streaming"""
        
        # Simple priority logic - in production, this would be more sophisticated
        high_priority_prefixes = ['lead_', 'commander_', 'critical_']
        return any(agent_id.startswith(prefix) for prefix in high_priority_prefixes)
    
    def calculate_average_latency(self) -> float:
        """Calculate average latency across all connections"""
        
        all_latencies = []
        
        for metrics in self.connection_metrics.values():
            if metrics.latency_samples:
                all_latencies.extend(metrics.latency_samples)
        
        return statistics.mean(all_latencies) if all_latencies else 0.0
    
    def calculate_communication_quality(self) -> float:
        """Calculate overall communication quality score"""
        
        if not self.connection_metrics:
            return 100.0
        
        # Calculate based on error rates, latency, and connection stability
        total_errors = sum(m.error_count for m in self.connection_metrics.values())
        total_messages = sum(m.messages_received for m in self.connection_metrics.values())
        
        error_rate = (total_errors / max(1, total_messages)) * 100
        avg_latency = self.calculate_average_latency()
        
        # Quality score (0-100)
        quality = 100.0
        quality -= min(50, error_rate * 10)  # Penalize errors
        quality -= min(30, avg_latency / 20)  # Penalize high latency
        
        return max(0.0, quality)
    
    async def handle_pointcloud_enhanced(self, msg: PointCloud2):
        """Enhanced point cloud handler with spatial optimization"""
        
        try:
            agent_id = msg.header.frame_id
            
            # Spatial downsampling for bandwidth efficiency
            downsampled_data = self.downsample_pointcloud(msg)
            
            pointcloud_data = {
                'type': 'pointcloud',
                'agent_id': agent_id,
                'width': downsampled_data.get('width', msg.width),
                'height': downsampled_data.get('height', msg.height),
                'point_count': downsampled_data.get('point_count', 0),
                'compression_ratio': downsampled_data.get('compression_ratio', 1.0),
                'timestamp': time.time()
            }
            
            # Send via WebSocket (point clouds are typically not as time-critical)
            await self.broadcast_websocket_enhanced(pointcloud_data)
            
        except Exception as e:
            self.get_logger().error(f"Error handling enhanced point cloud: {e}")
    
    def downsample_pointcloud(self, msg: PointCloud2) -> Dict[str, Any]:
        """Downsample point cloud for bandwidth efficiency"""
        
        # Simple downsampling - in production, use proper point cloud libraries
        original_points = msg.width * msg.height
        
        # Adaptive downsampling based on load
        active_connections = len(self.websocket_clients)
        
        if active_connections > 50:
            compression_ratio = 0.25  # Keep 25% of points
        elif active_connections > 20:
            compression_ratio = 0.5   # Keep 50% of points
        else:
            compression_ratio = 1.0   # Keep all points
        
        downsampled_points = int(original_points * compression_ratio)
        
        return {
            'width': int(msg.width * compression_ratio),
            'height': int(msg.height * compression_ratio),
            'point_count': downsampled_points,
            'compression_ratio': compression_ratio
        }
    
    def perform_health_check(self):
        """Perform comprehensive health check"""
        
        current_time = time.time()
        
        # Update global metrics
        self.global_metrics['uptime'] = current_time - self.global_metrics['start_time']
        self.global_metrics['active_connections'] = len(self.websocket_clients)
        self.global_metrics['average_latency'] = self.calculate_average_latency()
        
        # Calculate error rate
        if self.connection_metrics:
            total_errors = sum(m.error_count for m in self.connection_metrics.values())
            total_messages = sum(m.messages_received for m in self.connection_metrics.values())
            self.global_metrics['error_rate'] = (total_errors / max(1, total_messages)) * 100
        
        # Health report
        health_report = {
            'timestamp': current_time,
            'status': 'healthy' if self.global_metrics['error_rate'] < 5.0 else 'degraded',
            'metrics': dict(self.global_metrics),
            'connection_count': len(self.connection_metrics),
            'webrtc_connections': len(self.webrtc_connections),
            'data_channels': len(self.data_channels),
            'resilience_report': self.resilience_engine.get_system_resilience_report()
        }
        
        # Publish health report
        msg = String()
        msg.data = json.dumps(health_report)
        self.health_pub.publish(msg)
        
        self.get_logger().info(f"Health check completed - Status: {health_report['status']}")
    
    def cleanup_stale_connections(self):
        """Clean up stale connections and expired tokens"""
        
        current_time = time.time()
        
        # Clean up stale connection metrics
        stale_connections = []
        for client_id, metrics in self.connection_metrics.items():
            if current_time - metrics.last_activity > 300:  # 5 minutes
                stale_connections.append(client_id)
        
        for client_id in stale_connections:
            del self.connection_metrics[client_id]
            self.get_logger().info(f"Cleaned up stale connection: {client_id}")
        
        # Clean up expired tokens
        expired_tokens = []
        for token, security_token in self.security_tokens.items():
            if current_time > security_token.expires_at:
                expired_tokens.append(token)
        
        for token in expired_tokens:
            del self.security_tokens[token]
        
        # Clean up old failed auth attempts
        for ip in list(self.failed_auth_attempts.keys()):
            self.failed_auth_attempts[ip] = [
                t for t in self.failed_auth_attempts[ip] 
                if current_time - t < 3600  # Keep for 1 hour
            ]
            
            if not self.failed_auth_attempts[ip]:
                del self.failed_auth_attempts[ip]
        
        self.get_logger().info("Cleanup completed")
    
    def publish_enhanced_stats(self):
        """Publish enhanced statistics"""
        
        # Compile comprehensive statistics
        stats = {
            'timestamp': time.time(),
            'global_metrics': dict(self.global_metrics),
            'connection_metrics': {
                client_id: asdict(metrics) 
                for client_id, metrics in self.connection_metrics.items()
            },
            'performance_metrics': {
                'average_processing_time': statistics.mean(self.performance_metrics['message_processing_times']) 
                    if self.performance_metrics['message_processing_times'] else 0,
                'websocket_latency_avg': statistics.mean(self.performance_metrics['websocket_latencies'])
                    if self.performance_metrics['websocket_latencies'] else 0,
                'cpu_usage_avg': statistics.mean(self.performance_metrics['cpu_usage'])
                    if self.performance_metrics['cpu_usage'] else 0,
                'memory_usage_avg': statistics.mean(self.performance_metrics['memory_usage'])
                    if self.performance_metrics['memory_usage'] else 0
            },
            'security_metrics': {
                'active_tokens': len(self.security_tokens),
                'blocked_ips': len(self.blocked_ips),
                'failed_auth_attempts': sum(len(attempts) for attempts in self.failed_auth_attempts.values())
            }
        }
        
        # Publish stats
        msg = String()
        msg.data = json.dumps(stats)
        self.stats_pub.publish(msg)
    
    async def send_error(self, websocket: WebSocketServerProtocol, error_message: str):
        """Send error message to client"""
        
        error_response = {
            'type': 'error',
            'message': error_message,
            'timestamp': time.time()
        }
        
        try:
            await websocket.send(json.dumps(error_response))
        except:
            pass  # Connection might be closed
    
    async def start_server_enhanced(self):
        """Start the enhanced WebSocket server"""
        
        self.get_logger().info(f"Starting Enhanced WebRTC bridge server on {self.server_host}:{self.server_port}")
        
        # SSL context for production security
        ssl_context = None
        # In production, enable SSL:
        # ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        # ssl_context.load_cert_chain("cert.pem", "key.pem")
        
        try:
            # Start server with enhanced configuration
            server = await websockets.serve(
                self.handle_websocket_client_enhanced,
                self.server_host,
                self.server_port,
                ssl=ssl_context,
                max_size=self.max_message_size,
                max_queue=100,
                ping_interval=self.heartbeat_interval,
                ping_timeout=10,
                close_timeout=10
            )
            
            self.get_logger().info("Enhanced WebRTC bridge server started successfully")
            return server
            
        except Exception as e:
            self.get_logger().error(f"Failed to start enhanced server: {e}")
            raise
    
    def run_async_server(self):
        """Run the enhanced async server"""
        
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        
        try:
            server = self.event_loop.run_until_complete(self.start_server_enhanced())
            self.get_logger().info("Enhanced WebRTC bridge running")
            self.event_loop.run_forever()
        except KeyboardInterrupt:
            self.get_logger().info("Server shutdown requested")
        except Exception as e:
            self.get_logger().error(f"Server error: {e}")
        finally:
            self.event_loop.close()


def main(args=None):
    """Main entry point for enhanced WebRTC bridge"""
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Initialize ROS
    rclpy.init(args=args)
    
    try:
        # Create enhanced bridge
        bridge = EnhancedWebRTCBridge()
        
        # Start server in separate thread
        server_thread = Thread(target=bridge.run_async_server, daemon=True)
        server_thread.start()
        
        # Graceful shutdown handling
        def signal_handler(signum, frame):
            bridge.get_logger().info("Shutting down Enhanced WebRTC bridge...")
            bridge.destroy_node()
            rclpy.shutdown()
            exit(0)
        
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)
        
        # Spin ROS node
        bridge.get_logger().info("Enhanced WebRTC bridge is ready")
        rclpy.spin(bridge)
        
    except Exception as e:
        logging.error(f"Fatal error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()