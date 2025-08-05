#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import json
import time
import threading
import psutil
import logging
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, asdict
from enum import Enum
import subprocess
import os
import signal

class HealthStatus(Enum):
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"
    CRITICAL = "critical"

class ComponentType(Enum):
    AGENT = "agent"
    COORDINATOR = "coordinator"
    WEBRTC_BRIDGE = "webrtc_bridge"
    SYSTEM = "system"

@dataclass
class HealthMetrics:
    cpu_usage: float
    memory_usage: float
    disk_usage: float
    network_latency: float
    message_throughput: float
    error_rate: float
    uptime: float
    last_heartbeat: float

@dataclass
class ComponentHealth:
    component_id: str
    component_type: ComponentType
    status: HealthStatus
    metrics: HealthMetrics
    last_check: float
    error_count: int
    recovery_attempts: int
    details: Dict[str, Any]

class HealthMonitor(Node):
    """Comprehensive health monitoring and auto-recovery system"""
    
    def __init__(self):
        super().__init__('health_monitor')
        
        # Component registry
        self.components: Dict[str, ComponentHealth] = {}
        self.recovery_handlers: Dict[ComponentType, Callable] = {}
        
        # Monitoring configuration
        self.check_interval = 5.0  # seconds
        self.heartbeat_timeout = 30.0  # seconds
        self.max_recovery_attempts = 3
        self.critical_cpu_threshold = 80.0  # percent
        self.critical_memory_threshold = 85.0  # percent
        self.critical_disk_threshold = 90.0  # percent
        
        # Performance tracking
        self.message_counts: Dict[str, int] = {}
        self.error_counts: Dict[str, int] = {}
        self.start_time = time.time()
        
        # Publishers and subscribers
        self.setup_ros_communication()
        
        # Start monitoring threads
        self.setup_monitoring_threads()
        
        # Register recovery handlers
        self.register_recovery_handlers()
        
        self.get_logger().info("HealthMonitor initialized")
    
    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers"""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Health status publisher
        self.health_pub = self.create_publisher(
            String, '/system/health/status', qos_profile
        )
        
        # Subscribe to component heartbeats
        self.heartbeat_sub = self.create_subscription(
            String, '/system/heartbeat/+',
            self.handle_heartbeat, qos_profile
        )
        
        # Subscribe to error reports
        self.error_sub = self.create_subscription(
            String, '/system/errors/+',
            self.handle_error_report, qos_profile
        )
        
        # Health check request subscriber
        self.health_request_sub = self.create_subscription(
            String, '/system/health/request',
            self.handle_health_request, qos_profile
        )
        
        # Recovery action publisher
        self.recovery_pub = self.create_publisher(
            String, '/system/recovery/action', qos_profile
        )
        
        # Metrics publisher
        self.metrics_pub = self.create_publisher(
            String, '/system/metrics', qos_profile
        )
        
        # Timer for regular health checks
        self.health_timer = self.create_timer(self.check_interval, self.perform_health_check)
        
        # Timer for metrics publishing
        self.metrics_timer = self.create_timer(10.0, self.publish_metrics)
    
    def setup_monitoring_threads(self):
        """Setup background monitoring threads"""
        # System resource monitoring
        self.system_monitor_thread = threading.Thread(
            target=self.monitor_system_resources, daemon=True
        )
        self.system_monitor_thread.start()
        
        # Network monitoring
        self.network_monitor_thread = threading.Thread(
            target=self.monitor_network_health, daemon=True
        )
        self.network_monitor_thread.start()
        
        # Process monitoring
        self.process_monitor_thread = threading.Thread(
            target=self.monitor_processes, daemon=True
        )
        self.process_monitor_thread.start()
    
    def register_recovery_handlers(self):
        """Register automatic recovery handlers for different components"""
        self.recovery_handlers[ComponentType.AGENT] = self.recover_agent
        self.recovery_handlers[ComponentType.COORDINATOR] = self.recover_coordinator
        self.recovery_handlers[ComponentType.WEBRTC_BRIDGE] = self.recover_webrtc_bridge
        self.recovery_handlers[ComponentType.SYSTEM] = self.recover_system
    
    def handle_heartbeat(self, msg: String):
        """Handle heartbeat messages from components"""
        try:
            data = json.loads(msg.data)
            component_id = data.get('component_id')
            component_type = ComponentType(data.get('component_type', 'agent'))
            
            if component_id:
                # Update or create component health record
                if component_id not in self.components:
                    self.components[component_id] = ComponentHealth(
                        component_id=component_id,
                        component_type=component_type,
                        status=HealthStatus.HEALTHY,
                        metrics=HealthMetrics(0, 0, 0, 0, 0, 0, 0, time.time()),
                        last_check=time.time(),
                        error_count=0,
                        recovery_attempts=0,
                        details={}
                    )
                
                component = self.components[component_id]
                component.last_check = time.time()
                component.metrics.last_heartbeat = time.time()
                
                # Update metrics from heartbeat data
                if 'metrics' in data:
                    self.update_component_metrics(component_id, data['metrics'])
                
                self.get_logger().debug(f"Heartbeat received from {component_id}")
                
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().error(f"Invalid heartbeat message: {e}")
    
    def handle_error_report(self, msg: String):
        """Handle error reports from components"""
        try:
            data = json.loads(msg.data)
            component_id = data.get('component_id')
            error_type = data.get('error_type', 'unknown')
            severity = data.get('severity', 'medium')
            
            if component_id and component_id in self.components:
                component = self.components[component_id]
                component.error_count += 1
                
                # Update component status based on error severity
                if severity == 'critical':
                    component.status = HealthStatus.CRITICAL
                elif severity == 'high' and component.status == HealthStatus.HEALTHY:
                    component.status = HealthStatus.DEGRADED
                
                # Store error details
                if 'error_details' not in component.details:
                    component.details['error_details'] = []
                
                component.details['error_details'].append({
                    'type': error_type,
                    'severity': severity,
                    'timestamp': time.time(),
                    'message': data.get('message', '')
                })
                
                # Keep only last 10 errors
                if len(component.details['error_details']) > 10:
                    component.details['error_details'] = component.details['error_details'][-10:]
                
                self.get_logger().warn(f"Error reported by {component_id}: {error_type} ({severity})")
                
                # Trigger immediate health check
                self.check_component_health(component_id)
                
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid error report: {e}")
    
    def handle_health_request(self, msg: String):
        """Handle health status requests"""
        try:
            data = json.loads(msg.data)
            request_type = data.get('type', 'full')
            
            if request_type == 'full':
                response = {
                    'type': 'health_status_full',
                    'timestamp': time.time(),
                    'components': {
                        comp_id: asdict(comp) for comp_id, comp in self.components.items()
                    },
                    'system_metrics': self.get_system_metrics()
                }
            else:
                response = {
                    'type': 'health_status_summary',
                    'timestamp': time.time(),
                    'component_count': len(self.components),
                    'healthy_count': len([c for c in self.components.values() if c.status == HealthStatus.HEALTHY]),
                    'degraded_count': len([c for c in self.components.values() if c.status == HealthStatus.DEGRADED]),
                    'unhealthy_count': len([c for c in self.components.values() if c.status == HealthStatus.UNHEALTHY]),
                    'critical_count': len([c for c in self.components.values() if c.status == HealthStatus.CRITICAL])
                }
            
            msg_out = String()
            msg_out.data = json.dumps(response)
            self.health_pub.publish(msg_out)
            
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"Invalid health request: {e}")
    
    def perform_health_check(self):
        """Perform regular health checks on all components"""
        current_time = time.time()
        
        for component_id, component in list(self.components.items()):
            # Check for stale heartbeats
            if current_time - component.metrics.last_heartbeat > self.heartbeat_timeout:
                component.status = HealthStatus.UNHEALTHY
                self.get_logger().warn(f"Component {component_id} heartbeat timeout")
                
                # Attempt recovery
                self.attempt_recovery(component_id)
            
            # Check component-specific health
            self.check_component_health(component_id)
        
        # Publish overall health status
        self.publish_health_status()
    
    def check_component_health(self, component_id: str):
        """Check health of a specific component"""
        if component_id not in self.components:
            return
        
        component = self.components[component_id]
        
        # Determine status based on metrics and errors
        if component.error_count > 10:  # High error rate
            component.status = HealthStatus.DEGRADED
        
        if component.metrics.cpu_usage > self.critical_cpu_threshold:
            component.status = HealthStatus.DEGRADED
            
        if component.metrics.memory_usage > self.critical_memory_threshold:
            component.status = HealthStatus.CRITICAL
        
        # Check for critical conditions
        if (component.error_count > 50 or 
            component.metrics.memory_usage > 95.0 or
            component.metrics.cpu_usage > 95.0):
            component.status = HealthStatus.CRITICAL
            self.get_logger().error(f"Component {component_id} in CRITICAL state")
            self.attempt_recovery(component_id)
    
    def attempt_recovery(self, component_id: str):
        """Attempt to recover a failed component"""
        if component_id not in self.components:
            return
        
        component = self.components[component_id]
        
        if component.recovery_attempts >= self.max_recovery_attempts:
            self.get_logger().error(f"Max recovery attempts reached for {component_id}")
            return
        
        component.recovery_attempts += 1
        self.get_logger().info(f"Attempting recovery for {component_id} (attempt {component.recovery_attempts})")
        
        # Get recovery handler
        recovery_handler = self.recovery_handlers.get(component.component_type)
        if recovery_handler:
            try:
                success = recovery_handler(component_id, component)
                
                if success:
                    component.status = HealthStatus.HEALTHY
                    component.error_count = 0
                    component.recovery_attempts = 0
                    self.get_logger().info(f"Recovery successful for {component_id}")
                    
                    # Publish recovery success
                    recovery_msg = {
                        'type': 'recovery_success',
                        'component_id': component_id,
                        'timestamp': time.time()
                    }
                    msg = String()
                    msg.data = json.dumps(recovery_msg)
                    self.recovery_pub.publish(msg)
                else:
                    self.get_logger().warn(f"Recovery failed for {component_id}")
                    
            except Exception as e:
                self.get_logger().error(f"Recovery handler error for {component_id}: {e}")
        else:
            self.get_logger().warn(f"No recovery handler for component type {component.component_type}")
    
    def recover_agent(self, component_id: str, component: ComponentHealth) -> bool:
        """Recover a failed agent"""
        try:
            # Send restart command to agent
            restart_msg = {
                'type': 'restart_agent',
                'agent_id': component_id,
                'timestamp': time.time()
            }
            
            msg = String()
            msg.data = json.dumps(restart_msg)
            self.recovery_pub.publish(msg)
            
            # Wait a moment for restart
            time.sleep(2.0)
            
            # Reset error count
            component.error_count = 0
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Agent recovery failed: {e}")
            return False
    
    def recover_coordinator(self, component_id: str, component: ComponentHealth) -> bool:
        """Recover the swarm coordinator"""
        try:
            # Try to restart coordinator service
            result = subprocess.run(['systemctl', 'restart', 'swarm-coordinator'], 
                                  capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info("Coordinator service restarted successfully")
                return True
            else:
                self.get_logger().error(f"Failed to restart coordinator: {result.stderr}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Coordinator recovery failed: {e}")
            return False
    
    def recover_webrtc_bridge(self, component_id: str, component: ComponentHealth) -> bool:
        """Recover the WebRTC bridge"""
        try:
            # Send reset command to WebRTC bridge
            reset_msg = {
                'type': 'reset_webrtc',
                'timestamp': time.time()
            }
            
            msg = String()
            msg.data = json.dumps(reset_msg)
            self.recovery_pub.publish(msg)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"WebRTC bridge recovery failed: {e}")
            return False
    
    def recover_system(self, component_id: str, component: ComponentHealth) -> bool:
        """Recover system-level issues"""
        try:
            # Free up memory if needed
            if component.metrics.memory_usage > 90.0:
                self.get_logger().info("Attempting to free system memory")
                os.system('sync && echo 3 > /proc/sys/vm/drop_caches')
            
            # Restart networking if needed
            if component.metrics.network_latency > 1000:  # 1 second
                self.get_logger().info("Restarting network services")
                os.system('systemctl restart systemd-networkd')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"System recovery failed: {e}")
            return False
    
    def monitor_system_resources(self):
        """Monitor system-level resources"""
        while True:
            try:
                # Update system component health
                system_metrics = HealthMetrics(
                    cpu_usage=psutil.cpu_percent(interval=1),
                    memory_usage=psutil.virtual_memory().percent,
                    disk_usage=psutil.disk_usage('/').percent,
                    network_latency=0,  # Will be updated by network monitor
                    message_throughput=0,  # Will be calculated separately
                    error_rate=0,  # Will be calculated separately
                    uptime=time.time() - self.start_time,
                    last_heartbeat=time.time()
                )
                
                # Update or create system component
                if 'system' not in self.components:
                    self.components['system'] = ComponentHealth(
                        component_id='system',
                        component_type=ComponentType.SYSTEM,
                        status=HealthStatus.HEALTHY,
                        metrics=system_metrics,
                        last_check=time.time(),
                        error_count=0,
                        recovery_attempts=0,
                        details={}
                    )
                else:
                    self.components['system'].metrics = system_metrics
                    self.components['system'].last_check = time.time()
                
                # Check for critical system conditions
                if (system_metrics.cpu_usage > self.critical_cpu_threshold or
                    system_metrics.memory_usage > self.critical_memory_threshold or
                    system_metrics.disk_usage > self.critical_disk_threshold):
                    
                    self.components['system'].status = HealthStatus.CRITICAL
                    self.get_logger().error("System resources in critical state")
                    self.attempt_recovery('system')
                
                time.sleep(5.0)
                
            except Exception as e:
                self.get_logger().error(f"System monitoring error: {e}")
                time.sleep(10.0)
    
    def monitor_network_health(self):
        """Monitor network connectivity and latency"""
        while True:
            try:
                # Ping test to measure network latency
                result = subprocess.run(['ping', '-c', '1', '8.8.8.8'], 
                                      capture_output=True, text=True)
                
                if result.returncode == 0:
                    # Extract latency from ping output
                    lines = result.stdout.split('\n')
                    for line in lines:
                        if 'time=' in line:
                            latency_str = line.split('time=')[1].split(' ')[0]
                            latency = float(latency_str)
                            
                            # Update system network latency
                            if 'system' in self.components:
                                self.components['system'].metrics.network_latency = latency
                            
                            break
                else:
                    # Network connectivity issue
                    if 'system' in self.components:
                        self.components['system'].metrics.network_latency = 9999
                        self.components['system'].status = HealthStatus.DEGRADED
                
                time.sleep(10.0)
                
            except Exception as e:
                self.get_logger().error(f"Network monitoring error: {e}")
                time.sleep(30.0)
    
    def monitor_processes(self):
        """Monitor ROS and system processes"""
        while True:
            try:
                # Check if critical processes are running
                critical_processes = [
                    'swarm_coordinator',
                    'webrtc_server',
                    'roscore'
                ]
                
                for proc_name in critical_processes:
                    try:
                        # Use pgrep to check if process is running
                        result = subprocess.run(['pgrep', '-f', proc_name], 
                                              capture_output=True, text=True)
                        
                        if result.returncode != 0:
                            self.get_logger().error(f"Critical process {proc_name} not running")
                            
                            # Try to restart the process
                            self.restart_process(proc_name)
                    
                    except Exception as e:
                        self.get_logger().error(f"Process check error for {proc_name}: {e}")
                
                time.sleep(15.0)
                
            except Exception as e:
                self.get_logger().error(f"Process monitoring error: {e}")
                time.sleep(30.0)
    
    def restart_process(self, process_name: str):
        """Restart a critical process"""
        try:
            if process_name == 'swarm_coordinator':
                subprocess.run(['ros2', 'run', 'xr_swarm_core', 'swarm_coordinator.py'], 
                             start_new_session=True)
            elif process_name == 'webrtc_server':
                subprocess.run(['ros2', 'run', 'webrtc_bridge', 'webrtc_server.py'], 
                             start_new_session=True)
            
            self.get_logger().info(f"Restarted process: {process_name}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to restart process {process_name}: {e}")
    
    def update_component_metrics(self, component_id: str, metrics_data: Dict[str, Any]):
        """Update component metrics from heartbeat data"""
        if component_id not in self.components:
            return
        
        component = self.components[component_id]
        
        # Update metrics if provided
        if 'cpu_usage' in metrics_data:
            component.metrics.cpu_usage = metrics_data['cpu_usage']
        if 'memory_usage' in metrics_data:
            component.metrics.memory_usage = metrics_data['memory_usage']
        if 'message_throughput' in metrics_data:
            component.metrics.message_throughput = metrics_data['message_throughput']
        if 'error_rate' in metrics_data:
            component.metrics.error_rate = metrics_data['error_rate']
    
    def get_system_metrics(self) -> Dict[str, Any]:
        """Get overall system metrics"""
        total_components = len(self.components)
        healthy_components = len([c for c in self.components.values() if c.status == HealthStatus.HEALTHY])
        
        return {
            'total_components': total_components,
            'healthy_percentage': (healthy_components / total_components * 100) if total_components > 0 else 0,
            'system_uptime': time.time() - self.start_time,
            'total_errors': sum(c.error_count for c in self.components.values()),
            'total_recoveries': sum(c.recovery_attempts for c in self.components.values())
        }
    
    def publish_health_status(self):
        """Publish overall health status"""
        status_data = {
            'type': 'health_status_broadcast',
            'timestamp': time.time(),
            'system_health': self.get_overall_system_health(),
            'component_summary': {
                'total': len(self.components),
                'healthy': len([c for c in self.components.values() if c.status == HealthStatus.HEALTHY]),
                'degraded': len([c for c in self.components.values() if c.status == HealthStatus.DEGRADED]),
                'unhealthy': len([c for c in self.components.values() if c.status == HealthStatus.UNHEALTHY]),
                'critical': len([c for c in self.components.values() if c.status == HealthStatus.CRITICAL])
            }
        }
        
        msg = String()
        msg.data = json.dumps(status_data)
        self.health_pub.publish(msg)
    
    def publish_metrics(self):
        """Publish system metrics"""
        metrics_data = {
            'type': 'system_metrics',
            'timestamp': time.time(),
            'metrics': self.get_system_metrics(),
            'components': {
                comp_id: {
                    'status': comp.status.value,
                    'metrics': asdict(comp.metrics),
                    'error_count': comp.error_count,
                    'recovery_attempts': comp.recovery_attempts
                }
                for comp_id, comp in self.components.items()
            }
        }
        
        msg = String()
        msg.data = json.dumps(metrics_data)
        self.metrics_pub.publish(msg)
    
    def get_overall_system_health(self) -> str:
        """Determine overall system health status"""
        if not self.components:
            return HealthStatus.UNHEALTHY.value
        
        critical_count = len([c for c in self.components.values() if c.status == HealthStatus.CRITICAL])
        unhealthy_count = len([c for c in self.components.values() if c.status == HealthStatus.UNHEALTHY])
        degraded_count = len([c for c in self.components.values() if c.status == HealthStatus.DEGRADED])
        
        total_components = len(self.components)
        
        if critical_count > 0:
            return HealthStatus.CRITICAL.value
        elif unhealthy_count > total_components * 0.3:  # More than 30% unhealthy
            return HealthStatus.UNHEALTHY.value
        elif degraded_count > total_components * 0.5:  # More than 50% degraded
            return HealthStatus.DEGRADED.value
        else:
            return HealthStatus.HEALTHY.value


def main(args=None):
    rclpy.init(args=args)
    
    health_monitor = HealthMonitor()
    
    try:
        rclpy.spin(health_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        health_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()