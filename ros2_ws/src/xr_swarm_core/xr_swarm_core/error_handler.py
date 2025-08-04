#!/usr/bin/env python3

import json
import time
import traceback
from typing import Dict, Any, Optional, List, Callable
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ErrorSeverity(Enum):
    """Error severity levels"""
    DEBUG = "debug"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


class SwarmError:
    """Structured error representation"""
    
    def __init__(
        self,
        code: str,
        message: str,
        severity: ErrorSeverity = ErrorSeverity.ERROR,
        component: str = "unknown",
        agent_id: Optional[str] = None,
        context: Optional[Dict[str, Any]] = None,
        timestamp: Optional[float] = None
    ):
        self.code = code
        self.message = message
        self.severity = severity
        self.component = component
        self.agent_id = agent_id
        self.context = context or {}
        self.timestamp = timestamp or time.time()
        self.traceback = traceback.format_exc() if traceback.format_exc().strip() != "NoneType: None" else None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert error to dictionary for serialization"""
        return {
            "code": self.code,
            "message": self.message,
            "severity": self.severity.value,
            "component": self.component,
            "agent_id": self.agent_id,
            "context": self.context,
            "timestamp": self.timestamp,
            "traceback": self.traceback
        }
    
    def to_json(self) -> str:
        """Convert error to JSON string"""
        return json.dumps(self.to_dict(), default=str)


class ErrorHandler:
    """Centralized error handling system"""
    
    def __init__(self, node: Node, component_name: str):
        self.node = node
        self.component_name = component_name
        
        # Error statistics
        self.error_counts = {}
        self.last_errors = []
        self.max_error_history = 100
        
        # Error publishers
        self.error_pub = node.create_publisher(
            String, '/swarm/errors', 10
        )
        
        # Recovery handlers
        self.recovery_handlers: Dict[str, Callable[[SwarmError], bool]] = {}
        
        # Circuit breaker state
        self.circuit_breakers = {}
    
    def handle_error(
        self,
        code: str,
        message: str,
        severity: ErrorSeverity = ErrorSeverity.ERROR,
        agent_id: Optional[str] = None,
        context: Optional[Dict[str, Any]] = None,
        attempt_recovery: bool = True
    ) -> SwarmError:
        """Handle an error with optional recovery"""
        
        error = SwarmError(
            code=code,
            message=message,
            severity=severity,
            component=self.component_name,
            agent_id=agent_id,
            context=context
        )
        
        # Update statistics
        self.error_counts[code] = self.error_counts.get(code, 0) + 1
        self.last_errors.append(error)
        if len(self.last_errors) > self.max_error_history:
            self.last_errors.pop(0)
        
        # Log error
        self._log_error(error)
        
        # Publish error
        self._publish_error(error)
        
        # Attempt recovery if enabled
        if attempt_recovery and code in self.recovery_handlers:
            try:
                recovery_success = self.recovery_handlers[code](error)
                if recovery_success:
                    self.node.get_logger().info(f"Successfully recovered from error: {code}")
                else:
                    self.node.get_logger().warn(f"Recovery failed for error: {code}")
            except Exception as e:
                self.node.get_logger().error(f"Recovery handler failed: {e}")
        
        # Check circuit breaker
        self._check_circuit_breaker(code, error)
        
        return error
    
    def register_recovery_handler(self, error_code: str, handler: Callable[[SwarmError], bool]):
        """Register a recovery handler for specific error codes"""
        self.recovery_handlers[error_code] = handler
    
    def _log_error(self, error: SwarmError):
        """Log error with appropriate severity"""
        logger = self.node.get_logger()
        
        message = f"[{error.code}] {error.message}"
        if error.agent_id:
            message = f"[{error.agent_id}] {message}"
        
        if error.severity == ErrorSeverity.DEBUG:
            logger.debug(message)
        elif error.severity == ErrorSeverity.INFO:
            logger.info(message)
        elif error.severity == ErrorSeverity.WARNING:
            logger.warn(message)
        elif error.severity == ErrorSeverity.ERROR:
            logger.error(message)
        elif error.severity == ErrorSeverity.CRITICAL:
            logger.fatal(message)
    
    def _publish_error(self, error: SwarmError):
        """Publish error to ROS topic"""
        try:
            msg = String()
            msg.data = error.to_json()
            self.error_pub.publish(msg)
        except Exception as e:
            # Avoid infinite recursion if error publishing fails
            self.node.get_logger().error(f"Failed to publish error: {e}")
    
    def _check_circuit_breaker(self, error_code: str, error: SwarmError):
        """Check and update circuit breaker state"""
        if error_code not in self.circuit_breakers:
            self.circuit_breakers[error_code] = {
                'failure_count': 0,
                'last_failure_time': 0,
                'state': 'closed',  # closed, open, half-open
                'threshold': 5,
                'timeout': 60  # seconds
            }
        
        breaker = self.circuit_breakers[error_code]
        
        if error.severity in [ErrorSeverity.ERROR, ErrorSeverity.CRITICAL]:
            breaker['failure_count'] += 1
            breaker['last_failure_time'] = time.time()
            
            # Trip breaker if threshold exceeded
            if breaker['failure_count'] >= breaker['threshold'] and breaker['state'] == 'closed':
                breaker['state'] = 'open'
                self.node.get_logger().warn(f"Circuit breaker OPEN for error: {error_code}")
        
        # Check if breaker should transition to half-open
        if breaker['state'] == 'open':
            if time.time() - breaker['last_failure_time'] > breaker['timeout']:
                breaker['state'] = 'half-open'
                self.node.get_logger().info(f"Circuit breaker HALF-OPEN for error: {error_code}")
    
    def is_circuit_open(self, error_code: str) -> bool:
        """Check if circuit breaker is open for given error code"""
        if error_code not in self.circuit_breakers:
            return False
        return self.circuit_breakers[error_code]['state'] == 'open'
    
    def get_error_statistics(self) -> Dict[str, Any]:
        """Get error statistics"""
        return {
            'error_counts': self.error_counts,
            'total_errors': sum(self.error_counts.values()),
            'recent_errors': [error.to_dict() for error in self.last_errors[-10:]],
            'circuit_breakers': self.circuit_breakers,
            'component': self.component_name
        }
    
    def clear_error_history(self):
        """Clear error history and reset counters"""
        self.error_counts.clear()
        self.last_errors.clear()
        for breaker in self.circuit_breakers.values():
            breaker['failure_count'] = 0
            breaker['state'] = 'closed'


def with_error_handling(error_handler: ErrorHandler, error_code: str, default_return=None):
    """Decorator for automatic error handling"""
    def decorator(func):
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                error_handler.handle_error(
                    code=error_code,
                    message=str(e),
                    severity=ErrorSeverity.ERROR,
                    context={
                        'function': func.__name__,
                        'args': str(args)[:200],  # Limit context size
                        'kwargs': str(kwargs)[:200]
                    }
                )
                return default_return
        return wrapper
    return decorator


# Common error codes
class CommonErrors:
    """Common error codes used across the system"""
    
    # Connection errors
    CONNECTION_LOST = "CONN_001"
    CONNECTION_TIMEOUT = "CONN_002"
    WEBSOCKET_ERROR = "CONN_003"
    WEBRTC_ERROR = "CONN_004"
    
    # Agent errors
    AGENT_TIMEOUT = "AGENT_001"
    AGENT_CRASH = "AGENT_002"
    AGENT_COMMAND_FAILED = "AGENT_003"
    AGENT_LOW_BATTERY = "AGENT_004"
    
    # System errors
    SYSTEM_OVERLOAD = "SYS_001"
    MEMORY_ERROR = "SYS_002"
    DISK_FULL = "SYS_003"
    
    # Mission errors
    MISSION_FAILED = "MISSION_001"
    MISSION_TIMEOUT = "MISSION_002"
    INVALID_MISSION = "MISSION_003"
    
    # Safety errors
    COLLISION_RISK = "SAFETY_001"
    BOUNDARY_VIOLATION = "SAFETY_002"
    EMERGENCY_STOP = "SAFETY_003"


# Recovery handlers
class CommonRecoveryHandlers:
    """Common recovery handlers"""
    
    @staticmethod
    def retry_connection(error: SwarmError) -> bool:
        """Attempt to reconnect"""
        # Implementation would depend on specific connection type
        return False
    
    @staticmethod
    def restart_agent(error: SwarmError) -> bool:
        """Attempt to restart failed agent"""
        # Implementation would restart specific agent
        return False
    
    @staticmethod
    def emergency_stop_all(error: SwarmError) -> bool:
        """Emergency stop all agents on critical error"""
        # Implementation would send emergency stop to all agents
        return True


# Example usage in a SwarmAgent
class ErrorAwareSwarmAgent(Node):
    """Example of SwarmAgent with integrated error handling"""
    
    def __init__(self, agent_id: str):
        super().__init__(f"agent_{agent_id}")
        
        self.agent_id = agent_id
        self.error_handler = ErrorHandler(self, f"agent_{agent_id}")
        
        # Register recovery handlers
        self.error_handler.register_recovery_handler(
            CommonErrors.CONNECTION_LOST,
            self.recover_connection
        )
        
        self.error_handler.register_recovery_handler(
            CommonErrors.AGENT_LOW_BATTERY,
            self.handle_low_battery
        )
    
    @with_error_handling(None, CommonErrors.AGENT_COMMAND_FAILED)  # Will be set in __init__
    def execute_command(self, command: str):
        """Execute command with error handling"""
        # This decorator will catch exceptions and handle them
        if command == "fail":
            raise Exception("Simulated command failure")
        
        self.get_logger().info(f"Executing command: {command}")
    
    def recover_connection(self, error: SwarmError) -> bool:
        """Recover from connection loss"""
        self.get_logger().info("Attempting connection recovery...")
        # Implementation would attempt reconnection
        return True
    
    def handle_low_battery(self, error: SwarmError) -> bool:
        """Handle low battery situation"""
        self.get_logger().warn("Low battery detected, returning to base...")
        # Implementation would initiate return-to-base procedure
        return True
    
    def __post_init__(self):
        """Complete initialization after error_handler is created"""
        # Update decorator with actual error handler
        self.execute_command = with_error_handling(
            self.error_handler, CommonErrors.AGENT_COMMAND_FAILED
        )(self.execute_command.__func__)


def main():
    """Example usage"""
    rclpy.init()
    
    # Create error-aware agent
    agent = ErrorAwareSwarmAgent("test_01")
    
    try:
        # Simulate some operations with errors
        agent.execute_command("normal_command")
        agent.execute_command("fail")  # This will trigger error handling
        
        # Generate manual error
        agent.error_handler.handle_error(
            CommonErrors.AGENT_LOW_BATTERY,
            "Battery level below 20%",
            severity=ErrorSeverity.WARNING,
            context={"battery_level": 15}
        )
        
        # Print error statistics
        stats = agent.error_handler.get_error_statistics()
        print("Error Statistics:", json.dumps(stats, indent=2, default=str))
        
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()