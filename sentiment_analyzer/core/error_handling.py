"""
Comprehensive error handling and recovery mechanisms
"""

import traceback
import logging
import time
import functools
from typing import Optional, Callable, Any, Dict, List, Type, Union
from dataclasses import dataclass
from enum import Enum
import threading
from contextlib import contextmanager

logger = logging.getLogger(__name__)


class ErrorSeverity(Enum):
    """Error severity levels"""
    LOW = "low"
    MEDIUM = "medium" 
    HIGH = "high"
    CRITICAL = "critical"


class ErrorCategory(Enum):
    """Error categories for classification"""
    INPUT_ERROR = "input_error"
    MODEL_ERROR = "model_error" 
    PROCESSING_ERROR = "processing_error"
    SYSTEM_ERROR = "system_error"
    NETWORK_ERROR = "network_error"
    CONFIGURATION_ERROR = "configuration_error"
    RESOURCE_ERROR = "resource_error"


@dataclass
class ErrorDetails:
    """Detailed error information"""
    error_id: str
    timestamp: float
    category: ErrorCategory
    severity: ErrorSeverity
    message: str
    exception_type: str
    stack_trace: Optional[str] = None
    context: Optional[Dict[str, Any]] = None
    recovery_attempted: bool = False
    recovery_successful: Optional[bool] = None


class SentimentAnalysisError(Exception):
    """Base exception for sentiment analysis errors"""
    
    def __init__(self, message: str, category: ErrorCategory = ErrorCategory.PROCESSING_ERROR, 
                 severity: ErrorSeverity = ErrorSeverity.MEDIUM, context: Optional[Dict] = None):
        super().__init__(message)
        self.category = category
        self.severity = severity
        self.context = context or {}
        self.timestamp = time.time()


class InputValidationError(SentimentAnalysisError):
    """Error for invalid input data"""
    
    def __init__(self, message: str, context: Optional[Dict] = None):
        super().__init__(message, ErrorCategory.INPUT_ERROR, ErrorSeverity.MEDIUM, context)


class ModelLoadError(SentimentAnalysisError):
    """Error for model loading failures"""
    
    def __init__(self, message: str, model_name: str, context: Optional[Dict] = None):
        context = context or {}
        context['model_name'] = model_name
        super().__init__(message, ErrorCategory.MODEL_ERROR, ErrorSeverity.HIGH, context)


class ProcessingTimeoutError(SentimentAnalysisError):
    """Error for processing timeout"""
    
    def __init__(self, message: str, timeout_seconds: float, context: Optional[Dict] = None):
        context = context or {}
        context['timeout_seconds'] = timeout_seconds
        super().__init__(message, ErrorCategory.PROCESSING_ERROR, ErrorSeverity.HIGH, context)


class ResourceExhaustionError(SentimentAnalysisError):
    """Error for resource exhaustion (memory, disk, etc.)"""
    
    def __init__(self, message: str, resource_type: str, context: Optional[Dict] = None):
        context = context or {}
        context['resource_type'] = resource_type
        super().__init__(message, ErrorCategory.RESOURCE_ERROR, ErrorSeverity.HIGH, context)


class ErrorHandler:
    """Comprehensive error handling and recovery system"""
    
    def __init__(self, max_retries: int = 3, base_delay: float = 1.0, max_delay: float = 60.0):
        self.max_retries = max_retries
        self.base_delay = base_delay
        self.max_delay = max_delay
        self.error_history: List[ErrorDetails] = []
        self.error_counters: Dict[str, int] = {}
        self.circuit_breakers: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.RLock()
        
        logger.info("ErrorHandler initialized with max_retries={}, base_delay={}".format(
            max_retries, base_delay))
    
    def handle_error(self, error: Exception, context: Optional[Dict[str, Any]] = None,
                    operation_name: str = "unknown") -> ErrorDetails:
        """
        Handle and log an error with detailed information
        
        Args:
            error: The exception that occurred
            context: Additional context information
            operation_name: Name of the operation that failed
            
        Returns:
            ErrorDetails object with error information
        """
        error_id = f"{operation_name}_{int(time.time() * 1000)}"
        timestamp = time.time()
        
        # Classify error
        category, severity = self._classify_error(error)
        
        # Create error details
        error_details = ErrorDetails(
            error_id=error_id,
            timestamp=timestamp,
            category=category,
            severity=severity,
            message=str(error),
            exception_type=type(error).__name__,
            stack_trace=traceback.format_exc(),
            context=context
        )
        
        # Log error
        self._log_error(error_details)
        
        # Store error history
        with self._lock:
            self.error_history.append(error_details)
            self.error_counters[category.value] = self.error_counters.get(category.value, 0) + 1
        
        # Update circuit breaker if applicable
        self._update_circuit_breaker(operation_name, error_details)
        
        return error_details
    
    def _classify_error(self, error: Exception) -> tuple[ErrorCategory, ErrorSeverity]:
        """Classify error by category and severity"""
        
        if isinstance(error, SentimentAnalysisError):
            return error.category, error.severity
        
        # Built-in exception classification
        error_classifications = {
            ValueError: (ErrorCategory.INPUT_ERROR, ErrorSeverity.MEDIUM),
            TypeError: (ErrorCategory.INPUT_ERROR, ErrorSeverity.MEDIUM),
            AttributeError: (ErrorCategory.PROCESSING_ERROR, ErrorSeverity.MEDIUM),
            KeyError: (ErrorCategory.PROCESSING_ERROR, ErrorSeverity.MEDIUM),
            IndexError: (ErrorCategory.PROCESSING_ERROR, ErrorSeverity.MEDIUM),
            FileNotFoundError: (ErrorCategory.SYSTEM_ERROR, ErrorSeverity.HIGH),
            PermissionError: (ErrorCategory.SYSTEM_ERROR, ErrorSeverity.HIGH),
            MemoryError: (ErrorCategory.RESOURCE_ERROR, ErrorSeverity.CRITICAL),
            ConnectionError: (ErrorCategory.NETWORK_ERROR, ErrorSeverity.HIGH),
            TimeoutError: (ErrorCategory.PROCESSING_ERROR, ErrorSeverity.HIGH),
            ImportError: (ErrorCategory.SYSTEM_ERROR, ErrorSeverity.CRITICAL),
            ModuleNotFoundError: (ErrorCategory.SYSTEM_ERROR, ErrorSeverity.CRITICAL)
        }
        
        error_type = type(error)
        if error_type in error_classifications:
            return error_classifications[error_type]
        
        # Check error message for clues
        error_message = str(error).lower()
        if any(keyword in error_message for keyword in ['timeout', 'timed out']):
            return ErrorCategory.PROCESSING_ERROR, ErrorSeverity.HIGH
        elif any(keyword in error_message for keyword in ['memory', 'out of memory']):
            return ErrorCategory.RESOURCE_ERROR, ErrorSeverity.CRITICAL
        elif any(keyword in error_message for keyword in ['connection', 'network']):
            return ErrorCategory.NETWORK_ERROR, ErrorSeverity.HIGH
        elif any(keyword in error_message for keyword in ['permission', 'access denied']):
            return ErrorCategory.SYSTEM_ERROR, ErrorSeverity.HIGH
        
        # Default classification
        return ErrorCategory.PROCESSING_ERROR, ErrorSeverity.MEDIUM
    
    def _log_error(self, error_details: ErrorDetails):
        """Log error with appropriate level based on severity"""
        log_message = f"[{error_details.error_id}] {error_details.category.value}: {error_details.message}"
        
        if error_details.context:
            log_message += f" | Context: {error_details.context}"
        
        if error_details.severity == ErrorSeverity.CRITICAL:
            logger.critical(log_message)
            if error_details.stack_trace:
                logger.critical(f"Stack trace:\n{error_details.stack_trace}")
        elif error_details.severity == ErrorSeverity.HIGH:
            logger.error(log_message)
            if error_details.stack_trace:
                logger.error(f"Stack trace:\n{error_details.stack_trace}")
        elif error_details.severity == ErrorSeverity.MEDIUM:
            logger.warning(log_message)
        else:  # LOW
            logger.info(log_message)
    
    def retry_with_backoff(self, operation: Callable, *args, operation_name: str = "operation",
                          max_retries: Optional[int] = None, **kwargs) -> Any:
        """
        Execute operation with exponential backoff retry
        
        Args:
            operation: Function to execute
            args: Arguments for the operation
            operation_name: Name for logging purposes
            max_retries: Override default max retries
            kwargs: Keyword arguments for the operation
            
        Returns:
            Result of the operation
            
        Raises:
            Last exception if all retries fail
        """
        max_retries = max_retries or self.max_retries
        last_exception = None
        
        for attempt in range(max_retries + 1):
            try:
                return operation(*args, **kwargs)
            
            except Exception as e:
                last_exception = e
                
                # Handle the error
                error_details = self.handle_error(e, 
                    context={'attempt': attempt + 1, 'max_retries': max_retries + 1},
                    operation_name=operation_name)
                
                # Don't retry on critical errors or input validation errors
                if error_details.severity == ErrorSeverity.CRITICAL or \
                   error_details.category == ErrorCategory.INPUT_ERROR:
                    logger.error(f"Not retrying {operation_name} due to {error_details.severity.value} error")
                    break
                
                # Don't retry if this is the last attempt
                if attempt == max_retries:
                    break
                
                # Calculate backoff delay
                delay = min(self.base_delay * (2 ** attempt), self.max_delay)
                logger.info(f"Retrying {operation_name} in {delay:.2f} seconds (attempt {attempt + 1}/{max_retries + 1})")
                time.sleep(delay)
        
        # All retries failed
        logger.error(f"All retry attempts failed for {operation_name}")
        raise last_exception
    
    def _update_circuit_breaker(self, operation_name: str, error_details: ErrorDetails):
        """Update circuit breaker state based on error"""
        with self._lock:
            if operation_name not in self.circuit_breakers:
                self.circuit_breakers[operation_name] = {
                    'failure_count': 0,
                    'last_failure_time': None,
                    'state': 'closed',  # closed, open, half_open
                    'failure_threshold': 5,
                    'recovery_timeout': 60.0
                }
            
            breaker = self.circuit_breakers[operation_name]
            
            # Increment failure count
            breaker['failure_count'] += 1
            breaker['last_failure_time'] = error_details.timestamp
            
            # Open circuit breaker if threshold exceeded
            if (breaker['failure_count'] >= breaker['failure_threshold'] and 
                breaker['state'] == 'closed'):
                breaker['state'] = 'open'
                logger.warning(f"Circuit breaker opened for {operation_name}")
    
    def is_circuit_breaker_open(self, operation_name: str) -> bool:
        """Check if circuit breaker is open for an operation"""
        with self._lock:
            if operation_name not in self.circuit_breakers:
                return False
            
            breaker = self.circuit_breakers[operation_name]
            
            if breaker['state'] == 'closed':
                return False
            
            if breaker['state'] == 'open':
                # Check if recovery timeout has elapsed
                if (time.time() - breaker['last_failure_time'] > breaker['recovery_timeout']):
                    breaker['state'] = 'half_open'
                    logger.info(f"Circuit breaker half-opened for {operation_name}")
                    return False
                return True
            
            # half_open state
            return False
    
    def reset_circuit_breaker(self, operation_name: str):
        """Reset circuit breaker after successful operation"""
        with self._lock:
            if operation_name in self.circuit_breakers:
                breaker = self.circuit_breakers[operation_name]
                breaker['failure_count'] = 0
                breaker['state'] = 'closed'
                logger.info(f"Circuit breaker reset for {operation_name}")
    
    @contextmanager
    def error_context(self, operation_name: str, context: Optional[Dict] = None):
        """Context manager for error handling"""
        try:
            yield
            # Reset circuit breaker on success
            self.reset_circuit_breaker(operation_name)
            
        except Exception as e:
            # Handle the error
            self.handle_error(e, context, operation_name)
            raise
    
    def get_error_statistics(self) -> Dict[str, Any]:
        """Get error statistics and health metrics"""
        with self._lock:
            total_errors = len(self.error_history)
            
            if total_errors == 0:
                return {
                    'total_errors': 0,
                    'error_rate': 0.0,
                    'categories': {},
                    'severities': {},
                    'circuit_breakers': {}
                }
            
            # Calculate recent error rate (last hour)
            one_hour_ago = time.time() - 3600
            recent_errors = [e for e in self.error_history if e.timestamp > one_hour_ago]
            
            # Count by category
            category_counts = {}
            severity_counts = {}
            
            for error in self.error_history:
                category_counts[error.category.value] = category_counts.get(error.category.value, 0) + 1
                severity_counts[error.severity.value] = severity_counts.get(error.severity.value, 0) + 1
            
            # Circuit breaker status
            breaker_status = {}
            for name, breaker in self.circuit_breakers.items():
                breaker_status[name] = {
                    'state': breaker['state'],
                    'failure_count': breaker['failure_count'],
                    'last_failure': breaker['last_failure_time']
                }
            
            return {
                'total_errors': total_errors,
                'recent_errors_1h': len(recent_errors),
                'error_rate_1h': len(recent_errors) / 3600,  # errors per second
                'categories': category_counts,
                'severities': severity_counts,
                'circuit_breakers': breaker_status
            }
    
    def clear_error_history(self, older_than_hours: Optional[float] = None):
        """Clear error history, optionally only errors older than specified hours"""
        with self._lock:
            if older_than_hours is None:
                # Clear all history
                self.error_history.clear()
                self.error_counters.clear()
                logger.info("All error history cleared")
            else:
                # Clear only old errors
                cutoff_time = time.time() - (older_than_hours * 3600)
                original_count = len(self.error_history)
                self.error_history = [e for e in self.error_history if e.timestamp > cutoff_time]
                cleared_count = original_count - len(self.error_history)
                logger.info(f"Cleared {cleared_count} error records older than {older_than_hours} hours")


def with_error_handling(operation_name: str = None, max_retries: int = 3, 
                       context: Optional[Dict] = None):
    """Decorator for automatic error handling and retry"""
    
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs):
            # Get or create error handler
            if hasattr(wrapper, '_error_handler'):
                error_handler = wrapper._error_handler
            else:
                error_handler = ErrorHandler(max_retries=max_retries)
                wrapper._error_handler = error_handler
            
            op_name = operation_name or func.__name__
            
            # Check circuit breaker
            if error_handler.is_circuit_breaker_open(op_name):
                raise SentimentAnalysisError(
                    f"Circuit breaker is open for operation: {op_name}",
                    category=ErrorCategory.SYSTEM_ERROR,
                    severity=ErrorSeverity.HIGH,
                    context={'circuit_breaker_open': True}
                )
            
            # Execute with retry
            return error_handler.retry_with_backoff(
                func, *args, operation_name=op_name, **kwargs
            )
        
        return wrapper
    return decorator