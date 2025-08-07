"""
Security measures and input sanitization for sentiment analysis
"""

import hashlib
import hmac
import secrets
import time
import re
import json
from typing import Dict, List, Optional, Any, Set, Tuple
from dataclasses import dataclass
from enum import Enum
import logging
from functools import wraps
import threading

logger = logging.getLogger(__name__)


class SecurityLevel(Enum):
    """Security enforcement levels"""
    BASIC = "basic"
    STANDARD = "standard"
    HIGH = "high"
    MAXIMUM = "maximum"


@dataclass
class SecurityViolation:
    """Security violation details"""
    violation_type: str
    severity: str
    description: str
    timestamp: float
    source_ip: Optional[str] = None
    user_id: Optional[str] = None
    details: Optional[Dict[str, Any]] = None


class RateLimiter:
    """Thread-safe rate limiter for API protection"""
    
    def __init__(self, max_requests: int, time_window: int = 60):
        self.max_requests = max_requests
        self.time_window = time_window
        self.requests: Dict[str, List[float]] = {}
        self._lock = threading.RLock()
    
    def is_allowed(self, identifier: str) -> bool:
        """Check if request is allowed under rate limit"""
        current_time = time.time()
        
        with self._lock:
            if identifier not in self.requests:
                self.requests[identifier] = []
            
            # Clean old requests
            self.requests[identifier] = [
                req_time for req_time in self.requests[identifier]
                if current_time - req_time < self.time_window
            ]
            
            # Check rate limit
            if len(self.requests[identifier]) >= self.max_requests:
                return False
            
            # Add current request
            self.requests[identifier].append(current_time)
            return True
    
    def get_remaining_requests(self, identifier: str) -> int:
        """Get remaining requests for identifier"""
        current_time = time.time()
        
        with self._lock:
            if identifier not in self.requests:
                return self.max_requests
            
            # Clean old requests
            recent_requests = [
                req_time for req_time in self.requests[identifier]
                if current_time - req_time < self.time_window
            ]
            
            return max(0, self.max_requests - len(recent_requests))


class InputSanitizer:
    """Comprehensive input sanitization for security"""
    
    def __init__(self, security_level: SecurityLevel = SecurityLevel.STANDARD):
        self.security_level = security_level
        self.blocked_patterns = self._load_security_patterns()
        self.max_input_lengths = {
            SecurityLevel.BASIC: 50000,
            SecurityLevel.STANDARD: 10000,
            SecurityLevel.HIGH: 5000,
            SecurityLevel.MAXIMUM: 1000
        }
        
        logger.info(f"InputSanitizer initialized with security level: {security_level.value}")
    
    def _load_security_patterns(self) -> Dict[str, List[str]]:
        """Load security threat patterns"""
        patterns = {
            'sql_injection': [
                r'(\bUNION\b.*\bSELECT\b)',
                r'(\bSELECT\b.*\bFROM\b.*\bWHERE\b)',
                r'(\bINSERT\b.*\bINTO\b)',
                r'(\bDELETE\b.*\bFROM\b)',
                r'(\bUPDATE\b.*\bSET\b)',
                r'(\bDROP\b.*\bTABLE\b)',
                r'(\';.*--)',
                r'(\bOR\b.*\b1\s*=\s*1\b)',
                r'(\bAND\b.*\b1\s*=\s*1\b)'
            ],
            'xss': [
                r'<script[^>]*>.*?</script>',
                r'javascript:',
                r'vbscript:',
                r'onload\s*=',
                r'onerror\s*=',
                r'onclick\s*=',
                r'onmouseover\s*=',
                r'<iframe[^>]*>',
                r'<embed[^>]*>',
                r'<object[^>]*>'
            ],
            'command_injection': [
                r';\s*(rm|del|format|shutdown)',
                r'\|\s*(curl|wget|nc|netcat)',
                r'`[^`]*`',
                r'\$\([^)]*\)',
                r'&&\s*(rm|del|format)',
                r'\|\|\s*(rm|del|format)'
            ],
            'path_traversal': [
                r'\.\./',
                r'\.\.\\\\',
                r'/etc/passwd',
                r'/proc/',
                r'\\windows\\',
                r'file://',
                r'ftp://'
            ],
            'code_injection': [
                r'eval\s*\(',
                r'exec\s*\(',
                r'system\s*\(',
                r'shell_exec\s*\(',
                r'passthru\s*\(',
                r'proc_open\s*\(',
                r'popen\s*\(',
                r'__import__\s*\('
            ]
        }
        
        # Compile regex patterns for efficiency
        compiled_patterns = {}
        for category, pattern_list in patterns.items():
            compiled_patterns[category] = [
                re.compile(pattern, re.IGNORECASE | re.DOTALL)
                for pattern in pattern_list
            ]
        
        return compiled_patterns
    
    def sanitize_input(self, text: str, context: Optional[Dict] = None) -> Tuple[str, List[SecurityViolation]]:
        """
        Sanitize input text for security threats
        
        Args:
            text: Input text to sanitize
            context: Additional context for logging
            
        Returns:
            Tuple of (sanitized_text, security_violations)
        """
        violations = []
        original_text = text
        
        # Length check
        max_length = self.max_input_lengths[self.security_level]
        if len(text) > max_length:
            violations.append(SecurityViolation(
                violation_type="length_violation",
                severity="medium",
                description=f"Input length {len(text)} exceeds maximum {max_length}",
                timestamp=time.time(),
                details={'original_length': len(text), 'max_allowed': max_length}
            ))
            text = text[:max_length]
        
        # Check for security threats
        for threat_type, patterns in self.blocked_patterns.items():
            for pattern in patterns:
                matches = pattern.findall(text)
                if matches:
                    violations.append(SecurityViolation(
                        violation_type=threat_type,
                        severity="high",
                        description=f"Potential {threat_type} detected: {matches[:3]}",  # Limit logged matches
                        timestamp=time.time(),
                        details={'pattern': pattern.pattern, 'matches': len(matches)}
                    ))
                    
                    # Remove or neutralize threats based on security level
                    if self.security_level in [SecurityLevel.HIGH, SecurityLevel.MAXIMUM]:
                        text = pattern.sub('[BLOCKED]', text)
                    elif self.security_level == SecurityLevel.STANDARD:
                        text = pattern.sub('', text)
                    # BASIC level: just log, don't modify
        
        # Additional sanitization based on security level
        if self.security_level in [SecurityLevel.HIGH, SecurityLevel.MAXIMUM]:
            # Remove all HTML tags
            text = re.sub(r'<[^>]+>', '', text)
            
            # Remove potentially dangerous characters
            dangerous_chars = ['<', '>', '{', '}', '[', ']', '`', '$', '|', '&', ';']
            for char in dangerous_chars:
                if char in text and self.security_level == SecurityLevel.MAXIMUM:
                    text = text.replace(char, '')
        
        # Normalize whitespace
        text = ' '.join(text.split())
        
        # Log violations
        if violations:
            logger.warning(f"Security violations detected in input: {len(violations)} violations")
            for violation in violations:
                logger.warning(f"Violation: {violation.violation_type} - {violation.description}")
        
        return text, violations
    
    def check_suspicious_patterns(self, text: str) -> List[str]:
        """Check for suspicious patterns that might indicate malicious intent"""
        suspicious_indicators = []
        
        # Check for excessive special characters
        special_char_ratio = len(re.findall(r'[^a-zA-Z0-9\s]', text)) / len(text) if text else 0
        if special_char_ratio > 0.3:
            suspicious_indicators.append("high_special_char_ratio")
        
        # Check for very long words (possible obfuscation)
        words = text.split()
        long_words = [word for word in words if len(word) > 50]
        if long_words:
            suspicious_indicators.append("extremely_long_words")
        
        # Check for repeated patterns (possible DOS attack)
        if len(set(words)) / len(words) < 0.1 if words else False:
            suspicious_indicators.append("low_word_diversity")
        
        # Check for base64-like patterns
        base64_pattern = re.compile(r'[A-Za-z0-9+/]{20,}={0,2}')
        if base64_pattern.search(text):
            suspicious_indicators.append("base64_like_content")
        
        # Check for hex patterns
        hex_pattern = re.compile(r'[0-9a-fA-F]{32,}')
        if hex_pattern.search(text):
            suspicious_indicators.append("hex_like_content")
        
        return suspicious_indicators


class SecurityManager:
    """Comprehensive security management system"""
    
    def __init__(self, security_level: SecurityLevel = SecurityLevel.STANDARD):
        self.security_level = security_level
        self.rate_limiter = RateLimiter(max_requests=100, time_window=60)
        self.input_sanitizer = InputSanitizer(security_level)
        self.violation_history: List[SecurityViolation] = []
        self.blocked_ips: Set[str] = set()
        self.api_keys: Dict[str, Dict[str, Any]] = {}
        self._lock = threading.RLock()
        
        # Security metrics
        self.security_metrics = {
            'total_requests': 0,
            'blocked_requests': 0,
            'violations_detected': 0,
            'rate_limit_hits': 0
        }
        
        logger.info(f"SecurityManager initialized with level: {security_level.value}")
    
    def authenticate_request(self, api_key: Optional[str], source_ip: str) -> Tuple[bool, Optional[str]]:
        """
        Authenticate API request
        
        Args:
            api_key: API key for authentication
            source_ip: Source IP address
            
        Returns:
            Tuple of (is_authenticated, user_id)
        """
        with self._lock:
            self.security_metrics['total_requests'] += 1
            
            # Check if IP is blocked
            if source_ip in self.blocked_ips:
                self.security_metrics['blocked_requests'] += 1
                self._log_security_event("blocked_ip_access", source_ip=source_ip)
                return False, None
            
            # Check rate limiting
            if not self.rate_limiter.is_allowed(source_ip):
                self.security_metrics['rate_limit_hits'] += 1
                self._log_security_event("rate_limit_exceeded", source_ip=source_ip)
                return False, None
            
            # API key authentication (if required)
            if self.security_level in [SecurityLevel.HIGH, SecurityLevel.MAXIMUM]:
                if not api_key:
                    return False, None
                
                if api_key not in self.api_keys:
                    self._log_security_event("invalid_api_key", source_ip=source_ip)
                    return False, None
                
                key_info = self.api_keys[api_key]
                
                # Check if key is active
                if not key_info.get('active', True):
                    self._log_security_event("inactive_api_key", source_ip=source_ip)
                    return False, None
                
                # Check key expiration
                if 'expires_at' in key_info and time.time() > key_info['expires_at']:
                    self._log_security_event("expired_api_key", source_ip=source_ip)
                    return False, None
                
                return True, key_info.get('user_id')
            
            # For basic/standard security, allow all requests that pass rate limiting
            return True, None
    
    def validate_and_sanitize_input(self, text: str, source_ip: str, user_id: Optional[str] = None) -> Tuple[str, bool]:
        """
        Validate and sanitize input text
        
        Args:
            text: Input text to validate
            source_ip: Source IP address
            user_id: User ID if authenticated
            
        Returns:
            Tuple of (sanitized_text, is_safe)
        """
        with self._lock:
            # Sanitize input
            sanitized_text, violations = self.input_sanitizer.sanitize_input(text)
            
            # Record violations
            for violation in violations:
                violation.source_ip = source_ip
                violation.user_id = user_id
                self.violation_history.append(violation)
                self.security_metrics['violations_detected'] += 1
                
                # Block IP for high-severity violations in high security mode
                if (violation.severity == "high" and 
                    self.security_level in [SecurityLevel.HIGH, SecurityLevel.MAXIMUM]):
                    self._consider_ip_blocking(source_ip, violation)
            
            # Check if input is safe
            is_safe = len([v for v in violations if v.severity in ["high", "critical"]]) == 0
            
            return sanitized_text, is_safe
    
    def _consider_ip_blocking(self, source_ip: str, violation: SecurityViolation):
        """Consider blocking an IP based on violations"""
        # Count recent violations from this IP
        recent_violations = [
            v for v in self.violation_history[-100:]  # Last 100 violations
            if (v.source_ip == source_ip and 
                time.time() - v.timestamp < 3600 and  # Last hour
                v.severity == "high")
        ]
        
        # Block if too many high-severity violations
        if len(recent_violations) >= 3:
            self.blocked_ips.add(source_ip)
            logger.critical(f"IP {source_ip} blocked due to repeated security violations")
    
    def _log_security_event(self, event_type: str, source_ip: str, details: Optional[Dict] = None):
        """Log security events"""
        event_details = {
            'event_type': event_type,
            'source_ip': source_ip,
            'timestamp': time.time()
        }
        if details:
            event_details.update(details)
        
        logger.warning(f"Security event: {event_type} from {source_ip}")
    
    def generate_api_key(self, user_id: str, expires_in_days: Optional[int] = None) -> str:
        """Generate new API key for user"""
        api_key = secrets.token_urlsafe(32)
        
        key_info = {
            'user_id': user_id,
            'created_at': time.time(),
            'active': True,
            'usage_count': 0
        }
        
        if expires_in_days:
            key_info['expires_at'] = time.time() + (expires_in_days * 24 * 3600)
        
        with self._lock:
            self.api_keys[api_key] = key_info
        
        logger.info(f"API key generated for user: {user_id}")
        return api_key
    
    def revoke_api_key(self, api_key: str) -> bool:
        """Revoke an API key"""
        with self._lock:
            if api_key in self.api_keys:
                self.api_keys[api_key]['active'] = False
                logger.info(f"API key revoked for user: {self.api_keys[api_key].get('user_id')}")
                return True
            return False
    
    def unblock_ip(self, ip_address: str) -> bool:
        """Unblock an IP address"""
        with self._lock:
            if ip_address in self.blocked_ips:
                self.blocked_ips.remove(ip_address)
                logger.info(f"IP unblocked: {ip_address}")
                return True
            return False
    
    def get_security_metrics(self) -> Dict[str, Any]:
        """Get security metrics and statistics"""
        with self._lock:
            recent_violations = [
                v for v in self.violation_history
                if time.time() - v.timestamp < 3600  # Last hour
            ]
            
            violation_types = {}
            for violation in recent_violations:
                violation_types[violation.violation_type] = violation_types.get(violation.violation_type, 0) + 1
            
            return {
                'total_requests': self.security_metrics['total_requests'],
                'blocked_requests': self.security_metrics['blocked_requests'],
                'violations_detected': self.security_metrics['violations_detected'],
                'rate_limit_hits': self.security_metrics['rate_limit_hits'],
                'blocked_ips_count': len(self.blocked_ips),
                'active_api_keys': len([k for k in self.api_keys.values() if k.get('active', True)]),
                'recent_violations_1h': len(recent_violations),
                'violation_types_1h': violation_types
            }
    
    def clear_security_history(self, older_than_hours: Optional[float] = 24):
        """Clear old security history"""
        with self._lock:
            if older_than_hours:
                cutoff_time = time.time() - (older_than_hours * 3600)
                original_count = len(self.violation_history)
                self.violation_history = [
                    v for v in self.violation_history
                    if v.timestamp > cutoff_time
                ]
                cleared_count = original_count - len(self.violation_history)
                logger.info(f"Cleared {cleared_count} security records older than {older_than_hours} hours")
            else:
                self.violation_history.clear()
                logger.info("All security history cleared")


def secure_endpoint(security_level: SecurityLevel = SecurityLevel.STANDARD):
    """Decorator to secure API endpoints"""
    
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            # This would integrate with web framework
            # For now, it's a placeholder for security integration
            return func(*args, **kwargs)
        return wrapper
    return decorator