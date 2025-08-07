"""
Input validation and data sanitization for sentiment analysis
"""

import re
import html
import unicodedata
from typing import Union, List, Optional, Dict, Any
import logging
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class ValidationLevel(Enum):
    """Validation strictness levels"""
    STRICT = "strict"
    MODERATE = "moderate"  
    PERMISSIVE = "permissive"


@dataclass
class ValidationResult:
    """Result of input validation"""
    is_valid: bool
    cleaned_text: Optional[str] = None
    warnings: List[str] = None
    errors: List[str] = None
    metadata: Optional[Dict[str, Any]] = None


class InputValidator:
    """Comprehensive input validation and sanitization"""
    
    def __init__(self, validation_level: ValidationLevel = ValidationLevel.MODERATE):
        self.validation_level = validation_level
        self.max_length_limits = {
            ValidationLevel.STRICT: 1000,
            ValidationLevel.MODERATE: 5000,
            ValidationLevel.PERMISSIVE: 50000
        }
        self.min_length_limits = {
            ValidationLevel.STRICT: 3,
            ValidationLevel.MODERATE: 1,
            ValidationLevel.PERMISSIVE: 1
        }
        
        # Compile regex patterns for efficiency
        self._compile_patterns()
        logger.info(f"InputValidator initialized with level: {validation_level.value}")
    
    def _compile_patterns(self):
        """Compile regex patterns for text cleaning"""
        self.patterns = {
            'html_tags': re.compile(r'<[^>]+>'),
            'urls': re.compile(r'http[s]?://(?:[a-zA-Z]|[0-9]|[$-_@.&+]|[!*\\(\\),]|(?:%[0-9a-fA-F][0-9a-fA-F]))+'),
            'email': re.compile(r'\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}\b'),
            'phone': re.compile(r'\b(?:\+?1[-.]?)?\(?([0-9]{3})\)?[-.]?([0-9]{3})[-.]?([0-9]{4})\b'),
            'excessive_whitespace': re.compile(r'\s+'),
            'excessive_punctuation': re.compile(r'([.!?]){3,}'),
            'non_printable': re.compile(r'[\x00-\x08\x0b\x0c\x0e-\x1f\x7f-\x84\x86-\x9f]'),
            'special_chars': re.compile(r'[^\w\s.,!?;:\-\'\"()[\]{}]'),
            'repeated_chars': re.compile(r'(.)\1{4,}'),  # More than 4 repeated characters
            'script_tags': re.compile(r'<script[^>]*>.*?</script>', re.IGNORECASE | re.DOTALL),
            'style_tags': re.compile(r'<style[^>]*>.*?</style>', re.IGNORECASE | re.DOTALL)
        }
    
    def validate_and_clean(self, text: Union[str, List[str]]) -> Union[ValidationResult, List[ValidationResult]]:
        """
        Validate and clean input text(s)
        
        Args:
            text: Single text string or list of texts
            
        Returns:
            ValidationResult or list of ValidationResults
        """
        if isinstance(text, list):
            return [self._validate_single_text(t) for t in text]
        else:
            return self._validate_single_text(text)
    
    def _validate_single_text(self, text: Any) -> ValidationResult:
        """Validate and clean a single text input"""
        warnings = []
        errors = []
        metadata = {}
        
        # Type validation
        if not isinstance(text, str):
            if text is None:
                errors.append("Input text is None")
                return ValidationResult(False, None, warnings, errors, metadata)
            else:
                # Try to convert to string
                try:
                    text = str(text)
                    warnings.append(f"Input converted from {type(text).__name__} to string")
                except Exception as e:
                    errors.append(f"Cannot convert input to string: {e}")
                    return ValidationResult(False, None, warnings, errors, metadata)
        
        # Store original text info
        original_length = len(text)
        metadata['original_length'] = original_length
        
        # Empty text check
        if not text.strip():
            if self.validation_level == ValidationLevel.STRICT:
                errors.append("Empty text not allowed in strict mode")
                return ValidationResult(False, None, warnings, errors, metadata)
            else:
                warnings.append("Empty or whitespace-only text provided")
                return ValidationResult(True, "", warnings, errors, metadata)
        
        # Length validation
        max_length = self.max_length_limits[self.validation_level]
        min_length = self.min_length_limits[self.validation_level]
        
        if len(text) > max_length:
            if self.validation_level == ValidationLevel.STRICT:
                errors.append(f"Text length {len(text)} exceeds maximum {max_length}")
                return ValidationResult(False, None, warnings, errors, metadata)
            else:
                warnings.append(f"Text truncated from {len(text)} to {max_length} characters")
                text = text[:max_length]
        
        if len(text.strip()) < min_length:
            if self.validation_level == ValidationLevel.STRICT:
                errors.append(f"Text length {len(text.strip())} below minimum {min_length}")
                return ValidationResult(False, None, warnings, errors, metadata)
            else:
                warnings.append(f"Very short text: {len(text.strip())} characters")
        
        # Clean the text
        cleaned_text = self._clean_text(text, warnings, metadata)
        
        # Final validation
        if not cleaned_text.strip() and original_length > 0:
            warnings.append("Text became empty after cleaning")
        
        metadata['final_length'] = len(cleaned_text)
        metadata['cleaning_ratio'] = len(cleaned_text) / original_length if original_length > 0 else 0
        
        is_valid = len(errors) == 0
        
        return ValidationResult(is_valid, cleaned_text, warnings, errors, metadata)
    
    def _clean_text(self, text: str, warnings: List[str], metadata: Dict[str, Any]) -> str:
        """Clean and sanitize text based on validation level"""
        original_text = text
        cleaning_steps = []
        
        # Security cleaning (always applied)
        if self.patterns['script_tags'].search(text):
            text = self.patterns['script_tags'].sub('', text)
            warnings.append("Script tags removed for security")
            cleaning_steps.append("script_removal")
        
        if self.patterns['style_tags'].search(text):
            text = self.patterns['style_tags'].sub('', text)
            warnings.append("Style tags removed")
            cleaning_steps.append("style_removal")
        
        # HTML decoding
        if '&' in text and (';' in text):
            try:
                decoded = html.unescape(text)
                if decoded != text:
                    text = decoded
                    cleaning_steps.append("html_decode")
            except Exception as e:
                logger.warning(f"HTML decoding failed: {e}")
        
        # Remove HTML tags
        if self.patterns['html_tags'].search(text):
            text = self.patterns['html_tags'].sub(' ', text)
            cleaning_steps.append("html_tags")
            if self.validation_level == ValidationLevel.STRICT:
                warnings.append("HTML tags removed")
        
        # Handle URLs and emails based on validation level
        if self.validation_level in [ValidationLevel.STRICT, ValidationLevel.MODERATE]:
            # Replace URLs with placeholder
            url_matches = len(self.patterns['urls'].findall(text))
            if url_matches > 0:
                text = self.patterns['urls'].sub(' [URL] ', text)
                cleaning_steps.append("url_replacement")
                metadata['urls_found'] = url_matches
            
            # Replace emails with placeholder
            email_matches = len(self.patterns['email'].findall(text))
            if email_matches > 0:
                text = self.patterns['email'].sub(' [EMAIL] ', text)
                cleaning_steps.append("email_replacement")
                metadata['emails_found'] = email_matches
            
            # Replace phone numbers with placeholder
            phone_matches = len(self.patterns['phone'].findall(text))
            if phone_matches > 0:
                text = self.patterns['phone'].sub(' [PHONE] ', text)
                cleaning_steps.append("phone_replacement")
                metadata['phones_found'] = phone_matches
        
        # Unicode normalization
        try:
            normalized = unicodedata.normalize('NFKC', text)
            if normalized != text:
                text = normalized
                cleaning_steps.append("unicode_normalization")
        except Exception as e:
            logger.warning(f"Unicode normalization failed: {e}")
        
        # Remove non-printable characters
        non_printable_count = len(self.patterns['non_printable'].findall(text))
        if non_printable_count > 0:
            text = self.patterns['non_printable'].sub('', text)
            cleaning_steps.append("non_printable_removal")
            metadata['non_printable_removed'] = non_printable_count
        
        # Handle special characters based on validation level
        if self.validation_level == ValidationLevel.STRICT:
            special_char_count = len(self.patterns['special_chars'].findall(text))
            if special_char_count > 0:
                text = self.patterns['special_chars'].sub(' ', text)
                cleaning_steps.append("special_chars_removal")
                metadata['special_chars_removed'] = special_char_count
        
        # Fix excessive punctuation
        if self.patterns['excessive_punctuation'].search(text):
            text = self.patterns['excessive_punctuation'].sub(r'\1\1', text)
            cleaning_steps.append("punctuation_normalization")
        
        # Fix repeated characters (like "sooooo good" -> "soo good")
        if self.patterns['repeated_chars'].search(text):
            text = self.patterns['repeated_chars'].sub(r'\1\1', text)
            cleaning_steps.append("repeated_chars_normalization")
        
        # Normalize whitespace (always last step)
        text = self.patterns['excessive_whitespace'].sub(' ', text).strip()
        if 'excessive_whitespace' not in text or len(text.split()) != len(original_text.split()):
            cleaning_steps.append("whitespace_normalization")
        
        metadata['cleaning_steps'] = cleaning_steps
        metadata['cleaning_applied'] = len(cleaning_steps) > 0
        
        return text
    
    def validate_batch_size(self, batch_size: int, max_batch_size: int = 1000) -> bool:
        """Validate batch processing parameters"""
        if not isinstance(batch_size, int):
            raise ValueError("Batch size must be an integer")
        
        if batch_size <= 0:
            raise ValueError("Batch size must be positive")
        
        if batch_size > max_batch_size:
            raise ValueError(f"Batch size {batch_size} exceeds maximum {max_batch_size}")
        
        return True
    
    def validate_config(self, config: Dict[str, Any]) -> List[str]:
        """Validate configuration parameters"""
        errors = []
        
        # Check required fields
        required_fields = ['model_type']
        for field in required_fields:
            if field not in config:
                errors.append(f"Missing required config field: {field}")
        
        # Validate model_type
        if 'model_type' in config:
            valid_models = ['lexicon', 'rule_based', 'transformer', 'ensemble']
            if config['model_type'] not in valid_models:
                errors.append(f"Invalid model_type: {config['model_type']}. Valid options: {valid_models}")
        
        # Validate numeric parameters
        numeric_fields = {
            'batch_size': (1, 10000),
            'timeout_seconds': (1.0, 3600.0),
            'confidence_threshold': (0.0, 1.0)
        }
        
        for field, (min_val, max_val) in numeric_fields.items():
            if field in config:
                value = config[field]
                if not isinstance(value, (int, float)):
                    errors.append(f"{field} must be a number")
                elif not (min_val <= value <= max_val):
                    errors.append(f"{field} must be between {min_val} and {max_val}")
        
        # Validate boolean fields
        boolean_fields = ['enable_preprocessing', 'enable_postprocessing', 'cache_results']
        for field in boolean_fields:
            if field in config and not isinstance(config[field], bool):
                errors.append(f"{field} must be a boolean")
        
        return errors
    
    def sanitize_filename(self, filename: str) -> str:
        """Sanitize filename for safe file operations"""
        if not isinstance(filename, str):
            raise ValueError("Filename must be a string")
        
        # Remove/replace dangerous characters
        filename = re.sub(r'[<>:"/\\|?*]', '_', filename)
        
        # Remove leading/trailing dots and spaces
        filename = filename.strip('. ')
        
        # Ensure reasonable length
        if len(filename) > 255:
            name, ext = filename.rsplit('.', 1) if '.' in filename else (filename, '')
            filename = name[:250] + ('.' + ext if ext else '')
        
        # Prevent empty filename
        if not filename:
            filename = 'unnamed_file'
        
        return filename
    
    def check_text_language(self, text: str) -> Optional[str]:
        """Detect text language (basic implementation)"""
        try:
            # Simple heuristic-based language detection
            # In a full implementation, this would use proper language detection libraries
            
            # Count common English words
            english_indicators = [
                'the', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with',
                'by', 'from', 'up', 'about', 'into', 'through', 'during', 'before', 'after',
                'above', 'below', 'between', 'among', 'this', 'that', 'these', 'those',
                'i', 'you', 'he', 'she', 'it', 'we', 'they', 'me', 'him', 'her', 'us', 'them',
                'is', 'am', 'are', 'was', 'were', 'be', 'been', 'being', 'have', 'has', 'had',
                'do', 'does', 'did', 'will', 'would', 'could', 'should', 'may', 'might', 'must'
            ]
            
            words = text.lower().split()
            english_word_count = sum(1 for word in words if word.strip('.,!?";:()[]{}') in english_indicators)
            
            if len(words) > 0:
                english_ratio = english_word_count / len(words)
                if english_ratio > 0.1:  # If more than 10% are common English words
                    return 'en'
            
            return None  # Unknown language
            
        except Exception as e:
            logger.warning(f"Language detection failed: {e}")
            return None