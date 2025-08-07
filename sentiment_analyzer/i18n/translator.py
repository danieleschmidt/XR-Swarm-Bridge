"""
Translation and multi-language support utilities
"""

from typing import Dict, Optional, List
import logging

logger = logging.getLogger(__name__)


class MultiLanguageSupport:
    """Multi-language support utilities"""
    
    def __init__(self):
        self.supported_languages = ['en', 'es', 'fr', 'de', 'ja', 'zh', 'ar']
        logger.info("MultiLanguageSupport initialized")
    
    def get_supported_languages(self) -> List[str]:
        """Get list of supported languages"""
        return self.supported_languages.copy()
    
    def is_language_supported(self, language_code: str) -> bool:
        """Check if language is supported"""
        return language_code in self.supported_languages