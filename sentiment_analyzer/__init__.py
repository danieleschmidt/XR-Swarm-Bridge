"""
Sentiment Analyzer Pro - Advanced sentiment analysis with multiple model support
"""

__version__ = "1.0.0"
__author__ = "Terragon Labs"
__description__ = "Professional-grade sentiment analysis with research capabilities"

from .core.analyzer import SentimentAnalyzer
from .core.models import ModelRegistry
from .core.pipeline import AnalysisPipeline
from .utils.metrics import SentimentMetrics

__all__ = [
    'SentimentAnalyzer',
    'ModelRegistry', 
    'AnalysisPipeline',
    'SentimentMetrics'
]