"""
Core sentiment analysis engine with multiple model support
"""

import re
import json
from typing import Dict, List, Optional, Union, Tuple
from dataclasses import dataclass
from enum import Enum
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SentimentLabel(Enum):
    """Sentiment classification labels"""
    POSITIVE = "positive"
    NEGATIVE = "negative"
    NEUTRAL = "neutral"


@dataclass
class SentimentResult:
    """Result container for sentiment analysis"""
    text: str
    sentiment: SentimentLabel
    confidence: float
    scores: Dict[str, float]
    model_used: str
    processing_time: float
    metadata: Optional[Dict] = None


class SentimentAnalyzer:
    """
    Advanced sentiment analysis engine with multiple model support
    Generation 1: Basic lexicon-based and rule-based analysis
    """
    
    def __init__(self, model_type: str = "lexicon"):
        self.model_type = model_type
        self.models = {}
        self._load_lexicons()
        logger.info(f"SentimentAnalyzer initialized with model: {model_type}")
    
    def _load_lexicons(self):
        """Load sentiment lexicons for basic analysis"""
        # Simple lexicon for Generation 1
        self.positive_words = {
            'good', 'great', 'excellent', 'amazing', 'awesome', 'fantastic', 
            'wonderful', 'perfect', 'love', 'like', 'happy', 'joy', 'pleased',
            'satisfied', 'brilliant', 'outstanding', 'superb', 'magnificent'
        }
        
        self.negative_words = {
            'bad', 'terrible', 'awful', 'horrible', 'hate', 'dislike', 'angry',
            'sad', 'disappointed', 'frustrated', 'annoyed', 'upset', 'poor',
            'worst', 'disgusting', 'pathetic', 'useless', 'broken'
        }
        
        self.intensifiers = {
            'very': 1.5, 'really': 1.4, 'extremely': 2.0, 'incredibly': 1.8,
            'absolutely': 1.7, 'totally': 1.6, 'completely': 1.5, 'quite': 1.2
        }
        
        self.negations = {
            'not', 'no', 'never', 'none', 'nobody', 'nothing', 'neither',
            'nowhere', 'hardly', 'barely', 'rarely', 'seldom'
        }
        
        logger.info("Lexicons loaded successfully")
    
    def analyze(self, text: str) -> SentimentResult:
        """
        Analyze sentiment of input text
        
        Args:
            text: Input text to analyze
            
        Returns:
            SentimentResult with sentiment classification and metrics
        """
        import time
        start_time = time.time()
        
        if not text or not isinstance(text, str):
            raise ValueError("Input text must be a non-empty string")
        
        # Preprocess text
        processed_text = self._preprocess_text(text)
        
        # Analyze sentiment using lexicon-based approach
        scores = self._lexicon_analysis(processed_text)
        
        # Determine sentiment label
        sentiment = self._classify_sentiment(scores)
        
        # Calculate confidence
        confidence = self._calculate_confidence(scores)
        
        processing_time = time.time() - start_time
        
        result = SentimentResult(
            text=text,
            sentiment=sentiment,
            confidence=confidence,
            scores=scores,
            model_used=self.model_type,
            processing_time=processing_time,
            metadata={'text_length': len(text), 'processed_length': len(processed_text)}
        )
        
        logger.debug(f"Analyzed text sentiment: {sentiment.value} (confidence: {confidence:.3f})")
        return result
    
    def _preprocess_text(self, text: str) -> List[str]:
        """Preprocess text for analysis"""
        # Convert to lowercase and tokenize
        text = text.lower()
        # Remove special characters but keep basic punctuation
        text = re.sub(r'[^\w\s!?.,-]', '', text)
        # Simple tokenization
        tokens = re.findall(r'\b\w+\b', text)
        return tokens
    
    def _lexicon_analysis(self, tokens: List[str]) -> Dict[str, float]:
        """Perform lexicon-based sentiment analysis"""
        positive_score = 0.0
        negative_score = 0.0
        neutral_count = 0
        
        i = 0
        while i < len(tokens):
            token = tokens[i]
            
            # Check for negations
            negation_factor = 1.0
            if i > 0 and tokens[i-1] in self.negations:
                negation_factor = -0.8
            
            # Check for intensifiers
            intensifier_factor = 1.0
            if i > 0 and tokens[i-1] in self.intensifiers:
                intensifier_factor = self.intensifiers[tokens[i-1]]
            
            # Score the token
            if token in self.positive_words:
                positive_score += (1.0 * intensifier_factor * negation_factor)
            elif token in self.negative_words:
                negative_score += (1.0 * intensifier_factor * negation_factor)
            else:
                neutral_count += 1
            
            i += 1
        
        # Normalize scores
        total_tokens = len(tokens)
        if total_tokens > 0:
            positive_score = positive_score / total_tokens
            negative_score = abs(negative_score) / total_tokens
            neutral_score = neutral_count / total_tokens
        else:
            positive_score = negative_score = neutral_score = 0.0
        
        return {
            'positive': positive_score,
            'negative': negative_score,
            'neutral': neutral_score
        }
    
    def _classify_sentiment(self, scores: Dict[str, float]) -> SentimentLabel:
        """Classify sentiment based on scores"""
        pos_score = scores.get('positive', 0.0)
        neg_score = scores.get('negative', 0.0)
        
        # Decision threshold
        threshold = 0.1
        
        if pos_score > neg_score + threshold:
            return SentimentLabel.POSITIVE
        elif neg_score > pos_score + threshold:
            return SentimentLabel.NEGATIVE
        else:
            return SentimentLabel.NEUTRAL
    
    def _calculate_confidence(self, scores: Dict[str, float]) -> float:
        """Calculate confidence score based on sentiment scores"""
        pos_score = scores.get('positive', 0.0)
        neg_score = scores.get('negative', 0.0)
        neutral_score = scores.get('neutral', 0.0)
        
        # Confidence based on the difference between max and second max scores
        max_score = max(pos_score, neg_score, neutral_score)
        sorted_scores = sorted([pos_score, neg_score, neutral_score], reverse=True)
        
        if len(sorted_scores) >= 2:
            confidence = min(1.0, max(0.0, (sorted_scores[0] - sorted_scores[1]) + 0.5))
        else:
            confidence = max_score
        
        return confidence
    
    def analyze_batch(self, texts: List[str]) -> List[SentimentResult]:
        """Analyze multiple texts in batch"""
        if not texts or not isinstance(texts, list):
            raise ValueError("Input must be a non-empty list of strings")
        
        results = []
        for text in texts:
            try:
                result = self.analyze(text)
                results.append(result)
            except Exception as e:
                logger.error(f"Error analyzing text: {e}")
                # Create error result
                error_result = SentimentResult(
                    text=text,
                    sentiment=SentimentLabel.NEUTRAL,
                    confidence=0.0,
                    scores={'positive': 0.0, 'negative': 0.0, 'neutral': 1.0},
                    model_used=self.model_type,
                    processing_time=0.0,
                    metadata={'error': str(e)}
                )
                results.append(error_result)
        
        logger.info(f"Batch analysis completed: {len(results)} texts processed")
        return results