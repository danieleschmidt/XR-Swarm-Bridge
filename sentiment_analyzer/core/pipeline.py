"""
Analysis pipeline for processing sentiment analysis workflows
"""

from typing import List, Dict, Any, Optional, Callable
import time
import logging
from dataclasses import dataclass
from .analyzer import SentimentAnalyzer, SentimentResult
from .models import ModelRegistry

logger = logging.getLogger(__name__)


@dataclass
class PipelineConfig:
    """Configuration for analysis pipeline"""
    model_type: str = "lexicon"
    batch_size: int = 100
    enable_preprocessing: bool = True
    enable_postprocessing: bool = True
    cache_results: bool = True
    timeout_seconds: float = 30.0


class AnalysisPipeline:
    """
    Sentiment analysis pipeline for processing large volumes of text
    """
    
    def __init__(self, config: Optional[PipelineConfig] = None):
        self.config = config or PipelineConfig()
        self.model_registry = ModelRegistry()
        self.analyzer = SentimentAnalyzer(self.config.model_type)
        self.cache = {} if self.config.cache_results else None
        self.preprocessing_steps = []
        self.postprocessing_steps = []
        self._setup_default_processors()
        
        logger.info(f"AnalysisPipeline initialized with model: {self.config.model_type}")
    
    def _setup_default_processors(self):
        """Setup default preprocessing and postprocessing steps"""
        if self.config.enable_preprocessing:
            self.preprocessing_steps.extend([
                self._validate_input,
                self._clean_text,
                self._normalize_text
            ])
        
        if self.config.enable_postprocessing:
            self.postprocessing_steps.extend([
                self._validate_results,
                self._enrich_metadata,
                self._apply_business_rules
            ])
    
    def add_preprocessing_step(self, step_func: Callable):
        """Add a custom preprocessing step"""
        self.preprocessing_steps.append(step_func)
        logger.info("Custom preprocessing step added")
    
    def add_postprocessing_step(self, step_func: Callable):
        """Add a custom postprocessing step"""
        self.postprocessing_steps.append(step_func)
        logger.info("Custom postprocessing step added")
    
    def process_single(self, text: str, **kwargs) -> SentimentResult:
        """Process a single text through the pipeline"""
        start_time = time.time()
        
        try:
            # Check cache first
            if self.cache is not None:
                cache_key = hash(text)
                if cache_key in self.cache:
                    logger.debug("Cache hit for text analysis")
                    return self.cache[cache_key]
            
            # Preprocessing
            processed_text = text
            for step in self.preprocessing_steps:
                processed_text = step(processed_text)
            
            # Core analysis
            result = self.analyzer.analyze(processed_text)
            
            # Postprocessing
            for step in self.postprocessing_steps:
                result = step(result)
            
            # Cache result
            if self.cache is not None:
                self.cache[cache_key] = result
            
            processing_time = time.time() - start_time
            result.processing_time = processing_time
            
            logger.debug(f"Single text processed in {processing_time:.3f}s")
            return result
            
        except Exception as e:
            logger.error(f"Error in pipeline processing: {e}")
            raise
    
    def process_batch(self, texts: List[str], **kwargs) -> List[SentimentResult]:
        """Process a batch of texts through the pipeline"""
        start_time = time.time()
        
        if not texts:
            return []
        
        logger.info(f"Processing batch of {len(texts)} texts")
        
        results = []
        batch_size = self.config.batch_size
        
        # Process in chunks
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i+batch_size]
            batch_results = []
            
            for text in batch:
                try:
                    result = self.process_single(text, **kwargs)
                    batch_results.append(result)
                except Exception as e:
                    logger.error(f"Error processing text in batch: {e}")
                    # Create error result
                    error_result = SentimentResult(
                        text=text,
                        sentiment=self.analyzer._classify_sentiment({'positive': 0, 'negative': 0, 'neutral': 1}),
                        confidence=0.0,
                        scores={'positive': 0.0, 'negative': 0.0, 'neutral': 1.0},
                        model_used=self.config.model_type,
                        processing_time=0.0,
                        metadata={'error': str(e)}
                    )
                    batch_results.append(error_result)
            
            results.extend(batch_results)
            logger.debug(f"Processed batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")
        
        total_time = time.time() - start_time
        logger.info(f"Batch processing completed in {total_time:.2f}s")
        
        return results
    
    def process_stream(self, text_stream, **kwargs):
        """Process a stream of texts (generator/iterator)"""
        logger.info("Starting stream processing")
        
        for text in text_stream:
            try:
                result = self.process_single(text, **kwargs)
                yield result
            except Exception as e:
                logger.error(f"Error in stream processing: {e}")
                continue
    
    # Preprocessing steps
    def _validate_input(self, text: str) -> str:
        """Validate input text"""
        if not isinstance(text, str):
            raise ValueError("Input must be a string")
        
        if len(text.strip()) == 0:
            raise ValueError("Input text cannot be empty")
        
        if len(text) > 10000:  # Reasonable limit
            logger.warning("Text length exceeds 10,000 characters, truncating")
            text = text[:10000]
        
        return text
    
    def _clean_text(self, text: str) -> str:
        """Clean input text"""
        import re
        
        # Remove excessive whitespace
        text = re.sub(r'\s+', ' ', text).strip()
        
        # Remove or replace problematic characters
        text = re.sub(r'[^\w\s\.,!?;:\-\'"()]', ' ', text)
        
        return text
    
    def _normalize_text(self, text: str) -> str:
        """Normalize text for analysis"""
        # Basic normalization (can be enhanced in later generations)
        text = text.strip()
        return text
    
    # Postprocessing steps
    def _validate_results(self, result: SentimentResult) -> SentimentResult:
        """Validate analysis results"""
        # Ensure confidence is between 0 and 1
        result.confidence = max(0.0, min(1.0, result.confidence))
        
        # Ensure scores sum to reasonable values
        total_score = sum(result.scores.values())
        if total_score > 1.1 or total_score < 0.9:
            logger.warning(f"Score normalization issue detected: {total_score}")
        
        return result
    
    def _enrich_metadata(self, result: SentimentResult) -> SentimentResult:
        """Add enriched metadata to results"""
        if result.metadata is None:
            result.metadata = {}
        
        result.metadata.update({
            'pipeline_version': '1.0',
            'model_config': self.config.model_type,
            'timestamp': time.time()
        })
        
        return result
    
    def _apply_business_rules(self, result: SentimentResult) -> SentimentResult:
        """Apply business-specific rules to results"""
        # Example business rules (can be customized)
        
        # Adjust confidence for very short texts
        if len(result.text.split()) < 3:
            result.confidence *= 0.8
            if result.metadata:
                result.metadata['confidence_adjusted'] = 'short_text'
        
        # Flag uncertain predictions
        if result.confidence < 0.6:
            if result.metadata:
                result.metadata['uncertain_prediction'] = True
        
        return result
    
    def get_pipeline_stats(self) -> Dict[str, Any]:
        """Get pipeline performance statistics"""
        stats = {
            'model_type': self.config.model_type,
            'cache_size': len(self.cache) if self.cache else 0,
            'preprocessing_steps': len(self.preprocessing_steps),
            'postprocessing_steps': len(self.postprocessing_steps),
            'batch_size': self.config.batch_size
        }
        
        return stats
    
    def clear_cache(self):
        """Clear the results cache"""
        if self.cache:
            self.cache.clear()
            logger.info("Pipeline cache cleared")
    
    def switch_model(self, model_type: str):
        """Switch to a different analysis model"""
        self.config.model_type = model_type
        self.analyzer = SentimentAnalyzer(model_type)
        self.clear_cache()  # Clear cache when switching models
        logger.info(f"Switched to model: {model_type}")