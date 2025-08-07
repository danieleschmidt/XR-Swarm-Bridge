"""
Model registry and management for different sentiment analysis approaches
"""

from typing import Dict, Type, Optional, Any
from abc import ABC, abstractmethod
import logging

logger = logging.getLogger(__name__)


class BaseSentimentModel(ABC):
    """Abstract base class for sentiment analysis models"""
    
    def __init__(self, model_name: str, **kwargs):
        self.model_name = model_name
        self.config = kwargs
        self.is_loaded = False
    
    @abstractmethod
    def load(self) -> None:
        """Load the model"""
        pass
    
    @abstractmethod
    def predict(self, text: str) -> Dict[str, Any]:
        """Predict sentiment for input text"""
        pass
    
    @abstractmethod
    def predict_batch(self, texts: list) -> list:
        """Predict sentiment for batch of texts"""
        pass


class LexiconModel(BaseSentimentModel):
    """Lexicon-based sentiment analysis model"""
    
    def __init__(self, **kwargs):
        super().__init__("lexicon", **kwargs)
        self.lexicon_data = {}
    
    def load(self) -> None:
        """Load lexicon data"""
        # Basic lexicon loading (already implemented in analyzer)
        self.is_loaded = True
        logger.info("Lexicon model loaded successfully")
    
    def predict(self, text: str) -> Dict[str, Any]:
        """Predict using lexicon approach"""
        if not self.is_loaded:
            self.load()
        
        # Implementation would use the lexicon analysis
        # This is a placeholder for model interface compatibility
        return {
            'sentiment': 'neutral',
            'confidence': 0.5,
            'scores': {'positive': 0.0, 'negative': 0.0, 'neutral': 1.0}
        }
    
    def predict_batch(self, texts: list) -> list:
        """Batch prediction using lexicon approach"""
        return [self.predict(text) for text in texts]


class RuleBasedModel(BaseSentimentModel):
    """Rule-based sentiment analysis model with linguistic patterns"""
    
    def __init__(self, **kwargs):
        super().__init__("rule_based", **kwargs)
        self.rules = []
    
    def load(self) -> None:
        """Load rule patterns"""
        # Define basic sentiment rules
        self.rules = [
            # Positive patterns
            (r'\b(love|like|enjoy|appreciate)\b', 'positive', 0.7),
            (r'\b(excellent|great|amazing|wonderful)\b', 'positive', 0.8),
            (r'\b(good|nice|fine|okay)\b', 'positive', 0.5),
            
            # Negative patterns  
            (r'\b(hate|dislike|terrible|awful)\b', 'negative', 0.8),
            (r'\b(bad|poor|disappointing)\b', 'negative', 0.6),
            (r'\b(annoyed|frustrated|upset)\b', 'negative', 0.7),
            
            # Intensifiers
            (r'\b(very|really|extremely|incredibly)\s+(\w+)', 'intensifier', 1.3),
            
            # Negations
            (r'\bnot\s+(\w+)', 'negation', -0.8)
        ]
        
        self.is_loaded = True
        logger.info("Rule-based model loaded with {} rules".format(len(self.rules)))
    
    def predict(self, text: str) -> Dict[str, Any]:
        """Predict using rule-based approach"""
        if not self.is_loaded:
            self.load()
        
        import re
        positive_score = 0.0
        negative_score = 0.0
        
        text_lower = text.lower()
        
        for pattern, sentiment_type, weight in self.rules:
            matches = re.findall(pattern, text_lower)
            if matches:
                if sentiment_type == 'positive':
                    positive_score += weight * len(matches)
                elif sentiment_type == 'negative':
                    negative_score += weight * len(matches)
        
        # Normalize scores
        total_score = positive_score + negative_score
        if total_score > 0:
            pos_normalized = positive_score / total_score
            neg_normalized = negative_score / total_score
        else:
            pos_normalized = neg_normalized = 0.0
        
        neutral_normalized = 1.0 - pos_normalized - neg_normalized
        
        # Determine sentiment
        if pos_normalized > neg_normalized:
            sentiment = 'positive'
            confidence = pos_normalized
        elif neg_normalized > pos_normalized:
            sentiment = 'negative'
            confidence = neg_normalized
        else:
            sentiment = 'neutral'
            confidence = neutral_normalized
        
        return {
            'sentiment': sentiment,
            'confidence': min(1.0, confidence),
            'scores': {
                'positive': pos_normalized,
                'negative': neg_normalized, 
                'neutral': max(0.0, neutral_normalized)
            }
        }
    
    def predict_batch(self, texts: list) -> list:
        """Batch prediction using rules"""
        return [self.predict(text) for text in texts]


class ModelRegistry:
    """Registry for managing different sentiment analysis models"""
    
    def __init__(self):
        self._models: Dict[str, Type[BaseSentimentModel]] = {}
        self._loaded_models: Dict[str, BaseSentimentModel] = {}
        self._register_default_models()
    
    def _register_default_models(self):
        """Register default models"""
        self.register_model("lexicon", LexiconModel)
        self.register_model("rule_based", RuleBasedModel)
        logger.info("Default models registered: lexicon, rule_based")
    
    def register_model(self, name: str, model_class: Type[BaseSentimentModel]):
        """Register a new model class"""
        if not issubclass(model_class, BaseSentimentModel):
            raise ValueError("Model class must inherit from BaseSentimentModel")
        
        self._models[name] = model_class
        logger.info(f"Model '{name}' registered successfully")
    
    def get_model(self, name: str, **kwargs) -> BaseSentimentModel:
        """Get a model instance"""
        if name not in self._models:
            available_models = list(self._models.keys())
            raise ValueError(f"Model '{name}' not found. Available models: {available_models}")
        
        # Check if model is already loaded
        model_key = f"{name}_{hash(str(sorted(kwargs.items())))}"
        
        if model_key not in self._loaded_models:
            model_class = self._models[name]
            model_instance = model_class(**kwargs)
            model_instance.load()
            self._loaded_models[model_key] = model_instance
        
        return self._loaded_models[model_key]
    
    def list_models(self) -> list:
        """List all registered models"""
        return list(self._models.keys())
    
    def unload_model(self, name: str):
        """Unload a model from memory"""
        keys_to_remove = [key for key in self._loaded_models.keys() if key.startswith(name)]
        for key in keys_to_remove:
            del self._loaded_models[key]
        logger.info(f"Model '{name}' unloaded from memory")
    
    def clear_all_models(self):
        """Clear all loaded models from memory"""
        self._loaded_models.clear()
        logger.info("All models cleared from memory")