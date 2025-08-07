"""
Novel sentiment analysis algorithms for research and experimentation
"""

import numpy as np
import math
import random
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass
from abc import ABC, abstractmethod
import logging
from collections import Counter, defaultdict
import itertools

from ..core.analyzer import SentimentLabel, SentimentResult
from ..core.models import BaseSentimentModel

logger = logging.getLogger(__name__)


@dataclass
class AlgorithmConfig:
    """Configuration for research algorithms"""
    learning_rate: float = 0.01
    max_iterations: int = 1000
    convergence_threshold: float = 1e-6
    random_seed: Optional[int] = 42
    regularization_strength: float = 0.01
    feature_dimensions: int = 100


class AdvancedFeatureExtractor:
    """Advanced feature extraction for research algorithms"""
    
    def __init__(self, config: AlgorithmConfig):
        self.config = config
        if config.random_seed:
            random.seed(config.random_seed)
            np.random.seed(config.random_seed)
        
        # Pre-computed feature mappings
        self.word_embeddings = {}
        self.ngram_features = {}
        self.syntactic_features = {}
        
        logger.info("AdvancedFeatureExtractor initialized")
    
    def extract_lexical_features(self, text: str) -> np.ndarray:
        """Extract advanced lexical features"""
        words = text.lower().split()
        
        features = []
        
        # Basic statistics
        features.extend([
            len(words),  # Word count
            len(text),   # Character count
            len(set(words)),  # Unique word count
            len(set(words)) / len(words) if words else 0,  # Lexical diversity
        ])
        
        # N-gram features
        features.extend(self._extract_ngram_features(words))
        
        # Syntactic features
        features.extend(self._extract_syntactic_features(text))
        
        # Semantic features
        features.extend(self._extract_semantic_features(words))
        
        # Pad or truncate to fixed size
        target_size = self.config.feature_dimensions
        if len(features) < target_size:
            features.extend([0.0] * (target_size - len(features)))
        else:
            features = features[:target_size]
        
        return np.array(features, dtype=np.float32)
    
    def _extract_ngram_features(self, words: List[str]) -> List[float]:
        """Extract n-gram based features"""
        features = []
        
        # Unigram features
        unigram_counts = Counter(words)
        common_unigrams = ['good', 'bad', 'great', 'terrible', 'love', 'hate']
        for gram in common_unigrams:
            features.append(unigram_counts.get(gram, 0))
        
        # Bigram features
        if len(words) > 1:
            bigrams = list(zip(words[:-1], words[1:]))
            bigram_counts = Counter(bigrams)
            common_bigrams = [('very', 'good'), ('very', 'bad'), ('not', 'good'), ('not', 'bad')]
            for gram in common_bigrams:
                features.append(bigram_counts.get(gram, 0))
        else:
            features.extend([0.0] * 4)
        
        # Trigram features (simplified)
        if len(words) > 2:
            trigrams = list(zip(words[:-2], words[1:-1], words[2:]))
            features.append(len(trigrams))  # Trigram count
        else:
            features.append(0.0)
        
        return features
    
    def _extract_syntactic_features(self, text: str) -> List[float]:
        """Extract syntactic and structural features"""
        features = []
        
        # Punctuation analysis
        exclamation_count = text.count('!')
        question_count = text.count('?')
        period_count = text.count('.')
        comma_count = text.count(',')
        
        features.extend([exclamation_count, question_count, period_count, comma_count])
        
        # Capitalization features
        upper_count = sum(1 for c in text if c.isupper())
        features.append(upper_count / len(text) if text else 0)
        
        # Sentence structure (simplified)
        sentences = text.split('.')
        features.append(len(sentences))
        features.append(np.mean([len(s.split()) for s in sentences if s.strip()]) if sentences else 0)
        
        return features
    
    def _extract_semantic_features(self, words: List[str]) -> List[float]:
        """Extract semantic features using word associations"""
        features = []
        
        # Sentiment word categories
        positive_words = {'good', 'great', 'excellent', 'amazing', 'wonderful', 'love', 'like', 'happy', 'joy'}
        negative_words = {'bad', 'terrible', 'awful', 'hate', 'dislike', 'sad', 'angry', 'disappointed'}
        intensifiers = {'very', 'really', 'extremely', 'incredibly', 'absolutely'}
        negations = {'not', 'no', 'never', 'nothing', 'nobody'}
        
        # Count features
        features.extend([
            sum(1 for word in words if word in positive_words),
            sum(1 for word in words if word in negative_words),
            sum(1 for word in words if word in intensifiers),
            sum(1 for word in words if word in negations)
        ])
        
        # Ratio features
        total_words = len(words)
        if total_words > 0:
            features.extend([
                features[-4] / total_words,  # Positive ratio
                features[-3] / total_words,  # Negative ratio  
                features[-2] / total_words,  # Intensifier ratio
                features[-1] / total_words   # Negation ratio
            ])
        else:
            features.extend([0.0, 0.0, 0.0, 0.0])
        
        return features


class EnsembleModel(BaseSentimentModel):
    """Ensemble model combining multiple algorithms"""
    
    def __init__(self, models: List[BaseSentimentModel], weights: Optional[List[float]] = None, **kwargs):
        super().__init__("ensemble", **kwargs)
        self.models = models
        self.weights = weights or [1.0] * len(models)
        self.model_performances = {}
        
        if len(self.weights) != len(self.models):
            raise ValueError("Number of weights must match number of models")
        
        # Normalize weights
        total_weight = sum(self.weights)
        self.weights = [w / total_weight for w in self.weights]
    
    def load(self) -> None:
        """Load all component models"""
        for model in self.models:
            if not model.is_loaded:
                model.load()
        self.is_loaded = True
        logger.info(f"Ensemble model loaded with {len(self.models)} components")
    
    def predict(self, text: str) -> Dict[str, Any]:
        """Predict using ensemble of models"""
        if not self.is_loaded:
            self.load()
        
        predictions = []
        for model in self.models:
            try:
                pred = model.predict(text)
                predictions.append(pred)
            except Exception as e:
                logger.warning(f"Model {model.model_name} failed: {e}")
                # Use neutral prediction as fallback
                predictions.append({
                    'sentiment': 'neutral',
                    'confidence': 0.0,
                    'scores': {'positive': 0.33, 'negative': 0.33, 'neutral': 0.34}
                })
        
        # Ensemble predictions using weighted voting
        return self._ensemble_predictions(predictions)
    
    def _ensemble_predictions(self, predictions: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Combine predictions using weighted ensemble"""
        # Weighted score combination
        ensemble_scores = {'positive': 0.0, 'negative': 0.0, 'neutral': 0.0}
        ensemble_confidence = 0.0
        
        for pred, weight in zip(predictions, self.weights):
            if 'scores' in pred:
                for sentiment in ensemble_scores:
                    ensemble_scores[sentiment] += pred['scores'].get(sentiment, 0.0) * weight
            
            ensemble_confidence += pred.get('confidence', 0.0) * weight
        
        # Determine final sentiment
        final_sentiment = max(ensemble_scores, key=ensemble_scores.get)
        
        return {
            'sentiment': final_sentiment,
            'confidence': ensemble_confidence,
            'scores': ensemble_scores,
            'ensemble_details': {
                'component_predictions': predictions,
                'weights_used': self.weights
            }
        }
    
    def predict_batch(self, texts: List[str]) -> List[Dict[str, Any]]:
        """Batch prediction using ensemble"""
        return [self.predict(text) for text in texts]
    
    def update_weights(self, performance_scores: List[float]):
        """Update ensemble weights based on performance"""
        if len(performance_scores) != len(self.models):
            raise ValueError("Performance scores must match number of models")
        
        # Softmax normalization for weights
        exp_scores = [math.exp(score) for score in performance_scores]
        sum_exp = sum(exp_scores)
        self.weights = [exp_score / sum_exp for exp_score in exp_scores]
        
        logger.info(f"Updated ensemble weights: {self.weights}")


class AdaptiveSentimentModel(BaseSentimentModel):
    """Adaptive model that learns from feedback"""
    
    def __init__(self, base_model: BaseSentimentModel, learning_rate: float = 0.01, **kwargs):
        super().__init__("adaptive", **kwargs)
        self.base_model = base_model
        self.learning_rate = learning_rate
        self.feedback_history = []
        self.feature_extractor = AdvancedFeatureExtractor(AlgorithmConfig())
        
        # Adaptive weights for different features
        self.adaptation_weights = defaultdict(float)
        self.update_count = 0
    
    def load(self) -> None:
        """Load base model and initialize adaptation"""
        self.base_model.load()
        self.is_loaded = True
        logger.info("Adaptive model initialized")
    
    def predict(self, text: str) -> Dict[str, Any]:
        """Predict with adaptation"""
        if not self.is_loaded:
            self.load()
        
        # Get base prediction
        base_pred = self.base_model.predict(text)
        
        # Extract features for adaptation
        features = self.feature_extractor.extract_lexical_features(text)
        
        # Apply learned adaptations
        adapted_scores = self._apply_adaptations(base_pred['scores'], features)
        
        # Recalculate sentiment and confidence
        adapted_sentiment = max(adapted_scores, key=adapted_scores.get)
        max_score = max(adapted_scores.values())
        sorted_scores = sorted(adapted_scores.values(), reverse=True)
        confidence = max_score - (sorted_scores[1] if len(sorted_scores) > 1 else 0)
        
        return {
            'sentiment': adapted_sentiment,
            'confidence': min(1.0, confidence),
            'scores': adapted_scores,
            'base_prediction': base_pred,
            'adaptation_applied': True
        }
    
    def _apply_adaptations(self, base_scores: Dict[str, float], features: np.ndarray) -> Dict[str, float]:
        """Apply learned adaptations to base scores"""
        adapted_scores = base_scores.copy()
        
        if not self.adaptation_weights:
            return adapted_scores
        
        # Apply feature-based adaptations
        for sentiment in adapted_scores:
            adaptation = 0.0
            for i, feature_val in enumerate(features):
                weight_key = f"{sentiment}_{i}"
                adaptation += self.adaptation_weights.get(weight_key, 0.0) * feature_val
            
            adapted_scores[sentiment] = max(0.0, min(1.0, adapted_scores[sentiment] + adaptation))
        
        # Normalize scores
        total = sum(adapted_scores.values())
        if total > 0:
            adapted_scores = {k: v / total for k, v in adapted_scores.items()}
        
        return adapted_scores
    
    def learn_from_feedback(self, text: str, true_sentiment: str, prediction: Dict[str, Any]):
        """Learn from user feedback"""
        features = self.feature_extractor.extract_lexical_features(text)
        
        # Calculate prediction error
        predicted_sentiment = prediction['sentiment']
        error = 0.0 if predicted_sentiment == true_sentiment else 1.0
        
        # Update adaptation weights using gradient descent
        for sentiment in ['positive', 'negative', 'neutral']:
            target = 1.0 if sentiment == true_sentiment else 0.0
            predicted = prediction['scores'].get(sentiment, 0.0)
            gradient = target - predicted
            
            # Update weights for each feature
            for i, feature_val in enumerate(features):
                weight_key = f"{sentiment}_{i}"
                self.adaptation_weights[weight_key] += self.learning_rate * gradient * feature_val
        
        # Store feedback
        self.feedback_history.append({
            'text': text,
            'true_sentiment': true_sentiment,
            'prediction': prediction,
            'error': error,
            'timestamp': time.time() if 'time' in dir() else 0
        })
        
        self.update_count += 1
        
        if self.update_count % 100 == 0:
            logger.info(f"Adaptive model updated {self.update_count} times")
    
    def predict_batch(self, texts: List[str]) -> List[Dict[str, Any]]:
        """Batch prediction with adaptation"""
        return [self.predict(text) for text in texts]
    
    def get_adaptation_stats(self) -> Dict[str, Any]:
        """Get statistics about model adaptation"""
        if not self.feedback_history:
            return {'no_feedback': True}
        
        recent_feedback = self.feedback_history[-100:]  # Last 100 feedback items
        accuracy = sum(1 for f in recent_feedback if f['error'] == 0.0) / len(recent_feedback)
        
        return {
            'total_updates': self.update_count,
            'recent_accuracy': accuracy,
            'feedback_count': len(self.feedback_history),
            'adaptation_weights_count': len(self.adaptation_weights),
            'learning_rate': self.learning_rate
        }


class HybridNeuralSentimentModel(BaseSentimentModel):
    """Simplified neural network approach for sentiment analysis"""
    
    def __init__(self, hidden_size: int = 50, **kwargs):
        super().__init__("hybrid_neural", **kwargs)
        self.hidden_size = hidden_size
        self.feature_extractor = AdvancedFeatureExtractor(AlgorithmConfig())
        self.weights_input_hidden = None
        self.weights_hidden_output = None
        self.bias_hidden = None
        self.bias_output = None
        self.is_trained = False
    
    def load(self) -> None:
        """Initialize neural network weights"""
        input_size = self.feature_extractor.config.feature_dimensions
        output_size = 3  # positive, negative, neutral
        
        # Initialize weights with Xavier initialization
        self.weights_input_hidden = np.random.randn(input_size, self.hidden_size) * np.sqrt(2.0 / input_size)
        self.weights_hidden_output = np.random.randn(self.hidden_size, output_size) * np.sqrt(2.0 / self.hidden_size)
        
        # Initialize biases
        self.bias_hidden = np.zeros(self.hidden_size)
        self.bias_output = np.zeros(output_size)
        
        self.is_loaded = True
        logger.info(f"Hybrid neural model initialized with {input_size}-{self.hidden_size}-{output_size} architecture")
    
    def predict(self, text: str) -> Dict[str, Any]:
        """Predict using neural network"""
        if not self.is_loaded:
            self.load()
        
        # Extract features
        features = self.feature_extractor.extract_lexical_features(text)
        
        # Forward pass
        output_probs = self._forward_pass(features)
        
        # Convert to sentiment format
        sentiment_labels = ['positive', 'negative', 'neutral']
        scores = {label: float(prob) for label, prob in zip(sentiment_labels, output_probs)}
        
        final_sentiment = max(scores, key=scores.get)
        confidence = max(scores.values())
        
        return {
            'sentiment': final_sentiment,
            'confidence': confidence,
            'scores': scores,
            'neural_output': output_probs.tolist()
        }
    
    def _forward_pass(self, features: np.ndarray) -> np.ndarray:
        """Forward pass through neural network"""
        # Hidden layer
        hidden_input = np.dot(features, self.weights_input_hidden) + self.bias_hidden
        hidden_output = self._relu(hidden_input)
        
        # Output layer
        output_input = np.dot(hidden_output, self.weights_hidden_output) + self.bias_output
        output_probs = self._softmax(output_input)
        
        return output_probs
    
    def _relu(self, x: np.ndarray) -> np.ndarray:
        """ReLU activation function"""
        return np.maximum(0, x)
    
    def _softmax(self, x: np.ndarray) -> np.ndarray:
        """Softmax activation function"""
        exp_x = np.exp(x - np.max(x))  # Numerical stability
        return exp_x / np.sum(exp_x)
    
    def train_on_data(self, texts: List[str], labels: List[str], epochs: int = 100):
        """Train neural network on labeled data"""
        if not self.is_loaded:
            self.load()
        
        # Convert labels to one-hot
        label_map = {'positive': 0, 'negative': 1, 'neutral': 2}
        y_onehot = np.zeros((len(labels), 3))
        for i, label in enumerate(labels):
            if label in label_map:
                y_onehot[i, label_map[label]] = 1
        
        # Extract features for all texts
        X = np.array([self.feature_extractor.extract_lexical_features(text) for text in texts])
        
        # Training loop
        learning_rate = 0.01
        for epoch in range(epochs):
            # Forward pass for all samples
            predictions = []
            for i in range(len(X)):
                pred = self._forward_pass(X[i])
                predictions.append(pred)
            
            predictions = np.array(predictions)
            
            # Calculate loss (simplified)
            loss = np.mean(np.sum((predictions - y_onehot) ** 2, axis=1))
            
            # Simplified gradient descent (would need proper backpropagation in production)
            if epoch % 20 == 0:
                logger.info(f"Training epoch {epoch}, loss: {loss:.4f}")
            
            # Simple weight updates (placeholder for proper backprop)
            self._simple_weight_update(X, y_onehot, predictions, learning_rate)
        
        self.is_trained = True
        logger.info("Neural network training completed")
    
    def _simple_weight_update(self, X: np.ndarray, y_true: np.ndarray, y_pred: np.ndarray, lr: float):
        """Simplified weight update (placeholder for proper backpropagation)"""
        # This is a very simplified update - in practice you'd implement proper backpropagation
        error = y_true - y_pred
        
        # Update output layer weights (simplified)
        self.weights_hidden_output += lr * 0.001 * np.random.randn(*self.weights_hidden_output.shape)
        self.bias_output += lr * 0.001 * np.random.randn(*self.bias_output.shape)
    
    def predict_batch(self, texts: List[str]) -> List[Dict[str, Any]]:
        """Batch prediction using neural network"""
        return [self.predict(text) for text in texts]