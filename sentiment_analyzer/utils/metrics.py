"""
Metrics and evaluation utilities for sentiment analysis
"""

from typing import List, Dict, Tuple, Optional
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
import logging

from ..core.analyzer import SentimentResult, SentimentLabel

logger = logging.getLogger(__name__)


@dataclass
class ClassificationMetrics:
    """Classification metrics for sentiment analysis evaluation"""
    accuracy: float
    precision: Dict[str, float]
    recall: Dict[str, float]
    f1_score: Dict[str, float]
    macro_f1: float
    weighted_f1: float
    confusion_matrix: Dict[Tuple[str, str], int]
    support: Dict[str, int]


@dataclass
class PerformanceMetrics:
    """Performance metrics for sentiment analysis"""
    total_texts: int
    total_time: float
    avg_processing_time: float
    throughput_per_second: float
    memory_usage: Optional[float] = None


class SentimentMetrics:
    """
    Comprehensive metrics calculation for sentiment analysis results
    """
    
    def __init__(self):
        self.results_history = []
        logger.info("SentimentMetrics initialized")
    
    def add_results(self, results: List[SentimentResult]):
        """Add results to metrics tracking"""
        self.results_history.extend(results)
        logger.debug(f"Added {len(results)} results to metrics tracking")
    
    def calculate_classification_metrics(
        self,
        predictions: List[SentimentLabel],
        ground_truth: List[SentimentLabel]
    ) -> ClassificationMetrics:
        """
        Calculate comprehensive classification metrics
        
        Args:
            predictions: List of predicted sentiment labels
            ground_truth: List of true sentiment labels
            
        Returns:
            ClassificationMetrics object with all metrics
        """
        if len(predictions) != len(ground_truth):
            raise ValueError("Predictions and ground truth must have same length")
        
        if not predictions:
            raise ValueError("Cannot calculate metrics for empty data")
        
        # Convert enums to strings for easier processing
        pred_strings = [p.value for p in predictions]
        true_strings = [t.value for t in ground_truth]
        
        # Get unique labels
        labels = sorted(set(pred_strings + true_strings))
        
        # Calculate confusion matrix
        confusion_matrix = self._calculate_confusion_matrix(pred_strings, true_strings, labels)
        
        # Calculate per-class metrics
        precision = {}
        recall = {}
        f1_score = {}
        support = {}
        
        for label in labels:
            tp = confusion_matrix.get((label, label), 0)
            fp = sum(confusion_matrix.get((label, true_label), 0) 
                    for true_label in labels if true_label != label)
            fn = sum(confusion_matrix.get((pred_label, label), 0) 
                    for pred_label in labels if pred_label != label)
            tn = len(predictions) - tp - fp - fn
            
            # Precision = TP / (TP + FP)
            precision[label] = tp / (tp + fp) if (tp + fp) > 0 else 0.0
            
            # Recall = TP / (TP + FN)
            recall[label] = tp / (tp + fn) if (tp + fn) > 0 else 0.0
            
            # F1 Score = 2 * (Precision * Recall) / (Precision + Recall)
            if precision[label] + recall[label] > 0:
                f1_score[label] = 2 * (precision[label] * recall[label]) / (precision[label] + recall[label])
            else:
                f1_score[label] = 0.0
            
            # Support (actual occurrences)
            support[label] = true_strings.count(label)
        
        # Calculate overall accuracy
        correct_predictions = sum(1 for p, t in zip(pred_strings, true_strings) if p == t)
        accuracy = correct_predictions / len(predictions)
        
        # Calculate macro F1 (unweighted average)
        macro_f1 = sum(f1_score.values()) / len(labels)
        
        # Calculate weighted F1
        total_support = sum(support.values())
        weighted_f1 = sum(f1_score[label] * support[label] for label in labels) / total_support
        
        return ClassificationMetrics(
            accuracy=accuracy,
            precision=precision,
            recall=recall,
            f1_score=f1_score,
            macro_f1=macro_f1,
            weighted_f1=weighted_f1,
            confusion_matrix=confusion_matrix,
            support=support
        )
    
    def _calculate_confusion_matrix(
        self,
        predictions: List[str],
        ground_truth: List[str],
        labels: List[str]
    ) -> Dict[Tuple[str, str], int]:
        """Calculate confusion matrix"""
        confusion_matrix = defaultdict(int)
        
        for pred, true in zip(predictions, ground_truth):
            confusion_matrix[(pred, true)] += 1
        
        return dict(confusion_matrix)
    
    def calculate_performance_metrics(self, results: List[SentimentResult]) -> PerformanceMetrics:
        """Calculate performance metrics from results"""
        if not results:
            raise ValueError("Cannot calculate performance metrics for empty results")
        
        total_texts = len(results)
        total_time = sum(result.processing_time for result in results)
        avg_processing_time = total_time / total_texts if total_texts > 0 else 0.0
        throughput_per_second = total_texts / total_time if total_time > 0 else 0.0
        
        return PerformanceMetrics(
            total_texts=total_texts,
            total_time=total_time,
            avg_processing_time=avg_processing_time,
            throughput_per_second=throughput_per_second
        )
    
    def calculate_confidence_distribution(self, results: List[SentimentResult]) -> Dict[str, float]:
        """Calculate confidence score distribution statistics"""
        if not results:
            return {}
        
        confidences = [result.confidence for result in results]
        
        return {
            'mean': sum(confidences) / len(confidences),
            'median': sorted(confidences)[len(confidences) // 2],
            'min': min(confidences),
            'max': max(confidences),
            'std': self._calculate_std(confidences),
            'count': len(confidences)
        }
    
    def _calculate_std(self, values: List[float]) -> float:
        """Calculate standard deviation"""
        if len(values) < 2:
            return 0.0
        
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return math.sqrt(variance)
    
    def calculate_sentiment_distribution(self, results: List[SentimentResult]) -> Dict[str, Dict[str, float]]:
        """Calculate sentiment label distribution"""
        if not results:
            return {}
        
        sentiment_counts = Counter(result.sentiment.value for result in results)
        total = len(results)
        
        distribution = {
            'counts': dict(sentiment_counts),
            'percentages': {sentiment: count / total * 100 
                          for sentiment, count in sentiment_counts.items()}
        }
        
        return distribution
    
    def generate_summary_report(self, results: List[SentimentResult]) -> Dict[str, any]:
        """Generate comprehensive summary report"""
        if not results:
            return {'error': 'No results to analyze'}
        
        performance_metrics = self.calculate_performance_metrics(results)
        confidence_dist = self.calculate_confidence_distribution(results)
        sentiment_dist = self.calculate_sentiment_distribution(results)
        
        # Model usage statistics
        model_usage = Counter(result.model_used for result in results)
        
        # Error analysis
        error_count = sum(1 for result in results 
                         if result.metadata and 'error' in result.metadata)
        
        # Quality indicators
        low_confidence_count = sum(1 for result in results if result.confidence < 0.6)
        high_confidence_count = sum(1 for result in results if result.confidence > 0.8)
        
        report = {
            'summary': {
                'total_texts_analyzed': len(results),
                'total_processing_time': performance_metrics.total_time,
                'average_processing_time': performance_metrics.avg_processing_time,
                'throughput_per_second': performance_metrics.throughput_per_second
            },
            'sentiment_distribution': sentiment_dist,
            'confidence_metrics': confidence_dist,
            'quality_indicators': {
                'low_confidence_predictions': low_confidence_count,
                'high_confidence_predictions': high_confidence_count,
                'error_count': error_count,
                'success_rate': (len(results) - error_count) / len(results) * 100
            },
            'model_usage': dict(model_usage),
            'timestamp': time.time() if 'time' in dir() else None
        }
        
        return report
    
    def export_metrics(self, results: List[SentimentResult], format: str = 'json') -> str:
        """Export metrics in specified format"""
        report = self.generate_summary_report(results)
        
        if format.lower() == 'json':
            import json
            return json.dumps(report, indent=2, default=str)
        elif format.lower() == 'csv':
            return self._export_to_csv(results)
        else:
            raise ValueError(f"Unsupported export format: {format}")
    
    def _export_to_csv(self, results: List[SentimentResult]) -> str:
        """Export results to CSV format"""
        import csv
        from io import StringIO
        
        output = StringIO()
        writer = csv.writer(output)
        
        # Write header
        writer.writerow(['text', 'sentiment', 'confidence', 'positive_score', 
                        'negative_score', 'neutral_score', 'model_used', 
                        'processing_time', 'text_length'])
        
        # Write data
        for result in results:
            writer.writerow([
                result.text,
                result.sentiment.value,
                result.confidence,
                result.scores.get('positive', 0),
                result.scores.get('negative', 0),
                result.scores.get('neutral', 0),
                result.model_used,
                result.processing_time,
                len(result.text)
            ])
        
        return output.getvalue()
    
    def benchmark_models(self, texts: List[str], models: List[str]) -> Dict[str, Dict[str, float]]:
        """Benchmark multiple models on the same texts"""
        from ..core.analyzer import SentimentAnalyzer
        import time
        
        results = {}
        
        for model_name in models:
            logger.info(f"Benchmarking model: {model_name}")
            
            try:
                analyzer = SentimentAnalyzer(model_name)
                start_time = time.time()
                
                model_results = analyzer.analyze_batch(texts)
                
                end_time = time.time()
                
                # Calculate metrics
                performance_metrics = self.calculate_performance_metrics(model_results)
                confidence_dist = self.calculate_confidence_distribution(model_results)
                
                results[model_name] = {
                    'total_time': end_time - start_time,
                    'avg_processing_time': performance_metrics.avg_processing_time,
                    'throughput': performance_metrics.throughput_per_second,
                    'avg_confidence': confidence_dist.get('mean', 0),
                    'confidence_std': confidence_dist.get('std', 0),
                    'success_rate': len([r for r in model_results if not (r.metadata and 'error' in r.metadata)]) / len(texts)
                }
                
            except Exception as e:
                logger.error(f"Error benchmarking model {model_name}: {e}")
                results[model_name] = {'error': str(e)}
        
        return results