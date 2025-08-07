"""
Comprehensive benchmarking suite for sentiment analysis research
"""

import time
import statistics
import random
import json
from typing import List, Dict, Tuple, Optional, Any, Callable
from dataclasses import dataclass, asdict
from collections import defaultdict
import logging
import numpy as np

from ..core.analyzer import SentimentAnalyzer, SentimentLabel, SentimentResult
from ..core.models import BaseSentimentModel, ModelRegistry
from ..utils.metrics import SentimentMetrics, ClassificationMetrics
from .algorithms import EnsembleModel, AdaptiveSentimentModel, HybridNeuralSentimentModel

logger = logging.getLogger(__name__)


@dataclass
class BenchmarkDataset:
    """Container for benchmark dataset"""
    name: str
    texts: List[str]
    labels: List[str]
    metadata: Optional[Dict[str, Any]] = None
    
    def __post_init__(self):
        if len(self.texts) != len(self.labels):
            raise ValueError("Texts and labels must have same length")


@dataclass  
class BenchmarkResult:
    """Results from a single benchmark run"""
    model_name: str
    dataset_name: str
    accuracy: float
    precision: Dict[str, float]
    recall: Dict[str, float]
    f1_score: Dict[str, float]
    macro_f1: float
    weighted_f1: float
    processing_time: float
    throughput: float
    memory_usage: Optional[float] = None
    error_rate: float = 0.0
    confusion_matrix: Optional[Dict] = None


class DatasetGenerator:
    """Generate synthetic datasets for benchmarking"""
    
    def __init__(self, random_seed: int = 42):
        random.seed(random_seed)
        self.positive_templates = [
            "I love this {item}! It's {adjective}!",
            "This {item} is {adjective} and {adjective2}.",
            "Amazing {item}! Highly recommend it.",
            "Great {item}, very {adjective}.",
            "Excellent {item}! {adjective} quality.",
            "Wonderful {item}, I'm so {adjective}!",
            "Best {item} ever! {adjective} and {adjective2}.",
            "This {item} makes me feel {adjective}."
        ]
        
        self.negative_templates = [
            "I hate this {item}. It's {adjective}.",
            "This {item} is {adjective} and {adjective2}.",
            "Terrible {item}! Don't waste your money.",
            "Bad {item}, very {adjective}.",
            "Poor {item}! {adjective} quality.",
            "Awful {item}, I'm so {adjective}.",
            "Worst {item} ever! {adjective} and {adjective2}.",
            "This {item} makes me feel {adjective}."
        ]
        
        self.neutral_templates = [
            "This {item} is okay, nothing special.",
            "The {item} works as expected.",
            "Average {item}, meets basic requirements.",
            "Standard {item} with typical features.",
            "Regular {item}, neither good nor bad.",
            "This {item} is fine for the price.",
            "Acceptable {item}, does what it should.",
            "Normal {item} with standard quality."
        ]
        
        self.items = [
            "product", "service", "experience", "app", "website", "book", 
            "movie", "restaurant", "hotel", "phone", "computer", "software"
        ]
        
        self.positive_adjectives = [
            "amazing", "excellent", "fantastic", "wonderful", "great", "awesome",
            "perfect", "brilliant", "outstanding", "superb", "incredible", "happy"
        ]
        
        self.negative_adjectives = [
            "terrible", "awful", "horrible", "bad", "disappointing", "frustrating",
            "annoying", "useless", "broken", "pathetic", "disgusting", "angry"
        ]
        
        self.neutral_adjectives = [
            "standard", "typical", "normal", "regular", "basic", "average",
            "adequate", "sufficient", "acceptable", "reasonable", "ordinary"
        ]
    
    def generate_balanced_dataset(self, size: int = 1000, name: str = "synthetic") -> BenchmarkDataset:
        """Generate balanced dataset with equal positive/negative/neutral samples"""
        texts = []
        labels = []
        
        samples_per_class = size // 3
        
        # Generate positive samples
        for _ in range(samples_per_class):
            template = random.choice(self.positive_templates)
            text = template.format(
                item=random.choice(self.items),
                adjective=random.choice(self.positive_adjectives),
                adjective2=random.choice(self.positive_adjectives)
            )
            texts.append(text)
            labels.append("positive")
        
        # Generate negative samples  
        for _ in range(samples_per_class):
            template = random.choice(self.negative_templates)
            text = template.format(
                item=random.choice(self.items),
                adjective=random.choice(self.negative_adjectives),
                adjective2=random.choice(self.negative_adjectives)
            )
            texts.append(text)
            labels.append("negative")
        
        # Generate neutral samples
        for _ in range(samples_per_class):
            template = random.choice(self.neutral_templates)
            text = template.format(
                item=random.choice(self.items),
                adjective=random.choice(self.neutral_adjectives),
                adjective2=random.choice(self.neutral_adjectives)
            )
            texts.append(text)
            labels.append("neutral")
        
        # Add remaining samples if size not divisible by 3
        remaining = size - len(texts)
        for _ in range(remaining):
            sentiment = random.choice(["positive", "negative", "neutral"])
            if sentiment == "positive":
                template = random.choice(self.positive_templates)
                adjectives = self.positive_adjectives
            elif sentiment == "negative":
                template = random.choice(self.negative_templates)
                adjectives = self.negative_adjectives
            else:
                template = random.choice(self.neutral_templates)
                adjectives = self.neutral_adjectives
            
            text = template.format(
                item=random.choice(self.items),
                adjective=random.choice(adjectives),
                adjective2=random.choice(adjectives)
            )
            texts.append(text)
            labels.append(sentiment)
        
        # Shuffle the dataset
        combined = list(zip(texts, labels))
        random.shuffle(combined)
        texts, labels = zip(*combined)
        
        return BenchmarkDataset(
            name=name,
            texts=list(texts),
            labels=list(labels),
            metadata={
                'generated': True,
                'size': len(texts),
                'class_distribution': {
                    'positive': labels.count('positive'),
                    'negative': labels.count('negative'), 
                    'neutral': labels.count('neutral')
                }
            }
        )
    
    def generate_imbalanced_dataset(self, size: int = 1000, 
                                  positive_ratio: float = 0.6,
                                  negative_ratio: float = 0.3,
                                  neutral_ratio: float = 0.1,
                                  name: str = "imbalanced") -> BenchmarkDataset:
        """Generate imbalanced dataset with specified class ratios"""
        
        if abs(positive_ratio + negative_ratio + neutral_ratio - 1.0) > 0.01:
            raise ValueError("Class ratios must sum to 1.0")
        
        texts = []
        labels = []
        
        # Calculate sample counts
        positive_count = int(size * positive_ratio)
        negative_count = int(size * negative_ratio)
        neutral_count = size - positive_count - negative_count
        
        # Generate samples for each class
        class_configs = [
            (positive_count, "positive", self.positive_templates, self.positive_adjectives),
            (negative_count, "negative", self.negative_templates, self.negative_adjectives),
            (neutral_count, "neutral", self.neutral_templates, self.neutral_adjectives)
        ]
        
        for count, sentiment, templates, adjectives in class_configs:
            for _ in range(count):
                template = random.choice(templates)
                text = template.format(
                    item=random.choice(self.items),
                    adjective=random.choice(adjectives),
                    adjective2=random.choice(adjectives)
                )
                texts.append(text)
                labels.append(sentiment)
        
        # Shuffle
        combined = list(zip(texts, labels))
        random.shuffle(combined)
        texts, labels = zip(*combined)
        
        return BenchmarkDataset(
            name=name,
            texts=list(texts),
            labels=list(labels),
            metadata={
                'generated': True,
                'imbalanced': True,
                'size': len(texts),
                'ratios': {
                    'positive': positive_ratio,
                    'negative': negative_ratio,
                    'neutral': neutral_ratio
                },
                'actual_distribution': {
                    'positive': labels.count('positive'),
                    'negative': labels.count('negative'),
                    'neutral': labels.count('neutral')
                }
            }
        )


class ComparativeBenchmark:
    """Comprehensive benchmarking system for comparing sentiment analysis models"""
    
    def __init__(self):
        self.datasets = {}
        self.models = {}
        self.results = []
        self.dataset_generator = DatasetGenerator()
        self.metrics_calculator = SentimentMetrics()
        
        logger.info("ComparativeBenchmark initialized")
    
    def add_dataset(self, dataset: BenchmarkDataset):
        """Add dataset to benchmark suite"""
        self.datasets[dataset.name] = dataset
        logger.info(f"Added dataset '{dataset.name}' with {len(dataset.texts)} samples")
    
    def add_model(self, name: str, model: BaseSentimentModel):
        """Add model to benchmark"""
        self.models[name] = model
        logger.info(f"Added model '{name}' for benchmarking")
    
    def run_single_benchmark(self, model_name: str, dataset_name: str, 
                           warmup_runs: int = 3) -> BenchmarkResult:
        """Run benchmark for single model-dataset combination"""
        
        if model_name not in self.models:
            raise ValueError(f"Model '{model_name}' not found")
        if dataset_name not in self.datasets:
            raise ValueError(f"Dataset '{dataset_name}' not found")
        
        model = self.models[model_name]
        dataset = self.datasets[dataset_name]
        
        logger.info(f"Running benchmark: {model_name} on {dataset_name}")
        
        # Warmup runs
        if warmup_runs > 0:
            warmup_texts = dataset.texts[:min(10, len(dataset.texts))]
            for _ in range(warmup_runs):
                for text in warmup_texts:
                    try:
                        model.predict(text)
                    except Exception:
                        pass  # Ignore warmup errors
        
        # Benchmark run
        start_time = time.time()
        predictions = []
        errors = 0
        
        try:
            for text in dataset.texts:
                try:
                    pred = model.predict(text)
                    predictions.append(pred['sentiment'])
                except Exception as e:
                    logger.warning(f"Prediction error: {e}")
                    predictions.append('neutral')  # Default fallback
                    errors += 1
            
            end_time = time.time()
            processing_time = end_time - start_time
            throughput = len(dataset.texts) / processing_time
            
        except Exception as e:
            logger.error(f"Benchmark failed for {model_name}: {e}")
            raise
        
        # Calculate metrics
        sentiment_labels = [SentimentLabel(label) for label in dataset.labels]
        prediction_labels = [SentimentLabel(pred) for pred in predictions]
        
        classification_metrics = self.metrics_calculator.calculate_classification_metrics(
            prediction_labels, sentiment_labels
        )
        
        # Create result
        result = BenchmarkResult(
            model_name=model_name,
            dataset_name=dataset_name,
            accuracy=classification_metrics.accuracy,
            precision=classification_metrics.precision,
            recall=classification_metrics.recall,
            f1_score=classification_metrics.f1_score,
            macro_f1=classification_metrics.macro_f1,
            weighted_f1=classification_metrics.weighted_f1,
            processing_time=processing_time,
            throughput=throughput,
            error_rate=errors / len(dataset.texts),
            confusion_matrix=classification_metrics.confusion_matrix
        )
        
        self.results.append(result)
        logger.info(f"Benchmark completed: {model_name} achieved {result.accuracy:.3f} accuracy")
        
        return result
    
    def run_comprehensive_benchmark(self, models: Optional[List[str]] = None,
                                  datasets: Optional[List[str]] = None) -> List[BenchmarkResult]:
        """Run comprehensive benchmark across multiple models and datasets"""
        
        models_to_test = models or list(self.models.keys())
        datasets_to_test = datasets or list(self.datasets.keys())
        
        logger.info(f"Starting comprehensive benchmark: {len(models_to_test)} models x {len(datasets_to_test)} datasets")
        
        results = []
        total_combinations = len(models_to_test) * len(datasets_to_test)
        completed = 0
        
        for model_name in models_to_test:
            for dataset_name in datasets_to_test:
                try:
                    result = self.run_single_benchmark(model_name, dataset_name)
                    results.append(result)
                    completed += 1
                    logger.info(f"Progress: {completed}/{total_combinations} combinations completed")
                    
                except Exception as e:
                    logger.error(f"Failed benchmark {model_name} on {dataset_name}: {e}")
                    completed += 1
        
        logger.info(f"Comprehensive benchmark completed: {len(results)} successful runs")
        return results
    
    def generate_benchmark_report(self, results: Optional[List[BenchmarkResult]] = None) -> Dict[str, Any]:
        """Generate comprehensive benchmark report"""
        
        results_to_analyze = results or self.results
        
        if not results_to_analyze:
            return {"error": "No benchmark results available"}
        
        # Organize results by model and dataset
        results_by_model = defaultdict(list)
        results_by_dataset = defaultdict(list)
        
        for result in results_to_analyze:
            results_by_model[result.model_name].append(result)
            results_by_dataset[result.dataset_name].append(result)
        
        # Calculate summary statistics
        model_summaries = {}
        for model_name, model_results in results_by_model.items():
            accuracies = [r.accuracy for r in model_results]
            f1_scores = [r.macro_f1 for r in model_results]
            throughputs = [r.throughput for r in model_results]
            
            model_summaries[model_name] = {
                'num_datasets': len(model_results),
                'avg_accuracy': statistics.mean(accuracies),
                'std_accuracy': statistics.stdev(accuracies) if len(accuracies) > 1 else 0.0,
                'min_accuracy': min(accuracies),
                'max_accuracy': max(accuracies),
                'avg_f1': statistics.mean(f1_scores),
                'avg_throughput': statistics.mean(throughputs),
                'total_processing_time': sum(r.processing_time for r in model_results)
            }
        
        # Dataset difficulty analysis
        dataset_summaries = {}
        for dataset_name, dataset_results in results_by_dataset.items():
            accuracies = [r.accuracy for r in dataset_results]
            
            dataset_summaries[dataset_name] = {
                'num_models': len(dataset_results),
                'avg_accuracy': statistics.mean(accuracies),
                'difficulty_score': 1.0 - statistics.mean(accuracies),  # Higher = more difficult
                'best_model': max(dataset_results, key=lambda r: r.accuracy).model_name,
                'worst_model': min(dataset_results, key=lambda r: r.accuracy).model_name
            }
        
        # Overall rankings
        model_rankings = sorted(model_summaries.items(), 
                              key=lambda x: x[1]['avg_accuracy'], 
                              reverse=True)
        
        dataset_difficulty = sorted(dataset_summaries.items(),
                                  key=lambda x: x[1]['difficulty_score'],
                                  reverse=True)
        
        # Statistical significance tests (simplified)
        pairwise_comparisons = self._pairwise_model_comparison(results_by_model)
        
        report = {
            'summary': {
                'total_benchmarks': len(results_to_analyze),
                'num_models': len(results_by_model),
                'num_datasets': len(results_by_dataset),
                'avg_accuracy_overall': statistics.mean([r.accuracy for r in results_to_analyze]),
                'best_overall_result': max(results_to_analyze, key=lambda r: r.accuracy),
                'fastest_model': max(results_to_analyze, key=lambda r: r.throughput).model_name
            },
            'model_summaries': model_summaries,
            'dataset_summaries': dataset_summaries,
            'model_rankings': [{'model': name, **stats} for name, stats in model_rankings],
            'dataset_difficulty': [{'dataset': name, **stats} for name, stats in dataset_difficulty],
            'pairwise_comparisons': pairwise_comparisons,
            'detailed_results': [asdict(r) for r in results_to_analyze]
        }
        
        return report
    
    def _pairwise_model_comparison(self, results_by_model: Dict) -> Dict[str, Any]:
        """Perform pairwise comparison between models"""
        comparisons = {}
        
        model_names = list(results_by_model.keys())
        
        for i, model1 in enumerate(model_names):
            for model2 in model_names[i+1:]:
                # Find common datasets
                datasets1 = {r.dataset_name for r in results_by_model[model1]}
                datasets2 = {r.dataset_name for r in results_by_model[model2]}
                common_datasets = datasets1.intersection(datasets2)
                
                if not common_datasets:
                    continue
                
                # Get results for common datasets
                results1 = [r for r in results_by_model[model1] if r.dataset_name in common_datasets]
                results2 = [r for r in results_by_model[model2] if r.dataset_name in common_datasets]
                
                # Align results by dataset
                results1_dict = {r.dataset_name: r for r in results1}
                results2_dict = {r.dataset_name: r for r in results2}
                
                # Calculate differences
                accuracy_diffs = []
                throughput_diffs = []
                
                for dataset in common_datasets:
                    if dataset in results1_dict and dataset in results2_dict:
                        acc_diff = results1_dict[dataset].accuracy - results2_dict[dataset].accuracy
                        throughput_diff = results1_dict[dataset].throughput - results2_dict[dataset].throughput
                        accuracy_diffs.append(acc_diff)
                        throughput_diffs.append(throughput_diff)
                
                if accuracy_diffs:
                    comparison_key = f"{model1}_vs_{model2}"
                    comparisons[comparison_key] = {
                        'common_datasets': len(common_datasets),
                        'avg_accuracy_difference': statistics.mean(accuracy_diffs),
                        'accuracy_wins_model1': sum(1 for d in accuracy_diffs if d > 0),
                        'accuracy_wins_model2': sum(1 for d in accuracy_diffs if d < 0),
                        'avg_throughput_difference': statistics.mean(throughput_diffs),
                        'statistical_significance': 'placeholder'  # Would implement proper t-test
                    }
        
        return comparisons
    
    def export_results(self, filename: str, format: str = 'json'):
        """Export benchmark results to file"""
        
        report = self.generate_benchmark_report()
        
        if format.lower() == 'json':
            # Handle non-serializable objects
            def json_serializer(obj):
                if hasattr(obj, '__dict__'):
                    return obj.__dict__
                return str(obj)
            
            with open(filename, 'w') as f:
                json.dump(report, f, indent=2, default=json_serializer)
                
        elif format.lower() == 'csv':
            import csv
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                
                # Write header
                writer.writerow([
                    'Model', 'Dataset', 'Accuracy', 'Macro_F1', 'Weighted_F1',
                    'Processing_Time', 'Throughput', 'Error_Rate'
                ])
                
                # Write results
                for result in self.results:
                    writer.writerow([
                        result.model_name,
                        result.dataset_name,
                        result.accuracy,
                        result.macro_f1,
                        result.weighted_f1,
                        result.processing_time,
                        result.throughput,
                        result.error_rate
                    ])
        
        logger.info(f"Results exported to {filename}")
    
    def create_standard_benchmark_suite(self):
        """Create a standard benchmark suite with various datasets"""
        
        # Generate different types of datasets
        datasets = [
            self.dataset_generator.generate_balanced_dataset(1000, "balanced_1k"),
            self.dataset_generator.generate_balanced_dataset(5000, "balanced_5k"),
            self.dataset_generator.generate_imbalanced_dataset(
                1000, 0.7, 0.2, 0.1, "positive_heavy"
            ),
            self.dataset_generator.generate_imbalanced_dataset(
                1000, 0.1, 0.8, 0.1, "negative_heavy"
            ),
            self.dataset_generator.generate_imbalanced_dataset(
                1000, 0.2, 0.2, 0.6, "neutral_heavy"
            )
        ]
        
        for dataset in datasets:
            self.add_dataset(dataset)
        
        logger.info(f"Created standard benchmark suite with {len(datasets)} datasets")