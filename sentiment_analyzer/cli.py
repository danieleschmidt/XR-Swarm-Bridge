"""
Command-line interface for Sentiment Analyzer Pro
"""

import argparse
import sys
import json
import logging
from typing import List, Optional
from pathlib import Path

from .core.analyzer import SentimentAnalyzer
from .core.pipeline import AnalysisPipeline, PipelineConfig
from .utils.metrics import SentimentMetrics


def setup_logging(verbose: bool = False):
    """Setup logging configuration"""
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(sys.stdout)
        ]
    )


def analyze_text(args):
    """Analyze single text input"""
    analyzer = SentimentAnalyzer(args.model)
    result = analyzer.analyze(args.text)
    
    output = {
        'text': result.text,
        'sentiment': result.sentiment.value,
        'confidence': round(result.confidence, 3),
        'scores': {k: round(v, 3) for k, v in result.scores.items()},
        'model': result.model_used,
        'processing_time': round(result.processing_time, 4)
    }
    
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(output, f, indent=2)
    else:
        print(json.dumps(output, indent=2))


def analyze_file(args):
    """Analyze texts from file"""
    if not Path(args.file).exists():
        print(f"Error: File '{args.file}' not found", file=sys.stderr)
        sys.exit(1)
    
    # Read texts from file
    with open(args.file, 'r', encoding='utf-8') as f:
        if args.file.endswith('.json'):
            data = json.load(f)
            if isinstance(data, list):
                texts = data
            elif isinstance(data, dict) and 'texts' in data:
                texts = data['texts']
            else:
                texts = [str(data)]
        else:
            texts = [line.strip() for line in f.readlines() if line.strip()]
    
    # Setup pipeline
    config = PipelineConfig(
        model_type=args.model,
        batch_size=args.batch_size or 100
    )
    pipeline = AnalysisPipeline(config)
    
    # Process texts
    results = pipeline.process_batch(texts)
    
    # Format output
    output_data = []
    for result in results:
        output_data.append({
            'text': result.text,
            'sentiment': result.sentiment.value,
            'confidence': round(result.confidence, 3),
            'scores': {k: round(v, 3) for k, v in result.scores.items()},
            'model': result.model_used,
            'processing_time': round(result.processing_time, 4)
        })
    
    # Generate summary if requested
    if args.summary:
        metrics = SentimentMetrics()
        summary = metrics.generate_summary_report(results)
        output_data = {
            'results': output_data,
            'summary': summary
        }
    
    # Save or print results
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(output_data, f, indent=2)
    else:
        print(json.dumps(output_data, indent=2))


def benchmark_models(args):
    """Benchmark different models"""
    # Prepare test texts
    test_texts = [
        "I love this product, it's amazing!",
        "This is terrible, worst experience ever.",
        "It's okay, nothing special.",
        "Absolutely fantastic quality and service!",
        "Not good, disappointed with the results.",
        "Average performance, meets expectations.",
        "Outstanding work, highly recommended!",
        "Poor quality, needs improvement.",
        "Satisfactory results, could be better.",
        "Excellent value for money!"
    ]
    
    if args.file:
        with open(args.file, 'r') as f:
            if args.file.endswith('.json'):
                data = json.load(f)
                test_texts = data if isinstance(data, list) else data.get('texts', test_texts)
            else:
                test_texts = [line.strip() for line in f.readlines() if line.strip()]
    
    models_to_test = args.models if args.models else ['lexicon', 'rule_based']
    
    metrics = SentimentMetrics()
    benchmark_results = metrics.benchmark_models(test_texts, models_to_test)
    
    if args.output:
        with open(args.output, 'w') as f:
            json.dump(benchmark_results, f, indent=2)
    else:
        print(json.dumps(benchmark_results, indent=2))


def main():
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="Sentiment Analyzer Pro - Advanced sentiment analysis CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  sentiment-analyzer analyze "I love this product!" 
  sentiment-analyzer analyze-file texts.txt --model rule_based --summary
  sentiment-analyzer benchmark --models lexicon rule_based --file test_data.json
        """
    )
    
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Enable verbose logging')
    parser.add_argument('--model', '-m', default='lexicon',
                       choices=['lexicon', 'rule_based'],
                       help='Sentiment analysis model to use')
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Analyze single text command
    analyze_parser = subparsers.add_parser('analyze', 
                                          help='Analyze a single text')
    analyze_parser.add_argument('text', help='Text to analyze')
    analyze_parser.add_argument('--output', '-o', 
                               help='Output file (default: stdout)')
    
    # Analyze file command
    file_parser = subparsers.add_parser('analyze-file',
                                       help='Analyze texts from file')
    file_parser.add_argument('file', help='Input file (text or JSON)')
    file_parser.add_argument('--output', '-o',
                            help='Output file (default: stdout)')
    file_parser.add_argument('--batch-size', type=int,
                            help='Batch size for processing')
    file_parser.add_argument('--summary', action='store_true',
                            help='Include summary statistics')
    
    # Benchmark command
    benchmark_parser = subparsers.add_parser('benchmark',
                                           help='Benchmark different models')
    benchmark_parser.add_argument('--models', nargs='+',
                                 choices=['lexicon', 'rule_based'],
                                 help='Models to benchmark')
    benchmark_parser.add_argument('--file',
                                 help='Test data file')
    benchmark_parser.add_argument('--output', '-o',
                                 help='Output file for benchmark results')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        sys.exit(1)
    
    setup_logging(args.verbose)
    
    try:
        if args.command == 'analyze':
            analyze_text(args)
        elif args.command == 'analyze-file':
            analyze_file(args)
        elif args.command == 'benchmark':
            benchmark_models(args)
        else:
            parser.print_help()
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\nOperation cancelled by user", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        if args.verbose:
            import traceback
            traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()