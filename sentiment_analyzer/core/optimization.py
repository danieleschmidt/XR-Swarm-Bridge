"""
Performance optimization and scaling features for sentiment analysis
"""

import time
import threading
import asyncio
import multiprocessing
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor, as_completed
from typing import List, Dict, Any, Optional, Callable, Iterator, Tuple
from dataclasses import dataclass
import pickle
import hashlib
import logging
from collections import OrderedDict, deque
import weakref
import gc

logger = logging.getLogger(__name__)


@dataclass
class PerformanceConfig:
    """Configuration for performance optimizations"""
    enable_caching: bool = True
    cache_size: int = 10000
    enable_parallel_processing: bool = True
    max_workers: int = None  # None = auto-detect
    batch_size: int = 100
    enable_prefetching: bool = True
    prefetch_size: int = 1000
    enable_memory_optimization: bool = True
    gc_threshold: int = 1000


class LRUCache:
    """Thread-safe LRU cache implementation with TTL support"""
    
    def __init__(self, max_size: int = 1000, ttl_seconds: float = 3600):
        self.max_size = max_size
        self.ttl_seconds = ttl_seconds
        self.cache = OrderedDict()
        self.timestamps = {}
        self.lock = threading.RLock()
        self.hits = 0
        self.misses = 0
    
    def get(self, key: str) -> Optional[Any]:
        """Get value from cache"""
        with self.lock:
            current_time = time.time()
            
            if key not in self.cache:
                self.misses += 1
                return None
            
            # Check TTL
            if (key in self.timestamps and 
                current_time - self.timestamps[key] > self.ttl_seconds):
                del self.cache[key]
                del self.timestamps[key]
                self.misses += 1
                return None
            
            # Move to end (most recently used)
            value = self.cache[key]
            del self.cache[key]
            self.cache[key] = value
            self.timestamps[key] = current_time
            self.hits += 1
            return value
    
    def put(self, key: str, value: Any):
        """Put value in cache"""
        with self.lock:
            current_time = time.time()
            
            if key in self.cache:
                # Update existing
                del self.cache[key]
                self.cache[key] = value
                self.timestamps[key] = current_time
                return
            
            # Add new
            if len(self.cache) >= self.max_size:
                # Remove least recently used
                oldest_key, _ = self.cache.popitem(last=False)
                del self.timestamps[oldest_key]
            
            self.cache[key] = value
            self.timestamps[key] = current_time
    
    def clear(self):
        """Clear all cache entries"""
        with self.lock:
            self.cache.clear()
            self.timestamps.clear()
            self.hits = 0
            self.misses = 0
    
    def get_stats(self) -> Dict[str, Any]:
        """Get cache statistics"""
        with self.lock:
            total_requests = self.hits + self.misses
            hit_rate = self.hits / total_requests if total_requests > 0 else 0.0
            
            return {
                'hits': self.hits,
                'misses': self.misses,
                'hit_rate': hit_rate,
                'size': len(self.cache),
                'max_size': self.max_size
            }


class MemoryPool:
    """Memory pool for reusing objects and reducing GC pressure"""
    
    def __init__(self, factory: Callable, max_size: int = 100):
        self.factory = factory
        self.max_size = max_size
        self.pool = deque()
        self.lock = threading.RLock()
        self.created_count = 0
        self.reused_count = 0
    
    def acquire(self):
        """Acquire object from pool or create new"""
        with self.lock:
            if self.pool:
                self.reused_count += 1
                return self.pool.popleft()
            else:
                self.created_count += 1
                return self.factory()
    
    def release(self, obj):
        """Return object to pool"""
        with self.lock:
            if len(self.pool) < self.max_size:
                # Reset object if it has a reset method
                if hasattr(obj, 'reset'):
                    obj.reset()
                self.pool.append(obj)
    
    def get_stats(self) -> Dict[str, int]:
        """Get pool statistics"""
        with self.lock:
            return {
                'pool_size': len(self.pool),
                'created_count': self.created_count,
                'reused_count': self.reused_count,
                'reuse_rate': self.reused_count / (self.created_count + self.reused_count) if (self.created_count + self.reused_count) > 0 else 0.0
            }


class BatchProcessor:
    """Optimized batch processing for large datasets"""
    
    def __init__(self, config: PerformanceConfig):
        self.config = config
        self.max_workers = config.max_workers or multiprocessing.cpu_count()
        self.cache = LRUCache(config.cache_size) if config.enable_caching else None
        
        # Create memory pools for common objects
        self.result_pool = MemoryPool(lambda: {'scores': {}, 'metadata': {}})
        
        logger.info(f"BatchProcessor initialized with {self.max_workers} workers")
    
    def process_parallel(self, 
                        items: List[Any], 
                        processor_func: Callable,
                        use_processes: bool = False) -> List[Any]:
        """Process items in parallel using threads or processes"""
        
        if len(items) <= self.config.batch_size:
            # Small batch, process sequentially
            return [processor_func(item) for item in items]
        
        results = [None] * len(items)
        
        # Choose executor type
        executor_class = ProcessPoolExecutor if use_processes else ThreadPoolExecutor
        
        with executor_class(max_workers=self.max_workers) as executor:
            # Create batches
            batch_size = max(1, len(items) // self.max_workers)
            futures = {}
            
            for i in range(0, len(items), batch_size):
                batch = items[i:i + batch_size]
                future = executor.submit(self._process_batch, batch, processor_func)
                futures[future] = i
            
            # Collect results
            for future in as_completed(futures):
                start_idx = futures[future]
                try:
                    batch_results = future.result()
                    for j, result in enumerate(batch_results):
                        results[start_idx + j] = result
                except Exception as e:
                    logger.error(f"Batch processing error: {e}")
                    # Fill with default results
                    batch_size = len(items[start_idx:start_idx + self.config.batch_size])
                    for j in range(batch_size):
                        results[start_idx + j] = self._create_error_result(str(e))
        
        return results
    
    def _process_batch(self, batch: List[Any], processor_func: Callable) -> List[Any]:
        """Process a single batch of items"""
        results = []
        for item in batch:
            try:
                # Check cache first
                if self.cache:
                    cache_key = self._get_cache_key(item)
                    cached_result = self.cache.get(cache_key)
                    if cached_result:
                        results.append(cached_result)
                        continue
                
                # Process item
                result = processor_func(item)
                
                # Cache result
                if self.cache:
                    self.cache.put(cache_key, result)
                
                results.append(result)
                
            except Exception as e:
                logger.error(f"Error processing item: {e}")
                results.append(self._create_error_result(str(e)))
        
        return results
    
    def _get_cache_key(self, item: Any) -> str:
        """Generate cache key for item"""
        if isinstance(item, str):
            return hashlib.sha256(item.encode()).hexdigest()[:16]
        else:
            return hashlib.sha256(str(item).encode()).hexdigest()[:16]
    
    def _create_error_result(self, error_message: str) -> Dict[str, Any]:
        """Create error result object"""
        return {
            'error': True,
            'message': error_message,
            'sentiment': 'neutral',
            'confidence': 0.0,
            'scores': {'positive': 0.0, 'negative': 0.0, 'neutral': 1.0}
        }
    
    def process_streaming(self, 
                         item_stream: Iterator[Any], 
                         processor_func: Callable,
                         buffer_size: int = None) -> Iterator[Any]:
        """Process items as a stream with buffering"""
        buffer_size = buffer_size or self.config.batch_size
        buffer = []
        
        for item in item_stream:
            buffer.append(item)
            
            if len(buffer) >= buffer_size:
                # Process buffer
                results = self.process_parallel(buffer, processor_func)
                for result in results:
                    yield result
                buffer.clear()
                
                # Trigger garbage collection periodically
                if self.config.enable_memory_optimization:
                    gc.collect()
        
        # Process remaining items
        if buffer:
            results = self.process_parallel(buffer, processor_func)
            for result in results:
                yield result


class AsyncBatchProcessor:
    """Asynchronous batch processor for high-throughput scenarios"""
    
    def __init__(self, config: PerformanceConfig):
        self.config = config
        self.semaphore = asyncio.Semaphore(config.max_workers or 100)
        self.cache = LRUCache(config.cache_size) if config.enable_caching else None
        
    async def process_async(self, 
                          items: List[Any], 
                          async_processor: Callable) -> List[Any]:
        """Process items asynchronously"""
        
        tasks = []
        for item in items:
            task = asyncio.create_task(self._process_item_async(item, async_processor))
            tasks.append(task)
        
        results = await asyncio.gather(*tasks, return_exceptions=True)
        
        # Handle exceptions
        processed_results = []
        for result in results:
            if isinstance(result, Exception):
                logger.error(f"Async processing error: {result}")
                processed_results.append(self._create_error_result(str(result)))
            else:
                processed_results.append(result)
        
        return processed_results
    
    async def _process_item_async(self, item: Any, async_processor: Callable) -> Any:
        """Process single item asynchronously with semaphore"""
        async with self.semaphore:
            # Check cache
            if self.cache:
                cache_key = hashlib.sha256(str(item).encode()).hexdigest()[:16]
                cached_result = self.cache.get(cache_key)
                if cached_result:
                    return cached_result
            
            # Process item
            result = await async_processor(item)
            
            # Cache result
            if self.cache:
                self.cache.put(cache_key, result)
            
            return result
    
    def _create_error_result(self, error_message: str) -> Dict[str, Any]:
        """Create error result for async processing"""
        return {
            'error': True,
            'message': error_message,
            'sentiment': 'neutral',
            'confidence': 0.0,
            'scores': {'positive': 0.0, 'negative': 0.0, 'neutral': 1.0}
        }


class PerformanceOptimizer:
    """Main performance optimization coordinator"""
    
    def __init__(self, config: Optional[PerformanceConfig] = None):
        self.config = config or PerformanceConfig()
        self.batch_processor = BatchProcessor(self.config)
        self.async_processor = AsyncBatchProcessor(self.config)
        
        # Performance tracking
        self.performance_metrics = {
            'total_processed': 0,
            'total_time': 0.0,
            'average_throughput': 0.0,
            'cache_hits': 0,
            'cache_misses': 0
        }
        
        # Memory management
        self._gc_counter = 0
        
        logger.info("PerformanceOptimizer initialized")
    
    def optimize_for_throughput(self, 
                               items: List[Any], 
                               processor_func: Callable,
                               use_async: bool = False) -> List[Any]:
        """Optimize processing for maximum throughput"""
        start_time = time.time()
        
        try:
            if use_async and asyncio.iscoroutinefunction(processor_func):
                # Use async processing
                results = asyncio.run(self.async_processor.process_async(items, processor_func))
            else:
                # Use parallel processing
                results = self.batch_processor.process_parallel(items, processor_func)
            
            # Update metrics
            processing_time = time.time() - start_time
            self._update_performance_metrics(len(items), processing_time)
            
            # Memory management
            self._manage_memory()
            
            return results
            
        except Exception as e:
            logger.error(f"Throughput optimization error: {e}")
            raise
    
    def optimize_for_latency(self, 
                            item: Any, 
                            processor_func: Callable,
                            enable_cache: bool = True) -> Any:
        """Optimize single item processing for minimum latency"""
        start_time = time.time()
        
        try:
            # Check cache for ultra-low latency
            if enable_cache and self.batch_processor.cache:
                cache_key = hashlib.sha256(str(item).encode()).hexdigest()[:16]
                cached_result = self.batch_processor.cache.get(cache_key)
                if cached_result:
                    return cached_result
            
            # Process item
            result = processor_func(item)
            
            # Cache result
            if enable_cache and self.batch_processor.cache:
                self.batch_processor.cache.put(cache_key, result)
            
            processing_time = time.time() - start_time
            self._update_performance_metrics(1, processing_time)
            
            return result
            
        except Exception as e:
            logger.error(f"Latency optimization error: {e}")
            raise
    
    def optimize_memory_usage(self):
        """Optimize memory usage by cleaning up caches and running GC"""
        # Clear old cache entries
        if self.batch_processor.cache:
            cache_stats = self.batch_processor.cache.get_stats()
            logger.info(f"Cache stats before cleanup: {cache_stats}")
            
            # Clear cache if hit rate is very low
            if cache_stats['hit_rate'] < 0.1 and cache_stats['size'] > 100:
                self.batch_processor.cache.clear()
                logger.info("Cache cleared due to low hit rate")
        
        # Run garbage collection
        collected = gc.collect()
        logger.info(f"Garbage collection freed {collected} objects")
        
        # Clear performance metrics history if it's getting large
        if self.performance_metrics['total_processed'] > 100000:
            self.performance_metrics = {
                'total_processed': 0,
                'total_time': 0.0,
                'average_throughput': 0.0,
                'cache_hits': 0,
                'cache_misses': 0
            }
    
    def _update_performance_metrics(self, item_count: int, processing_time: float):
        """Update performance tracking metrics"""
        self.performance_metrics['total_processed'] += item_count
        self.performance_metrics['total_time'] += processing_time
        
        if self.performance_metrics['total_time'] > 0:
            self.performance_metrics['average_throughput'] = (
                self.performance_metrics['total_processed'] / 
                self.performance_metrics['total_time']
            )
        
        # Update cache metrics if available
        if self.batch_processor.cache:
            cache_stats = self.batch_processor.cache.get_stats()
            self.performance_metrics['cache_hits'] = cache_stats['hits']
            self.performance_metrics['cache_misses'] = cache_stats['misses']
    
    def _manage_memory(self):
        """Manage memory usage during processing"""
        self._gc_counter += 1
        
        # Run garbage collection periodically
        if (self.config.enable_memory_optimization and 
            self._gc_counter >= self.config.gc_threshold):
            gc.collect()
            self._gc_counter = 0
    
    def get_performance_report(self) -> Dict[str, Any]:
        """Get comprehensive performance report"""
        report = {
            'performance_metrics': self.performance_metrics.copy(),
            'cache_stats': None,
            'memory_info': {
                'gc_collections': gc.get_count(),
                'gc_thresholds': gc.get_threshold()
            },
            'config': {
                'max_workers': self.config.max_workers,
                'batch_size': self.config.batch_size,
                'cache_enabled': self.config.enable_caching,
                'cache_size': self.config.cache_size if self.config.enable_caching else 0
            }
        }
        
        # Add cache statistics
        if self.batch_processor.cache:
            report['cache_stats'] = self.batch_processor.cache.get_stats()
        
        # Add memory pool stats
        if hasattr(self.batch_processor, 'result_pool'):
            report['memory_pool_stats'] = self.batch_processor.result_pool.get_stats()
        
        return report
    
    def benchmark_processing_methods(self, 
                                   test_items: List[Any], 
                                   processor_func: Callable) -> Dict[str, Dict[str, float]]:
        """Benchmark different processing methods"""
        results = {}
        
        # Sequential processing
        start_time = time.time()
        sequential_results = [processor_func(item) for item in test_items]
        sequential_time = time.time() - start_time
        results['sequential'] = {
            'time': sequential_time,
            'throughput': len(test_items) / sequential_time,
            'items_processed': len(test_items)
        }
        
        # Parallel processing (threads)
        start_time = time.time()
        parallel_results = self.batch_processor.process_parallel(test_items, processor_func, use_processes=False)
        parallel_time = time.time() - start_time
        results['parallel_threads'] = {
            'time': parallel_time,
            'throughput': len(test_items) / parallel_time,
            'speedup': sequential_time / parallel_time,
            'items_processed': len(test_items)
        }
        
        # Parallel processing (processes) - for CPU-intensive tasks
        start_time = time.time()
        process_results = self.batch_processor.process_parallel(test_items, processor_func, use_processes=True)
        process_time = time.time() - start_time
        results['parallel_processes'] = {
            'time': process_time,
            'throughput': len(test_items) / process_time,
            'speedup': sequential_time / process_time,
            'items_processed': len(test_items)
        }
        
        logger.info(f"Benchmark results: {results}")
        return results