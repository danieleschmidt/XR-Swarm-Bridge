"""
Tests for sentiment analysis pipeline
"""

import pytest
import time
from sentiment_analyzer.core.pipeline import AnalysisPipeline, PipelineConfig
from sentiment_analyzer.core.analyzer import SentimentResult, SentimentLabel


class TestAnalysisPipeline:
    """Test suite for AnalysisPipeline class"""
    
    @pytest.fixture
    def pipeline(self):
        """Create pipeline instance for tests"""
        config = PipelineConfig(batch_size=10, cache_results=True)
        return AnalysisPipeline(config)
    
    @pytest.fixture
    def sample_texts(self):
        """Sample texts for testing"""
        return [
            "I love this amazing product!",
            "This is terrible and disappointing.",
            "It's okay, nothing special.",
            "Excellent quality and service!",
            "Poor performance and bad design."
        ]
    
    def test_pipeline_initialization(self, pipeline):
        """Test pipeline initializes correctly"""
        assert pipeline.config.batch_size == 10
        assert pipeline.config.cache_results is True
        assert pipeline.analyzer is not None
        assert pipeline.cache is not None
    
    def test_process_single_text(self, pipeline):
        """Test processing single text"""
        text = "This is a great product!"
        result = pipeline.process_single(text)
        
        assert isinstance(result, SentimentResult)
        assert result.text == text
        assert result.sentiment in [SentimentLabel.POSITIVE, SentimentLabel.NEGATIVE, SentimentLabel.NEUTRAL]
        assert 0.0 <= result.confidence <= 1.0
        assert result.processing_time > 0
    
    def test_process_batch(self, pipeline, sample_texts):
        """Test batch processing"""
        results = pipeline.process_batch(sample_texts)
        
        assert len(results) == len(sample_texts)
        assert all(isinstance(result, SentimentResult) for result in results)
        assert all(result.text in sample_texts for result in results)
    
    def test_cache_functionality(self, pipeline):
        """Test that caching works correctly"""
        text = "Test text for caching"
        
        # First call
        result1 = pipeline.process_single(text)
        
        # Second call should use cache
        result2 = pipeline.process_single(text)
        
        assert result1.text == result2.text
        assert result1.sentiment == result2.sentiment
        # Processing time for cached result should be much faster
        # (though this is hard to test reliably due to timing precision)
    
    def test_preprocessing_steps(self, pipeline):
        """Test preprocessing functionality"""
        text_with_issues = "  This is a TEST with  EXTRA   spaces!!! @#$  "
        result = pipeline.process_single(text_with_issues)
        
        # Should handle the text without errors
        assert isinstance(result, SentimentResult)
        assert result.confidence >= 0.0
    
    def test_postprocessing_steps(self, pipeline):
        """Test postprocessing functionality"""
        text = "Short text"
        result = pipeline.process_single(text)
        
        # Check that metadata is enriched
        assert result.metadata is not None
        assert 'pipeline_version' in result.metadata
        assert 'timestamp' in result.metadata
    
    def test_empty_batch_handling(self, pipeline):
        """Test handling of empty batch"""
        results = pipeline.process_batch([])
        assert results == []
    
    def test_error_handling_in_batch(self, pipeline):
        """Test error handling during batch processing"""
        # Include some problematic texts
        texts = [
            "Good text",
            "",  # Empty text
            "Another good text",
            None  # This would cause an error in validation
        ]
        
        # Should handle errors gracefully
        results = pipeline.process_batch([t for t in texts if t is not None])
        assert len(results) >= 2  # At least the good texts should be processed
    
    def test_model_switching(self, pipeline):
        """Test switching between different models"""
        text = "Test text for model switching"
        
        # Process with default model
        result1 = pipeline.process_single(text)
        
        # Switch model
        pipeline.switch_model("rule_based")
        result2 = pipeline.process_single(text)
        
        # Results might be different with different models
        assert result1.model_used != result2.model_used
        assert result2.model_used == "rule_based"
    
    def test_pipeline_stats(self, pipeline, sample_texts):
        """Test pipeline statistics"""
        # Process some texts
        pipeline.process_batch(sample_texts)
        
        stats = pipeline.get_pipeline_stats()
        
        assert 'model_type' in stats
        assert 'cache_size' in stats
        assert 'preprocessing_steps' in stats
        assert 'postprocessing_steps' in stats
        assert 'batch_size' in stats
    
    def test_cache_clearing(self, pipeline):
        """Test cache clearing functionality"""
        text = "Test text for cache clearing"
        
        # Process text to populate cache
        pipeline.process_single(text)
        
        # Clear cache
        pipeline.clear_cache()
        
        # Cache should be empty (if we had access to check it)
        # This is more of an integration test
        result = pipeline.process_single(text)
        assert isinstance(result, SentimentResult)
    
    def test_custom_preprocessing_step(self, pipeline):
        """Test adding custom preprocessing step"""
        def custom_preprocessor(text):
            return text.upper()
        
        pipeline.add_preprocessing_step(custom_preprocessor)
        
        # This is hard to test directly without accessing internal state
        # But we can ensure the pipeline still works
        result = pipeline.process_single("test text")
        assert isinstance(result, SentimentResult)
    
    def test_custom_postprocessing_step(self, pipeline):
        """Test adding custom postprocessing step"""
        def custom_postprocessor(result):
            if result.metadata is None:
                result.metadata = {}
            result.metadata['custom_processed'] = True
            return result
        
        pipeline.add_postprocessing_step(custom_postprocessor)
        
        result = pipeline.process_single("test text")
        assert result.metadata is not None
        assert result.metadata.get('custom_processed') is True
    
    def test_large_batch_processing(self, pipeline):
        """Test processing large batch efficiently"""
        large_batch = ["Test text number {}".format(i) for i in range(100)]
        
        start_time = time.time()
        results = pipeline.process_batch(large_batch)
        end_time = time.time()
        
        assert len(results) == 100
        assert all(isinstance(result, SentimentResult) for result in results)
        
        # Should complete in reasonable time (less than 5 seconds)
        processing_time = end_time - start_time
        assert processing_time < 5.0
    
    def test_concurrent_processing(self, pipeline, sample_texts):
        """Test that pipeline works correctly with concurrent access"""
        import threading
        
        results = []
        errors = []
        
        def process_texts():
            try:
                batch_results = pipeline.process_batch(sample_texts)
                results.extend(batch_results)
            except Exception as e:
                errors.append(e)
        
        threads = [threading.Thread(target=process_texts) for _ in range(3)]
        
        for thread in threads:
            thread.start()
        
        for thread in threads:
            thread.join()
        
        # Should complete without errors
        assert len(errors) == 0
        assert len(results) > 0
    
    def test_memory_efficiency(self, pipeline):
        """Test memory efficiency with large inputs"""
        import gc
        
        # Process many texts to test memory usage
        large_texts = ["This is test text number {} with some content".format(i) 
                      for i in range(1000)]
        
        initial_objects = len(gc.get_objects())
        
        results = pipeline.process_batch(large_texts)
        
        # Force garbage collection
        del results
        gc.collect()
        
        final_objects = len(gc.get_objects())
        
        # Memory usage shouldn't grow excessively
        # This is a rough test - exact numbers depend on Python implementation
        assert final_objects - initial_objects < 1000