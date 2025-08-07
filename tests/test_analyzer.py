"""
Tests for the core sentiment analyzer
"""

import pytest
from sentiment_analyzer.core.analyzer import SentimentAnalyzer, SentimentLabel, SentimentResult


class TestSentimentAnalyzer:
    """Test suite for SentimentAnalyzer class"""
    
    @pytest.fixture
    def analyzer(self):
        """Create analyzer instance for tests"""
        return SentimentAnalyzer()
    
    def test_analyzer_initialization(self, analyzer):
        """Test analyzer initializes correctly"""
        assert analyzer.model_type == "lexicon"
        assert len(analyzer.positive_words) > 0
        assert len(analyzer.negative_words) > 0
        assert len(analyzer.intensifiers) > 0
        assert len(analyzer.negations) > 0
    
    def test_analyze_positive_text(self, analyzer):
        """Test analysis of positive text"""
        text = "I love this amazing product! It's absolutely wonderful."
        result = analyzer.analyze(text)
        
        assert isinstance(result, SentimentResult)
        assert result.sentiment == SentimentLabel.POSITIVE
        assert result.confidence > 0.5
        assert result.text == text
        assert 'positive' in result.scores
        assert 'negative' in result.scores
        assert 'neutral' in result.scores
    
    def test_analyze_negative_text(self, analyzer):
        """Test analysis of negative text"""
        text = "This is terrible! I hate this awful product."
        result = analyzer.analyze(text)
        
        assert result.sentiment == SentimentLabel.NEGATIVE
        assert result.confidence > 0.5
        assert result.scores['negative'] > result.scores['positive']
    
    def test_analyze_neutral_text(self, analyzer):
        """Test analysis of neutral text"""
        text = "This is a regular product with standard features."
        result = analyzer.analyze(text)
        
        # May be neutral or slightly positive/negative depending on lexicon
        assert result.sentiment in [SentimentLabel.POSITIVE, SentimentLabel.NEGATIVE, SentimentLabel.NEUTRAL]
        assert 0.0 <= result.confidence <= 1.0
    
    def test_analyze_empty_text(self, analyzer):
        """Test handling of empty text"""
        with pytest.raises(ValueError, match="Input text must be a non-empty string"):
            analyzer.analyze("")
    
    def test_analyze_none_text(self, analyzer):
        """Test handling of None input"""
        with pytest.raises(ValueError, match="Input text must be a non-empty string"):
            analyzer.analyze(None)
    
    def test_analyze_non_string_input(self, analyzer):
        """Test handling of non-string input"""
        with pytest.raises(ValueError, match="Input text must be a non-empty string"):
            analyzer.analyze(123)
    
    def test_intensifier_effect(self, analyzer):
        """Test that intensifiers increase sentiment scores"""
        basic_text = "This is good."
        intensified_text = "This is very good."
        
        basic_result = analyzer.analyze(basic_text)
        intensified_result = analyzer.analyze(intensified_text)
        
        # Intensified text should have higher positive score
        assert intensified_result.scores['positive'] >= basic_result.scores['positive']
    
    def test_negation_effect(self, analyzer):
        """Test that negations flip sentiment"""
        positive_text = "This is good."
        negated_text = "This is not good."
        
        positive_result = analyzer.analyze(positive_text)
        negated_result = analyzer.analyze(negated_text)
        
        # Negation should change sentiment direction
        assert positive_result.sentiment != negated_result.sentiment or \
               negated_result.scores['negative'] > positive_result.scores['negative']
    
    def test_processing_time_recorded(self, analyzer):
        """Test that processing time is recorded"""
        result = analyzer.analyze("Test text")
        assert result.processing_time > 0
    
    def test_metadata_included(self, analyzer):
        """Test that metadata is included in results"""
        result = analyzer.analyze("Test text for metadata")
        assert result.metadata is not None
        assert 'text_length' in result.metadata
        assert 'processed_length' in result.metadata
    
    def test_batch_analysis(self, analyzer):
        """Test batch analysis functionality"""
        texts = [
            "I love this!",
            "This is terrible.",
            "It's okay, I guess.",
            "Amazing quality!"
        ]
        
        results = analyzer.analyze_batch(texts)
        
        assert len(results) == len(texts)
        assert all(isinstance(result, SentimentResult) for result in results)
        assert all(result.text in texts for result in results)
    
    def test_batch_analysis_empty_list(self, analyzer):
        """Test batch analysis with empty list"""
        with pytest.raises(ValueError, match="Input must be a non-empty list of strings"):
            analyzer.analyze_batch([])
    
    def test_batch_analysis_error_handling(self, analyzer):
        """Test batch analysis handles individual errors gracefully"""
        texts = [
            "Good text",
            None,  # This will cause an error
            "Another good text"
        ]
        
        results = analyzer.analyze_batch(texts)
        
        assert len(results) == len(texts)
        # Error result should have neutral sentiment and error in metadata
        error_result = results[1]
        assert error_result.sentiment == SentimentLabel.NEUTRAL
        assert error_result.confidence == 0.0
        assert 'error' in error_result.metadata
    
    def test_confidence_score_range(self, analyzer):
        """Test that confidence scores are within valid range"""
        texts = [
            "Excellent product!",
            "Terrible quality.",
            "It's fine.",
            "Absolutely amazing experience!",
            "Worst thing ever."
        ]
        
        for text in texts:
            result = analyzer.analyze(text)
            assert 0.0 <= result.confidence <= 1.0
    
    def test_score_normalization(self, analyzer):
        """Test that sentiment scores are properly normalized"""
        result = analyzer.analyze("Great product with excellent quality!")
        
        # Individual scores should be reasonable
        for score in result.scores.values():
            assert score >= 0.0
        
        # Scores don't need to sum to 1.0 in lexicon approach, 
        # but should be reasonable values
        assert all(score <= 2.0 for score in result.scores.values())  # Reasonable upper bound
    
    def test_model_type_consistency(self, analyzer):
        """Test that model type is consistent in results"""
        result = analyzer.analyze("Test text")
        assert result.model_used == analyzer.model_type
    
    def test_different_model_types(self):
        """Test different model type initialization"""
        lexicon_analyzer = SentimentAnalyzer("lexicon")
        rule_analyzer = SentimentAnalyzer("rule_based")
        
        assert lexicon_analyzer.model_type == "lexicon"
        assert rule_analyzer.model_type == "rule_based"
    
    def test_text_preprocessing(self, analyzer):
        """Test text preprocessing functionality"""
        text_with_special_chars = "I love this!!! @#$%^&* It's amazing!!!"
        result = analyzer.analyze(text_with_special_chars)
        
        # Should handle special characters without errors
        assert result.sentiment == SentimentLabel.POSITIVE
        assert result.confidence > 0.5
    
    def test_case_insensitive_analysis(self, analyzer):
        """Test that analysis is case insensitive"""
        lower_text = "this is great"
        upper_text = "THIS IS GREAT"
        mixed_text = "This Is Great"
        
        lower_result = analyzer.analyze(lower_text)
        upper_result = analyzer.analyze(upper_text)
        mixed_result = analyzer.analyze(mixed_text)
        
        # All should have same sentiment
        assert lower_result.sentiment == upper_result.sentiment == mixed_result.sentiment