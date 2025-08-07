"""
FastAPI server for sentiment analysis web service
"""

import time
import asyncio
import logging
from typing import List, Optional, Dict, Any
from datetime import datetime
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Depends, Security, BackgroundTasks
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field, validator
import uvicorn

from ..core.analyzer import SentimentAnalyzer, SentimentResult, SentimentLabel
from ..core.pipeline import AnalysisPipeline, PipelineConfig
from ..core.validation import InputValidator, ValidationLevel
from ..core.error_handling import ErrorHandler, SentimentAnalysisError
from ..core.security import SecurityManager, SecurityLevel
from ..utils.metrics import SentimentMetrics

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Global instances
analyzer = None
pipeline = None
error_handler = None
security_manager = None
metrics = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan management"""
    global analyzer, pipeline, error_handler, security_manager, metrics
    
    # Startup
    logger.info("Starting Sentiment Analyzer API server...")
    
    # Initialize components
    analyzer = SentimentAnalyzer()
    pipeline_config = PipelineConfig(batch_size=50, cache_results=True)
    pipeline = AnalysisPipeline(pipeline_config)
    error_handler = ErrorHandler(max_retries=3)
    security_manager = SecurityManager(SecurityLevel.STANDARD)
    metrics = SentimentMetrics()
    
    logger.info("All components initialized successfully")
    
    yield
    
    # Shutdown
    logger.info("Shutting down Sentiment Analyzer API server...")


# Create FastAPI app
app = FastAPI(
    title="Sentiment Analyzer Pro API",
    description="Professional-grade sentiment analysis with research capabilities",
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc"
)

# Security
security = HTTPBearer(auto_error=False)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Trusted host middleware for production
app.add_middleware(
    TrustedHostMiddleware, 
    allowed_hosts=["*"]  # Configure appropriately for production
)


# Request/Response models
class AnalysisRequest(BaseModel):
    """Request model for sentiment analysis"""
    text: str = Field(..., min_length=1, max_length=10000, description="Text to analyze")
    model_type: Optional[str] = Field("lexicon", description="Model type to use")
    include_metadata: bool = Field(False, description="Include detailed metadata")
    
    @validator('text')
    def validate_text(cls, v):
        if not v.strip():
            raise ValueError("Text cannot be empty or whitespace only")
        return v


class BatchAnalysisRequest(BaseModel):
    """Request model for batch sentiment analysis"""
    texts: List[str] = Field(..., min_items=1, max_items=100, description="List of texts to analyze")
    model_type: Optional[str] = Field("lexicon", description="Model type to use")
    include_metadata: bool = Field(False, description="Include detailed metadata")
    
    @validator('texts')
    def validate_texts(cls, v):
        if not all(text.strip() for text in v):
            raise ValueError("All texts must be non-empty")
        return v


class AnalysisResponse(BaseModel):
    """Response model for sentiment analysis"""
    text: str
    sentiment: str
    confidence: float = Field(..., ge=0.0, le=1.0)
    scores: Dict[str, float]
    model_used: str
    processing_time: float
    metadata: Optional[Dict[str, Any]] = None


class BatchAnalysisResponse(BaseModel):
    """Response model for batch analysis"""
    results: List[AnalysisResponse]
    summary: Dict[str, Any]
    total_processing_time: float


class HealthResponse(BaseModel):
    """Health check response"""
    status: str
    timestamp: str
    version: str
    components: Dict[str, str]


class MetricsResponse(BaseModel):
    """Metrics response"""
    system_metrics: Dict[str, Any]
    security_metrics: Dict[str, Any]
    error_metrics: Dict[str, Any]


# Dependency functions
def get_client_ip(request) -> str:
    """Extract client IP address"""
    forwarded = request.headers.get("X-Forwarded-For")
    if forwarded:
        return forwarded.split(",")[0].strip()
    return request.client.host if request.client else "unknown"


async def authenticate_request(
    credentials: HTTPAuthorizationCredentials = Security(security),
    request = None
) -> Optional[str]:
    """Authenticate API request"""
    client_ip = get_client_ip(request) if request else "unknown"
    
    # For basic authentication, we'll allow requests without tokens
    # In production, implement proper token validation
    api_key = credentials.token if credentials else None
    
    is_authenticated, user_id = security_manager.authenticate_request(api_key, client_ip)
    
    if not is_authenticated:
        raise HTTPException(
            status_code=401,
            detail="Authentication failed or rate limit exceeded",
            headers={"WWW-Authenticate": "Bearer"}
        )
    
    return user_id


def convert_result_to_response(result: SentimentResult, include_metadata: bool = False) -> AnalysisResponse:
    """Convert SentimentResult to API response"""
    return AnalysisResponse(
        text=result.text,
        sentiment=result.sentiment.value,
        confidence=result.confidence,
        scores=result.scores,
        model_used=result.model_used,
        processing_time=result.processing_time,
        metadata=result.metadata if include_metadata else None
    )


# API Endpoints
@app.get("/health", response_model=HealthResponse)
async def health_check():
    """Health check endpoint"""
    return HealthResponse(
        status="healthy",
        timestamp=datetime.now().isoformat(),
        version="1.0.0",
        components={
            "analyzer": "ready",
            "pipeline": "ready",
            "security": "ready",
            "metrics": "ready"
        }
    )


@app.post("/analyze", response_model=AnalysisResponse)
async def analyze_sentiment(
    request: AnalysisRequest,
    background_tasks: BackgroundTasks,
    user_id: Optional[str] = Depends(authenticate_request)
):
    """Analyze sentiment of a single text"""
    try:
        start_time = time.time()
        
        # Validate and sanitize input
        client_ip = "unknown"  # Would get from request in full implementation
        sanitized_text, is_safe = security_manager.validate_and_sanitize_input(
            request.text, client_ip, user_id
        )
        
        if not is_safe:
            raise HTTPException(
                status_code=400,
                detail="Input contains potentially malicious content"
            )
        
        # Analyze sentiment
        with error_handler.error_context("sentiment_analysis"):
            result = analyzer.analyze(sanitized_text)
        
        # Convert to response format
        response = convert_result_to_response(result, request.include_metadata)
        
        # Add metrics in background
        background_tasks.add_task(metrics.add_results, [result])
        
        logger.info(f"Analysis completed for user {user_id} in {time.time() - start_time:.3f}s")
        return response
        
    except SentimentAnalysisError as e:
        logger.error(f"Sentiment analysis error: {e}")
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        logger.error(f"Unexpected error in analysis: {e}")
        error_handler.handle_error(e, {"endpoint": "analyze", "user_id": user_id})
        raise HTTPException(status_code=500, detail="Internal server error")


@app.post("/analyze/batch", response_model=BatchAnalysisResponse)
async def analyze_batch(
    request: BatchAnalysisRequest,
    background_tasks: BackgroundTasks,
    user_id: Optional[str] = Depends(authenticate_request)
):
    """Analyze sentiment of multiple texts"""
    try:
        start_time = time.time()
        
        # Validate input size
        if len(request.texts) > 100:
            raise HTTPException(
                status_code=413,
                detail="Batch size too large. Maximum 100 texts allowed."
            )
        
        # Sanitize all texts
        client_ip = "unknown"
        sanitized_texts = []
        for text in request.texts:
            sanitized_text, is_safe = security_manager.validate_and_sanitize_input(
                text, client_ip, user_id
            )
            if not is_safe:
                raise HTTPException(
                    status_code=400,
                    detail=f"Input contains potentially malicious content: {text[:50]}..."
                )
            sanitized_texts.append(sanitized_text)
        
        # Process batch
        with error_handler.error_context("batch_analysis"):
            results = pipeline.process_batch(sanitized_texts)
        
        # Convert results
        response_results = [
            convert_result_to_response(result, request.include_metadata)
            for result in results
        ]
        
        # Generate summary
        total_processing_time = time.time() - start_time
        summary = metrics.generate_summary_report(results)
        
        response = BatchAnalysisResponse(
            results=response_results,
            summary=summary,
            total_processing_time=total_processing_time
        )
        
        # Add metrics in background
        background_tasks.add_task(metrics.add_results, results)
        
        logger.info(f"Batch analysis of {len(request.texts)} texts completed for user {user_id}")
        return response
        
    except SentimentAnalysisError as e:
        logger.error(f"Batch analysis error: {e}")
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        logger.error(f"Unexpected error in batch analysis: {e}")
        error_handler.handle_error(e, {"endpoint": "batch_analyze", "user_id": user_id})
        raise HTTPException(status_code=500, detail="Internal server error")


@app.get("/models")
async def list_models(user_id: Optional[str] = Depends(authenticate_request)):
    """List available sentiment analysis models"""
    models = [
        {
            "name": "lexicon",
            "description": "Lexicon-based sentiment analysis with word scoring",
            "type": "rule_based",
            "languages": ["en"],
            "performance": "fast"
        },
        {
            "name": "rule_based", 
            "description": "Rule-based analysis with linguistic patterns",
            "type": "rule_based",
            "languages": ["en"],
            "performance": "fast"
        }
    ]
    
    return {"models": models}


@app.get("/metrics", response_model=MetricsResponse)
async def get_metrics(user_id: Optional[str] = Depends(authenticate_request)):
    """Get system metrics and statistics"""
    try:
        # Get various metrics
        security_metrics = security_manager.get_security_metrics()
        error_metrics = error_handler.get_error_statistics()
        pipeline_stats = pipeline.get_pipeline_stats()
        
        system_metrics = {
            "pipeline_stats": pipeline_stats,
            "uptime": time.time(),  # Would track actual uptime
            "version": "1.0.0"
        }
        
        return MetricsResponse(
            system_metrics=system_metrics,
            security_metrics=security_metrics,
            error_metrics=error_metrics
        )
        
    except Exception as e:
        logger.error(f"Error getting metrics: {e}")
        raise HTTPException(status_code=500, detail="Error retrieving metrics")


@app.delete("/cache")
async def clear_cache(user_id: Optional[str] = Depends(authenticate_request)):
    """Clear analysis cache"""
    try:
        pipeline.clear_cache()
        return {"message": "Cache cleared successfully"}
    except Exception as e:
        logger.error(f"Error clearing cache: {e}")
        raise HTTPException(status_code=500, detail="Error clearing cache")


# Error handlers
@app.exception_handler(ValueError)
async def value_error_handler(request, exc):
    return JSONResponse(
        status_code=400,
        content={"detail": f"Invalid input: {str(exc)}"}
    )


@app.exception_handler(SentimentAnalysisError)
async def sentiment_error_handler(request, exc):
    return JSONResponse(
        status_code=422,
        content={"detail": str(exc), "category": exc.category.value}
    )


# Main function for running server
def main():
    """Run the API server"""
    uvicorn.run(
        "sentiment_analyzer.api.server:app",
        host="0.0.0.0",
        port=8000,
        reload=False,  # Set to True for development
        log_level="info",
        access_log=True
    )


if __name__ == "__main__":
    main()