"""
Setup configuration for Sentiment Analyzer Pro
"""

from setuptools import setup, find_packages
from pathlib import Path

# Read the contents of README file
this_directory = Path(__file__).parent
long_description = (this_directory / "README.md").read_text() if (this_directory / "README.md").exists() else ""

# Read requirements
requirements = []
if (this_directory / "requirements.txt").exists():
    with open("requirements.txt") as f:
        requirements = [line.strip() for line in f if line.strip() and not line.startswith("#")]

setup(
    name="sentiment-analyzer-pro",
    version="1.0.0",
    author="Terragon Labs",
    author_email="daniel@terragon.dev",
    description="Professional-grade sentiment analysis with research capabilities and multiple model support",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/danieleschmidt/sentiment-analyzer-pro",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Text Processing :: Linguistic",
        "Topic :: Software Development :: Libraries :: Python Modules"
    ],
    python_requires=">=3.8",
    install_requires=[
        # Core dependencies only (minimal for basic functionality)
        "numpy>=1.21.0",
        "pandas>=1.3.0", 
        "scikit-learn>=1.0.0",
        "requests>=2.28.0",
        "pydantic>=1.9.0"
    ],
    extras_require={
        "full": requirements,  # All requirements for full functionality
        "nlp": [
            "nltk>=3.7",
            "spacy>=3.4.0", 
            "textblob>=0.17.1",
            "transformers>=4.20.0",
            "torch>=1.12.0"
        ],
        "web": [
            "flask>=2.1.0",
            "flask-cors>=3.0.10",
            "fastapi>=0.78.0",
            "uvicorn>=0.18.0"
        ],
        "dev": [
            "pytest>=7.1.0",
            "pytest-asyncio>=0.19.0",
            "pytest-cov>=3.0.0",
            "black>=22.3.0",
            "flake8>=4.0.0",
            "mypy>=0.961"
        ],
        "ml": [
            "tensorflow>=2.9.0",
            "torch>=1.12.0",
            "transformers>=4.20.0",
            "onnx>=1.12.0",
            "onnxruntime>=1.12.0"
        ]
    },
    entry_points={
        "console_scripts": [
            "sentiment-analyzer=sentiment_analyzer.cli:main",
        ],
    },
    package_data={
        "sentiment_analyzer": ["data/*.json", "models/*.pkl"],
    },
    include_package_data=True,
    keywords=[
        "sentiment-analysis", "nlp", "machine-learning", "text-analysis",
        "natural-language-processing", "ai", "research", "linguistics"
    ],
    project_urls={
        "Bug Reports": "https://github.com/danieleschmidt/sentiment-analyzer-pro/issues",
        "Source": "https://github.com/danieleschmidt/sentiment-analyzer-pro",
        "Documentation": "https://sentiment-analyzer-pro.readthedocs.io",
    }
)