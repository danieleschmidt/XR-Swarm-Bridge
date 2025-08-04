# CI/CD Pipeline Documentation

## Overview

This document describes the CI/CD pipeline for XR-Swarm-Bridge. The pipeline includes automated testing, security scanning, building, and deployment across multiple environments.

## Pipeline Workflow

### 1. Pre-flight Checks
- **Change Detection**: Determines if builds are needed based on changed files
- **Version Generation**: Creates version tags for releases and development builds
- **Prerequisites**: Validates required tools and permissions

### 2. Testing Phase

#### ROS 2 Package Tests
```yaml
Strategy:
  - ROS Distro: Humble
  - Container: ros:humble
  - Tests: Unit tests for all ROS packages
  - Coverage: Minimum 80% required
```

#### Webapp Tests
```yaml
Strategy:
  - Node.js Versions: 18, 20
  - Tests: Unit, integration, type checking, linting
  - Build Validation: Production build test
  - Coverage: Minimum 85% required
```

### 3. Security & Quality

#### Security Scanning
- **Trivy**: Vulnerability scanning for dependencies
- **TruffleHog**: Secret detection in codebase
- **SARIF Upload**: Integration with GitHub Security tab

#### Code Quality
- **SonarCloud**: Code quality analysis
- **ESLint**: JavaScript/TypeScript linting
- **Type Checking**: Full TypeScript validation

### 4. Docker Build

#### Multi-Component Build
```yaml
Components:
  - swarm-coordinator
  - webrtc-bridge  
  - webapp
  - drone-agent

Platforms:
  - linux/amd64
  - linux/arm64

Registry: ghcr.io
```

### 5. Integration Testing

#### Test Environment
- **Docker Compose**: Multi-service test environment
- **Database**: PostgreSQL test instance
- **Services**: All components running together
- **Tests**: End-to-end workflow validation

### 6. Performance Testing

#### Load Testing (K6)
- **Concurrent Users**: Up to 100 operators
- **Robot Load**: Up to 1000 simulated agents
- **Latency Testing**: Command response times
- **Throughput**: Messages per second capacity

### 7. Deployment

#### Staging Deployment
- **Trigger**: Push to `develop` branch
- **Environment**: Kubernetes staging cluster
- **Validation**: Smoke tests post-deployment
- **Rollback**: Automatic on failure

#### Production Deployment
- **Trigger**: Release creation
- **Strategy**: Blue-green deployment
- **Validation**: Comprehensive health checks
- **Notifications**: Slack integration

## Setting Up CI/CD

### 1. GitHub Repository Setup

Create the CI/CD workflow file at `.github/workflows/ci.yml`:

```yaml
name: XR-Swarm-Bridge CI/CD

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main, develop ]
  release:
    types: [ created ]

env:
  REGISTRY: ghcr.io
  IMAGE_NAME: ${{ github.repository }}

jobs:
  # [Complete workflow content available in scripts/ci-template.yml]
```

### 2. Required Secrets

Configure these secrets in GitHub repository settings:

```bash
# Container Registry
GITHUB_TOKEN=<automatic>

# SonarCloud Integration
SONAR_TOKEN=<your_sonar_token>

# Kubernetes Clusters
KUBE_CONFIG_STAGING=<base64_encoded_kubeconfig>
KUBE_CONFIG_PRODUCTION=<base64_encoded_kubeconfig>

# Notifications
SLACK_WEBHOOK_URL=<slack_webhook_for_notifications>

# Optional: Cloud Provider Credentials
AWS_ACCESS_KEY_ID=<aws_key>
AWS_SECRET_ACCESS_KEY=<aws_secret>
GOOGLE_APPLICATION_CREDENTIALS=<gcp_service_account>
AZURE_CREDENTIALS=<azure_sp_credentials>
```

### 3. Branch Protection Rules

Configure branch protection for `main`:

```yaml
Required Status Checks:
  - ros-tests
  - webapp-tests
  - security-scan
  - integration-tests

Required Reviews: 1
Dismiss Stale Reviews: true
Require Up-to-date Branches: true
Restrict Pushes: true
```

## Local Development Testing

### Run All Tests Locally

```bash
# Complete test suite
./scripts/run_tests.sh

# Specific test types
./scripts/run_tests.sh --no-integration
./scripts/run_tests.sh --performance
```

### Pre-commit Hooks

Setup pre-commit hooks to ensure code quality:

```bash
# Install pre-commit
pip install pre-commit

# Install hooks
pre-commit install

# Run manually
pre-commit run --all-files
```

### Docker Testing

Test the complete system locally:

```bash
# Build all images
docker-compose build

# Run integration tests
docker-compose -f docker-compose.test.yml up

# Performance testing
docker-compose -f docker-compose.perf.yml up
```

## Environment Configuration

### Staging Environment

```yaml
Kubernetes Cluster: staging-cluster
Namespace: xr-swarm-staging
Replicas: 2 (HA setup)
Resources: Medium (2 CPU, 4GB RAM per pod)
Database: PostgreSQL (shared dev instance)
Monitoring: Basic metrics and logs
```

### Production Environment

```yaml
Kubernetes Cluster: production-cluster
Namespace: xr-swarm-production
Replicas: 3+ (auto-scaling)
Resources: High (4+ CPU, 8+ GB RAM per pod)
Database: PostgreSQL (dedicated cluster)
Monitoring: Full observability stack
Load Balancer: Cloud provider LB with SSL
```

## Monitoring & Alerting

### Key Metrics

```yaml
Build Metrics:
  - Build success rate
  - Build duration
  - Test coverage percentage
  - Security vulnerabilities

Deployment Metrics:
  - Deployment frequency
  - Lead time for changes
  - Mean time to recovery
  - Change failure rate
```

### Alerts

```yaml
Build Failures:
  - Immediate notification to team
  - Auto-assign to last committer

Security Issues:
  - Critical: Immediate escalation
  - High: Within 24 hours
  - Medium/Low: Weekly review

Performance Degradation:
  - >10% increase in build time
  - Test timeout failures
  - Resource usage spikes
```

## Troubleshooting

### Common Issues

#### Build Failures

```bash
# Check logs
gh run view <run_id> --log

# Re-run failed jobs
gh run rerun <run_id> --failed

# Debug locally
act pull_request
```

#### Security Scan Failures

```bash
# Review vulnerabilities
trivy fs . --severity HIGH,CRITICAL

# Update dependencies
npm audit fix
pip-audit --fix

# Check for secrets
trufflehog --regex --entropy=False .
```

#### Deployment Issues

```bash
# Check pod status
kubectl get pods -n xr-swarm-production

# View logs
kubectl logs -f deployment/swarm-coordinator

# Rollback deployment
kubectl rollout undo deployment/swarm-coordinator
```

### Performance Optimization

#### Faster Builds

```yaml
Optimizations:
  - Layer caching in Docker builds
  - Parallel job execution
  - Dependency caching
  - Test parallelization
  - Selective building based on changes
```

#### Resource Management

```yaml
GitHub Actions:
  - Use matrix builds for parallel execution
  - Cache dependencies between runs
  - Use self-hosted runners for performance
  - Optimize Docker layer ordering
```

## Best Practices

### Code Quality

1. **Automated Testing**: Minimum 85% coverage
2. **Static Analysis**: No critical security vulnerabilities
3. **Code Style**: Consistent formatting with automated tools
4. **Documentation**: Updated with code changes

### Security

1. **Dependency Scanning**: Regular vulnerability assessment
2. **Secret Management**: No hardcoded secrets
3. **Access Control**: Least privilege principle
4. **Audit Logging**: Track all deployments

### Deployment

1. **Blue-Green**: Zero-downtime deployments
2. **Health Checks**: Comprehensive validation
3. **Rollback Plan**: Automated failure recovery
4. **Monitoring**: Real-time deployment tracking

## Future Enhancements

### Planned Improvements

1. **GitOps**: Argo CD for deployment automation
2. **Chaos Engineering**: Automated resilience testing
3. **Progressive Delivery**: Feature flags and canary deployments
4. **Advanced Security**: SAST/DAST integration
5. **Performance Regression**: Automated performance testing

### Metrics and KPIs

```yaml
Target Metrics:
  - Deployment Frequency: Multiple per day
  - Lead Time: <4 hours
  - MTTR: <30 minutes
  - Change Failure Rate: <5%
  - Build Success Rate: >95%
```

This CI/CD pipeline ensures high-quality, secure, and reliable deployments of the XR-Swarm-Bridge system while maintaining development velocity and operational excellence.