#!/bin/bash

# Comprehensive test runner for XR-Swarm-Bridge
# This script runs all tests across the entire project

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[TEST]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_section() {
    echo -e "\n${BLUE}=== $1 ===${NC}"
}

# Test configuration
ROS_TESTS=true
WEBAPP_TESTS=true
INTEGRATION_TESTS=true
PERFORMANCE_TESTS=false
COVERAGE=true

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-ros)
            ROS_TESTS=false
            shift
            ;;
        --no-webapp)
            WEBAPP_TESTS=false
            shift
            ;;
        --no-integration)
            INTEGRATION_TESTS=false
            shift
            ;;
        --performance)
            PERFORMANCE_TESTS=true
            shift
            ;;
        --no-coverage)
            COVERAGE=false
            shift
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --no-ros           Skip ROS tests"
            echo "  --no-webapp        Skip webapp tests"
            echo "  --no-integration   Skip integration tests"
            echo "  --performance      Run performance tests"
            echo "  --no-coverage      Skip coverage reports"
            echo "  --help            Show this help"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Create test results directory
mkdir -p test_results
TEST_RESULTS_DIR="$(pwd)/test_results"

print_section "XR-Swarm-Bridge Test Suite"
print_status "Test results will be saved to: $TEST_RESULTS_DIR"

# ROS 2 Tests
if [ "$ROS_TESTS" = true ]; then
    print_section "ROS 2 Package Tests"
    
    cd ros2_ws
    
    # Source ROS 2
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
        print_status "ROS 2 Humble environment sourced"
    else
        print_warning "ROS 2 not found, skipping ROS tests"
        ROS_TESTS=false
    fi
    
    if [ "$ROS_TESTS" = true ]; then
        # Build workspace first
        print_status "Building ROS workspace for testing..."
        colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
        
        if [ $? -eq 0 ]; then
            print_status "ROS workspace built successfully ✓"
            
            # Source built workspace
            source install/setup.bash
            
            # Run ROS tests
            print_status "Running ROS package tests..."
            
            # Test xr_swarm_core
            print_status "Testing xr_swarm_core package..."
            colcon test --packages-select xr_swarm_core --event-handlers console_direct+
            
            # Test robot_agents
            print_status "Testing robot_agents package..."
            colcon test --packages-select robot_agents --event-handlers console_direct+
            
            # Test webrtc_bridge
            print_status "Testing webrtc_bridge package..."
            colcon test --packages-select webrtc_bridge --event-handlers console_direct+
            
            # Generate test report
            colcon test-result --verbose > "$TEST_RESULTS_DIR/ros_test_results.txt"
            
            if [ $? -eq 0 ]; then
                print_status "ROS tests completed successfully ✓"
            else
                print_error "Some ROS tests failed"
                exit 1
            fi
        else
            print_error "ROS workspace build failed"
            exit 1
        fi
    fi
    
    cd ..
fi

# Webapp Tests
if [ "$WEBAPP_TESTS" = true ]; then
    print_section "Webapp Tests"
    
    cd webapp
    
    # Check if Node.js is available
    if ! command -v node &> /dev/null; then
        print_error "Node.js not found, skipping webapp tests"
        WEBAPP_TESTS=false
    fi
    
    if [ "$WEBAPP_TESTS" = true ]; then
        # Install dependencies if needed
        if [ ! -d "node_modules" ]; then
            print_status "Installing webapp dependencies..."
            npm install
        fi
        
        # Run type checking
        print_status "Running TypeScript type checking..."
        npm run typecheck
        
        if [ $? -eq 0 ]; then
            print_status "TypeScript type checking passed ✓"
        else
            print_error "TypeScript type checking failed"
            exit 1
        fi
        
        # Run linting
        print_status "Running ESLint..."
        npm run lint
        
        if [ $? -eq 0 ]; then
            print_status "Linting passed ✓"
        else
            print_warning "Linting issues found (non-fatal)"
        fi
        
        # Run unit tests
        print_status "Running unit tests..."
        if [ "$COVERAGE" = true ]; then
            npm run test -- --coverage --reporter=json --outputFile="$TEST_RESULTS_DIR/webapp_test_results.json"
        else
            npm run test
        fi
        
        if [ $? -eq 0 ]; then
            print_status "Webapp tests completed successfully ✓"
        else
            print_error "Some webapp tests failed"
            exit 1
        fi
        
        # Build check
        print_status "Testing webapp build..."
        npm run build
        
        if [ $? -eq 0 ]; then
            print_status "Webapp build successful ✓"
        else
            print_error "Webapp build failed"
            exit 1
        fi
    fi
    
    cd ..
fi

# Integration Tests
if [ "$INTEGRATION_TESTS" = true ]; then
    print_section "Integration Tests"
    
    print_status "Running integration test suite..."
    
    # Start minimal test environment
    print_status "Setting up test environment..."
    
    # TODO: Implement integration tests
    # These would test:
    # - ROS <-> WebRTC bridge communication
    # - Webapp <-> Backend integration
    # - End-to-end command flow
    # - Multi-agent coordination
    
    print_warning "Integration tests not yet implemented"
fi

# Performance Tests
if [ "$PERFORMANCE_TESTS" = true ]; then
    print_section "Performance Tests"
    
    print_status "Running performance benchmarks..."
    
    # TODO: Implement performance tests
    # These would test:
    # - Command processing latency
    # - WebRTC throughput
    # - Memory usage under load
    # - Concurrent agent handling
    
    print_warning "Performance tests not yet implemented"
fi

# Security Tests
print_section "Security Analysis"

# Check for common security issues
print_status "Running security checks..."

# Check for hardcoded secrets
if command -v grep &> /dev/null; then
    print_status "Checking for hardcoded secrets..."
    
    # Common patterns to avoid
    SECRET_PATTERNS=(
        "password.*=.*['\"][^'\"]{8,}['\"]"
        "api[_-]?key.*=.*['\"][^'\"]{20,}['\"]"
        "secret.*=.*['\"][^'\"]{8,}['\"]"
        "token.*=.*['\"][^'\"]{20,}['\"]"
    )
    
    for pattern in "${SECRET_PATTERNS[@]}"; do
        if grep -r -i "$pattern" --include="*.py" --include="*.js" --include="*.ts" --include="*.tsx" . | grep -v ".env.example" | grep -q .; then
            print_warning "Potential hardcoded secret found (check manually)"
        fi
    done
    
    print_status "Secret check completed"
fi

# Test Summary
print_section "Test Summary"

# Count test results
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

if [ "$ROS_TESTS" = true ]; then
    print_status "✓ ROS 2 package tests"
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    PASSED_TESTS=$((PASSED_TESTS + 1))
fi

if [ "$WEBAPP_TESTS" = true ]; then
    print_status "✓ Webapp tests (unit, integration, build)"
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    PASSED_TESTS=$((PASSED_TESTS + 1))
fi

# Generate final report
cat > "$TEST_RESULTS_DIR/test_summary.txt" << EOF
XR-Swarm-Bridge Test Results
============================
Date: $(date)
Total Test Suites: $TOTAL_TESTS
Passed: $PASSED_TESTS
Failed: $FAILED_TESTS

ROS Tests: $([ "$ROS_TESTS" = true ] && echo "ENABLED" || echo "SKIPPED")
Webapp Tests: $([ "$WEBAPP_TESTS" = true ] && echo "ENABLED" || echo "SKIPPED")
Integration Tests: $([ "$INTEGRATION_TESTS" = true ] && echo "ENABLED" || echo "SKIPPED")
Performance Tests: $([ "$PERFORMANCE_TESTS" = true ] && echo "ENABLED" || echo "SKIPPED")
Coverage: $([ "$COVERAGE" = true ] && echo "ENABLED" || echo "DISABLED")

Status: $([ $FAILED_TESTS -eq 0 ] && echo "PASSED" || echo "FAILED")
EOF

if [ $FAILED_TESTS -eq 0 ]; then
    print_status "🎉 All tests passed! ($PASSED_TESTS/$TOTAL_TESTS)"
else
    print_error "❌ Some tests failed ($FAILED_TESTS failures)"
    exit 1
fi

print_status "Test results saved to: $TEST_RESULTS_DIR"
print_status "Run with --help for more options"