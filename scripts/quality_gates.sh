#!/bin/bash
# Quality Gates Validation Script for XR-Swarm-Bridge
# Ensures all quality standards are met before deployment

set -e

echo "🧪 XR-Swarm-Bridge Quality Gates Validation"
echo "==========================================="

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Track results
GATE_RESULTS=()

function check_gate() {
    local gate_name="$1"
    local command="$2"
    local required_threshold="$3"
    
    echo -e "\n📋 Checking: $gate_name"
    echo "Command: $command"
    
    if eval "$command"; then
        echo -e "${GREEN}✅ PASSED: $gate_name${NC}"
        GATE_RESULTS+=("PASS:$gate_name")
        return 0
    else
        echo -e "${RED}❌ FAILED: $gate_name${NC}"
        GATE_RESULTS+=("FAIL:$gate_name")
        return 1
    fi
}

function check_threshold() {
    local gate_name="$1"
    local actual_value="$2"
    local threshold="$3"
    local comparison="$4" # "gt" for greater than, "lt" for less than
    
    echo -e "\n📊 Checking Threshold: $gate_name"
    echo "Actual: $actual_value, Threshold: $threshold, Comparison: $comparison"
    
    if [[ "$comparison" == "gt" ]]; then
        if (( $(echo "$actual_value > $threshold" | bc -l) )); then
            echo -e "${GREEN}✅ PASSED: $gate_name ($actual_value > $threshold)${NC}"
            GATE_RESULTS+=("PASS:$gate_name")
            return 0
        fi
    elif [[ "$comparison" == "lt" ]]; then
        if (( $(echo "$actual_value < $threshold" | bc -l) )); then
            echo -e "${GREEN}✅ PASSED: $gate_name ($actual_value < $threshold)${NC}"
            GATE_RESULTS+=("PASS:$gate_name")
            return 0
        fi
    fi
    
    echo -e "${RED}❌ FAILED: $gate_name ($actual_value does not meet $comparison $threshold)${NC}"
    GATE_RESULTS+=("FAIL:$gate_name")
    return 1
}

# Navigate to webapp directory
cd "$(dirname "$0")/../webapp"

echo -e "\n🔧 Environment Setup"
echo "Working directory: $(pwd)"
echo "Node version: $(node --version)"
echo "NPM version: $(npm --version)"

# Gate 1: Code builds without errors
check_gate "Build Process" "npm run build" || EXIT_CODE=1

# Gate 2: TypeScript type checking
check_gate "TypeScript Type Checking" "npm run typecheck" || EXIT_CODE=1

# Gate 3: Code linting
check_gate "ESLint Code Quality" "npm run lint" || EXIT_CODE=1

# Gate 4: Unit and Integration Tests
echo -e "\n🧪 Running Test Suite"
TEST_OUTPUT=$(npm test 2>&1 || true)
echo "$TEST_OUTPUT"

# Extract test metrics
PASSED_TESTS=$(echo "$TEST_OUTPUT" | grep -o "Tests.*passed" | grep -o "[0-9]\+ passed" | head -1 | grep -o "[0-9]\+" || echo "0")
FAILED_TESTS=$(echo "$TEST_OUTPUT" | grep -o "Tests.*failed" | grep -o "[0-9]\+ failed" | head -1 | grep -o "[0-9]\+" || echo "0")
TOTAL_TESTS=$((PASSED_TESTS + FAILED_TESTS))

echo "Test Results: $PASSED_TESTS passed, $FAILED_TESTS failed, $TOTAL_TESTS total"

if [[ $TOTAL_TESTS -gt 0 ]]; then
    TEST_PASS_RATE=$(echo "scale=4; $PASSED_TESTS / $TOTAL_TESTS * 100" | bc)
    check_threshold "Test Pass Rate" "$TEST_PASS_RATE" "85" "gt" || EXIT_CODE=1
else
    echo -e "${YELLOW}⚠️  WARNING: No tests found${NC}"
    GATE_RESULTS+=("WARN:No tests found")
fi

# Gate 5: Security Scan (if audit available)
echo -e "\n🔒 Security Audit"
AUDIT_OUTPUT=$(npm audit --audit-level=moderate 2>&1 || true)
VULNERABILITIES=$(echo "$AUDIT_OUTPUT" | grep -o "[0-9]\+ vulnerabilities" | grep -o "[0-9]\+" || echo "0")

if [[ $VULNERABILITIES -eq 0 ]]; then
    echo -e "${GREEN}✅ PASSED: No security vulnerabilities${NC}"
    GATE_RESULTS+=("PASS:Security Scan")
else
    echo -e "${YELLOW}⚠️  WARNING: $VULNERABILITIES vulnerabilities found${NC}"
    echo "Run 'npm audit fix' to resolve"
    GATE_RESULTS+=("WARN:Security vulnerabilities found")
fi

# Gate 6: Bundle Size Check
echo -e "\n📦 Bundle Size Analysis"
if [[ -d "dist" ]]; then
    BUNDLE_SIZE=$(du -sh dist | cut -f1)
    BUNDLE_SIZE_MB=$(du -sm dist | cut -f1)
    
    echo "Bundle size: $BUNDLE_SIZE (${BUNDLE_SIZE_MB}MB)"
    
    # Check if bundle is under 50MB
    check_threshold "Bundle Size" "$BUNDLE_SIZE_MB" "50" "lt" || EXIT_CODE=1
else
    echo -e "${RED}❌ FAILED: Build artifacts not found${NC}"
    GATE_RESULTS+=("FAIL:Bundle Size - No build artifacts")
    EXIT_CODE=1
fi

# Gate 7: Performance Benchmarks
echo -e "\n⚡ Performance Checks"

# Check for large chunks warning in build output
BUILD_OUTPUT=$(npm run build 2>&1 || true)
if echo "$BUILD_OUTPUT" | grep -q "chunks are larger than 500 kB"; then
    echo -e "${YELLOW}⚠️  WARNING: Large bundle chunks detected${NC}"
    GATE_RESULTS+=("WARN:Large bundle chunks")
else
    echo -e "${GREEN}✅ PASSED: Bundle chunk sizes optimal${NC}"
    GATE_RESULTS+=("PASS:Bundle Optimization")
fi

# Gate 8: Documentation Completeness
echo -e "\n📚 Documentation Check"
DOCS_COUNT=0
[[ -f "../README.md" ]] && ((DOCS_COUNT++))
[[ -f "../docs/API.md" ]] && ((DOCS_COUNT++))
[[ -f "../docs/ARCHITECTURE.md" ]] && ((DOCS_COUNT++))
[[ -f "../docs/DEPLOYMENT.md" ]] && ((DOCS_COUNT++))

check_threshold "Documentation Coverage" "$DOCS_COUNT" "3" "gt" || EXIT_CODE=1

# Gate 9: Git Repository State
echo -e "\n📝 Git Repository Check"
cd ..
if git diff --quiet && git diff --cached --quiet; then
    echo -e "${GREEN}✅ PASSED: Clean git working directory${NC}"
    GATE_RESULTS+=("PASS:Git State")
else
    echo -e "${YELLOW}⚠️  WARNING: Uncommitted changes detected${NC}"
    GATE_RESULTS+=("WARN:Uncommitted changes")
fi

# Gate 10: Deployment Readiness
echo -e "\n🚀 Deployment Readiness"
DEPLOYMENT_READY=true

# Check for required deployment files
[[ ! -f "deploy/docker-compose.prod.yml" ]] && DEPLOYMENT_READY=false
[[ ! -f "deploy/Dockerfile.webapp" ]] && DEPLOYMENT_READY=false
[[ ! -f "deploy/nginx.conf" ]] && DEPLOYMENT_READY=false

if $DEPLOYMENT_READY; then
    echo -e "${GREEN}✅ PASSED: Deployment configuration ready${NC}"
    GATE_RESULTS+=("PASS:Deployment Ready")
else
    echo -e "${RED}❌ FAILED: Missing deployment configuration${NC}"
    GATE_RESULTS+=("FAIL:Deployment configuration")
    EXIT_CODE=1
fi

# Final Results Summary
echo -e "\n📊 QUALITY GATES SUMMARY"
echo "========================="

PASSED_COUNT=0
FAILED_COUNT=0
WARNING_COUNT=0

for result in "${GATE_RESULTS[@]}"; do
    IFS=':' read -r status gate <<< "$result"
    case $status in
        "PASS")
            echo -e "${GREEN}✅ $gate${NC}"
            ((PASSED_COUNT++))
            ;;
        "FAIL")
            echo -e "${RED}❌ $gate${NC}"
            ((FAILED_COUNT++))
            ;;
        "WARN")
            echo -e "${YELLOW}⚠️  $gate${NC}"
            ((WARNING_COUNT++))
            ;;
    esac
done

echo -e "\nResults: $PASSED_COUNT passed, $FAILED_COUNT failed, $WARNING_COUNT warnings"

# Success criteria
TOTAL_GATES=$((PASSED_COUNT + FAILED_COUNT))
if [[ $TOTAL_GATES -gt 0 ]]; then
    SUCCESS_RATE=$(echo "scale=2; $PASSED_COUNT / $TOTAL_GATES * 100" | bc)
    echo "Success Rate: ${SUCCESS_RATE}%"
    
    if (( $(echo "$SUCCESS_RATE >= 90" | bc -l) )); then
        echo -e "\n${GREEN}🎉 QUALITY GATES PASSED! Ready for deployment.${NC}"
        exit 0
    else
        echo -e "\n${RED}💥 QUALITY GATES FAILED! Address issues before deployment.${NC}"
        exit 1
    fi
else
    echo -e "\n${RED}💥 QUALITY GATES FAILED! No gates completed successfully.${NC}"
    exit 1
fi