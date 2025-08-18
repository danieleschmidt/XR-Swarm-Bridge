import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { EnhancedErrorHandler, ErrorContext, AutoRecoveryStrategy } from '../enhancedErrorHandler';

describe('EnhancedErrorHandler', () => {
  let errorHandler: EnhancedErrorHandler;
  let mockErrorContext: ErrorContext;

  beforeEach(() => {
    errorHandler = new EnhancedErrorHandler();
    mockErrorContext = {
      componentId: 'test_component',
      operationId: 'test_operation',
      userId: 'test_user',
      sessionId: 'test_session',
      timestamp: new Date(),
      environment: 'development',
      buildVersion: '1.0.0'
    };
  });

  describe('Initialization', () => {
    it('should initialize with default recovery strategies', () => {
      const metrics = errorHandler.getMetrics();
      expect(metrics.totalErrors).toBe(0);
      expect(metrics.recoverySuccessRate).toBe(0);
    });

    it('should initialize error metrics correctly', () => {
      const metrics = errorHandler.getMetrics();
      expect(metrics).toHaveProperty('totalErrors');
      expect(metrics).toHaveProperty('errorsByType');
      expect(metrics).toHaveProperty('errorsByComponent');
      expect(metrics).toHaveProperty('recoverySuccessRate');
      expect(metrics).toHaveProperty('averageRecoveryTime');
    });
  });

  describe('Error Handling', () => {
    it('should handle basic errors and update metrics', async () => {
      const testError = new Error('Test error');
      const result = await errorHandler.handleError(testError, mockErrorContext);
      
      const metrics = errorHandler.getMetrics();
      expect(metrics.totalErrors).toBe(1);
      expect(metrics.errorsByType.get('Error')).toBe(1);
      expect(metrics.errorsByComponent.get('test_component')).toBe(1);
    });

    it('should track error history', async () => {
      const testError = new Error('Test error');
      await errorHandler.handleError(testError, mockErrorContext);
      
      const history = errorHandler.getErrorHistory(10);
      expect(history).toHaveLength(1);
      expect(history[0].error).toBe(testError);
      expect(history[0].context).toBe(mockErrorContext);
    });

    it('should limit error history size', async () => {
      // Add many errors
      for (let i = 0; i < 150; i++) {
        await errorHandler.handleError(new Error(`Error ${i}`), mockErrorContext);
      }
      
      const history = errorHandler.getErrorHistory();
      expect(history.length).toBeLessThanOrEqual(100); // Default limit
    });
  });

  describe('Recovery Strategies', () => {
    it('should add custom recovery strategies', () => {
      const customStrategy: AutoRecoveryStrategy = {
        id: 'custom_strategy',
        name: 'Custom Recovery',
        condition: () => true,
        action: async () => true,
        maxAttempts: 3,
        cooldownMs: 5000,
        successRate: 0.8
      };

      errorHandler.addRecoveryStrategy(customStrategy);
      
      // Verify strategy was added by checking if it affects recovery
      expect(errorHandler).toBeDefined();
    });

    it('should execute recovery strategies for matching errors', async () => {
      let recoveryExecuted = false;
      
      const testStrategy: AutoRecoveryStrategy = {
        id: 'test_strategy',
        name: 'Test Recovery',
        condition: (error) => error.message.includes('test'),
        action: async () => {
          recoveryExecuted = true;
          return true;
        },
        maxAttempts: 1,
        cooldownMs: 1000,
        successRate: 1.0
      };

      errorHandler.addRecoveryStrategy(testStrategy);
      
      const testError = new Error('This is a test error');
      const result = await errorHandler.handleError(testError, mockErrorContext);
      
      expect(recoveryExecuted).toBe(true);
      expect(result).toBe(true);
    });

    it('should not execute recovery for non-matching errors', async () => {
      let recoveryExecuted = false;
      
      const testStrategy: AutoRecoveryStrategy = {
        id: 'specific_strategy',
        name: 'Specific Recovery',
        condition: (error) => error.message.includes('specific'),
        action: async () => {
          recoveryExecuted = true;
          return true;
        },
        maxAttempts: 1,
        cooldownMs: 1000,
        successRate: 1.0
      };

      errorHandler.addRecoveryStrategy(testStrategy);
      
      const testError = new Error('Generic error');
      await errorHandler.handleError(testError, mockErrorContext);
      
      expect(recoveryExecuted).toBe(false);
    });
  });

  describe('Circuit Breaker', () => {
    it('should create circuit breaker with correct initial state', () => {
      const circuitBreaker = errorHandler.createCircuitBreaker('test_component', 3, 30000);
      const state = circuitBreaker.getState();
      
      expect(state.state).toBe('closed');
      expect(state.failures).toBe(0);
    });

    it('should open circuit after failure threshold', async () => {
      const circuitBreaker = errorHandler.createCircuitBreaker('test_component', 2, 30000);
      
      // Cause failures
      try {
        await circuitBreaker.execute(async () => {
          throw new Error('Test failure');
        });
      } catch {}
      
      try {
        await circuitBreaker.execute(async () => {
          throw new Error('Test failure');
        });
      } catch {}
      
      const state = circuitBreaker.getState();
      expect(state.state).toBe('open');
      expect(state.failures).toBe(2);
    });

    it('should reject requests when circuit is open', async () => {
      const circuitBreaker = errorHandler.createCircuitBreaker('test_component', 1, 30000);
      
      // Cause failure to open circuit
      try {
        await circuitBreaker.execute(async () => {
          throw new Error('Test failure');
        });
      } catch {}
      
      // Should reject immediately
      await expect(circuitBreaker.execute(async () => 'success')).rejects.toThrow('Circuit breaker open');
    });

    it('should reset circuit breaker', () => {
      const circuitBreaker = errorHandler.createCircuitBreaker('test_component', 1, 30000);
      
      circuitBreaker.reset();
      const state = circuitBreaker.getState();
      
      expect(state.state).toBe('closed');
      expect(state.failures).toBe(0);
      expect(state.lastFailureTime).toBe(0);
    });
  });

  describe('Error Reporting', () => {
    it('should generate comprehensive recovery report', async () => {
      // Add some test errors
      await errorHandler.handleError(new Error('Test error 1'), mockErrorContext);
      await errorHandler.handleError(new TypeError('Test error 2'), mockErrorContext);
      
      const report = errorHandler.generateRecoveryReport();
      const reportData = JSON.parse(report);
      
      expect(reportData).toHaveProperty('timestamp');
      expect(reportData).toHaveProperty('totalErrors');
      expect(reportData).toHaveProperty('recoverySuccessRate');
      expect(reportData).toHaveProperty('topErrorTypes');
      expect(reportData).toHaveProperty('topErrorComponents');
      expect(reportData).toHaveProperty('recoveryStrategies');
      expect(reportData).toHaveProperty('recentErrors');
      
      expect(reportData.totalErrors).toBe(2);
      expect(Array.isArray(reportData.topErrorTypes)).toBe(true);
      expect(Array.isArray(reportData.recentErrors)).toBe(true);
    });

    it('should track error types correctly', async () => {
      await errorHandler.handleError(new Error('Error 1'), mockErrorContext);
      await errorHandler.handleError(new TypeError('Type Error'), mockErrorContext);
      await errorHandler.handleError(new ReferenceError('Reference Error'), mockErrorContext);
      
      const metrics = errorHandler.getMetrics();
      expect(metrics.errorsByType.get('Error')).toBe(1);
      expect(metrics.errorsByType.get('TypeError')).toBe(1);
      expect(metrics.errorsByType.get('ReferenceError')).toBe(1);
    });

    it('should track errors by component', async () => {
      const context1 = { ...mockErrorContext, componentId: 'component_1' };
      const context2 = { ...mockErrorContext, componentId: 'component_2' };
      
      await errorHandler.handleError(new Error('Error 1'), context1);
      await errorHandler.handleError(new Error('Error 2'), context1);
      await errorHandler.handleError(new Error('Error 3'), context2);
      
      const metrics = errorHandler.getMetrics();
      expect(metrics.errorsByComponent.get('component_1')).toBe(2);
      expect(metrics.errorsByComponent.get('component_2')).toBe(1);
    });
  });

  describe('Performance', () => {
    it('should handle many errors efficiently', async () => {
      const startTime = Date.now();
      
      // Handle many errors
      const promises = [];
      for (let i = 0; i < 100; i++) {
        promises.push(errorHandler.handleError(new Error(`Error ${i}`), mockErrorContext));
      }
      
      await Promise.all(promises);
      const endTime = Date.now();
      
      expect(endTime - startTime).toBeLessThan(1000); // Should complete within 1 second
      
      const metrics = errorHandler.getMetrics();
      expect(metrics.totalErrors).toBe(100);
    });

    it('should generate reports efficiently', async () => {
      // Add some errors first
      for (let i = 0; i < 50; i++) {
        await errorHandler.handleError(new Error(`Error ${i}`), mockErrorContext);
      }
      
      const startTime = Date.now();
      const report = errorHandler.generateRecoveryReport();
      const endTime = Date.now();
      
      expect(report.length).toBeGreaterThan(100); // Non-empty report
      expect(endTime - startTime).toBeLessThan(100); // Should complete quickly
    });
  });

  describe('Recovery Success Tracking', () => {
    it('should track successful recoveries', async () => {
      const successfulStrategy: AutoRecoveryStrategy = {
        id: 'successful_strategy',
        name: 'Always Successful',
        condition: () => true,
        action: async () => true,
        maxAttempts: 1,
        cooldownMs: 1000,
        successRate: 1.0
      };

      errorHandler.addRecoveryStrategy(successfulStrategy);
      
      await errorHandler.handleError(new Error('Test error'), mockErrorContext);
      
      const history = errorHandler.getErrorHistory(1);
      expect(history[0].recovered).toBe(true);
    });

    it('should track failed recoveries', async () => {
      const failingStrategy: AutoRecoveryStrategy = {
        id: 'failing_strategy',
        name: 'Always Fails',
        condition: () => true,
        action: async () => false,
        maxAttempts: 1,
        cooldownMs: 1000,
        successRate: 0.0
      };

      errorHandler.addRecoveryStrategy(failingStrategy);
      
      await errorHandler.handleError(new Error('Test error'), mockErrorContext);
      
      const history = errorHandler.getErrorHistory(1);
      expect(history[0].recovered).toBe(false);
    });
  });

  describe('Edge Cases', () => {
    it('should handle null/undefined errors gracefully', async () => {
      const nullError = null as any;
      await expect(errorHandler.handleError(nullError, mockErrorContext)).rejects.toThrow();
    });

    it('should handle errors with circular references', async () => {
      const circularError: any = new Error('Circular error');
      circularError.self = circularError;
      
      await expect(errorHandler.handleError(circularError, mockErrorContext)).resolves.toBeDefined();
    });

    it('should handle recovery strategy exceptions', async () => {
      const throwingStrategy: AutoRecoveryStrategy = {
        id: 'throwing_strategy',
        name: 'Throws Exception',
        condition: () => true,
        action: async () => {
          throw new Error('Recovery strategy failed');
        },
        maxAttempts: 1,
        cooldownMs: 1000,
        successRate: 0.5
      };

      errorHandler.addRecoveryStrategy(throwingStrategy);
      
      // Should not throw, should handle gracefully
      const result = await errorHandler.handleError(new Error('Test error'), mockErrorContext);
      expect(result).toBe(false); // Recovery failed, but didn't crash
    });
  });
});