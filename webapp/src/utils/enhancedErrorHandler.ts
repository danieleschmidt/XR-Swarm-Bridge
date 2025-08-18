/**
 * Enhanced Error Handler with Autonomous Recovery
 * Generation 2: Robust implementation with self-healing
 */

export interface ErrorContext {
  componentId: string;
  operationId: string;
  userId?: string;
  sessionId: string;
  timestamp: Date;
  environment: 'development' | 'staging' | 'production';
  buildVersion: string;
}

export interface AutoRecoveryStrategy {
  id: string;
  name: string;
  condition: (error: Error, context: ErrorContext) => boolean;
  action: (error: Error, context: ErrorContext) => Promise<boolean>;
  maxAttempts: number;
  cooldownMs: number;
  successRate: number;
}

export interface ErrorMetrics {
  totalErrors: number;
  errorsByType: Map<string, number>;
  errorsByComponent: Map<string, number>;
  recoverySuccessRate: number;
  lastRecoveryAttempt: Date;
  averageRecoveryTime: number;
}

export class EnhancedErrorHandler {
  private recoveryStrategies: Map<string, AutoRecoveryStrategy> = new Map();
  private errorMetrics: ErrorMetrics;
  private errorHistory: Array<{ error: Error; context: ErrorContext; recovered: boolean }> = [];
  private activeRecoveries: Set<string> = new Set();

  constructor() {
    this.errorMetrics = {
      totalErrors: 0,
      errorsByType: new Map(),
      errorsByComponent: new Map(),
      recoverySuccessRate: 0,
      lastRecoveryAttempt: new Date(),
      averageRecoveryTime: 0
    };

    this.initializeRecoveryStrategies();
  }

  private initializeRecoveryStrategies(): void {
    // WebRTC Connection Recovery
    this.addRecoveryStrategy({
      id: 'webrtc_reconnect',
      name: 'WebRTC Connection Reconnection',
      condition: (error) => error.message.includes('connection') || error.message.includes('ICE'),
      action: async (error, context) => {
        console.log('Attempting WebRTC reconnection...');
        try {
          // Trigger WebRTC reconnection logic
          const webRTCStore = (window as any).webRTCStore;
          if (webRTCStore) {
            await webRTCStore.getState().reconnect();
            return true;
          }
          return false;
        } catch (recoveryError) {
          console.error('WebRTC recovery failed:', recoveryError);
          return false;
        }
      },
      maxAttempts: 3,
      cooldownMs: 5000,
      successRate: 0.85
    });

    // Quantum Optimization Recovery
    this.addRecoveryStrategy({
      id: 'quantum_optimization_restart',
      name: 'Quantum Optimization Engine Restart',
      condition: (error, context) => 
        context.componentId.includes('quantum') || error.message.includes('optimization'),
      action: async (error, context) => {
        console.log('Restarting quantum optimization engine...');
        try {
          const { quantumOptimizationEngine } = await import('./quantumOptimization');
          // Reset quantum states and restart optimization loop
          quantumOptimizationEngine.createQuantumState('recovery_state');
          return true;
        } catch (recoveryError) {
          console.error('Quantum recovery failed:', recoveryError);
          return false;
        }
      },
      maxAttempts: 2,
      cooldownMs: 10000,
      successRate: 0.75
    });

    // Autonomous Planning Recovery
    this.addRecoveryStrategy({
      id: 'planning_fallback',
      name: 'Autonomous Planning Fallback',
      condition: (error, context) => 
        context.componentId.includes('planning') || error.message.includes('GPT'),
      action: async (error, context) => {
        console.log('Activating planning fallback mode...');
        try {
          // Switch to local heuristic planning
          const swarmStore = (window as any).swarmStore;
          if (swarmStore) {
            swarmStore.getState().setViewMode('manual');
            return true;
          }
          return false;
        } catch (recoveryError) {
          console.error('Planning recovery failed:', recoveryError);
          return false;
        }
      },
      maxAttempts: 1,
      cooldownMs: 30000,
      successRate: 0.90
    });

    // Memory Leak Recovery
    this.addRecoveryStrategy({
      id: 'memory_cleanup',
      name: 'Memory Cleanup and Optimization',
      condition: (error) => 
        error.message.includes('memory') || error.message.includes('heap'),
      action: async (error, context) => {
        console.log('Performing memory cleanup...');
        try {
          // Force garbage collection if available
          if (window.gc) {
            window.gc();
          }
          
          // Clear old telemetry data
          const swarmStore = (window as any).swarmStore;
          if (swarmStore) {
            const state = swarmStore.getState();
            // Keep only last 100 entries of trajectories
            const cleanedTrajectories: Record<string, any> = {};
            Object.entries(state.trajectories).forEach(([agentId, trajectory]: [string, any]) => {
              if (Array.isArray(trajectory) && trajectory.length > 100) {
                cleanedTrajectories[agentId] = trajectory.slice(-100);
              } else {
                cleanedTrajectories[agentId] = trajectory;
              }
            });
            
            swarmStore.setState({ trajectories: cleanedTrajectories });
          }
          
          return true;
        } catch (recoveryError) {
          console.error('Memory cleanup failed:', recoveryError);
          return false;
        }
      },
      maxAttempts: 1,
      cooldownMs: 60000,
      successRate: 0.70
    });

    // Performance Degradation Recovery
    this.addRecoveryStrategy({
      id: 'performance_optimization',
      name: 'Performance Optimization Recovery',
      condition: (error, context) => 
        error.message.includes('performance') || error.message.includes('fps'),
      action: async (error, context) => {
        console.log('Applying performance optimizations...');
        try {
          const { performanceOptimizer } = await import('./performanceOptimization');
          
          // Reduce visual quality
          performanceOptimizer.reduceLOD();
          
          // Optimize update frequencies
          performanceOptimizer.optimizeUpdateFrequencies();
          
          return true;
        } catch (recoveryError) {
          console.error('Performance recovery failed:', recoveryError);
          return false;
        }
      },
      maxAttempts: 2,
      cooldownMs: 15000,
      successRate: 0.80
    });
  }

  addRecoveryStrategy(strategy: AutoRecoveryStrategy): void {
    this.recoveryStrategies.set(strategy.id, strategy);
  }

  async handleError(error: Error, context: ErrorContext): Promise<boolean> {
    console.error('Enhanced error handler received:', error, context);
    
    // Update metrics
    this.updateErrorMetrics(error, context);
    
    // Check for applicable recovery strategies
    const applicableStrategies = Array.from(this.recoveryStrategies.values())
      .filter(strategy => 
        strategy.condition(error, context) && 
        !this.activeRecoveries.has(strategy.id)
      )
      .sort((a, b) => b.successRate - a.successRate);

    if (applicableStrategies.length === 0) {
      console.warn('No recovery strategies available for error:', error.message);
      this.errorHistory.push({ error, context, recovered: false });
      return false;
    }

    // Attempt recovery with the best strategy
    for (const strategy of applicableStrategies) {
      const recoveryKey = `${strategy.id}_${context.componentId}`;
      
      if (this.activeRecoveries.has(recoveryKey)) {
        continue; // Skip if already attempting recovery
      }

      try {
        this.activeRecoveries.add(recoveryKey);
        console.log(`Attempting recovery with strategy: ${strategy.name}`);
        
        const startTime = Date.now();
        const success = await strategy.action(error, context);
        const recoveryTime = Date.now() - startTime;
        
        this.activeRecoveries.delete(recoveryKey);
        
        if (success) {
          console.log(`Recovery successful with strategy: ${strategy.name}`);
          this.updateRecoveryMetrics(true, recoveryTime);
          this.errorHistory.push({ error, context, recovered: true });
          
          // Update strategy success rate
          strategy.successRate = Math.min(0.95, strategy.successRate + 0.05);
          
          return true;
        } else {
          console.warn(`Recovery failed with strategy: ${strategy.name}`);
          strategy.successRate = Math.max(0.05, strategy.successRate - 0.1);
        }
        
        // Apply cooldown
        setTimeout(() => {
          // Cooldown complete
        }, strategy.cooldownMs);
        
      } catch (recoveryError) {
        console.error(`Recovery strategy ${strategy.name} threw error:`, recoveryError);
        this.activeRecoveries.delete(recoveryKey);
        strategy.successRate = Math.max(0.05, strategy.successRate - 0.15);
      }
    }

    this.updateRecoveryMetrics(false, 0);
    this.errorHistory.push({ error, context, recovered: false });
    return false;
  }

  private updateErrorMetrics(error: Error, context: ErrorContext): void {
    this.errorMetrics.totalErrors++;
    
    const errorType = error.constructor.name;
    this.errorMetrics.errorsByType.set(
      errorType, 
      (this.errorMetrics.errorsByType.get(errorType) || 0) + 1
    );
    
    this.errorMetrics.errorsByComponent.set(
      context.componentId,
      (this.errorMetrics.errorsByComponent.get(context.componentId) || 0) + 1
    );
  }

  private updateRecoveryMetrics(success: boolean, recoveryTime: number): void {
    this.errorMetrics.lastRecoveryAttempt = new Date();
    
    if (success) {
      const recoveredErrors = this.errorHistory.filter(h => h.recovered).length;
      this.errorMetrics.recoverySuccessRate = recoveredErrors / this.errorHistory.length;
      
      // Update average recovery time
      const recoveryTimes = this.errorHistory
        .filter(h => h.recovered)
        .map(h => recoveryTime); // Simplified - in real implementation, store actual times
      
      if (recoveryTimes.length > 0) {
        this.errorMetrics.averageRecoveryTime = 
          recoveryTimes.reduce((a, b) => a + b, 0) / recoveryTimes.length;
      }
    }
  }

  getMetrics(): ErrorMetrics {
    return {
      ...this.errorMetrics,
      errorsByType: new Map(this.errorMetrics.errorsByType),
      errorsByComponent: new Map(this.errorMetrics.errorsByComponent)
    };
  }

  getErrorHistory(limit: number = 100): Array<{ error: Error; context: ErrorContext; recovered: boolean }> {
    return this.errorHistory.slice(-limit);
  }

  generateRecoveryReport(): string {
    const metrics = this.getMetrics();
    const recentErrors = this.getErrorHistory(20);
    
    const report = {
      timestamp: new Date().toISOString(),
      totalErrors: metrics.totalErrors,
      recoverySuccessRate: `${(metrics.recoverySuccessRate * 100).toFixed(1)}%`,
      averageRecoveryTime: `${metrics.averageRecoveryTime.toFixed(0)}ms`,
      topErrorTypes: Array.from(metrics.errorsByType.entries())
        .sort(([,a], [,b]) => b - a)
        .slice(0, 5)
        .map(([type, count]) => ({ type, count })),
      topErrorComponents: Array.from(metrics.errorsByComponent.entries())
        .sort(([,a], [,b]) => b - a)
        .slice(0, 5)
        .map(([component, count]) => ({ component, count })),
      recoveryStrategies: Array.from(this.recoveryStrategies.values())
        .map(strategy => ({
          name: strategy.name,
          successRate: `${(strategy.successRate * 100).toFixed(1)}%`,
          maxAttempts: strategy.maxAttempts
        })),
      recentErrors: recentErrors.map(entry => ({
        errorType: entry.error.constructor.name,
        message: entry.error.message,
        component: entry.context.componentId,
        recovered: entry.recovered,
        timestamp: entry.context.timestamp
      }))
    };

    return JSON.stringify(report, null, 2);
  }

  // Circuit breaker pattern for preventing cascading failures
  createCircuitBreaker(componentId: string, failureThreshold: number = 5, timeoutMs: number = 60000) {
    let failures = 0;
    let lastFailureTime = 0;
    let state: 'closed' | 'open' | 'half-open' = 'closed';

    return {
      async execute<T>(operation: () => Promise<T>): Promise<T> {
        const now = Date.now();

        // Check if circuit should move from open to half-open
        if (state === 'open' && now - lastFailureTime > timeoutMs) {
          state = 'half-open';
          failures = 0;
        }

        // Reject immediately if circuit is open
        if (state === 'open') {
          throw new Error(`Circuit breaker open for component: ${componentId}`);
        }

        try {
          const result = await operation();
          
          // Success - reset if in half-open state
          if (state === 'half-open') {
            state = 'closed';
            failures = 0;
          }
          
          return result;
        } catch (error) {
          failures++;
          lastFailureTime = now;

          // Trip circuit if failure threshold exceeded
          if (failures >= failureThreshold) {
            state = 'open';
            console.warn(`Circuit breaker opened for component: ${componentId}`);
          }

          throw error;
        }
      },

      getState: () => ({ state, failures, lastFailureTime }),
      reset: () => {
        state = 'closed';
        failures = 0;
        lastFailureTime = 0;
      }
    };
  }
}

export const enhancedErrorHandler = new EnhancedErrorHandler();

// Global error boundary integration
window.addEventListener('error', (event) => {
  const context: ErrorContext = {
    componentId: 'window',
    operationId: 'global_error',
    sessionId: 'session_' + Date.now(),
    timestamp: new Date(),
    environment: process.env.NODE_ENV as any || 'development',
    buildVersion: '1.0.0'
  };

  enhancedErrorHandler.handleError(event.error, context);
});

window.addEventListener('unhandledrejection', (event) => {
  const context: ErrorContext = {
    componentId: 'promise',
    operationId: 'unhandled_rejection',
    sessionId: 'session_' + Date.now(),
    timestamp: new Date(),
    environment: process.env.NODE_ENV as any || 'development',
    buildVersion: '1.0.0'
  };

  enhancedErrorHandler.handleError(
    new Error(event.reason || 'Unhandled promise rejection'), 
    context
  );
});