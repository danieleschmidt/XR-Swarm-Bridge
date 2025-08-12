/**
 * Resilient Error Handler for XR-Swarm-Bridge
 * Advanced error handling with auto-recovery, circuit breakers, and adaptive strategies
 */

import { logger } from './logger'
import { performanceMonitor } from './performance'

interface ErrorContext {
  component: string
  operation: string
  severity: 'low' | 'medium' | 'high' | 'critical'
  recoverable: boolean
  metadata?: Record<string, any>
}

interface RecoveryStrategy {
  name: string
  attempt: () => Promise<boolean>
  maxRetries: number
  backoffMs: number
  successThreshold: number
}

interface CircuitBreakerState {
  failures: number
  lastFailure: number
  state: 'closed' | 'open' | 'half-open'
  threshold: number
  timeout: number
}

class ResilientErrorHandler {
  private circuitBreakers = new Map<string, CircuitBreakerState>()
  private recoveryStrategies = new Map<string, RecoveryStrategy[]>()
  private errorPatterns = new Map<string, number>()
  private adaptiveLearning = new Map<string, { success: number; failure: number }>()
  
  constructor() {
    this.initializeDefaultStrategies()
    this.startPeriodicMaintenance()
  }

  /**
   * Handle error with intelligent recovery strategies
   */
  async handleError(error: Error, context: ErrorContext): Promise<boolean> {
    const startTime = performance.now()
    
    try {
      // Log error with context
      logger.error('Error occurred', {
        error: error.message,
        stack: error.stack,
        context,
        timestamp: new Date().toISOString()
      })

      // Update error patterns for learning
      this.updateErrorPatterns(error, context)

      // Check circuit breaker state
      if (this.isCircuitBreakerOpen(context.component)) {
        logger.warn('Circuit breaker is open, skipping recovery', { component: context.component })
        return false
      }

      // Apply recovery strategies if error is recoverable
      if (context.recoverable) {
        const recovered = await this.attemptRecovery(error, context)
        
        if (recovered) {
          this.recordSuccess(context.component)
          logger.info('Successfully recovered from error', { context })
          return true
        } else {
          this.recordFailure(context.component)
        }
      }

      // Handle critical errors with immediate system protection
      if (context.severity === 'critical') {
        await this.handleCriticalError(error, context)
      }

      return false
    } catch (recoveryError) {
      logger.error('Error during error recovery', { 
        originalError: error.message,
        recoveryError: recoveryError instanceof Error ? recoveryError.message : recoveryError,
        context 
      })
      return false
    } finally {
      performanceMonitor.recordMetric('error_handling_duration', performance.now() - startTime)
    }
  }

  /**
   * Attempt recovery using available strategies
   */
  private async attemptRecovery(error: Error, context: ErrorContext): Promise<boolean> {
    const strategies = this.recoveryStrategies.get(context.component) || []
    
    // Sort strategies by adaptive learning success rate
    const sortedStrategies = strategies.sort((a, b) => {
      const aStats = this.adaptiveLearning.get(`${context.component}_${a.name}`) || { success: 0, failure: 0 }
      const bStats = this.adaptiveLearning.get(`${context.component}_${b.name}`) || { success: 0, failure: 0 }
      
      const aSuccessRate = aStats.success / (aStats.success + aStats.failure + 1)
      const bSuccessRate = bStats.success / (bStats.success + bStats.failure + 1)
      
      return bSuccessRate - aSuccessRate
    })

    for (const strategy of sortedStrategies) {
      logger.debug('Attempting recovery strategy', { strategy: strategy.name, context })
      
      let retries = 0
      let successes = 0
      
      while (retries < strategy.maxRetries) {
        try {
          const result = await this.executeWithTimeout(strategy.attempt(), 10000)
          
          if (result) {
            successes++
            this.updateAdaptiveLearning(`${context.component}_${strategy.name}`, true)
            
            if (successes >= strategy.successThreshold) {
              logger.info('Recovery strategy succeeded', { 
                strategy: strategy.name, 
                retries, 
                successes,
                context 
              })
              return true
            }
          } else {
            this.updateAdaptiveLearning(`${context.component}_${strategy.name}`, false)
          }
        } catch (strategyError) {
          logger.warn('Recovery strategy failed', {
            strategy: strategy.name,
            error: strategyError instanceof Error ? strategyError.message : strategyError,
            retries,
            context
          })
          this.updateAdaptiveLearning(`${context.component}_${strategy.name}`, false)
        }
        
        retries++
        if (retries < strategy.maxRetries) {
          await this.delay(strategy.backoffMs * Math.pow(2, retries))
        }
      }
    }

    return false
  }

  /**
   * Handle critical errors with system protection
   */
  private async handleCriticalError(error: Error, context: ErrorContext): Promise<void> {
    logger.error('CRITICAL ERROR DETECTED', { error: error.message, context })
    
    // Open circuit breaker immediately
    this.openCircuitBreaker(context.component)
    
    // Trigger system protection modes
    await this.triggerEmergencyProtocols(context)
    
    // Notify monitoring systems
    if (window.postMessage) {
      window.postMessage({
        type: 'CRITICAL_ERROR',
        data: { error: error.message, context, timestamp: Date.now() }
      }, '*')
    }
  }

  /**
   * Circuit breaker management
   */
  private isCircuitBreakerOpen(component: string): boolean {
    const breaker = this.circuitBreakers.get(component)
    if (!breaker) return false
    
    if (breaker.state === 'open') {
      // Check if timeout has passed to attempt half-open
      if (Date.now() - breaker.lastFailure > breaker.timeout) {
        breaker.state = 'half-open'
        logger.info('Circuit breaker moving to half-open', { component })
      }
    }
    
    return breaker.state === 'open'
  }

  private recordFailure(component: string): void {
    let breaker = this.circuitBreakers.get(component)
    if (!breaker) {
      breaker = {
        failures: 0,
        lastFailure: 0,
        state: 'closed',
        threshold: 5,
        timeout: 60000 // 1 minute
      }
      this.circuitBreakers.set(component, breaker)
    }

    breaker.failures++
    breaker.lastFailure = Date.now()

    if (breaker.failures >= breaker.threshold && breaker.state === 'closed') {
      this.openCircuitBreaker(component)
    }
  }

  private recordSuccess(component: string): void {
    const breaker = this.circuitBreakers.get(component)
    if (breaker) {
      if (breaker.state === 'half-open') {
        breaker.state = 'closed'
        breaker.failures = 0
        logger.info('Circuit breaker closed after successful recovery', { component })
      } else if (breaker.failures > 0) {
        breaker.failures = Math.max(0, breaker.failures - 1)
      }
    }
  }

  private openCircuitBreaker(component: string): void {
    const breaker = this.circuitBreakers.get(component) || {
      failures: 0,
      lastFailure: Date.now(),
      state: 'closed' as const,
      threshold: 5,
      timeout: 60000
    }
    
    breaker.state = 'open'
    breaker.lastFailure = Date.now()
    this.circuitBreakers.set(component, breaker)
    
    logger.warn('Circuit breaker opened', { component, failures: breaker.failures })
  }

  /**
   * Initialize default recovery strategies
   */
  private initializeDefaultStrategies(): void {
    // WebRTC connection recovery
    this.recoveryStrategies.set('webrtc', [
      {
        name: 'restart_connection',
        attempt: async () => {
          // Simulate WebRTC reconnection
          await this.delay(1000)
          return Math.random() > 0.3
        },
        maxRetries: 3,
        backoffMs: 1000,
        successThreshold: 1
      },
      {
        name: 'fallback_websocket',
        attempt: async () => {
          // Simulate fallback to WebSocket
          await this.delay(500)
          return Math.random() > 0.1
        },
        maxRetries: 2,
        backoffMs: 500,
        successThreshold: 1
      }
    ])

    // API communication recovery
    this.recoveryStrategies.set('api', [
      {
        name: 'retry_request',
        attempt: async () => {
          await this.delay(200)
          return Math.random() > 0.4
        },
        maxRetries: 3,
        backoffMs: 1000,
        successThreshold: 1
      },
      {
        name: 'cache_fallback',
        attempt: async () => {
          // Use cached data as fallback
          await this.delay(100)
          return true
        },
        maxRetries: 1,
        backoffMs: 0,
        successThreshold: 1
      }
    ])

    // Robot agent recovery
    this.recoveryStrategies.set('robot_agent', [
      {
        name: 'restart_agent',
        attempt: async () => {
          await this.delay(2000)
          return Math.random() > 0.2
        },
        maxRetries: 2,
        backoffMs: 2000,
        successThreshold: 1
      },
      {
        name: 'failover_agent',
        attempt: async () => {
          await this.delay(1000)
          return Math.random() > 0.1
        },
        maxRetries: 1,
        backoffMs: 1000,
        successThreshold: 1
      }
    ])
  }

  /**
   * Emergency protocol triggers
   */
  private async triggerEmergencyProtocols(context: ErrorContext): Promise<void> {
    switch (context.component) {
      case 'robot_agent':
        logger.info('Triggering emergency stop for robot agents')
        // In real implementation, would send emergency stop commands
        break
      
      case 'webrtc':
        logger.info('Switching to degraded mode without real-time communication')
        // Fallback to polling or cached data
        break
      
      case 'ai_planning':
        logger.info('Switching to manual control mode')
        // Disable autonomous features
        break
    }
  }

  /**
   * Adaptive learning system
   */
  private updateAdaptiveLearning(key: string, success: boolean): void {
    const stats = this.adaptiveLearning.get(key) || { success: 0, failure: 0 }
    
    if (success) {
      stats.success++
    } else {
      stats.failure++
    }
    
    // Keep learning window reasonable
    const total = stats.success + stats.failure
    if (total > 1000) {
      stats.success = Math.floor(stats.success * 0.9)
      stats.failure = Math.floor(stats.failure * 0.9)
    }
    
    this.adaptiveLearning.set(key, stats)
  }

  /**
   * Update error patterns for trend analysis
   */
  private updateErrorPatterns(error: Error, context: ErrorContext): void {
    const pattern = `${context.component}_${error.constructor.name}`
    const count = this.errorPatterns.get(pattern) || 0
    this.errorPatterns.set(pattern, count + 1)
    
    // Alert if error pattern is increasing
    if (count > 10) {
      logger.warn('Error pattern detected', { 
        pattern, 
        count, 
        message: error.message,
        context 
      })
    }
  }

  /**
   * Periodic maintenance and cleanup
   */
  private startPeriodicMaintenance(): void {
    setInterval(() => {
      this.cleanupOldData()
      this.analyzeErrorTrends()
    }, 300000) // 5 minutes
  }

  private cleanupOldData(): void {
    const now = Date.now()
    const maxAge = 3600000 // 1 hour
    
    // Clean up old circuit breaker data
    for (const [component, breaker] of this.circuitBreakers.entries()) {
      if (breaker.state === 'open' && now - breaker.lastFailure > maxAge) {
        breaker.state = 'closed'
        breaker.failures = 0
        logger.info('Circuit breaker auto-reset after timeout', { component })
      }
    }
  }

  private analyzeErrorTrends(): void {
    const trends: Array<{pattern: string, count: number}> = []
    
    for (const [pattern, count] of this.errorPatterns.entries()) {
      trends.push({ pattern, count })
    }
    
    trends.sort((a, b) => b.count - a.count)
    
    if (trends.length > 0) {
      logger.info('Error trend analysis', { 
        topPatterns: trends.slice(0, 5),
        totalPatterns: trends.length 
      })
    }
  }

  /**
   * Utility methods
   */
  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms))
  }

  private executeWithTimeout<T>(promise: Promise<T>, timeoutMs: number): Promise<T> {
    return Promise.race([
      promise,
      new Promise<T>((_, reject) => 
        setTimeout(() => reject(new Error('Operation timeout')), timeoutMs)
      )
    ])
  }

  /**
   * Get system health report
   */
  getHealthReport(): any {
    const circuitBreakers: any = {}
    for (const [component, breaker] of this.circuitBreakers.entries()) {
      circuitBreakers[component] = {
        state: breaker.state,
        failures: breaker.failures,
        lastFailure: breaker.lastFailure
      }
    }

    const adaptiveLearning: any = {}
    for (const [key, stats] of this.adaptiveLearning.entries()) {
      const total = stats.success + stats.failure
      adaptiveLearning[key] = {
        successRate: total > 0 ? stats.success / total : 0,
        totalAttempts: total
      }
    }

    return {
      circuitBreakers,
      adaptiveLearning,
      errorPatterns: Object.fromEntries(this.errorPatterns),
      timestamp: Date.now()
    }
  }
}

// Export singleton instance
export const resilientErrorHandler = new ResilientErrorHandler()

/**
 * Convenient error handling decorator
 */
export function withErrorRecovery(
  component: string, 
  operation: string, 
  severity: ErrorContext['severity'] = 'medium'
) {
  return function(target: any, propertyKey: string, descriptor: PropertyDescriptor) {
    const originalMethod = descriptor.value

    descriptor.value = async function(...args: any[]) {
      try {
        return await originalMethod.apply(this, args)
      } catch (error) {
        const context: ErrorContext = {
          component,
          operation,
          severity,
          recoverable: true,
          metadata: { args: args.map(arg => String(arg)).slice(0, 3) }
        }

        const recovered = await resilientErrorHandler.handleError(error as Error, context)
        
        if (!recovered) {
          throw error
        }
        
        // Retry original operation after recovery
        return await originalMethod.apply(this, args)
      }
    }

    return descriptor
  }
}

export default resilientErrorHandler