// Circuit breaker pattern implementation for resilient service calls
import { logger } from './logger'

export enum CircuitState {
  CLOSED = 'closed',
  OPEN = 'open',
  HALF_OPEN = 'half_open'
}

export interface CircuitBreakerConfig {
  failureThreshold: number
  recoveryTimeout: number
  monitoringPeriod: number
  halfOpenMaxCalls: number
  onStateChange?: (state: CircuitState) => void
  onFallback?: () => any
}

export interface CircuitBreakerMetrics {
  state: CircuitState
  failures: number
  successes: number
  totalCalls: number
  lastFailureTime?: number
  stateChangedAt: number
}

class CircuitBreaker {
  private config: CircuitBreakerConfig
  private state: CircuitState = CircuitState.CLOSED
  private failures = 0
  private successes = 0
  private totalCalls = 0
  private lastFailureTime?: number
  private stateChangedAt = Date.now()
  private halfOpenCalls = 0

  constructor(config: CircuitBreakerConfig) {
    this.config = {
      failureThreshold: 5,
      recoveryTimeout: 60000, // 1 minute
      monitoringPeriod: 60000, // 1 minute
      halfOpenMaxCalls: 3,
      ...config
    }

    // Start monitoring
    setInterval(() => this.checkState(), 5000)
  }

  async execute<T>(operation: () => Promise<T>): Promise<T> {
    if (this.state === CircuitState.OPEN) {
      if (this.shouldAttemptReset()) {
        this.setState(CircuitState.HALF_OPEN)
        this.halfOpenCalls = 0
      } else {
        return this.handleFallback()
      }
    }

    if (this.state === CircuitState.HALF_OPEN && this.halfOpenCalls >= this.config.halfOpenMaxCalls) {
      return this.handleFallback()
    }

    this.totalCalls++
    
    if (this.state === CircuitState.HALF_OPEN) {
      this.halfOpenCalls++
    }

    try {
      const result = await operation()
      this.onSuccess()
      return result
    } catch (error) {
      this.onFailure()
      throw error
    }
  }

  private onSuccess(): void {
    this.successes++

    if (this.state === CircuitState.HALF_OPEN) {
      if (this.halfOpenCalls >= this.config.halfOpenMaxCalls) {
        // All half-open calls succeeded, circuit can be closed
        this.setState(CircuitState.CLOSED)
        this.failures = 0
        this.halfOpenCalls = 0
        
        logger.info('Circuit breaker closed after successful recovery', {
          successes: this.successes,
          totalCalls: this.totalCalls
        })
      }
    } else if (this.state === CircuitState.CLOSED) {
      // Reset failure count on success in closed state
      this.failures = Math.max(0, this.failures - 1)
    }
  }

  private onFailure(): void {
    this.failures++
    this.lastFailureTime = Date.now()

    logger.warn('Circuit breaker operation failed', {
      failures: this.failures,
      threshold: this.config.failureThreshold,
      state: this.state
    })

    if (this.state === CircuitState.HALF_OPEN) {
      // Any failure in half-open state opens the circuit
      this.setState(CircuitState.OPEN)
      this.halfOpenCalls = 0
      
      logger.warn('Circuit breaker opened due to failure in half-open state')
    } else if (this.state === CircuitState.CLOSED && this.failures >= this.config.failureThreshold) {
      // Too many failures in closed state
      this.setState(CircuitState.OPEN)
      
      logger.error('Circuit breaker opened due to failure threshold exceeded', {
        failures: this.failures,
        threshold: this.config.failureThreshold
      })
    }
  }

  private shouldAttemptReset(): boolean {
    if (!this.lastFailureTime) return false
    
    const timeSinceLastFailure = Date.now() - this.lastFailureTime
    return timeSinceLastFailure >= this.config.recoveryTimeout
  }

  private setState(newState: CircuitState): void {
    if (newState !== this.state) {
      const oldState = this.state
      this.state = newState
      this.stateChangedAt = Date.now()

      logger.info('Circuit breaker state changed', {
        from: oldState,
        to: newState,
        metrics: this.getMetrics()
      })

      if (this.config.onStateChange) {
        this.config.onStateChange(newState)
      }
    }
  }

  private handleFallback<T>(): T {
    logger.warn('Circuit breaker fallback triggered', {
      state: this.state,
      failures: this.failures
    })

    if (this.config.onFallback) {
      return this.config.onFallback()
    }

    throw new Error(`Circuit breaker is ${this.state}. Service unavailable.`)
  }

  private checkState(): void {
    // Reset counters periodically
    const now = Date.now()
    const timeSinceStateChange = now - this.stateChangedAt

    if (timeSinceStateChange >= this.config.monitoringPeriod) {
      if (this.state === CircuitState.CLOSED) {
        // Reset failure count periodically in closed state
        this.failures = Math.max(0, this.failures - 1)
      }
    }
  }

  getMetrics(): CircuitBreakerMetrics {
    return {
      state: this.state,
      failures: this.failures,
      successes: this.successes,
      totalCalls: this.totalCalls,
      lastFailureTime: this.lastFailureTime,
      stateChangedAt: this.stateChangedAt
    }
  }

  reset(): void {
    this.setState(CircuitState.CLOSED)
    this.failures = 0
    this.successes = 0
    this.totalCalls = 0
    this.lastFailureTime = undefined
    this.halfOpenCalls = 0

    logger.info('Circuit breaker manually reset')
  }

  forceOpen(): void {
    this.setState(CircuitState.OPEN)
    logger.warn('Circuit breaker manually opened')
  }

  forceClose(): void {
    this.setState(CircuitState.CLOSED)
    this.failures = 0
    this.halfOpenCalls = 0
    logger.info('Circuit breaker manually closed')
  }
}

// Factory for creating circuit breakers
export class CircuitBreakerFactory {
  private static instances = new Map<string, CircuitBreaker>()

  static create(name: string, config: Partial<CircuitBreakerConfig> = {}): CircuitBreaker {
    if (this.instances.has(name)) {
      return this.instances.get(name)!
    }

    const circuitBreaker = new CircuitBreaker({
      failureThreshold: 5,
      recoveryTimeout: 60000,
      monitoringPeriod: 60000,
      halfOpenMaxCalls: 3,
      ...config,
      onStateChange: (state) => {
        logger.info(`Circuit breaker ${name} state changed to ${state}`)
        config.onStateChange?.(state)
      }
    })

    this.instances.set(name, circuitBreaker)
    return circuitBreaker
  }

  static get(name: string): CircuitBreaker | undefined {
    return this.instances.get(name)
  }

  static getAll(): Map<string, CircuitBreaker> {
    return new Map(this.instances)
  }

  static remove(name: string): boolean {
    return this.instances.delete(name)
  }

  static clear(): void {
    this.instances.clear()
  }
}

// Pre-configured circuit breakers for common services
export const webrtcCircuitBreaker = CircuitBreakerFactory.create('webrtc', {
  failureThreshold: 3,
  recoveryTimeout: 30000,
  onFallback: () => {
    logger.warn('WebRTC connection failed, switching to polling mode')
    return null
  }
})

export const gptCircuitBreaker = CircuitBreakerFactory.create('gpt', {
  failureThreshold: 2,
  recoveryTimeout: 120000,
  onFallback: () => {
    logger.warn('GPT API unavailable, using local fallback')
    return {
      choices: [{
        message: {
          content: 'AI service temporarily unavailable. Please try manual operation.'
        }
      }]
    }
  }
})

export const robotCommandCircuitBreaker = CircuitBreakerFactory.create('robot_commands', {
  failureThreshold: 5,
  recoveryTimeout: 10000,
  onFallback: () => {
    logger.error('Robot command service unavailable')
    throw new Error('Robot command service is currently unavailable')
  }
})

export { CircuitBreaker }