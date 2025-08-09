interface RetryOptions {
  maxAttempts?: number
  baseDelay?: number
  maxDelay?: number
  exponentialBackoff?: boolean
  jitter?: boolean
  shouldRetry?: (error: unknown) => boolean
}

interface RetryResult<T> {
  result?: T
  error?: Error
  attempts: number
  totalTime: number
}

export class RetryableError extends Error {
  constructor(message: string, public readonly shouldRetry: boolean = true) {
    super(message)
    this.name = 'RetryableError'
  }
}

export class NetworkError extends RetryableError {
  constructor(message: string, public readonly status?: number) {
    super(message, status ? status >= 500 || status === 429 : true)
    this.name = 'NetworkError'
  }
}

export class WebRTCError extends RetryableError {
  constructor(message: string, public readonly connectionState?: RTCPeerConnectionState) {
    super(message, connectionState !== 'failed')
    this.name = 'WebRTCError'
  }
}

export async function withRetry<T>(
  operation: () => Promise<T>,
  options: RetryOptions = {}
): Promise<RetryResult<T>> {
  const {
    maxAttempts = 3,
    baseDelay = 1000,
    maxDelay = 30000,
    exponentialBackoff = true,
    jitter = true,
    shouldRetry = (error) => {
      if (error instanceof RetryableError) {
        return error.shouldRetry
      }
      if (error instanceof TypeError && error.message.includes('fetch')) {
        return true
      }
      return false
    }
  } = options

  let lastError: Error
  let attempts = 0
  const startTime = Date.now()

  while (attempts < maxAttempts) {
    attempts++

    try {
      const result = await operation()
      return {
        result,
        attempts,
        totalTime: Date.now() - startTime
      }
    } catch (error) {
      lastError = error instanceof Error ? error : new Error(String(error))
      
      // Don't retry on last attempt or if error shouldn't be retried
      if (attempts >= maxAttempts || !shouldRetry(lastError)) {
        break
      }

      // Calculate delay
      let delay = baseDelay
      if (exponentialBackoff) {
        delay = Math.min(baseDelay * Math.pow(2, attempts - 1), maxDelay)
      }
      
      // Add jitter to prevent thundering herd
      if (jitter) {
        delay = delay * (0.5 + Math.random() * 0.5)
      }

      console.warn(`Operation failed (attempt ${attempts}/${maxAttempts}), retrying in ${delay}ms:`, lastError.message)
      
      await new Promise(resolve => setTimeout(resolve, delay))
    }
  }

  return {
    error: lastError!,
    attempts,
    totalTime: Date.now() - startTime
  }
}

// Circuit breaker pattern for preventing cascading failures
export class CircuitBreaker {
  private failureCount = 0
  private lastFailureTime = 0
  private state: 'closed' | 'open' | 'half-open' = 'closed'

  constructor(
    private readonly failureThreshold: number = 5,
    private readonly recoveryTimeout: number = 60000,
    private readonly halfOpenMaxCalls: number = 3
  ) {}

  async call<T>(operation: () => Promise<T>): Promise<T> {
    if (this.state === 'open') {
      if (Date.now() - this.lastFailureTime > this.recoveryTimeout) {
        this.state = 'half-open'
        console.log('CircuitBreaker: Moving to half-open state')
      } else {
        throw new Error('CircuitBreaker: Circuit is open')
      }
    }

    try {
      const result = await operation()
      
      if (this.state === 'half-open') {
        this.reset()
      }
      
      return result
    } catch (error) {
      this.recordFailure()
      throw error
    }
  }

  private recordFailure() {
    this.failureCount++
    this.lastFailureTime = Date.now()

    if (this.failureCount >= this.failureThreshold) {
      this.state = 'open'
      console.warn('CircuitBreaker: Circuit opened due to repeated failures')
    }
  }

  private reset() {
    this.failureCount = 0
    this.state = 'closed'
    console.log('CircuitBreaker: Circuit closed - normal operation resumed')
  }

  getState() {
    return {
      state: this.state,
      failureCount: this.failureCount,
      lastFailureTime: this.lastFailureTime
    }
  }
}

// Connection health monitor
export class ConnectionHealthMonitor {
  private healthScore = 100
  private measurements: number[] = []
  private readonly maxMeasurements = 100

  constructor(
    private readonly degradedThreshold = 70,
    private readonly failedThreshold = 30
  ) {}

  recordLatency(latency: number) {
    // Score based on latency: 100 (0ms) to 0 (1000ms+)
    const latencyScore = Math.max(0, 100 - (latency / 10))
    this.addMeasurement(latencyScore)
  }

  recordPacketLoss(packetLoss: number) {
    // Score based on packet loss: 100 (0%) to 0 (10%+)
    const packetLossScore = Math.max(0, 100 - (packetLoss * 1000))
    this.addMeasurement(packetLossScore)
  }

  recordConnectionFailure() {
    this.addMeasurement(0)
  }

  recordConnectionSuccess() {
    this.addMeasurement(100)
  }

  private addMeasurement(score: number) {
    this.measurements.push(score)
    if (this.measurements.length > this.maxMeasurements) {
      this.measurements.shift()
    }
    
    // Calculate rolling average
    this.healthScore = this.measurements.reduce((sum, s) => sum + s, 0) / this.measurements.length
  }

  getHealthStatus() {
    if (this.healthScore >= this.degradedThreshold) {
      return { status: 'healthy', score: this.healthScore }
    } else if (this.healthScore >= this.failedThreshold) {
      return { status: 'degraded', score: this.healthScore }
    } else {
      return { status: 'failed', score: this.healthScore }
    }
  }

  shouldUseBackup(): boolean {
    return this.healthScore < this.degradedThreshold
  }

  shouldDisconnect(): boolean {
    return this.healthScore < this.failedThreshold
  }
}

// Graceful degradation manager
export class GracefulDegradationManager {
  private features: Map<string, boolean> = new Map()
  
  constructor() {
    // Initialize with all features enabled
    this.features.set('webrtc', true)
    this.features.set('video_streaming', true)
    this.features.set('ai_commands', true)
    this.features.set('telemetry_visualization', true)
    this.features.set('formation_control', true)
  }

  disableFeature(feature: string, reason: string) {
    this.features.set(feature, false)
    console.warn(`Feature '${feature}' disabled: ${reason}`)
    
    // Notify user about degraded functionality
    this.notifyDegradation(feature, reason)
  }

  enableFeature(feature: string) {
    this.features.set(feature, true)
    console.log(`Feature '${feature}' re-enabled`)
  }

  isFeatureEnabled(feature: string): boolean {
    return this.features.get(feature) ?? false
  }

  private notifyDegradation(feature: string, reason: string) {
    const event = new CustomEvent('feature-degraded', {
      detail: { feature, reason }
    })
    window.dispatchEvent(event)
  }

  getStatus() {
    const enabled: string[] = []
    const disabled: string[] = []
    
    this.features.forEach((isEnabled, feature) => {
      if (isEnabled) {
        enabled.push(feature)
      } else {
        disabled.push(feature)
      }
    })
    
    return { enabled, disabled }
  }
}

// Singleton instances for global use
export const connectionHealthMonitor = new ConnectionHealthMonitor()
export const gracefulDegradationManager = new GracefulDegradationManager()

// WebRTC circuit breaker
export const webrtcCircuitBreaker = new CircuitBreaker(3, 30000, 1)

// API circuit breaker
export const apiCircuitBreaker = new CircuitBreaker(5, 60000, 3)