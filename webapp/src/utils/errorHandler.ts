export enum ErrorSeverity {
  DEBUG = 'debug',
  INFO = 'info',
  WARNING = 'warning',
  ERROR = 'error',
  CRITICAL = 'critical'
}

export interface SwarmError {
  id: string
  code: string
  message: string
  severity: ErrorSeverity
  component: string
  agentId?: string
  context?: Record<string, any>
  timestamp: number
  stackTrace?: string
  recovered: boolean
}

export interface ErrorMetrics {
  totalErrors: number
  errorsByCode: Record<string, number>
  errorsBySeverity: Record<ErrorSeverity, number>
  recentErrors: SwarmError[]
  recoveryRate: number
}

class ErrorHandler {
  private errors: SwarmError[] = []
  private maxErrorHistory = 1000
  private recoveryHandlers: Map<string, (error: SwarmError) => Promise<boolean>> = new Map()
  private errorSubscribers: Set<(error: SwarmError) => void> = new Set()
  private metricsSubscribers: Set<(metrics: ErrorMetrics) => void> = new Set()

  constructor() {
    // Set up global error handlers
    this.setupGlobalHandlers()
  }

  private setupGlobalHandlers() {
    // Handle unhandled promise rejections
    window.addEventListener('unhandledrejection', (event) => {
      this.handleError({
        code: 'UNHANDLED_PROMISE_REJECTION',
        message: event.reason?.message || 'Unhandled promise rejection',
        severity: ErrorSeverity.ERROR,
        component: 'global',
        context: {
          reason: event.reason,
          promise: event.promise
        }
      })
    })

    // Handle JavaScript errors
    window.addEventListener('error', (event) => {
      this.handleError({
        code: 'JAVASCRIPT_ERROR',
        message: event.message || 'JavaScript error',
        severity: ErrorSeverity.ERROR,
        component: 'global',
        context: {
          filename: event.filename,
          lineno: event.lineno,
          colno: event.colno,
          error: event.error
        }
      })
    })

    // Handle WebRTC errors
    this.registerRecoveryHandler('WEBRTC_CONNECTION_FAILED', async (error) => {
      console.log('Attempting WebRTC reconnection...')
      // Implementation would attempt WebRTC reconnection
      return false
    })

    // Handle network errors
    this.registerRecoveryHandler('NETWORK_ERROR', async (error) => {
      console.log('Attempting network recovery...')
      // Implementation would check network status and retry
      return false
    })
  }

  handleError(errorInfo: {
    code: string
    message: string
    severity?: ErrorSeverity
    component?: string
    agentId?: string
    context?: Record<string, any>
  }): SwarmError {
    const error: SwarmError = {
      id: this.generateErrorId(),
      code: errorInfo.code,
      message: errorInfo.message,
      severity: errorInfo.severity || ErrorSeverity.ERROR,
      component: errorInfo.component || 'unknown',
      agentId: errorInfo.agentId,
      context: errorInfo.context,
      timestamp: Date.now(),
      stackTrace: new Error().stack,
      recovered: false
    }

    // Add to error history
    this.errors.push(error)
    if (this.errors.length > this.maxErrorHistory) {
      this.errors.shift()
    }

    // Log error
    this.logError(error)

    // Notify subscribers
    this.notifyErrorSubscribers(error)

    // Attempt recovery
    this.attemptRecovery(error)

    // Update metrics
    this.updateMetrics()

    return error
  }

  private generateErrorId(): string {
    return `error_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
  }

  private logError(error: SwarmError) {
    const logMessage = `[${error.component}${error.agentId ? `:${error.agentId}` : ''}] ${error.code}: ${error.message}`
    
    switch (error.severity) {
      case ErrorSeverity.DEBUG:
        console.debug(logMessage, error.context)
        break
      case ErrorSeverity.INFO:
        console.info(logMessage, error.context)
        break
      case ErrorSeverity.WARNING:
        console.warn(logMessage, error.context)
        break
      case ErrorSeverity.ERROR:
        console.error(logMessage, error.context)
        break
      case ErrorSeverity.CRITICAL:
        console.error(`🚨 CRITICAL: ${logMessage}`, error.context)
        break
    }
  }

  private async attemptRecovery(error: SwarmError) {
    const handler = this.recoveryHandlers.get(error.code)
    if (handler) {
      try {
        const recovered = await handler(error)
        if (recovered) {
          error.recovered = true
          console.log(`✅ Recovered from error: ${error.code}`)
        } else {
          console.warn(`❌ Failed to recover from error: ${error.code}`)
        }
      } catch (recoveryError) {
        console.error(`Recovery handler failed for ${error.code}:`, recoveryError)
      }
    }
  }

  registerRecoveryHandler(errorCode: string, handler: (error: SwarmError) => Promise<boolean>) {
    this.recoveryHandlers.set(errorCode, handler)
  }

  subscribeToErrors(callback: (error: SwarmError) => void) {
    this.errorSubscribers.add(callback)
    return () => this.errorSubscribers.delete(callback)
  }

  subscribeToMetrics(callback: (metrics: ErrorMetrics) => void) {
    this.metricsSubscribers.add(callback)
    return () => this.metricsSubscribers.delete(callback)
  }

  private notifyErrorSubscribers(error: SwarmError) {
    this.errorSubscribers.forEach(callback => {
      try {
        callback(error)
      } catch (err) {
        console.error('Error in error subscriber:', err)
      }
    })
  }

  private updateMetrics() {
    const metrics = this.getMetrics()
    this.metricsSubscribers.forEach(callback => {
      try {
        callback(metrics)
      } catch (err) {
        console.error('Error in metrics subscriber:', err)
      }
    })
  }

  getMetrics(): ErrorMetrics {
    const errorsByCode: Record<string, number> = {}
    const errorsBySeverity: Record<ErrorSeverity, number> = {
      [ErrorSeverity.DEBUG]: 0,
      [ErrorSeverity.INFO]: 0,
      [ErrorSeverity.WARNING]: 0,
      [ErrorSeverity.ERROR]: 0,
      [ErrorSeverity.CRITICAL]: 0
    }

    let recoveredCount = 0

    this.errors.forEach(error => {
      errorsByCode[error.code] = (errorsByCode[error.code] || 0) + 1
      errorsBySeverity[error.severity]++
      if (error.recovered) recoveredCount++
    })

    return {
      totalErrors: this.errors.length,
      errorsByCode,
      errorsBySeverity,
      recentErrors: this.errors.slice(-10),
      recoveryRate: this.errors.length > 0 ? recoveredCount / this.errors.length : 0
    }
  }

  getErrorHistory(): SwarmError[] {
    return [...this.errors]
  }

  clearErrors() {
    this.errors.length = 0
    this.updateMetrics()
  }

  // Convenience methods for common error patterns
  handleNetworkError(message: string, context?: Record<string, any>) {
    return this.handleError({
      code: 'NETWORK_ERROR',
      message,
      severity: ErrorSeverity.ERROR,
      component: 'network',
      context
    })
  }

  handleWebRTCError(message: string, context?: Record<string, any>) {
    return this.handleError({
      code: 'WEBRTC_ERROR',
      message,
      severity: ErrorSeverity.ERROR,
      component: 'webrtc',
      context
    })
  }

  handleAgentError(agentId: string, message: string, context?: Record<string, any>) {
    return this.handleError({
      code: 'AGENT_ERROR',
      message,
      severity: ErrorSeverity.WARNING,
      component: 'agent',
      agentId,
      context
    })
  }

  handleUIError(message: string, context?: Record<string, any>) {
    return this.handleError({
      code: 'UI_ERROR',
      message,
      severity: ErrorSeverity.ERROR,
      component: 'ui',
      context
    })
  }

  handleGPTError(message: string, context?: Record<string, any>) {
    return this.handleError({
      code: 'GPT_ERROR',
      message,
      severity: ErrorSeverity.WARNING,
      component: 'gpt',
      context
    })
  }
}

// Singleton instance
export const errorHandler = new ErrorHandler()

// React hook for using error handler
import { useEffect, useState } from 'react'

export function useErrorHandler() {
  const [errors, setErrors] = useState<SwarmError[]>([])
  const [metrics, setMetrics] = useState<ErrorMetrics | null>(null)

  useEffect(() => {
    const unsubscribeErrors = errorHandler.subscribeToErrors((error) => {
      setErrors(prev => [...prev.slice(-99), error]) // Keep last 100 errors
    })

    const unsubscribeMetrics = errorHandler.subscribeToMetrics((newMetrics) => {
      setMetrics(newMetrics)
    })

    // Initialize with current state
    setErrors(errorHandler.getErrorHistory().slice(-100))
    setMetrics(errorHandler.getMetrics())

    return () => {
      unsubscribeErrors()
      unsubscribeMetrics()
    }
  }, [])

  return {
    errors,
    metrics,
    handleError: errorHandler.handleError.bind(errorHandler),
    clearErrors: errorHandler.clearErrors.bind(errorHandler),
    registerRecoveryHandler: errorHandler.registerRecoveryHandler.bind(errorHandler)
  }
}

// Utility function for wrapping async functions with error handling
export function withErrorHandling<T extends any[], R>(
  fn: (...args: T) => Promise<R>,
  errorCode: string,
  component: string = 'unknown'
) {
  return async (...args: T): Promise<R | null> => {
    try {
      return await fn(...args)
    } catch (error: any) {
      errorHandler.handleError({
        code: errorCode,
        message: error.message || 'Unknown error',
        severity: ErrorSeverity.ERROR,
        component,
        context: {
          args: args,
          stack: error.stack
        }
      })
      return null
    }
  }
}

// Common error codes
export const ErrorCodes = {
  // Network errors
  NETWORK_CONNECTION_FAILED: 'NETWORK_001',
  NETWORK_TIMEOUT: 'NETWORK_002',
  WEBSOCKET_CONNECTION_FAILED: 'WEBSOCKET_001',
  WEBRTC_CONNECTION_FAILED: 'WEBRTC_001',
  
  // Agent errors
  AGENT_CONNECTION_LOST: 'AGENT_001',
  AGENT_COMMAND_FAILED: 'AGENT_002',
  AGENT_TIMEOUT: 'AGENT_003',
  
  // UI errors
  COMPONENT_RENDER_ERROR: 'UI_001',
  USER_INPUT_ERROR: 'UI_002',
  
  // GPT/AI errors
  GPT_API_ERROR: 'GPT_001',
  GPT_TIMEOUT: 'GPT_002',
  GPT_INVALID_RESPONSE: 'GPT_003',
  
  // VR/XR errors
  XR_NOT_SUPPORTED: 'XR_001',
  XR_SESSION_FAILED: 'XR_002',
  
  // System errors
  MEMORY_ERROR: 'SYS_001',
  PERFORMANCE_DEGRADATION: 'SYS_002'
} as const