// Advanced logging and monitoring system for XR-Swarm-Bridge

export enum LogLevel {
  DEBUG = 0,
  INFO = 1,
  WARN = 2,
  ERROR = 3,
  CRITICAL = 4
}

export interface LogEntry {
  id: string
  timestamp: number
  level: LogLevel
  message: string
  category: string
  data?: any
  userId?: string
  sessionId: string
  agentId?: string
  component: string
  stackTrace?: string
  performance?: {
    duration: number
    memoryUsage: number
    renderTime?: number
  }
}

export interface LogMetrics {
  totalLogs: number
  logsByLevel: Record<LogLevel, number>
  logsByCategory: Record<string, number>
  recentErrors: LogEntry[]
  performanceMetrics: {
    averageRenderTime: number
    memoryTrend: number[]
    errorRate: number
  }
}

class Logger {
  private logs: LogEntry[] = []
  private maxLogs = 10000
  private sessionId: string
  private userId?: string
  private currentLogLevel: LogLevel = LogLevel.INFO
  private subscribers: Set<(log: LogEntry) => void> = new Set()
  private metricsSubscribers: Set<(metrics: LogMetrics) => void> = new Set()
  
  // Performance monitoring
  private performanceEntries: Map<string, number> = new Map()
  private memoryUsageHistory: number[] = []
  private renderTimeHistory: number[] = []
  
  constructor() {
    this.sessionId = this.generateSessionId()
    this.setupPerformanceMonitoring()
    
    // Set log level from environment
    const envLogLevel = process.env.REACT_APP_LOG_LEVEL?.toLowerCase()
    switch (envLogLevel) {
      case 'debug':
        this.currentLogLevel = LogLevel.DEBUG
        break
      case 'info':
        this.currentLogLevel = LogLevel.INFO
        break
      case 'warn':
        this.currentLogLevel = LogLevel.WARN
        break
      case 'error':
        this.currentLogLevel = LogLevel.ERROR
        break
      case 'critical':
        this.currentLogLevel = LogLevel.CRITICAL
        break
    }
  }

  private generateSessionId(): string {
    return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
  }

  private setupPerformanceMonitoring() {
    // Monitor memory usage
    if ('memory' in performance) {
      setInterval(() => {
        const memory = (performance as any).memory
        if (memory) {
          const usage = memory.usedJSHeapSize / (1024 * 1024) // MB
          this.memoryUsageHistory.push(usage)
          if (this.memoryUsageHistory.length > 100) {
            this.memoryUsageHistory.shift()
          }
          
          // Log memory warnings
          if (usage > 256) { // 256MB threshold
            this.warn('High memory usage detected', 'performance', { memoryUsage: usage })
          }
        }
      }, 5000) // Check every 5 seconds
    }

    // Monitor frame rate
    let lastTime = performance.now()
    let frameCount = 0
    
    const monitorFrameRate = () => {
      frameCount++
      const currentTime = performance.now()
      
      if (currentTime - lastTime >= 1000) { // Every second
        const fps = frameCount
        frameCount = 0
        lastTime = currentTime
        
        if (fps < 30) { // Low FPS warning
          this.warn('Low frame rate detected', 'performance', { fps })
        }
      }
      
      requestAnimationFrame(monitorFrameRate)
    }
    
    requestAnimationFrame(monitorFrameRate)
  }

  private shouldLog(level: LogLevel): boolean {
    return level >= this.currentLogLevel
  }

  private createLogEntry(
    level: LogLevel,
    message: string,
    category: string,
    data?: any,
    component: string = 'unknown',
    agentId?: string
  ): LogEntry {
    const entry: LogEntry = {
      id: this.generateLogId(),
      timestamp: Date.now(),
      level,
      message,
      category,
      data,
      userId: this.userId,
      sessionId: this.sessionId,
      agentId,
      component,
      stackTrace: level >= LogLevel.ERROR ? new Error().stack : undefined,
      performance: {
        duration: 0,
        memoryUsage: this.getCurrentMemoryUsage()
      }
    }

    return entry
  }

  private generateLogId(): string {
    return `log_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
  }

  private getCurrentMemoryUsage(): number {
    if ('memory' in performance) {
      const memory = (performance as any).memory
      return memory ? memory.usedJSHeapSize / (1024 * 1024) : 0
    }
    return 0
  }

  private addLogEntry(entry: LogEntry) {
    this.logs.push(entry)
    
    // Maintain log history limit
    if (this.logs.length > this.maxLogs) {
      this.logs.shift()
    }

    // Console output
    this.outputToConsole(entry)

    // Notify subscribers
    this.subscribers.forEach(subscriber => {
      try {
        subscriber(entry)
      } catch (error) {
        console.error('Error in log subscriber:', error)
      }
    })

    // Update metrics
    this.updateMetrics()

    // Send to external monitoring in production
    if (process.env.NODE_ENV === 'production' && entry.level >= LogLevel.ERROR) {
      this.sendToExternalMonitoring(entry)
    }
  }

  private outputToConsole(entry: LogEntry) {
    const timestamp = new Date(entry.timestamp).toISOString()
    const prefix = `[${timestamp}] [${LogLevel[entry.level]}] [${entry.category}${entry.agentId ? `:${entry.agentId}` : ''}]`
    const message = `${prefix} ${entry.message}`

    switch (entry.level) {
      case LogLevel.DEBUG:
        console.debug(message, entry.data)
        break
      case LogLevel.INFO:
        console.info(message, entry.data)
        break
      case LogLevel.WARN:
        console.warn(message, entry.data)
        break
      case LogLevel.ERROR:
        console.error(message, entry.data)
        break
      case LogLevel.CRITICAL:
        console.error(`🚨 CRITICAL: ${message}`, entry.data)
        break
    }
  }

  private sendToExternalMonitoring(entry: LogEntry) {
    // This would integrate with services like Sentry, DataDog, etc.
    // For now, we'll just prepare the data structure
    const monitoringData = {
      level: LogLevel[entry.level],
      message: entry.message,
      timestamp: entry.timestamp,
      user: entry.userId,
      session: entry.sessionId,
      component: entry.component,
      agent: entry.agentId,
      data: entry.data,
      stackTrace: entry.stackTrace,
      url: window.location.href,
      userAgent: navigator.userAgent,
      performance: entry.performance
    }

    // In a real implementation, you would send this to your monitoring service
    console.log('📊 External monitoring data:', monitoringData)
  }

  private updateMetrics() {
    const metrics = this.getMetrics()
    this.metricsSubscribers.forEach(subscriber => {
      try {
        subscriber(metrics)
      } catch (error) {
        console.error('Error in metrics subscriber:', error)
      }
    })
  }

  // Public logging methods
  debug(message: string, category: string = 'general', data?: any, component?: string, agentId?: string) {
    if (!this.shouldLog(LogLevel.DEBUG)) return
    const entry = this.createLogEntry(LogLevel.DEBUG, message, category, data, component, agentId)
    this.addLogEntry(entry)
  }

  info(message: string, category: string = 'general', data?: any, component?: string, agentId?: string) {
    if (!this.shouldLog(LogLevel.INFO)) return
    const entry = this.createLogEntry(LogLevel.INFO, message, category, data, component, agentId)
    this.addLogEntry(entry)
  }

  warn(message: string, category: string = 'general', data?: any, component?: string, agentId?: string) {
    if (!this.shouldLog(LogLevel.WARN)) return
    const entry = this.createLogEntry(LogLevel.WARN, message, category, data, component, agentId)
    this.addLogEntry(entry)
  }

  error(message: string, category: string = 'general', data?: any, component?: string, agentId?: string) {
    if (!this.shouldLog(LogLevel.ERROR)) return
    const entry = this.createLogEntry(LogLevel.ERROR, message, category, data, component, agentId)
    this.addLogEntry(entry)
  }

  critical(message: string, category: string = 'general', data?: any, component?: string, agentId?: string) {
    const entry = this.createLogEntry(LogLevel.CRITICAL, message, category, data, component, agentId)
    this.addLogEntry(entry)
  }

  // Performance logging
  startPerformanceTimer(label: string): void {
    this.performanceEntries.set(label, performance.now())
  }

  endPerformanceTimer(label: string, category: string = 'performance', component?: string): number {
    const startTime = this.performanceEntries.get(label)
    if (!startTime) {
      this.warn(`Performance timer '${label}' not found`, 'performance', { label })
      return 0
    }

    const duration = performance.now() - startTime
    this.performanceEntries.delete(label)

    this.info(`Performance: ${label} took ${duration.toFixed(2)}ms`, category, { 
      label, 
      duration,
      component 
    })

    // Track render times separately
    if (category === 'render') {
      this.renderTimeHistory.push(duration)
      if (this.renderTimeHistory.length > 100) {
        this.renderTimeHistory.shift()
      }
    }

    return duration
  }

  // User context
  setUserId(userId: string) {
    this.userId = userId
    this.info('User context set', 'auth', { userId })
  }

  // Subscription methods
  subscribe(callback: (log: LogEntry) => void): () => void {
    this.subscribers.add(callback)
    return () => this.subscribers.delete(callback)
  }

  subscribeToMetrics(callback: (metrics: LogMetrics) => void): () => void {
    this.metricsSubscribers.add(callback)
    return () => this.metricsSubscribers.delete(callback)
  }

  // Data access methods
  getLogs(filter?: {
    level?: LogLevel
    category?: string
    component?: string
    agentId?: string
    since?: number
  }): LogEntry[] {
    let filteredLogs = [...this.logs]

    if (filter) {
      if (filter.level !== undefined) {
        filteredLogs = filteredLogs.filter(log => log.level >= filter.level!)
      }
      if (filter.category) {
        filteredLogs = filteredLogs.filter(log => log.category === filter.category)
      }
      if (filter.component) {
        filteredLogs = filteredLogs.filter(log => log.component === filter.component)
      }
      if (filter.agentId) {
        filteredLogs = filteredLogs.filter(log => log.agentId === filter.agentId)
      }
      if (filter.since) {
        filteredLogs = filteredLogs.filter(log => log.timestamp >= filter.since!)
      }
    }

    return filteredLogs
  }

  getMetrics(): LogMetrics {
    const logsByLevel: Record<LogLevel, number> = {
      [LogLevel.DEBUG]: 0,
      [LogLevel.INFO]: 0,
      [LogLevel.WARN]: 0,
      [LogLevel.ERROR]: 0,
      [LogLevel.CRITICAL]: 0
    }

    const logsByCategory: Record<string, number> = {}
    const recentErrors: LogEntry[] = []

    this.logs.forEach(log => {
      logsByLevel[log.level]++
      logsByCategory[log.category] = (logsByCategory[log.category] || 0) + 1
      
      if (log.level >= LogLevel.ERROR && recentErrors.length < 10) {
        recentErrors.push(log)
      }
    })

    const totalErrors = logsByLevel[LogLevel.ERROR] + logsByLevel[LogLevel.CRITICAL]
    const errorRate = this.logs.length > 0 ? totalErrors / this.logs.length : 0

    const averageRenderTime = this.renderTimeHistory.length > 0
      ? this.renderTimeHistory.reduce((sum, time) => sum + time, 0) / this.renderTimeHistory.length
      : 0

    return {
      totalLogs: this.logs.length,
      logsByLevel,
      logsByCategory,
      recentErrors: recentErrors.reverse(), // Most recent first
      performanceMetrics: {
        averageRenderTime,
        memoryTrend: [...this.memoryUsageHistory],
        errorRate
      }
    }
  }

  // Utility methods
  exportLogs(format: 'json' | 'csv' = 'json'): string {
    if (format === 'json') {
      return JSON.stringify(this.logs, null, 2)
    } else {
      const headers = ['timestamp', 'level', 'category', 'component', 'message', 'agentId']
      const csvRows = [headers.join(',')]
      
      this.logs.forEach(log => {
        const row = [
          new Date(log.timestamp).toISOString(),
          LogLevel[log.level],
          log.category,
          log.component,
          `"${log.message.replace(/"/g, '""')}"`,
          log.agentId || ''
        ]
        csvRows.push(row.join(','))
      })
      
      return csvRows.join('\n')
    }
  }

  clearLogs() {
    this.logs = []
    this.memoryUsageHistory = []
    this.renderTimeHistory = []
    this.updateMetrics()
  }

  setLogLevel(level: LogLevel) {
    this.currentLogLevel = level
    this.info(`Log level changed to ${LogLevel[level]}`, 'system')
  }
}

// Singleton instance
export const logger = new Logger()

// React hook for logging
import { useEffect, useState, useCallback } from 'react'

export function useLogger(component: string) {
  const [logs, setLogs] = useState<LogEntry[]>([])
  const [metrics, setMetrics] = useState<LogMetrics | null>(null)

  useEffect(() => {
    const unsubscribeLogs = logger.subscribe((log) => {
      setLogs(prev => [...prev.slice(-99), log]) // Keep last 100 logs
    })

    const unsubscribeMetrics = logger.subscribeToMetrics((newMetrics) => {
      setMetrics(newMetrics)
    })

    // Initialize with current data
    setLogs(logger.getLogs().slice(-100))
    setMetrics(logger.getMetrics())

    return () => {
      unsubscribeLogs()
      unsubscribeMetrics()
    }
  }, [])

  const debug = useCallback((message: string, category?: string, data?: any, agentId?: string) => {
    logger.debug(message, category, data, component, agentId)
  }, [component])

  const info = useCallback((message: string, category?: string, data?: any, agentId?: string) => {
    logger.info(message, category, data, component, agentId)
  }, [component])

  const warn = useCallback((message: string, category?: string, data?: any, agentId?: string) => {
    logger.warn(message, category, data, component, agentId)
  }, [component])

  const error = useCallback((message: string, category?: string, data?: any, agentId?: string) => {
    logger.error(message, category, data, component, agentId)
  }, [component])

  const critical = useCallback((message: string, category?: string, data?: any, agentId?: string) => {
    logger.critical(message, category, data, component, agentId)
  }, [component])

  const startTimer = useCallback((label: string) => {
    logger.startPerformanceTimer(`${component}:${label}`)
  }, [component])

  const endTimer = useCallback((label: string, category?: string) => {
    return logger.endPerformanceTimer(`${component}:${label}`, category, component)
  }, [component])

  return {
    logs,
    metrics,
    debug,
    info,
    warn,
    error,
    critical,
    startTimer,
    endTimer
  }
}

// Decorator for automatic performance logging
export function logPerformance(label?: string) {
  return function (target: any, propertyName: string, descriptor: PropertyDescriptor) {
    const method = descriptor.value
    const perfLabel = label || `${target.constructor.name}.${propertyName}`

    descriptor.value = function (...args: any[]) {
      logger.startPerformanceTimer(perfLabel)
      
      try {
        const result = method.apply(this, args)
        
        if (result instanceof Promise) {
          return result.finally(() => {
            logger.endPerformanceTimer(perfLabel, 'method')
          })
        } else {
          logger.endPerformanceTimer(perfLabel, 'method')
          return result
        }
      } catch (error) {
        logger.endPerformanceTimer(perfLabel, 'method')
        logger.error(`Method ${perfLabel} failed`, 'method', { error })
        throw error
      }
    }

    return descriptor
  }
}