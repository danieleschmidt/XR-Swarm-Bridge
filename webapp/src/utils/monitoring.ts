/**
 * Monitoring and telemetry utilities for XR-Swarm-Bridge
 * Provides performance monitoring, error tracking, and system health checks
 */

// Performance metrics collection
interface PerformanceMetrics {
  timestamp: number
  frameRate: number
  memoryUsage: number
  networkLatency: number
  packetLoss: number
  bandwidth: number
  activeConnections: number
  errorCount: number
  commandProcessingTime: number
  renderTime: number
}

interface SystemHealth {
  cpu: number
  memory: number
  network: {
    latency: number
    throughput: number
    errorRate: number
  }
  webrtc: {
    activeConnections: number
    dataChannelStatus: 'open' | 'connecting' | 'closed'
    peerConnectionState: RTCPeerConnectionState
  }
  agents: {
    total: number
    active: number
    responding: number
    lowBattery: number
  }
}

class PerformanceMonitor {
  private metrics: PerformanceMetrics[] = []
  private readonly maxMetricsHistory = 1000
  private frameCount = 0
  private lastFrameTime = 0
  private observer?: PerformanceObserver
  
  constructor() {
    this.initializeObservers()
    this.startPerformanceTracking()
  }

  private initializeObservers() {
    // Performance Observer for navigation and resource timing
    if ('PerformanceObserver' in window) {
      this.observer = new PerformanceObserver((list) => {
        const entries = list.getEntries()
        entries.forEach(entry => {
          if (entry.entryType === 'navigation') {
            this.trackPageLoad(entry as PerformanceNavigationTiming)
          } else if (entry.entryType === 'resource') {
            this.trackResourceLoad(entry as PerformanceResourceTiming)
          } else if (entry.entryType === 'measure') {
            this.trackCustomMeasure(entry as PerformanceMeasure)
          }
        })
      })

      try {
        this.observer.observe({ entryTypes: ['navigation', 'resource', 'measure'] })
      } catch (error) {
        console.warn('Performance Observer not fully supported:', error)
      }
    }
  }

  private startPerformanceTracking() {
    // Track frame rate
    const trackFrame = () => {
      const now = performance.now()
      if (this.lastFrameTime) {
        const delta = now - this.lastFrameTime
        const fps = 1000 / delta
        this.updateFrameRate(fps)
      }
      this.lastFrameTime = now
      this.frameCount++
      requestAnimationFrame(trackFrame)
    }
    requestAnimationFrame(trackFrame)

    // Collect metrics every second
    setInterval(() => {
      this.collectMetrics()
    }, 1000)
  }

  private updateFrameRate(fps: number) {
    // Smooth FPS calculation using exponential moving average
    const alpha = 0.1
    const currentMetrics = this.getCurrentMetrics()
    if (currentMetrics) {
      currentMetrics.frameRate = alpha * fps + (1 - alpha) * currentMetrics.frameRate
    }
  }

  private trackPageLoad(entry: PerformanceNavigationTiming) {
    const loadTime = entry.loadEventEnd - entry.navigationStart
    console.log(`Page load time: ${loadTime}ms`)
    
    // Track critical loading phases
    const phases = {
      dns: entry.domainLookupEnd - entry.domainLookupStart,
      connection: entry.connectEnd - entry.connectStart,
      request: entry.responseStart - entry.requestStart,
      response: entry.responseEnd - entry.responseStart,
      domReady: entry.domContentLoadedEventEnd - entry.navigationStart,
      complete: entry.loadEventEnd - entry.navigationStart
    }
    
    this.reportMetric('page_load', phases)
  }

  private trackResourceLoad(entry: PerformanceResourceTiming) {
    const loadTime = entry.responseEnd - entry.startTime
    if (loadTime > 1000) { // Only track slow resources
      console.warn(`Slow resource load: ${entry.name} took ${loadTime}ms`)
      this.reportMetric('slow_resource', {
        url: entry.name,
        duration: loadTime,
        size: entry.transferSize
      })
    }
  }

  private trackCustomMeasure(entry: PerformanceMeasure) {
    this.reportMetric('custom_measure', {
      name: entry.name,
      duration: entry.duration
    })
  }

  collectMetrics(): PerformanceMetrics {
    const now = Date.now()
    
    const metrics: PerformanceMetrics = {
      timestamp: now,
      frameRate: this.getCurrentFrameRate(),
      memoryUsage: this.getMemoryUsage(),
      networkLatency: this.getNetworkLatency(),
      packetLoss: this.getPacketLoss(),
      bandwidth: this.getBandwidth(),
      activeConnections: this.getActiveConnections(),
      errorCount: this.getErrorCount(),
      commandProcessingTime: this.getCommandProcessingTime(),
      renderTime: this.getRenderTime()
    }

    this.metrics.push(metrics)
    if (this.metrics.length > this.maxMetricsHistory) {
      this.metrics.shift()
    }

    // Check for performance issues
    this.checkPerformanceThresholds(metrics)

    return metrics
  }

  private getCurrentFrameRate(): number {
    // Return smoothed frame rate
    const currentMetrics = this.getCurrentMetrics()
    return currentMetrics?.frameRate || 60
  }

  private getMemoryUsage(): number {
    if ('memory' in performance) {
      const memory = (performance as any).memory
      return memory.usedJSHeapSize / memory.totalJSHeapSize
    }
    return 0
  }

  private getNetworkLatency(): number {
    // Get from swarm store or WebRTC stats
    const swarmStore = (window as any).swarmStore?.getState?.()
    return swarmStore?.latency || 0
  }

  private getPacketLoss(): number {
    const swarmStore = (window as any).swarmStore?.getState?.()
    return swarmStore?.packetLoss || 0
  }

  private getBandwidth(): number {
    const swarmStore = (window as any).swarmStore?.getState?.()
    return swarmStore?.bandwidth || 0
  }

  private getActiveConnections(): number {
    const swarmStore = (window as any).swarmStore?.getState?.()
    return Object.keys(swarmStore?.agents || {}).length
  }

  private getErrorCount(): number {
    return this.errorTracker.getRecentErrorCount()
  }

  private getCommandProcessingTime(): number {
    // Average of recent command processing times
    return this.averageCommandTime
  }

  private getRenderTime(): number {
    // Time spent in last render cycle
    return this.lastRenderTime
  }

  private getCurrentMetrics(): PerformanceMetrics | undefined {
    return this.metrics[this.metrics.length - 1]
  }

  private checkPerformanceThresholds(metrics: PerformanceMetrics) {
    const warnings: string[] = []

    if (metrics.frameRate < 30) {
      warnings.push(`Low frame rate: ${metrics.frameRate.toFixed(1)} FPS`)
    }

    if (metrics.memoryUsage > 0.8) {
      warnings.push(`High memory usage: ${(metrics.memoryUsage * 100).toFixed(1)}%`)
    }

    if (metrics.networkLatency > 200) {
      warnings.push(`High network latency: ${metrics.networkLatency}ms`)
    }

    if (metrics.packetLoss > 0.05) {
      warnings.push(`High packet loss: ${(metrics.packetLoss * 100).toFixed(1)}%`)
    }

    if (warnings.length > 0) {
      console.warn('Performance warnings:', warnings)
      this.reportPerformanceIssue(warnings, metrics)
    }
  }

  // Public API
  getMetricsHistory(): PerformanceMetrics[] {
    return [...this.metrics]
  }

  getAverageMetrics(timeWindowMs: number = 60000): Partial<PerformanceMetrics> {
    const cutoff = Date.now() - timeWindowMs
    const recentMetrics = this.metrics.filter(m => m.timestamp > cutoff)
    
    if (recentMetrics.length === 0) return {}

    const averages: Partial<PerformanceMetrics> = {}
    const keys = Object.keys(recentMetrics[0]) as (keyof PerformanceMetrics)[]
    
    for (const key of keys) {
      if (key === 'timestamp') continue
      const values = recentMetrics.map(m => m[key] as number).filter(v => typeof v === 'number')
      averages[key] = values.reduce((sum, val) => sum + val, 0) / values.length
    }

    return averages
  }

  markMeasureStart(name: string) {
    performance.mark(`${name}-start`)
  }

  markMeasureEnd(name: string) {
    performance.mark(`${name}-end`)
    performance.measure(name, `${name}-start`, `${name}-end`)
  }

  reportMetric(name: string, data: any) {
    // Send to analytics service in production
    if (process.env.NODE_ENV === 'development') {
      console.log(`Metric [${name}]:`, data)
    }
  }

  private reportPerformanceIssue(warnings: string[], metrics: PerformanceMetrics) {
    this.reportMetric('performance_warning', {
      warnings,
      metrics,
      timestamp: Date.now()
    })
  }

  // Error tracking
  private errorTracker = new ErrorTracker()
  private averageCommandTime = 0
  private lastRenderTime = 0

  trackCommandTime(duration: number) {
    const alpha = 0.1
    this.averageCommandTime = alpha * duration + (1 - alpha) * this.averageCommandTime
  }

  trackRenderTime(duration: number) {
    this.lastRenderTime = duration
  }

  cleanup() {
    if (this.observer) {
      this.observer.disconnect()
    }
  }
}

class ErrorTracker {
  private errors: Array<{ timestamp: number; error: any; stack?: string }> = []
  private readonly maxErrorHistory = 100

  trackError(error: any, context?: string) {
    const errorInfo = {
      timestamp: Date.now(),
      error: {
        message: error.message || error.toString(),
        name: error.name,
        stack: error.stack,
        context
      },
      stack: error.stack
    }

    this.errors.push(errorInfo)
    if (this.errors.length > this.maxErrorHistory) {
      this.errors.shift()
    }

    // Report critical errors immediately
    if (this.isCriticalError(error)) {
      this.reportCriticalError(errorInfo)
    }

    console.error('Tracked error:', errorInfo)
  }

  getRecentErrorCount(timeWindowMs: number = 60000): number {
    const cutoff = Date.now() - timeWindowMs
    return this.errors.filter(e => e.timestamp > cutoff).length
  }

  getErrorHistory(): Array<{ timestamp: number; error: any; stack?: string }> {
    return [...this.errors]
  }

  private isCriticalError(error: any): boolean {
    const criticalPatterns = [
      /network/i,
      /webrtc/i,
      /websocket/i,
      /connection/i,
      /timeout/i,
      /security/i
    ]

    const errorString = error.message || error.toString()
    return criticalPatterns.some(pattern => pattern.test(errorString))
  }

  private reportCriticalError(errorInfo: any) {
    // In production, send to error monitoring service
    console.error('CRITICAL ERROR:', errorInfo)
  }
}

class HealthChecker {
  private healthStatus: SystemHealth = {
    cpu: 0,
    memory: 0,
    network: { latency: 0, throughput: 0, errorRate: 0 },
    webrtc: { activeConnections: 0, dataChannelStatus: 'closed', peerConnectionState: 'closed' },
    agents: { total: 0, active: 0, responding: 0, lowBattery: 0 }
  }

  async checkSystemHealth(): Promise<SystemHealth> {
    const health: SystemHealth = {
      cpu: await this.checkCPU(),
      memory: this.checkMemory(),
      network: await this.checkNetwork(),
      webrtc: await this.checkWebRTC(),
      agents: this.checkAgents()
    }

    this.healthStatus = health
    return health
  }

  private async checkCPU(): Promise<number> {
    // Estimate CPU usage by measuring timing precision
    const start = performance.now()
    await new Promise(resolve => setTimeout(resolve, 0))
    const end = performance.now()
    
    const expectedDelay = 0
    const actualDelay = end - start
    const cpuLoad = Math.min(actualDelay / Math.max(expectedDelay, 1), 1)
    
    return cpuLoad
  }

  private checkMemory(): number {
    if ('memory' in performance) {
      const memory = (performance as any).memory
      return memory.usedJSHeapSize / memory.totalJSHeapSize
    }
    return 0
  }

  private async checkNetwork(): Promise<SystemHealth['network']> {
    try {
      const start = performance.now()
      const response = await fetch('/health', { 
        method: 'HEAD',
        cache: 'no-cache'
      })
      const latency = performance.now() - start

      return {
        latency,
        throughput: response.ok ? 1000 : 0, // Simplified throughput measure
        errorRate: response.ok ? 0 : 1
      }
    } catch (error) {
      return {
        latency: 9999,
        throughput: 0,
        errorRate: 1
      }
    }
  }

  private async checkWebRTC(): Promise<SystemHealth['webrtc']> {
    const webrtcHook = (window as any).useWebRTC?.()
    
    return {
      activeConnections: webrtcHook?.activeConnections || 0,
      dataChannelStatus: 'closed', // Would be determined from actual WebRTC state
      peerConnectionState: 'closed' // Would be determined from actual WebRTC state
    }
  }

  private checkAgents(): SystemHealth['agents'] {
    const swarmStore = (window as any).swarmStore?.getState?.()
    const agents = Object.values(swarmStore?.agents || {}) as any[]

    return {
      total: agents.length,
      active: agents.filter(a => a.status === 'active').length,
      responding: agents.filter(a => Date.now() - a.lastSeen < 10000).length,
      lowBattery: agents.filter(a => a.battery < 20).length
    }
  }

  getHealthStatus(): SystemHealth {
    return { ...this.healthStatus }
  }

  isHealthy(): boolean {
    const health = this.healthStatus
    
    return (
      health.cpu < 0.8 &&
      health.memory < 0.8 &&
      health.network.latency < 500 &&
      health.network.errorRate < 0.1 &&
      health.agents.responding / Math.max(health.agents.total, 1) > 0.8
    )
  }
}

// Global instances
export const performanceMonitor = new PerformanceMonitor()
export const errorTracker = new ErrorTracker()
export const healthChecker = new HealthChecker()

// Global error handler
window.addEventListener('error', (event) => {
  errorTracker.trackError(event.error, 'Global error handler')
})

window.addEventListener('unhandledrejection', (event) => {
  errorTracker.trackError(event.reason, 'Unhandled promise rejection')
})

// Periodic health checks
setInterval(async () => {
  await healthChecker.checkSystemHealth()
}, 30000) // Check every 30 seconds

// Performance monitoring hooks for React components
export function usePerformanceMonitoring(componentName: string) {
  const startTime = performance.now()

  return {
    markRenderStart: () => performanceMonitor.markMeasureStart(`${componentName}-render`),
    markRenderEnd: () => performanceMonitor.markMeasureEnd(`${componentName}-render`),
    trackError: (error: any) => errorTracker.trackError(error, componentName),
    trackCommandTime: (duration: number) => performanceMonitor.trackCommandTime(duration)
  }
}

// Export utilities
export {
  PerformanceMetrics,
  SystemHealth,
  PerformanceMonitor,
  ErrorTracker,
  HealthChecker
}