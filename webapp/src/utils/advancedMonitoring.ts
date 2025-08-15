// Advanced monitoring and observability system
import { logger } from './logger'
import { performanceOptimizer } from './performanceOptimization'

export interface MetricData {
  name: string
  value: number
  tags?: Record<string, string>
  timestamp?: number
}

export interface AlertRule {
  id: string
  metric: string
  threshold: number
  condition: 'gt' | 'lt' | 'eq'
  severity: 'low' | 'medium' | 'high' | 'critical'
  callback?: (metric: MetricData) => void
}

export interface HealthCheck {
  name: string
  check: () => Promise<boolean>
  interval: number
  timeout: number
}

class AdvancedMonitoringSystem {
  private metrics: Map<string, MetricData[]> = new Map()
  private alerts: Map<string, AlertRule> = new Map()
  private healthChecks: Map<string, HealthCheck> = new Map()
  private isMonitoring = false
  private intervals: NodeJS.Timeout[] = []

  // Core metric collection
  recordMetric(data: MetricData): void {
    const key = data.name
    const timestamp = data.timestamp || Date.now()
    const metricWithTimestamp = { ...data, timestamp }

    if (!this.metrics.has(key)) {
      this.metrics.set(key, [])
    }

    const metrics = this.metrics.get(key)!
    metrics.push(metricWithTimestamp)

    // Keep only last 1000 metrics per type
    if (metrics.length > 1000) {
      metrics.shift()
    }

    // Check alert rules
    this.checkAlerts(metricWithTimestamp)

    logger.debug('Metric recorded', {
      metric: data.name,
      value: data.value,
      tags: data.tags
    })
  }

  // Alert management
  addAlert(rule: AlertRule): void {
    this.alerts.set(rule.id, rule)
    logger.info('Alert rule added', { ruleId: rule.id, metric: rule.metric })
  }

  removeAlert(ruleId: string): void {
    this.alerts.delete(ruleId)
    logger.info('Alert rule removed', { ruleId })
  }

  private checkAlerts(metric: MetricData): void {
    for (const [ruleId, rule] of this.alerts) {
      if (rule.metric === metric.name) {
        let triggered = false

        switch (rule.condition) {
          case 'gt':
            triggered = metric.value > rule.threshold
            break
          case 'lt':
            triggered = metric.value < rule.threshold
            break
          case 'eq':
            triggered = metric.value === rule.threshold
            break
        }

        if (triggered) {
          this.triggerAlert(rule, metric)
        }
      }
    }
  }

  private triggerAlert(rule: AlertRule, metric: MetricData): void {
    const alertData = {
      ruleId: rule.id,
      severity: rule.severity,
      metric: metric.name,
      value: metric.value,
      threshold: rule.threshold,
      timestamp: Date.now()
    }

    logger.warn('Alert triggered', alertData)

    // Execute callback if provided
    if (rule.callback) {
      try {
        rule.callback(metric)
      } catch (error) {
        logger.error('Alert callback failed', { error, ruleId: rule.id })
      }
    }

    // Record alert as metric
    this.recordMetric({
      name: 'system.alerts.triggered',
      value: 1,
      tags: {
        severity: rule.severity,
        rule_id: rule.id
      }
    })
  }

  // Health checks
  addHealthCheck(check: HealthCheck): void {
    this.healthChecks.set(check.name, check)
    
    if (this.isMonitoring) {
      this.startHealthCheck(check)
    }

    logger.info('Health check added', { name: check.name })
  }

  removeHealthCheck(name: string): void {
    this.healthChecks.delete(name)
    logger.info('Health check removed', { name })
  }

  private startHealthCheck(check: HealthCheck): void {
    const interval = setInterval(async () => {
      const startTime = Date.now()
      
      try {
        const timeoutPromise = new Promise<boolean>((_, reject) => {
          setTimeout(() => reject(new Error('Health check timeout')), check.timeout)
        })

        const result = await Promise.race([
          check.check(),
          timeoutPromise
        ])

        const duration = Date.now() - startTime

        this.recordMetric({
          name: 'health.check.duration',
          value: duration,
          tags: { check_name: check.name }
        })

        this.recordMetric({
          name: 'health.check.result',
          value: result ? 1 : 0,
          tags: { check_name: check.name }
        })

        if (!result) {
          logger.warn('Health check failed', { 
            check: check.name, 
            duration 
          })
        }

      } catch (error) {
        const duration = Date.now() - startTime
        
        logger.error('Health check error', { 
          check: check.name, 
          error,
          duration
        })

        this.recordMetric({
          name: 'health.check.result',
          value: 0,
          tags: { check_name: check.name, error: 'true' }
        })
      }
    }, check.interval)

    this.intervals.push(interval)
  }

  // System monitoring
  startMonitoring(): void {
    if (this.isMonitoring) return

    this.isMonitoring = true
    logger.info('Advanced monitoring started')

    // Start system metrics collection
    this.startSystemMetrics()

    // Start all health checks
    for (const check of this.healthChecks.values()) {
      this.startHealthCheck(check)
    }

    // Setup default alerts
    this.setupDefaultAlerts()
  }

  stopMonitoring(): void {
    if (!this.isMonitoring) return

    this.isMonitoring = false
    
    // Clear all intervals
    this.intervals.forEach(interval => clearInterval(interval))
    this.intervals = []

    logger.info('Advanced monitoring stopped')
  }

  private startSystemMetrics(): void {
    // Memory usage
    const memoryInterval = setInterval(() => {
      if (performance.memory) {
        this.recordMetric({
          name: 'system.memory.used',
          value: performance.memory.usedJSHeapSize
        })

        this.recordMetric({
          name: 'system.memory.total',
          value: performance.memory.totalJSHeapSize
        })

        this.recordMetric({
          name: 'system.memory.limit',
          value: performance.memory.jsHeapSizeLimit
        })
      }
    }, 5000)

    // Performance metrics
    const perfInterval = setInterval(() => {
      const navigation = performance.getEntriesByType('navigation')[0] as PerformanceNavigationTiming
      if (navigation) {
        this.recordMetric({
          name: 'system.page.load_time',
          value: navigation.loadEventEnd - navigation.loadEventStart
        })
      }

      // FPS monitoring (approximate)
      let lastTime = performance.now()
      let frameCount = 0

      const measureFPS = () => {
        frameCount++
        const currentTime = performance.now()
        
        if (currentTime - lastTime >= 1000) {
          this.recordMetric({
            name: 'system.rendering.fps',
            value: frameCount
          })
          
          frameCount = 0
          lastTime = currentTime
        }
        
        if (this.isMonitoring) {
          requestAnimationFrame(measureFPS)
        }
      }

      requestAnimationFrame(measureFPS)
    }, 1000)

    this.intervals.push(memoryInterval, perfInterval)
  }

  private setupDefaultAlerts(): void {
    // Memory usage alert
    this.addAlert({
      id: 'high_memory_usage',
      metric: 'system.memory.used',
      threshold: 100 * 1024 * 1024, // 100MB
      condition: 'gt',
      severity: 'medium',
      callback: () => {
        logger.warn('High memory usage detected')
        performanceOptimizer.optimizeMemory()
      }
    })

    // Low FPS alert
    this.addAlert({
      id: 'low_fps',
      metric: 'system.rendering.fps',
      threshold: 30,
      condition: 'lt',
      severity: 'medium',
      callback: () => {
        logger.warn('Low FPS detected')
        performanceOptimizer.optimizeRendering()
      }
    })

    // Error rate alert
    this.addAlert({
      id: 'high_error_rate',
      metric: 'system.errors.rate',
      threshold: 5,
      condition: 'gt',
      severity: 'high'
    })
  }

  // Analytics and reporting
  getMetrics(name: string, since?: number): MetricData[] {
    const metrics = this.metrics.get(name) || []
    
    if (since) {
      return metrics.filter(m => m.timestamp! >= since)
    }
    
    return [...metrics]
  }

  getAggregatedMetrics(name: string, since?: number): {
    count: number
    min: number
    max: number
    avg: number
    sum: number
  } {
    const metrics = this.getMetrics(name, since)
    
    if (metrics.length === 0) {
      return { count: 0, min: 0, max: 0, avg: 0, sum: 0 }
    }

    const values = metrics.map(m => m.value)
    const sum = values.reduce((a, b) => a + b, 0)
    
    return {
      count: metrics.length,
      min: Math.min(...values),
      max: Math.max(...values),
      avg: sum / metrics.length,
      sum
    }
  }

  generateReport(): {
    timestamp: number
    metrics: Record<string, any>
    alerts: AlertRule[]
    health: Record<string, boolean>
  } {
    const report = {
      timestamp: Date.now(),
      metrics: {} as Record<string, any>,
      alerts: Array.from(this.alerts.values()),
      health: {} as Record<string, boolean>
    }

    // Aggregate metrics
    for (const [name] of this.metrics) {
      report.metrics[name] = this.getAggregatedMetrics(name, Date.now() - 3600000) // Last hour
    }

    return report
  }

  // Integration with external services
  exportMetrics(format: 'json' | 'prometheus' = 'json'): string {
    if (format === 'prometheus') {
      return this.exportPrometheusMetrics()
    }

    return JSON.stringify({
      timestamp: Date.now(),
      metrics: Object.fromEntries(this.metrics)
    }, null, 2)
  }

  private exportPrometheusMetrics(): string {
    let output = ''
    
    for (const [name, metrics] of this.metrics) {
      if (metrics.length === 0) continue
      
      const latest = metrics[metrics.length - 1]
      const safeName = name.replace(/[^a-zA-Z0-9_]/g, '_')
      
      output += `# TYPE ${safeName} gauge\n`
      
      if (latest.tags) {
        const labels = Object.entries(latest.tags)
          .map(([k, v]) => `${k}="${v}"`)
          .join(',')
        output += `${safeName}{${labels}} ${latest.value} ${latest.timestamp}\n`
      } else {
        output += `${safeName} ${latest.value} ${latest.timestamp}\n`
      }
    }
    
    return output
  }
}

// Singleton instance
export const monitoringSystem = new AdvancedMonitoringSystem()

// Auto-start monitoring in production
if (typeof window !== 'undefined' && process.env.NODE_ENV === 'production') {
  monitoringSystem.startMonitoring()
}

// Default health checks
monitoringSystem.addHealthCheck({
  name: 'webgl_context',
  check: async () => {
    try {
      const canvas = document.createElement('canvas')
      const gl = canvas.getContext('webgl2') || canvas.getContext('webgl')
      return gl !== null
    } catch {
      return false
    }
  },
  interval: 30000,
  timeout: 5000
})

monitoringSystem.addHealthCheck({
  name: 'local_storage',
  check: async () => {
    try {
      const testKey = '__health_check__'
      localStorage.setItem(testKey, 'test')
      localStorage.removeItem(testKey)
      return true
    } catch {
      return false
    }
  },
  interval: 60000,
  timeout: 1000
})