// Advanced performance optimization and caching system for XR-Swarm-Bridge

import { logger } from './logger'

export interface PerformanceMetrics {
  renderTime: number
  memoryUsage: number
  frameRate: number
  networkLatency: number
  cacheHitRate: number
  cpuUsage: number
}

export interface CacheOptions {
  maxSize: number
  ttl: number // time to live in milliseconds
  priority: 'lru' | 'lfu' | 'fifo'
  compression?: boolean
}

class PerformanceCache<T = any> {
  private cache = new Map<string, { value: T; timestamp: number; accessCount: number; size: number }>()
  private accessOrder: string[] = []
  private currentSize = 0
  private hitCount = 0
  private missCount = 0

  constructor(private options: CacheOptions) {}

  set(key: string, value: T): void {
    const now = Date.now()
    const serialized = JSON.stringify(value)
    const size = new Blob([serialized]).size

    // Check if we need to evict
    while (this.currentSize + size > this.options.maxSize && this.cache.size > 0) {
      this.evict()
    }

    // Remove existing entry if it exists
    if (this.cache.has(key)) {
      const existing = this.cache.get(key)!
      this.currentSize -= existing.size
      this.removeFromAccessOrder(key)
    }

    // Add new entry
    this.cache.set(key, {
      value: this.options.compression ? this.compress(value) : value,
      timestamp: now,
      accessCount: 1,
      size
    })

    this.currentSize += size
    this.accessOrder.push(key)

    // Clean up expired entries
    this.cleanupExpired()
  }

  get(key: string): T | null {
    const entry = this.cache.get(key)
    
    if (!entry) {
      this.missCount++
      return null
    }

    const now = Date.now()

    // Check if expired
    if (now - entry.timestamp > this.options.ttl) {
      this.delete(key)
      this.missCount++
      return null
    }

    // Update access tracking
    entry.accessCount++
    this.updateAccessOrder(key)
    this.hitCount++

    return this.options.compression ? this.decompress(entry.value) : entry.value
  }

  delete(key: string): boolean {
    const entry = this.cache.get(key)
    if (entry) {
      this.currentSize -= entry.size
      this.removeFromAccessOrder(key)
      this.cache.delete(key)
      return true
    }
    return false
  }

  clear(): void {
    this.cache.clear()
    this.accessOrder = []
    this.currentSize = 0
    this.hitCount = 0
    this.missCount = 0
  }

  getStats() {
    const totalRequests = this.hitCount + this.missCount
    return {
      size: this.cache.size,
      hitRate: totalRequests > 0 ? this.hitCount / totalRequests : 0,
      memoryUsage: this.currentSize,
      maxSize: this.options.maxSize
    }
  }

  private evict(): void {
    let keyToEvict: string

    switch (this.options.priority) {
      case 'lru':
        keyToEvict = this.accessOrder[0]
        break
      case 'lfu':
        keyToEvict = this.findLeastFrequentlyUsed()
        break
      case 'fifo':
      default:
        keyToEvict = this.accessOrder[0]
        break
    }

    if (keyToEvict) {
      this.delete(keyToEvict)
    }
  }

  private findLeastFrequentlyUsed(): string {
    let minAccessCount = Infinity
    let leastUsedKey = ''

    for (const [key, entry] of this.cache) {
      if (entry.accessCount < minAccessCount) {
        minAccessCount = entry.accessCount
        leastUsedKey = key
      }
    }

    return leastUsedKey
  }

  private updateAccessOrder(key: string): void {
    if (this.options.priority === 'lru') {
      this.removeFromAccessOrder(key)
      this.accessOrder.push(key)
    }
  }

  private removeFromAccessOrder(key: string): void {
    const index = this.accessOrder.indexOf(key)
    if (index > -1) {
      this.accessOrder.splice(index, 1)
    }
  }

  private compress(value: T): any {
    // Simple compression simulation - in production use actual compression
    return value
  }

  private decompress(value: any): T {
    // Simple decompression simulation
    return value
  }

  private cleanupExpired(): void {
    const now = Date.now()
    const keysToDelete: string[] = []

    for (const [key, entry] of this.cache) {
      if (now - entry.timestamp > this.options.ttl) {
        keysToDelete.push(key)
      }
    }

    keysToDelete.forEach(key => this.delete(key))
  }
}

class PerformanceOptimizer {
  private frameCounter = 0
  private lastFrameTime = performance.now()
  private frameRateHistory: number[] = []
  private memoryHistory: number[] = []
  private renderTimeHistory: number[] = []
  
  // Caches for different data types
  private agentDataCache: PerformanceCache
  private telemetryCache: PerformanceCache
  private renderCache: PerformanceCache
  private gptResponseCache: PerformanceCache

  // Performance monitoring
  private performanceObserver: PerformanceObserver | null = null
  private memoryMonitorInterval: NodeJS.Timeout | null = null

  // Adaptive quality settings
  private adaptiveQuality = {
    targetFPS: 60,
    minFPS: 30,
    maxAgentsToRender: 100,
    currentLOD: 1, // Level of Detail: 1 = high, 2 = medium, 3 = low
    enableShadows: true,
    enableReflections: true,
    particleCount: 1000
  }

  constructor() {
    // Initialize caches with different configurations
    this.agentDataCache = new PerformanceCache({
      maxSize: 50 * 1024 * 1024, // 50MB
      ttl: 30000, // 30 seconds
      priority: 'lru',
      compression: true
    })

    this.telemetryCache = new PerformanceCache({
      maxSize: 10 * 1024 * 1024, // 10MB
      ttl: 5000, // 5 seconds
      priority: 'fifo'
    })

    this.renderCache = new PerformanceCache({
      maxSize: 100 * 1024 * 1024, // 100MB
      ttl: 60000, // 1 minute
      priority: 'lru'
    })

    this.gptResponseCache = new PerformanceCache({
      maxSize: 20 * 1024 * 1024, // 20MB
      ttl: 300000, // 5 minutes
      priority: 'lfu'
    })

    this.setupPerformanceMonitoring()
    this.startAdaptiveQualitySystem()
  }

  private setupPerformanceMonitoring() {
    // Performance Observer for measuring render times
    if ('PerformanceObserver' in window) {
      this.performanceObserver = new PerformanceObserver((list) => {
        for (const entry of list.getEntries()) {
          if (entry.entryType === 'measure' && entry.name.startsWith('render-')) {
            this.renderTimeHistory.push(entry.duration)
            if (this.renderTimeHistory.length > 100) {
              this.renderTimeHistory.shift()
            }
          }
        }
      })

      this.performanceObserver.observe({ entryTypes: ['measure'] })
    }

    // Memory monitoring
    this.memoryMonitorInterval = setInterval(() => {
      if ('memory' in performance) {
        const memory = (performance as any).memory
        const usage = memory.usedJSHeapSize / (1024 * 1024) // MB
        this.memoryHistory.push(usage)
        
        if (this.memoryHistory.length > 100) {
          this.memoryHistory.shift()
        }

        // Trigger garbage collection hint if memory usage is high
        if (usage > 256 && 'gc' in window) {
          (window as any).gc()
        }
      }
    }, 5000)

    // Frame rate monitoring
    this.monitorFrameRate()
  }

  private monitorFrameRate() {
    const measureFrameRate = () => {
      this.frameCounter++
      const currentTime = performance.now()
      
      if (currentTime - this.lastFrameTime >= 1000) {
        const fps = this.frameCounter
        this.frameRateHistory.push(fps)
        
        if (this.frameRateHistory.length > 60) {
          this.frameRateHistory.shift()
        }

        this.frameCounter = 0
        this.lastFrameTime = currentTime

        // Adapt quality based on frame rate
        this.adaptQualityBasedOnFPS(fps)
      }

      requestAnimationFrame(measureFrameRate)
    }

    requestAnimationFrame(measureFrameRate)
  }

  private startAdaptiveQualitySystem() {
    setInterval(() => {
      this.optimizeBasedOnSystemLoad()
    }, 10000) // Check every 10 seconds
  }

  private adaptQualityBasedOnFPS(currentFPS: number) {
    if (currentFPS < this.adaptiveQuality.minFPS) {
      // Reduce quality
      if (this.adaptiveQuality.currentLOD < 3) {
        this.adaptiveQuality.currentLOD++
        this.adaptiveQuality.enableShadows = false
        this.adaptiveQuality.enableReflections = false
        this.adaptiveQuality.particleCount = Math.max(100, this.adaptiveQuality.particleCount * 0.5)
        this.adaptiveQuality.maxAgentsToRender = Math.max(20, this.adaptiveQuality.maxAgentsToRender * 0.8)
        
        logger.info('Reduced render quality due to low FPS', 'performance', {
          fps: currentFPS,
          newLOD: this.adaptiveQuality.currentLOD
        })
      }
    } else if (currentFPS > this.adaptiveQuality.targetFPS * 0.9) {
      // Increase quality
      if (this.adaptiveQuality.currentLOD > 1) {
        this.adaptiveQuality.currentLOD--
        this.adaptiveQuality.enableShadows = true
        this.adaptiveQuality.enableReflections = true
        this.adaptiveQuality.particleCount = Math.min(1000, this.adaptiveQuality.particleCount * 1.2)
        this.adaptiveQuality.maxAgentsToRender = Math.min(100, this.adaptiveQuality.maxAgentsToRender * 1.1)
        
        logger.info('Increased render quality due to good FPS', 'performance', {
          fps: currentFPS,
          newLOD: this.adaptiveQuality.currentLOD
        })
      }
    }
  }

  private optimizeBasedOnSystemLoad() {
    const currentMemory = this.getCurrentMemoryUsage()
    const avgFPS = this.getAverageFrameRate()
    
    // Memory pressure optimization
    if (currentMemory > 400) { // 400MB threshold
      this.performMemoryOptimization()
    }

    // CPU optimization
    if (avgFPS < this.adaptiveQuality.minFPS) {
      this.performCPUOptimization()
    }
  }

  private performMemoryOptimization() {
    logger.info('Performing memory optimization', 'performance')
    
    // Clear less important caches
    this.telemetryCache.clear()
    
    // Reduce cache sizes
    this.agentDataCache = new PerformanceCache({
      maxSize: 25 * 1024 * 1024, // Reduce to 25MB
      ttl: 15000, // Reduce TTL
      priority: 'lru'
    })

    // Trigger garbage collection if available
    if ('gc' in window) {
      (window as any).gc()
    }
  }

  private performCPUOptimization() {
    logger.info('Performing CPU optimization', 'performance')
    
    // Reduce update frequencies
    this.adaptiveQuality.maxAgentsToRender = Math.max(10, this.adaptiveQuality.maxAgentsToRender * 0.7)
    this.adaptiveQuality.currentLOD = Math.min(3, this.adaptiveQuality.currentLOD + 1)
  }

  // Public API methods
  
  cacheAgentData(agentId: string, data: any): void {
    this.agentDataCache.set(`agent_${agentId}`, data)
  }

  getCachedAgentData(agentId: string): any | null {
    return this.agentDataCache.get(`agent_${agentId}`)
  }

  cacheTelemetry(agentId: string, telemetry: any): void {
    this.telemetryCache.set(`telemetry_${agentId}_${Date.now()}`, telemetry)
  }

  getCachedTelemetry(agentId: string): any | null {
    // Get the most recent telemetry
    const keys = Array.from((this.telemetryCache as any).cache.keys())
      .filter(key => key.startsWith(`telemetry_${agentId}_`))
      .sort()
      .reverse()

    return keys.length > 0 ? this.telemetryCache.get(keys[0]) : null
  }

  cacheGPTResponse(prompt: string, response: any): void {
    const key = this.hashString(prompt)
    this.gptResponseCache.set(key, response)
  }

  getCachedGPTResponse(prompt: string): any | null {
    const key = this.hashString(prompt)
    return this.gptResponseCache.get(key)
  }

  // Optimized rendering helpers
  
  shouldRenderAgent(agentId: string, distance: number): boolean {
    // Distance-based culling
    const maxDistance = this.adaptiveQuality.currentLOD * 100
    if (distance > maxDistance) return false

    // Agent limit
    const totalAgents = Object.keys(this.agentDataCache as any).length
    if (totalAgents > this.adaptiveQuality.maxAgentsToRender) {
      // Only render closest agents
      return distance < maxDistance * 0.5
    }

    return true
  }

  getLODLevel(distance: number): number {
    if (distance < 50) return 1 // High detail
    if (distance < 150) return 2 // Medium detail
    return 3 // Low detail
  }

  getAdaptiveQualitySettings() {
    return { ...this.adaptiveQuality }
  }

  getCurrentMetrics(): PerformanceMetrics {
    return {
      renderTime: this.getAverageRenderTime(),
      memoryUsage: this.getCurrentMemoryUsage(),
      frameRate: this.getAverageFrameRate(),
      networkLatency: 0, // Would be provided by network monitor
      cacheHitRate: this.getCacheHitRate(),
      cpuUsage: 0 // Would be provided by system monitor
    }
  }

  // Batch processing helpers
  
  batchProcess<T, R>(
    items: T[],
    processor: (item: T) => R,
    batchSize: number = 10,
    delay: number = 0
  ): Promise<R[]> {
    return new Promise((resolve) => {
      const results: R[] = []
      let currentIndex = 0

      const processBatch = () => {
        const batch = items.slice(currentIndex, currentIndex + batchSize)
        const batchResults = batch.map(processor)
        results.push(...batchResults)
        
        currentIndex += batchSize

        if (currentIndex < items.length) {
          if (delay > 0) {
            setTimeout(processBatch, delay)
          } else {
            // Use requestIdleCallback if available
            if ('requestIdleCallback' in window) {
              requestIdleCallback(processBatch)
            } else {
              setTimeout(processBatch, 0)
            }
          }
        } else {
          resolve(results)
        }
      }

      processBatch()
    })
  }

  // Debounced function creator for expensive operations
  debounce<T extends (...args: any[]) => any>(
    func: T,
    wait: number,
    immediate = false
  ): (...args: Parameters<T>) => void {
    let timeout: NodeJS.Timeout | null = null

    return (...args: Parameters<T>) => {
      const later = () => {
        timeout = null
        if (!immediate) func(...args)
      }

      const callNow = immediate && !timeout

      if (timeout) clearTimeout(timeout)
      timeout = setTimeout(later, wait)

      if (callNow) func(...args)
    }
  }

  // Throttled function creator for high-frequency operations
  throttle<T extends (...args: any[]) => any>(
    func: T,
    limit: number
  ): (...args: Parameters<T>) => void {
    let inThrottle = false

    return (...args: Parameters<T>) => {
      if (!inThrottle) {
        func(...args)
        inThrottle = true
        setTimeout(() => (inThrottle = false), limit)
      }
    }
  }

  // Memory management
  cleanupUnusedData(): void {
    // Clear expired cache entries
    this.agentDataCache.clear()
    this.telemetryCache.clear()
    
    // Clear old history
    if (this.frameRateHistory.length > 60) {
      this.frameRateHistory = this.frameRateHistory.slice(-60)
    }
    
    if (this.memoryHistory.length > 100) {
      this.memoryHistory = this.memoryHistory.slice(-100)
    }
    
    if (this.renderTimeHistory.length > 100) {
      this.renderTimeHistory = this.renderTimeHistory.slice(-100)
    }

    logger.info('Cleaned up unused performance data', 'performance')
  }

  // Helper methods

  private getAverageRenderTime(): number {
    if (this.renderTimeHistory.length === 0) return 0
    return this.renderTimeHistory.reduce((sum, time) => sum + time, 0) / this.renderTimeHistory.length
  }

  private getCurrentMemoryUsage(): number {
    if ('memory' in performance) {
      const memory = (performance as any).memory
      return memory.usedJSHeapSize / (1024 * 1024) // MB
    }
    return 0
  }

  private getAverageFrameRate(): number {
    if (this.frameRateHistory.length === 0) return 0
    return this.frameRateHistory.reduce((sum, fps) => sum + fps, 0) / this.frameRateHistory.length
  }

  private getCacheHitRate(): number {
    const stats = [
      this.agentDataCache.getStats(),
      this.telemetryCache.getStats(),
      this.renderCache.getStats(),
      this.gptResponseCache.getStats()
    ]

    const totalHits = stats.reduce((sum, stat) => sum + (stat.hitRate * stat.size), 0)
    const totalRequests = stats.reduce((sum, stat) => sum + stat.size, 0)

    return totalRequests > 0 ? totalHits / totalRequests : 0
  }

  private hashString(str: string): string {
    let hash = 0
    for (let i = 0; i < str.length; i++) {
      const char = str.charCodeAt(i)
      hash = ((hash << 5) - hash) + char
      hash = hash & hash // Convert to 32-bit integer
    }
    return hash.toString()
  }

  // Cleanup
  destroy(): void {
    if (this.performanceObserver) {
      this.performanceObserver.disconnect()
    }
    
    if (this.memoryMonitorInterval) {
      clearInterval(this.memoryMonitorInterval)
    }

    this.agentDataCache.clear()
    this.telemetryCache.clear()
    this.renderCache.clear()
    this.gptResponseCache.clear()
  }
}

// Singleton instance
export const performanceOptimizer = new PerformanceOptimizer()

// Legacy alias for backward compatibility
export const performanceMonitor = performanceOptimizer

// React hook for performance monitoring
import { useState, useEffect } from 'react'

export function usePerformance() {
  const [metrics, setMetrics] = useState<PerformanceMetrics>(performanceOptimizer.getCurrentMetrics())
  const [adaptiveSettings, setAdaptiveSettings] = useState(performanceOptimizer.getAdaptiveQualitySettings())

  useEffect(() => {
    const interval = setInterval(() => {
      setMetrics(performanceOptimizer.getCurrentMetrics())
      setAdaptiveSettings(performanceOptimizer.getAdaptiveQualitySettings())
    }, 1000)

    return () => clearInterval(interval)
  }, [])

  return {
    metrics,
    adaptiveSettings,
    batchProcess: performanceOptimizer.batchProcess.bind(performanceOptimizer),
    debounce: performanceOptimizer.debounce.bind(performanceOptimizer),
    throttle: performanceOptimizer.throttle.bind(performanceOptimizer),
    shouldRenderAgent: performanceOptimizer.shouldRenderAgent.bind(performanceOptimizer),
    getLODLevel: performanceOptimizer.getLODLevel.bind(performanceOptimizer),
    cleanupUnusedData: performanceOptimizer.cleanupUnusedData.bind(performanceOptimizer)
  }
}