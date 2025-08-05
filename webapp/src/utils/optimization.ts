/**
 * Performance optimization utilities for XR-Swarm-Bridge
 * Provides caching, memoization, resource pooling, and adaptive performance features
 */

import { LRUCache } from 'lru-cache'

// Performance configuration
const PERF_CONFIG = {
  CACHE_SIZE: 1000,
  CACHE_TTL: 300000, // 5 minutes
  RENDER_THROTTLE: 16, // ~60 FPS
  TELEMETRY_BATCH_SIZE: 50,
  TELEMETRY_BATCH_INTERVAL: 1000, // 1 second
  ADAPTIVE_QUALITY_THRESHOLD: 30, // FPS threshold for quality adjustment
  WORKER_POOL_SIZE: 4,
  CONNECTION_POOL_SIZE: 10
}

// Global caches
const responseCache = new LRUCache<string, any>({
  max: PERF_CONFIG.CACHE_SIZE,
  ttl: PERF_CONFIG.CACHE_TTL
})

const computationCache = new LRUCache<string, any>({
  max: PERF_CONFIG.CACHE_SIZE,
  ttl: PERF_CONFIG.CACHE_TTL
})

// Memoization utility with configurable cache
export function memoize<T extends (...args: any[]) => any>(
  fn: T,
  keyGenerator?: (...args: Parameters<T>) => string,
  ttl?: number
): T {
  const cache = new LRUCache<string, ReturnType<T>>({
    max: 100,
    ttl: ttl || PERF_CONFIG.CACHE_TTL
  })

  return ((...args: Parameters<T>): ReturnType<T> => {
    const key = keyGenerator ? keyGenerator(...args) : JSON.stringify(args)
    
    if (cache.has(key)) {
      return cache.get(key)!
    }

    const result = fn(...args)
    cache.set(key, result)
    return result
  }) as T
}

// Throttle utility for performance-critical operations
export function throttle<T extends (...args: any[]) => any>(
  fn: T,
  delay: number
): (...args: Parameters<T>) => void {
  let lastCall = 0
  let timeoutId: number | null = null

  return (...args: Parameters<T>) => {
    const now = Date.now()
    
    if (now - lastCall >= delay) {
      lastCall = now
      fn(...args)
    } else if (!timeoutId) {
      timeoutId = window.setTimeout(() => {
        lastCall = Date.now()
        timeoutId = null
        fn(...args)
      }, delay - (now - lastCall))
    }
  }
}

// Debounce utility for input handling
export function debounce<T extends (...args: any[]) => any>(
  fn: T,
  delay: number
): (...args: Parameters<T>) => void {
  let timeoutId: number | null = null

  return (...args: Parameters<T>) => {
    if (timeoutId) {
      clearTimeout(timeoutId)
    }
    
    timeoutId = window.setTimeout(() => {
      fn(...args)
    }, delay)
  }
}

// Batch processing utility for telemetry data
class BatchProcessor<T> {
  private batch: T[] = []
  private timeoutId: number | null = null

  constructor(
    private processBatch: (items: T[]) => void,
    private batchSize: number = PERF_CONFIG.TELEMETRY_BATCH_SIZE,
    private batchInterval: number = PERF_CONFIG.TELEMETRY_BATCH_INTERVAL
  ) {}

  add(item: T): void {
    this.batch.push(item)

    if (this.batch.length >= this.batchSize) {
      this.flush()
    } else if (!this.timeoutId) {
      this.timeoutId = window.setTimeout(() => this.flush(), this.batchInterval)
    }
  }

  flush(): void {
    if (this.batch.length > 0) {
      this.processBatch([...this.batch])
      this.batch.length = 0
    }

    if (this.timeoutId) {
      clearTimeout(this.timeoutId)
      this.timeoutId = null
    }
  }

  size(): number {
    return this.batch.length
  }
}

// Telemetry batch processor
export const telemetryProcessor = new BatchProcessor<any>(
  (telemetryBatch) => {
    // Process batch of telemetry data
    console.log('Processing telemetry batch:', telemetryBatch.length, 'items')
    
    // Group by agent for efficient processing
    const byAgent = telemetryBatch.reduce((acc, data) => {
      const agentId = data.agent_id || 'unknown'
      if (!acc[agentId]) acc[agentId] = []
      acc[agentId].push(data)
      return acc
    }, {} as Record<string, any[]>)

    // Process each agent's data
    Object.entries(byAgent).forEach(([agentId, data]) => {
      // Update store with latest data for each agent
      const latest = data[data.length - 1]
      // Update agent in store...
    })
  }
)

// Adaptive quality management
class AdaptiveQualityManager {
  private currentQuality = 1.0 // 1.0 = highest quality
  private frameRateHistory: number[] = []
  private readonly historySize = 60 // 1 second at 60 FPS

  updateFrameRate(fps: number): void {
    this.frameRateHistory.push(fps)
    if (this.frameRateHistory.length > this.historySize) {
      this.frameRateHistory.shift()
    }

    this.adjustQuality()
  }

  private adjustQuality(): void {
    if (this.frameRateHistory.length < 10) return

    const avgFps = this.frameRateHistory.reduce((sum, fps) => sum + fps, 0) / this.frameRateHistory.length
    const minFps = Math.min(...this.frameRateHistory.slice(-10)) // Last 10 frames

    // Adjust quality based on performance
    if (minFps < PERF_CONFIG.ADAPTIVE_QUALITY_THRESHOLD) {
      this.currentQuality = Math.max(0.3, this.currentQuality - 0.1)
      this.applyQualitySettings()
    } else if (avgFps > PERF_CONFIG.ADAPTIVE_QUALITY_THRESHOLD + 10 && this.currentQuality < 1.0) {
      this.currentQuality = Math.min(1.0, this.currentQuality + 0.05)
      this.applyQualitySettings()
    }
  }

  private applyQualitySettings(): void {
    // Apply quality settings to different systems
    this.adjustRenderQuality()
    this.adjustNetworkQuality()
    this.adjustComputationQuality()
  }

  private adjustRenderQuality(): void {
    const quality = this.currentQuality
    
    // Adjust Three.js renderer settings
    const renderer = (window as any).threeRenderer
    if (renderer) {
      renderer.setPixelRatio(Math.max(1, window.devicePixelRatio * quality))
      
      // Adjust shadow quality
      renderer.shadowMap.enabled = quality > 0.5
      if (renderer.shadowMap.enabled) {
        const shadowMapSize = Math.floor(1024 * quality)
        renderer.shadowMap.mapSize.width = shadowMapSize
        renderer.shadowMap.mapSize.height = shadowMapSize
      }
    }

    // Adjust LOD (Level of Detail) for 3D models
    const scene = (window as any).threeScene
    if (scene) {
      scene.traverse((object: any) => {
        if (object.levels) { // LOD object
          object.update = this.createLODUpdate(quality)
        }
      })
    }
  }

  private adjustNetworkQuality(): void {
    const quality = this.currentQuality
    
    // Adjust telemetry frequency
    const telemetryInterval = Math.floor(PERF_CONFIG.TELEMETRY_BATCH_INTERVAL / quality)
    
    // Adjust video quality for robot streams
    const videoQuality = {
      width: Math.floor(1920 * quality),
      height: Math.floor(1080 * quality),
      framerate: Math.floor(30 * quality),
      bitrate: Math.floor(2000 * quality) // kbps
    }

    // Apply to WebRTC connections
    this.updateWebRTCQuality(videoQuality)
  }

  private adjustComputationQuality(): void {
    const quality = this.currentQuality
    
    // Adjust physics simulation quality
    const physicsSteps = Math.ceil(60 * quality)
    
    // Adjust pathfinding resolution
    const pathfindingResolution = Math.max(0.5, 2.0 * quality)
    
    // Adjust AI processing frequency
    const aiProcessingInterval = Math.floor(1000 / quality)
    
    // Apply these settings to respective systems
    this.updatePhysicsQuality(physicsSteps)
    this.updatePathfindingQuality(pathfindingResolution)
    this.updateAIProcessingQuality(aiProcessingInterval)
  }

  private createLODUpdate(quality: number) {
    return function(camera: any) {
      const distance = this.position.distanceTo(camera.position)
      const adjustedDistance = distance / quality // Closer objects get higher detail
      
      // Standard LOD update logic with quality adjustment
      for (let i = 0; i < this.levels.length; i++) {
        if (adjustedDistance >= this.levels[i].distance) {
          this.currentLevel = i
          break
        }
      }
    }
  }

  private updateWebRTCQuality(settings: any): void {
    // Update WebRTC video constraints
    console.log('Updating WebRTC quality:', settings)
  }

  private updatePhysicsQuality(steps: number): void {
    console.log('Updating physics quality:', steps, 'steps per second')
  }

  private updatePathfindingQuality(resolution: number): void {
    console.log('Updating pathfinding resolution:', resolution)
  }

  private updateAIProcessingQuality(interval: number): void {
    console.log('Updating AI processing interval:', interval, 'ms')
  }

  getCurrentQuality(): number {
    return this.currentQuality
  }

  getQualityLevel(): 'low' | 'medium' | 'high' {
    if (this.currentQuality < 0.4) return 'low'
    if (this.currentQuality < 0.8) return 'medium'
    return 'high'
  }
}

export const adaptiveQualityManager = new AdaptiveQualityManager()

// Resource pooling for expensive objects
class ResourcePool<T> {
  private available: T[] = []
  private inUse = new Set<T>()

  constructor(
    private createResource: () => T,
    private resetResource: (resource: T) => void,
    private maxSize: number = 10
  ) {
    // Pre-populate pool
    for (let i = 0; i < Math.min(3, maxSize); i++) {
      this.available.push(this.createResource())
    }
  }

  acquire(): T {
    let resource: T

    if (this.available.length > 0) {
      resource = this.available.pop()!
    } else if (this.inUse.size < this.maxSize) {
      resource = this.createResource()
    } else {
      throw new Error('Resource pool exhausted')
    }

    this.inUse.add(resource)
    return resource
  }

  release(resource: T): void {
    if (!this.inUse.has(resource)) {
      throw new Error('Resource not in use')
    }

    this.inUse.delete(resource)
    this.resetResource(resource)
    this.available.push(resource)
  }

  size(): { available: number; inUse: number; total: number } {
    return {
      available: this.available.length,
      inUse: this.inUse.size,
      total: this.available.length + this.inUse.size
    }
  }

  cleanup(): void {
    this.available.length = 0
    this.inUse.clear()
  }
}

// WebWorker pool for CPU-intensive tasks
class WorkerPool {
  private workers: Worker[] = []
  private availableWorkers: Worker[] = []
  private taskQueue: Array<{ data: any; resolve: Function; reject: Function }> = []

  constructor(workerScript: string, poolSize: number = PERF_CONFIG.WORKER_POOL_SIZE) {
    for (let i = 0; i < poolSize; i++) {
      const worker = new Worker(workerScript)
      this.workers.push(worker)
      this.availableWorkers.push(worker)
    }
  }

  execute<T>(data: any): Promise<T> {
    return new Promise((resolve, reject) => {
      if (this.availableWorkers.length > 0) {
        this.runTask(this.availableWorkers.pop()!, data, resolve, reject)
      } else {
        this.taskQueue.push({ data, resolve, reject })
      }
    })
  }

  private runTask(worker: Worker, data: any, resolve: Function, reject: Function): void {
    const timeoutId = setTimeout(() => {
      reject(new Error('Worker task timeout'))
    }, 30000) // 30 second timeout

    const handleMessage = (event: MessageEvent) => {
      clearTimeout(timeoutId)
      worker.removeEventListener('message', handleMessage)
      worker.removeEventListener('error', handleError)
      
      this.availableWorkers.push(worker)
      
      // Process next task in queue
      if (this.taskQueue.length > 0) {
        const next = this.taskQueue.shift()!
        this.runTask(this.availableWorkers.pop()!, next.data, next.resolve, next.reject)
      }
      
      resolve(event.data)
    }

    const handleError = (error: ErrorEvent) => {
      clearTimeout(timeoutId)
      worker.removeEventListener('message', handleMessage)
      worker.removeEventListener('error', handleError)
      
      this.availableWorkers.push(worker)
      reject(error)
    }

    worker.addEventListener('message', handleMessage)
    worker.addEventListener('error', handleError)
    worker.postMessage(data)
  }

  terminate(): void {
    this.workers.forEach(worker => worker.terminate())
    this.workers.length = 0
    this.availableWorkers.length = 0
    this.taskQueue.length = 0
  }
}

// Connection pooling for network requests
class ConnectionPool {
  private connections: Map<string, any[]> = new Map()
  private inUse: Map<string, Set<any>> = new Map()

  constructor(private maxConnectionsPerHost: number = PERF_CONFIG.CONNECTION_POOL_SIZE) {}

  async getConnection(host: string): Promise<any> {
    if (!this.connections.has(host)) {
      this.connections.set(host, [])
      this.inUse.set(host, new Set())
    }

    const available = this.connections.get(host)!
    const inUse = this.inUse.get(host)!

    if (available.length > 0) {
      const connection = available.pop()!
      inUse.add(connection)
      return connection
    }

    if (inUse.size < this.maxConnectionsPerHost) {
      const connection = await this.createConnection(host)
      inUse.add(connection)
      return connection
    }

    // Wait for a connection to become available
    return new Promise((resolve) => {
      const checkForConnection = () => {
        const available = this.connections.get(host)!
        if (available.length > 0) {
          const connection = available.pop()!
          inUse.add(connection)
          resolve(connection)
        } else {
          setTimeout(checkForConnection, 10)
        }
      }
      checkForConnection()
    })
  }

  releaseConnection(host: string, connection: any): void {
    const inUse = this.inUse.get(host)
    const available = this.connections.get(host)

    if (inUse && available && inUse.has(connection)) {
      inUse.delete(connection)
      available.push(connection)
    }
  }

  private async createConnection(host: string): Promise<any> {
    // Create actual connection (WebSocket, WebRTC, etc.)
    console.log(`Creating new connection to ${host}`)
    return { host, created: Date.now() }
  }

  cleanup(): void {
    this.connections.clear()
    this.inUse.clear()
  }
}

export const connectionPool = new ConnectionPool()

// Performance monitoring integration
export function withPerformanceMonitoring<T extends (...args: any[]) => any>(
  fn: T,
  name: string
): T {
  return ((...args: Parameters<T>): ReturnType<T> => {
    const start = performance.now()
    const result = fn(...args)
    const duration = performance.now() - start

    // Report performance metric
    if (duration > 16) { // Report if slower than 60 FPS
      console.warn(`Slow operation: ${name} took ${duration.toFixed(2)}ms`)
    }

    return result
  }) as T
}

// Lazy loading utility
export function lazy<T>(factory: () => T): () => T {
  let instance: T
  let initialized = false

  return () => {
    if (!initialized) {
      instance = factory()
      initialized = true
    }
    return instance
  }
}

// Memory management utilities
export class MemoryManager {
  private static instance: MemoryManager
  private cleanupTasks: Array<() => void> = []
  private memoryThreshold = 0.8 // 80% of available memory

  static getInstance(): MemoryManager {
    if (!MemoryManager.instance) {
      MemoryManager.instance = new MemoryManager()
    }
    return MemoryManager.instance
  }

  registerCleanupTask(cleanup: () => void): void {
    this.cleanupTasks.push(cleanup)
  }

  checkMemoryUsage(): number {
    if ('memory' in performance) {
      const memory = (performance as any).memory
      return memory.usedJSHeapSize / memory.totalJSHeapSize
    }
    return 0
  }

  performCleanup(): void {
    const memoryUsage = this.checkMemoryUsage()
    
    if (memoryUsage > this.memoryThreshold) {
      console.warn(`High memory usage detected: ${(memoryUsage * 100).toFixed(1)}%`)
      
      // Clear caches
      responseCache.clear()
      computationCache.clear()
      
      // Run cleanup tasks
      this.cleanupTasks.forEach(cleanup => {
        try {
          cleanup()
        } catch (error) {
          console.error('Cleanup task failed:', error)
        }
      })
      
      // Suggest garbage collection
      if ('gc' in window) {
        (window as any).gc()
      }
    }
  }

  startMemoryMonitoring(): void {
    setInterval(() => {
      this.performCleanup()
    }, 30000) // Check every 30 seconds
  }
}

// Initialize memory monitoring
MemoryManager.getInstance().startMemoryMonitoring()

// Export utilities
export {
  responseCache,
  computationCache,
  BatchProcessor,
  ResourcePool,
  WorkerPool,
  AdaptiveQualityManager
}