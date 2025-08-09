// Performance optimization utilities for XR-Swarm-Bridge

interface CacheEntry<T> {
  data: T
  timestamp: number
  expiresAt: number
}

// High-performance caching system
export class PerformanceCache<T = any> {
  private cache = new Map<string, CacheEntry<T>>()
  private timers = new Map<string, NodeJS.Timeout>()
  
  constructor(
    private defaultTTL = 300000, // 5 minutes
    private maxSize = 10000,
    private cleanupInterval = 60000 // 1 minute
  ) {
    // Start cleanup interval
    setInterval(() => this.cleanup(), cleanupInterval)
  }
  
  set(key: string, data: T, ttl?: number): void {
    const now = Date.now()
    const expiry = now + (ttl || this.defaultTTL)
    
    // Remove existing entry if present
    if (this.cache.has(key)) {
      this.delete(key)
    }
    
    // Check size limit
    if (this.cache.size >= this.maxSize) {
      this.evictLRU()
    }
    
    this.cache.set(key, {
      data,
      timestamp: now,
      expiresAt: expiry
    })
    
    // Set expiry timer
    const timer = setTimeout(() => this.delete(key), ttl || this.defaultTTL)
    this.timers.set(key, timer)
  }
  
  get(key: string): T | null {
    const entry = this.cache.get(key)
    
    if (!entry) {
      return null
    }
    
    if (Date.now() > entry.expiresAt) {
      this.delete(key)
      return null
    }
    
    return entry.data
  }
  
  has(key: string): boolean {
    return this.get(key) !== null
  }
  
  delete(key: string): boolean {
    const timer = this.timers.get(key)
    if (timer) {
      clearTimeout(timer)
      this.timers.delete(key)
    }
    
    return this.cache.delete(key)
  }
  
  clear(): void {
    for (const timer of this.timers.values()) {
      clearTimeout(timer)
    }
    this.timers.clear()
    this.cache.clear()
  }
  
  private evictLRU(): void {
    let oldestKey: string | null = null
    let oldestTime = Date.now()
    
    for (const [key, entry] of this.cache.entries()) {
      if (entry.timestamp < oldestTime) {
        oldestTime = entry.timestamp
        oldestKey = key
      }
    }
    
    if (oldestKey) {
      this.delete(oldestKey)
    }
  }
  
  private cleanup(): void {
    const now = Date.now()
    const expiredKeys: string[] = []
    
    for (const [key, entry] of this.cache.entries()) {
      if (now > entry.expiresAt) {
        expiredKeys.push(key)
      }
    }
    
    for (const key of expiredKeys) {
      this.delete(key)
    }
  }
  
  getStats() {
    return {
      size: this.cache.size,
      maxSize: this.maxSize,
      usage: (this.cache.size / this.maxSize) * 100
    }
  }
}

// WebWorker pool for CPU-intensive tasks
export class WorkerPool {
  private workers: Worker[] = []
  private availableWorkers: Worker[] = []
  private taskQueue: Array<{ resolve: Function; reject: Function; data: any }> = []
  
  constructor(
    workerScript: string,
    poolSize: number = navigator.hardwareConcurrency || 4
  ) {
    // Initialize workers
    for (let i = 0; i < poolSize; i++) {
      const worker = new Worker(workerScript)
      worker.onmessage = (event) => this.handleWorkerMessage(worker, event)
      worker.onerror = (error) => this.handleWorkerError(worker, error)
      
      this.workers.push(worker)
      this.availableWorkers.push(worker)
    }
  }
  
  execute<T>(data: any): Promise<T> {
    return new Promise((resolve, reject) => {
      const task = { resolve, reject, data }
      
      const worker = this.availableWorkers.pop()
      if (worker) {
        this.runTask(worker, task)
      } else {
        this.taskQueue.push(task)
      }
    })
  }
  
  private runTask(worker: Worker, task: { resolve: Function; reject: Function; data: any }) {
    // Store task reference on worker for completion handling
    ;(worker as any).currentTask = task
    worker.postMessage(task.data)
  }
  
  private handleWorkerMessage(worker: Worker, event: MessageEvent) {
    const task = (worker as any).currentTask
    if (task) {
      task.resolve(event.data)
      ;(worker as any).currentTask = null
      this.returnWorker(worker)
    }
  }
  
  private handleWorkerError(worker: Worker, error: ErrorEvent) {
    const task = (worker as any).currentTask
    if (task) {
      task.reject(new Error(error.message))
      ;(worker as any).currentTask = null
      this.returnWorker(worker)
    }
  }
  
  private returnWorker(worker: Worker) {
    // Check if there are queued tasks
    const nextTask = this.taskQueue.shift()
    if (nextTask) {
      this.runTask(worker, nextTask)
    } else {
      this.availableWorkers.push(worker)
    }
  }
  
  terminate() {
    for (const worker of this.workers) {
      worker.terminate()
    }
    this.workers.length = 0
    this.availableWorkers.length = 0
    this.taskQueue.length = 0
  }
  
  getStats() {
    return {
      totalWorkers: this.workers.length,
      availableWorkers: this.availableWorkers.length,
      queuedTasks: this.taskQueue.length,
      utilization: ((this.workers.length - this.availableWorkers.length) / this.workers.length) * 100
    }
  }
}

// Request deduplication and batching
export class RequestBatcher {
  private batches = new Map<string, Array<{ resolve: Function; reject: Function; data: any }>>()
  private timers = new Map<string, NodeJS.Timeout>()
  
  constructor(
    private batchSize = 10,
    private batchTimeout = 100 // ms
  ) {}
  
  add<T>(batchKey: string, data: any): Promise<T> {
    return new Promise((resolve, reject) => {
      if (!this.batches.has(batchKey)) {
        this.batches.set(batchKey, [])
      }
      
      const batch = this.batches.get(batchKey)!
      batch.push({ resolve, reject, data })
      
      // Execute batch if it reaches size limit
      if (batch.length >= this.batchSize) {
        this.executeBatch(batchKey)
      } else if (!this.timers.has(batchKey)) {
        // Set timeout for batch execution
        const timer = setTimeout(() => {
          this.executeBatch(batchKey)
        }, this.batchTimeout)
        this.timers.set(batchKey, timer)
      }
    })
  }
  
  private executeBatch(batchKey: string) {
    const batch = this.batches.get(batchKey)
    if (!batch || batch.length === 0) {
      return
    }
    
    // Clear timer
    const timer = this.timers.get(batchKey)
    if (timer) {
      clearTimeout(timer)
      this.timers.delete(batchKey)
    }
    
    // Execute batch
    this.processBatch(batchKey, batch)
    
    // Clear batch
    this.batches.delete(batchKey)
  }
  
  private async processBatch(batchKey: string, batch: Array<{ resolve: Function; reject: Function; data: any }>) {
    try {
      const batchData = batch.map(item => item.data)
      const results = await this.handleBatch(batchKey, batchData)
      
      // Resolve individual promises
      batch.forEach((item, index) => {
        item.resolve(results[index])
      })
    } catch (error) {
      // Reject all promises in batch
      batch.forEach(item => {
        item.reject(error)
      })
    }
  }
  
  // Override this method in subclasses
  protected async handleBatch(batchKey: string, batchData: any[]): Promise<any[]> {
    throw new Error('handleBatch must be implemented')
  }
}

// Telemetry data batching
export class TelemetryBatcher extends RequestBatcher {
  constructor(
    private sendFunction: (data: any[]) => Promise<any[]>
  ) {
    super(50, 16) // 50 items or 16ms batches for high-frequency telemetry
  }
  
  protected async handleBatch(batchKey: string, batchData: any[]): Promise<any[]> {
    return this.sendFunction(batchData)
  }
}

// Adaptive quality management
export class AdaptiveQualityManager {
  private performanceMetrics = {
    frameRate: 60,
    latency: 0,
    packetLoss: 0,
    bandwidth: 0,
    cpuUsage: 0,
    memoryUsage: 0
  }
  
  private qualitySettings = {
    renderDistance: 1000,
    particleCount: 1000,
    shadowQuality: 'high',
    antialiasing: true,
    motionBlur: true,
    telemetryFrequency: 10, // Hz
    videoQuality: 'high'
  }
  
  updateMetrics(metrics: Partial<typeof this.performanceMetrics>) {
    Object.assign(this.performanceMetrics, metrics)
    this.adaptQuality()
  }
  
  private adaptQuality() {
    const { frameRate, latency, cpuUsage, memoryUsage, packetLoss } = this.performanceMetrics
    
    // Adapt based on frame rate
    if (frameRate < 30) {
      this.qualitySettings.renderDistance = Math.max(500, this.qualitySettings.renderDistance * 0.8)
      this.qualitySettings.particleCount = Math.max(100, this.qualitySettings.particleCount * 0.7)
      this.qualitySettings.shadowQuality = 'low'
      this.qualitySettings.antialiasing = false
      this.qualitySettings.motionBlur = false
    } else if (frameRate > 50) {
      this.qualitySettings.renderDistance = Math.min(2000, this.qualitySettings.renderDistance * 1.1)
      this.qualitySettings.particleCount = Math.min(2000, this.qualitySettings.particleCount * 1.1)
      if (frameRate > 55) {
        this.qualitySettings.shadowQuality = 'high'
        this.qualitySettings.antialiasing = true
      }
    }
    
    // Adapt based on network conditions
    if (latency > 200 || packetLoss > 0.05) {
      this.qualitySettings.telemetryFrequency = Math.max(5, this.qualitySettings.telemetryFrequency - 1)
      this.qualitySettings.videoQuality = 'medium'
    } else if (latency < 100 && packetLoss < 0.01) {
      this.qualitySettings.telemetryFrequency = Math.min(20, this.qualitySettings.telemetryFrequency + 1)
      this.qualitySettings.videoQuality = 'high'
    }
    
    // Adapt based on system resources
    if (cpuUsage > 80 || memoryUsage > 80) {
      this.qualitySettings.renderDistance *= 0.9
      this.qualitySettings.particleCount *= 0.8
    }
  }
  
  getSettings() {
    return { ...this.qualitySettings }
  }
  
  getMetrics() {
    return { ...this.performanceMetrics }
  }
}

// Connection pooling for WebSocket connections
export class ConnectionPool {
  private connections: WebSocket[] = []
  private availableConnections: WebSocket[] = []
  private pendingRequests: Array<{ resolve: Function; reject: Function }> = []
  
  constructor(
    private createConnection: () => WebSocket,
    private maxConnections = 10
  ) {}
  
  async getConnection(): Promise<WebSocket> {
    return new Promise((resolve, reject) => {
      const available = this.availableConnections.pop()
      if (available && available.readyState === WebSocket.OPEN) {
        resolve(available)
        return
      }
      
      if (this.connections.length < this.maxConnections) {
        try {
          const conn = this.createConnection()
          this.connections.push(conn)
          
          conn.onopen = () => resolve(conn)
          conn.onerror = () => reject(new Error('Connection failed'))
          conn.onclose = () => {
            this.removeConnection(conn)
          }
        } catch (error) {
          reject(error)
        }
      } else {
        this.pendingRequests.push({ resolve, reject })
      }
    })
  }
  
  returnConnection(connection: WebSocket) {
    if (connection.readyState === WebSocket.OPEN) {
      const pending = this.pendingRequests.shift()
      if (pending) {
        pending.resolve(connection)
      } else {
        this.availableConnections.push(connection)
      }
    } else {
      this.removeConnection(connection)
    }
  }
  
  private removeConnection(connection: WebSocket) {
    const index = this.connections.indexOf(connection)
    if (index > -1) {
      this.connections.splice(index, 1)
    }
    
    const availableIndex = this.availableConnections.indexOf(connection)
    if (availableIndex > -1) {
      this.availableConnections.splice(availableIndex, 1)
    }
  }
  
  close() {
    for (const conn of this.connections) {
      conn.close()
    }
    this.connections.length = 0
    this.availableConnections.length = 0
    
    // Reject pending requests
    for (const pending of this.pendingRequests) {
      pending.reject(new Error('Connection pool closed'))
    }
    this.pendingRequests.length = 0
  }
}

// Memory management utilities
export class MemoryManager {
  private objectPools = new Map<string, any[]>()
  private refs = new WeakMap<object, string>()
  
  getFromPool<T>(poolName: string, factory: () => T): T {
    if (!this.objectPools.has(poolName)) {
      this.objectPools.set(poolName, [])
    }
    
    const pool = this.objectPools.get(poolName)!
    const obj = pool.pop() || factory()
    
    this.refs.set(obj, poolName)
    return obj
  }
  
  returnToPool<T extends object>(obj: T) {
    const poolName = this.refs.get(obj)
    if (poolName && this.objectPools.has(poolName)) {
      const pool = this.objectPools.get(poolName)!
      if (pool.length < 100) { // Limit pool size
        pool.push(obj)
      }
    }
  }
  
  clearPool(poolName: string) {
    this.objectPools.delete(poolName)
  }
  
  clearAllPools() {
    this.objectPools.clear()
  }
  
  getMemoryUsage(): MemoryInfo | undefined {
    // @ts-ignore - performance.memory is not in all browsers
    return (performance as any).memory
  }
}

// Singleton instances
export const performanceCache = new PerformanceCache()
export const qualityManager = new AdaptiveQualityManager()
export const memoryManager = new MemoryManager()

// Performance monitoring
export class PerformanceMonitor {
  private metrics: Array<{ timestamp: number; [key: string]: any }> = []
  private observers: PerformanceObserver[] = []
  
  constructor() {
    this.setupObservers()
  }
  
  private setupObservers() {
    // Frame timing
    if ('requestIdleCallback' in window) {
      const measureFrameTime = () => {
        const start = performance.now()
        requestIdleCallback(() => {
          const frameTime = performance.now() - start
          this.recordMetric('frameTime', frameTime)
        })
        requestAnimationFrame(measureFrameTime)
      }
      measureFrameTime()
    }
    
    // Long tasks
    try {
      const longTaskObserver = new PerformanceObserver((list) => {
        for (const entry of list.getEntries()) {
          this.recordMetric('longTask', entry.duration)
        }
      })
      longTaskObserver.observe({ entryTypes: ['longtask'] })
      this.observers.push(longTaskObserver)
    } catch (e) {
      // PerformanceObserver not supported
    }
    
    // Navigation timing
    try {
      const navigationObserver = new PerformanceObserver((list) => {
        for (const entry of list.getEntries()) {
          this.recordMetric('navigation', {
            domContentLoaded: entry.domContentLoadedEventEnd - entry.domContentLoadedEventStart,
            loadComplete: entry.loadEventEnd - entry.loadEventStart
          })
        }
      })
      navigationObserver.observe({ entryTypes: ['navigation'] })
      this.observers.push(navigationObserver)
    } catch (e) {
      // Not supported
    }
  }
  
  recordMetric(name: string, value: any) {
    this.metrics.push({
      timestamp: Date.now(),
      name,
      value
    })
    
    // Keep only recent metrics
    if (this.metrics.length > 1000) {
      this.metrics = this.metrics.slice(-1000)
    }
    
    // Update quality manager
    if (name === 'frameTime' && typeof value === 'number') {
      const fps = 1000 / value
      qualityManager.updateMetrics({ frameRate: fps })
    }
  }
  
  getMetrics(name?: string, since?: number) {
    let filtered = this.metrics
    
    if (name) {
      filtered = filtered.filter(m => m.name === name)
    }
    
    if (since) {
      filtered = filtered.filter(m => m.timestamp > since)
    }
    
    return filtered
  }
  
  getAverageMetric(name: string, since?: number): number {
    const metrics = this.getMetrics(name, since)
    if (metrics.length === 0) return 0
    
    const sum = metrics.reduce((acc, m) => acc + (typeof m.value === 'number' ? m.value : 0), 0)
    return sum / metrics.length
  }
  
  destroy() {
    for (const observer of this.observers) {
      observer.disconnect()
    }
    this.observers.length = 0
  }
}

export const performanceMonitor = new PerformanceMonitor()

export default {
  PerformanceCache,
  WorkerPool,
  RequestBatcher,
  TelemetryBatcher,
  AdaptiveQualityManager,
  ConnectionPool,
  MemoryManager,
  PerformanceMonitor,
  performanceCache,
  qualityManager,
  memoryManager,
  performanceMonitor
}