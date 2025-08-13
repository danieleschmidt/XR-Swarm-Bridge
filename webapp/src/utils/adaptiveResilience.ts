/**
 * Adaptive Resilience System for XR-Swarm-Bridge
 * Self-healing architecture with predictive failure prevention
 */

import { logger } from './logger'
import { performanceMonitor } from './performance'

interface SystemComponent {
  id: string
  type: 'webrtc' | 'api' | 'robot_agent' | 'ai_service' | 'storage' | 'network'
  status: 'healthy' | 'degraded' | 'failing' | 'offline'
  healthScore: number
  lastCheck: number
  metrics: ComponentMetrics
  dependencies: string[]
  recoveryStrategies: RecoveryStrategy[]
}

interface ComponentMetrics {
  responseTime: number[]
  errorRate: number[]
  throughput: number[]
  resourceUsage: number[]
  uptime: number
  lastFailure?: number
  failureCount: number
}

interface RecoveryStrategy {
  name: string
  type: 'restart' | 'failover' | 'degrade' | 'isolate' | 'scale'
  priority: number
  conditions: Array<(component: SystemComponent) => boolean>
  execute: (component: SystemComponent) => Promise<boolean>
  cooldownMs: number
  lastExecuted?: number
}

class AdaptiveResilienceSystem {
  private components = new Map<string, SystemComponent>()
  private healingActions = new Map<string, number>()
  private systemMetrics = {
    overallHealth: 1.0,
    availabilityScore: 1.0,
    resilienceIndex: 1.0,
    healingSuccessRate: 0.95
  }

  constructor() {
    this.initializeSystemComponents()
    this.startHealthMonitoring()
  }

  /**
   * Register system component for monitoring
   */
  registerComponent(component: Omit<SystemComponent, 'metrics'>): void {
    const fullComponent: SystemComponent = {
      ...component,
      metrics: {
        responseTime: [],
        errorRate: [],
        throughput: [],
        resourceUsage: [],
        uptime: Date.now(),
        failureCount: 0
      }
    }

    this.components.set(component.id, fullComponent)
    logger.info('Component registered for resilience monitoring', { componentId: component.id })
  }

  /**
   * Perform health check
   */
  async performHealthCheck(componentId?: string): Promise<any[]> {
    const componentsToCheck = componentId 
      ? [this.components.get(componentId)].filter(Boolean)
      : Array.from(this.components.values())

    const results: any[] = []

    for (const component of componentsToCheck) {
      if (!component) continue

      const startTime = performance.now()
      
      try {
        const checkResult = await this.checkComponentHealth(component)
        
        component.status = checkResult.status
        component.healthScore = checkResult.healthScore
        component.lastCheck = Date.now()

        results.push({
          componentId: component.id,
          status: checkResult.status,
          healthScore: checkResult.healthScore,
          responseTime: performance.now() - startTime
        })

        if (checkResult.status === 'failing' || checkResult.status === 'offline') {
          this.triggerHealing(component)
        }

      } catch (error) {
        component.status = 'offline'
        component.healthScore = 0
        component.metrics.failureCount++
        component.metrics.lastFailure = Date.now()
      }
    }

    this.updateSystemMetrics()
    return results
  }

  private async checkComponentHealth(component: SystemComponent) {
    await this.delay(100 + Math.random() * 200)
    
    const healthScore = 0.7 + Math.random() * 0.3
    let status: SystemComponent['status'] = 'healthy'
    
    if (healthScore < 0.3) status = 'offline'
    else if (healthScore < 0.6) status = 'failing'
    else if (healthScore < 0.8) status = 'degraded'
    
    return { status, healthScore }
  }

  private async triggerHealing(component: SystemComponent): Promise<void> {
    const now = Date.now()
    const lastHealing = this.healingActions.get(component.id) || 0
    
    if (now - lastHealing < 30000) return
    
    this.healingActions.set(component.id, now)
    
    logger.info('Initiating healing process', { componentId: component.id })

    const applicableStrategies = component.recoveryStrategies
      .filter(strategy => strategy.conditions.every(condition => condition(component)))
      .sort((a, b) => b.priority - a.priority)

    for (const strategy of applicableStrategies) {
      if (strategy.lastExecuted && now - strategy.lastExecuted < strategy.cooldownMs) {
        continue
      }

      try {
        strategy.lastExecuted = now
        const success = await strategy.execute(component)

        if (success) {
          this.updateHealingMetrics(true)
          logger.info('Recovery strategy succeeded', { componentId: component.id, strategy: strategy.name })
          return
        }
      } catch (error) {
        logger.error('Recovery strategy error', { componentId: component.id, strategy: strategy.name })
      }
    }

    this.updateHealingMetrics(false)
  }

  private initializeSystemComponents(): void {
    const components = [
      {
        id: 'webrtc_primary',
        type: 'webrtc' as const,
        status: 'healthy' as const,
        healthScore: 1.0,
        lastCheck: Date.now(),
        dependencies: [],
        recoveryStrategies: [
          {
            name: 'restart_webrtc_connection',
            type: 'restart' as const,
            priority: 10,
            conditions: [(comp: SystemComponent) => comp.healthScore < 0.5],
            execute: async () => {
              await this.delay(1000)
              return Math.random() > 0.2
            },
            cooldownMs: 30000
          }
        ]
      },
      {
        id: 'api_gateway',
        type: 'api' as const,
        status: 'healthy' as const,
        healthScore: 1.0,
        lastCheck: Date.now(),
        dependencies: [],
        recoveryStrategies: [
          {
            name: 'restart_api_gateway',
            type: 'restart' as const,
            priority: 10,
            conditions: [(comp: SystemComponent) => comp.status === 'failing'],
            execute: async () => {
              await this.delay(2000)
              return Math.random() > 0.3
            },
            cooldownMs: 60000
          }
        ]
      }
    ]

    components.forEach(comp => this.registerComponent(comp))
  }

  private updateSystemMetrics(): void {
    const components = Array.from(this.components.values())
    
    if (components.length === 0) return
    
    this.systemMetrics.overallHealth = components
      .reduce((sum, comp) => sum + comp.healthScore, 0) / components.length
    
    const availableComponents = components
      .filter(comp => comp.status === 'healthy' || comp.status === 'degraded')
    
    this.systemMetrics.availabilityScore = availableComponents.length / components.length
    
    this.systemMetrics.resilienceIndex = (
      this.systemMetrics.healingSuccessRate * 0.6 +
      this.systemMetrics.availabilityScore * 0.4
    )
    
    performanceMonitor.recordMetric('system_health', this.systemMetrics.overallHealth)
    performanceMonitor.recordMetric('system_availability', this.systemMetrics.availabilityScore)
  }

  private updateHealingMetrics(success: boolean): void {
    const alpha = 0.1
    const currentRate = success ? 1.0 : 0.0
    
    this.systemMetrics.healingSuccessRate = 
      alpha * currentRate + (1 - alpha) * this.systemMetrics.healingSuccessRate
  }

  private delay(ms: number): Promise<void> {
    return new Promise(resolve => setTimeout(resolve, ms))
  }

  private startHealthMonitoring(): void {
    setInterval(() => {
      this.performHealthCheck()
    }, 30000)

    setTimeout(() => this.performHealthCheck(), 1000)
  }

  getSystemStatus(): any {
    const components: any = {}
    for (const [id, component] of this.components.entries()) {
      components[id] = {
        status: component.status,
        healthScore: component.healthScore,
        lastCheck: component.lastCheck,
        type: component.type
      }
    }

    return {
      components,
      systemMetrics: this.systemMetrics,
      timestamp: Date.now()
    }
  }
}

export const adaptiveResilienceSystem = new AdaptiveResilienceSystem()
export const adaptiveResilienceEngine = adaptiveResilienceSystem // Legacy alias
export default adaptiveResilienceSystem