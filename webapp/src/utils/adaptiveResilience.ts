/**
 * Adaptive Resilience Engine for XR-Swarm-Bridge
 * Implements self-healing, circuit breakers, and adaptive monitoring
 */

export interface HealthCheck {
  id: string;
  name: string;
  type: 'system' | 'network' | 'robot' | 'service';
  status: 'healthy' | 'warning' | 'critical' | 'unknown';
  lastCheck: Date;
  metrics: Record<string, number>;
  threshold: Record<string, number>;
}

export interface CircuitBreaker {
  id: string;
  service: string;
  state: 'closed' | 'open' | 'half-open';
  failureCount: number;
  failureThreshold: number;
  timeout: number;
  lastFailure?: Date;
  successCount: number;
}

export interface ResiliencePolicy {
  id: string;
  name: string;
  triggers: string[];
  actions: Array<{
    type: 'retry' | 'fallback' | 'circuit-break' | 'scale' | 'isolate';
    parameters: Record<string, any>;
    priority: number;
  }>;
  cooldown: number;
  maxExecutions: number;
}

export interface SystemRecoveryPlan {
  scenario: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  steps: Array<{
    action: string;
    timeout: number;
    rollback?: string;
  }>;
  estimatedRecoveryTime: number;
}

export class AdaptiveResilienceEngine {
  private healthChecks: Map<string, HealthCheck> = new Map();
  private circuitBreakers: Map<string, CircuitBreaker> = new Map();
  private resiliencePolicies: Map<string, ResiliencePolicy> = new Map();
  private recoveryPlans: Map<string, SystemRecoveryPlan> = new Map();
  private monitoringInterval?: NodeJS.Timeout;
  private adaptationHistory: Array<{ timestamp: Date; action: string; result: string }> = [];
  private systemMetrics: Map<string, number[]> = new Map();

  constructor() {
    this.initializeHealthChecks();
    this.initializeCircuitBreakers();
    this.initializeResiliencePolicies();
    this.initializeRecoveryPlans();
    this.startContinuousMonitoring();
  }

  private initializeHealthChecks(): void {
    const healthChecks: HealthCheck[] = [
      {
        id: 'webrtc_connection',
        name: 'WebRTC Connection Health',
        type: 'network',
        status: 'healthy',
        lastCheck: new Date(),
        metrics: { latency: 0, packetLoss: 0, bandwidth: 0 },
        threshold: { latency: 500, packetLoss: 0.05, bandwidth: 1000000 }
      },
      {
        id: 'robot_swarm_health',
        name: 'Robot Swarm Health',
        type: 'robot',
        status: 'healthy',
        lastCheck: new Date(),
        metrics: { activeRobots: 0, responseRate: 0, errorRate: 0 },
        threshold: { activeRobots: 50, responseRate: 0.95, errorRate: 0.05 }
      },
      {
        id: 'gpt_api_health',
        name: 'GPT API Health',
        type: 'service',
        status: 'healthy',
        lastCheck: new Date(),
        metrics: { responseTime: 0, successRate: 0, rateLimitUsage: 0 },
        threshold: { responseTime: 5000, successRate: 0.98, rateLimitUsage: 0.8 }
      },
      {
        id: 'system_resources',
        name: 'System Resource Health',
        type: 'system',
        status: 'healthy',
        lastCheck: new Date(),
        metrics: { cpuUsage: 0, memoryUsage: 0, diskUsage: 0 },
        threshold: { cpuUsage: 0.8, memoryUsage: 0.85, diskUsage: 0.9 }
      }
    ];

    healthChecks.forEach(hc => this.healthChecks.set(hc.id, hc));
  }

  private initializeCircuitBreakers(): void {
    const circuitBreakers: CircuitBreaker[] = [
      {
        id: 'gpt_api_breaker',
        service: 'GPT API',
        state: 'closed',
        failureCount: 0,
        failureThreshold: 5,
        timeout: 60000, // 1 minute
        successCount: 0
      },
      {
        id: 'robot_command_breaker',
        service: 'Robot Commands',
        state: 'closed',
        failureCount: 0,
        failureThreshold: 10,
        timeout: 30000, // 30 seconds
        successCount: 0
      },
      {
        id: 'webrtc_stream_breaker',
        service: 'WebRTC Streams',
        state: 'closed',
        failureCount: 0,
        failureThreshold: 3,
        timeout: 15000, // 15 seconds
        successCount: 0
      }
    ];

    circuitBreakers.forEach(cb => this.circuitBreakers.set(cb.id, cb));
  }

  private initializeResiliencePolicies(): void {
    const policies: ResiliencePolicy[] = [
      {
        id: 'high_latency_policy',
        name: 'High Latency Mitigation',
        triggers: ['latency > 500ms', 'packet_loss > 0.05'],
        actions: [
          { type: 'fallback', parameters: { mode: 'autonomous' }, priority: 1 },
          { type: 'retry', parameters: { attempts: 3, delay: 1000 }, priority: 2 }
        ],
        cooldown: 30000,
        maxExecutions: 5
      },
      {
        id: 'robot_failure_policy',
        name: 'Robot Failure Recovery',
        triggers: ['robot_error_rate > 0.1', 'robot_count_drop > 0.2'],
        actions: [
          { type: 'isolate', parameters: { failedRobots: true }, priority: 1 },
          { type: 'scale', parameters: { replaceCount: 2 }, priority: 2 }
        ],
        cooldown: 60000,
        maxExecutions: 3
      },
      {
        id: 'service_degradation_policy',
        name: 'Service Degradation Response',
        triggers: ['api_success_rate < 0.95', 'response_time > 5000ms'],
        actions: [
          { type: 'circuit-break', parameters: { service: 'api' }, priority: 1 },
          { type: 'fallback', parameters: { localMode: true }, priority: 2 }
        ],
        cooldown: 120000,
        maxExecutions: 2
      }
    ];

    policies.forEach(policy => this.resiliencePolicies.set(policy.id, policy));
  }

  private initializeRecoveryPlans(): void {
    const plans: SystemRecoveryPlan[] = [
      {
        scenario: 'total_communication_loss',
        severity: 'critical',
        estimatedRecoveryTime: 300, // 5 minutes
        steps: [
          { action: 'activate_autonomous_mode', timeout: 10000 },
          { action: 'establish_backup_channels', timeout: 30000, rollback: 'revert_communication' },
          { action: 'redistribute_robot_assignments', timeout: 60000 },
          { action: 'notify_operators', timeout: 5000 }
        ]
      },
      {
        scenario: 'massive_robot_failure',
        severity: 'high',
        estimatedRecoveryTime: 180, // 3 minutes
        steps: [
          { action: 'isolate_failed_robots', timeout: 15000 },
          { action: 'reallocate_tasks', timeout: 30000, rollback: 'restore_original_allocation' },
          { action: 'deploy_backup_robots', timeout: 120000 },
          { action: 'validate_new_formation', timeout: 15000 }
        ]
      },
      {
        scenario: 'system_overload',
        severity: 'medium',
        estimatedRecoveryTime: 120, // 2 minutes
        steps: [
          { action: 'reduce_processing_load', timeout: 5000 },
          { action: 'pause_non_critical_operations', timeout: 10000, rollback: 'resume_operations' },
          { action: 'scale_resources', timeout: 60000 },
          { action: 'optimize_data_flow', timeout: 45000 }
        ]
      }
    ];

    plans.forEach(plan => this.recoveryPlans.set(plan.scenario, plan));
  }

  private startContinuousMonitoring(): void {
    this.monitoringInterval = setInterval(async () => {
      await this.runHealthChecks();
      await this.evaluateCircuitBreakers();
      await this.checkResiliencePolicies();
      await this.adaptSystemBehavior();
    }, 5000); // Check every 5 seconds
  }

  private async runHealthChecks(): Promise<void> {
    for (const [id, healthCheck] of this.healthChecks.entries()) {
      try {
        const metrics = await this.collectHealthMetrics(healthCheck);
        healthCheck.metrics = metrics;
        healthCheck.lastCheck = new Date();
        healthCheck.status = this.evaluateHealthStatus(healthCheck);

        // Store metrics history
        for (const [metricName, value] of Object.entries(metrics)) {
          const key = `${id}_${metricName}`;
          const history = this.systemMetrics.get(key) || [];
          history.push(value);
          
          // Keep only last 100 data points
          if (history.length > 100) {
            history.shift();
          }
          
          this.systemMetrics.set(key, history);
        }

      } catch (error) {
        healthCheck.status = 'unknown';
        console.error(`Health check failed for ${id}:`, error);
      }
    }
  }

  private async collectHealthMetrics(healthCheck: HealthCheck): Promise<Record<string, number>> {
    switch (healthCheck.type) {
      case 'network':
        return await this.collectNetworkMetrics();
      case 'robot':
        return await this.collectRobotMetrics();
      case 'service':
        return await this.collectServiceMetrics();
      case 'system':
        return await this.collectSystemMetrics();
      default:
        return {};
    }
  }

  private async collectNetworkMetrics(): Promise<Record<string, number>> {
    // Simulate network metrics collection
    return {
      latency: 150 + Math.random() * 200,
      packetLoss: Math.random() * 0.08,
      bandwidth: 5000000 + Math.random() * 5000000
    };
  }

  private async collectRobotMetrics(): Promise<Record<string, number>> {
    // Simulate robot swarm metrics
    return {
      activeRobots: Math.floor(45 + Math.random() * 10),
      responseRate: 0.92 + Math.random() * 0.07,
      errorRate: Math.random() * 0.08
    };
  }

  private async collectServiceMetrics(): Promise<Record<string, number>> {
    // Simulate API service metrics
    return {
      responseTime: 1500 + Math.random() * 3000,
      successRate: 0.96 + Math.random() * 0.03,
      rateLimitUsage: Math.random() * 0.6
    };
  }

  private async collectSystemMetrics(): Promise<Record<string, number>> {
    // Simulate system resource metrics
    return {
      cpuUsage: 0.3 + Math.random() * 0.4,
      memoryUsage: 0.4 + Math.random() * 0.3,
      diskUsage: 0.5 + Math.random() * 0.2
    };
  }

  private evaluateHealthStatus(healthCheck: HealthCheck): HealthCheck['status'] {
    let warningCount = 0;
    let criticalCount = 0;

    for (const [metricName, value] of Object.entries(healthCheck.metrics)) {
      const threshold = healthCheck.threshold[metricName];
      if (threshold !== undefined) {
        const ratio = value / threshold;
        
        if (ratio > 1.2) {
          criticalCount++;
        } else if (ratio > 0.8) {
          warningCount++;
        }
      }
    }

    if (criticalCount > 0) return 'critical';
    if (warningCount > 1) return 'warning';
    return 'healthy';
  }

  private async evaluateCircuitBreakers(): Promise<void> {
    for (const [id, breaker] of this.circuitBreakers.entries()) {
      switch (breaker.state) {
        case 'open':
          if (breaker.lastFailure && Date.now() - breaker.lastFailure.getTime() > breaker.timeout) {
            breaker.state = 'half-open';
            breaker.successCount = 0;
            console.log(`Circuit breaker ${id} moved to half-open state`);
          }
          break;

        case 'half-open':
          // In production, this would check actual service calls
          const testSuccess = Math.random() > 0.3; // 70% success rate
          if (testSuccess) {
            breaker.successCount++;
            if (breaker.successCount >= 3) {
              breaker.state = 'closed';
              breaker.failureCount = 0;
              console.log(`Circuit breaker ${id} closed - service recovered`);
            }
          } else {
            breaker.state = 'open';
            breaker.lastFailure = new Date();
            breaker.failureCount++;
            console.log(`Circuit breaker ${id} reopened due to continued failures`);
          }
          break;
      }
    }
  }

  private async checkResiliencePolicies(): Promise<void> {
    for (const [id, policy] of this.resiliencePolicies.entries()) {
      const shouldTrigger = await this.evaluatePolicyTriggers(policy);
      if (shouldTrigger) {
        await this.executePolicyActions(policy);
      }
    }
  }

  private async evaluatePolicyTriggers(policy: ResiliencePolicy): Promise<boolean> {
    const currentMetrics = await this.getCurrentSystemMetrics();
    
    return policy.triggers.some(trigger => {
      // Simple trigger evaluation - can be enhanced with more sophisticated logic
      if (trigger.includes('latency > 500ms')) {
        return currentMetrics.latency > 500;
      }
      if (trigger.includes('packet_loss > 0.05')) {
        return currentMetrics.packet_loss > 0.05;
      }
      if (trigger.includes('robot_error_rate > 0.1')) {
        return currentMetrics.robot_error_rate > 0.1;
      }
      if (trigger.includes('api_success_rate < 0.95')) {
        return currentMetrics.api_success_rate < 0.95;
      }
      return false;
    });
  }

  private async getCurrentSystemMetrics(): Promise<Record<string, number>> {
    const metrics: Record<string, number> = {};
    
    for (const [key, history] of this.systemMetrics.entries()) {
      if (history.length > 0) {
        metrics[key] = history[history.length - 1];
        // Simplified key mapping for trigger evaluation
        const simplifiedKey = key.split('_').slice(1).join('_');
        metrics[simplifiedKey] = history[history.length - 1];
      }
    }
    
    return metrics;
  }

  private async executePolicyActions(policy: ResiliencePolicy): Promise<void> {
    console.log(`Executing resilience policy: ${policy.name}`);
    
    const sortedActions = policy.actions.sort((a, b) => a.priority - b.priority);
    
    for (const action of sortedActions) {
      try {
        await this.executeResilienceAction(action);
        this.adaptationHistory.push({
          timestamp: new Date(),
          action: `${policy.name}: ${action.type}`,
          result: 'success'
        });
      } catch (error) {
        console.error(`Resilience action failed:`, error);
        this.adaptationHistory.push({
          timestamp: new Date(),
          action: `${policy.name}: ${action.type}`,
          result: 'failed'
        });
      }
    }
  }

  private async executeResilienceAction(action: any): Promise<void> {
    switch (action.type) {
      case 'retry':
        await this.executeRetryAction(action.parameters);
        break;
      case 'fallback':
        await this.executeFallbackAction(action.parameters);
        break;
      case 'circuit-break':
        await this.executeCircuitBreakAction(action.parameters);
        break;
      case 'scale':
        await this.executeScaleAction(action.parameters);
        break;
      case 'isolate':
        await this.executeIsolateAction(action.parameters);
        break;
    }
  }

  private async executeRetryAction(params: any): Promise<void> {
    console.log(`Executing retry action with ${params.attempts} attempts`);
    // In production, this would retry failed operations
  }

  private async executeFallbackAction(params: any): Promise<void> {
    console.log(`Executing fallback action:`, params);
    
    if (params.mode === 'autonomous') {
      // Activate autonomous mode for robots
      await this.activateAutonomousMode();
    }
    
    if (params.localMode) {
      // Switch to local processing mode
      await this.activateLocalMode();
    }
  }

  private async executeCircuitBreakAction(params: any): Promise<void> {
    const breakerId = `${params.service}_breaker`;
    const breaker = this.circuitBreakers.get(breakerId);
    
    if (breaker) {
      breaker.state = 'open';
      breaker.lastFailure = new Date();
      console.log(`Circuit breaker activated for ${params.service}`);
    }
  }

  private async executeScaleAction(params: any): Promise<void> {
    console.log(`Executing scale action - replacing ${params.replaceCount} units`);
    // In production, this would scale resources or deploy backup robots
  }

  private async executeIsolateAction(params: any): Promise<void> {
    console.log(`Executing isolate action for failed components`);
    // In production, this would isolate failed robots or services
  }

  private async activateAutonomousMode(): Promise<void> {
    console.log('Activating autonomous mode for increased resilience');
    // Integration with autonomous planning engine
  }

  private async activateLocalMode(): Promise<void> {
    console.log('Switching to local processing mode');
    // Reduce dependency on external services
  }

  private async adaptSystemBehavior(): Promise<void> {
    // Analyze recent adaptation history to improve future responses
    const recentAdaptations = this.adaptationHistory
      .filter(a => Date.now() - a.timestamp.getTime() < 300000) // Last 5 minutes
      .filter(a => a.result === 'success');

    if (recentAdaptations.length > 5) {
      console.log('High adaptation frequency detected - analyzing patterns');
      await this.optimizeResiliencePolicies();
    }
  }

  private async optimizeResiliencePolicies(): Promise<void> {
    // Analyze which policies are most effective
    const actionEffectiveness = this.calculateActionEffectiveness();
    
    for (const [policyId, policy] of this.resiliencePolicies.entries()) {
      // Adjust policy parameters based on effectiveness
      for (const action of policy.actions) {
        const effectiveness = actionEffectiveness.get(action.type) || 0;
        if (effectiveness > 0.8) {
          action.priority = Math.max(1, action.priority - 1); // Increase priority
        } else if (effectiveness < 0.4) {
          action.priority = Math.min(10, action.priority + 1); // Decrease priority
        }
      }
    }

    console.log('Resilience policies optimized based on historical effectiveness');
  }

  private calculateActionEffectiveness(): Map<string, number> {
    const effectiveness = new Map<string, number>();
    const actionCounts = new Map<string, { success: number; total: number }>();

    for (const adaptation of this.adaptationHistory) {
      const actionType = adaptation.action.split(':')[1]?.trim();
      if (actionType) {
        const current = actionCounts.get(actionType) || { success: 0, total: 0 };
        current.total++;
        if (adaptation.result === 'success') {
          current.success++;
        }
        actionCounts.set(actionType, current);
      }
    }

    for (const [actionType, counts] of actionCounts.entries()) {
      effectiveness.set(actionType, counts.success / counts.total);
    }

    return effectiveness;
  }

  async executeRecoveryPlan(scenario: string): Promise<void> {
    const plan = this.recoveryPlans.get(scenario);
    if (!plan) {
      throw new Error(`Recovery plan not found for scenario: ${scenario}`);
    }

    console.log(`Executing recovery plan for ${scenario} (severity: ${plan.severity})`);
    
    for (let i = 0; i < plan.steps.length; i++) {
      const step = plan.steps[i];
      try {
        console.log(`Step ${i + 1}: ${step.action}`);
        await this.executeRecoveryStep(step);
        console.log(`Step ${i + 1} completed successfully`);
      } catch (error) {
        console.error(`Step ${i + 1} failed:`, error);
        
        if (step.rollback) {
          console.log(`Executing rollback: ${step.rollback}`);
          await this.executeRecoveryStep({ action: step.rollback, timeout: step.timeout });
        }
        
        throw new Error(`Recovery plan failed at step ${i + 1}`);
      }
    }

    console.log(`Recovery plan for ${scenario} completed successfully`);
  }

  private async executeRecoveryStep(step: any): Promise<void> {
    return new Promise((resolve, reject) => {
      const timeout = setTimeout(() => {
        reject(new Error(`Recovery step timed out: ${step.action}`));
      }, step.timeout);

      // Simulate recovery step execution
      const success = Math.random() > 0.1; // 90% success rate
      
      setTimeout(() => {
        clearTimeout(timeout);
        if (success) {
          resolve();
        } else {
          reject(new Error(`Recovery step failed: ${step.action}`));
        }
      }, Math.random() * step.timeout * 0.5);
    });
  }

  // Public interface methods
  getSystemHealth(): Record<string, any> {
    const health: Record<string, any> = {};
    
    for (const [id, healthCheck] of this.healthChecks.entries()) {
      health[id] = {
        status: healthCheck.status,
        lastCheck: healthCheck.lastCheck,
        metrics: healthCheck.metrics
      };
    }

    return health;
  }

  getCircuitBreakerStatus(): Record<string, any> {
    const status: Record<string, any> = {};
    
    for (const [id, breaker] of this.circuitBreakers.entries()) {
      status[id] = {
        state: breaker.state,
        service: breaker.service,
        failureCount: breaker.failureCount,
        lastFailure: breaker.lastFailure
      };
    }

    return status;
  }

  getAdaptationHistory(): Array<{ timestamp: Date; action: string; result: string }> {
    return [...this.adaptationHistory].slice(-50); // Last 50 adaptations
  }

  async generateResilienceReport(): Promise<string> {
    const report = {
      timestamp: new Date().toISOString(),
      systemHealth: this.getSystemHealth(),
      circuitBreakers: this.getCircuitBreakerStatus(),
      adaptationHistory: this.getAdaptationHistory(),
      resilienceMetrics: {
        totalAdaptations: this.adaptationHistory.length,
        successRate: this.adaptationHistory.filter(a => a.result === 'success').length / Math.max(1, this.adaptationHistory.length),
        avgRecoveryTime: this.calculateAverageRecoveryTime(),
        mostEffectiveActions: Array.from(this.calculateActionEffectiveness.entries()).slice(0, 5)
      },
      recommendations: this.generateResilienceRecommendations()
    };

    return JSON.stringify(report, null, 2);
  }

  private calculateAverageRecoveryTime(): number {
    // Simplified calculation - in production would track actual recovery times
    return Math.random() * 60 + 30; // 30-90 seconds average
  }

  private generateResilienceRecommendations(): string[] {
    const recommendations = [];
    
    const healthIssues = Array.from(this.healthChecks.values())
      .filter(hc => hc.status === 'critical' || hc.status === 'warning');
    
    if (healthIssues.length > 2) {
      recommendations.push('Multiple health issues detected - consider system maintenance');
    }

    const openBreakers = Array.from(this.circuitBreakers.values())
      .filter(cb => cb.state === 'open');
    
    if (openBreakers.length > 0) {
      recommendations.push('Circuit breakers active - investigate service degradations');
    }

    const recentAdaptations = this.adaptationHistory
      .filter(a => Date.now() - a.timestamp.getTime() < 600000); // Last 10 minutes
    
    if (recentAdaptations.length > 10) {
      recommendations.push('High adaptation frequency - consider adjusting baseline thresholds');
    }

    return recommendations;
  }

  destroy(): void {
    if (this.monitoringInterval) {
      clearInterval(this.monitoringInterval);
    }
  }
}

export const adaptiveResilienceEngine = new AdaptiveResilienceEngine();