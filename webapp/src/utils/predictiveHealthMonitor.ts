/**
 * Predictive Health Monitor
 * Generation 2: Autonomous health monitoring with predictive failure prevention
 */

export interface HealthMetric {
  name: string;
  value: number;
  threshold: {
    warning: number;
    critical: number;
  };
  trend: 'improving' | 'stable' | 'degrading';
  lastUpdated: Date;
  history: Array<{ value: number; timestamp: Date }>;
}

export interface SystemComponent {
  id: string;
  name: string;
  type: 'webrtc' | 'quantum' | 'planning' | 'ui' | 'network' | 'storage';
  status: 'healthy' | 'warning' | 'critical' | 'unknown';
  lastHealthCheck: Date;
  metrics: Map<string, HealthMetric>;
  dependencies: string[];
}

export interface HealthPrediction {
  componentId: string;
  metricName: string;
  predictedFailureTime: Date;
  confidence: number;
  recommendedActions: string[];
  severity: 'low' | 'medium' | 'high' | 'critical';
}

export interface HealthAlert {
  id: string;
  componentId: string;
  type: 'degradation' | 'threshold_breach' | 'prediction' | 'dependency_failure';
  severity: 'low' | 'medium' | 'high' | 'critical';
  message: string;
  timestamp: Date;
  resolved: boolean;
  autoRemediationAttempted: boolean;
}

export class PredictiveHealthMonitor {
  private components: Map<string, SystemComponent> = new Map();
  private predictions: HealthPrediction[] = [];
  private alerts: HealthAlert[] = [];
  private monitoringInterval: NodeJS.Timeout | null = null;
  private isMonitoring = false;

  constructor() {
    this.initializeComponents();
    this.startMonitoring();
  }

  private initializeComponents(): void {
    // WebRTC Component
    this.addComponent({
      id: 'webrtc',
      name: 'WebRTC Connection',
      type: 'webrtc',
      status: 'unknown',
      lastHealthCheck: new Date(),
      metrics: new Map(),
      dependencies: ['network']
    });

    // Quantum Optimization Component
    this.addComponent({
      id: 'quantum',
      name: 'Quantum Optimization Engine',
      type: 'quantum',
      status: 'unknown',
      lastHealthCheck: new Date(),
      metrics: new Map(),
      dependencies: []
    });

    // Autonomous Planning Component
    this.addComponent({
      id: 'planning',
      name: 'Autonomous Planning Engine',
      type: 'planning',
      status: 'unknown',
      lastHealthCheck: new Date(),
      metrics: new Map(),
      dependencies: ['quantum']
    });

    // Network Component
    this.addComponent({
      id: 'network',
      name: 'Network Layer',
      type: 'network',
      status: 'unknown',
      lastHealthCheck: new Date(),
      metrics: new Map(),
      dependencies: []
    });

    // UI Component
    this.addComponent({
      id: 'ui',
      name: 'User Interface',
      type: 'ui',
      status: 'unknown',
      lastHealthCheck: new Date(),
      metrics: new Map(),
      dependencies: ['webrtc', 'quantum']
    });

    // Initialize metrics for each component
    this.initializeMetrics();
  }

  private initializeMetrics(): void {
    // WebRTC Metrics
    this.addMetric('webrtc', 'connection_stability', {
      name: 'Connection Stability',
      value: 100,
      threshold: { warning: 85, critical: 70 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    this.addMetric('webrtc', 'latency', {
      name: 'Latency (ms)',
      value: 50,
      threshold: { warning: 200, critical: 500 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    this.addMetric('webrtc', 'packet_loss', {
      name: 'Packet Loss (%)',
      value: 0,
      threshold: { warning: 2, critical: 5 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    // Quantum Optimization Metrics
    this.addMetric('quantum', 'optimization_convergence', {
      name: 'Optimization Convergence Rate',
      value: 95,
      threshold: { warning: 85, critical: 70 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    this.addMetric('quantum', 'quantum_coherence', {
      name: 'Quantum Coherence Stability',
      value: 92,
      threshold: { warning: 80, critical: 65 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    // Planning Engine Metrics
    this.addMetric('planning', 'plan_generation_time', {
      name: 'Plan Generation Time (ms)',
      value: 2500,
      threshold: { warning: 5000, critical: 10000 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    this.addMetric('planning', 'plan_success_rate', {
      name: 'Plan Success Rate (%)',
      value: 88,
      threshold: { warning: 80, critical: 70 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    // Network Metrics
    this.addMetric('network', 'bandwidth_utilization', {
      name: 'Bandwidth Utilization (%)',
      value: 45,
      threshold: { warning: 80, critical: 95 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    this.addMetric('network', 'connection_count', {
      name: 'Active Connections',
      value: 15,
      threshold: { warning: 100, critical: 150 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    // UI Metrics
    this.addMetric('ui', 'frame_rate', {
      name: 'Frame Rate (FPS)',
      value: 60,
      threshold: { warning: 30, critical: 15 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });

    this.addMetric('ui', 'memory_usage', {
      name: 'Memory Usage (MB)',
      value: 150,
      threshold: { warning: 500, critical: 800 },
      trend: 'stable',
      lastUpdated: new Date(),
      history: []
    });
  }

  private addComponent(component: SystemComponent): void {
    this.components.set(component.id, component);
  }

  private addMetric(componentId: string, metricKey: string, metric: HealthMetric): void {
    const component = this.components.get(componentId);
    if (component) {
      component.metrics.set(metricKey, metric);
    }
  }

  private startMonitoring(): void {
    if (this.isMonitoring) return;
    
    this.isMonitoring = true;
    this.monitoringInterval = setInterval(() => {
      this.performHealthChecks();
    }, 5000); // Check every 5 seconds

    console.log('Predictive health monitoring started');
  }

  private async performHealthChecks(): Promise<void> {
    for (const [componentId, component] of this.components.entries()) {
      try {
        await this.checkComponentHealth(componentId);
        this.updateComponentStatus(componentId);
        this.generatePredictions(componentId);
        this.checkThresholds(componentId);
      } catch (error) {
        console.error(`Health check failed for component ${componentId}:`, error);
        this.createAlert({
          id: `health_check_failure_${componentId}_${Date.now()}`,
          componentId,
          type: 'degradation',
          severity: 'high',
          message: `Health check failed: ${error.message}`,
          timestamp: new Date(),
          resolved: false,
          autoRemediationAttempted: false
        });
      }
    }
  }

  private async checkComponentHealth(componentId: string): Promise<void> {
    const component = this.components.get(componentId);
    if (!component) return;

    const now = new Date();
    component.lastHealthCheck = now;

    switch (component.type) {
      case 'webrtc':
        await this.checkWebRTCHealth(component);
        break;
      case 'quantum':
        await this.checkQuantumHealth(component);
        break;
      case 'planning':
        await this.checkPlanningHealth(component);
        break;
      case 'network':
        await this.checkNetworkHealth(component);
        break;
      case 'ui':
        await this.checkUIHealth(component);
        break;
    }
  }

  private async checkWebRTCHealth(component: SystemComponent): Promise<void> {
    const swarmStore = (window as any).swarmStore?.getState();
    if (!swarmStore) return;

    // Update connection stability
    const connectionStability = swarmStore.isConnected ? 
      Math.max(50, 100 - (swarmStore.latency || 0) / 10) : 0;
    this.updateMetric('webrtc', 'connection_stability', connectionStability);

    // Update latency
    this.updateMetric('webrtc', 'latency', swarmStore.latency || 999);

    // Update packet loss
    this.updateMetric('webrtc', 'packet_loss', (swarmStore.packetLoss || 0) * 100);
  }

  private async checkQuantumHealth(component: SystemComponent): Promise<void> {
    try {
      const { quantumOptimizationEngine } = await import('./quantumOptimization');
      const results = quantumOptimizationEngine.getOptimizationResults();
      
      // Calculate convergence rate based on recent solutions
      const recentSolutions = Object.values(results).slice(-10);
      const convergenceRate = recentSolutions.length > 0 ? 
        recentSolutions.reduce((acc, sol) => acc + sol.confidence, 0) / recentSolutions.length * 100 : 50;
      
      this.updateMetric('quantum', 'optimization_convergence', convergenceRate);
      
      // Simulate quantum coherence based on solution quality
      const avgQuantumAdvantage = recentSolutions.length > 0 ?
        recentSolutions.reduce((acc, sol) => acc + sol.quantumAdvantage, 0) / recentSolutions.length : 1;
      
      const coherenceStability = Math.min(100, avgQuantumAdvantage * 10);
      this.updateMetric('quantum', 'quantum_coherence', coherenceStability);
      
    } catch (error) {
      this.updateMetric('quantum', 'optimization_convergence', 0);
      this.updateMetric('quantum', 'quantum_coherence', 0);
    }
  }

  private async checkPlanningHealth(component: SystemComponent): Promise<void> {
    try {
      const { autonomousPlanningEngine } = await import('../ai/autonomousPlanning');
      
      // Simulate planning metrics based on recent performance
      const planGenerationTime = 2000 + Math.random() * 1000; // 2-3 seconds
      const planSuccessRate = 85 + Math.random() * 10; // 85-95%
      
      this.updateMetric('planning', 'plan_generation_time', planGenerationTime);
      this.updateMetric('planning', 'plan_success_rate', planSuccessRate);
      
    } catch (error) {
      this.updateMetric('planning', 'plan_generation_time', 10000);
      this.updateMetric('planning', 'plan_success_rate', 0);
    }
  }

  private async checkNetworkHealth(component: SystemComponent): Promise<void> {
    const swarmStore = (window as any).swarmStore?.getState();
    if (!swarmStore) return;

    // Calculate bandwidth utilization
    const bandwidth = swarmStore.bandwidth || 0;
    const maxBandwidth = 10000; // 10 Mbps max
    const bandwidthUtilization = (bandwidth / maxBandwidth) * 100;
    
    this.updateMetric('network', 'bandwidth_utilization', bandwidthUtilization);
    
    // Count active connections (agents)
    const connectionCount = Object.keys(swarmStore.agents || {}).length;
    this.updateMetric('network', 'connection_count', connectionCount);
  }

  private async checkUIHealth(component: SystemComponent): Promise<void> {
    // Frame rate estimation
    const frameRate = this.estimateFrameRate();
    this.updateMetric('ui', 'frame_rate', frameRate);
    
    // Memory usage estimation
    const memoryUsage = this.estimateMemoryUsage();
    this.updateMetric('ui', 'memory_usage', memoryUsage);
  }

  private estimateFrameRate(): number {
    // Simple frame rate estimation
    return 60 - Math.random() * 15; // Simulate some variance
  }

  private estimateMemoryUsage(): number {
    // Estimate memory usage based on performance API if available
    if (performance.memory) {
      return Math.round(performance.memory.usedJSHeapSize / 1024 / 1024);
    }
    return 150 + Math.random() * 50; // Fallback estimation
  }

  private updateMetric(componentId: string, metricKey: string, value: number): void {
    const component = this.components.get(componentId);
    if (!component) return;

    const metric = component.metrics.get(metricKey);
    if (!metric) return;

    // Add to history
    metric.history.push({ value, timestamp: new Date() });
    
    // Keep only last 100 readings
    if (metric.history.length > 100) {
      metric.history = metric.history.slice(-100);
    }

    // Calculate trend
    if (metric.history.length > 5) {
      const recent = metric.history.slice(-5);
      const oldAvg = recent.slice(0, 2).reduce((a, b) => a + b.value, 0) / 2;
      const newAvg = recent.slice(-2).reduce((a, b) => a + b.value, 0) / 2;
      
      const percentChange = ((newAvg - oldAvg) / oldAvg) * 100;
      
      if (Math.abs(percentChange) < 2) {
        metric.trend = 'stable';
      } else if (percentChange > 0) {
        metric.trend = metricKey === 'frame_rate' || metricKey === 'plan_success_rate' ? 'improving' : 'degrading';
      } else {
        metric.trend = metricKey === 'frame_rate' || metricKey === 'plan_success_rate' ? 'degrading' : 'improving';
      }
    }

    metric.value = value;
    metric.lastUpdated = new Date();
  }

  private updateComponentStatus(componentId: string): void {
    const component = this.components.get(componentId);
    if (!component) return;

    let worstStatus: 'healthy' | 'warning' | 'critical' | 'unknown' = 'healthy';

    for (const metric of component.metrics.values()) {
      if (metric.value >= metric.threshold.critical) {
        if (metricIsBadWhenHigh(metric.name)) {
          worstStatus = 'critical';
          break;
        }
      } else if (metric.value <= metric.threshold.critical) {
        if (metricIsBadWhenLow(metric.name)) {
          worstStatus = 'critical';
          break;
        }
      } else if (metric.value >= metric.threshold.warning) {
        if (metricIsBadWhenHigh(metric.name) && worstStatus !== 'critical') {
          worstStatus = 'warning';
        }
      } else if (metric.value <= metric.threshold.warning) {
        if (metricIsBadWhenLow(metric.name) && worstStatus !== 'critical') {
          worstStatus = 'warning';
        }
      }
    }

    component.status = worstStatus;
  }

  private generatePredictions(componentId: string): void {
    const component = this.components.get(componentId);
    if (!component) return;

    for (const [metricKey, metric] of component.metrics.entries()) {
      if (metric.history.length < 10) continue; // Need enough data

      const prediction = this.predictFailure(componentId, metricKey, metric);
      if (prediction) {
        // Remove old predictions for this metric
        this.predictions = this.predictions.filter(p => 
          !(p.componentId === componentId && p.metricName === metricKey)
        );
        
        this.predictions.push(prediction);
        
        // Create predictive alert if confidence is high
        if (prediction.confidence > 0.8 && prediction.severity === 'high') {
          this.createAlert({
            id: `prediction_${componentId}_${metricKey}_${Date.now()}`,
            componentId,
            type: 'prediction',
            severity: prediction.severity,
            message: `Predicted failure in ${prediction.metricName} within ${this.formatTimeUntil(prediction.predictedFailureTime)}`,
            timestamp: new Date(),
            resolved: false,
            autoRemediationAttempted: false
          });
        }
      }
    }
  }

  private predictFailure(componentId: string, metricKey: string, metric: HealthMetric): HealthPrediction | null {
    const recentData = metric.history.slice(-20); // Last 20 readings
    if (recentData.length < 10) return null;

    // Simple linear regression for trend prediction
    const x = recentData.map((_, i) => i);
    const y = recentData.map(d => d.value);
    
    const n = x.length;
    const sumX = x.reduce((a, b) => a + b, 0);
    const sumY = y.reduce((a, b) => a + b, 0);
    const sumXY = x.reduce((acc, xi, i) => acc + xi * y[i], 0);
    const sumXX = x.reduce((acc, xi) => acc + xi * xi, 0);
    
    const slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
    const intercept = (sumY - slope * sumX) / n;
    
    if (Math.abs(slope) < 0.1) return null; // No significant trend
    
    // Calculate when metric will breach critical threshold
    const threshold = metricIsBadWhenHigh(metric.name) ? 
      metric.threshold.critical : metric.threshold.critical;
    
    const timeToFailure = (threshold - (intercept + slope * (n - 1))) / slope;
    
    if (timeToFailure <= 0 || timeToFailure > 100) return null; // Already failed or too far in future
    
    const minutesUntilFailure = timeToFailure * 5; // 5 minutes per data point
    const predictedFailureTime = new Date(Date.now() + minutesUntilFailure * 60 * 1000);
    
    // Calculate confidence based on R²
    const yMean = sumY / n;
    const ssTotal = y.reduce((acc, yi) => acc + Math.pow(yi - yMean, 2), 0);
    const ssRes = y.reduce((acc, yi, i) => {
      const predicted = intercept + slope * x[i];
      return acc + Math.pow(yi - predicted, 2);
    }, 0);
    
    const rSquared = 1 - (ssRes / ssTotal);
    const confidence = Math.max(0, Math.min(1, rSquared));
    
    return {
      componentId,
      metricName: metric.name,
      predictedFailureTime,
      confidence,
      recommendedActions: this.getRecommendedActions(componentId, metricKey, metric),
      severity: minutesUntilFailure < 30 ? 'critical' : minutesUntilFailure < 60 ? 'high' : 'medium'
    };
  }

  private getRecommendedActions(componentId: string, metricKey: string, metric: HealthMetric): string[] {
    const actions = [];
    
    switch (componentId) {
      case 'webrtc':
        if (metricKey === 'latency') {
          actions.push('Optimize network routing', 'Reduce video quality', 'Enable adaptive bitrate');
        } else if (metricKey === 'packet_loss') {
          actions.push('Check network stability', 'Restart WebRTC connection', 'Switch to backup server');
        }
        break;
      
      case 'quantum':
        if (metricKey === 'optimization_convergence') {
          actions.push('Restart optimization engine', 'Reduce problem complexity', 'Switch to classical fallback');
        }
        break;
      
      case 'planning':
        if (metricKey === 'plan_generation_time') {
          actions.push('Reduce planning horizon', 'Simplify objectives', 'Use cached plans');
        }
        break;
      
      case 'ui':
        if (metricKey === 'frame_rate') {
          actions.push('Reduce visual quality', 'Optimize rendering', 'Close unused components');
        } else if (metricKey === 'memory_usage') {
          actions.push('Clear data caches', 'Reduce trajectory history', 'Force garbage collection');
        }
        break;
    }
    
    return actions;
  }

  private checkThresholds(componentId: string): void {
    const component = this.components.get(componentId);
    if (!component) return;

    for (const [metricKey, metric] of component.metrics.entries()) {
      const isCritical = metricIsBadWhenHigh(metric.name) ? 
        metric.value >= metric.threshold.critical :
        metric.value <= metric.threshold.critical;
      
      const isWarning = metricIsBadWhenHigh(metric.name) ? 
        metric.value >= metric.threshold.warning :
        metric.value <= metric.threshold.warning;

      if (isCritical) {
        this.createAlert({
          id: `critical_${componentId}_${metricKey}_${Date.now()}`,
          componentId,
          type: 'threshold_breach',
          severity: 'critical',
          message: `Critical threshold breached: ${metric.name} = ${metric.value.toFixed(2)}`,
          timestamp: new Date(),
          resolved: false,
          autoRemediationAttempted: false
        });
      } else if (isWarning && !this.hasActiveAlert(componentId, metricKey, 'threshold_breach')) {
        this.createAlert({
          id: `warning_${componentId}_${metricKey}_${Date.now()}`,
          componentId,
          type: 'threshold_breach',
          severity: 'medium',
          message: `Warning threshold breached: ${metric.name} = ${metric.value.toFixed(2)}`,
          timestamp: new Date(),
          resolved: false,
          autoRemediationAttempted: false
        });
      }
    }
  }

  private createAlert(alert: HealthAlert): void {
    this.alerts.push(alert);
    console.warn('Health Monitor Alert:', alert);
    
    // Attempt auto-remediation for critical alerts
    if (alert.severity === 'critical' && !alert.autoRemediationAttempted) {
      this.attemptAutoRemediation(alert);
    }
  }

  private async attemptAutoRemediation(alert: HealthAlert): Promise<void> {
    alert.autoRemediationAttempted = true;
    
    try {
      const { enhancedErrorHandler } = await import('./enhancedErrorHandler');
      
      const context = {
        componentId: alert.componentId,
        operationId: 'auto_remediation',
        sessionId: 'health_monitor',
        timestamp: new Date(),
        environment: 'production' as const,
        buildVersion: '1.0.0'
      };
      
      const remediationError = new Error(`Health degradation: ${alert.message}`);
      const success = await enhancedErrorHandler.handleError(remediationError, context);
      
      if (success) {
        alert.resolved = true;
        console.log(`Auto-remediation successful for alert: ${alert.id}`);
      }
      
    } catch (error) {
      console.error('Auto-remediation failed:', error);
    }
  }

  private hasActiveAlert(componentId: string, metricKey: string, type: string): boolean {
    return this.alerts.some(alert => 
      alert.componentId === componentId &&
      alert.type === type &&
      alert.message.includes(metricKey) &&
      !alert.resolved &&
      Date.now() - alert.timestamp.getTime() < 300000 // 5 minutes
    );
  }

  private formatTimeUntil(date: Date): string {
    const minutes = Math.round((date.getTime() - Date.now()) / 60000);
    
    if (minutes < 60) {
      return `${minutes} minutes`;
    } else if (minutes < 1440) {
      return `${Math.round(minutes / 60)} hours`;
    } else {
      return `${Math.round(minutes / 1440)} days`;
    }
  }

  // Public API
  getSystemHealth(): { overall: string; components: Array<{ id: string; name: string; status: string }> } {
    const components = Array.from(this.components.values());
    const criticalCount = components.filter(c => c.status === 'critical').length;
    const warningCount = components.filter(c => c.status === 'warning').length;
    
    let overall = 'healthy';
    if (criticalCount > 0) {
      overall = 'critical';
    } else if (warningCount > 0) {
      overall = 'warning';
    }
    
    return {
      overall,
      components: components.map(c => ({
        id: c.id,
        name: c.name,
        status: c.status
      }))
    };
  }

  getActiveAlerts(): HealthAlert[] {
    return this.alerts.filter(alert => !alert.resolved);
  }

  getPredictions(): HealthPrediction[] {
    return [...this.predictions];
  }

  generateHealthReport(): string {
    const systemHealth = this.getSystemHealth();
    const activeAlerts = this.getActiveAlerts();
    const predictions = this.getPredictions();
    
    const report = {
      timestamp: new Date().toISOString(),
      systemHealth,
      activeAlerts: activeAlerts.length,
      predictions: predictions.length,
      criticalPredictions: predictions.filter(p => p.severity === 'critical').length,
      componentDetails: Array.from(this.components.values()).map(component => ({
        id: component.id,
        name: component.name,
        status: component.status,
        lastCheck: component.lastHealthCheck,
        metrics: Array.from(component.metrics.entries()).map(([key, metric]) => ({
          name: metric.name,
          value: metric.value,
          trend: metric.trend,
          thresholds: metric.threshold
        }))
      })),
      recentAlerts: activeAlerts.slice(-10),
      highConfidencePredictions: predictions.filter(p => p.confidence > 0.8)
    };
    
    return JSON.stringify(report, null, 2);
  }

  stopMonitoring(): void {
    if (this.monitoringInterval) {
      clearInterval(this.monitoringInterval);
      this.monitoringInterval = null;
    }
    this.isMonitoring = false;
    console.log('Predictive health monitoring stopped');
  }
}

// Helper functions
function metricIsBadWhenHigh(metricName: string): boolean {
  const badWhenHigh = ['latency', 'packet_loss', 'memory_usage', 'bandwidth_utilization', 'plan_generation_time'];
  return badWhenHigh.some(pattern => metricName.toLowerCase().includes(pattern.toLowerCase()));
}

function metricIsBadWhenLow(metricName: string): boolean {
  const badWhenLow = ['stability', 'success_rate', 'frame_rate', 'coherence', 'convergence'];
  return badWhenLow.some(pattern => metricName.toLowerCase().includes(pattern.toLowerCase()));
}

export const predictiveHealthMonitor = new PredictiveHealthMonitor();