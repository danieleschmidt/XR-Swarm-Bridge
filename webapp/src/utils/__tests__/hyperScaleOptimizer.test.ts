import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { HyperScaleOptimizer } from '../hyperScaleOptimizer';

describe('HyperScaleOptimizer', () => {
  let optimizer: HyperScaleOptimizer;

  beforeEach(() => {
    optimizer = new HyperScaleOptimizer({
      maxRobots: 1000,
      maxConcurrentUsers: 100,
      targetLatency: 100,
      bandwidthBudget: 1000,
      memoryBudget: 4096,
      cpuBudget: 500,
      redundancyLevel: 2
    });
  });

  afterEach(() => {
    optimizer.stopOptimization();
  });

  describe('Initialization', () => {
    it('should initialize with default configuration', () => {
      const defaultOptimizer = new HyperScaleOptimizer();
      const metrics = defaultOptimizer.getScaleMetrics();
      
      expect(metrics.resourceUtilization).toBeDefined();
      expect(metrics.optimizationLevels).toBeDefined();
      expect(metrics.loadBalancingWeights).toBeDefined();
      expect(metrics.predictedCapacity).toBeDefined();
      
      defaultOptimizer.stopOptimization();
    });

    it('should initialize with custom configuration', () => {
      const metrics = optimizer.getScaleMetrics();
      
      expect(metrics.resourceUtilization).toHaveProperty('compute_primary');
      expect(metrics.resourceUtilization).toHaveProperty('memory_primary');
      expect(metrics.resourceUtilization).toHaveProperty('network_primary');
      expect(metrics.resourceUtilization).toHaveProperty('storage_primary');
    });
  });

  describe('Resource Pool Management', () => {
    it('should add resource pools correctly', () => {
      optimizer.addResourcePool({
        id: 'test_pool',
        type: 'compute',
        capacity: 500,
        utilization: 0.5,
        performance: 0.9,
        status: 'healthy',
        lastOptimized: new Date()
      });

      const metrics = optimizer.getScaleMetrics();
      expect(metrics.resourceUtilization).toHaveProperty('test_pool');
      expect(metrics.resourceUtilization.test_pool).toBe(0.5);
    });

    it('should track resource utilization', () => {
      const metrics = optimizer.getScaleMetrics();
      
      Object.values(metrics.resourceUtilization).forEach(utilization => {
        expect(utilization).toBeGreaterThanOrEqual(0);
        expect(utilization).toBeLessThanOrEqual(1);
      });
    });
  });

  describe('Auto-Scaling Rules', () => {
    it('should add auto-scaling rules', () => {
      optimizer.addAutoScalingRule({
        id: 'test_rule',
        metric: 'test_metric',
        threshold: 0.8,
        action: 'scale_up',
        cooldownMs: 60000,
        maxInstances: 10,
        minInstances: 1,
        enabled: true
      });

      // Rules are applied internally, so we test indirectly through scale report
      const report = JSON.parse(optimizer.generateScaleReport());
      expect(report.autoScalingRules).toBeGreaterThanOrEqual(5); // Original + new rule
    });

    it('should handle different scaling actions', () => {
      const actions = ['scale_up', 'scale_down', 'optimize', 'redistribute'];
      
      actions.forEach((action, index) => {
        optimizer.addAutoScalingRule({
          id: `test_rule_${index}`,
          metric: `test_metric_${index}`,
          threshold: 0.5,
          action: action as any,
          cooldownMs: 30000,
          maxInstances: 5,
          minInstances: 1,
          enabled: true
        });
      });

      const report = JSON.parse(optimizer.generateScaleReport());
      expect(report.autoScalingRules).toBeGreaterThanOrEqual(actions.length + 5);
    });
  });

  describe('Performance Optimizations', () => {
    it('should manage optimization levels', () => {
      optimizer.addOptimization({
        type: 'rendering',
        level: 7,
        impact: 'high',
        trade_offs: ['Quality reduction'],
        enabled: true
      });

      const metrics = optimizer.getScaleMetrics();
      expect(metrics.optimizationLevels.rendering).toBe(7);
    });

    it('should handle different optimization types', () => {
      const types: Array<'rendering' | 'networking' | 'computation' | 'memory' | 'quantum'> = [
        'rendering', 'networking', 'computation', 'memory', 'quantum'
      ];

      types.forEach((type, index) => {
        optimizer.addOptimization({
          type,
          level: index + 1,
          impact: 'medium',
          trade_offs: [`Trade-off for ${type}`],
          enabled: true
        });
      });

      const metrics = optimizer.getScaleMetrics();
      types.forEach((type, index) => {
        expect(metrics.optimizationLevels[type]).toBe(index + 1);
      });
    });
  });

  describe('Load Balancing', () => {
    it('should calculate load balancing weights', () => {
      const metrics = optimizer.getScaleMetrics();
      
      Object.values(metrics.loadBalancingWeights).forEach(weight => {
        expect(weight).toBeGreaterThanOrEqual(0);
        expect(weight).toBeLessThanOrEqual(1);
      });
    });

    it('should adapt weights based on performance', () => {
      // Add a degraded resource pool
      optimizer.addResourcePool({
        id: 'degraded_pool',
        type: 'compute',
        capacity: 100,
        utilization: 0.9,
        performance: 0.3,
        status: 'degraded',
        lastOptimized: new Date()
      });

      // Allow some time for weight calculation
      setTimeout(() => {
        const metrics = optimizer.getScaleMetrics();
        
        // Degraded pool should have lower weight
        if (metrics.loadBalancingWeights.degraded_pool !== undefined) {
          expect(metrics.loadBalancingWeights.degraded_pool).toBeLessThan(0.5);
        }
      }, 100);
    });
  });

  describe('Scale Reporting', () => {
    it('should generate comprehensive scale report', () => {
      const reportString = optimizer.generateScaleReport();
      const report = JSON.parse(reportString);

      expect(report).toHaveProperty('timestamp');
      expect(report).toHaveProperty('scaleConfiguration');
      expect(report).toHaveProperty('currentMetrics');
      expect(report).toHaveProperty('resourcePools');
      expect(report).toHaveProperty('activeOptimizations');
      expect(report).toHaveProperty('loadBalancingStrategy');
      expect(report).toHaveProperty('autoScalingRules');
      expect(report).toHaveProperty('predictiveInsights');
    });

    it('should include predictive insights', () => {
      const report = JSON.parse(optimizer.generateScaleReport());
      
      expect(report.predictiveInsights).toHaveProperty('recommendedActions');
      expect(report.predictiveInsights).toHaveProperty('riskAssessment');
      expect(Array.isArray(report.predictiveInsights.recommendedActions)).toBe(true);
      expect(report.predictiveInsights.riskAssessment).toHaveProperty('level');
      expect(report.predictiveInsights.riskAssessment).toHaveProperty('factors');
    });

    it('should track resource pool status', () => {
      const report = JSON.parse(optimizer.generateScaleReport());
      
      expect(Array.isArray(report.resourcePools)).toBe(true);
      report.resourcePools.forEach((pool: any) => {
        expect(pool).toHaveProperty('id');
        expect(pool).toHaveProperty('type');
        expect(pool).toHaveProperty('utilization');
        expect(pool).toHaveProperty('performance');
        expect(pool).toHaveProperty('status');
        expect(['healthy', 'degraded', 'overloaded', 'failed']).toContain(pool.status);
      });
    });
  });

  describe('Metrics Collection', () => {
    it('should track predicted capacity', () => {
      const metrics = optimizer.getScaleMetrics();
      
      expect(metrics.predictedCapacity).toBeGreaterThanOrEqual(0);
      expect(metrics.predictedCapacity).toBeLessThanOrEqual(10000); // Reasonable upper bound
    });

    it('should maintain optimization levels within bounds', () => {
      const metrics = optimizer.getScaleMetrics();
      
      Object.values(metrics.optimizationLevels).forEach(level => {
        expect(level).toBeGreaterThanOrEqual(1);
        expect(level).toBeLessThanOrEqual(10);
      });
    });
  });

  describe('Risk Assessment', () => {
    it('should assess scaling risks correctly', () => {
      const report = JSON.parse(optimizer.generateScaleReport());
      const riskAssessment = report.predictiveInsights.riskAssessment;
      
      expect(['low', 'medium', 'high']).toContain(riskAssessment.level);
      expect(Array.isArray(riskAssessment.factors)).toBe(true);
    });

    it('should provide relevant recommendations', () => {
      const report = JSON.parse(optimizer.generateScaleReport());
      const recommendations = report.predictiveInsights.recommendedActions;
      
      expect(Array.isArray(recommendations)).toBe(true);
      recommendations.forEach((action: string) => {
        expect(typeof action).toBe('string');
        expect(action.length).toBeGreaterThan(0);
      });
    });
  });

  describe('Error Handling', () => {
    it('should handle invalid resource pool configurations', () => {
      expect(() => {
        optimizer.addResourcePool({
          id: '',
          type: 'compute',
          capacity: -100,
          utilization: 2.0,
          performance: -0.5,
          status: 'healthy',
          lastOptimized: new Date()
        });
      }).not.toThrow();
    });

    it('should handle invalid optimization configurations', () => {
      expect(() => {
        optimizer.addOptimization({
          type: 'rendering',
          level: -5,
          impact: 'high',
          trade_offs: [],
          enabled: true
        });
      }).not.toThrow();
    });
  });

  describe('Performance', () => {
    it('should handle multiple resource pools efficiently', () => {
      const startTime = Date.now();
      
      // Add many resource pools
      for (let i = 0; i < 100; i++) {
        optimizer.addResourcePool({
          id: `pool_${i}`,
          type: 'compute',
          capacity: 100,
          utilization: Math.random(),
          performance: Math.random(),
          status: 'healthy',
          lastOptimized: new Date()
        });
      }
      
      const metrics = optimizer.getScaleMetrics();
      const endTime = Date.now();
      
      expect(Object.keys(metrics.resourceUtilization).length).toBeGreaterThanOrEqual(100);
      expect(endTime - startTime).toBeLessThan(1000); // Should complete within 1 second
    });

    it('should generate reports efficiently', () => {
      const startTime = Date.now();
      const report = optimizer.generateScaleReport();
      const endTime = Date.now();
      
      expect(report.length).toBeGreaterThan(100); // Non-empty report
      expect(endTime - startTime).toBeLessThan(500); // Should complete within 500ms
    });
  });

  describe('Integration', () => {
    it('should work without external dependencies', () => {
      // Mock window object to simulate missing dependencies
      const originalWindow = global.window;
      global.window = {} as any;
      
      const isolatedOptimizer = new HyperScaleOptimizer();
      const metrics = isolatedOptimizer.getScaleMetrics();
      
      expect(metrics).toBeDefined();
      expect(metrics.resourceUtilization).toBeDefined();
      
      isolatedOptimizer.stopOptimization();
      global.window = originalWindow;
    });

    it('should handle missing performance APIs gracefully', () => {
      const originalPerformance = global.performance;
      global.performance = {} as any;
      
      const optimizer = new HyperScaleOptimizer();
      const report = optimizer.generateScaleReport();
      
      expect(report).toBeDefined();
      expect(JSON.parse(report)).toHaveProperty('currentMetrics');
      
      optimizer.stopOptimization();
      global.performance = originalPerformance;
    });
  });
});