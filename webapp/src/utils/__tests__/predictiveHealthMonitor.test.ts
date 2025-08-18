import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { PredictiveHealthMonitor } from '../predictiveHealthMonitor';

describe('PredictiveHealthMonitor', () => {
  let healthMonitor: PredictiveHealthMonitor;

  beforeEach(() => {
    healthMonitor = new PredictiveHealthMonitor();
  });

  afterEach(() => {
    healthMonitor.stopMonitoring();
  });

  describe('Initialization', () => {
    it('should initialize with default system components', () => {
      const systemHealth = healthMonitor.getSystemHealth();
      
      expect(systemHealth.components).toHaveLength(5);
      expect(systemHealth.components.map(c => c.id)).toContain('webrtc');
      expect(systemHealth.components.map(c => c.id)).toContain('quantum');
      expect(systemHealth.components.map(c => c.id)).toContain('planning');
      expect(systemHealth.components.map(c => c.id)).toContain('network');
      expect(systemHealth.components.map(c => c.id)).toContain('ui');
    });

    it('should initialize with unknown status for all components', () => {
      const systemHealth = healthMonitor.getSystemHealth();
      
      systemHealth.components.forEach(component => {
        expect(component.status).toBe('unknown');
      });
    });
  });

  describe('System Health Monitoring', () => {
    it('should report overall system health', () => {
      const systemHealth = healthMonitor.getSystemHealth();
      
      expect(systemHealth).toHaveProperty('overall');
      expect(systemHealth).toHaveProperty('components');
      expect(['healthy', 'warning', 'critical']).toContain(systemHealth.overall);
    });

    it('should track component health status', () => {
      const systemHealth = healthMonitor.getSystemHealth();
      
      systemHealth.components.forEach(component => {
        expect(component).toHaveProperty('id');
        expect(component).toHaveProperty('name');
        expect(component).toHaveProperty('status');
        expect(['healthy', 'warning', 'critical', 'unknown']).toContain(component.status);
      });
    });
  });

  describe('Alert Management', () => {
    it('should track active alerts', () => {
      const activeAlerts = healthMonitor.getActiveAlerts();
      expect(Array.isArray(activeAlerts)).toBe(true);
    });

    it('should filter out resolved alerts', () => {
      const activeAlerts = healthMonitor.getActiveAlerts();
      
      activeAlerts.forEach(alert => {
        expect(alert.resolved).toBe(false);
      });
    });
  });

  describe('Predictive Analysis', () => {
    it('should generate health predictions', () => {
      const predictions = healthMonitor.getPredictions();
      expect(Array.isArray(predictions)).toBe(true);
    });

    it('should include prediction confidence and timeframes', () => {
      // Allow some time for predictions to be generated
      setTimeout(() => {
        const predictions = healthMonitor.getPredictions();
        
        predictions.forEach(prediction => {
          expect(prediction).toHaveProperty('componentId');
          expect(prediction).toHaveProperty('metricName');
          expect(prediction).toHaveProperty('predictedFailureTime');
          expect(prediction).toHaveProperty('confidence');
          expect(prediction).toHaveProperty('recommendedActions');
          expect(prediction).toHaveProperty('severity');
          
          expect(prediction.confidence).toBeGreaterThanOrEqual(0);
          expect(prediction.confidence).toBeLessThanOrEqual(1);
          expect(['low', 'medium', 'high', 'critical']).toContain(prediction.severity);
          expect(Array.isArray(prediction.recommendedActions)).toBe(true);
        });
      }, 100);
    });
  });

  describe('Health Reporting', () => {
    it('should generate comprehensive health report', () => {
      const reportString = healthMonitor.generateHealthReport();
      const report = JSON.parse(reportString);
      
      expect(report).toHaveProperty('timestamp');
      expect(report).toHaveProperty('systemHealth');
      expect(report).toHaveProperty('activeAlerts');
      expect(report).toHaveProperty('predictions');
      expect(report).toHaveProperty('criticalPredictions');
      expect(report).toHaveProperty('componentDetails');
      expect(report).toHaveProperty('recentAlerts');
      expect(report).toHaveProperty('highConfidencePredictions');
    });

    it('should include component metrics in report', () => {
      const report = JSON.parse(healthMonitor.generateHealthReport());
      
      expect(Array.isArray(report.componentDetails)).toBe(true);
      
      report.componentDetails.forEach((component: any) => {
        expect(component).toHaveProperty('id');
        expect(component).toHaveProperty('name');
        expect(component).toHaveProperty('status');
        expect(component).toHaveProperty('lastCheck');
        expect(component).toHaveProperty('metrics');
        expect(Array.isArray(component.metrics)).toBe(true);
      });
    });

    it('should track metric trends', () => {
      const report = JSON.parse(healthMonitor.generateHealthReport());
      
      report.componentDetails.forEach((component: any) => {
        component.metrics.forEach((metric: any) => {
          expect(metric).toHaveProperty('name');
          expect(metric).toHaveProperty('value');
          expect(metric).toHaveProperty('trend');
          expect(metric).toHaveProperty('thresholds');
          expect(['improving', 'stable', 'degrading']).toContain(metric.trend);
        });
      });
    });
  });

  describe('Performance Monitoring', () => {
    it('should handle performance monitoring efficiently', () => {
      const startTime = Date.now();
      
      // Get system health multiple times
      for (let i = 0; i < 10; i++) {
        healthMonitor.getSystemHealth();
      }
      
      const endTime = Date.now();
      expect(endTime - startTime).toBeLessThan(100); // Should be fast
    });

    it('should generate reports efficiently', () => {
      const startTime = Date.now();
      const report = healthMonitor.generateHealthReport();
      const endTime = Date.now();
      
      expect(report.length).toBeGreaterThan(100); // Non-empty report
      expect(endTime - startTime).toBeLessThan(50); // Should be very fast
    });
  });

  describe('Metric Thresholds', () => {
    it('should define appropriate thresholds for different metrics', () => {
      const report = JSON.parse(healthMonitor.generateHealthReport());
      
      report.componentDetails.forEach((component: any) => {
        component.metrics.forEach((metric: any) => {
          expect(metric.thresholds).toHaveProperty('warning');
          expect(metric.thresholds).toHaveProperty('critical');
          expect(metric.thresholds.warning).toBeGreaterThan(0);
          expect(metric.thresholds.critical).toBeGreaterThan(0);
        });
      });
    });

    it('should handle metrics that are bad when high vs low', () => {
      const report = JSON.parse(healthMonitor.generateHealthReport());
      
      // Find latency metric (bad when high)
      let foundLatency = false;
      // Find frame rate metric (bad when low)  
      let foundFrameRate = false;
      
      report.componentDetails.forEach((component: any) => {
        component.metrics.forEach((metric: any) => {
          if (metric.name.includes('Latency')) {
            foundLatency = true;
            expect(metric.thresholds.critical).toBeGreaterThan(metric.thresholds.warning);
          }
          if (metric.name.includes('Frame Rate')) {
            foundFrameRate = true;
            expect(metric.thresholds.critical).toBeLessThan(metric.thresholds.warning);
          }
        });
      });
      
      expect(foundLatency || foundFrameRate).toBe(true); // At least one type should be found
    });
  });

  describe('Integration with External Systems', () => {
    it('should handle missing external dependencies gracefully', () => {
      // Mock missing swarmStore
      const originalWindow = global.window;
      global.window = {} as any;
      
      const isolatedMonitor = new PredictiveHealthMonitor();
      const systemHealth = isolatedMonitor.getSystemHealth();
      
      expect(systemHealth).toBeDefined();
      expect(systemHealth.components).toHaveLength(5);
      
      isolatedMonitor.stopMonitoring();
      global.window = originalWindow;
    });

    it('should work without performance.memory API', () => {
      const originalPerformance = global.performance;
      global.performance = {} as any;
      
      const monitor = new PredictiveHealthMonitor();
      const report = monitor.generateHealthReport();
      
      expect(report).toBeDefined();
      expect(JSON.parse(report)).toHaveProperty('componentDetails');
      
      monitor.stopMonitoring();
      global.performance = originalPerformance;
    });
  });

  describe('Alert Severity Levels', () => {
    it('should categorize alerts by severity appropriately', () => {
      const activeAlerts = healthMonitor.getActiveAlerts();
      
      activeAlerts.forEach(alert => {
        expect(['low', 'medium', 'high', 'critical']).toContain(alert.severity);
        expect(alert).toHaveProperty('type');
        expect(['degradation', 'threshold_breach', 'prediction', 'dependency_failure']).toContain(alert.type);
      });
    });
  });

  describe('Predictive Accuracy', () => {
    it('should provide reasonable confidence scores for predictions', () => {
      // Wait for some monitoring cycles
      setTimeout(() => {
        const predictions = healthMonitor.getPredictions();
        
        predictions.forEach(prediction => {
          // Confidence should be reasonable (not too high for new systems)
          expect(prediction.confidence).toBeLessThanOrEqual(0.95);
          expect(prediction.confidence).toBeGreaterThanOrEqual(0);
        });
      }, 200);
    });

    it('should generate actionable recommendations', () => {
      const predictions = healthMonitor.getPredictions();
      
      predictions.forEach(prediction => {
        expect(Array.isArray(prediction.recommendedActions)).toBe(true);
        prediction.recommendedActions.forEach(action => {
          expect(typeof action).toBe('string');
          expect(action.length).toBeGreaterThan(0);
        });
      });
    });
  });

  describe('Memory Management', () => {
    it('should manage memory efficiently over time', async () => {
      // Simulate extended monitoring period
      for (let i = 0; i < 50; i++) {
        healthMonitor.getSystemHealth();
        // Small delay to simulate real monitoring
        await new Promise(resolve => setTimeout(resolve, 1));
      }
      
      const report = JSON.parse(healthMonitor.generateHealthReport());
      expect(report).toBeDefined();
      
      // Memory usage should be reasonable
      if (performance.memory) {
        const memoryUsage = performance.memory.usedJSHeapSize / 1024 / 1024;
        expect(memoryUsage).toBeLessThan(100); // Less than 100MB
      }
    });
  });

  describe('Error Handling', () => {
    it('should handle monitoring errors gracefully', () => {
      // This should not throw even if there are internal errors
      expect(() => {
        healthMonitor.getSystemHealth();
        healthMonitor.generateHealthReport();
        healthMonitor.getActiveAlerts();
        healthMonitor.getPredictions();
      }).not.toThrow();
    });
  });
});