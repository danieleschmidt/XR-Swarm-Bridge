import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { createQuantumHybridOptimizer, runQuickBenchmark } from '../utils/quantumHybridOptimizer';

describe('Quantum Hybrid Optimizer', () => {
  let optimizer: any;

  beforeEach(() => {
    optimizer = createQuantumHybridOptimizer({
      quantum_qubits: 8,
      classical_threads: 4,
      hybrid_ratio: 0.6,
      error_mitigation: true,
      statistical_validation: true
    });
  });

  afterEach(() => {
    vi.restoreAllMocks();
  });

  describe('Initialization', () => {
    it('should initialize with correct configuration', () => {
      expect(optimizer).toBeDefined();
      expect(optimizer.config.quantum_qubits).toBe(8);
      expect(optimizer.config.classical_threads).toBe(4);
      expect(optimizer.config.hybrid_ratio).toBe(0.6);
    });

    it('should initialize all required models', () => {
      expect(optimizer.models.size).toBeGreaterThan(0);
      expect(optimizer.quantum_optimizer).toBeDefined();
      expect(optimizer.statistical_validator).toBeDefined();
    });
  });

  describe('Problem Analysis', () => {
    it('should analyze problem characteristics correctly', () => {
      const test_problem = {
        robots: Array(10).fill(null).map((_, i) => ({
          id: `robot_${i}`,
          position: { x: i * 10, y: i * 5, z: i * 2 },
          velocity: { x: 0, y: 0, z: 0 },
          capabilities: ['move', 'sense'],
          energy_level: 0.8
        })),
        objectives: [
          { type: 'minimize_energy', weight: 0.4 },
          { type: 'maximize_coverage', weight: 0.6 }
        ],
        constraints: [
          { type: 'collision_avoidance', parameters: { min_distance: 2.0 } }
        ],
        environment: {
          obstacles: [],
          communication_zones: [],
          energy_sources: [],
          bounds: { min: { x: 0, y: 0, z: 0 }, max: { x: 100, y: 100, z: 50 } }
        }
      };

      const characteristics = optimizer.analyzeProblemCharacteristics(test_problem);
      
      expect(characteristics.dimensionality).toBe(60); // 10 robots * 6 DOF
      expect(characteristics.non_linearity).toBeGreaterThanOrEqual(0);
      expect(characteristics.constraint_density).toBeGreaterThan(0);
    });

    it('should adapt hybrid ratio based on problem complexity', () => {
      const simple_characteristics = {
        dimensionality: 30,
        non_linearity: 0.2,
        constraint_density: 0.05,
        objective_complexity: 1
      };

      const complex_characteristics = {
        dimensionality: 150,
        non_linearity: 0.9,
        constraint_density: 0.8,
        objective_complexity: 4
      };

      const simple_ratio = optimizer.adaptHybridRatio(simple_characteristics);
      const complex_ratio = optimizer.adaptHybridRatio(complex_characteristics);

      expect(complex_ratio).toBeGreaterThan(simple_ratio);
      expect(simple_ratio).toBeGreaterThanOrEqual(0);
      expect(complex_ratio).toBeLessThanOrEqual(1);
    });
  });

  describe('Quantum State Preparation', () => {
    it('should prepare quantum states for all robots', async () => {
      const robots = [
        { id: 'robot_1', position: { x: 0, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [], energy_level: 0.8 },
        { id: 'robot_2', position: { x: 10, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [], energy_level: 0.7 }
      ];

      await optimizer.prepareQuantumStates(robots);

      robots.forEach(robot => {
        expect(robot.quantum_state).toBeDefined();
        expect(robot.quantum_state.amplitude).toBeGreaterThanOrEqual(0);
        expect(robot.quantum_state.amplitude).toBeLessThanOrEqual(1);
        expect(robot.quantum_state.phase).toBeGreaterThanOrEqual(0);
        expect(robot.quantum_state.phase).toBeLessThanOrEqual(2 * Math.PI);
        expect(robot.quantum_state.coherence_time).toBeGreaterThan(0);
      });
    });
  });

  describe('Entanglement Network Construction', () => {
    it('should create hierarchical entanglement networks', () => {
      const robots = Array(20).fill(null).map((_, i) => ({
        id: `robot_${i}`,
        position: { x: i * 5, y: (i % 4) * 5, z: 0 },
        velocity: { x: 0, y: 0, z: 0 },
        capabilities: ['move'],
        energy_level: 0.5 + Math.random() * 0.5
      }));

      optimizer.constructEntanglementNetwork(robots);

      expect(optimizer.entanglement_network.size).toBe(robots.length);
      
      // Check that each robot has some entanglement partners
      robots.forEach(robot => {
        const partners = optimizer.entanglement_network.get(robot.id);
        expect(partners).toBeDefined();
        expect(partners.size).toBeGreaterThan(0);
      });

      // Check network connectivity
      const total_connections = Array.from(optimizer.entanglement_network.values())
        .reduce((sum, partners) => sum + partners.size, 0) / 2;
      
      expect(total_connections).toBeGreaterThan(robots.length); // Should have more than minimal spanning tree
    });

    it('should select appropriate global leaders', () => {
      const robots = [
        { id: 'robot_1', energy_level: 0.9, position: { x: 0, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [] },
        { id: 'robot_2', energy_level: 0.3, position: { x: 0, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [] },
        { id: 'robot_3', energy_level: 0.8, position: { x: 0, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [] },
        { id: 'robot_4', energy_level: 0.5, position: { x: 0, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [] }
      ];

      const leaders = optimizer.selectGlobalLeaders(robots, 2);

      expect(leaders).toHaveLength(2);
      expect(leaders[0].energy_level).toBeGreaterThanOrEqual(leaders[1].energy_level);
      expect(leaders[0].id).toBe('robot_1'); // Highest energy
    });
  });

  describe('Optimization Execution', () => {
    it('should optimize swarm problems and return valid results', async () => {
      const test_problem = {
        robots: Array(5).fill(null).map((_, i) => ({
          id: `robot_${i}`,
          position: { x: i * 10, y: 0, z: 0 },
          velocity: { x: 0, y: 0, z: 0 },
          capabilities: ['move'],
          energy_level: 0.8
        })),
        objectives: [
          { type: 'minimize_energy', weight: 0.5 },
          { type: 'maximize_coverage', weight: 0.5 }
        ],
        constraints: [
          { type: 'collision_avoidance', parameters: { min_distance: 2.0 } }
        ],
        environment: {
          obstacles: [],
          communication_zones: [],
          energy_sources: [],
          bounds: { min: { x: 0, y: 0, z: 0 }, max: { x: 100, y: 100, z: 50 } }
        }
      };

      const result = await optimizer.optimize(test_problem);

      expect(result).toBeDefined();
      expect(result.solution).toHaveLength(test_problem.robots.length);
      expect(result.objective_value).toBeGreaterThan(0);
      expect(result.execution_time).toBeGreaterThan(0);
      expect(result.quantum_advantage).toBeGreaterThanOrEqual(0);
      expect(result.statistical_significance).toBeGreaterThanOrEqual(0);
      expect(result.statistical_significance).toBeLessThanOrEqual(1);
    }, 10000); // Longer timeout for optimization
  });

  describe('Error Mitigation', () => {
    it('should apply error mitigation when enabled', async () => {
      const test_solution = [
        { id: 'robot_1', position: { x: 0, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [], energy_level: 0.8 }
      ];

      const test_problem = {
        robots: test_solution,
        objectives: [],
        constraints: [],
        environment: { obstacles: [], communication_zones: [], energy_sources: [], bounds: { min: { x: 0, y: 0, z: 0 }, max: { x: 100, y: 100, z: 50 } } }
      };

      const corrected_solution = await optimizer.applyErrorMitigation(test_solution, test_problem);

      expect(corrected_solution).toBeDefined();
      expect(corrected_solution).toHaveLength(test_solution.length);
    });

    it('should measure error syndromes', async () => {
      const test_solution = [
        { id: 'robot_1', position: { x: 0, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [], energy_level: 0.8 },
        { id: 'robot_2', position: { x: 10, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [], energy_level: 0.7 }
      ];

      const syndromes = await optimizer.measureErrorSyndromes(test_solution);

      expect(Array.isArray(syndromes)).toBe(true);
      syndromes.forEach(syndrome => {
        expect(syndrome.robot_id).toBeDefined();
        expect(syndrome.confidence).toBeGreaterThanOrEqual(0);
        expect(syndrome.confidence).toBeLessThanOrEqual(1);
      });
    });
  });

  describe('Statistical Validation', () => {
    it('should validate statistical significance correctly', async () => {
      const quantum_results = [0.8, 0.85, 0.9, 0.82, 0.88];
      const classical_results = [0.7, 0.72, 0.75, 0.68, 0.71];

      const validation = await optimizer.validateStatisticalSignificance(quantum_results, classical_results);

      expect(validation.significant).toBeDefined();
      expect(validation.p_value).toBeGreaterThanOrEqual(0);
      expect(validation.p_value).toBeLessThanOrEqual(1);
      expect(validation.effect_size).toBeDefined();
      expect(validation.confidence_interval).toHaveLength(2);
      expect(validation.confidence_interval[0]).toBeLessThanOrEqual(validation.confidence_interval[1]);
    });

    it('should calculate performance gains correctly', () => {
      const execution_time = 1000; // ms
      const gain = optimizer.calculatePerformanceGain(execution_time);

      expect(gain).toBeGreaterThan(0);
      expect(typeof gain).toBe('number');
    });
  });

  describe('Comprehensive Benchmarking', () => {
    it('should run benchmark suite and generate publication metrics', async () => {
      const benchmark_results = await optimizer.runComprehensiveBenchmark([5, 10], 3);

      expect(benchmark_results.results).toBeDefined();
      expect(benchmark_results.results.length).toBeGreaterThan(0);
      expect(benchmark_results.quantum_advantage_trend).toBeDefined();
      expect(benchmark_results.statistical_summary).toBeDefined();
      expect(benchmark_results.publication_metrics).toBeDefined();

      // Check quantum advantage trend
      expect(benchmark_results.quantum_advantage_trend).toHaveLength(2); // 2 problem sizes
      benchmark_results.quantum_advantage_trend.forEach(advantage => {
        expect(advantage).toBeGreaterThan(0);
      });

      // Check publication metrics
      expect(benchmark_results.publication_metrics.quantum_speedup_factor).toBeGreaterThan(0);
      expect(benchmark_results.publication_metrics.scalability_trend).toBeGreaterThanOrEqual(0);
      expect(benchmark_results.publication_metrics.statistical_power).toBeGreaterThanOrEqual(0);
      expect(benchmark_results.publication_metrics.reproducibility_score).toBeGreaterThanOrEqual(0);
    }, 15000); // Extended timeout for comprehensive benchmark
  });

  describe('Utility Functions', () => {
    it('should calculate distances correctly', () => {
      const pos1 = { x: 0, y: 0, z: 0 };
      const pos2 = { x: 3, y: 4, z: 0 };

      const distance = optimizer.calculateDistance(pos1, pos2);

      expect(distance).toBe(5); // 3-4-5 triangle
    });

    it('should find k-nearest neighbors correctly', () => {
      const target_robot = { id: 'target', position: { x: 0, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [], energy_level: 0.8 };
      const other_robots = [
        { id: 'robot_1', position: { x: 1, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [], energy_level: 0.8 },
        { id: 'robot_2', position: { x: 5, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [], energy_level: 0.8 },
        { id: 'robot_3', position: { x: 2, y: 0, z: 0 }, velocity: { x: 0, y: 0, z: 0 }, capabilities: [], energy_level: 0.8 }
      ];

      const all_robots = [target_robot, ...other_robots];
      const neighbors = optimizer.findKNearestNeighbors(target_robot, all_robots, 2);

      expect(neighbors).toHaveLength(2);
      expect(neighbors[0].id).toBe('robot_1'); // Closest at distance 1
      expect(neighbors[1].id).toBe('robot_3'); // Second closest at distance 2
    });

    it('should perform quantum clustering', () => {
      const robots = Array(12).fill(null).map((_, i) => ({
        id: `robot_${i}`,
        position: { x: i * 5, y: 0, z: 0 },
        velocity: { x: 0, y: 0, z: 0 },
        capabilities: [],
        energy_level: 0.8
      }));

      const clusters = optimizer.performQuantumClustering(robots, 3);

      expect(clusters).toHaveLength(3);
      expect(clusters.flat()).toHaveLength(robots.length);
      
      // Check that each cluster has appropriate size
      clusters.forEach(cluster => {
        expect(cluster.length).toBeGreaterThan(0);
        expect(cluster.length).toBeLessThanOrEqual(5); // Max cluster size for 12 robots in 3 clusters
      });
    });
  });

  describe('Integration with Quantum Performance Optimizer', () => {
    it('should integrate with quantum optimizer for resource allocation', async () => {
      const targets = [
        { id: 'robot_1', priority: 1, resource_cost: 10, quantum_state: { amplitude: 0.8, phase: 0, entanglement: [] } },
        { id: 'robot_2', priority: 2, resource_cost: 15, quantum_state: { amplitude: 0.6, phase: Math.PI/4, entanglement: [] } }
      ];

      const result = await optimizer.quantum_optimizer.optimizeResourceAllocation(targets, 50);

      expect(result.allocation).toBeDefined();
      expect(result.performance_gain).toBeGreaterThan(0);
      expect(result.allocation.size).toBeGreaterThan(0);
    });
  });
});

describe('Statistical Validator', () => {
  let validator: any;

  beforeEach(() => {
    const optimizer = createQuantumHybridOptimizer();
    validator = optimizer.statistical_validator;
  });

  describe('Paired T-Test', () => {
    it('should perform paired t-test correctly', () => {
      const sample1 = [1, 2, 3, 4, 5];
      const sample2 = [1.1, 2.1, 3.1, 4.1, 5.1];

      const result = validator.pairedTTest(sample1, sample2);

      expect(result.t_statistic).toBeDefined();
      expect(result.p_value).toBeGreaterThanOrEqual(0);
      expect(result.p_value).toBeLessThanOrEqual(1);
      expect(result.degrees_freedom).toBe(4); // n - 1
    });

    it('should throw error for unequal sample sizes', () => {
      const sample1 = [1, 2, 3];
      const sample2 = [1, 2];

      expect(() => validator.pairedTTest(sample1, sample2)).toThrow();
    });
  });

  describe('Mann-Whitney U Test', () => {
    it('should perform Mann-Whitney U test correctly', () => {
      const sample1 = [1, 3, 5, 7, 9];
      const sample2 = [2, 4, 6, 8, 10];

      const result = validator.mannWhitneyUTest(sample1, sample2);

      expect(result.u_statistic).toBeDefined();
      expect(result.p_value).toBeGreaterThanOrEqual(0);
      expect(result.p_value).toBeLessThanOrEqual(1);
    });
  });

  describe('Cohen\'s D', () => {
    it('should calculate effect size correctly', () => {
      const sample1 = [1, 2, 3, 4, 5];
      const sample2 = [2, 3, 4, 5, 6]; // Same variance, different means

      const cohens_d = validator.cohensD(sample1, sample2);

      expect(typeof cohens_d).toBe('number');
      expect(cohens_d).toBeGreaterThan(0); // Should be positive since sample2 > sample1
    });
  });

  describe('Confidence Interval', () => {
    it('should calculate confidence interval correctly', () => {
      const sample1 = [1, 2, 3, 4, 5];
      const sample2 = [2, 3, 4, 5, 6];

      const ci = validator.confidenceInterval(sample1, sample2, 0.95);

      expect(ci).toHaveLength(2);
      expect(ci[0]).toBeLessThanOrEqual(ci[1]);
      expect(typeof ci[0]).toBe('number');
      expect(typeof ci[1]).toBe('number');
    });
  });
});

describe('Quick Benchmark', () => {
  it('should run quick benchmark successfully', async () => {
    const results = await runQuickBenchmark();

    expect(results).toBeDefined();
    expect(results.quantum_advantage).toBeGreaterThan(0);
    expect(results.scalability).toBeGreaterThanOrEqual(0);
    expect(results.statistical_power).toBeGreaterThanOrEqual(0);
  }, 20000); // Extended timeout for benchmark
});

describe('Integration Tests', () => {
  it('should handle large-scale optimization problems', async () => {
    const large_optimizer = createQuantumHybridOptimizer({
      quantum_qubits: 16,
      classical_threads: 8,
      hybrid_ratio: 0.8
    });

    const large_problem = {
      robots: Array(50).fill(null).map((_, i) => ({
        id: `robot_${i}`,
        position: { 
          x: (Math.random() - 0.5) * 200, 
          y: (Math.random() - 0.5) * 200, 
          z: Math.random() * 100 
        },
        velocity: { x: 0, y: 0, z: 0 },
        capabilities: ['move', 'sense'],
        energy_level: 0.5 + Math.random() * 0.5
      })),
      objectives: [
        { type: 'minimize_energy', weight: 0.3 },
        { type: 'maximize_coverage', weight: 0.4 },
        { type: 'maximize_resilience', weight: 0.3 }
      ],
      constraints: [
        { type: 'collision_avoidance', parameters: { min_distance: 5.0 } },
        { type: 'energy_budget', parameters: { max_consumption: 1000 } },
        { type: 'communication_range', parameters: { max_range: 50 } }
      ],
      environment: {
        obstacles: [
          { position: { x: 0, y: 0, z: 25 }, shape: 'sphere', dimensions: { x: 10, y: 10, z: 10 }, static: true },
          { position: { x: 50, y: 50, z: 25 }, shape: 'box', dimensions: { x: 20, y: 20, z: 30 }, static: true }
        ],
        communication_zones: [
          { center: { x: 0, y: 0, z: 0 }, radius: 100, bandwidth: 1000, latency: 10 }
        ],
        energy_sources: [
          { position: { x: -50, y: -50, z: 0 }, power_output: 500, range: 30 }
        ],
        bounds: { min: { x: -100, y: -100, z: 0 }, max: { x: 100, y: 100, z: 100 } }
      }
    };

    const result = await large_optimizer.optimize(large_problem);

    expect(result.solution).toHaveLength(50);
    expect(result.objective_value).toBeGreaterThan(0);
    expect(result.execution_time).toBeGreaterThan(0);
    expect(result.quantum_advantage).toBeGreaterThan(0);
  }, 30000); // Very extended timeout for large-scale test

  it('should maintain performance across multiple optimization runs', async () => {
    const optimizer = createQuantumHybridOptimizer();
    const test_problem = {
      robots: Array(10).fill(null).map((_, i) => ({
        id: `robot_${i}`,
        position: { x: i * 5, y: 0, z: 0 },
        velocity: { x: 0, y: 0, z: 0 },
        capabilities: ['move'],
        energy_level: 0.8
      })),
      objectives: [{ type: 'minimize_energy', weight: 1.0 }],
      constraints: [{ type: 'collision_avoidance', parameters: { min_distance: 2.0 } }],
      environment: {
        obstacles: [],
        communication_zones: [],
        energy_sources: [],
        bounds: { min: { x: 0, y: 0, z: 0 }, max: { x: 50, y: 50, z: 25 } }
      }
    };

    const results = [];
    for (let i = 0; i < 5; i++) {
      const result = await optimizer.optimize(test_problem);
      results.push(result);
    }

    // Check that all runs completed successfully
    expect(results).toHaveLength(5);
    results.forEach(result => {
      expect(result.solution).toHaveLength(10);
      expect(result.objective_value).toBeGreaterThan(0);
    });

    // Check performance consistency
    const execution_times = results.map(r => r.execution_time);
    const avg_time = execution_times.reduce((sum, t) => sum + t, 0) / execution_times.length;
    const max_deviation = Math.max(...execution_times.map(t => Math.abs(t - avg_time)));
    
    expect(max_deviation / avg_time).toBeLessThan(2.0); // Performance should be reasonably consistent
  }, 25000); // Extended timeout for multiple runs
});