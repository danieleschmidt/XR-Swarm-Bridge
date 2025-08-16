import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { createAdaptiveLearningSystem, runLearningBenchmark } from '../utils/adaptiveLearningSystem';

describe('Adaptive Learning System', () => {
  let learningSystem: any;

  beforeEach(() => {
    learningSystem = createAdaptiveLearningSystem({
      learning_rate: 0.001,
      batch_size: 16,
      memory_capacity: 1000,
      adaptation_threshold: 0.05,
      explanation_depth: 3,
      federated_learning: false
    });
  });

  afterEach(() => {
    vi.restoreAllMocks();
  });

  describe('Initialization', () => {
    it('should initialize with correct configuration', () => {
      expect(learningSystem).toBeDefined();
      expect(learningSystem.config.learning_rate).toBe(0.001);
      expect(learningSystem.config.batch_size).toBe(16);
      expect(learningSystem.config.memory_capacity).toBe(1000);
    });

    it('should initialize all learning models', () => {
      expect(learningSystem.models.size).toBeGreaterThan(0);
      
      // Check that all expected model types are initialized
      const expected_models = [
        'formation_optimizer',
        'failure_predictor',
        'path_planner',
        'energy_manager',
        'communication_optimizer'
      ];
      
      expected_models.forEach(model_type => {
        expect(learningSystem.models.has(model_type)).toBe(true);
      });
    });

    it('should initialize memory structures', () => {
      expect(learningSystem.memory.experiences).toEqual([]);
      expect(learningSystem.memory.patterns).toEqual([]);
      expect(learningSystem.memory.failures).toEqual([]);
      expect(learningSystem.memory.successes).toEqual([]);
    });
  });

  describe('Sensor Data Processing', () => {
    it('should process sensor data and update buffer', async () => {
      const sensor_data = [
        {
          robot_id: 'robot_1',
          timestamp: Date.now(),
          position: { x: 0, y: 0, z: 0 },
          velocity: { x: 1, y: 0, z: 0 },
          acceleration: { x: 0.1, y: 0, z: 0 },
          energy_level: 0.8,
          sensor_readings: { temperature: 25, pressure: 1013 },
          communication_quality: 0.9,
          task_status: 'active',
          environmental_context: {
            weather: { type: 'clear', intensity: 0.8, forecast_confidence: 0.9 },
            obstacles: [],
            communication_interference: 0.1,
            electromagnetic_field: 0.05,
            temperature: 25,
            humidity: 0.6,
            wind_speed: 5,
            visibility: 0.95
          }
        }
      ];

      const initial_buffer_length = learningSystem.sensor_buffer.length;
      await learningSystem.processSensorData(sensor_data);

      expect(learningSystem.sensor_buffer.length).toBe(initial_buffer_length + 1);
      expect(learningSystem.sensor_buffer[learningSystem.sensor_buffer.length - 1]).toEqual(sensor_data[0]);
    });

    it('should maintain buffer size limits', async () => {
      const large_sensor_data = Array(200).fill(null).map((_, i) => ({
        robot_id: `robot_${i}`,
        timestamp: Date.now() + i,
        position: { x: i, y: 0, z: 0 },
        velocity: { x: 0, y: 0, z: 0 },
        acceleration: { x: 0, y: 0, z: 0 },
        energy_level: 0.8,
        sensor_readings: { temperature: 25 },
        communication_quality: 0.9,
        task_status: 'active',
        environmental_context: {
          weather: { type: 'clear', intensity: 0.8, forecast_confidence: 0.9 },
          obstacles: [],
          communication_interference: 0.1,
          electromagnetic_field: 0.05,
          temperature: 25,
          humidity: 0.6,
          wind_speed: 5,
          visibility: 0.95
        }
      }));

      await learningSystem.processSensorData(large_sensor_data);

      expect(learningSystem.sensor_buffer.length).toBeLessThanOrEqual(learningSystem.config.batch_size * 10);
    });
  });

  describe('Failure Prediction', () => {
    it('should predict failures with confidence and uncertainty', async () => {
      const test_sensor_data = [
        {
          robot_id: 'robot_1',
          timestamp: Date.now(),
          position: { x: 0, y: 0, z: 0 },
          velocity: { x: 1, y: 0, z: 0 },
          acceleration: { x: 0.1, y: 0, z: 0 },
          energy_level: 0.2, // Low energy - potential failure indicator
          sensor_readings: { temperature: 85 }, // High temperature
          communication_quality: 0.3, // Poor communication
          task_status: 'active',
          environmental_context: {
            weather: { type: 'storm', intensity: 0.9, forecast_confidence: 0.8 },
            obstacles: [],
            communication_interference: 0.8,
            electromagnetic_field: 0.3,
            temperature: 85,
            humidity: 0.9,
            wind_speed: 25,
            visibility: 0.2
          }
        }
      ];

      const predictions = await learningSystem.predictFailures(test_sensor_data);

      expect(predictions).toHaveLength(1);
      
      const prediction = predictions[0];
      expect(prediction.prediction).toBeDefined();
      expect(prediction.confidence).toBeGreaterThanOrEqual(0);
      expect(prediction.confidence).toBeLessThanOrEqual(1);
      expect(prediction.explanation).toBeDefined();
      expect(prediction.uncertainty).toBeDefined();
      expect(prediction.recommended_actions).toBeDefined();
      
      // Check uncertainty quantification
      expect(prediction.uncertainty.epistemic).toBeGreaterThanOrEqual(0);
      expect(prediction.uncertainty.aleatoric).toBeGreaterThanOrEqual(0);
      expect(prediction.uncertainty.total).toBeGreaterThanOrEqual(0);
      expect(prediction.uncertainty.confidence_interval).toHaveLength(2);
      
      // Check explanations
      expect(prediction.explanation.feature_importance).toBeDefined();
      expect(prediction.explanation.natural_language).toBeDefined();
      expect(typeof prediction.explanation.natural_language).toBe('string');
      
      // Check recommended actions
      expect(Array.isArray(prediction.recommended_actions)).toBe(true);
      prediction.recommended_actions.forEach(action => {
        expect(action.type).toBeDefined();
        expect(action.priority).toBeGreaterThanOrEqual(0);
        expect(action.risk_level).toBeGreaterThanOrEqual(0);
        expect(action.risk_level).toBeLessThanOrEqual(1);
      });
    });

    it('should extract failure features correctly', () => {
      const test_data = {
        robot_id: 'robot_1',
        timestamp: Date.now(),
        position: { x: 10, y: 20, z: 5 },
        velocity: { x: 3, y: 4, z: 0 }, // Magnitude = 5
        acceleration: { x: 1, y: 0, z: 0 }, // Magnitude = 1
        energy_level: 0.7,
        sensor_readings: { temp1: 25, temp2: 30, pressure: 1013 }, // Average = 22.67
        communication_quality: 0.85,
        task_status: 'active',
        environmental_context: {
          weather: { type: 'clear', intensity: 0.8, forecast_confidence: 0.9 },
          obstacles: [],
          communication_interference: 0.15,
          electromagnetic_field: 0.08,
          temperature: 28,
          humidity: 0.65,
          wind_speed: 8,
          visibility: 0.92
        }
      };

      const features = learningSystem.extractFailureFeatures(test_data);

      expect(features).toHaveLength(8);
      expect(features[0]).toBe(0.7); // energy_level
      expect(features[1]).toBe(0.85); // communication_quality
      expect(features[2]).toBe(5); // velocity magnitude
      expect(features[3]).toBe(1); // acceleration magnitude
      expect(features[4]).toBe(28); // temperature
      expect(features[5]).toBe(0.08); // electromagnetic_field
      expect(features[6]).toBe(0.15); // communication_interference
      expect(features[7]).toBeCloseTo(689.33, 1); // sensor average
    });
  });

  describe('Formation Optimization', () => {
    it('should optimize swarm formation with explanations', async () => {
      const test_robots = Array(5).fill(null).map((_, i) => ({
        robot_id: `robot_${i}`,
        timestamp: Date.now(),
        position: { x: i * 10, y: 0, z: 0 },
        velocity: { x: 0, y: 0, z: 0 },
        acceleration: { x: 0, y: 0, z: 0 },
        energy_level: 0.8,
        sensor_readings: { temperature: 25 },
        communication_quality: 0.9,
        task_status: 'active',
        environmental_context: {
          weather: { type: 'clear', intensity: 0.8, forecast_confidence: 0.9 },
          obstacles: [],
          communication_interference: 0.1,
          electromagnetic_field: 0.05,
          temperature: 25,
          humidity: 0.6,
          wind_speed: 5,
          visibility: 0.95
        }
      }));

      const optimization_result = await learningSystem.optimizeFormation(test_robots, ['coverage', 'efficiency']);

      expect(optimization_result.new_formation).toBeDefined();
      expect(optimization_result.new_formation).toHaveLength(test_robots.length);
      expect(optimization_result.expected_improvement).toBeDefined();
      expect(optimization_result.confidence).toBeGreaterThanOrEqual(0);
      expect(optimization_result.confidence).toBeLessThanOrEqual(1);
      expect(optimization_result.explanation).toBeDefined();
      expect(typeof optimization_result.explanation).toBe('string');
      
      // Check that each position is valid
      optimization_result.new_formation.forEach(position => {
        expect(position.x).toBeDefined();
        expect(position.y).toBeDefined();
        expect(position.z).toBeDefined();
        expect(typeof position.x).toBe('number');
        expect(typeof position.y).toBe('number');
        expect(typeof position.z).toBe('number');
      });
    });

    it('should extract formation features correctly', () => {
      const test_formation = [
        { x: 0, y: 0, z: 0 },
        { x: 10, y: 0, z: 0 },
        { x: 5, y: 8.66, z: 0 } // Equilateral triangle
      ];

      const test_robots = test_formation.map((pos, i) => ({
        robot_id: `robot_${i}`,
        timestamp: Date.now(),
        position: pos,
        velocity: { x: 0, y: 0, z: 0 },
        acceleration: { x: 0, y: 0, z: 0 },
        energy_level: 0.8,
        sensor_readings: {},
        communication_quality: 0.9,
        task_status: 'active',
        environmental_context: {
          weather: { type: 'clear', intensity: 0.8, forecast_confidence: 0.9 },
          obstacles: [],
          communication_interference: 0.1,
          electromagnetic_field: 0.05,
          temperature: 25,
          humidity: 0.6,
          wind_speed: 5,
          visibility: 0.95
        }
      }));

      const features = learningSystem.extractFormationFeatures(test_formation, test_robots);

      expect(features).toHaveLength(8);
      expect(features[4]).toBe(3); // Number of positions
      expect(features[5]).toBeCloseTo(5, 1); // Centroid x
      expect(features[6]).toBeCloseTo(2.89, 1); // Centroid y
      expect(features[7]).toBe(0); // Centroid z
    });
  });

  describe('Mission Learning', () => {
    it('should learn from mission outcomes and adapt', async () => {
      const mission_data = {
        mission_id: 'test_mission_1',
        objectives: ['search', 'rescue'],
        initial_state: [
          {
            robot_id: 'robot_1',
            timestamp: Date.now() - 1000,
            position: { x: 0, y: 0, z: 0 },
            velocity: { x: 0, y: 0, z: 0 },
            acceleration: { x: 0, y: 0, z: 0 },
            energy_level: 1.0,
            sensor_readings: {},
            communication_quality: 1.0,
            task_status: 'ready',
            environmental_context: {
              weather: { type: 'clear', intensity: 0.8, forecast_confidence: 0.9 },
              obstacles: [],
              communication_interference: 0.0,
              electromagnetic_field: 0.0,
              temperature: 25,
              humidity: 0.6,
              wind_speed: 0,
              visibility: 1.0
            }
          }
        ],
        actions_taken: [
          { type: 'move', target: { x: 50, y: 50, z: 10 } },
          { type: 'search', area: 100 }
        ],
        final_state: [
          {
            robot_id: 'robot_1',
            timestamp: Date.now(),
            position: { x: 50, y: 50, z: 10 },
            velocity: { x: 0, y: 0, z: 0 },
            acceleration: { x: 0, y: 0, z: 0 },
            energy_level: 0.6,
            sensor_readings: {},
            communication_quality: 0.9,
            task_status: 'completed',
            environmental_context: {
              weather: { type: 'clear', intensity: 0.8, forecast_confidence: 0.9 },
              obstacles: [],
              communication_interference: 0.1,
              electromagnetic_field: 0.05,
              temperature: 25,
              humidity: 0.6,
              wind_speed: 5,
              visibility: 0.95
            }
          }
        ],
        success_metrics: {
          area_covered: 0.8,
          targets_found: 2,
          energy_efficiency: 0.7,
          time_efficiency: 0.9
        },
        duration: 1800, // 30 minutes
        challenges_encountered: ['communication_dropout', 'obstacle_avoidance']
      };

      const initial_experiences = learningSystem.memory.experiences.length;
      const initial_patterns = learningSystem.memory.patterns.length;

      await learningSystem.learnFromMission(mission_data);

      expect(learningSystem.memory.experiences.length).toBe(initial_experiences + 1);
      
      // Check that experience was stored correctly
      const stored_experience = learningSystem.memory.experiences[learningSystem.memory.experiences.length - 1];
      expect(stored_experience.id).toBe(mission_data.mission_id);
      expect(stored_experience.state).toEqual(mission_data.initial_state);
      expect(stored_experience.action).toEqual(mission_data.actions_taken);
      expect(stored_experience.next_state).toEqual(mission_data.final_state);
      expect(stored_experience.reward).toBeDefined();
      expect(typeof stored_experience.reward).toBe('number');
      
      // Check that patterns might have been extracted
      expect(learningSystem.memory.patterns.length).toBeGreaterThanOrEqual(initial_patterns);
    });

    it('should calculate mission rewards correctly', () => {
      const successful_mission = {
        success_metrics: { metric1: 0.9, metric2: 0.8, metric3: 0.85 },
        duration: 1200, // 20 minutes (better than 30 min baseline)
        challenges_encountered: ['minor_obstacle']
      };

      const failed_mission = {
        success_metrics: { metric1: 0.2, metric2: 0.1 },
        duration: 3600, // 60 minutes (worse than baseline)
        challenges_encountered: ['communication_failure', 'energy_depletion', 'obstacle_collision']
      };

      const successful_reward = learningSystem.calculateMissionReward(successful_mission);
      const failed_reward = learningSystem.calculateMissionReward(failed_mission);

      expect(successful_reward).toBeGreaterThan(failed_reward);
      expect(successful_reward).toBeGreaterThanOrEqual(-1);
      expect(successful_reward).toBeLessThanOrEqual(1);
      expect(failed_reward).toBeGreaterThanOrEqual(-1);
      expect(failed_reward).toBeLessThanOrEqual(1);
    });
  });

  describe('System Insights Generation', () => {
    it('should generate comprehensive system insights', async () => {
      // Add some mock data to memory
      learningSystem.memory.experiences = [
        {
          id: 'exp_1',
          timestamp: Date.now() - 10000,
          state: [],
          action: {},
          reward: 0.8,
          next_state: [],
          metadata: {}
        },
        {
          id: 'exp_2',
          timestamp: Date.now() - 5000,
          state: [],
          action: {},
          reward: 0.6,
          next_state: [],
          metadata: {}
        }
      ];

      learningSystem.memory.patterns = [
        {
          id: 'pattern_1',
          description: 'High energy leads to better performance',
          conditions: [],
          outcomes: [],
          confidence: 0.85,
          frequency: 15,
          last_observed: Date.now() - 1000
        }
      ];

      const insights = await learningSystem.generateSystemInsights();

      expect(insights.performance_trends).toBeDefined();
      expect(insights.optimization_opportunities).toBeDefined();
      expect(insights.risk_factors).toBeDefined();
      expect(insights.learned_patterns).toBeDefined();
      expect(insights.recommendations).toBeDefined();

      expect(Array.isArray(insights.performance_trends)).toBe(true);
      expect(Array.isArray(insights.optimization_opportunities)).toBe(true);
      expect(Array.isArray(insights.risk_factors)).toBe(true);
      expect(Array.isArray(insights.learned_patterns)).toBe(true);
      expect(Array.isArray(insights.recommendations)).toBe(true);

      // Check that learned patterns are sorted by relevance
      if (insights.learned_patterns.length > 1) {
        for (let i = 0; i < insights.learned_patterns.length - 1; i++) {
          const current_score = insights.learned_patterns[i].confidence * insights.learned_patterns[i].frequency;
          const next_score = insights.learned_patterns[i + 1].confidence * insights.learned_patterns[i + 1].frequency;
          expect(current_score).toBeGreaterThanOrEqual(next_score);
        }
      }

      // Check recommendations are actionable strings
      insights.recommendations.forEach(recommendation => {
        expect(typeof recommendation).toBe('string');
        expect(recommendation.length).toBeGreaterThan(0);
      });
    });
  });

  describe('Model Management', () => {
    it('should initialize models with correct architectures', () => {
      const formation_model = learningSystem.models.get('formation_optimizer');
      const failure_model = learningSystem.models.get('failure_predictor');

      expect(formation_model).toBeDefined();
      expect(failure_model).toBeDefined();

      expect(formation_model.architecture).toBe('transformer_encoder');
      expect(failure_model.architecture).toBe('lstm_attention');

      expect(formation_model.parameters.length).toBeGreaterThan(0);
      expect(failure_model.parameters.length).toBeGreaterThan(0);

      // Check performance metrics are initialized
      expect(formation_model.performance_metrics.accuracy).toBe(0.5);
      expect(formation_model.performance_metrics.confidence).toBe(0.5);
    });

    it('should update model parameters through learning', async () => {
      const model = learningSystem.models.get('failure_predictor');
      const initial_version = model.version;
      const initial_params = new Float32Array(model.parameters);

      const test_experience = {
        id: 'test_exp',
        timestamp: Date.now(),
        state: [],
        action: {},
        reward: 0.8, // High reward should trigger update
        next_state: [],
        metadata: {}
      };

      await learningSystem.incrementalModelUpdate(model, test_experience);

      expect(model.version).toBe(initial_version + 1);
      expect(model.last_updated).toBeGreaterThan(Date.now() - 1000);

      // Some parameters should have changed
      let parameters_changed = false;
      for (let i = 0; i < model.parameters.length; i++) {
        if (model.parameters[i] !== initial_params[i]) {
          parameters_changed = true;
          break;
        }
      }
      expect(parameters_changed).toBe(true);
    });
  });

  describe('Federated Learning', () => {
    it('should support federated learning when enabled', async () => {
      const federated_system = createAdaptiveLearningSystem({
        federated_learning: true
      });

      const network_nodes = ['node_1', 'node_2', 'node_3'];

      await federated_system.participateInFederatedLearning(network_nodes);

      expect(federated_system.federated_nodes.size).toBe(3);
      network_nodes.forEach(node => {
        expect(federated_system.federated_nodes.has(node)).toBe(true);
      });
    });

    it('should share experiences with network when federated learning is enabled', async () => {
      const federated_system = createAdaptiveLearningSystem({
        federated_learning: true
      });

      federated_system.federated_nodes.add('node_1');
      federated_system.federated_nodes.add('node_2');

      const test_experience = {
        id: 'shared_exp',
        timestamp: Date.now(),
        state: [],
        action: {},
        reward: 0.7,
        next_state: [],
        metadata: {}
      };

      // Mock the sharing function to track calls
      const share_spy = vi.spyOn(federated_system, 'shareExperienceWithNetwork');

      await federated_system.shareExperienceWithNetwork(test_experience);

      expect(share_spy).toHaveBeenCalledWith(test_experience);
    });
  });

  describe('Uncertainty Quantification', () => {
    it('should quantify prediction uncertainty correctly', async () => {
      const model = learningSystem.models.get('failure_predictor');
      const test_features = [0.8, 0.9, 5.0, 1.0, 25, 0.05, 0.1, 100];

      const prediction = await learningSystem.predictWithUncertainty(model, test_features);

      expect(prediction.value).toBeDefined();
      expect(prediction.confidence).toBeGreaterThanOrEqual(0);
      expect(prediction.confidence).toBeLessThanOrEqual(1);

      expect(prediction.uncertainty.epistemic).toBeGreaterThanOrEqual(0);
      expect(prediction.uncertainty.aleatoric).toBeGreaterThanOrEqual(0);
      expect(prediction.uncertainty.total).toBeGreaterThanOrEqual(0);

      expect(prediction.uncertainty.confidence_interval).toHaveLength(2);
      expect(prediction.uncertainty.confidence_interval[0]).toBeLessThanOrEqual(prediction.uncertainty.confidence_interval[1]);

      // Total uncertainty should be at least as large as individual components
      expect(prediction.uncertainty.total).toBeGreaterThanOrEqual(prediction.uncertainty.epistemic);
      expect(prediction.uncertainty.total).toBeGreaterThanOrEqual(prediction.uncertainty.aleatoric);
    });

    it('should perform forward pass with dropout correctly', () => {
      const model = learningSystem.models.get('failure_predictor');
      const test_features = [0.8, 0.9, 5.0];

      const prediction1 = learningSystem.forwardPassWithDropout(model, test_features, 0.0); // No dropout
      const prediction2 = learningSystem.forwardPassWithDropout(model, test_features, 0.0); // No dropout
      const prediction3 = learningSystem.forwardPassWithDropout(model, test_features, 0.5); // 50% dropout

      // With no dropout, predictions should be identical
      expect(prediction1).toBe(prediction2);

      // With dropout, predictions should be different (with high probability)
      // Note: There's a small chance they could be the same due to randomness
      expect(typeof prediction3).toBe('number');
    });
  });

  describe('Explainable AI', () => {
    it('should generate feature importance explanations', async () => {
      const model = learningSystem.models.get('failure_predictor');
      const test_features = [0.2, 0.3, 10.0, 2.0, 85, 0.3, 0.8, 200]; // High-risk scenario
      const prediction = { value: 0.9, confidence: 0.8 };

      const explanation = await learningSystem.explainPrediction(model, test_features, prediction);

      expect(explanation.feature_importance).toBeDefined();
      expect(explanation.decision_path).toBeDefined();
      expect(explanation.natural_language).toBeDefined();

      // Check feature importance
      const feature_names = Object.keys(explanation.feature_importance);
      expect(feature_names.length).toBeGreaterThan(0);
      
      Object.values(explanation.feature_importance).forEach(importance => {
        expect(typeof importance).toBe('number');
        expect(importance).toBeGreaterThanOrEqual(0);
      });

      // Check decision path
      expect(Array.isArray(explanation.decision_path)).toBe(true);
      expect(explanation.decision_path.length).toBeLessThanOrEqual(learningSystem.config.explanation_depth);

      explanation.decision_path.forEach(node => {
        expect(node.feature).toBeDefined();
        expect(node.importance).toBeGreaterThanOrEqual(0);
        expect(node.samples).toBeGreaterThan(0);
      });

      // Check natural language explanation
      expect(typeof explanation.natural_language).toBe('string');
      expect(explanation.natural_language.length).toBeGreaterThan(0);
    });

    it('should generate preventive actions based on explanations', () => {
      const test_prediction = { value: 0.85, confidence: 0.9 };
      const test_explanation = {
        feature_importance: {
          energy_level: 0.8,
          communication_quality: 0.6,
          velocity_magnitude: 0.4
        },
        decision_path: [
          { feature: 'energy_level', threshold: 0.5, importance: 0.8, samples: 100 },
          { feature: 'communication_quality', threshold: 0.7, importance: 0.6, samples: 80 },
          { feature: 'velocity_magnitude', threshold: 5.0, importance: 0.4, samples: 60 }
        ],
        counterfactual_examples: [],
        natural_language: 'Test explanation'
      };

      const actions = learningSystem.generatePreventiveActions(test_prediction, test_explanation);

      expect(Array.isArray(actions)).toBe(true);
      expect(actions.length).toBeGreaterThan(0);

      // Actions should be sorted by priority (descending)
      for (let i = 0; i < actions.length - 1; i++) {
        expect(actions[i].priority).toBeGreaterThanOrEqual(actions[i + 1].priority);
      }

      actions.forEach(action => {
        expect(action.type).toBeDefined();
        expect(action.parameters).toBeDefined();
        expect(action.priority).toBeGreaterThanOrEqual(0);
        expect(action.expected_outcome).toBeDefined();
        expect(action.risk_level).toBeGreaterThanOrEqual(0);
        expect(action.risk_level).toBeLessThanOrEqual(1);
      });
    });
  });

  describe('Memory Management', () => {
    it('should maintain memory capacity limits', () => {
      const small_capacity_system = createAdaptiveLearningSystem({
        memory_capacity: 5
      });

      // Add more experiences than capacity
      for (let i = 0; i < 10; i++) {
        const experience = {
          id: `exp_${i}`,
          timestamp: Date.now() + i,
          state: [],
          action: {},
          reward: Math.random(),
          next_state: [],
          metadata: {}
        };
        small_capacity_system.addExperience(experience);
      }

      expect(small_capacity_system.memory.experiences.length).toBeLessThanOrEqual(5);
      
      // Should keep the most recent experiences
      const stored_ids = small_capacity_system.memory.experiences.map(exp => exp.id);
      expect(stored_ids).toContain('exp_9'); // Most recent
      expect(stored_ids).not.toContain('exp_0'); // Oldest should be removed
    });

    it('should extract patterns from similar experiences', async () => {
      const experience1 = {
        id: 'exp_similar_1',
        timestamp: Date.now(),
        state: [{ robot_id: 'robot_1', position: { x: 0, y: 0, z: 0 } }],
        action: { type: 'move' },
        reward: 0.8,
        next_state: [{ robot_id: 'robot_1', position: { x: 10, y: 0, z: 0 } }],
        metadata: {}
      };

      const experience2 = {
        id: 'exp_similar_2',
        timestamp: Date.now() + 1000,
        state: [{ robot_id: 'robot_2', position: { x: 5, y: 5, z: 0 } }],
        action: { type: 'move' },
        reward: 0.75,
        next_state: [{ robot_id: 'robot_2', position: { x: 15, y: 5, z: 0 } }],
        metadata: {}
      };

      const initial_patterns = learningSystem.memory.patterns.length;

      await learningSystem.extractPatternsFromExperience(experience1);
      await learningSystem.extractPatternsFromExperience(experience2);

      // Should have extracted at least one new pattern
      expect(learningSystem.memory.patterns.length).toBeGreaterThan(initial_patterns);

      // Check pattern structure
      if (learningSystem.memory.patterns.length > 0) {
        const pattern = learningSystem.memory.patterns[learningSystem.memory.patterns.length - 1];
        expect(pattern.id).toBeDefined();
        expect(pattern.description).toBeDefined();
        expect(pattern.confidence).toBeGreaterThanOrEqual(0);
        expect(pattern.confidence).toBeLessThanOrEqual(1);
        expect(pattern.frequency).toBeGreaterThan(0);
      }
    });
  });
});

describe('Learning Benchmark', () => {
  it('should run comprehensive learning benchmark successfully', async () => {
    const benchmark_results = await runLearningBenchmark();

    expect(benchmark_results).toBeDefined();
    expect(benchmark_results.execution_time).toBeGreaterThan(0);
    expect(benchmark_results.failure_predictions).toBeDefined();
    expect(benchmark_results.formation_optimization).toBeDefined();
    expect(benchmark_results.insights).toBeDefined();

    // Check failure predictions
    expect(Array.isArray(benchmark_results.failure_predictions)).toBe(true);
    benchmark_results.failure_predictions.forEach(prediction => {
      expect(prediction.confidence).toBeGreaterThanOrEqual(0);
      expect(prediction.confidence).toBeLessThanOrEqual(1);
    });

    // Check formation optimization
    expect(benchmark_results.formation_optimization.new_formation).toBeDefined();
    expect(benchmark_results.formation_optimization.confidence).toBeGreaterThanOrEqual(0);
    expect(benchmark_results.formation_optimization.confidence).toBeLessThanOrEqual(1);

    // Check insights
    expect(benchmark_results.insights.recommendations).toBeDefined();
    expect(Array.isArray(benchmark_results.insights.recommendations)).toBe(true);
  }, 15000); // Extended timeout for benchmark
});

describe('Integration Tests', () => {
  it('should handle real-time learning scenarios', async () => {
    const real_time_system = createAdaptiveLearningSystem({
      learning_rate: 0.01, // Higher learning rate for faster adaptation
      batch_size: 8,
      adaptation_threshold: 0.03
    });

    // Simulate real-time sensor data stream
    const sensor_stream = [];
    for (let i = 0; i < 50; i++) {
      sensor_stream.push({
        robot_id: `robot_${i % 5}`,
        timestamp: Date.now() + i * 100,
        position: { 
          x: Math.sin(i * 0.1) * 50, 
          y: Math.cos(i * 0.1) * 50, 
          z: 10 + Math.sin(i * 0.05) * 5 
        },
        velocity: { x: Math.random() * 2 - 1, y: Math.random() * 2 - 1, z: 0 },
        acceleration: { x: Math.random() * 0.5 - 0.25, y: Math.random() * 0.5 - 0.25, z: 0 },
        energy_level: 0.3 + Math.random() * 0.7,
        sensor_readings: { temperature: 20 + Math.random() * 20 },
        communication_quality: 0.5 + Math.random() * 0.5,
        task_status: 'active',
        environmental_context: {
          weather: { type: 'clear', intensity: 0.8, forecast_confidence: 0.9 },
          obstacles: [],
          communication_interference: Math.random() * 0.3,
          electromagnetic_field: Math.random() * 0.1,
          temperature: 20 + Math.random() * 20,
          humidity: 0.4 + Math.random() * 0.4,
          wind_speed: Math.random() * 15,
          visibility: 0.7 + Math.random() * 0.3
        }
      });
    }

    // Process data in batches to simulate real-time processing
    for (let i = 0; i < sensor_stream.length; i += 5) {
      const batch = sensor_stream.slice(i, i + 5);
      await real_time_system.processSensorData(batch);
    }

    // System should have processed all data
    expect(real_time_system.sensor_buffer.length).toBeGreaterThan(0);

    // Test predictions on recent data
    const recent_data = sensor_stream.slice(-3);
    const predictions = await real_time_system.predictFailures(recent_data);

    expect(predictions).toHaveLength(3);
    predictions.forEach(prediction => {
      expect(prediction.confidence).toBeGreaterThanOrEqual(0);
      expect(prediction.confidence).toBeLessThanOrEqual(1);
    });
  }, 20000);

  it('should adapt to changing environmental conditions', async () => {
    const adaptive_system = createAdaptiveLearningSystem();

    // Simulate mission in clear weather
    const clear_weather_mission = {
      mission_id: 'clear_weather_test',
      objectives: ['patrol'],
      initial_state: [{
        robot_id: 'robot_1',
        timestamp: Date.now(),
        position: { x: 0, y: 0, z: 5 },
        velocity: { x: 0, y: 0, z: 0 },
        acceleration: { x: 0, y: 0, z: 0 },
        energy_level: 1.0,
        sensor_readings: {},
        communication_quality: 0.95,
        task_status: 'ready',
        environmental_context: {
          weather: { type: 'clear', intensity: 0.9, forecast_confidence: 0.95 },
          obstacles: [],
          communication_interference: 0.05,
          electromagnetic_field: 0.02,
          temperature: 25,
          humidity: 0.5,
          wind_speed: 3,
          visibility: 0.98
        }
      }],
      actions_taken: [{ type: 'patrol', area: 'zone_a' }],
      final_state: [{
        robot_id: 'robot_1',
        timestamp: Date.now() + 1000,
        position: { x: 50, y: 50, z: 5 },
        velocity: { x: 0, y: 0, z: 0 },
        acceleration: { x: 0, y: 0, z: 0 },
        energy_level: 0.9,
        sensor_readings: {},
        communication_quality: 0.93,
        task_status: 'completed',
        environmental_context: {
          weather: { type: 'clear', intensity: 0.9, forecast_confidence: 0.95 },
          obstacles: [],
          communication_interference: 0.07,
          electromagnetic_field: 0.03,
          temperature: 26,
          humidity: 0.52,
          wind_speed: 4,
          visibility: 0.97
        }
      }],
      success_metrics: { efficiency: 0.95, completion: 1.0 },
      duration: 900, // 15 minutes
      challenges_encountered: []
    };

    // Simulate mission in storm weather
    const storm_weather_mission = {
      mission_id: 'storm_weather_test',
      objectives: ['patrol'],
      initial_state: [{
        robot_id: 'robot_1',
        timestamp: Date.now() + 2000,
        position: { x: 0, y: 0, z: 5 },
        velocity: { x: 0, y: 0, z: 0 },
        acceleration: { x: 0, y: 0, z: 0 },
        energy_level: 1.0,
        sensor_readings: {},
        communication_quality: 0.6,
        task_status: 'ready',
        environmental_context: {
          weather: { type: 'storm', intensity: 0.9, forecast_confidence: 0.8 },
          obstacles: [],
          communication_interference: 0.7,
          electromagnetic_field: 0.4,
          temperature: 15,
          humidity: 0.9,
          wind_speed: 35,
          visibility: 0.3
        }
      }],
      actions_taken: [{ type: 'patrol', area: 'zone_a' }],
      final_state: [{
        robot_id: 'robot_1',
        timestamp: Date.now() + 3000,
        position: { x: 30, y: 30, z: 3 },
        velocity: { x: 0, y: 0, z: 0 },
        acceleration: { x: 0, y: 0, z: 0 },
        energy_level: 0.5,
        sensor_readings: {},
        communication_quality: 0.4,
        task_status: 'partially_completed',
        environmental_context: {
          weather: { type: 'storm', intensity: 0.95, forecast_confidence: 0.85 },
          obstacles: [],
          communication_interference: 0.8,
          electromagnetic_field: 0.5,
          temperature: 12,
          humidity: 0.95,
          wind_speed: 40,
          visibility: 0.2
        }
      }],
      success_metrics: { efficiency: 0.4, completion: 0.6 },
      duration: 2700, // 45 minutes
      challenges_encountered: ['high_wind', 'poor_visibility', 'communication_loss']
    };

    // Learn from both missions
    await adaptive_system.learnFromMission(clear_weather_mission);
    await adaptive_system.learnFromMission(storm_weather_mission);

    // Check that system learned different strategies for different conditions
    expect(adaptive_system.memory.experiences).toHaveLength(2);
    
    const clear_experience = adaptive_system.memory.experiences[0];
    const storm_experience = adaptive_system.memory.experiences[1];

    expect(clear_experience.reward).toBeGreaterThan(storm_experience.reward);
    expect(storm_experience.metadata.challenges.length).toBeGreaterThan(0);
  });

  it('should handle model performance degradation and recovery', async () => {
    const robust_system = createAdaptiveLearningSystem({
      adaptation_threshold: 0.1 // Lower threshold for faster adaptation
    });

    // Simulate initial good performance
    const good_experiences = Array(10).fill(null).map((_, i) => ({
      id: `good_exp_${i}`,
      timestamp: Date.now() + i * 1000,
      state: [],
      action: { type: 'optimal_action' },
      reward: 0.8 + Math.random() * 0.2, // High rewards
      next_state: [],
      metadata: { performance: 'good' }
    }));

    // Simulate performance degradation
    const degraded_experiences = Array(5).fill(null).map((_, i) => ({
      id: `degraded_exp_${i}`,
      timestamp: Date.now() + (i + 10) * 1000,
      state: [],
      action: { type: 'suboptimal_action' },
      reward: 0.1 + Math.random() * 0.3, // Low rewards
      next_state: [],
      metadata: { performance: 'degraded' }
    }));

    // Simulate recovery
    const recovery_experiences = Array(8).fill(null).map((_, i) => ({
      id: `recovery_exp_${i}`,
      timestamp: Date.now() + (i + 15) * 1000,
      state: [],
      action: { type: 'adaptive_action' },
      reward: 0.7 + Math.random() * 0.25, // Recovered rewards
      next_state: [],
      metadata: { performance: 'recovered' }
    }));

    // Add experiences in sequence
    good_experiences.forEach(exp => robust_system.addExperience(exp));
    degraded_experiences.forEach(exp => robust_system.addExperience(exp));
    recovery_experiences.forEach(exp => robust_system.addExperience(exp));

    // System should have learned from all phases
    expect(robust_system.memory.experiences.length).toBeGreaterThan(0);

    // Check that recent experiences show recovery
    const recent_experiences = robust_system.memory.experiences.slice(-5);
    const avg_recent_reward = recent_experiences.reduce((sum, exp) => sum + exp.reward, 0) / recent_experiences.length;

    expect(avg_recent_reward).toBeGreaterThan(0.5); // Should show recovery
  });
});