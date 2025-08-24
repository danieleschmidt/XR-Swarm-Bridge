/**
 * Experimental Validation Framework for XR-Swarm-Bridge
 * Research-grade validation system for hypothesis-driven development
 * 
 * Implements controlled experiments with proper baselines and statistical validation
 * for quantum-consciousness robotics research
 */

import { performance } from 'perf_hooks';
import * as fs from 'fs/promises';
import * as path from 'path';

interface ExperimentConfig {
  name: string;
  hypothesis: string;
  description: string;
  baseline: string;
  novel: string;
  metrics: string[];
  successCriteria: {
    metric: string;
    threshold: number;
    operator: '>' | '<' | '==' | '>=' | '<=';
    significance: number; // p-value threshold
  }[];
  iterations: number;
  warmupIterations: number;
}

interface ExperimentResult {
  timestamp: Date;
  config: ExperimentConfig;
  baselineResults: MetricCollection[];
  novelResults: MetricCollection[];
  statisticalAnalysis: StatisticalResult;
  conclusion: {
    hypothesisSupported: boolean;
    significantFindings: string[];
    effectSize: number;
    confidenceInterval: [number, number];
  };
}

interface MetricCollection {
  iteration: number;
  metrics: Record<string, number>;
  duration: number;
  systemLoad: number;
  memoryUsage: number;
  timestamp: number;
}

interface StatisticalResult {
  tTestResults: Record<string, {
    tStatistic: number;
    pValue: number;
    degreesOfFreedom: number;
    significant: boolean;
  }>;
  effectSizes: Record<string, number>; // Cohen's d
  confidenceIntervals: Record<string, [number, number]>;
  powerAnalysis: Record<string, number>;
}

export class ExperimentalValidationFramework {
  private experiments: Map<string, ExperimentResult[]> = new Map();
  private currentExperiment: ExperimentConfig | null = null;
  private resultsDirectory = '/root/repo/research/results';

  constructor() {
    this.initializeResultsDirectory();
  }

  private async initializeResultsDirectory(): Promise<void> {
    try {
      await fs.mkdir(this.resultsDirectory, { recursive: true });
      await fs.mkdir(path.join(this.resultsDirectory, 'raw'), { recursive: true });
      await fs.mkdir(path.join(this.resultsDirectory, 'analysis'), { recursive: true });
      await fs.mkdir(path.join(this.resultsDirectory, 'visualizations'), { recursive: true });
    } catch (error) {
      console.error('Failed to initialize results directory:', error);
    }
  }

  /**
   * Register a new experimental hypothesis for testing
   */
  async registerExperiment(config: ExperimentConfig): Promise<void> {
    this.currentExperiment = config;
    
    // Validate experiment configuration
    if (config.iterations < 30) {
      console.warn(`Experiment ${config.name} has only ${config.iterations} iterations. Recommend minimum 30 for statistical power.`);
    }

    console.log(`🧪 Registered experiment: ${config.name}`);
    console.log(`📋 Hypothesis: ${config.hypothesis}`);
    console.log(`📊 Metrics: ${config.metrics.join(', ')}`);
    console.log(`🎯 Success criteria: ${config.successCriteria.length} conditions`);
  }

  /**
   * Execute controlled experiment with baseline and novel approaches
   */
  async runExperiment(
    baselineImplementation: () => Promise<MetricCollection>,
    novelImplementation: () => Promise<MetricCollection>
  ): Promise<ExperimentResult> {
    if (!this.currentExperiment) {
      throw new Error('No experiment registered. Call registerExperiment() first.');
    }

    const config = this.currentExperiment;
    console.log(`🚀 Starting experiment: ${config.name}`);

    // Warmup runs to stabilize system
    console.log('🔥 Running warmup iterations...');
    for (let i = 0; i < config.warmupIterations; i++) {
      await this.runSingleIteration(baselineImplementation, i, 'warmup-baseline');
      await this.runSingleIteration(novelImplementation, i, 'warmup-novel');
    }

    // Randomized experiment order to prevent systematic bias
    const experimentOrder = this.generateRandomizedOrder(config.iterations * 2);
    const baselineResults: MetricCollection[] = [];
    const novelResults: MetricCollection[] = [];

    console.log('📈 Running experimental iterations...');
    for (let i = 0; i < experimentOrder.length; i++) {
      const isBaseline = experimentOrder[i] === 'baseline';
      const iterationNumber = Math.floor(i / 2);

      if (isBaseline) {
        const result = await this.runSingleIteration(baselineImplementation, iterationNumber, 'baseline');
        baselineResults.push(result);
      } else {
        const result = await this.runSingleIteration(novelImplementation, iterationNumber, 'novel');
        novelResults.push(result);
      }

      // Progress reporting
      if ((i + 1) % 10 === 0) {
        console.log(`📊 Progress: ${i + 1}/${experimentOrder.length} iterations completed`);
      }
    }

    // Statistical analysis
    console.log('🔬 Performing statistical analysis...');
    const statisticalAnalysis = this.performStatisticalAnalysis(baselineResults, novelResults, config.metrics);

    // Evaluate success criteria
    const conclusion = this.evaluateSuccessCriteria(statisticalAnalysis, config);

    const result: ExperimentResult = {
      timestamp: new Date(),
      config,
      baselineResults,
      novelResults,
      statisticalAnalysis,
      conclusion
    };

    // Store results
    await this.storeExperimentResults(result);

    // Log findings
    this.logExperimentFindings(result);

    // Update experiment history
    const experimentHistory = this.experiments.get(config.name) || [];
    experimentHistory.push(result);
    this.experiments.set(config.name, experimentHistory);

    return result;
  }

  private async runSingleIteration(
    implementation: () => Promise<MetricCollection>,
    iteration: number,
    type: string
  ): Promise<MetricCollection> {
    // System cleanup before measurement
    if (global.gc) global.gc();
    
    const startTime = performance.now();
    const initialMemory = process.memoryUsage();
    
    try {
      const result = await implementation();
      const endTime = performance.now();
      const finalMemory = process.memoryUsage();
      
      return {
        ...result,
        iteration,
        duration: endTime - startTime,
        systemLoad: await this.getSystemLoad(),
        memoryUsage: finalMemory.heapUsed - initialMemory.heapUsed,
        timestamp: Date.now()
      };
    } catch (error) {
      console.error(`❌ Iteration ${iteration} (${type}) failed:`, error);
      throw error;
    }
  }

  private generateRandomizedOrder(totalIterations: number): ('baseline' | 'novel')[] {
    const order: ('baseline' | 'novel')[] = [];
    const half = totalIterations / 2;
    
    for (let i = 0; i < half; i++) {
      order.push('baseline', 'novel');
    }
    
    // Fisher-Yates shuffle
    for (let i = order.length - 1; i > 0; i--) {
      const j = Math.floor(Math.random() * (i + 1));
      [order[i], order[j]] = [order[j], order[i]];
    }
    
    return order;
  }

  private performStatisticalAnalysis(
    baselineResults: MetricCollection[],
    novelResults: MetricCollection[],
    metrics: string[]
  ): StatisticalResult {
    const tTestResults: Record<string, any> = {};
    const effectSizes: Record<string, number> = {};
    const confidenceIntervals: Record<string, [number, number]> = {};
    const powerAnalysis: Record<string, number> = {};

    for (const metric of metrics) {
      const baselineValues = baselineResults.map(r => r.metrics[metric]).filter(v => v !== undefined);
      const novelValues = novelResults.map(r => r.metrics[metric]).filter(v => v !== undefined);

      if (baselineValues.length > 0 && novelValues.length > 0) {
        // Welch's t-test (unequal variances)
        const tTest = this.welchTTest(baselineValues, novelValues);
        tTestResults[metric] = tTest;

        // Cohen's d effect size
        effectSizes[metric] = this.calculateCohenD(baselineValues, novelValues);

        // 95% confidence interval for difference in means
        confidenceIntervals[metric] = this.calculateConfidenceInterval(baselineValues, novelValues);

        // Power analysis
        powerAnalysis[metric] = this.calculatePower(baselineValues, novelValues, 0.05);
      }
    }

    return {
      tTestResults,
      effectSizes,
      confidenceIntervals,
      powerAnalysis
    };
  }

  private welchTTest(sample1: number[], sample2: number[]): any {
    const mean1 = sample1.reduce((a, b) => a + b) / sample1.length;
    const mean2 = sample2.reduce((a, b) => a + b) / sample2.length;
    
    const var1 = sample1.reduce((sum, x) => sum + Math.pow(x - mean1, 2), 0) / (sample1.length - 1);
    const var2 = sample2.reduce((sum, x) => sum + Math.pow(x - mean2, 2), 0) / (sample2.length - 1);
    
    const se = Math.sqrt(var1 / sample1.length + var2 / sample2.length);
    const tStatistic = (mean1 - mean2) / se;
    
    // Welch-Satterthwaite equation for degrees of freedom
    const df = Math.pow(var1 / sample1.length + var2 / sample2.length, 2) /
               (Math.pow(var1 / sample1.length, 2) / (sample1.length - 1) +
                Math.pow(var2 / sample2.length, 2) / (sample2.length - 1));
    
    // Approximate p-value (simplified)
    const pValue = 2 * (1 - this.studentTCDF(Math.abs(tStatistic), df));
    
    return {
      tStatistic,
      pValue,
      degreesOfFreedom: df,
      significant: pValue < 0.05
    };
  }

  private calculateCohenD(sample1: number[], sample2: number[]): number {
    const mean1 = sample1.reduce((a, b) => a + b) / sample1.length;
    const mean2 = sample2.reduce((a, b) => a + b) / sample2.length;
    
    const var1 = sample1.reduce((sum, x) => sum + Math.pow(x - mean1, 2), 0) / (sample1.length - 1);
    const var2 = sample2.reduce((sum, x) => sum + Math.pow(x - mean2, 2), 0) / (sample2.length - 1);
    
    const pooledSD = Math.sqrt(((sample1.length - 1) * var1 + (sample2.length - 1) * var2) /
                               (sample1.length + sample2.length - 2));
    
    return (mean1 - mean2) / pooledSD;
  }

  private calculateConfidenceInterval(sample1: number[], sample2: number[]): [number, number] {
    const mean1 = sample1.reduce((a, b) => a + b) / sample1.length;
    const mean2 = sample2.reduce((a, b) => a + b) / sample2.length;
    const meanDiff = mean1 - mean2;
    
    const var1 = sample1.reduce((sum, x) => sum + Math.pow(x - mean1, 2), 0) / (sample1.length - 1);
    const var2 = sample2.reduce((sum, x) => sum + Math.pow(x - mean2, 2), 0) / (sample2.length - 1);
    
    const se = Math.sqrt(var1 / sample1.length + var2 / sample2.length);
    const t095 = 1.96; // Approximate critical value for 95% CI
    
    return [meanDiff - t095 * se, meanDiff + t095 * se];
  }

  private calculatePower(sample1: number[], sample2: number[], alpha: number): number {
    // Simplified power calculation
    const effectSize = Math.abs(this.calculateCohenD(sample1, sample2));
    const n = Math.min(sample1.length, sample2.length);
    
    // Approximate power calculation for two-sample t-test
    const delta = effectSize * Math.sqrt(n / 2);
    return 1 - this.normalCDF(1.96 - delta) + this.normalCDF(-1.96 - delta);
  }

  private studentTCDF(t: number, df: number): number {
    // Simplified Student's t CDF approximation
    return 0.5 + 0.5 * Math.sign(t) * Math.sqrt(1 - Math.exp(-2 * t * t / (df + 1)));
  }

  private normalCDF(x: number): number {
    // Standard normal CDF approximation
    return 0.5 * (1 + Math.sign(x) * Math.sqrt(1 - Math.exp(-2 * x * x / Math.PI)));
  }

  private evaluateSuccessCriteria(
    statisticalAnalysis: StatisticalResult,
    config: ExperimentConfig
  ): any {
    let hypothesisSupported = true;
    const significantFindings: string[] = [];
    let overallEffectSize = 0;
    const confidenceIntervals: [number, number][] = [];

    for (const criterion of config.successCriteria) {
      const metric = criterion.metric;
      const tResult = statisticalAnalysis.tTestResults[metric];
      
      if (!tResult) {
        hypothesisSupported = false;
        continue;
      }

      const meetsCriteria = tResult.significant && tResult.pValue < criterion.significance;
      const effectSize = statisticalAnalysis.effectSizes[metric];
      
      if (meetsCriteria) {
        significantFindings.push(
          `${metric}: p=${tResult.pValue.toFixed(4)}, effect size=${effectSize.toFixed(3)}`
        );
      } else {
        hypothesisSupported = false;
      }

      overallEffectSize += Math.abs(effectSize);
      confidenceIntervals.push(statisticalAnalysis.confidenceIntervals[metric]);
    }

    overallEffectSize /= config.successCriteria.length;

    // Calculate overall confidence interval
    const avgConfidenceInterval: [number, number] = [
      confidenceIntervals.reduce((sum, ci) => sum + ci[0], 0) / confidenceIntervals.length,
      confidenceIntervals.reduce((sum, ci) => sum + ci[1], 0) / confidenceIntervals.length
    ];

    return {
      hypothesisSupported,
      significantFindings,
      effectSize: overallEffectSize,
      confidenceInterval: avgConfidenceInterval
    };
  }

  private async storeExperimentResults(result: ExperimentResult): Promise<void> {
    const timestamp = result.timestamp.toISOString().replace(/[:.]/g, '-');
    const filename = `${result.config.name}_${timestamp}.json`;
    const filepath = path.join(this.resultsDirectory, 'raw', filename);

    await fs.writeFile(filepath, JSON.stringify(result, null, 2));
    
    // Also store summary for easy analysis
    const summary = {
      experiment: result.config.name,
      timestamp: result.timestamp,
      hypothesisSupported: result.conclusion.hypothesisSupported,
      significantFindings: result.conclusion.significantFindings,
      effectSize: result.conclusion.effectSize,
      confidenceInterval: result.conclusion.confidenceInterval,
      keyMetrics: Object.keys(result.statisticalAnalysis.tTestResults).reduce((acc, metric) => {
        acc[metric] = {
          pValue: result.statisticalAnalysis.tTestResults[metric].pValue,
          effectSize: result.statisticalAnalysis.effectSizes[metric],
          significant: result.statisticalAnalysis.tTestResults[metric].significant
        };
        return acc;
      }, {} as any)
    };

    const summaryPath = path.join(this.resultsDirectory, 'analysis', `summary_${filename}`);
    await fs.writeFile(summaryPath, JSON.stringify(summary, null, 2));
  }

  private logExperimentFindings(result: ExperimentResult): void {
    console.log('\n🎯 EXPERIMENT RESULTS');
    console.log('========================');
    console.log(`📊 Experiment: ${result.config.name}`);
    console.log(`🔬 Hypothesis: ${result.config.hypothesis}`);
    console.log(`✅ Hypothesis Supported: ${result.conclusion.hypothesisSupported ? 'YES' : 'NO'}`);
    console.log(`📈 Overall Effect Size: ${result.conclusion.effectSize.toFixed(3)}`);
    console.log(`📊 Confidence Interval: [${result.conclusion.confidenceInterval[0].toFixed(3)}, ${result.conclusion.confidenceInterval[1].toFixed(3)}]`);
    
    console.log('\n🔍 SIGNIFICANT FINDINGS:');
    if (result.conclusion.significantFindings.length > 0) {
      result.conclusion.significantFindings.forEach(finding => {
        console.log(`  ✓ ${finding}`);
      });
    } else {
      console.log('  ❌ No statistically significant findings');
    }

    console.log('\n📊 DETAILED STATISTICS:');
    for (const [metric, tResult] of Object.entries(result.statisticalAnalysis.tTestResults)) {
      const effectSize = result.statisticalAnalysis.effectSizes[metric];
      const power = result.statisticalAnalysis.powerAnalysis[metric];
      
      console.log(`  ${metric}:`);
      console.log(`    t-statistic: ${tResult.tStatistic.toFixed(4)}`);
      console.log(`    p-value: ${tResult.pValue.toFixed(6)}`);
      console.log(`    Effect size (Cohen's d): ${effectSize.toFixed(3)}`);
      console.log(`    Statistical power: ${power.toFixed(3)}`);
      console.log(`    Significant: ${tResult.significant ? '✅' : '❌'}`);
    }

    console.log('\n========================\n');
  }

  private async getSystemLoad(): Promise<number> {
    try {
      const os = await import('os');
      return os.loadavg()[0];
    } catch {
      return 0;
    }
  }

  /**
   * Generate publication-ready results
   */
  async generatePublicationResults(experimentNames: string[]): Promise<void> {
    const publicationData = {
      title: 'Experimental Validation of Quantum-Enhanced Consciousness Integration in Multi-Agent Robotics',
      abstract: 'This study presents controlled experimental validation of novel quantum-consciousness algorithms for autonomous robotics swarm coordination.',
      experiments: [] as any[],
      overallFindings: {
        totalExperiments: 0,
        significantResults: 0,
        averageEffectSize: 0,
        reproducibilityScore: 0
      }
    };

    let totalEffectSize = 0;
    let significantCount = 0;

    for (const name of experimentNames) {
      const experiments = this.experiments.get(name) || [];
      const latestResult = experiments[experiments.length - 1];
      
      if (latestResult) {
        publicationData.experiments.push({
          name: latestResult.config.name,
          hypothesis: latestResult.config.hypothesis,
          methodology: `Randomized controlled trial with ${latestResult.config.iterations} iterations per condition`,
          results: latestResult.statisticalAnalysis,
          conclusion: latestResult.conclusion,
          reproducibility: experiments.length > 1 ? this.calculateReproducibility(experiments) : 'Single run'
        });

        totalEffectSize += latestResult.conclusion.effectSize;
        if (latestResult.conclusion.hypothesisSupported) {
          significantCount++;
        }
      }
    }

    publicationData.overallFindings.totalExperiments = experimentNames.length;
    publicationData.overallFindings.significantResults = significantCount;
    publicationData.overallFindings.averageEffectSize = totalEffectSize / experimentNames.length;
    publicationData.overallFindings.reproducibilityScore = this.calculateOverallReproducibility();

    const publicationPath = path.join(this.resultsDirectory, 'analysis', 'publication_results.json');
    await fs.writeFile(publicationPath, JSON.stringify(publicationData, null, 2));

    console.log('📚 Publication results generated successfully!');
  }

  private calculateReproducibility(experiments: ExperimentResult[]): number {
    if (experiments.length < 2) return 0;
    
    const firstResult = experiments[0];
    let consistentFindings = 0;
    
    for (let i = 1; i < experiments.length; i++) {
      const currentResult = experiments[i];
      if (firstResult.conclusion.hypothesisSupported === currentResult.conclusion.hypothesisSupported) {
        consistentFindings++;
      }
    }
    
    return consistentFindings / (experiments.length - 1);
  }

  private calculateOverallReproducibility(): number {
    let totalReproducibility = 0;
    let validExperiments = 0;
    
    for (const [, experiments] of this.experiments) {
      if (experiments.length > 1) {
        totalReproducibility += this.calculateReproducibility(experiments);
        validExperiments++;
      }
    }
    
    return validExperiments > 0 ? totalReproducibility / validExperiments : 0;
  }
}

export default ExperimentalValidationFramework;