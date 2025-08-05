import { describe, it, expect, beforeEach, vi } from 'vitest'
import { gptIntegration, GPTIntegration } from '../ai/gptIntegration'
import type { GPTRequest, MissionPlan, SwarmCommand } from '../ai/gptIntegration'
import axios from 'axios'

// Mock axios
vi.mock('axios')
const mockedAxios = vi.mocked(axios)

describe('GPTIntegration', () => {
  let gptInstance: GPTIntegration

  beforeEach(() => {
    vi.clearAllMocks()
    // Create instance with test API key
    gptInstance = new (GPTIntegration as any)('test-api-key')
  })

  describe('Constructor', () => {
    it('should initialize with API key from parameter', () => {
      const instance = new (GPTIntegration as any)('test-key')
      expect(instance.apiKey).toBe('test-key')
    })

    it('should initialize with API key from environment', () => {
      const originalEnv = process.env.REACT_APP_OPENAI_API_KEY
      process.env.REACT_APP_OPENAI_API_KEY = 'env-key'
      
      const instance = new (GPTIntegration as any)()
      expect(instance.apiKey).toBe('env-key')
      
      process.env.REACT_APP_OPENAI_API_KEY = originalEnv
    })

    it('should handle missing API key gracefully', () => {
      const originalEnv = process.env.REACT_APP_OPENAI_API_KEY
      delete process.env.REACT_APP_OPENAI_API_KEY
      
      const instance = new (GPTIntegration as any)()
      expect(instance.apiKey).toBe('')
      
      process.env.REACT_APP_OPENAI_API_KEY = originalEnv
    })
  })

  describe('generateMissionPlan', () => {
    const mockContext = {
      agents: [
        { id: 'drone_01', type: 'drone', status: 'active' },
        { id: 'ugv_01', type: 'ugv', status: 'active' }
      ]
    }

    it('should generate mission plan successfully', async () => {
      const mockPlan: MissionPlan = {
        id: 'mission_123',
        name: 'Search and Rescue',
        description: 'Search for missing person in forest area',
        estimatedDuration: 3600,
        phases: [
          {
            phase: 1,
            description: 'Initial reconnaissance',
            duration: 1800,
            assignments: {
              'drone_01': 'Aerial survey of search area',
              'ugv_01': 'Establish base camp'
            }
          }
        ],
        contingencies: [
          {
            trigger: 'Weather deterioration',
            action: 'Return to base immediately',
            priority: 'high'
          }
        ],
        successCriteria: ['Target located', 'All agents returned safely'],
        resources: {
          requiredAgents: 2,
          minBatteryLevel: 70,
          requiredCapabilities: ['navigate', 'camera']
        }
      }

      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: JSON.stringify(mockPlan)
              }
            }
          ]
        }
      })

      const result = await gptInstance.generateMissionPlan(
        'Search for missing person in forest',
        mockContext
      )

      expect(result.error).toBeUndefined()
      expect(result.plan).toEqual(mockPlan)
      expect(mockedAxios.post).toHaveBeenCalledWith(
        'https://api.openai.com/v1/chat/completions',
        expect.objectContaining({
          model: 'gpt-4o',
          messages: expect.arrayContaining([
            expect.objectContaining({ role: 'system' }),
            expect.objectContaining({ role: 'user' })
          ]),
          response_format: { type: "json_object" }
        }),
        expect.objectContaining({
          headers: expect.objectContaining({
            'Authorization': 'Bearer test-api-key'
          })
        })
      )
    })

    it('should handle API error gracefully', async () => {
      mockedAxios.post.mockRejectedValueOnce(new Error('API request failed'))

      const result = await gptInstance.generateMissionPlan(
        'Test objective',
        mockContext
      )

      expect(result.error).toBe('API request failed')
      expect(result.plan).toBeUndefined()
    })

    it('should handle missing API key', async () => {
      const instanceWithoutKey = new (GPTIntegration as any)('')

      const result = await instanceWithoutKey.generateMissionPlan(
        'Test objective',
        mockContext
      )

      expect(result.error).toBe('OpenAI API key not configured')
      expect(result.plan).toBeUndefined()
    })

    it('should handle empty response', async () => {
      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: null
              }
            }
          ]
        }
      })

      const result = await gptInstance.generateMissionPlan(
        'Test objective',
        mockContext
      )

      expect(result.error).toBe('Empty response from GPT-4o')
    })

    it('should handle invalid JSON response', async () => {
      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: 'Invalid JSON response'
              }
            }
          ]
        }
      })

      const result = await gptInstance.generateMissionPlan(
        'Test objective',
        mockContext
      )

      expect(result.error).toContain('Unexpected token')
    })
  })

  describe('generateSwarmCommands', () => {
    const mockContext = {
      agents: [
        { id: 'drone_01', type: 'drone', selected: true },
        { id: 'drone_02', type: 'drone', selected: false }
      ]
    }

    it('should generate swarm commands successfully', async () => {
      const mockCommands: SwarmCommand[] = [
        {
          type: 'formation',
          target: 'selected',
          parameters: { formation: 'circle', radius: 10 },
          priority: 'medium'
        }
      ]

      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: JSON.stringify({ commands: mockCommands })
              }
            }
          ]
        }
      })

      const result = await gptInstance.generateSwarmCommands(
        'Form a circle formation',
        mockContext
      )

      expect(result.error).toBeUndefined()
      expect(result.commands).toEqual(mockCommands)
    })

    it('should handle complex natural language commands', async () => {
      const complexCommand = 'Have the drones search the area in a grid pattern while the ground vehicles establish a perimeter'
      
      const mockCommands: SwarmCommand[] = [
        {
          type: 'search',
          target: ['drone_01', 'drone_02'],
          parameters: { pattern: 'grid', area: { width: 100, height: 100 } },
          priority: 'high'
        },
        {
          type: 'formation',
          target: ['ugv_01', 'ugv_02'],
          parameters: { formation: 'perimeter' },
          priority: 'medium'
        }
      ]

      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: JSON.stringify({ commands: mockCommands })
              }
            }
          ]
        }
      })

      const result = await gptInstance.generateSwarmCommands(complexCommand, mockContext)

      expect(result.commands).toHaveLength(2)
      expect(result.commands?.[0].type).toBe('search')
      expect(result.commands?.[1].type).toBe('formation')
    })
  })

  describe('analyzeSwarmPerformance', () => {
    const mockMetrics = {
      totalAgents: 5,
      activeAgents: 4,
      averageBattery: 75,
      networkLatency: 150,
      packetLoss: 0.02,
      completedMissions: 3,
      failedMissions: 1
    }

    it('should analyze performance successfully', async () => {
      const mockAnalysis = 'The swarm is performing well with 80% operational capacity. Consider replacing agent_05 due to low battery levels.'

      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: mockAnalysis
              }
            }
          ]
        }
      })

      const result = await gptInstance.analyzeSwarmPerformance(mockMetrics)

      expect(result.error).toBeUndefined()
      expect(result.analysis).toBe(mockAnalysis)
    })

    it('should provide actionable insights', async () => {
      const detailedAnalysis = `
Performance Analysis:
- Network latency (150ms) is within acceptable range but could be optimized
- Packet loss (2%) indicates potential connectivity issues
- Average battery level (75%) is good but monitor closely
- Mission success rate (75%) suggests room for improvement

Recommendations:
1. Investigate connectivity issues for packet loss reduction
2. Implement predictive battery management
3. Review failed mission patterns for optimization opportunities
      `.trim()

      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: detailedAnalysis
              }
            }
          ]
        }
      })

      const result = await gptInstance.analyzeSwarmPerformance(mockMetrics)

      expect(result.analysis).toContain('Performance Analysis')
      expect(result.analysis).toContain('Recommendations')
    })
  })

  describe('optimizeFormation', () => {
    const mockAgents = [
      { id: 'drone_01', type: 'drone', position: [0, 0, 5] },
      { id: 'drone_02', type: 'drone', position: [10, 0, 5] },
      { id: 'ugv_01', type: 'ugv', position: [5, 0, 0] }
    ]

    it('should optimize formation successfully', async () => {
      const mockCommands: SwarmCommand[] = [
        {
          type: 'formation',
          target: 'all',
          parameters: {
            formation: 'optimized_search',
            positions: [
              { agentId: 'drone_01', position: [0, 0, 8] },
              { agentId: 'drone_02', position: [20, 0, 8] },
              { agentId: 'ugv_01', position: [10, 0, 0] }
            ]
          },
          priority: 'high'
        }
      ]

      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: JSON.stringify({ commands: mockCommands })
              }
            }
          ]
        }
      })

      const result = await gptInstance.optimizeFormation(
        mockAgents,
        'maximize search coverage'
      )

      expect(result.error).toBeUndefined()
      expect(result.commands).toEqual(mockCommands)
    })
  })

  describe('Error Handling', () => {
    it('should handle network timeout', async () => {
      mockedAxios.post.mockRejectedValueOnce({
        code: 'ECONNABORTED',
        message: 'timeout of 30000ms exceeded'
      })

      const result = await gptInstance.generateMissionPlan('test', {})

      expect(result.error).toContain('timeout')
    })

    it('should handle API rate limiting', async () => {
      mockedAxios.post.mockRejectedValueOnce({
        response: {
          status: 429,
          data: {
            error: {
              message: 'Rate limit exceeded'
            }
          }
        }
      })

      const result = await gptInstance.generateMissionPlan('test', {})

      expect(result.error).toBe('Rate limit exceeded')
    })

    it('should handle authentication errors', async () => {
      mockedAxios.post.mockRejectedValueOnce({
        response: {
          status: 401,
          data: {
            error: {
              message: 'Invalid API key'
            }
          }
        }
      })

      const result = await gptInstance.generateMissionPlan('test', {})

      expect(result.error).toBe('Invalid API key')
    })

    it('should handle server errors gracefully', async () => {
      mockedAxios.post.mockRejectedValueOnce({
        response: {
          status: 500,
          data: {
            error: {
              message: 'Internal server error'
            }
          }
        }
      })

      const result = await gptInstance.generateMissionPlan('test', {})

      expect(result.error).toBe('Internal server error')
    })
  })

  describe('Input Validation', () => {
    it('should handle empty objectives', async () => {
      const result = await gptInstance.generateMissionPlan('', {})

      expect(mockedAxios.post).toHaveBeenCalled()
      // Should still make request but with empty objective
    })

    it('should handle null context', async () => {
      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: JSON.stringify({
                  id: 'test',
                  name: 'Test Mission',
                  description: 'Test',
                  estimatedDuration: 3600,
                  phases: [],
                  contingencies: [],
                  successCriteria: [],
                  resources: { requiredAgents: 0, minBatteryLevel: 0, requiredCapabilities: [] }
                })
              }
            }
          ]
        }
      })

      const result = await gptInstance.generateMissionPlan('test objective', null as any)

      expect(result.error).toBeUndefined()
      expect(result.plan).toBeDefined()
    })

    it('should handle very long objectives', async () => {
      const longObjective = 'a'.repeat(10000)

      mockedAxios.post.mockResolvedValueOnce({
        data: {
          choices: [
            {
              message: {
                content: JSON.stringify({
                  id: 'test',
                  name: 'Test Mission',
                  description: 'Test',
                  estimatedDuration: 3600,
                  phases: [],
                  contingencies: [],
                  successCriteria: [],
                  resources: { requiredAgents: 0, minBatteryLevel: 0, requiredCapabilities: [] }
                })
              }
            }
          ]
        }
      })

      const result = await gptInstance.generateMissionPlan(longObjective, {})

      expect(result.error).toBeUndefined()
      expect(result.plan).toBeDefined()
    })
  })
})