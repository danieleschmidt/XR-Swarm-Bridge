import axios from 'axios'

export interface GPTRequest {
  prompt: string
  context?: {
    agents: any[]
    mission?: any
    environment?: any
  }
  maxTokens?: number
  temperature?: number
}

export interface GPTResponse {
  plan?: MissionPlan
  commands?: SwarmCommand[]
  analysis?: string
  error?: string
}

export interface MissionPlan {
  id: string
  name: string
  description: string
  estimatedDuration: number
  phases: Array<{
    phase: number
    description: string
    duration: number
    assignments: Record<string, string>
    conditions?: string[]
  }>
  contingencies: Array<{
    trigger: string
    action: string
    priority: 'low' | 'medium' | 'high'
  }>
  successCriteria: string[]
  resources: {
    requiredAgents: number
    minBatteryLevel: number
    requiredCapabilities: string[]
  }
}

export interface SwarmCommand {
  type: 'formation' | 'search' | 'navigate' | 'custom'
  target: 'all' | 'selected' | string[]
  parameters: Record<string, any>
  priority: 'low' | 'medium' | 'high'
  timeout?: number
}

export class GPTIntegration {
  private apiKey: string
  private baseUrl: string = 'https://api.openai.com/v1'
  private model: string = 'gpt-4o'
  
  constructor(apiKey?: string) {
    this.apiKey = apiKey || import.meta.env.VITE_OPENAI_API_KEY || ''
    
    if (!this.apiKey) {
      console.warn('OpenAI API key not configured. GPT-4o features will be disabled.')
    }
  }

  async generateMissionPlan(objective: string, context: GPTRequest['context']): Promise<GPTResponse> {
    if (!this.apiKey) {
      return { error: 'OpenAI API key not configured' }
    }

    const systemPrompt = `You are an AI mission planner for robotic swarms. Generate detailed, executable mission plans based on user objectives.

Available Robot Types:
- Drones: Can fly, have cameras, limited battery, good for reconnaissance
- UGVs: Ground vehicles, good payload capacity, longer endurance
- Manipulators: Can interact with objects, precise movements

Your response must be valid JSON matching the MissionPlan interface. Include:
1. Realistic time estimates
2. Specific robot assignments
3. Contingency plans for common failure modes
4. Clear success criteria
5. Safety considerations

Current Context:
- Available agents: ${context?.agents?.length || 0}
- Agent types: ${context?.agents?.map(a => a.type).join(', ') || 'unknown'}
- Current mission: ${context?.mission?.name || 'none'}`

    const userPrompt = `Objective: ${objective}

Generate a detailed mission plan that uses available robots effectively. Consider:
- Battery life and operational limits
- Coordination between different robot types  
- Safety and redundancy
- Real-world constraints and physics

Respond with only the JSON mission plan.`

    try {
      const response = await axios.post(
        `${this.baseUrl}/chat/completions`,
        {
          model: this.model,
          messages: [
            { role: 'system', content: systemPrompt },
            { role: 'user', content: userPrompt }
          ],
          max_tokens: 2000,
          temperature: 0.7,
          response_format: { type: "json_object" }
        },
        {
          headers: {
            'Authorization': `Bearer ${this.apiKey}`,
            'Content-Type': 'application/json'
          },
          timeout: 30000
        }
      )

      const content = response.data.choices[0]?.message?.content
      if (!content) {
        throw new Error('Empty response from GPT-4o')
      }

      const plan = JSON.parse(content) as MissionPlan
      
      // Validate and enhance the plan
      plan.id = plan.id || `mission_${Date.now()}`
      
      return { plan }
    } catch (error: any) {
      console.error('GPT-4o mission planning error:', error)
      return { 
        error: error.response?.data?.error?.message || error.message || 'Failed to generate mission plan'
      }
    }
  }

  async generateSwarmCommands(naturalLanguageCommand: string, context: GPTRequest['context']): Promise<GPTResponse> {
    if (!this.apiKey) {
      return { error: 'OpenAI API key not configured' }
    }

    const systemPrompt = `You are an AI commander for robotic swarms. Convert natural language commands into executable swarm commands.

Available Command Types:
- formation: Arrange robots in patterns (line, grid, circle, custom)
- search: Search patterns (spiral, grid, perimeter, random)
- navigate: Move to positions or waypoints
- emergency_stop: Immediate halt of all operations
- return_to_base: Return all agents to starting positions

Your response must be valid JSON with a "commands" array matching the SwarmCommand interface.

Current Context:
- Available agents: ${context?.agents?.length || 0}
- Selected agents: ${context?.agents?.filter(a => a.selected)?.length || 0}
- Agent positions: Available
- Current status: Active`

    const userPrompt = `Command: "${naturalLanguageCommand}"

Convert this to executable swarm commands. Consider:
- Which robots should execute the command
- Appropriate parameters for the command type
- Safety constraints and collision avoidance
- Battery levels and operational limits

Respond with only the JSON commands array.`

    try {
      const response = await axios.post(
        `${this.baseUrl}/chat/completions`,
        {
          model: this.model,
          messages: [
            { role: 'system', content: systemPrompt },
            { role: 'user', content: userPrompt }
          ],
          max_tokens: 1000,
          temperature: 0.3,
          response_format: { type: "json_object" }
        },
        {
          headers: {
            'Authorization': `Bearer ${this.apiKey}`,
            'Content-Type': 'application/json'
          },
          timeout: 20000
        }
      )

      const content = response.data.choices[0]?.message?.content
      if (!content) {
        throw new Error('Empty response from GPT-4o')
      }

      const result = JSON.parse(content)
      return { commands: result.commands || [] }
    } catch (error: any) {
      console.error('GPT-4o command generation error:', error)
      return { 
        error: error.response?.data?.error?.message || error.message || 'Failed to generate commands'
      }
    }
  }

  async analyzeSwarmPerformance(metrics: any): Promise<GPTResponse> {
    if (!this.apiKey) {
      return { error: 'OpenAI API key not configured' }
    }

    const systemPrompt = `You are an AI analyst for robotic swarm operations. Analyze performance metrics and provide actionable insights.`

    const userPrompt = `Analyze these swarm metrics:
${JSON.stringify(metrics, null, 2)}

Provide insights on:
- Performance efficiency
- Potential issues or bottlenecks
- Optimization recommendations
- Safety concerns

Respond with a clear analysis.`

    try {
      const response = await axios.post(
        `${this.baseUrl}/chat/completions`,
        {
          model: this.model,
          messages: [
            { role: 'system', content: systemPrompt },
            { role: 'user', content: userPrompt }
          ],
          max_tokens: 800,
          temperature: 0.4
        },
        {
          headers: {
            'Authorization': `Bearer ${this.apiKey}`,
            'Content-Type': 'application/json'
          },
          timeout: 20000
        }
      )

      const content = response.data.choices[0]?.message?.content
      return { analysis: content || 'No analysis available' }
    } catch (error: any) {
      console.error('GPT-4o analysis error:', error)
      return { 
        error: error.response?.data?.error?.message || error.message || 'Failed to analyze performance'
      }
    }
  }

  async optimizeFormation(agents: any[], objective: string): Promise<GPTResponse> {
    if (!this.apiKey) {
      return { error: 'OpenAI API key not configured' }
    }

    const systemPrompt = `You are an AI formation optimizer for robotic swarms. Design optimal formations based on objectives and constraints.`

    const userPrompt = `Optimize formation for ${agents.length} agents with objective: "${objective}"

Agent capabilities:
${agents.map(a => `${a.id}: ${a.type} at [${a.position.join(', ')}]`).join('\n')}

Generate formation commands that:
- Maximize mission effectiveness
- Maintain safe distances
- Consider agent capabilities
- Account for environmental factors

Respond with JSON formation commands.`

    try {
      const response = await axios.post(
        `${this.baseUrl}/chat/completions`,
        {
          model: this.model,
          messages: [
            { role: 'system', content: systemPrompt },
            { role: 'user', content: userPrompt }
          ],
          max_tokens: 1000,
          temperature: 0.2,
          response_format: { type: "json_object" }
        },
        {
          headers: {
            'Authorization': `Bearer ${this.apiKey}`,
            'Content-Type': 'application/json'
          },
          timeout: 20000
        }
      )

      const content = response.data.choices[0]?.message?.content
      if (!content) {
        throw new Error('Empty response from GPT-4o')
      }

      const result = JSON.parse(content)
      return { commands: result.commands || [] }
    } catch (error: any) {
      console.error('GPT-4o formation optimization error:', error)
      return { 
        error: error.response?.data?.error?.message || error.message || 'Failed to optimize formation'
      }
    }
  }
}

// Singleton instance
export const gptIntegration = new GPTIntegration()

// React hook for using GPT integration
import { useCallback, useState } from 'react'

export function useGPTIntegration() {
  const [isLoading, setIsLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)

  const generateMissionPlan = useCallback(async (objective: string, context?: GPTRequest['context']) => {
    setIsLoading(true)
    setError(null)
    
    try {
      const result = await gptIntegration.generateMissionPlan(objective, context)
      if (result.error) {
        setError(result.error)
      }
      return result
    } catch (err: any) {
      setError(err.message)
      return { error: err.message }
    } finally {
      setIsLoading(false)
    }
  }, [])

  const generateCommands = useCallback(async (command: string, context?: GPTRequest['context']) => {
    setIsLoading(true)
    setError(null)
    
    try {
      const result = await gptIntegration.generateSwarmCommands(command, context)
      if (result.error) {
        setError(result.error)
      }
      return result
    } catch (err: any) {
      setError(err.message)
      return { error: err.message }
    } finally {
      setIsLoading(false)
    }
  }, [])

  const analyzePerformance = useCallback(async (metrics: any) => {
    setIsLoading(true)
    setError(null)
    
    try {
      const result = await gptIntegration.analyzeSwarmPerformance(metrics)
      if (result.error) {
        setError(result.error)
      }
      return result
    } catch (err: any) {
      setError(err.message)
      return { error: err.message }
    } finally {
      setIsLoading(false)
    }
  }, [])

  return {
    generateMissionPlan,
    generateCommands,
    analyzePerformance,
    isLoading,
    error,
    clearError: () => setError(null)
  }
}