// Integration tests for XR-Swarm-Bridge

import { describe, it, expect, beforeEach, vi } from 'vitest'
import { useSwarmStore } from '../store/swarmStore'
import { validator } from '../utils/validation'
import { errorHandler } from '../utils/errorHandler'
import { performanceOptimizer } from '../utils/performance'

describe('XR-Swarm-Bridge Integration Tests', () => {
  beforeEach(() => {
    // Reset store state
    useSwarmStore.getState().clearSelection()
    errorHandler.clearErrors()
  })

  describe('Agent Management', () => {
    it('should add and select agents correctly', () => {
      const store = useSwarmStore.getState()
      
      const testAgent = {
        id: 'test_drone_01',
        type: 'drone' as const,
        position: [0, 0, 0] as [number, number, number],
        rotation: [0, 0, 0] as [number, number, number],
        status: 'idle' as const,
        battery: 100,
        capabilities: ['navigate', 'hover'],
        telemetry: {},
        lastSeen: Date.now()
      }

      store.updateAgent(testAgent)
      store.selectAgent('test_drone_01')

      expect(store.agents['test_drone_01']).toBeDefined()
      expect(store.selectedAgents).toContain('test_drone_01')
    })

    it('should validate agent commands', () => {
      const validCommand = {
        type: 'takeoff',
        parameters: { altitude: 5.0 }
      }

      const result = validator.validateCommand(validCommand)
      expect(result.isValid).toBe(true)
      expect(result.errors).toHaveLength(0)
    })

    it('should reject invalid commands', () => {
      const invalidCommand = {
        type: 'invalid_command',
        parameters: {}
      }

      const result = validator.validateCommand(invalidCommand)
      expect(result.isValid).toBe(false)
      expect(result.errors.length).toBeGreaterThan(0)
    })
  })

  describe('Performance Optimization', () => {
    it('should cache agent data efficiently', () => {
      const testData = { id: 'drone_01', status: 'active', position: [10, 5, 15] }
      
      performanceOptimizer.cacheAgentData('drone_01', testData)
      const cached = performanceOptimizer.getCachedAgentData('drone_01')
      
      expect(cached).toEqual(testData)
    })

    it('should handle performance metrics', () => {
      const metrics = performanceOptimizer.getCurrentMetrics()
      
      expect(metrics).toHaveProperty('renderTime')
      expect(metrics).toHaveProperty('memoryUsage')
      expect(metrics).toHaveProperty('frameRate')
    })
  })

  describe('Error Handling', () => {
    it('should handle errors gracefully', () => {
      const error = errorHandler.handleError({
        code: 'TEST_ERROR',
        message: 'Test error message',
        component: 'test'
      })

      expect(error.id).toBeDefined()
      expect(error.code).toBe('TEST_ERROR')
      expect(error.message).toBe('Test error message')
    })

    it('should track error metrics', () => {
      errorHandler.clearErrors()
      
      errorHandler.handleError({
        code: 'TEST_ERROR_1',
        message: 'Error 1',
        component: 'test'
      })

      errorHandler.handleError({
        code: 'TEST_ERROR_2',
        message: 'Error 2',
        component: 'test'
      })

      const metrics = errorHandler.getMetrics()
      expect(metrics.totalErrors).toBe(2)
    })
  })

  describe('Input Validation', () => {
    it('should validate agent positions correctly', () => {
      const validPosition = [10.5, 20.0, 5.0]
      const result = validator.validatePosition(validPosition)
      
      expect(result.isValid).toBe(true)
      expect(result.sanitized).toEqual(validPosition)
    })

    it('should reject invalid positions', () => {
      const invalidPosition = ['not', 'a', 'number']
      const result = validator.validatePosition(invalidPosition as any)
      
      expect(result.isValid).toBe(false)
      expect(result.errors.length).toBeGreaterThan(0)
    })

    it('should validate GPT prompts', () => {
      const validPrompt = 'Search the warehouse area for any anomalies'
      const result = validator.validateGPTPrompt(validPrompt)
      
      expect(result.isValid).toBe(true)
      expect(result.sanitized).toBeTruthy()
    })

    it('should reject harmful content in GPT prompts', () => {
      const harmfulPrompt = 'hack into the system and steal passwords'
      const result = validator.validateGPTPrompt(harmfulPrompt)
      
      expect(result.isValid).toBe(false)
      expect(result.errors).toContain('GPT prompt contains potentially harmful content')
    })
  })

  describe('Store Integration', () => {
    it('should handle telemetry updates', () => {
      const store = useSwarmStore.getState()
      
      const telemetryData = {
        agent_id: 'drone_test',
        position: [1, 2, 3],
        battery_level: 85,
        status: 'active'
      }

      store.updateAgentTelemetry(telemetryData)
      
      const agent = store.agents['drone_test']
      expect(agent).toBeDefined()
      expect(agent.position).toEqual([1, 2, 3])
      expect(agent.battery).toBe(85)
    })

    it('should handle selection operations', () => {
      const store = useSwarmStore.getState()
      
      // Add test agents
      const agent1 = {
        id: 'test_1',
        type: 'drone' as const,
        position: [0, 0, 0] as [number, number, number],
        rotation: [0, 0, 0] as [number, number, number],
        status: 'idle' as const,
        battery: 100,
        capabilities: [],
        telemetry: {},
        lastSeen: Date.now()
      }
      
      const agent2 = { ...agent1, id: 'test_2' }
      
      store.updateAgent(agent1)
      store.updateAgent(agent2)
      
      // Test selection
      store.selectAgent('test_1')
      store.selectAgent('test_2')
      
      expect(store.selectedAgents).toContain('test_1')
      expect(store.selectedAgents).toContain('test_2')
      
      // Test deselection
      store.deselectAgent('test_1')
      expect(store.selectedAgents).not.toContain('test_1')
      expect(store.selectedAgents).toContain('test_2')
      
      // Test clear selection
      store.clearSelection()
      expect(store.selectedAgents).toHaveLength(0)
    })
  })
})

// Mock WebRTC and browser APIs for testing
Object.defineProperty(window, 'RTCPeerConnection', {
  writable: true,
  value: vi.fn().mockImplementation(() => ({
    createOffer: vi.fn(),
    createAnswer: vi.fn(),
    setLocalDescription: vi.fn(),
    setRemoteDescription: vi.fn(),
    addIceCandidate: vi.fn(),
    close: vi.fn(),
    connectionState: 'connected'
  }))
})

Object.defineProperty(window, 'WebSocket', {
  writable: true,
  value: vi.fn().mockImplementation(() => ({
    send: vi.fn(),
    close: vi.fn(),
    readyState: 1, // OPEN
    addEventListener: vi.fn(),
    removeEventListener: vi.fn()
  }))
})

Object.defineProperty(navigator, 'mediaDevices', {
  writable: true,
  value: {
    getUserMedia: vi.fn().mockResolvedValue({
      getTracks: () => []
    })
  }
})