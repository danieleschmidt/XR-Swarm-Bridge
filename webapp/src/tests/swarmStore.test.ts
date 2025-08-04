import { describe, it, expect, beforeEach } from 'vitest'
import { useSwarmStore } from '../store/swarmStore'
import type { Agent } from '../store/swarmStore'

describe('SwarmStore', () => {
  beforeEach(() => {
    // Reset store state before each test
    useSwarmStore.setState({
      agents: {},
      selectedAgents: [],
      currentMission: null,
      missionHistory: [],
      isConnected: false,
      connectionStatus: 'Disconnected'
    })
  })

  describe('Connection Management', () => {
    it('should update connection status', () => {
      const { setConnection } = useSwarmStore.getState()
      
      setConnection(true, 'Connected')
      
      const state = useSwarmStore.getState()
      expect(state.isConnected).toBe(true)
      expect(state.connectionStatus).toBe('Connected')
    })
  })

  describe('Agent Management', () => {
    const mockAgent: Agent = {
      id: 'test_drone_01',
      type: 'drone',
      position: [10, 5, 2],
      rotation: [0, 0, 0],
      status: 'active',
      battery: 85,
      capabilities: ['navigate', 'hover', 'stream_video'],
      telemetry: { altitude: 2, speed: 1.5 },
      lastSeen: Date.now()
    }

    it('should add new agent', () => {
      const { updateAgent } = useSwarmStore.getState()
      
      updateAgent(mockAgent)
      
      const state = useSwarmStore.getState()
      expect(state.agents[mockAgent.id]).toEqual({
        ...mockAgent,
        lastSeen: expect.any(Number)
      })
    })

    it('should update existing agent', () => {
      const { updateAgent } = useSwarmStore.getState()
      
      // Add initial agent
      updateAgent(mockAgent)
      
      // Update agent with new data
      const updatedAgent = {
        ...mockAgent,
        battery: 70,
        position: [15, 8, 3] as [number, number, number]
      }
      updateAgent(updatedAgent)
      
      const state = useSwarmStore.getState()
      expect(state.agents[mockAgent.id].battery).toBe(70)
      expect(state.agents[mockAgent.id].position).toEqual([15, 8, 3])
    })

    it('should remove agent', () => {
      const { updateAgent, removeAgent } = useSwarmStore.getState()
      
      updateAgent(mockAgent)
      removeAgent(mockAgent.id)
      
      const state = useSwarmStore.getState()
      expect(state.agents[mockAgent.id]).toBeUndefined()
    })

    it('should remove agent from selection when removed', () => {
      const { updateAgent, selectAgent, removeAgent } = useSwarmStore.getState()
      
      updateAgent(mockAgent)
      selectAgent(mockAgent.id)
      removeAgent(mockAgent.id)
      
      const state = useSwarmStore.getState()
      expect(state.selectedAgents).not.toContain(mockAgent.id)
    })
  })

  describe('Agent Selection', () => {
    const agents: Agent[] = [
      {
        id: 'drone_01',
        type: 'drone',
        position: [0, 0, 5],
        rotation: [0, 0, 0],
        status: 'idle',
        battery: 100,
        capabilities: [],
        telemetry: {},
        lastSeen: Date.now()
      },
      {
        id: 'ugv_01',
        type: 'ugv',
        position: [10, 0, 0],
        rotation: [0, 0, 0],
        status: 'active',
        battery: 80,
        capabilities: [],
        telemetry: {},
        lastSeen: Date.now()
      }
    ]

    beforeEach(() => {
      const { updateAgent } = useSwarmStore.getState()
      agents.forEach(updateAgent)
    })

    it('should select single agent', () => {
      const { selectAgent } = useSwarmStore.getState()
      
      selectAgent('drone_01')
      
      const state = useSwarmStore.getState()
      expect(state.selectedAgents).toContain('drone_01')
    })

    it('should not duplicate selection', () => {
      const { selectAgent } = useSwarmStore.getState()
      
      selectAgent('drone_01')
      selectAgent('drone_01')
      
      const state = useSwarmStore.getState()
      expect(state.selectedAgents.filter(id => id === 'drone_01')).toHaveLength(1)
    })

    it('should select multiple agents', () => {
      const { selectAgent } = useSwarmStore.getState()
      
      selectAgent('drone_01')
      selectAgent('ugv_01')
      
      const state = useSwarmStore.getState()
      expect(state.selectedAgents).toContain('drone_01')
      expect(state.selectedAgents).toContain('ugv_01')
    })

    it('should deselect agent', () => {
      const { selectAgent, deselectAgent } = useSwarmStore.getState()
      
      selectAgent('drone_01')
      deselectAgent('drone_01')
      
      const state = useSwarmStore.getState()
      expect(state.selectedAgents).not.toContain('drone_01')
    })

    it('should clear all selections', () => {
      const { selectAgent, clearSelection } = useSwarmStore.getState()
      
      selectAgent('drone_01')
      selectAgent('ugv_01')
      clearSelection()
      
      const state = useSwarmStore.getState()
      expect(state.selectedAgents).toHaveLength(0)
    })

    it('should select multiple agents at once', () => {
      const { selectMultiple } = useSwarmStore.getState()
      
      selectMultiple(['drone_01', 'ugv_01'])
      
      const state = useSwarmStore.getState()
      expect(state.selectedAgents).toEqual(['drone_01', 'ugv_01'])
    })
  })

  describe('Mission Management', () => {
    const mockMission = {
      id: 'mission_001',
      name: 'Test Mission',
      description: 'A test mission',
      phases: [
        {
          phase: 1,
          description: 'Phase 1',
          assignments: { 'drone_01': 'patrol area A' }
        }
      ],
      status: 'pending' as const
    }

    it('should start mission', () => {
      const { startMission } = useSwarmStore.getState()
      
      startMission(mockMission)
      
      const state = useSwarmStore.getState()
      expect(state.currentMission).toEqual({
        ...mockMission,
        status: 'active',
        startTime: expect.any(Number)
      })
      expect(state.missionHistory).toContain(mockMission)
    })

    it('should update mission', () => {
      const { startMission, updateMission } = useSwarmStore.getState()
      
      startMission(mockMission)
      
      const updatedMission = {
        ...mockMission,
        status: 'active' as const,
        description: 'Updated mission'
      }
      updateMission(updatedMission)
      
      const state = useSwarmStore.getState()
      expect(state.currentMission?.description).toBe('Updated mission')
    })

    it('should complete mission', () => {
      const { startMission, completeMission } = useSwarmStore.getState()
      
      startMission(mockMission)
      completeMission()
      
      const state = useSwarmStore.getState()
      expect(state.currentMission?.status).toBe('completed')
      expect(state.currentMission?.endTime).toBeDefined()
    })
  })

  describe('UI State Management', () => {
    it('should change view mode', () => {
      const { setViewMode } = useSwarmStore.getState()
      
      setViewMode('vr')
      
      const state = useSwarmStore.getState()
      expect(state.viewMode).toBe('vr')
    })

    it('should toggle telemetry display', () => {
      const { toggleTelemetry } = useSwarmStore.getState()
      
      const initialState = useSwarmStore.getState().showTelemetry
      toggleTelemetry()
      
      const newState = useSwarmStore.getState().showTelemetry
      expect(newState).toBe(!initialState)
    })

    it('should toggle minimap display', () => {
      const { toggleMinimap } = useSwarmStore.getState()
      
      const initialState = useSwarmStore.getState().showMinimap
      toggleMinimap()
      
      const newState = useSwarmStore.getState().showMinimap
      expect(newState).toBe(!initialState)
    })
  })

  describe('Network Metrics', () => {
    it('should update network metrics', () => {
      const { updateNetworkMetrics } = useSwarmStore.getState()
      
      const metrics = {
        latency: 150,
        packetLoss: 0.02,
        bandwidth: 1000
      }
      updateNetworkMetrics(metrics)
      
      const state = useSwarmStore.getState()
      expect(state.latency).toBe(150)
      expect(state.packetLoss).toBe(0.02)
      expect(state.bandwidth).toBe(1000)
    })

    it('should partially update network metrics', () => {
      const { updateNetworkMetrics } = useSwarmStore.getState()
      
      updateNetworkMetrics({ latency: 100 })
      updateNetworkMetrics({ packetLoss: 0.01 })
      
      const state = useSwarmStore.getState()
      expect(state.latency).toBe(100)
      expect(state.packetLoss).toBe(0.01)
      expect(state.bandwidth).toBe(0) // Should remain unchanged
    })
  })
})