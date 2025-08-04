import { create } from 'zustand'
import { subscribeWithSelector } from 'zustand/middleware'

export interface Agent {
  id: string
  type: 'drone' | 'ugv' | 'manipulator' | 'generic'
  position: [number, number, number]
  rotation: [number, number, number]
  status: 'idle' | 'active' | 'error' | 'emergency_stop' | 'timeout'
  battery: number
  capabilities: string[]
  telemetry: Record<string, any>
  lastSeen: number
}

export interface Mission {
  id: string
  name: string
  description: string
  phases: Array<{
    phase: number
    description: string
    assignments: Record<string, string>
  }>
  status: 'pending' | 'active' | 'completed' | 'failed'
  startTime?: number
  endTime?: number
}

export interface SwarmState {
  // Connection
  isConnected: boolean
  connectionStatus: string
  websocketUrl: string
  
  // Agents
  agents: Record<string, Agent>
  selectedAgents: string[]
  
  // Mission
  currentMission: Mission | null
  missionHistory: Mission[]
  
  // UI State
  viewMode: '2d' | '3d' | 'vr' | 'ar'
  showTelemetry: boolean
  showMinimap: boolean
  
  // Network
  latency: number
  packetLoss: number
  bandwidth: number
  
  // Actions
  setConnection: (connected: boolean, status: string) => void
  updateAgent: (agent: Agent) => void
  removeAgent: (agentId: string) => void
  selectAgent: (agentId: string) => void
  deselectAgent: (agentId: string) => void
  clearSelection: () => void
  selectMultiple: (agentIds: string[]) => void
  
  startMission: (mission: Mission) => void
  updateMission: (mission: Mission) => void
  completeMission: () => void
  
  setViewMode: (mode: '2d' | '3d' | 'vr' | 'ar') => void
  toggleTelemetry: () => void
  toggleMinimap: () => void
  
  updateNetworkMetrics: (metrics: { latency?: number; packetLoss?: number; bandwidth?: number }) => void
}

export const useSwarmStore = create<SwarmState>()(
  subscribeWithSelector((set, get) => ({
    // Initial state
    isConnected: false,
    connectionStatus: 'Disconnected',
    websocketUrl: 'ws://localhost:8443',
    
    agents: {},
    selectedAgents: [],
    
    currentMission: null,
    missionHistory: [],
    
    viewMode: '3d',
    showTelemetry: true,
    showMinimap: true,
    
    latency: 0,
    packetLoss: 0,
    bandwidth: 0,
    
    // Actions
    setConnection: (connected, status) => set({
      isConnected: connected,
      connectionStatus: status
    }),
    
    updateAgent: (agent) => set((state) => ({
      agents: {
        ...state.agents,
        [agent.id]: {
          ...state.agents[agent.id],
          ...agent,
          lastSeen: Date.now()
        }
      }
    })),
    
    removeAgent: (agentId) => set((state) => ({
      agents: Object.fromEntries(
        Object.entries(state.agents).filter(([id]) => id !== agentId)
      ),
      selectedAgents: state.selectedAgents.filter(id => id !== agentId)
    })),
    
    selectAgent: (agentId) => set((state) => ({
      selectedAgents: state.selectedAgents.includes(agentId)
        ? state.selectedAgents
        : [...state.selectedAgents, agentId]
    })),
    
    deselectAgent: (agentId) => set((state) => ({
      selectedAgents: state.selectedAgents.filter(id => id !== agentId)
    })),
    
    clearSelection: () => set({ selectedAgents: [] }),
    
    selectMultiple: (agentIds) => set({ selectedAgents: agentIds }),
    
    startMission: (mission) => set((state) => ({
      currentMission: { ...mission, status: 'active', startTime: Date.now() },
      missionHistory: [...state.missionHistory, mission]
    })),
    
    updateMission: (mission) => set({ currentMission: mission }),
    
    completeMission: () => set((state) => ({
      currentMission: state.currentMission 
        ? { ...state.currentMission, status: 'completed', endTime: Date.now() }
        : null
    })),
    
    setViewMode: (mode) => set({ viewMode: mode }),
    
    toggleTelemetry: () => set((state) => ({ showTelemetry: !state.showTelemetry })),
    
    toggleMinimap: () => set((state) => ({ showMinimap: !state.showMinimap })),
    
    updateNetworkMetrics: (metrics) => set((state) => ({
      ...state,
      ...metrics
    }))
  }))
)

// Selectors for computed values
export const useSelectedAgents = () => {
  const agents = useSwarmStore(state => state.agents)
  const selectedIds = useSwarmStore(state => state.selectedAgents)
  return selectedIds.map(id => agents[id]).filter(Boolean)
}

export const useAgentsByType = (type?: Agent['type']) => {
  const agents = useSwarmStore(state => state.agents)
  return Object.values(agents).filter(agent => !type || agent.type === type)
}

export const useActiveAgents = () => {
  const agents = useSwarmStore(state => state.agents)
  const now = Date.now()
  const timeout = 10000 // 10 seconds
  
  return Object.values(agents).filter(agent => 
    now - agent.lastSeen < timeout && agent.status !== 'timeout'
  )
}