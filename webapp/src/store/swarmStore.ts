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
  trajectories: Record<string, number[][]>
  
  // Mission
  currentMission: Mission | null
  missionHistory: Mission[]
  
  // UI State
  viewMode: '2d' | '3d' | 'vr' | 'ar'
  showTelemetry: boolean
  showMinimap: boolean
  
  // Selection
  isSelecting: boolean
  selectionBox: { start: { x: number; y: number; z: number }; end: { x: number; y: number; z: number } } | null
  
  // Network
  latency: number
  packetLoss: number
  bandwidth: number
  webrtcStats: any
  
  // Actions
  setConnection: (connected: boolean, status: string) => void
  updateAgent: (agent: Agent) => void
  removeAgent: (agentId: string) => void
  selectAgent: (agentId: string) => void
  deselectAgent: (agentId: string) => void
  clearSelection: () => void
  selectMultiple: (agentIds: string[]) => void
  
  // Selection actions
  startSelection: (position: number[]) => void
  endSelection: (position: number[]) => void
  
  // Command actions
  sendCommand: (command: string, data?: any) => void
  addWaypoint: (position: number[]) => void
  setTargetDirection: (direction: number[]) => void
  setSendCommand: (sendFn: (command: any) => void) => void
  openAICommandInterface: () => void
  
  // Telemetry actions
  updateAgentTelemetry: (data: any) => void
  updateAgentVideo: (agentId: string, videoData: any) => void
  updateAgentHeartbeat: (agentId: string, timestamp: number) => void
  setWebRTCStats: (stats: any) => void
  setConnectionStatus: (status: string) => void
  
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
    trajectories: {},
    
    currentMission: null,
    missionHistory: [],
    
    viewMode: '3d',
    showTelemetry: true,
    showMinimap: true,
    
    isSelecting: false,
    selectionBox: null,
    
    latency: 0,
    packetLoss: 0,
    bandwidth: 0,
    webrtcStats: null,
    
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
    })),
    
    // Selection actions
    startSelection: (position) => set({
      isSelecting: true,
      selectionBox: {
        start: { x: position[0], y: position[1], z: position[2] },
        end: { x: position[0], y: position[1], z: position[2] }
      }
    }),
    
    endSelection: (position) => set((state) => {
      if (!state.selectionBox) return state
      
      const updatedBox = {
        ...state.selectionBox,
        end: { x: position[0], y: position[1], z: position[2] }
      }
      
      // Select agents within the box
      const selectedIds = Object.values(state.agents)
        .filter(agent => {
          const [x, y, z] = agent.position
          return x >= Math.min(updatedBox.start.x, updatedBox.end.x) &&
                 x <= Math.max(updatedBox.start.x, updatedBox.end.x) &&
                 y >= Math.min(updatedBox.start.y, updatedBox.end.y) &&
                 y <= Math.max(updatedBox.start.y, updatedBox.end.y) &&
                 z >= Math.min(updatedBox.start.z, updatedBox.end.z) &&
                 z <= Math.max(updatedBox.start.z, updatedBox.end.z)
        })
        .map(agent => agent.id)
      
      return {
        isSelecting: false,
        selectionBox: null,
        selectedAgents: selectedIds
      }
    }),
    
    // Command actions - these will be populated by WebRTC hook
    sendCommand: (command, data) => {
      console.log('Send command:', command, data)
      // This will be overridden by the WebRTC hook
    },
    
    addWaypoint: (position) => set((state) => {
      const updatedAgents = { ...state.agents }
      state.selectedAgents.forEach(agentId => {
        if (updatedAgents[agentId]) {
          updatedAgents[agentId] = {
            ...updatedAgents[agentId],
            telemetry: {
              ...updatedAgents[agentId].telemetry,
              waypoints: [...(updatedAgents[agentId].telemetry.waypoints || []), position]
            }
          }
        }
      })
      return { agents: updatedAgents }
    }),
    
    setTargetDirection: (direction) => {
      get().sendCommand('set_target_direction', { direction })
    },
    
    setSendCommand: (sendFn) => set({ sendCommand: sendFn }),
    
    openAICommandInterface: () => {
      console.log('Opening AI command interface')
      // This would open a modal or panel for AI commands
    },
    
    // Telemetry actions
    updateAgentTelemetry: (data) => {
      if (!data.agent_id) return
      
      set((state) => {
        const agent = state.agents[data.agent_id]
        if (!agent) return state
        
        // Update trajectory
        const newTrajectories = { ...state.trajectories }
        if (!newTrajectories[data.agent_id]) {
          newTrajectories[data.agent_id] = []
        }
        
        if (data.position) {
          newTrajectories[data.agent_id].push(data.position)
          // Keep only last 100 positions
          if (newTrajectories[data.agent_id].length > 100) {
            newTrajectories[data.agent_id] = newTrajectories[data.agent_id].slice(-100)
          }
        }
        
        return {
          agents: {
            ...state.agents,
            [data.agent_id]: {
              ...agent,
              position: data.position || agent.position,
              rotation: data.rotation || agent.rotation,
              status: data.status || agent.status,
              battery: data.battery_level !== undefined ? data.battery_level : agent.battery,
              telemetry: { ...agent.telemetry, ...data },
              lastSeen: Date.now()
            }
          },
          trajectories: newTrajectories
        }
      })
    },
    
    updateAgentVideo: (agentId, videoData) => set((state) => ({
      agents: {
        ...state.agents,
        [agentId]: state.agents[agentId] ? {
          ...state.agents[agentId],
          telemetry: {
            ...state.agents[agentId].telemetry,
            videoFrame: videoData
          }
        } : state.agents[agentId]
      }
    })),
    
    updateAgentHeartbeat: (agentId, timestamp) => set((state) => ({
      agents: {
        ...state.agents,
        [agentId]: state.agents[agentId] ? {
          ...state.agents[agentId],
          lastSeen: timestamp
        } : state.agents[agentId]
      }
    })),
    
    setWebRTCStats: (stats) => set({ webrtcStats: stats }),
    
    setConnectionStatus: (status) => set({ connectionStatus: status })
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