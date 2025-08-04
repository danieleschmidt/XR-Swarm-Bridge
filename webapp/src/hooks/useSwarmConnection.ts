import { useEffect, useRef, useCallback } from 'react'
import { io, Socket } from 'socket.io-client'
import { useSwarmStore } from '../store/swarmStore'
import type { Agent } from '../store/swarmStore'

interface SwarmMessage {
  type: string
  data?: any
  agent_id?: string
  timestamp?: number
}

export function useSwarmConnection() {
  const socketRef = useRef<Socket | null>(null)
  const reconnectTimeoutRef = useRef<NodeJS.Timeout>()
  
  const {
    websocketUrl,
    setConnection,
    updateAgent,
    removeAgent,
    updateNetworkMetrics,
    updateMission
  } = useSwarmStore()

  const handleConnect = useCallback(() => {
    console.log('Connected to swarm coordinator')
    setConnection(true, 'Connected')
    
    // Request initial status
    if (socketRef.current) {
      socketRef.current.emit('message', JSON.stringify({
        type: 'get_status'
      }))
    }
  }, [setConnection])

  const handleDisconnect = useCallback(() => {
    console.log('Disconnected from swarm coordinator')
    setConnection(false, 'Disconnected')
    
    // Schedule reconnection
    reconnectTimeoutRef.current = setTimeout(() => {
      if (socketRef.current) {
        socketRef.current.connect()
      }
    }, 5000)
  }, [setConnection])

  const handleMessage = useCallback((data: string) => {
    try {
      const message: SwarmMessage = JSON.parse(data)
      
      switch (message.type) {
        case 'swarm_status':
          // Update all agents from status message
          if (message.data?.agents) {
            Object.entries(message.data.agents).forEach(([id, agentData]: [string, any]) => {
              const agent: Agent = {
                id,
                type: agentData.agent_type || 'generic',
                position: [
                  agentData.telemetry?.position?.x || 0,
                  agentData.telemetry?.position?.y || 0,
                  agentData.telemetry?.position?.z || 0
                ],
                rotation: [0, 0, 0],
                status: agentData.status || 'idle',
                battery: agentData.telemetry?.battery_level || 100,
                capabilities: agentData.capabilities || [],
                telemetry: agentData.telemetry || {},
                lastSeen: Date.now()
              }
              updateAgent(agent)
            })
          }
          break

        case 'agent_telemetry':
          // Update specific agent telemetry
          if (message.agent_id && message.data) {
            const agent: Agent = {
              id: message.agent_id,
              type: message.data.agent_type || 'generic',
              position: [
                message.data.position?.x || 0,
                message.data.position?.y || 0,
                message.data.position?.z || 0
              ],
              rotation: [0, 0, 0],
              status: message.data.status || 'idle',
              battery: message.data.battery_level || 100,
              capabilities: message.data.capabilities || [],
              telemetry: message.data,
              lastSeen: Date.now()
            }
            updateAgent(agent)
          }
          break

        case 'agent_disconnected':
          if (message.agent_id) {
            removeAgent(message.agent_id)
          }
          break

        case 'mission_update':
          if (message.data) {
            updateMission(message.data)
          }
          break

        case 'network_metrics':
          if (message.data) {
            updateNetworkMetrics(message.data)
          }
          break

        case 'error':
          console.error('Swarm error:', message.data)
          break

        default:
          console.log('Unknown message type:', message.type)
      }
    } catch (error) {
      console.error('Error parsing swarm message:', error)
    }
  }, [updateAgent, removeAgent, updateMission, updateNetworkMetrics])

  const sendCommand = useCallback((command: any) => {
    if (socketRef.current?.connected) {
      socketRef.current.emit('message', JSON.stringify(command))
    } else {
      console.warn('Cannot send command - not connected to swarm')
    }
  }, [])

  const sendGlobalCommand = useCallback((command: any) => {
    sendCommand({
      type: 'swarm_command',
      command
    })
  }, [sendCommand])

  const sendAgentCommand = useCallback((agentId: string, command: any) => {
    sendCommand({
      type: 'agent_command',
      agent_id: agentId,
      command
    })
  }, [sendCommand])

  const startMission = useCallback((mission: any) => {
    sendCommand({
      type: 'mission_start',
      mission
    })
  }, [sendCommand])

  const stopMission = useCallback(() => {
    sendCommand({
      type: 'mission_stop'
    })
  }, [sendCommand])

  useEffect(() => {
    // Initialize WebSocket connection
    const wsUrl = websocketUrl.replace('ws://', 'http://').replace('wss://', 'https://')
    
    socketRef.current = io(wsUrl, {
      transports: ['websocket'],
      reconnection: true,
      reconnectionDelay: 1000,
      reconnectionAttempts: 5,
      timeout: 20000
    })

    const socket = socketRef.current

    socket.on('connect', handleConnect)
    socket.on('disconnect', handleDisconnect)
    socket.on('message', handleMessage)
    
    socket.on('connect_error', (error) => {
      console.error('Connection error:', error)
      setConnection(false, 'Connection Error')
    })

    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current)
      }
      
      socket.off('connect', handleConnect)
      socket.off('disconnect', handleDisconnect)
      socket.off('message', handleMessage)
      socket.disconnect()
    }
  }, [websocketUrl, handleConnect, handleDisconnect, handleMessage, setConnection])

  // Expose command functions globally
  useEffect(() => {
    ;(window as any).swarmCommands = {
      sendGlobalCommand,
      sendAgentCommand,
      startMission,
      stopMission
    }
  }, [sendGlobalCommand, sendAgentCommand, startMission, stopMission])

  return {
    sendGlobalCommand,
    sendAgentCommand,
    startMission,
    stopMission,
    isConnected: socketRef.current?.connected || false
  }
}