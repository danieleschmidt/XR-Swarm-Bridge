```typescript
import { useEffect, useRef, useCallback, useState } from 'react'
import { useSwarmStore } from '../store/swarmStore'

interface WebRTCConnection {
  peerConnection: RTCPeerConnection
  dataChannel: RTCDataChannel | null
  remoteStream: MediaStream | null
}

interface WebRTCStats {
  latency: number
  packetLoss: number
  bandwidth: number
  connectionState: RTCPeerConnectionState
}

export function useWebRTC() {
  const [isWebRTCSupported] = useState(() => {
    return typeof RTCPeerConnection !== 'undefined'
  })
  
  const connections = useRef<Map<string, WebRTCConnection>>(new Map())
  const localStreamRef = useRef<MediaStream | null>(null)
  const websocketRef = useRef<WebSocket | null>(null)
  const statsInterval = useRef<NodeJS.Timeout | null>(null)
  
  const { websocketUrl, updateNetworkMetrics, setConnection, setWebRTCStats, setConnectionStatus, setSendCommand } = useSwarmStore()

  const initializeWebSocket = useCallback(() => {
    if (!isWebRTCSupported) {
      console.warn('WebRTC not supported in this browser')
      return
    }

    const wsUrl = websocketUrl.replace('http://', 'ws://').replace('https://', 'wss://')
    const ws = new WebSocket(wsUrl)
    websocketRef.current = ws

    ws.onopen = () => {
      console.log('WebRTC signaling WebSocket connected')
      setConnectionStatus('websocket_connected')
    }

    ws.onmessage = async (event) => {
      try {
        const message = JSON.parse(event.data)
        await handleSignalingMessage(message)
      } catch (error) {
        console.error('Error handling signaling message:', error)
      }
    }

    ws.onclose = () => {
      console.log('WebRTC signaling WebSocket disconnected')
      setConnectionStatus('disconnected')
      // Attempt reconnection after delay
      setTimeout(initializeWebSocket, 5000)
    }

    ws.onerror = (error) => {
      console.error('WebRTC signaling WebSocket error:', error)
      setConnectionStatus('error')
    }
  }, [websocketUrl, isWebRTCSupported, setConnectionStatus])

  const createPeerConnection = useCallback((agentId: string): RTCPeerConnection => {
    const configuration: RTCConfiguration = {
      iceServers: [
        { urls: 'stun:stun.l.google.com:19302' },
        { urls: 'stun:stun1.l.google.com:19302' }
      ],
      iceCandidatePoolSize: 10
    }

    const peerConnection = new RTCPeerConnection(configuration)
    
    // Handle ICE candidates
    peerConnection.onicecandidate = (event) => {
      if (event.candidate && websocketRef.current) {
        websocketRef.current.send(JSON.stringify({
          type: 'ice_candidate',
          agentId,
          candidate: event.candidate
        }))
      }
    }

    // Handle connection state changes
    peerConnection.onconnectionstatechange = () => {
      const state = peerConnection.connectionState
      console.log(`WebRTC connection state for ${agentId}:`, state)
      
      if (state === 'connected') {
        setConnectionStatus('webrtc_connected')
        // Start measuring latency
        measureLatency(agentId)
        startStatsCollection(agentId)
      } else if (state === 'disconnected' || state === 'failed') {
        connections.current.delete(agentId)
        setConnectionStatus('disconnected')
      }
    }

    // Handle remote streams (video from robots)
    peerConnection.ontrack = (event) => {
      const connection = connections.current.get(agentId)
      if (connection) {
        connection.remoteStream = event.streams[0]
        console.log(`Received remote stream from ${agentId}`)
      }
    }

    // Handle data channels
    peerConnection.ondatachannel = (event) => {
      const dataChannel = event.channel
      const connection = connections.current.get(agentId)
      
      if (connection) {
        connection.dataChannel = dataChannel
        setupDataChannel(dataChannel, agentId)
      }
    }

    return peerConnection
  }, [setConnectionStatus])

  const setupDataChannel = (dataChannel: RTCDataChannel, agentId: string) => {
    dataChannel.onopen = () => {
      console.log(`Data channel opened for ${agentId}`)
      setConnectionStatus('webrtc_connected')
    }

    dataChannel.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data)
        handleDataChannelMessage(agentId, data)
      } catch (error) {
        console.error(`Error parsing data channel message from ${agentId}:`, error)
      }
    }

    dataChannel.onclose = () => {
      console.log(`Data channel closed for ${agentId}`)
      setConnectionStatus('websocket_connected')
    }

    dataChannel.onerror = (error) => {
      console.error(`Data channel error for ${agentId}:`, error)
    }
  }

  const handleDataChannelMessage = (agentId: string, data: any) => {
    switch (data.type) {
      case 'telemetry':
      case 'agent_telemetry':
        // Update agent telemetry with low-latency data
        const { pos, bat, stat } = data
        if (pos && bat !== undefined && stat) {
          useSwarmStore.getState().updateAgent({
            id: agentId,
            type: 'generic',
            position: [pos.x || 0, pos.y || 0, pos.z || 0],
            rotation: [0, 0, 0],
            status: stat,
            battery: bat,
            capabilities: [],
            telemetry: data,
            lastSeen: Date.now()
          })
        }
        if (data.data) {
          useSwarmStore.getState().updateAgentTelemetry(data.data)
        }
        break
      
      case 'video_frame':
        // Handle compressed video frames
        handleVideoFrame(agentId, data)
        if (data.data) {
          useSwarmStore.getState().updateAgentVideo(agentId, data.data)
        }
        break
      
      case 'heartbeat':
        // Update agent heartbeat
        useSwarmStore.getState().updateAgentHeartbeat(agentId, data.timestamp)
        break
      
      case 'pong':
        // Handled in measureLatency
        break
    }
  }

  const handleVideoFrame = (agentId: string, frameData: any) => {
    // Decode and display video frame
    // This would typically involve creating a video element or canvas
    console.log(`Received video frame from ${agentId}`, frameData.width, 'x', frameData.height)
  }

  const handleSignalingMessage = async (message: any) => {
    const { type, agentId, agent_id } = message
    const effectiveAgentId = agentId || agent_id

    switch (type) {
      case 'webrtc_offer':
        await handleOffer(effectiveAgentId, message)
        break
      case 'webrtc_answer':
        await handleAnswer(effectiveAgentId, message)
        break
      case 'ice_candidate':
        await handleIceCandidate(effectiveAgentId, message)
        break
      case 'agent_telemetry':
        // Handle incoming telemetry data
        useSwarmStore.getState().updateAgentTelemetry(message.data)
        break
      case 'error':
        console.error('WebRTC signaling error:', message.message)
        break
      default:
        console.log('Unknown signaling message type:', type)
    }
  }

  const handleOffer = async (agentId: string, message: any) => {
    const peerConnection = createPeerConnection(agentId)
    
    // Create data channel for low-latency commands
    const dataChannel = peerConnection.createDataChannel(`commands_${agentId}`, {
      ordered: false,
      maxRetransmits: 0
    })
    
    const connection: WebRTCConnection = {
      peerConnection,
      dataChannel,
      remoteStream: null
    }
    
    connections.current.set(agentId, connection)
    setupDataChannel(dataChannel, agentId)

    // Set remote description
    await peerConnection.setRemoteDescription(new RTCSessionDescription({
      type: 'offer',
      sdp: message.sdp
    }))

    // Create answer
    const answer = await peerConnection.createAnswer()
    await peerConnection.setLocalDescription(answer)

    // Send answer back
    if (websocketRef.current) {
      websocketRef.current.send(JSON.stringify({
        type: 'webrtc_answer',
        agentId,
        sdp: answer.sdp,
        type: answer.type
      }))
    }
  }

  const handleAnswer = async (agentId: string, message: any) => {
    const connection = connections.current.get(agentId)
    if (connection) {
      await connection.peerConnection.setRemoteDescription(new RTCSessionDescription({
        type: 'answer',
        sdp: message.sdp
      }))
    }
  }

  const handleIceCandidate = async (agentId: string, message: any) => {
    const connection = connections.current.get(agentId)
    if (connection && message.candidate) {
      await connection.peerConnection.addIceCandidate(new RTCIceCandidate(message.candidate))
    }
  }

  const sendDataChannelMessage = useCallback((agentId: string, data: any) => {
    const connection = connections.current.get(agentId)
    if (connection?.dataChannel && connection.dataChannel.readyState === 'open') {
      connection.dataChannel.send(JSON.stringify(data))
      return true
    }
    return false
  }, [])

  const sendLowLatencyCommand = useCallback((agentId: string, command: any) => {
    // Try WebRTC data channel first for ultra-low latency
    if (sendDataChannelMessage(agentId, {
      type: 'command',
      ...command,
      timestamp: Date.now()
    })) {
      return 'webrtc'
    }
    
    // Fallback to WebSocket
    if (websocketRef.current && websocketRef.current.readyState === WebSocket.OPEN) {
      websocketRef.current.send(JSON.stringify({
        type: 'agent_command',
        agent_id: agentId,
        command
      }))
      return 'websocket'
    }
    
    return null
  }, [sendDataChannelMessage])

  const sendCommand = useCallback((command: any) => {
    // If command has agent_id, send to specific agent
    if (command.agent_id) {
      return sendLowLatencyCommand(command.agent_id, command)
    }
    
    // Otherwise broadcast to all connected agents
    let sent = false
    connections.current.forEach((connection, agentId) => {
      if (sendDataChannelMessage(agentId, {
        type: 'command',
        ...command,
        timestamp: Date.now()
      })) {
        sent = true
      }
    })
    
    // Fallback to WebSocket broadcast
    if (!sent && websocketRef.current && websocketRef.current.readyState === WebSocket.OPEN) {
      websocketRef.current.send(JSON.stringify({
        type: 'swarm_command',
        ...command,
        timestamp: Date.now()
      }))
      return 'websocket'
    }
    
    return sent ? 'webrtc' : null
  }, [sendDataChannelMessage, sendLowLatencyCommand])

  const measureLatency = useCallback((agentId: string) => {
    const startTime = Date.now()
    
    if (sendDataChannelMessage(agentId, {
      type: 'ping',
      timestamp: startTime
    })) {
      // Set up pong handler
      const connection = connections.current.get(agentId)
      if (connection?.dataChannel) {
        const originalOnMessage = connection.dataChannel.onmessage
        
        connection.dataChannel.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data)
            if (data.type === 'pong' && data.timestamp === startTime) {
              const latency = Date.now() - startTime
              updateNetworkMetrics({ latency })
              
              // Restore original handler
              connection.dataChannel!.onmessage = originalOnMessage
            }
          } catch (error) {
            // Ignore parsing errors for non-ping messages
          }
          
          // Call original handler if it exists
          if (originalOnMessage) {
            originalOnMessage(event)
          }
        }
      }
    }
  }, [sendDataChannelMessage, updateNetworkMetrics])

  const parseRTCStats = (stats: RTCStatsReport): WebRTCStats => {
    let latency = 0
    let packetLoss = 0
    let bandwidth = 0
    let connectionState: RTCPeerConnectionState = 'new'
    
    stats.forEach((report) => {
      if (report.type === 'candidate-pair' && report.state === 'succeeded') {
        latency = report.currentRoundTripTime ? report.currentRoundTripTime * 1000 : 0
        bandwidth = report.availableIncomingBitrate || 0
      }
      
      if (report.type === 'inbound-rtp') {
        const packetsLost = report.packetsLost || 0
        const packetsReceived = report.packetsReceived || 0
        const totalPackets = packetsLost + packetsReceived
        packetLoss = totalPackets > 0 ? packetsLost / totalPackets : 0
      }
      
      if (report.type === 'data-channel') {
        // Additional data channel stats if needed
      }
      
      if (report.type === 'transport') {
        // Additional transport stats if needed
      }
    })
    
    return {
      latency,
      packetLoss,
      bandwidth,
      connectionState
    }
  }

  const startStatsCollection = (agentId: string) => {
    const connection = connections.current.get(agentId)
    if (!connection) return
    
    const collectStats = async () => {
      if (!connection.peerConnection) return
      
      try {
        const stats = await connection.peerConnection.getStats()
        const webrtcStats = parseRTCStats(stats)
        webrtcStats.connectionState = connection.peerConnection.connectionState
        setWebRTCStats(webrtcStats)
      } catch (error) {
        console.error(`Error collecting WebRTC stats for ${agentId}:`, error)
      }
    }
    
    // Initial collection
    collectStats()
    
    // Set up periodic collection
    if (!statsInterval.current) {
      statsInterval.current = setInterval(async () => {
        // Collect stats for all connections
        for (const [agentId, connection] of connections.current) {
          if (connection.peerConnection) {
            try {
              const stats = await connection.peerConnection.getStats()
              const webrtcStats = parseRTCStats(stats)
              webrtcStats.connectionState = connection.peerConnection.connectionState
              setWebRTCStats(webrtcStats)
            } catch (error) {
              console.error(`Error collecting stats for ${agentId}:`, error)
            }
          }
        }
      }, 1000)
    }
  }

  const getConnectionStats = useCallback(async () => {
    const stats = {
      totalConnections: connections.current.size,
      activeDataChannels: 0,
      bytesTransferred: 0,
      packetsLost: 0
    }

    for (const [agentId, connection] of connections.current) {
      if (connection.dataChannel?.readyState === 'open') {
        stats.activeDataChannels++
      }

      try {
        const rtcStats = await connection.peerConnection.getStats()
        rtcStats.forEach(stat => {
          if (stat.type === 'data-channel') {
            stats.bytesTransferred += stat.bytesReceived || 0
            stats.bytesTransferred += stat.bytesSent || 0
          }
          if (stat.type === 'transport') {
            stats.packetsLost += stat.packetsLost || 0
          }
        })
      } catch (error) {
        console.error(`Error getting stats for ${agentId}:`, error)
      }
    }

    return stats
  }, [])

  const cleanup = useCallback(() => {
    if (statsInterval.current) {
      clearInterval(statsInterval.current)
      statsInterval.current = null
    }
    
    // Close all peer connections
    connections.current.forEach(connection => {
      connection.dataChannel?.close()
      connection.peerConnection.close()
    })
    connections.current.clear()
    
    if (websocketRef.current) {
      websocketRef.current.close()
      websocketRef.current = null
    }
  }, [])

  // Initialize WebSocket connection
  useEffect(() => {
    initializeWebSocket()
    
    return () => {
      cleanup()
    }
  }, [initializeWebSocket, cleanup])

  // Periodic stats collection
  useEffect(() => {
    if (!isWebRTCSupported) return

    const interval = setInterval(async () => {
      const stats = await getConnectionStats()
      updateNetworkMetrics({
        packetLoss: stats.packetsLost > 0 ? 0.01 : 0, // Simplified calculation
        bandwidth: stats.bytesTransferred * 8 / 1000 // Convert to kbps
      })
    }, 5000)

    return () => clearInterval(interval)
  }, [getConnectionStats, updateNetworkMetrics, isWebRTCSupported])

  // Expose sendCommand for use by other components
  useEffect(() => {
    setSendCommand(sendCommand)
  }, [sendCommand, setSendCommand])

  return {
    isWebRTCSupported,
    sendLowLatencyCommand,
    sendCommand,
    getConnectionStats,
    activeConnections: connections.current.size,
    isConnected: Array.from(connections.current.values()).some(
      conn => conn.dataChannel?.readyState === 'open'
    ) || websocketRef.current?.readyState === WebSocket.OPEN
  }
}
```
