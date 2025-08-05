import { useEffect, useRef, useCallback, useState } from 'react'
import { useSwarmStore } from '../store/swarmStore'

interface WebRTCConnection {
  peerConnection: RTCPeerConnection
  dataChannel: RTCDataChannel | null
  remoteStream: MediaStream | null
}

export function useWebRTC() {
  const [isWebRTCSupported] = useState(() => {
    return typeof RTCPeerConnection !== 'undefined'
  })
  
  const connections = useRef<Map<string, WebRTCConnection>>(new Map())
  const localStreamRef = useRef<MediaStream | null>(null)
  const websocketRef = useRef<WebSocket | null>(null)
  
  const { websocketUrl, updateNetworkMetrics, setConnection } = useSwarmStore()

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
      // Attempt reconnection after delay
      setTimeout(initializeWebSocket, 5000)
    }

    ws.onerror = (error) => {
      console.error('WebRTC signaling WebSocket error:', error)
    }
  }, [websocketUrl, isWebRTCSupported])

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
        // Start measuring latency
        measureLatency(agentId)
      } else if (state === 'disconnected' || state === 'failed') {
        connections.current.delete(agentId)
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
  }, [])

  const setupDataChannel = (dataChannel: RTCDataChannel, agentId: string) => {
    dataChannel.onopen = () => {
      console.log(`Data channel opened for ${agentId}`)
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
    }

    dataChannel.onerror = (error) => {
      console.error(`Data channel error for ${agentId}:`, error)
    }
  }

  const handleDataChannelMessage = (agentId: string, data: any) => {
    if (data.type === 'telemetry') {
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
    } else if (data.type === 'video_frame') {
      // Handle compressed video frames
      handleVideoFrame(agentId, data)
    }
  }

  const handleVideoFrame = (agentId: string, frameData: any) => {
    // Decode and display video frame
    // This would typically involve creating a video element or canvas
    console.log(`Received video frame from ${agentId}`, frameData.width, 'x', frameData.height)
  }

  const handleSignalingMessage = async (message: any) => {
    const { type, agentId } = message

    switch (type) {
      case 'webrtc_offer':
        await handleOffer(agentId, message)
        break
      case 'webrtc_answer':
        await handleAnswer(agentId, message)
        break
      case 'ice_candidate':
        await handleIceCandidate(agentId, message)
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
    if (websocketRef.current) {
      websocketRef.current.send(JSON.stringify({
        type: 'agent_command',
        agent_id: agentId,
        command
      }))
      return 'websocket'
    }
    
    return null
  }, [sendDataChannelMessage])

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

  // Initialize WebSocket connection
  useEffect(() => {
    initializeWebSocket()
    
    return () => {
      if (websocketRef.current) {
        websocketRef.current.close()
      }
      
      // Close all peer connections
      connections.current.forEach(connection => {
        connection.peerConnection.close()
        connection.dataChannel?.close()
      })
      connections.current.clear()
    }
  }, [initializeWebSocket])

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

  return {
    isWebRTCSupported,
    sendLowLatencyCommand,
    getConnectionStats,
    activeConnections: connections.current.size
  }
}