import { useEffect, useRef } from 'react'
import { useSwarmStore } from '../store/swarmStore'

interface WebRTCStats {
  latency: number
  packetLoss: number
  bandwidth: number
  connectionState: RTCPeerConnectionState
}

export function useWebRTC() {
  const peerConnection = useRef<RTCPeerConnection | null>(null)
  const dataChannel = useRef<RTCDataChannel | null>(null)
  const websocket = useRef<WebSocket | null>(null)
  const statsInterval = useRef<NodeJS.Timeout | null>(null)
  
  const { setWebRTCStats, setConnectionStatus } = useSwarmStore()

  useEffect(() => {
    initializeWebRTC()
    
    return () => {
      cleanup()
    }
  }, [])

  const initializeWebRTC = async () => {
    try {
      // Initialize WebSocket for signaling
      const ws = new WebSocket('ws://localhost:8443')
      websocket.current = ws
      
      ws.onopen = () => {
        console.log('WebSocket connected')
        setConnectionStatus('websocket_connected')
        initializePeerConnection()
      }
      
      ws.onmessage = (event) => {
        handleSignalingMessage(JSON.parse(event.data))
      }
      
      ws.onclose = () => {
        console.log('WebSocket disconnected')
        setConnectionStatus('disconnected')
      }
      
      ws.onerror = (error) => {
        console.error('WebSocket error:', error)
        setConnectionStatus('error')
      }
      
    } catch (error) {
      console.error('Failed to initialize WebRTC:', error)
      setConnectionStatus('error')
    }
  }

  const initializePeerConnection = () => {
    const pc = new RTCPeerConnection({
      iceServers: [
        { urls: 'stun:stun.l.google.com:19302' },
        { urls: 'stun:stun1.l.google.com:19302' }
      ]
    })
    
    peerConnection.current = pc
    
    // Create data channel for low-latency communication
    const dc = pc.createDataChannel('swarm_control', {
      ordered: false, // Allow out-of-order delivery for lower latency
      maxRetransmits: 0 // Don't retransmit for real-time data
    })
    
    dataChannel.current = dc
    
    dc.onopen = () => {
      console.log('Data channel opened')
      setConnectionStatus('webrtc_connected')
      startStatsCollection()
    }
    
    dc.onclose = () => {
      console.log('Data channel closed')
      setConnectionStatus('websocket_connected')
    }
    
    dc.onmessage = (event) => {
      handleDataChannelMessage(event.data)
    }
    
    pc.onicecandidate = (event) => {
      if (event.candidate && websocket.current) {
        websocket.current.send(JSON.stringify({
          type: 'ice_candidate',
          candidate: event.candidate
        }))
      }
    }
    
    pc.onconnectionstatechange = () => {
      console.log('Connection state:', pc.connectionState)
      if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
        setConnectionStatus('disconnected')
        cleanup()
        // Attempt reconnection after delay
        setTimeout(initializeWebRTC, 5000)
      }
    }
    
    // Create offer
    createOffer()
  }

  const createOffer = async () => {
    if (!peerConnection.current || !websocket.current) return
    
    try {
      const offer = await peerConnection.current.createOffer()
      await peerConnection.current.setLocalDescription(offer)
      
      websocket.current.send(JSON.stringify({
        type: 'webrtc_offer',
        sdp: offer.sdp,
        type: offer.type
      }))
    } catch (error) {
      console.error('Failed to create offer:', error)
    }
  }

  const handleSignalingMessage = async (message: any) => {
    if (!peerConnection.current) return
    
    try {
      switch (message.type) {
        case 'webrtc_answer':
          await peerConnection.current.setRemoteDescription(
            new RTCSessionDescription({
              type: message.type,
              sdp: message.sdp
            })
          )
          break
          
        case 'ice_candidate':
          if (message.candidate) {
            await peerConnection.current.addIceCandidate(
              new RTCIceCandidate(message.candidate)
            )
          }
          break
          
        case 'agent_telemetry':
          // Handle incoming telemetry data
          useSwarmStore.getState().updateAgentTelemetry(message.data)
          break
          
        case 'error':
          console.error('Server error:', message.message)
          break
      }
    } catch (error) {
      console.error('Error handling signaling message:', error)
    }
  }

  const handleDataChannelMessage = (data: string) => {
    try {
      const message = JSON.parse(data)
      
      switch (message.type) {
        case 'agent_telemetry':
          useSwarmStore.getState().updateAgentTelemetry(message.data)
          break
          
        case 'video_frame':
          useSwarmStore.getState().updateAgentVideo(message.agent_id, message.data)
          break
          
        case 'heartbeat':
          // Update agent heartbeat
          useSwarmStore.getState().updateAgentHeartbeat(message.agent_id, message.timestamp)
          break
      }
    } catch (error) {
      console.error('Error parsing data channel message:', error)
    }
  }

  const sendCommand = (command: any) => {
    if (dataChannel.current?.readyState === 'open') {
      // Send via WebRTC data channel for low latency
      dataChannel.current.send(JSON.stringify({
        type: 'command',
        ...command,
        timestamp: Date.now()
      }))
    } else if (websocket.current?.readyState === WebSocket.OPEN) {
      // Fallback to WebSocket
      websocket.current.send(JSON.stringify({
        type: 'swarm_command',
        ...command,
        timestamp: Date.now()
      }))
    } else {
      console.warn('No connection available for sending command')
    }
  }

  const startStatsCollection = () => {
    if (statsInterval.current) {
      clearInterval(statsInterval.current)
    }
    
    statsInterval.current = setInterval(async () => {
      if (!peerConnection.current) return
      
      try {
        const stats = await peerConnection.current.getStats()
        const webrtcStats = parseRTCStats(stats)
        setWebRTCStats(webrtcStats)
      } catch (error) {
        console.error('Error collecting WebRTC stats:', error)
      }
    }, 1000)
  }

  const parseRTCStats = (stats: RTCStatsReport): WebRTCStats => {
    let latency = 0
    let packetLoss = 0
    let bandwidth = 0
    
    stats.forEach((report) => {
      if (report.type === 'candidate-pair' && report.state === 'succeeded') {
        latency = report.currentRoundTripTime ? report.currentRoundTripTime * 1000 : 0
      }
      
      if (report.type === 'inbound-rtp') {
        const packetsLost = report.packetsLost || 0
        const packetsReceived = report.packetsReceived || 0
        const totalPackets = packetsLost + packetsReceived
        packetLoss = totalPackets > 0 ? packetsLost / totalPackets : 0
      }
      
      if (report.type === 'candidate-pair' && report.state === 'succeeded') {
        bandwidth = report.availableIncomingBitrate || 0
      }
    })
    
    return {
      latency,
      packetLoss,
      bandwidth,
      connectionState: peerConnection.current?.connectionState || 'new'
    }
  }

  const cleanup = () => {
    if (statsInterval.current) {
      clearInterval(statsInterval.current)
      statsInterval.current = null
    }
    
    if (dataChannel.current) {
      dataChannel.current.close()
      dataChannel.current = null
    }
    
    if (peerConnection.current) {
      peerConnection.current.close()
      peerConnection.current = null
    }
    
    if (websocket.current) {
      websocket.current.close()
      websocket.current = null
    }
  }

  // Expose sendCommand for use by other components
  useEffect(() => {
    useSwarmStore.getState().setSendCommand(sendCommand)
  }, [])

  return {
    sendCommand,
    isConnected: dataChannel.current?.readyState === 'open' || 
                 websocket.current?.readyState === WebSocket.OPEN
  }
}