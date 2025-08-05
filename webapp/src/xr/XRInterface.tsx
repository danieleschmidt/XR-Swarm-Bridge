import React, { useRef } from 'react'
import { useXR, Interactive } from '@react-three/xr'
import { useFrame } from '@react-three/fiber'
import { useSwarmStore } from '../store/swarmStore'
import XRControlPanel from './XRControlPanel'
import XRGestureHandler from './XRGestureHandler'
import XRMiniMap from './XRMiniMap'
import * as THREE from 'three'

export default function XRInterface() {
  const { isPresenting, player } = useXR()
  const controllerRef = useRef<THREE.Group>(null)
  const { selectedAgents, agents } = useSwarmStore()

  // Handle controller input for agent selection and commands
  useFrame(() => {
    if (!isPresenting || !controllerRef.current) return
    
    // Controller-based interaction logic would go here
    // For now, we'll use simplified hand tracking simulation
  })

  if (!isPresenting) return null

  return (
    <group>
      {/* XR Control Panel - floating interface */}
      <group position={[-2, 1.5, -1]} rotation={[0, Math.PI/4, 0]}>
        <XRControlPanel />
      </group>
      
      {/* Mini-map in XR space */}
      <group position={[2, 1.5, -1]} rotation={[0, -Math.PI/4, 0]}>
        <XRMiniMap />
      </group>
      
      {/* Gesture handler for hand tracking */}
      <XRGestureHandler />
      
      {/* Interactive waypoint placement */}
      <InteractiveSpace />
      
      {/* Agent information displays in 3D space */}
      {selectedAgents.map(agentId => {
        const agent = agents[agentId]
        if (!agent) return null
        
        return (
          <group 
            key={agentId} 
            position={[agent.position[0], agent.position[1] + 1, agent.position[2]]}
          >
            <AgentInfoPanel agent={agent} />
          </group>
        )
      })}
    </group>
  )
}

function InteractiveSpace() {
  const { isPresenting } = useXR()
  const planeRef = useRef<THREE.Mesh>(null)

  const handleSelect = (event: any) => {
    const intersection = event.intersection
    if (intersection) {
      // Place waypoint at selected position
      useSwarmStore.getState().addWaypoint(intersection.point.toArray())
    }
  }

  if (!isPresenting) return null

  return (
    <Interactive onSelect={handleSelect}>
      <mesh ref={planeRef} rotation={[-Math.PI/2, 0, 0]} position={[0, 0, 0]}>
        <planeGeometry args={[100, 100]} />
        <meshBasicMaterial transparent opacity={0} />
      </mesh>
    </Interactive>
  )
}

function AgentInfoPanel({ agent }: { agent: any }) {
  return (
    <group>
      {/* Info background */}
      <mesh>
        <planeGeometry args={[1, 0.6]} />
        <meshBasicMaterial color="#000000" transparent opacity={0.8} />
      </mesh>
      
      {/* Status indicator */}
      <mesh position={[0, 0.2, 0.01]}>
        <circleGeometry args={[0.05, 16]} />
        <meshBasicMaterial color={getStatusColor(agent.status)} />
      </mesh>
      
      {/* Battery indicator */}
      <mesh position={[0, 0, 0.01]}>
        <planeGeometry args={[0.6, 0.1]} />
        <meshBasicMaterial color="#333333" />
      </mesh>
      <mesh position={[-0.3 + (agent.battery_level / 100) * 0.3, 0, 0.02]}>
        <planeGeometry args={[(agent.battery_level / 100) * 0.6, 0.08]} />
        <meshBasicMaterial color={getBatteryColor(agent.battery_level)} />
      </mesh>
      
      {/* Agent ID text would go here - simplified for now */}
      <mesh position={[0, -0.2, 0.01]}>
        <planeGeometry args={[0.8, 0.1]} />
        <meshBasicMaterial color="#ffffff" />
      </mesh>
    </group>
  )
}

function getStatusColor(status: string): string {
  switch (status) {
    case 'active': return '#00ff00'
    case 'idle': return '#0066ff'
    case 'error': return '#ff0000'
    case 'emergency_stop': return '#ff6600'
    default: return '#999999'
  }
}

function getBatteryColor(level: number): string {
  if (level > 60) return '#00ff00'
  if (level > 30) return '#ffff00'
  return '#ff0000'
}