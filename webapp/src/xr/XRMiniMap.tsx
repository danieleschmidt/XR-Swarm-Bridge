import React, { useMemo } from 'react'
import { useSwarmStore } from '../store/swarmStore'
import { Text } from '@react-three/drei'
import * as THREE from 'three'

export default function XRMiniMap() {
  const { agents, selectedAgents } = useSwarmStore()
  
  const agentPositions = useMemo(() => {
    return Object.values(agents).map(agent => ({
      id: agent.id,
      position: agent.position,
      status: agent.status,
      type: agent.type,
      isSelected: selectedAgents.includes(agent.id)
    }))
  }, [agents, selectedAgents])

  const mapScale = 0.05 // Scale down real world coordinates for minimap

  return (
    <group>
      {/* Minimap background */}
      <mesh rotation={[-Math.PI/2, 0, 0]}>
        <planeGeometry args={[1.5, 1.5]} />
        <meshBasicMaterial color="#000000" transparent opacity={0.8} />
      </mesh>
      
      {/* Minimap border */}
      <mesh rotation={[-Math.PI/2, 0, 0]} position={[0, 0.001, 0]}>
        <ringGeometry args={[0.74, 0.75, 32]} />
        <meshBasicMaterial color="#ffffff" />
      </mesh>
      
      {/* Title */}
      <Text
        position={[0, 0.9, 0]}
        fontSize={0.08}
        color="white"
        anchorX="center"
      >
        SWARM MAP
      </Text>
      
      {/* Center indicator */}
      <mesh rotation={[-Math.PI/2, 0, 0]} position={[0, 0.002, 0]}>
        <circleGeometry args={[0.02, 16]} />
        <meshBasicMaterial color="#ffffff" />
      </mesh>
      
      {/* Agent indicators */}
      {agentPositions.map(agent => {
        const mapX = agent.position[0] * mapScale
        const mapZ = agent.position[2] * mapScale
        
        // Clamp to minimap bounds
        const clampedX = Math.max(-0.7, Math.min(0.7, mapX))
        const clampedZ = Math.max(-0.7, Math.min(0.7, mapZ))
        
        return (
          <group key={agent.id}>
            {/* Agent dot */}
            <mesh 
              rotation={[-Math.PI/2, 0, 0]} 
              position={[clampedX, 0.003, clampedZ]}
            >
              <circleGeometry args={[agent.isSelected ? 0.03 : 0.02, 8]} />
              <meshBasicMaterial color={getAgentMapColor(agent)} />
            </mesh>
            
            {/* Selection ring */}
            {agent.isSelected && (
              <mesh 
                rotation={[-Math.PI/2, 0, 0]} 
                position={[clampedX, 0.004, clampedZ]}
              >
                <ringGeometry args={[0.025, 0.035, 16]} />
                <meshBasicMaterial color="#00ff00" />
              </mesh>
            )}
            
            {/* Agent type indicator */}
            <mesh 
              rotation={[-Math.PI/2, 0, 0]} 
              position={[clampedX, 0.005, clampedZ]}
            >
              {agent.type === 'drone' ? (
                <boxGeometry args={[0.01, 0.01, 0.005]} />
              ) : (
                <cylinderGeometry args={[0.008, 0.008, 0.005, 4]} />
              )}
              <meshBasicMaterial color="#ffffff" />
            </mesh>
          </group>
        )
      })}
      
      {/* Compass */}
      <group position={[0.5, 0.01, 0.5]}>
        <Text
          position={[0, 0, 0.05]}
          fontSize={0.04}
          color="white"
          anchorX="center"
        >
          N
        </Text>
        <Text
          position={[0.05, 0, 0]}
          fontSize={0.04}
          color="white"
          anchorX="center"
        >
          E
        </Text>
        <Text
          position={[0, 0, -0.05]}
          fontSize={0.04}
          color="white"
          anchorX="center"
        >
          S
        </Text>
        <Text
          position={[-0.05, 0, 0]}
          fontSize={0.04}
          color="white"
          anchorX="center"
        >
          W
        </Text>
      </group>
      
      {/* Stats */}
      <Text
        position={[0, -0.9, 0]}
        fontSize={0.06}
        color="white"
        anchorX="center"
      >
        Agents: {agentPositions.length} | Selected: {selectedAgents.length}
      </Text>
    </group>
  )
}

function getAgentMapColor(agent: any): string {
  if (agent.isSelected) return '#00ff00'
  
  switch (agent.status) {
    case 'active': return '#00aa00'
    case 'idle': return '#0066aa'
    case 'error': return '#aa0000'
    case 'emergency_stop': return '#aa4400'
    default: return '#666666'
  }
}