import React, { useRef, useEffect } from 'react'
import { useFrame } from '@react-three/fiber'
import { useSwarmStore } from '../../store/swarmStore'
import { Text } from '@react-three/drei'
import * as THREE from 'three'
import type { Agent } from '../../store/swarmStore'

interface DroneModelProps {
  agent: Agent
  isSelected: boolean
}

export default function DroneModel({ agent, isSelected }: DroneModelProps) {
  const meshRef = useRef<THREE.Group>(null)
  const propellerRefs = useRef<THREE.Mesh[]>([])
  const { selectAgent, deselectAgent } = useSwarmStore()

  // Animate propellers
  useFrame((state, delta) => {
    if (propellerRefs.current) {
      propellerRefs.current.forEach(propeller => {
        if (propeller && agent.status === 'active') {
          propeller.rotation.y += delta * 50 // Fast rotation when active
        }
      })
    }
    
    // Smooth hover animation for drones
    if (meshRef.current && agent.status === 'active') {
      meshRef.current.position.y = agent.position[1] + Math.sin(state.clock.elapsedTime * 2) * 0.1
    }
  })

  const handleClick = (event: any) => {
    event.stopPropagation()
    if (isSelected) {
      deselectAgent(agent.id)
    } else {
      selectAgent(agent.id)
    }
  }

  const getStatusColor = () => {
    switch (agent.status) {
      case 'active': return '#00ff00'
      case 'idle': return '#0066ff'
      case 'error': return '#ff0000'
      case 'emergency_stop': return '#ff6600'
      case 'timeout': return '#666666'
      default: return '#999999'
    }
  }

  const getBatteryColor = () => {
    if (agent.battery > 60) return '#00ff00'
    if (agent.battery > 30) return '#ffff00'
    return '#ff0000'
  }

  return (
    <group 
      ref={meshRef}
      position={[agent.position[0], agent.position[1], agent.position[2]]}
      rotation={agent.rotation}
      onClick={handleClick}
    >
      {/* Main body */}
      <mesh>
        <boxGeometry args={[0.8, 0.2, 0.8]} />
        <meshStandardMaterial 
          color={isSelected ? '#ffff00' : getStatusColor()}
          emissive={isSelected ? '#333300' : '#000000'}
        />
      </mesh>

      {/* Propeller arms */}
      <mesh position={[0.5, 0, 0.5]}>
        <cylinderGeometry args={[0.02, 0.02, 0.6]} />
        <meshStandardMaterial color="#333333" />
      </mesh>
      <mesh position={[-0.5, 0, -0.5]}>
        <cylinderGeometry args={[0.02, 0.02, 0.6]} />
        <meshStandardMaterial color="#333333" />
      </mesh>
      <mesh position={[0.5, 0, -0.5]}>
        <cylinderGeometry args={[0.02, 0.02, 0.6]} />
        <meshStandardMaterial color="#333333" />
      </mesh>
      <mesh position={[-0.5, 0, 0.5]}>
        <cylinderGeometry args={[0.02, 0.02, 0.6]} />
        <meshStandardMaterial color="#333333" />
      </mesh>

      {/* Propellers */}
      {[
        [0.5, 0.15, 0.5],
        [-0.5, 0.15, -0.5],
        [0.5, 0.15, -0.5],
        [-0.5, 0.15, 0.5]
      ].map((position, index) => (
        <mesh
          key={index}
          ref={el => {
            if (el) propellerRefs.current[index] = el
          }}
          position={position as [number, number, number]}
        >
          <cylinderGeometry args={[0.25, 0.25, 0.02]} />
          <meshStandardMaterial 
            color="#cccccc" 
            transparent 
            opacity={agent.status === 'active' ? 0.3 : 0.8}
          />
        </mesh>
      ))}

      {/* LED indicators */}
      <mesh position={[0, 0.15, 0.4]}>
        <sphereGeometry args={[0.05]} />
        <meshStandardMaterial 
          color={getStatusColor()} 
          emissive={getStatusColor()}
          emissiveIntensity={0.5}
        />
      </mesh>

      {/* Battery indicator */}
      <mesh position={[0, 0.15, -0.4]}>
        <boxGeometry args={[0.2, 0.05, 0.1]} />
        <meshStandardMaterial 
          color={getBatteryColor()}
          emissive={getBatteryColor()}
          emissiveIntensity={0.3}
        />
      </mesh>

      {/* Agent ID label */}
      <Text
        position={[0, 0.5, 0]}
        fontSize={0.2}
        color="white"
        anchorX="center"
        anchorY="middle"
        font="/fonts/inter-medium.woff"
      >
        {agent.id}
      </Text>

      {/* Battery percentage */}
      <Text
        position={[0, -0.3, 0]}
        fontSize={0.15}
        color={getBatteryColor()}
        anchorX="center"
        anchorY="middle"
        font="/fonts/inter-medium.woff"
      >
        {Math.round(agent.battery)}%
      </Text>

      {/* Selection indicator */}
      {isSelected && (
        <mesh position={[0, -0.1, 0]}>
          <ringGeometry args={[1.2, 1.4, 32]} />
          <meshBasicMaterial 
            color="#ffff00" 
            transparent 
            opacity={0.6}
            side={THREE.DoubleSide}
          />
        </mesh>
      )}

      {/* Status indicator for emergency/error states */}
      {(agent.status === 'emergency_stop' || agent.status === 'error') && (
        <mesh position={[0, 0.8, 0]}>
          <octahedronGeometry args={[0.2]} />
          <meshStandardMaterial 
            color="#ff0000" 
            emissive="#ff0000"
            emissiveIntensity={0.8}
          />
        </mesh>
      )}
    </group>
  )
}
