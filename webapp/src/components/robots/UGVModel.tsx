import React, { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import { useSwarmStore } from '../../store/swarmStore'
import { Text } from '@react-three/drei'
import * as THREE from 'three'
import type { Agent } from '../../store/swarmStore'

interface UGVModelProps {
  agent: Agent
  isSelected: boolean
}

export default function UGVModel({ agent, isSelected }: UGVModelProps) {
  const meshRef = useRef<THREE.Group>(null)
  const wheelRefs = useRef<THREE.Mesh[]>([])
  const { selectAgent, deselectAgent } = useSwarmStore()

  // Animate wheels when moving
  useFrame((state, delta) => {
    if (wheelRefs.current && agent.status === 'active') {
      wheelRefs.current.forEach(wheel => {
        if (wheel) {
          wheel.rotation.x += delta * 5 // Wheel rotation
        }
      })
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
      position={[agent.position[0], agent.position[1] + 0.3, agent.position[2]]}
      rotation={agent.rotation}
      onClick={handleClick}
    >
      {/* Main chassis */}
      <mesh position={[0, 0, 0]}>
        <boxGeometry args={[1.5, 0.4, 1.0]} />
        <meshStandardMaterial 
          color={isSelected ? '#ffff00' : getStatusColor()}
          emissive={isSelected ? '#333300' : '#000000'}
        />
      </mesh>

      {/* Wheels */}
      {[
        [0.6, -0.3, 0.6],
        [0.6, -0.3, -0.6],
        [-0.6, -0.3, 0.6],
        [-0.6, -0.3, -0.6]
      ].map((position, index) => (
        <mesh
          key={index}
          ref={el => {
            if (el) wheelRefs.current[index] = el
          }}
          position={position as [number, number, number]}
          rotation={[Math.PI / 2, 0, 0]}
        >
          <cylinderGeometry args={[0.2, 0.2, 0.1]} />
          <meshStandardMaterial color="#222222" />
        </mesh>
      ))}

      {/* Sensor array on top */}
      <mesh position={[0, 0.3, 0]}>
        <cylinderGeometry args={[0.3, 0.3, 0.1]} />
        <meshStandardMaterial color="#444444" />
      </mesh>

      {/* LIDAR sensor */}
      <mesh position={[0, 0.4, 0]}>
        <cylinderGeometry args={[0.15, 0.15, 0.2]} />
        <meshStandardMaterial 
          color="#666666"
          emissive="#002200"
          emissiveIntensity={agent.status === 'active' ? 0.3 : 0}
        />
      </mesh>

      {/* Front LED array */}
      <mesh position={[0.8, 0, 0]}>
        <boxGeometry args={[0.1, 0.2, 0.6]} />
        <meshStandardMaterial 
          color={getStatusColor()} 
          emissive={getStatusColor()}
          emissiveIntensity={0.5}
        />
      </mesh>

      {/* Battery pack */}
      <mesh position={[0, -0.1, -0.3]}>
        <boxGeometry args={[0.8, 0.2, 0.4]} />
        <meshStandardMaterial 
          color={getBatteryColor()}
          emissive={getBatteryColor()}
          emissiveIntensity={0.2}
        />
      </mesh>

      {/* Antenna */}
      <mesh position={[-0.6, 0.3, 0]}>
        <cylinderGeometry args={[0.02, 0.02, 0.8]} />
        <meshStandardMaterial color="#333333" />
      </mesh>

      {/* Agent ID label */}
      <Text
        position={[0, 0.8, 0]}
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
        position={[0, -0.6, 0]}
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
        <mesh position={[0, -0.3, 0]} rotation={[Math.PI / 2, 0, 0]}>
          <ringGeometry args={[1.5, 1.7, 32]} />
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
        <mesh position={[0, 1.0, 0]}>
          <octahedronGeometry args={[0.3]} />
          <meshStandardMaterial 
            color="#ff0000" 
            emissive="#ff0000"
            emissiveIntensity={0.8}
          />
        </mesh>
      )}

      {/* Movement direction indicator */}
      {agent.status === 'active' && (
        <mesh position={[1.2, 0, 0]}>
          <coneGeometry args={[0.1, 0.3]} />
          <meshStandardMaterial 
            color="#00ff00"
            emissive="#00ff00"
            emissiveIntensity={0.5}
          />
        </mesh>
      )}
    </group>
  )
}