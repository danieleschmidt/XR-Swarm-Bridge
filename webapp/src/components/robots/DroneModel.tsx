import React, { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import { useSwarmStore } from '../../store/swarmStore'
import * as THREE from 'three'

interface DroneModelProps {
  agent: any
  isSelected: boolean
}

export default function DroneModel({ agent, isSelected }: DroneModelProps) {
  const meshRef = useRef<THREE.Group>(null)
  
  // Animate propellers
  useFrame((state, delta) => {
    if (meshRef.current && agent.status === 'active') {
      const propellers = meshRef.current.children.filter(child => 
        child.userData.type === 'propeller'
      )
      propellers.forEach(propeller => {
        propeller.rotation.y += delta * 50 // Fast spinning
      })
    }
  })

  const handleClick = (event: any) => {
    event.stopPropagation()
    useSwarmStore.getState().selectAgent(agent.id)
  }

  return (
    <group
      ref={meshRef}
      position={agent.position}
      rotation={agent.rotation || [0, 0, 0]}
      onClick={handleClick}
    >
      {/* Main body */}
      <mesh>
        <boxGeometry args={[0.3, 0.1, 0.3]} />
        <meshStandardMaterial 
          color={isSelected ? '#00ff00' : getStatusColor(agent.status)}
          emissive={isSelected ? '#003300' : '#000000'}
        />
      </mesh>
      
      {/* Arms */}
      <mesh position={[0.2, 0, 0.2]}>
        <cylinderGeometry args={[0.02, 0.02, 0.05, 8]} />
        <meshStandardMaterial color="#333333" />
      </mesh>
      <mesh position={[-0.2, 0, 0.2]}>
        <cylinderGeometry args={[0.02, 0.02, 0.05, 8]} />
        <meshStandardMaterial color="#333333" />
      </mesh>
      <mesh position={[0.2, 0, -0.2]}>
        <cylinderGeometry args={[0.02, 0.02, 0.05, 8]} />
        <meshStandardMaterial color="#333333" />
      </mesh>
      <mesh position={[-0.2, 0, -0.2]}>
        <cylinderGeometry args={[0.02, 0.02, 0.05, 8]} />
        <meshStandardMaterial color="#333333" />
      </mesh>
      
      {/* Propellers */}
      <mesh position={[0.2, 0.05, 0.2]} userData={{ type: 'propeller' }}>
        <cylinderGeometry args={[0.08, 0.08, 0.01, 8]} />
        <meshStandardMaterial color="#666666" transparent opacity={0.7} />
      </mesh>
      <mesh position={[-0.2, 0.05, 0.2]} userData={{ type: 'propeller' }}>
        <cylinderGeometry args={[0.08, 0.08, 0.01, 8]} />
        <meshStandardMaterial color="#666666" transparent opacity={0.7} />
      </mesh>
      <mesh position={[0.2, 0.05, -0.2]} userData={{ type: 'propeller' }}>
        <cylinderGeometry args={[0.08, 0.08, 0.01, 8]} />
        <meshStandardMaterial color="#666666" transparent opacity={0.7} />
      </mesh>
      <mesh position={[-0.2, 0.05, -0.2]} userData={{ type: 'propeller' }}>
        <cylinderGeometry args={[0.08, 0.08, 0.01, 8]} />
        <meshStandardMaterial color="#666666" transparent opacity={0.7} />
      </mesh>
      
      {/* Battery indicator */}
      <mesh position={[0, 0.1, 0]}>
        <cylinderGeometry args={[0.02, 0.02, getBatteryHeight(agent.battery_level), 8]} />
        <meshStandardMaterial color={getBatteryColor(agent.battery_level)} />
      </mesh>
      
      {/* Agent ID label */}
      <mesh position={[0, 0.2, 0]}>
        <planeGeometry args={[0.4, 0.1]} />
        <meshBasicMaterial color="#ffffff" transparent opacity={0.8} />
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
    case 'timeout': return '#666666'
    default: return '#999999'
  }
}

function getBatteryColor(level: number): string {
  if (level > 60) return '#00ff00'
  if (level > 30) return '#ffff00'
  return '#ff0000'
}

function getBatteryHeight(level: number): number {
  return Math.max(0.01, (level / 100) * 0.1)
}