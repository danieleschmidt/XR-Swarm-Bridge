import React, { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import { useSwarmStore } from '../../store/swarmStore'
import * as THREE from 'three'

interface UGVModelProps {
  agent: any
  isSelected: boolean
}

export default function UGVModel({ agent, isSelected }: UGVModelProps) {
  const meshRef = useRef<THREE.Group>(null)
  const wheelRefs = useRef<THREE.Mesh[]>([])
  
  // Animate wheels based on movement
  useFrame((state, delta) => {
    if (meshRef.current && agent.status === 'active') {
      const speed = agent.velocity ? Math.sqrt(
        agent.velocity[0]**2 + agent.velocity[1]**2 + agent.velocity[2]**2
      ) : 0
      
      wheelRefs.current.forEach(wheel => {
        if (wheel) {
          wheel.rotation.x += delta * speed * 10
        }
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
      {/* Main chassis */}
      <mesh>
        <boxGeometry args={[0.6, 0.2, 0.4]} />
        <meshStandardMaterial 
          color={isSelected ? '#00ff00' : getStatusColor(agent.status)}
          emissive={isSelected ? '#003300' : '#000000'}
        />
      </mesh>
      
      {/* Wheels */}
      <mesh 
        position={[0.25, -0.15, 0.15]} 
        rotation={[Math.PI/2, 0, 0]}
        ref={el => { if (el) wheelRefs.current[0] = el }}
      >
        <cylinderGeometry args={[0.08, 0.08, 0.05, 16]} />
        <meshStandardMaterial color="#222222" />
      </mesh>
      <mesh 
        position={[0.25, -0.15, -0.15]} 
        rotation={[Math.PI/2, 0, 0]}
        ref={el => { if (el) wheelRefs.current[1] = el }}
      >
        <cylinderGeometry args={[0.08, 0.08, 0.05, 16]} />
        <meshStandardMaterial color="#222222" />
      </mesh>
      <mesh 
        position={[-0.25, -0.15, 0.15]} 
        rotation={[Math.PI/2, 0, 0]}
        ref={el => { if (el) wheelRefs.current[2] = el }}
      >
        <cylinderGeometry args={[0.08, 0.08, 0.05, 16]} />
        <meshStandardMaterial color="#222222" />
      </mesh>
      <mesh 
        position={[-0.25, -0.15, -0.15]} 
        rotation={[Math.PI/2, 0, 0]}
        ref={el => { if (el) wheelRefs.current[3] = el }}
      >
        <cylinderGeometry args={[0.08, 0.08, 0.05, 16]} />
        <meshStandardMaterial color="#222222" />
      </mesh>
      
      {/* Sensor turret */}
      <mesh position={[0, 0.15, 0]}>
        <cylinderGeometry args={[0.1, 0.1, 0.1, 16]} />
        <meshStandardMaterial color="#444444" />
      </mesh>
      
      {/* Sensor */}
      <mesh position={[0.1, 0.2, 0]}>
        <sphereGeometry args={[0.03, 16, 16]} />
        <meshStandardMaterial color="#ff0000" emissive="#330000" />
      </mesh>
      
      {/* Battery indicator */}
      <mesh position={[0, 0.12, -0.15]}>
        <boxGeometry args={[0.1, getBatteryHeight(agent.battery_level), 0.05]} />
        <meshStandardMaterial color={getBatteryColor(agent.battery_level)} />
      </mesh>
      
      {/* Agent ID label */}
      <mesh position={[0, 0.3, 0]} rotation={[-Math.PI/2, 0, 0]}>
        <planeGeometry args={[0.5, 0.1]} />
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