import React, { useRef, useMemo } from 'react'
import { useFrame } from '@react-three/fiber'
import { useSwarmStore, useActiveAgents } from '../store/swarmStore'
import DroneModel from './robots/DroneModel'
import UGVModel from './robots/UGVModel'
import SelectionBox from './SelectionBox'
import TrajectoryLines from './TrajectoryLines'
import GroundPlane from './GroundPlane'
import * as THREE from 'three'

export default function SwarmScene() {
  const activeAgents = useActiveAgents()
  const selectedAgents = useSwarmStore(state => state.selectedAgents)
  const groupRef = useRef<THREE.Group>(null)

  // Update scene based on agent positions
  useFrame(() => {
    if (groupRef.current && activeAgents.length > 0) {
      // Calculate center of swarm for camera following
      const center = new THREE.Vector3()
      activeAgents.forEach(agent => {
        center.add(new THREE.Vector3(...agent.position))
      })
      center.divideScalar(activeAgents.length)
      
      // Store center for other components to use
      groupRef.current.userData.swarmCenter = center
    }
  })

  const renderAgent = (agent: any) => {
    const isSelected = selectedAgents.includes(agent.id)
    
    switch (agent.type) {
      case 'drone':
        return (
          <DroneModel
            key={agent.id}
            agent={agent}
            isSelected={isSelected}
          />
        )
      case 'ugv':
        return (
          <UGVModel
            key={agent.id}
            agent={agent}
            isSelected={isSelected}
          />
        )
      default:
        // Generic robot representation
        return (
          <mesh
            key={agent.id}
            position={agent.position}
            onClick={() => useSwarmStore.getState().selectAgent(agent.id)}
          >
            <boxGeometry args={[1, 1, 1]} />
            <meshStandardMaterial 
              color={isSelected ? '#00ff00' : getStatusColor(agent.status)} 
            />
          </mesh>
        )
    }
  }

  return (
    <group ref={groupRef}>
      {/* Ground plane */}
      <GroundPlane />
      
      {/* Render all active agents */}
      {activeAgents.map(renderAgent)}
      
      {/* Selection visualization */}
      <SelectionBox />
      
      {/* Trajectory lines for selected agents */}
      <TrajectoryLines />
      
      {/* Coordinate system helper */}
      <axesHelper args={[5]} />
      
      {/* Lighting helpers for development */}
      <gridHelper args={[100, 100]} />
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