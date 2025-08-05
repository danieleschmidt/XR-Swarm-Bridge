import React, { useMemo } from 'react'
import { useSwarmStore } from '../store/swarmStore'
import * as THREE from 'three'

export default function TrajectoryLines() {
  const { selectedAgents, agents, trajectories } = useSwarmStore(state => ({
    selectedAgents: state.selectedAgents,
    agents: state.agents,
    trajectories: state.trajectories
  }))

  const trajectoryLines = useMemo(() => {
    return selectedAgents.map(agentId => {
      const trajectory = trajectories[agentId]
      if (!trajectory || trajectory.length < 2) return null

      const points = trajectory.map(point => new THREE.Vector3(...point))
      const geometry = new THREE.BufferGeometry().setFromPoints(points)
      
      return {
        agentId,
        geometry,
        points: trajectory
      }
    }).filter(Boolean)
  }, [selectedAgents, trajectories])

  return (
    <group>
      {trajectoryLines.map((line, index) => (
        <line key={line!.agentId} geometry={line!.geometry}>
          <lineBasicMaterial 
            color={getTrajectoryColor(index)} 
            linewidth={2}
            transparent
            opacity={0.7}
          />
        </line>
      ))}
      
      {/* Waypoints for selected agents */}
      {selectedAgents.map(agentId => {
        const agent = agents[agentId]
        if (!agent?.waypoints) return null
        
        return agent.waypoints.map((waypoint: number[], index: number) => (
          <mesh key={`${agentId}-waypoint-${index}`} position={waypoint}>
            <sphereGeometry args={[0.1, 8, 8]} />
            <meshStandardMaterial 
              color="#ffff00" 
              emissive="#333300"
              transparent
              opacity={0.8}
            />
          </mesh>
        ))
      })}
    </group>
  )
}

function getTrajectoryColor(index: number): string {
  const colors = ['#ff0000', '#00ff00', '#0000ff', '#ffff00', '#ff00ff', '#00ffff']
  return colors[index % colors.length]
}