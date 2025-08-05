```typescript
import React, { useMemo } from 'react'
import { useSwarmStore, useSelectedAgents } from '../store/swarmStore'
import { Line } from '@react-three/drei'
import * as THREE from 'three'

export default function TrajectoryLines() {
  const selectedAgents = useSelectedAgents()
  const { agents, trajectories } = useSwarmStore(state => ({
    agents: state.agents,
    trajectories: state.trajectories
  }))

  // Generate trajectory paths for selected agents
  const trajectoryPaths = useMemo(() => {
    return selectedAgents.map(agent => {
      const telemetry = agent.telemetry
      const waypoints = telemetry?.waypoints || []
      const currentPosition = agent.position
      const trajectory = trajectories[agent.id]
      
      // If we have trajectory data, use it
      if (trajectory && trajectory.length >= 2) {
        return {
          agentId: agent.id,
          points: trajectory,
          color: getAgentColor(agent.id),
          type: 'trajectory'
        }
      }
      
      // If no waypoints, create a simple predicted path
      if (waypoints.length === 0) {
        const velocity = telemetry?.velocity || [0, 0, 0]
        const predictedPoints = []
        
        for (let i = 0; i < 10; i++) {
          const time = i * 0.5 // 0.5 second intervals
          predictedPoints.push([
            currentPosition[0] + velocity[0] * time,
            currentPosition[1] + velocity[1] * time,
            currentPosition[2] + velocity[2] * time
          ])
        }
        
        return {
          agentId: agent.id,
          points: [currentPosition, ...predictedPoints],
          color: getAgentColor(agent.id),
          type: 'predicted'
        }
      }

      // Use planned waypoints
      return {
        agentId: agent.id,
        points: [currentPosition, ...waypoints.map((wp: any) => [wp.x, wp.y, wp.z])],
        color: getAgentColor(agent.id),
        type: 'planned'
      }
    })
  }, [selectedAgents, trajectories])

  // Historical trajectory paths (last N positions)
  const historicalPaths = useMemo(() => {
    return selectedAgents.map(agent => {
      const history = agent.telemetry?.position_history || []
      
      if (history.length < 2) return null

      return {
        agentId: agent.id,
        points: history.map((pos: any) => [pos.x, pos.y, pos.z]),
        color: getAgentColor(agent.id, 0.5),
        type: 'historical'
      }
    }).filter(Boolean)
  }, [selectedAgents])

  return (
    <group>
      {/* Future/planned trajectories */}
      {trajectoryPaths.map(path => (
        <Line
          key={`future-${path.agentId}`}
          points={path.points}
          color={path.color}
          lineWidth={path.type === 'planned' || path.type === 'trajectory' ? 3 : 2}
          dashed={path.type === 'predicted'}
          dashScale={50}
          dashSize={1}
          gapSize={0.5}
        />
      ))}

      {/* Historical trajectories */}
      {historicalPaths.map(path => path && (
        <Line
          key={`history-${path.agentId}`}
          points={path.points}
          color={path.color}
          lineWidth={1}
          transparent
          opacity={0.6}
        />
      ))}

      {/* Waypoint markers for selected agents */}
      {trajectoryPaths.map(path => 
        path.type === 'planned' && path.points.slice(1).map((point, index) => (
          <mesh
            key={`waypoint-${path.agentId}-${index}`}
            position={point as [number, number, number]}
          >
            <sphereGeometry args={[0.15]} />
            <meshStandardMaterial 
              color={path.color}
              emissive={path.color}
              emissiveIntensity={0.3}
              transparent
              opacity={0.8}
            />
          </mesh>
        ))
      )}

      {/* Distance markers along trajectory */}
      {trajectoryPaths.map(path => {
        if (path.points.length < 2) return null
        
        const totalDistance = calculatePathDistance(path.points)
        const markers = []
        let accumulatedDistance = 0
        
        for (let i = 1; i < path.points.length; i++) {
          const segmentDistance = new THREE.Vector3()
            .fromArray(path.points[i])
            .distanceTo(new THREE.Vector3().fromArray(path.points[i - 1]))
          
          accumulatedDistance += segmentDistance
          
          // Place marker every 10 units
          if (Math.floor(accumulatedDistance / 10) > Math.floor((accumulatedDistance - segmentDistance) / 10)) {
            markers.push(
              <mesh
                key={`marker-${path.agentId}-${i}`}
                position={path.points[i] as [number, number, number]}
              >
                <ringGeometry args={[0.3, 0.4, 8]} />
                <meshBasicMaterial 
                  color={path.color}
                  transparent
                  opacity={0.5}
                  side={THREE.DoubleSide}
                />
              </mesh>
            )
          }
        }
        
        return markers
      })}
    </group>
  )
}

function getAgentColor(agentId: string, alpha: number = 1): string {
  // Generate consistent color based on agent ID
  const hash = agentId.split('').reduce((a, b) => {
    a = ((a << 5) - a) + b.charCodeAt(0)
    return a & a
  }, 0)
  
  const hue = Math.abs(hash) % 360
  const saturation = 70
  const lightness = 50
  
  if (alpha < 1) {
    return `hsla(${hue}, ${saturation}%, ${lightness}%, ${alpha})`
  }
  
  return `hsl(${hue}, ${saturation}%, ${lightness}%)`
}

function calculatePathDistance(points: number[][]): number {
  let distance = 0
  for (let i = 1; i < points.length; i++) {
    const p1 = new THREE.Vector3().fromArray(points[i - 1])
    const p2 = new THREE.Vector3().fromArray(points[i])
    distance += p1.distanceTo(p2)
  }
  return distance
}
```
