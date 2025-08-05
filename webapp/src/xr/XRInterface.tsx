import React, { useRef } from 'react'
import { useXR, useController, useHitTest, Interactive } from '@react-three/xr'
import { useSwarmStore, useSelectedAgents } from '../store/swarmStore'
import { useSwarmConnection } from '../hooks/useSwarmConnection'
import { Text } from '@react-three/drei'
import * as THREE from 'three'

export default function XRInterface() {
  const { isPresenting, player } = useXR()
  const leftController = useController('left')
  const rightController = useController('right')
  const { sendGlobalCommand, sendAgentCommand } = useSwarmConnection()
  const selectedAgents = useSelectedAgents()
  const agents = useSwarmStore(state => state.agents)
  
  // XR-specific UI panels
  const leftPanelRef = useRef<THREE.Group>(null)
  const rightPanelRef = useRef<THREE.Group>(null)

  // Hit test for placing waypoints
  const hitTestResults = useHitTest()

  if (!isPresenting) return null

  const handleFormationCommand = (formation: string) => {
    if (selectedAgents.length === 0) {
      // Use all agents if none selected
      sendGlobalCommand({
        type: 'formation',
        formation,
        agents: Object.keys(agents)
      })
    } else {
      sendGlobalCommand({
        type: 'formation',
        formation,
        agents: selectedAgents.map(a => a.id)
      })
    }
  }

  const handleEmergencyStop = () => {
    sendGlobalCommand({
      type: 'emergency_stop'
    })
  }

  const handleReturnToBase = () => {
    sendGlobalCommand({
      type: 'return_to_base'
    })
  }

  return (
    <group>
      {/* Left controller UI panel */}
      {leftController && (
        <group
          ref={leftPanelRef}
          position={leftController.grip?.position || [0, 0, 0]}
          rotation={leftController.grip?.rotation || [0, 0, 0]}
        >
          <group position={[0, 0.1, -0.3]}>
            {/* Status panel background */}
            <mesh>
              <planeGeometry args={[0.4, 0.6]} />
              <meshBasicMaterial 
                color="#000000" 
                transparent 
                opacity={0.8}
              />
            </mesh>

            {/* Swarm status display */}
            <Text
              position={[0, 0.25, 0.01]}
              fontSize={0.03}
              color="#ffffff"
              anchorX="center"
              anchorY="middle"
              font="/fonts/inter-medium.woff"
            >
              SWARM STATUS
            </Text>

            <Text
              position={[0, 0.15, 0.01]}
              fontSize={0.025}
              color="#00ff00"
              anchorX="center"
              anchorY="middle"
              font="/fonts/inter-medium.woff"
            >
              Active: {Object.values(agents).filter(a => a.status === 'active').length}
            </Text>

            <Text
              position={[0, 0.05, 0.01]}
              fontSize={0.025}
              color="#ffff00"
              anchorX="center"
              anchorY="middle"
              font="/fonts/inter-medium.woff"
            >
              Selected: {selectedAgents.length}
            </Text>

            <Text
              position={[0, -0.05, 0.01]}
              fontSize={0.025}
              color="#ff6600"
              anchorX="center"
              anchorY="middle"
              font="/fonts/inter-medium.woff"
            >
              Total: {Object.keys(agents).length}
            </Text>

            {/* Network status */}
            <Text
              position={[0, -0.2, 0.01]}
              fontSize={0.02}
              color="#888888"
              anchorX="center"
              anchorY="middle"
              font="/fonts/inter-medium.woff"
            >
              Latency: {useSwarmStore.getState().latency}ms
            </Text>
          </group>
        </group>
      )}

      {/* Right controller command panel */}
      {rightController && (
        <group
          position={rightController.grip?.position || [0, 0, 0]}
          rotation={rightController.grip?.rotation || [0, 0, 0]}
        >
          <group position={[0, 0.1, -0.3]}>
            {/* Command panel background */}
            <mesh>
              <planeGeometry args={[0.5, 0.8]} />
              <meshBasicMaterial 
                color="#000000" 
                transparent 
                opacity={0.8}
              />
            </mesh>

            {/* Title */}
            <Text
              position={[0, 0.35, 0.01]}
              fontSize={0.03}
              color="#ffffff"
              anchorX="center"
              anchorY="middle"
              font="/fonts/inter-medium.woff"
            >
              COMMANDS
            </Text>

            {/* Formation commands */}
            <Interactive
              onSelect={() => handleFormationCommand('line')}
            >
              <mesh position={[0, 0.2, 0.01]}>
                <planeGeometry args={[0.4, 0.08]} />
                <meshBasicMaterial color="#0066ff" />
              </mesh>
              <Text
                position={[0, 0.2, 0.02]}
                fontSize={0.025}
                color="#ffffff"
                anchorX="center"
                anchorY="middle"
                font="/fonts/inter-medium.woff"
              >
                LINE FORMATION
              </Text>
            </Interactive>

            <Interactive
              onSelect={() => handleFormationCommand('grid')}
            >
              <mesh position={[0, 0.1, 0.01]}>
                <planeGeometry args={[0.4, 0.08]} />
                <meshBasicMaterial color="#0066ff" />
              </mesh>
              <Text
                position={[0, 0.1, 0.02]}
                fontSize={0.025}
                color="#ffffff"
                anchorX="center"
                anchorY="middle"
                font="/fonts/inter-medium.woff"
              >
                GRID FORMATION
              </Text>
            </Interactive>

            <Interactive
              onSelect={() => handleFormationCommand('circle')}
            >
              <mesh position={[0, 0, 0.01]}>
                <planeGeometry args={[0.4, 0.08]} />
                <meshBasicMaterial color="#0066ff" />
              </mesh>
              <Text
                position={[0, 0, 0.02]}
                fontSize={0.025}
                color="#ffffff"
                anchorX="center"
                anchorY="middle"
                font="/fonts/inter-medium.woff"
              >
                CIRCLE FORMATION
              </Text>
            </Interactive>

            {/* Emergency commands */}
            <Interactive
              onSelect={handleEmergencyStop}
            >
              <mesh position={[0, -0.15, 0.01]}>
                <planeGeometry args={[0.4, 0.08]} />
                <meshBasicMaterial color="#ff0000" />
              </mesh>
              <Text
                position={[0, -0.15, 0.02]}
                fontSize={0.025}
                color="#ffffff"
                anchorX="center"
                anchorY="middle"
                font="/fonts/inter-medium.woff"
              >
                EMERGENCY STOP
              </Text>
            </Interactive>

            <Interactive
              onSelect={handleReturnToBase}
            >
              <mesh position={[0, -0.25, 0.01]}>
                <planeGeometry args={[0.4, 0.08]} />
                <meshBasicMaterial color="#ff6600" />
              </mesh>
              <Text
                position={[0, -0.25, 0.02]}
                fontSize={0.025}
                color="#ffffff"
                anchorX="center"
                anchorY="middle"
                font="/fonts/inter-medium.woff"
              >
                RETURN TO BASE
              </Text>
            </Interactive>
          </group>
        </group>
      )}

      {/* Hand tracking gestures */}
      <XRHandGestureHandler />

      {/* Spatial waypoint placement */}
      <WaypointPlacer hitTestResults={hitTestResults} />

      {/* 3D minimap in XR space */}
      <XRMinimap />
    </group>
  )
}

function XRHandGestureHandler() {
  // Placeholder for hand gesture recognition
  // In a real implementation, this would use WebXR hand tracking APIs
  return null
}

function WaypointPlacer({ hitTestResults }: { hitTestResults: any[] }) {
  const { sendAgentCommand } = useSwarmConnection()
  const selectedAgents = useSelectedAgents()

  const handleWaypointPlace = (position: THREE.Vector3) => {
    selectedAgents.forEach(agent => {
      sendAgentCommand(agent.id, {
        type: 'navigate',
        position: [position.x, position.y, position.z]
      })
    })
  }

  return (
    <group>
      {hitTestResults.map((result, index) => (
        <Interactive
          key={index}
          onSelect={() => handleWaypointPlace(result.position)}
        >
          <mesh position={result.position}>
            <sphereGeometry args={[0.1]} />
            <meshBasicMaterial 
              color="#ffff00" 
              transparent 
              opacity={0.7}
            />
          </mesh>
        </Interactive>
      ))}
    </group>
  )
}

function XRMinimap() {
  const agents = useSwarmStore(state => state.agents)
  
  return (
    <group position={[2, 1, -1]}>
      {/* Minimap background */}
      <mesh>
        <planeGeometry args={[1, 1]} />
        <meshBasicMaterial 
          color="#000000" 
          transparent 
          opacity={0.8}
        />
      </mesh>

      {/* Agent positions on minimap */}
      {Object.values(agents).map(agent => (
        <mesh
          key={agent.id}
          position={[
            agent.position[0] * 0.01, // Scale down for minimap
            agent.position[2] * 0.01,
            0.01
          ]}
        >
          <sphereGeometry args={[0.02]} />
          <meshBasicMaterial 
            color={agent.status === 'active' ? '#00ff00' : '#666666'}
          />
        </mesh>
      ))}

      {/* Minimap title */}
      <Text
        position={[0, 0.6, 0.01]}
        fontSize={0.05}
        color="#ffffff"
        anchorX="center"
        anchorY="middle"
        font="/fonts/inter-medium.woff"
      >
        MINIMAP
      </Text>
    </group>
  )
}