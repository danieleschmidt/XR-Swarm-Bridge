import { useRef, useState } from 'react'
import { useXR, useController, useHitTest, Interactive } from '@react-three/xr'
import { useFrame } from '@react-three/fiber'
import { useSwarmStore, useSelectedAgents } from '../store/swarmStore'
import { useSwarmConnection } from '../hooks/useSwarmConnection'
import { Text } from '@react-three/drei'
import XRControlPanel from './XRControlPanel'
import XRGestureHandler from './XRGestureHandler'
import XRMiniMap from './XRMiniMap'
import * as THREE from 'three'

export default function XRInterface() {
  const { isPresenting } = useXR()
  const leftController = useController('left')
  const rightController = useController('right')
  const { sendGlobalCommand } = useSwarmConnection()
  const selectedAgents = useSelectedAgents()
  const agents = useSwarmStore(state => state.agents)
  
  // XR-specific UI panels
  const leftPanelRef = useRef<THREE.Group>(null)
  // const rightPanelRef = useRef<THREE.Group>(null) // Not currently used
  const controllerRef = useRef<THREE.Group>(null)

  // Hit test for placing waypoints
  const [hitTestResults, setHitTestResults] = useState<THREE.Matrix4[]>([])
  
  useHitTest((hitMatrix, _hit) => {
    setHitTestResults(prev => [...prev.slice(-4), hitMatrix]) // Keep last 5 results
  })

  // Handle controller input for agent selection and commands
  useFrame(() => {
    if (!isPresenting || !controllerRef.current) return
    
    // Controller-based interaction logic would go here
    // For now, we'll use simplified hand tracking simulation
  })

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

      {/* Spatial waypoint placement */}
      <WaypointPlacer hitTestResults={hitTestResults} />

      {/* Interactive waypoint placement */}
      <InteractiveSpace />
      
      {/* Agent information displays in 3D space */}
      {selectedAgents.map(agent => {
        const agentData = agents[agent.id]
        if (!agentData) return null
        
        return (
          <group 
            key={agent.id} 
            position={[agentData.position[0], agentData.position[1] + 1, agentData.position[2]]}
          >
            <AgentInfoPanel agent={agentData} />
          </group>
        )
      })}
    </group>
  )
}

function WaypointPlacer({ hitTestResults }: { hitTestResults: THREE.Matrix4[] }) {
  const { sendAgentCommand } = useSwarmConnection()
  const selectedAgents = useSelectedAgents()

  const handleWaypointPlace = (hitMatrix: THREE.Matrix4) => {
    const position = new THREE.Vector3()
    const quaternion = new THREE.Quaternion()
    const scale = new THREE.Vector3()
    hitMatrix.decompose(position, quaternion, scale)
    
    selectedAgents.forEach(agent => {
      sendAgentCommand(agent.id, {
        type: 'navigate',
        position: [position.x, position.y, position.z]
      })
    })
  }

  return (
    <group>
      {hitTestResults.map((hitMatrix, index) => {
        const position = new THREE.Vector3()
        const quaternion = new THREE.Quaternion()
        const scale = new THREE.Vector3()
        hitMatrix.decompose(position, quaternion, scale)
        
        return (
          <Interactive
            key={index}
            onSelect={() => handleWaypointPlace(hitMatrix)}
          >
            <mesh position={position}>
              <sphereGeometry args={[0.1]} />
              <meshBasicMaterial 
                color="#ffff00" 
                transparent 
                opacity={0.7}
              />
            </mesh>
          </Interactive>
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
      
      {/* Agent ID text */}
      <Text
        position={[0, -0.2, 0.01]}
        fontSize={0.03}
        color="#ffffff"
        anchorX="center"
        anchorY="middle"
        font="/fonts/inter-medium.woff"
      >
        {agent.id}
      </Text>
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
