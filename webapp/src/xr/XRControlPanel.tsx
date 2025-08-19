import { useState } from 'react'
import { Interactive } from '@react-three/xr'
import { useSwarmStore } from '../store/swarmStore'
import { Text } from '@react-three/drei'

export default function XRControlPanel() {
  // const [activeTab, setActiveTab] = useState('command') // Not currently used
  const { selectedAgents, isConnected } = useSwarmStore()

  return (
    <group>
      {/* Panel background */}
      <mesh>
        <planeGeometry args={[2, 1.5]} />
        <meshBasicMaterial color="#1a1a1a" transparent opacity={0.9} />
      </mesh>
      
      {/* Connection status */}
      <mesh position={[0, 0.6, 0.01]}>
        <circleGeometry args={[0.05, 16]} />
        <meshBasicMaterial color={isConnected ? '#00ff00' : '#ff0000'} />
      </mesh>
      
      <Text
        position={[0.2, 0.6, 0.01]}
        fontSize={0.08}
        color="white"
        anchorX="left"
      >
        {isConnected ? 'Connected' : 'Disconnected'}
      </Text>
      
      {/* Selected agents count */}
      <Text
        position={[0, 0.4, 0.01]}
        fontSize={0.1}
        color="white"
        anchorX="center"
      >
        Selected: {selectedAgents.length}
      </Text>
      
      {/* Command buttons */}
      <CommandButton
        position={[-0.6, 0.1, 0.02]}
        label="Start"
        color="#00ff00"
        onClick={() => useSwarmStore.getState().sendCommand('start')}
      />
      
      <CommandButton
        position={[0, 0.1, 0.02]}
        label="Stop"
        color="#ff6600"
        onClick={() => useSwarmStore.getState().sendCommand('stop')}
      />
      
      <CommandButton
        position={[0.6, 0.1, 0.02]}
        label="Emergency"
        color="#ff0000"
        onClick={() => useSwarmStore.getState().sendCommand('emergency_stop')}
      />
      
      {/* Formation buttons */}
      <CommandButton
        position={[-0.6, -0.2, 0.02]}
        label="Grid"
        color="#0066ff"
        onClick={() => useSwarmStore.getState().sendCommand('formation_grid')}
      />
      
      <CommandButton
        position={[0, -0.2, 0.02]}
        label="Circle"
        color="#0066ff"
        onClick={() => useSwarmStore.getState().sendCommand('formation_circle')}
      />
      
      <CommandButton
        position={[0.6, -0.2, 0.02]}
        label="Line"
        color="#0066ff"
        onClick={() => useSwarmStore.getState().sendCommand('formation_line')}
      />
      
      {/* AI Command button */}
      <CommandButton
        position={[0, -0.5, 0.02]}
        label="AI Command"
        color="#ff00ff"
        onClick={() => useSwarmStore.getState().openAICommandInterface()}
        scale={[1.2, 1, 1]}
      />
    </group>
  )
}

interface CommandButtonProps {
  position: [number, number, number]
  label: string
  color: string
  onClick: () => void
  scale?: [number, number, number]
}

function CommandButton({ position, label, color, onClick, scale = [1, 1, 1] }: CommandButtonProps) {
  const [hovered, setHovered] = useState(false)
  
  return (
    <Interactive 
      onSelect={onClick}
      onHover={() => setHovered(true)}
      onBlur={() => setHovered(false)}
    >
      <group position={position} scale={scale}>
        <mesh>
          <boxGeometry args={[0.35, 0.15, 0.02]} />
          <meshBasicMaterial 
            color={hovered ? '#ffffff' : color} 
            transparent 
            opacity={hovered ? 0.3 : 0.8}
          />
        </mesh>
        
        <mesh>
          <planeGeometry args={[0.34, 0.14]} />
          <meshBasicMaterial color={color} transparent opacity={0.1} />
        </mesh>
        
        <Text
          position={[0, 0, 0.02]}
          fontSize={0.06}
          color={hovered ? color : 'white'}
          anchorX="center"
          anchorY="middle"
        >
          {label}
        </Text>
      </group>
    </Interactive>
  )
}