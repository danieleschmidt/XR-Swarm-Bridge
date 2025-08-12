import React from 'react'
import * as THREE from 'three'

export default function GroundPlane() {
  return (
    <group>
      {/* Main ground plane */}
      <mesh position={[0, -0.1, 0]} rotation={[-Math.PI / 2, 0, 0]} receiveShadow>
        <planeGeometry args={[200, 200]} />
        <meshStandardMaterial 
          color="#2a2a2a" 
          roughness={0.8}
          metalness={0.1}
          transparent
          opacity={0.8}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* Grid pattern */}
      <gridHelper 
        args={[200, 100, '#444444', '#333333']} 
        position={[0, 0, 0]}
      />

      {/* Coordinate system origin marker */}
      <mesh position={[0, 0.01, 0]}>
        <ringGeometry args={[0.8, 1.0, 32]} />
        <meshBasicMaterial 
          color="#ffff00" 
          transparent 
          opacity={0.6}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* Distance reference circles */}
      {[10, 25, 50, 100].map(radius => (
        <mesh key={radius} position={[0, 0.005, 0]}>
          <ringGeometry args={[radius - 0.1, radius + 0.1, 64]} />
          <meshBasicMaterial 
            color="#666666" 
            transparent 
            opacity={0.3}
            side={THREE.DoubleSide}
          />
        </mesh>
      ))}

      {/* Cardinal direction markers */}
      <group>
        {/* North */}
        <mesh position={[0, 0.02, 25]}>
          <coneGeometry args={[0.5, 2]} />
          <meshBasicMaterial color="#ff0000" />
        </mesh>
        
        {/* East */}
        <mesh position={[25, 0.02, 0]} rotation={[0, 0, -Math.PI / 2]}>
          <coneGeometry args={[0.5, 2]} />
          <meshBasicMaterial color="#00ff00" />
        </mesh>
        
        {/* South */}
        <mesh position={[0, 0.02, -25]} rotation={[0, 0, Math.PI]}>
          <coneGeometry args={[0.5, 2]} />
          <meshBasicMaterial color="#0000ff" />
        </mesh>
        
        {/* West */}
        <mesh position={[-25, 0.02, 0]} rotation={[0, 0, Math.PI / 2]}>
          <coneGeometry args={[0.5, 2]} />
          <meshBasicMaterial color="#ffff00" />
        </mesh>
      </group>

      {/* Danger zones (example static obstacles) */}
      <mesh position={[30, 2, 30]}>
        <boxGeometry args={[8, 4, 8]} />
        <meshStandardMaterial 
          color="#ff4444" 
          transparent 
          opacity={0.3}
        />
      </mesh>
      
      <mesh position={[-20, 3, -40]}>
        <cylinderGeometry args={[6, 6, 6]} />
        <meshStandardMaterial 
          color="#ff4444" 
          transparent 
          opacity={0.3}
        />
      </mesh>

      {/* Landing pads */}
      {[
        [50, 50],
        [-50, 50],
        [50, -50],
        [-50, -50]
      ].map(([x, z], index) => (
        <group key={index} position={[x, 0.01, z]}>
          {/* Landing pad circle */}
          <mesh>
            <cylinderGeometry args={[3, 3, 0.02]} />
            <meshBasicMaterial 
              color="#00ff00" 
              transparent 
              opacity={0.7}
            />
          </mesh>
          
          {/* Landing pad cross */}
          <mesh>
            <boxGeometry args={[6, 0.03, 0.5]} />
            <meshBasicMaterial color="#ffffff" />
          </mesh>
          <mesh rotation={[0, Math.PI / 2, 0]}>
            <boxGeometry args={[6, 0.03, 0.5]} />
            <meshBasicMaterial color="#ffffff" />
          </mesh>
          
          {/* Landing pad number */}
          <mesh position={[0, 0.04, 0]} rotation={[-Math.PI / 2, 0, 0]}>
            <planeGeometry args={[2, 2]} />
            <meshBasicMaterial 
              color="#000000"
              transparent
              opacity={0.8}
            />
          </mesh>
        </group>
      ))}
    </group>
  )
}
