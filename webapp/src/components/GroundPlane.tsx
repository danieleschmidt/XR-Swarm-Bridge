import React from 'react'
import * as THREE from 'three'

export default function GroundPlane() {
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.1, 0]} receiveShadow>
      <planeGeometry args={[200, 200]} />
      <meshStandardMaterial 
        color="#2a2a2a"
        transparent
        opacity={0.8}
        side={THREE.DoubleSide}
      />
    </mesh>
  )
}