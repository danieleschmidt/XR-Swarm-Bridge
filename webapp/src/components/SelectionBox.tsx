import React, { useRef } from 'react'
import { useFrame } from '@react-three/fiber'
import { useSwarmStore } from '../store/swarmStore'
import * as THREE from 'three'

export default function SelectionBox() {
  const selectionBoxRef = useRef<THREE.Mesh>(null)
  const { selectionBox, isSelecting } = useSwarmStore(state => ({
    selectionBox: state.selectionBox,
    isSelecting: state.isSelecting
  }))

  useFrame(() => {
    if (selectionBoxRef.current && isSelecting && selectionBox) {
      const box = selectionBoxRef.current
      const { start, end } = selectionBox
      
      // Calculate box dimensions and position
      const width = Math.abs(end.x - start.x)
      const height = Math.abs(end.y - start.y)
      const depth = Math.abs(end.z - start.z)
      
      const centerX = (start.x + end.x) / 2
      const centerY = (start.y + end.y) / 2
      const centerZ = (start.z + end.z) / 2
      
      box.position.set(centerX, centerY, centerZ)
      box.scale.set(width, height, depth)
      box.visible = true
    } else if (selectionBoxRef.current) {
      selectionBoxRef.current.visible = false
    }
  })

  return (
    <mesh ref={selectionBoxRef} visible={false}>
      <boxGeometry args={[1, 1, 1]} />
      <meshBasicMaterial 
        color="#00ff00" 
        transparent 
        opacity={0.2} 
        wireframe 
      />
    </mesh>
  )
}