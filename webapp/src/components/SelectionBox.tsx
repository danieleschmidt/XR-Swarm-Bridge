import React, { useRef, useState } from 'react'
import { useThree, useFrame } from '@react-three/fiber'
import * as THREE from 'three'

interface SelectionBoxProps {
  onSelection?: (bounds: THREE.Box3) => void
}

export default function SelectionBox({ onSelection }: SelectionBoxProps) {
  const meshRef = useRef<THREE.Mesh>(null)
  const [isSelecting, setIsSelecting] = useState(false)
  const [startPoint, setStartPoint] = useState<THREE.Vector2 | null>(null)
  const [endPoint, setEndPoint] = useState<THREE.Vector2 | null>(null)
  
  const { camera, gl } = useThree()

  const handlePointerDown = (event: any) => {
    if (event.ctrlKey || event.metaKey) {
      setIsSelecting(true)
      const rect = gl.domElement.getBoundingClientRect()
      const x = ((event.clientX - rect.left) / rect.width) * 2 - 1
      const y = -((event.clientY - rect.top) / rect.height) * 2 + 1
      setStartPoint(new THREE.Vector2(x, y))
      setEndPoint(new THREE.Vector2(x, y))
    }
  }

  const handlePointerMove = (event: any) => {
    if (isSelecting && startPoint) {
      const rect = gl.domElement.getBoundingClientRect()
      const x = ((event.clientX - rect.left) / rect.width) * 2 - 1
      const y = -((event.clientY - rect.top) / rect.height) * 2 + 1
      setEndPoint(new THREE.Vector2(x, y))
    }
  }

  const handlePointerUp = () => {
    if (isSelecting && startPoint && endPoint && onSelection) {
      // Convert screen coordinates to world space bounds
      const raycaster = new THREE.Raycaster()
      
      raycaster.setFromCamera(startPoint, camera)
      const startWorld = raycaster.ray.origin.clone()
      
      raycaster.setFromCamera(endPoint, camera)
      const endWorld = raycaster.ray.origin.clone()
      
      const bounds = new THREE.Box3()
      bounds.setFromPoints([startWorld, endWorld])
      
      onSelection(bounds)
    }
    
    setIsSelecting(false)
    setStartPoint(null)
    setEndPoint(null)
  }

  useFrame(() => {
    if (meshRef.current && isSelecting && startPoint && endPoint) {
      const width = Math.abs(endPoint.x - startPoint.x)
      const height = Math.abs(endPoint.y - startPoint.y)
      const centerX = (startPoint.x + endPoint.x) / 2
      const centerY = (startPoint.y + endPoint.y) / 2

      meshRef.current.scale.set(width, height, 1)
      meshRef.current.position.set(centerX, centerY, 0)
    }
  })

  if (!isSelecting) {
    return (
      <group onPointerDown={handlePointerDown}>
        <mesh visible={false}>
          <planeGeometry args={[1000, 1000]} />
          <meshBasicMaterial transparent opacity={0} />
        </mesh>
      </group>
    )
  }

  return (
    <group 
      onPointerMove={handlePointerMove} 
      onPointerUp={handlePointerUp}
    >
      <mesh ref={meshRef}>
        <planeGeometry args={[1, 1]} />
        <meshBasicMaterial 
          color="#3b82f6" 
          transparent 
          opacity={0.2} 
          side={THREE.DoubleSide}
        />
      </mesh>
      <lineSegments>
        <edgesGeometry args={[new THREE.PlaneGeometry(1, 1)]} />
        <lineBasicMaterial color="#3b82f6" opacity={0.8} transparent />
      </lineSegments>
    </group>
  )
}