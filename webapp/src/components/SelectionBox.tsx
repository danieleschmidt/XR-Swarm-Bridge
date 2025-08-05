import React, { useRef, useState } from 'react'
import { useFrame, useThree } from '@react-three/fiber'
import { useSwarmStore } from '../store/swarmStore'
import * as THREE from 'three'

export default function SelectionBox() {
  const { camera, raycaster, mouse } = useThree()
  const { agents, selectMultiple, clearSelection } = useSwarmStore()
  const [isSelecting, setIsSelecting] = useState(false)
  const [startPoint, setStartPoint] = useState<THREE.Vector2 | null>(null)
  const [endPoint, setEndPoint] = useState<THREE.Vector2 | null>(null)
  const selectionBoxRef = useRef<THREE.Mesh>(null)

  // Handle mouse events for box selection
  const handleMouseDown = (event: MouseEvent) => {
    if (event.button === 0 && event.shiftKey) { // Left click + Shift
      setIsSelecting(true)
      setStartPoint(new THREE.Vector2(mouse.x, mouse.y))
      setEndPoint(new THREE.Vector2(mouse.x, mouse.y))
    }
  }

  const handleMouseMove = (event: MouseEvent) => {
    if (isSelecting && startPoint) {
      setEndPoint(new THREE.Vector2(mouse.x, mouse.y))
    }
  }

  const handleMouseUp = (event: MouseEvent) => {
    if (isSelecting && startPoint && endPoint) {
      performBoxSelection()
      setIsSelecting(false)
      setStartPoint(null)
      setEndPoint(null)
    }
  }

  const performBoxSelection = () => {
    if (!startPoint || !endPoint) return

    const selectedAgents: string[] = []
    const frustum = new THREE.Frustum()
    const matrix = new THREE.Matrix4()
    
    // Create selection frustum
    const minX = Math.min(startPoint.x, endPoint.x)
    const maxX = Math.max(startPoint.x, endPoint.x)
    const minY = Math.min(startPoint.y, endPoint.y)
    const maxY = Math.max(startPoint.y, endPoint.y)

    // Convert screen coordinates to world selection
    Object.values(agents).forEach(agent => {
      const agentPosition = new THREE.Vector3(...agent.position)
      const screenPosition = agentPosition.clone().project(camera)
      
      if (screenPosition.x >= minX && screenPosition.x <= maxX && 
          screenPosition.y >= minY && screenPosition.y <= maxY) {
        selectedAgents.push(agent.id)
      }
    })

    if (selectedAgents.length > 0) {
      selectMultiple(selectedAgents)
    } else {
      clearSelection()
    }
  }

  // Set up event listeners
  React.useEffect(() => {
    const canvas = document.querySelector('canvas')
    if (!canvas) return

    canvas.addEventListener('mousedown', handleMouseDown)
    canvas.addEventListener('mousemove', handleMouseMove)
    canvas.addEventListener('mouseup', handleMouseUp)

    return () => {
      canvas.removeEventListener('mousedown', handleMouseDown)
      canvas.removeEventListener('mousemove', handleMouseMove)
      canvas.removeEventListener('mouseup', handleMouseUp)
    }
  }, [isSelecting, startPoint])

  // Update selection box visualization
  useFrame(() => {
    if (selectionBoxRef.current && isSelecting && startPoint && endPoint) {
      const width = Math.abs(endPoint.x - startPoint.x)
      const height = Math.abs(endPoint.y - startPoint.y)
      const centerX = (startPoint.x + endPoint.x) / 2
      const centerY = (startPoint.y + endPoint.y) / 2

      // Update selection box geometry
      selectionBoxRef.current.scale.set(width * 10, height * 10, 1)
      selectionBoxRef.current.position.set(centerX * 10, centerY * 10, -5)
      selectionBoxRef.current.visible = true
    } else if (selectionBoxRef.current) {
      selectionBoxRef.current.visible = false
    }
  })

  return (
    <>
      {/* Visual selection box */}
      <mesh ref={selectionBoxRef} visible={false}>
        <planeGeometry args={[1, 1]} />
        <meshBasicMaterial 
          color="#ffff00" 
          transparent 
          opacity={0.2}
          side={THREE.DoubleSide}
        />
      </mesh>

      {/* Selection box border */}
      {isSelecting && startPoint && endPoint && (
        <line>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              count={5}
              array={new Float32Array([
                startPoint.x * 10, startPoint.y * 10, -4,
                endPoint.x * 10, startPoint.y * 10, -4,
                endPoint.x * 10, endPoint.y * 10, -4,
                startPoint.x * 10, endPoint.y * 10, -4,
                startPoint.x * 10, startPoint.y * 10, -4,
              ])}
              itemSize={3}
            />
          </bufferGeometry>
          <lineBasicMaterial color="#ffff00" linewidth={2} />
        </line>
      )}
    </>
  )
}