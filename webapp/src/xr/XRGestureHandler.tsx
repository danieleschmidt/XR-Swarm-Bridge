import { useRef } from 'react'
import { useXR } from '@react-three/xr'
import { useFrame } from '@react-three/fiber'
import { useSwarmStore } from '../store/swarmStore'
import * as THREE from 'three'

export default function XRGestureHandler() {
  const { controllers } = useXR()
  const lastPositions = useRef<{ [key: string]: THREE.Vector3 }>({})
  const gestureStates = useRef<{ [key: string]: any }>({})

  useFrame(() => {
    controllers.forEach((controller, index) => {
      if (!controller.grip) return

      const controllerId = `controller_${index}`
      const currentPosition = controller.grip.position.clone()
      const lastPosition = lastPositions.current[controllerId]

      if (lastPosition) {
        const velocity = currentPosition.clone().sub(lastPosition)
        const speed = velocity.length()
        
        // Detect gestures based on controller movement
        detectGestures(controllerId, controller, velocity, speed)
      }

      lastPositions.current[controllerId] = currentPosition
    })
  })

  const detectGestures = (controllerId: string, controller: any, velocity: THREE.Vector3, speed: number) => {
    const state = gestureStates.current[controllerId] || { gesture: 'none', startTime: 0, data: {} }
    
    // Pinch gesture (using trigger button as approximation)
    if (controller.inputSource?.gamepad?.buttons[0]?.pressed) {
      if (state.gesture !== 'pinch') {
        state.gesture = 'pinch'
        state.startTime = Date.now()
        state.data.startPosition = controller.grip.position.clone()
        onGestureStart('pinch', controller.grip.position)
      }
    } else if (state.gesture === 'pinch') {
      onGestureEnd('pinch', controller.grip.position, state.data)
      state.gesture = 'none'
    }
    
    // Swipe gestures
    if (speed > 0.1 && state.gesture === 'none') {
      const direction = velocity.normalize()
      
      if (Math.abs(direction.x) > 0.7) {
        const swipeType = direction.x > 0 ? 'swipe_right' : 'swipe_left'
        onGestureDetected(swipeType, controller.grip.position)
      } else if (Math.abs(direction.y) > 0.7) {
        const swipeType = direction.y > 0 ? 'swipe_up' : 'swipe_down'
        onGestureDetected(swipeType, controller.grip.position)
      } else if (Math.abs(direction.z) > 0.7) {
        const swipeType = direction.z > 0 ? 'swipe_forward' : 'swipe_back'
        onGestureDetected(swipeType, controller.grip.position)
      }
    }
    
    // Point gesture (controller pointing)
    if (controller.inputSource?.gamepad?.buttons[1]?.pressed) {
      const direction = new THREE.Vector3(0, 0, -1).applyQuaternion(controller.grip.quaternion)
      onGestureDetected('point', controller.grip.position, { direction })
    }

    gestureStates.current[controllerId] = state
  }

  const onGestureStart = (gestureType: string, position: THREE.Vector3) => {
    console.log(`Gesture started: ${gestureType} at`, position)
    
    switch (gestureType) {
      case 'pinch':
        // Start selection mode
        useSwarmStore.getState().startSelection(position.toArray())
        break
    }
  }

  const onGestureEnd = (gestureType: string, position: THREE.Vector3, _data: any) => {
    console.log(`Gesture ended: ${gestureType} at`, position)
    
    switch (gestureType) {
      case 'pinch':
        // End selection mode
        useSwarmStore.getState().endSelection(position.toArray())
        break
    }
  }

  const onGestureDetected = (gestureType: string, position: THREE.Vector3, data?: any) => {
    console.log(`Gesture detected: ${gestureType} at`, position)
    
    switch (gestureType) {
      case 'swipe_right':
        useSwarmStore.getState().sendCommand('move_right')
        break
      case 'swipe_left':
        useSwarmStore.getState().sendCommand('move_left')
        break
      case 'swipe_up':
        useSwarmStore.getState().sendCommand('move_up')
        break
      case 'swipe_down':
        useSwarmStore.getState().sendCommand('move_down')
        break
      case 'swipe_forward':
        useSwarmStore.getState().sendCommand('move_forward')
        break
      case 'swipe_back':
        useSwarmStore.getState().sendCommand('move_back')
        break
      case 'point':
        if (data?.direction) {
          // Raycast to find what the user is pointing at
          // const raycaster = new THREE.Raycaster(position, data.direction) // Not currently used
          // In a real implementation, you'd raycast against scene objects
          useSwarmStore.getState().setTargetDirection(data.direction.toArray())
        }
        break
    }
  }

  return null // This component doesn't render anything visible
}