import { Suspense, useEffect } from 'react'
import { Routes, Route } from 'react-router-dom'
import { Canvas } from '@react-three/fiber'
import { VRButton, ARButton, XR } from '@react-three/xr'
import { OrbitControls, Stats, Environment } from '@react-three/drei'
import { Leva } from 'leva'

// Internationalization
import './i18n'
import { useTranslation } from 'react-i18next'
import { isRTL } from './i18n'

// Store
import { useSwarmStore } from './store/swarmStore'

// Components
import SwarmScene from './components/SwarmScene'
import UI from './components/UI'
import LoadingScreen from './components/LoadingScreen'
import Dashboard from './components/Dashboard'
import LanguageSelector from './components/LanguageSelector'
import { AccessibilityProvider } from './components/AccessibilityProvider'
import ConsentBanner from './components/ConsentBanner'

// XR Components
import XRInterface from './xr/XRInterface'

// Hooks
import { useWebRTC } from './hooks/useWebRTC'
import { useSwarmConnection } from './hooks/useSwarmConnection'

function App() {
  const { isConnected, connectionStatus } = useSwarmStore()
  const { i18n } = useTranslation()
  
  // Initialize connections
  useWebRTC()
  useSwarmConnection()

  // Set document direction based on language
  useEffect(() => {
    document.documentElement.dir = isRTL(i18n.language) ? 'rtl' : 'ltr'
    document.documentElement.lang = i18n.language
  }, [i18n.language])

  return (
    <AccessibilityProvider>
      <div className="w-full h-screen bg-black overflow-hidden" dir={isRTL(i18n.language) ? 'rtl' : 'ltr'}>
        {/* Development Tools */}
        <Leva collapsed />
        
        {/* Language Selector */}
        <div className="absolute top-4 left-4 z-50">
          <LanguageSelector variant="dropdown" className="bg-white/90 backdrop-blur" />
        </div>
        
        {/* VR/AR Entry Buttons */}
        <div className="absolute top-4 right-4 z-50 space-x-2">
          <VRButton />
          <ARButton />
        </div>
        
        {/* Connection Status */}
        <div className="absolute top-16 left-4 z-50">
          <div className={`px-3 py-1 rounded text-sm font-medium ${
            isConnected 
              ? 'bg-green-600 text-white' 
              : 'bg-red-600 text-white'
          }`}>
            {connectionStatus}
          </div>
        </div>

      <Routes>
        <Route path="/" element={
          <>
            {/* 3D Canvas */}
            <Canvas
              camera={{ position: [0, 10, 20], fov: 60 }}
              gl={{ 
                antialias: true, 
                alpha: false,
                powerPreference: 'high-performance'
              }}
              shadows
            >
              <XR>
                <Suspense fallback={null}>
                  {/* Environment */}
                  <Environment preset="sunset" />
                  
                  {/* Lighting */}
                  <ambientLight intensity={0.4} />
                  <directionalLight
                    position={[10, 10, 5]}
                    intensity={1}
                    castShadow
                    shadow-mapSize={[2048, 2048]}
                  />
                  
                  {/* Controls (disabled in XR) */}
                  <OrbitControls
                    enablePan={true}
                    enableZoom={true}
                    enableRotate={true}
                    target={[0, 0, 0]}
                  />
                  
                  {/* Main Swarm Scene */}
                  <SwarmScene />
                  
                  {/* XR Interface Components */}
                  <XRInterface />
                  
                  {/* Development Stats */}
                  <Stats showPanel={0} className="stats" />
                </Suspense>
              </XR>
            </Canvas>

            {/* 2D UI Overlay */}
            <UI />
          </>
        } />
        
        <Route path="/dashboard" element={<Dashboard />} />
      </Routes>

        {/* Loading Screen */}
        <Suspense fallback={<LoadingScreen />}>
          {/* App content loads here */}
        </Suspense>
        
        {/* Global Consent Banner */}
        <ConsentBanner position="bottom" />
      </div>
    </AccessibilityProvider>
  )
}

export default App