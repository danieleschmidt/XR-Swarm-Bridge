import React from 'react'

export default function LoadingScreen() {
  return (
    <div className="fixed inset-0 bg-black flex items-center justify-center z-50">
      <div className="text-center">
        {/* Loading Spinner */}
        <div className="relative w-16 h-16 mx-auto mb-6">
          <div className="absolute inset-0 border-4 border-gray-800 rounded-full"></div>
          <div className="absolute inset-0 border-4 border-transparent border-t-blue-500 rounded-full animate-spin"></div>
        </div>
        
        {/* Title */}
        <h1 className="text-2xl font-bold text-white mb-2">
          XR-Swarm-Bridge
        </h1>
        
        {/* Subtitle */}
        <p className="text-gray-400 text-sm mb-6">
          Initializing immersive robot control system...
        </p>
        
        {/* Progress Steps */}
        <div className="space-y-2 text-left max-w-sm">
          <LoadingStep text="Connecting to swarm coordinator" completed />
          <LoadingStep text="Establishing WebRTC channels" loading />
          <LoadingStep text="Loading 3D environment" />
          <LoadingStep text="Initializing VR interface" />
        </div>
      </div>
    </div>
  )
}

interface LoadingStepProps {
  text: string
  completed?: boolean
  loading?: boolean
}

function LoadingStep({ text, completed = false, loading = false }: LoadingStepProps) {
  return (
    <div className="flex items-center space-x-3">
      {completed && (
        <div className="w-4 h-4 bg-green-500 rounded-full flex items-center justify-center">
          <svg className="w-2 h-2 text-white" fill="currentColor" viewBox="0 0 20 20">
            <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
          </svg>
        </div>
      )}
      
      {loading && (
        <div className="w-4 h-4 border-2 border-gray-600 border-t-blue-500 rounded-full animate-spin"></div>
      )}
      
      {!completed && !loading && (
        <div className="w-4 h-4 border-2 border-gray-600 rounded-full"></div>
      )}
      
      <span className={`text-sm ${
        completed ? 'text-green-400' : 
        loading ? 'text-blue-400' : 
        'text-gray-500'
      }`}>
        {text}
      </span>
    </div>
  )
}