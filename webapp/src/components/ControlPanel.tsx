import React from 'react'
import { useSwarmStore } from '../store/swarmStore'
import CommandInterface from './CommandInterface'
import AICommandInterface from './AICommandInterface'

export default function ControlPanel() {
  const { isConnected, connectionStatus } = useSwarmStore()

  return (
    <div className="fixed bottom-4 left-4 right-4 z-40">
      <div className="max-w-4xl mx-auto">
        <div className="flex items-end justify-between space-x-4">
          {/* Status Indicator */}
          <div className="flex items-center space-x-2">
            <div className={`w-3 h-3 rounded-full ${
              isConnected ? 'bg-green-500' : 'bg-red-500'
            }`} />
            <span className="text-sm text-white/80">{connectionStatus}</span>
          </div>

          {/* Command Interface */}
          <div className="flex-1 max-w-2xl">
            <CommandInterface />
          </div>

          {/* AI Command Interface */}
          <div>
            <AICommandInterface />
          </div>
        </div>
      </div>
    </div>
  )
}