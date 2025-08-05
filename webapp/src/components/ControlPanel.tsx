import React, { useState } from 'react'
import { useSwarmStore } from '../store/swarmStore'
import AICommandInterface from './AICommandInterface'

export default function ControlPanel() {
  const [showAI, setShowAI] = useState(false)
  const { selectedAgents, sendCommand } = useSwarmStore()

  const handleCommand = (commandType: string, params?: any) => {
    sendCommand(commandType, params)
  }

  return (
    <>
      <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg p-4">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-semibold text-white">Control Panel</h3>
          <div className="text-sm text-gray-400">
            {selectedAgents.length} selected
          </div>
        </div>
        
        <div className="grid grid-cols-2 md:grid-cols-4 lg:grid-cols-6 gap-2">
          {/* Movement Commands */}
          <button
            onClick={() => handleCommand('takeoff')}
            className="bg-green-600 hover:bg-green-700 px-3 py-2 rounded text-sm"
            disabled={selectedAgents.length === 0}
          >
            Takeoff
          </button>
          
          <button
            onClick={() => handleCommand('land')}
            className="bg-orange-600 hover:bg-orange-700 px-3 py-2 rounded text-sm"
            disabled={selectedAgents.length === 0}
          >
            Land
          </button>
          
          <button
            onClick={() => handleCommand('hover')}
            className="bg-blue-600 hover:bg-blue-700 px-3 py-2 rounded text-sm"
            disabled={selectedAgents.length === 0}
          >
            Hover
          </button>
          
          {/* Formation Commands */}
          <button
            onClick={() => handleCommand('formation_line')}
            className="bg-purple-600 hover:bg-purple-700 px-3 py-2 rounded text-sm"
            disabled={selectedAgents.length === 0}
          >
            Line
          </button>
          
          <button
            onClick={() => handleCommand('formation_grid')}
            className="bg-purple-600 hover:bg-purple-700 px-3 py-2 rounded text-sm"
            disabled={selectedAgents.length === 0}
          >
            Grid
          </button>
          
          <button
            onClick={() => handleCommand('formation_circle')}
            className="bg-purple-600 hover:bg-purple-700 px-3 py-2 rounded text-sm"
            disabled={selectedAgents.length === 0}
          >
            Circle
          </button>
          
          {/* Search Commands */}
          <button
            onClick={() => handleCommand('search_spiral')}
            className="bg-cyan-600 hover:bg-cyan-700 px-3 py-2 rounded text-sm"
            disabled={selectedAgents.length === 0}
          >
            Search
          </button>
          
          <button
            onClick={() => handleCommand('return_to_base')}
            className="bg-yellow-600 hover:bg-yellow-700 px-3 py-2 rounded text-sm"
            disabled={selectedAgents.length === 0}
          >
            RTB
          </button>
          
          {/* Emergency */}
          <button
            onClick={() => handleCommand('emergency_stop')}
            className="bg-red-600 hover:bg-red-700 px-3 py-2 rounded text-sm"
          >
            STOP
          </button>
          
          {/* AI Command */}
          <button
            onClick={() => setShowAI(true)}
            className="bg-gradient-to-r from-purple-600 to-pink-600 hover:from-purple-700 hover:to-pink-700 px-3 py-2 rounded text-sm font-semibold"
          >
            🤖 AI
          </button>
        </div>
      </div>
      
      <AICommandInterface isOpen={showAI} onClose={() => setShowAI(false)} />
    </>
  )
}