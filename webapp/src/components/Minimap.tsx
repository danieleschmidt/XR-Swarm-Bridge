import React from 'react'
import { useActiveAgents, useSwarmStore } from '../store/swarmStore'

export default function Minimap() {
  const activeAgents = useActiveAgents()
  const { selectedAgents } = useSwarmStore()
  
  const mapScale = 0.05 // Scale down coordinates for minimap
  const mapSize = 200 // Minimap size in pixels

  return (
    <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg p-3">
      <h4 className="text-sm font-semibold text-white mb-2">Minimap</h4>
      
      <div 
        className="relative bg-gray-900 border border-gray-600"
        style={{ width: mapSize, height: mapSize }}
      >
        {/* Grid lines */}
        <svg className="absolute inset-0 w-full h-full">
          <defs>
            <pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse">
              <path d="M 20 0 L 0 0 0 20" fill="none" stroke="#374151" strokeWidth="0.5"/>
            </pattern>
          </defs>
          <rect width="100%" height="100%" fill="url(#grid)" />
        </svg>
        
        {/* Center marker */}
        <div 
          className="absolute w-1 h-1 bg-white rounded-full"
          style={{ 
            left: mapSize / 2 - 2, 
            top: mapSize / 2 - 2 
          }}
        />
        
        {/* Agent markers */}
        {activeAgents.map((agent) => {
          const x = Math.max(0, Math.min(mapSize - 8, (agent.position[0] * mapScale) + mapSize / 2))
          const y = Math.max(0, Math.min(mapSize - 8, (-agent.position[1] * mapScale) + mapSize / 2))
          const isSelected = selectedAgents.includes(agent.id)
          
          return (
            <div
              key={agent.id}
              className={`absolute w-2 h-2 rounded-full ${
                isSelected 
                  ? 'bg-blue-400 ring-2 ring-blue-200' 
                  : getAgentColor(agent.status)
              }`}
              style={{ left: x, top: y }}
              title={`${agent.id} - ${agent.status}`}
            />
          )
        })}
        
        {/* Compass */}
        <div className="absolute top-1 right-1 text-xs text-gray-400">
          <div className="text-center">N</div>
        </div>
      </div>
      
      <div className="mt-2 text-xs text-gray-400">
        <div>Agents: {activeAgents.length}</div>
        <div>Selected: {selectedAgents.length}</div>
      </div>
    </div>
  )
}

function getAgentColor(status: string): string {
  switch (status) {
    case 'active': return 'bg-green-400'
    case 'idle': return 'bg-blue-400'
    case 'error': return 'bg-red-400'
    case 'emergency_stop': return 'bg-orange-400'
    default: return 'bg-gray-400'
  }
}