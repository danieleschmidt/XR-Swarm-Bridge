import React from 'react'
import { useSwarmStore, useActiveAgents } from '../store/swarmStore'

export default function RobotPanel() {
  const { selectedAgents, selectAgent, deselectAgent, clearSelection } = useSwarmStore()
  const activeAgents = useActiveAgents()

  return (
    <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg p-4 h-full overflow-y-auto">
      <div className="flex justify-between items-center mb-4">
        <h3 className="text-lg font-semibold text-white">Robots</h3>
        <button
          onClick={clearSelection}
          className="text-xs bg-gray-600 hover:bg-gray-500 px-2 py-1 rounded"
        >
          Clear
        </button>
      </div>
      
      <div className="space-y-2">
        {activeAgents.map((agent) => (
          <div
            key={agent.id}
            className={`p-3 rounded border cursor-pointer transition-colors ${
              selectedAgents.includes(agent.id)
                ? 'bg-blue-600/20 border-blue-500'
                : 'bg-gray-800 border-gray-600 hover:bg-gray-700'
            }`}
            onClick={() => {
              if (selectedAgents.includes(agent.id)) {
                deselectAgent(agent.id)
              } else {
                selectAgent(agent.id)
              }
            }}
          >
            <div className="flex justify-between items-center">
              <span className="font-mono text-sm">{agent.id}</span>
              <span className={`text-xs px-2 py-1 rounded ${getStatusColor(agent.status)}`}>
                {agent.status}
              </span>
            </div>
            <div className="text-xs text-gray-400 mt-1">
              Type: {agent.type} | Battery: {agent.battery}%
            </div>
            <div className="text-xs text-gray-400">
              Pos: [{agent.position.map(p => p.toFixed(1)).join(', ')}]
            </div>
          </div>
        ))}
      </div>
    </div>
  )
}

function getStatusColor(status: string): string {
  switch (status) {
    case 'active': return 'bg-green-600'
    case 'idle': return 'bg-blue-600'
    case 'error': return 'bg-red-600'
    case 'emergency_stop': return 'bg-orange-600'
    default: return 'bg-gray-600'
  }
}