import React, { useState } from 'react'
import { useSwarmStore, useActiveAgents, useSelectedAgents } from '../store/swarmStore'
import { useSwarmConnection } from '../hooks/useSwarmConnection'

export default function RobotPanel() {
  const activeAgents = useActiveAgents()
  const selectedAgents = useSelectedAgents()
  const { selectAgent, deselectAgent, clearSelection } = useSwarmStore()
  const { sendAgentCommand } = useSwarmConnection()
  const [searchTerm, setSearchTerm] = useState('')
  const [sortBy, setSortBy] = useState<'id' | 'type' | 'battery' | 'status'>('id')

  const filteredAgents = activeAgents
    .filter(agent => 
      agent.id.toLowerCase().includes(searchTerm.toLowerCase()) ||
      agent.type.toLowerCase().includes(searchTerm.toLowerCase())
    )
    .sort((a, b) => {
      switch (sortBy) {
        case 'id': return a.id.localeCompare(b.id)
        case 'type': return a.type.localeCompare(b.type)
        case 'battery': return b.battery - a.battery
        case 'status': return a.status.localeCompare(b.status)
        default: return 0
      }
    })

  const handleSelectAll = () => {
    if (selectedAgents.length === filteredAgents.length) {
      clearSelection()
    } else {
      filteredAgents.forEach(agent => selectAgent(agent.id))
    }
  }

  const handleAgentClick = (agentId: string) => {
    if (selectedAgents.map(a => a.id).includes(agentId)) {
      deselectAgent(agentId)
    } else {
      selectAgent(agentId)
    }
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case 'active': return 'text-green-400'
      case 'idle': return 'text-blue-400'
      case 'error': return 'text-red-400'
      case 'emergency_stop': return 'text-orange-400'
      case 'timeout': return 'text-gray-400'
      default: return 'text-gray-400'
    }
  }

  const getBatteryColor = (battery: number) => {
    if (battery > 60) return 'text-green-400'
    if (battery > 30) return 'text-yellow-400'
    return 'text-red-400'
  }

  return (
    <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg h-full flex flex-col">
      {/* Header */}
      <div className="p-4 border-b border-gray-700">
        <div className="flex items-center justify-between mb-3">
          <h3 className="text-lg font-semibold text-white">Robots</h3>
          <span className="text-sm text-gray-400">
            {selectedAgents.length}/{activeAgents.length}
          </span>
        </div>

        {/* Search */}
        <input
          type="text"
          placeholder="Search robots..."
          value={searchTerm}
          onChange={(e) => setSearchTerm(e.target.value)}
          className="w-full px-3 py-2 bg-gray-800 border border-gray-600 rounded-md text-white text-sm focus:border-blue-500 focus:outline-none"
        />

        {/* Controls */}
        <div className="flex items-center justify-between mt-3">
          <div className="flex space-x-2">
            <button
              onClick={handleSelectAll}
              className="px-3 py-1 bg-blue-600 hover:bg-blue-700 rounded text-xs font-medium"
            >
              {selectedAgents.length === filteredAgents.length ? 'Clear' : 'All'}
            </button>
            <select
              value={sortBy}
              onChange={(e) => setSortBy(e.target.value as any)}
              className="px-2 py-1 bg-gray-800 border border-gray-600 rounded text-xs"
            >
              <option value="id">Sort by ID</option>
              <option value="type">Sort by Type</option>
              <option value="battery">Sort by Battery</option>
              <option value="status">Sort by Status</option>
            </select>
          </div>
        </div>
      </div>

      {/* Agent List */}
      <div className="flex-1 overflow-y-auto p-2">
        {filteredAgents.length === 0 ? (
          <div className="text-center text-gray-400 mt-8">
            {searchTerm ? 'No robots match your search' : 'No active robots'}
          </div>
        ) : (
          <div className="space-y-2">
            {filteredAgents.map(agent => {
              const isSelected = selectedAgents.map(a => a.id).includes(agent.id)
              
              return (
                <div
                  key={agent.id}
                  onClick={() => handleAgentClick(agent.id)}
                  className={`p-3 rounded-lg border cursor-pointer transition-all hover:border-blue-500 ${
                    isSelected 
                      ? 'bg-blue-900/50 border-blue-500' 
                      : 'bg-gray-800/50 border-gray-600'
                  }`}
                >
                  <div className="flex items-center justify-between mb-2">
                    <div className="flex items-center space-x-2">
                      <div className={`w-3 h-3 rounded-full ${
                        agent.status === 'active' ? 'bg-green-500' :
                        agent.status === 'idle' ? 'bg-blue-500' :
                        agent.status === 'error' ? 'bg-red-500' :
                        'bg-gray-500'
                      }`} />
                      <span className="font-mono text-sm font-medium text-white">
                        {agent.id}
                      </span>
                    </div>
                    <div className="flex items-center space-x-2">
                      <span className="text-xs text-gray-400 capitalize">
                        {agent.type}
                      </span>
                      {isSelected && (
                        <div className="w-4 h-4 bg-blue-500 rounded-full flex items-center justify-center">
                          <svg className="w-2 h-2 text-white" fill="currentColor" viewBox="0 0 20 20">
                            <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                          </svg>
                        </div>
                      )}
                    </div>
                  </div>

                  <div className="flex items-center justify-between text-xs">
                    <span className={getStatusColor(agent.status)}>
                      {agent.status.replace('_', ' ').toUpperCase()}
                    </span>
                    <span className={getBatteryColor(agent.battery)}>
                      {Math.round(agent.battery)}%
                    </span>
                  </div>

                  <div className="mt-2 text-xs text-gray-400 font-mono">
                    [{agent.position.map(p => p.toFixed(1)).join(', ')}]
                  </div>

                  {/* Quick Actions */}
                  <div className="mt-2 flex space-x-1">
                    <button
                      onClick={(e) => {
                        e.stopPropagation()
                        sendAgentCommand(agent.id, { type: 'return_to_base' })
                      }}
                      className="px-2 py-1 bg-orange-600 hover:bg-orange-700 rounded text-xs"
                    >
                      RTB
                    </button>
                    <button
                      onClick={(e) => {
                        e.stopPropagation()
                        sendAgentCommand(agent.id, { type: 'emergency_stop' })
                      }}
                      className="px-2 py-1 bg-red-600 hover:bg-red-700 rounded text-xs"
                    >
                      Stop
                    </button>
                    {agent.status === 'idle' && (
                      <button
                        onClick={(e) => {
                          e.stopPropagation()
                          sendAgentCommand(agent.id, { type: 'set_mode', mode: 'active' })
                        }}
                        className="px-2 py-1 bg-green-600 hover:bg-green-700 rounded text-xs"
                      >
                        Start
                      </button>
                    )}
                  </div>
                </div>
              )
            })}
          </div>
        )}
      </div>

      {/* Selection Summary */}
      {selectedAgents.length > 0 && (
        <div className="p-4 border-t border-gray-700">
          <div className="text-sm text-white mb-2">
            {selectedAgents.length} robot{selectedAgents.length !== 1 ? 's' : ''} selected
          </div>
          <div className="flex space-x-2">
            <button
              onClick={() => {
                selectedAgents.forEach(agent => 
                  sendAgentCommand(agent.id, { type: 'return_to_base' })
                )
              }}
              className="px-3 py-1 bg-orange-600 hover:bg-orange-700 rounded text-xs"
            >
              Return All
            </button>
            <button
              onClick={() => {
                selectedAgents.forEach(agent => 
                  sendAgentCommand(agent.id, { type: 'emergency_stop' })
                )
              }}
              className="px-3 py-1 bg-red-600 hover:bg-red-700 rounded text-xs"
            >
              Stop All
            </button>
          </div>
        </div>
      )}
    </div>
  )
}