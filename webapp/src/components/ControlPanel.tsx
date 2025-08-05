import React, { useState } from 'react'
import { useSwarmStore, useSelectedAgents } from '../store/swarmStore'
import { useSwarmConnection } from '../hooks/useSwarmConnection'
import { useGPTIntegration } from '../ai/gptIntegration'

export default function ControlPanel() {
  const selectedAgents = useSelectedAgents()
  const { sendGlobalCommand, sendAgentCommand } = useSwarmConnection()
  const { generateCommands, isLoading } = useGPTIntegration()
  const { currentMission, setViewMode, viewMode, toggleTelemetry, toggleMinimap } = useSwarmStore()
  
  const [activeTab, setActiveTab] = useState<'formation' | 'movement' | 'mission' | 'ai'>('formation')
  const [aiCommand, setAiCommand] = useState('')

  const handleFormationCommand = (formation: string) => {
    const command = {
      type: 'formation',
      formation,
      agents: selectedAgents.length > 0 ? selectedAgents.map(a => a.id) : undefined
    }
    sendGlobalCommand(command)
  }

  const handleMovementCommand = (command: string, params?: any) => {
    if (selectedAgents.length > 0) {
      selectedAgents.forEach(agent => {
        sendAgentCommand(agent.id, { type: command, ...params })
      })
    } else {
      sendGlobalCommand({ type: command, ...params })
    }
  }

  const handleAICommand = async () => {
    if (!aiCommand.trim()) return

    const result = await generateCommands(aiCommand, {
      agents: selectedAgents
    })

    if (result.commands) {
      result.commands.forEach(command => {
        if (command.target === 'all') {
          sendGlobalCommand(command.parameters)
        } else if (Array.isArray(command.target)) {
          command.target.forEach(agentId => {
            sendAgentCommand(agentId, command.parameters)
          })
        }
      })
    }

    setAiCommand('')
  }

  return (
    <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg">
      <div className="flex items-center justify-between p-4 border-b border-gray-700">
        <div className="flex space-x-4">
          {/* Tab Navigation */}
          {[
            { id: 'formation', label: 'Formation', icon: '⚡' },
            { id: 'movement', label: 'Movement', icon: '🎯' },
            { id: 'mission', label: 'Mission', icon: '📋' },
            { id: 'ai', label: 'AI', icon: '🤖' }
          ].map(tab => (
            <button
              key={tab.id}
              onClick={() => setActiveTab(tab.id as any)}
              className={`px-3 py-2 rounded-md text-sm font-medium transition-colors ${
                activeTab === tab.id
                  ? 'bg-blue-600 text-white'
                  : 'text-gray-400 hover:text-white hover:bg-gray-700'
              }`}
            >
              <span className="mr-2">{tab.icon}</span>
              {tab.label}
            </button>
          ))}
        </div>

        <div className="flex items-center space-x-3">
          {/* View Mode Toggle */}
          <div className="flex bg-gray-800 rounded-md overflow-hidden">
            {['2d', '3d'].map(mode => (
              <button
                key={mode}
                onClick={() => setViewMode(mode as any)}
                className={`px-3 py-1 text-xs uppercase font-medium ${
                  viewMode === mode ? 'bg-blue-600 text-white' : 'text-gray-400 hover:text-white'
                }`}
              >
                {mode}
              </button>
            ))}
          </div>

          {/* UI Toggles */}
          <button
            onClick={toggleTelemetry}
            className="px-2 py-1 text-xs bg-gray-700 hover:bg-gray-600 rounded"
          >
            📊
          </button>
          <button
            onClick={toggleMinimap}
            className="px-2 py-1 text-xs bg-gray-700 hover:bg-gray-600 rounded"
          >
            🗺️
          </button>
        </div>
      </div>

      <div className="p-4">
        {/* Target Selection Info */}
        <div className="mb-4 text-sm text-gray-400">
          Target: {selectedAgents.length > 0 
            ? `${selectedAgents.length} selected robot${selectedAgents.length !== 1 ? 's' : ''}`
            : 'All robots'
          }
        </div>

        {/* Formation Tab */}
        {activeTab === 'formation' && (
          <div className="grid grid-cols-2 md:grid-cols-4 gap-3">
            <FormationButton
              label="Line"
              onClick={() => handleFormationCommand('line')}
              icon="━"
            />
            <FormationButton
              label="Grid"
              onClick={() => handleFormationCommand('grid')}
              icon="⊞"
            />
            <FormationButton
              label="Circle"
              onClick={() => handleFormationCommand('circle')}
              icon="○"
            />
            <FormationButton
              label="V-Formation"
              onClick={() => handleFormationCommand('v')}
              icon="∨"
            />
            <FormationButton
              label="Diamond"
              onClick={() => handleFormationCommand('diamond')}
              icon="◊"
            />
            <FormationButton
              label="Wedge"
              onClick={() => handleFormationCommand('wedge')}
              icon="△"
            />
            <FormationButton
              label="Column"
              onClick={() => handleFormationCommand('column')}
              icon="⫍"
            />
            <FormationButton
              label="Scatter"
              onClick={() => handleFormationCommand('scatter')}
              icon="✧"
            />
          </div>
        )}

        {/* Movement Tab */}
        {activeTab === 'movement' && (
          <div className="space-y-4">
            <div className="grid grid-cols-2 gap-3">
              <ActionButton
                label="Return to Base"
                onClick={() => handleMovementCommand('return_to_base')}
                color="orange"
                icon="🏠"
              />
              <ActionButton
                label="Emergency Stop"
                onClick={() => handleMovementCommand('emergency_stop')}
                color="red"
                icon="🛑"
              />
              <ActionButton
                label="Hold Position"
                onClick={() => handleMovementCommand('hold')}
                color="blue"
                icon="⏸️"
              />
              <ActionButton
                label="Resume"
                onClick={() => handleMovementCommand('resume')}
                color="green"
                icon="▶️"
              />
            </div>

            <div className="border-t border-gray-700 pt-4">
              <div className="text-sm text-gray-400 mb-2">Search Patterns</div>
              <div className="grid grid-cols-2 gap-3">
                <ActionButton
                  label="Grid Search"
                  onClick={() => handleMovementCommand('search', { pattern: 'grid' })}
                  color="blue"
                  icon="⊞"
                />
                <ActionButton
                  label="Spiral Search"
                  onClick={() => handleMovementCommand('search', { pattern: 'spiral' })}
                  color="blue"
                  icon="🌀"
                />
              </div>
            </div>
          </div>
        )}

        {/* Mission Tab */}
        {activeTab === 'mission' && (
          <div className="space-y-4">
            {currentMission ? (
              <div>
                <div className="bg-gray-800 rounded-lg p-4 mb-4">
                  <h4 className="font-medium text-white mb-2">{currentMission.name}</h4>
                  <p className="text-sm text-gray-400 mb-3">{currentMission.description}</p>
                  <div className="flex items-center justify-between">
                    <span className={`px-2 py-1 rounded text-xs ${
                      currentMission.status === 'active' ? 'bg-green-600' :
                      currentMission.status === 'completed' ? 'bg-blue-600' :
                      currentMission.status === 'failed' ? 'bg-red-600' :
                      'bg-yellow-600'
                    }`}>
                      {currentMission.status.toUpperCase()}
                    </span>
                    {currentMission.status === 'active' && (
                      <button
                        onClick={() => sendGlobalCommand({ type: 'mission_stop' })}
                        className="px-3 py-1 bg-red-600 hover:bg-red-700 rounded text-xs"
                      >
                        Stop Mission
                      </button>
                    )}
                  </div>
                </div>
              </div>
            ) : (
              <div className="text-center text-gray-400 py-8">
                <p>No active mission</p>
                <p className="text-sm mt-2">Use the AI tab to generate mission plans</p>
              </div>
            )}
          </div>
        )}

        {/* AI Tab */}
        {activeTab === 'ai' && (
          <div className="space-y-4">
            <div>
              <label className="block text-sm text-gray-400 mb-2">
                Natural Language Command
              </label>
              <div className="flex space-x-2">
                <input
                  type="text"
                  value={aiCommand}
                  onChange={(e) => setAiCommand(e.target.value)}
                  onKeyPress={(e) => e.key === 'Enter' && handleAICommand()}
                  placeholder="e.g., 'Form a circle around the building'"
                  className="flex-1 px-3 py-2 bg-gray-800 border border-gray-600 rounded-md text-white text-sm focus:border-blue-500 focus:outline-none"
                />
                <button
                  onClick={handleAICommand}
                  disabled={!aiCommand.trim() || isLoading}
                  className="px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 rounded-md text-sm font-medium"
                >
                  {isLoading ? '...' : 'Send'}
                </button>
              </div>
            </div>

            <div className="border-t border-gray-700 pt-4">
              <div className="text-sm text-gray-400 mb-2">Quick AI Commands</div>
              <div className="grid grid-cols-1 gap-2">
                {[
                  'Form a perimeter around the area',
                  'Search the building systematically',
                  'Create a reconnaissance formation',
                  'Establish overwatch positions',
                  'Return to base in formation'
                ].map((command, index) => (
                  <button
                    key={index}
                    onClick={() => setAiCommand(command)}
                    className="text-left px-3 py-2 bg-gray-800 hover:bg-gray-700 rounded text-sm text-gray-300"
                  >
                    "{command}"
                  </button>
                ))}
              </div>
            </div>
          </div>
        )}
      </div>
    </div>
  )
}

function FormationButton({ 
  label, 
  onClick, 
  icon 
}: { 
  label: string
  onClick: () => void
  icon: string
}) {
  return (
    <button
      onClick={onClick}
      className="flex flex-col items-center justify-center p-3 bg-gray-800 hover:bg-gray-700 border border-gray-600 hover:border-blue-500 rounded-lg transition-all"
    >
      <span className="text-2xl mb-1">{icon}</span>
      <span className="text-xs font-medium">{label}</span>
    </button>
  )
}

function ActionButton({ 
  label, 
  onClick, 
  color = 'blue',
  icon 
}: { 
  label: string
  onClick: () => void
  color?: 'blue' | 'green' | 'orange' | 'red'
  icon: string
}) {
  const colorClasses = {
    blue: 'bg-blue-600 hover:bg-blue-700',
    green: 'bg-green-600 hover:bg-green-700',
    orange: 'bg-orange-600 hover:bg-orange-700',
    red: 'bg-red-600 hover:bg-red-700'
  }

  return (
    <button
      onClick={onClick}
      className={`flex items-center justify-center space-x-2 p-3 ${colorClasses[color]} rounded-lg text-sm font-medium transition-colors`}
    >
      <span>{icon}</span>
      <span>{label}</span>
    </button>
  )
}