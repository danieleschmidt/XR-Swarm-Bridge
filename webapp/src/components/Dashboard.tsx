import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { useSwarmStore, useActiveAgents, useSelectedAgents } from '../store/swarmStore'
import { useSwarmConnection } from '../hooks/useSwarmConnection'
import { useGPTIntegration } from '../ai/gptIntegration'
import type { Agent, Mission } from '../store/swarmStore'

export default function Dashboard() {
  const { t } = useTranslation()
  const activeAgents = useActiveAgents()
  const selectedAgents = useSelectedAgents()
  const { 
    latency, 
    packetLoss, 
    bandwidth,
    currentMission,
    showTelemetry,
    toggleTelemetry
  } = useSwarmStore()
  
  const { sendGlobalCommand, sendAgentCommand, startMission, stopMission } = useSwarmConnection()
  const { generateMissionPlan, generateCommands, isLoading } = useGPTIntegration()
  
  const [commandInput, setCommandInput] = useState('')
  const [missionObjective, setMissionObjective] = useState('')
  const [selectedTab, setSelectedTab] = useState<'overview' | 'agents' | 'missions' | 'ai'>('overview')

  return (
    <div className="min-h-screen bg-gray-900 text-white p-6">
      <div className="max-w-7xl mx-auto">
        {/* Header */}
        <header className="mb-8">
          <h1 className="text-3xl font-bold mb-2">{t('app.title')} {t('navigation.dashboard')}</h1>
          <div className="flex space-x-6 text-sm">
            <div className="flex items-center space-x-2">
              <div className="w-3 h-3 bg-green-500 rounded-full"></div>
              <span>{t('agents.active')}: {activeAgents.length}</span>
            </div>
            <div className="flex items-center space-x-2">
              <div className="w-3 h-3 bg-yellow-500 rounded-full"></div>
              <span>{t('agents.selected')}: {selectedAgents.length}</span>
            </div>
            <div className="flex items-center space-x-2">
              <div className="w-3 h-3 bg-blue-500 rounded-full"></div>
              <span>{t('system.latency')}: {latency}{t('system.ms')}</span>
            </div>
            <div className="flex items-center space-x-2">
              <div className="w-3 h-3 bg-purple-500 rounded-full"></div>
              <span>{t('system.packetLoss')}: {(packetLoss * 100).toFixed(1)}%</span>
            </div>
          </div>
        </header>

        {/* Navigation Tabs */}
        <nav className="mb-8">
          <div className="flex space-x-1 bg-gray-800 rounded-lg p-1">
            {[
              { id: 'overview', label: t('navigation.overview') },
              { id: 'agents', label: t('navigation.agents') },
              { id: 'missions', label: t('navigation.missions') },
              { id: 'ai', label: t('navigation.ai_commander') }
            ].map(tab => (
              <button
                key={tab.id}
                onClick={() => setSelectedTab(tab.id as any)}
                className={`px-4 py-2 rounded-md transition-colors ${
                  selectedTab === tab.id
                    ? 'bg-blue-600 text-white'
                    : 'text-gray-300 hover:text-white hover:bg-gray-700'
                }`}
              >
                {tab.label}
              </button>
            ))}
          </div>
        </nav>

        {/* Tab Content */}
        {selectedTab === 'overview' && <OverviewTab />}
        {selectedTab === 'agents' && <AgentsTab />}
        {selectedTab === 'missions' && <MissionsTab />}
        {selectedTab === 'ai' && <AICommanderTab />}
      </div>
    </div>
  )
}

function OverviewTab() {
  const activeAgents = useActiveAgents()
  const { currentMission, latency, packetLoss, bandwidth } = useSwarmStore()

  const agentsByType = activeAgents.reduce((acc, agent) => {
    acc[agent.type] = (acc[agent.type] || 0) + 1
    return acc
  }, {} as Record<string, number>)

  const averageBattery = activeAgents.length > 0 
    ? activeAgents.reduce((sum, agent) => sum + agent.battery, 0) / activeAgents.length
    : 0

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
      {/* System Status */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">System Status</h3>
        <div className="space-y-3">
          <div className="flex justify-between">
            <span>Active Agents:</span>
            <span className="text-green-400">{activeAgents.length}</span>
          </div>
          <div className="flex justify-between">
            <span>Network Latency:</span>
            <span className={latency > 200 ? 'text-red-400' : 'text-green-400'}>
              {latency}ms
            </span>
          </div>
          <div className="flex justify-between">
            <span>Packet Loss:</span>
            <span className={packetLoss > 0.05 ? 'text-red-400' : 'text-green-400'}>
              {(packetLoss * 100).toFixed(1)}%
            </span>
          </div>
          <div className="flex justify-between">
            <span>Bandwidth:</span>
            <span className="text-blue-400">{(bandwidth / 1000).toFixed(1)} Mbps</span>
          </div>
        </div>
      </div>

      {/* Agent Distribution */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Agent Types</h3>
        <div className="space-y-3">
          {Object.entries(agentsByType).map(([type, count]) => (
            <div key={type} className="flex justify-between">
              <span className="capitalize">{type}s:</span>
              <span className="text-blue-400">{count}</span>
            </div>
          ))}
          {Object.keys(agentsByType).length === 0 && (
            <p className="text-gray-400">No active agents</p>
          )}
        </div>
      </div>

      {/* Mission Status */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Current Mission</h3>
        {currentMission ? (
          <div className="space-y-2">
            <p className="font-medium">{currentMission.name}</p>
            <p className="text-sm text-gray-400">{currentMission.description}</p>
            <div className="flex justify-between">
              <span>Status:</span>
              <span className={`capitalize ${
                currentMission.status === 'active' ? 'text-green-400' :
                currentMission.status === 'completed' ? 'text-blue-400' :
                currentMission.status === 'failed' ? 'text-red-400' :
                'text-yellow-400'
              }`}>
                {currentMission.status}
              </span>
            </div>
          </div>
        ) : (
          <p className="text-gray-400">No active mission</p>
        )}
      </div>

      {/* Battery Overview */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Battery Status</h3>
        <div className="space-y-3">
          <div className="flex justify-between">
            <span>Average Battery:</span>
            <span className={`${
              averageBattery > 60 ? 'text-green-400' :
              averageBattery > 30 ? 'text-yellow-400' :
              'text-red-400'
            }`}>
              {averageBattery.toFixed(1)}%
            </span>
          </div>
          <div className="w-full bg-gray-700 rounded-full h-2">
            <div 
              className={`h-2 rounded-full ${
                averageBattery > 60 ? 'bg-green-500' :
                averageBattery > 30 ? 'bg-yellow-500' :
                'bg-red-500'
              }`}
              style={{ width: `${averageBattery}%` }}
            />
          </div>
          <div className="text-xs text-gray-400">
            Low battery agents: {activeAgents.filter(a => a.battery < 30).length}
          </div>
        </div>
      </div>

      {/* Quick Actions */}
      <div className="bg-gray-800 rounded-lg p-6 md:col-span-2">
        <h3 className="text-lg font-semibold mb-4">Quick Actions</h3>
        <div className="grid grid-cols-2 gap-3">
          <QuickActionButton 
            label="Emergency Stop" 
            color="red"
            onClick={() => useSwarmConnection().sendGlobalCommand({ type: 'emergency_stop' })}
          />
          <QuickActionButton 
            label="Return to Base" 
            color="orange"
            onClick={() => useSwarmConnection().sendGlobalCommand({ type: 'return_to_base' })}
          />
          <QuickActionButton 
            label="Line Formation" 
            color="blue"
            onClick={() => useSwarmConnection().sendGlobalCommand({ type: 'formation', formation: 'line' })}
          />
          <QuickActionButton 
            label="Grid Formation" 
            color="blue"
            onClick={() => useSwarmConnection().sendGlobalCommand({ type: 'formation', formation: 'grid' })}
          />
        </div>
      </div>
    </div>
  )
}

function AgentsTab() {
  const activeAgents = useActiveAgents()
  const { selectAgent, deselectAgent, selectedAgents } = useSwarmStore()
  const { sendAgentCommand } = useSwarmConnection()

  return (
    <div className="space-y-6">
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Agent List</h3>
        <div className="overflow-x-auto">
          <table className="w-full text-sm">
            <thead>
              <tr className="border-b border-gray-700">
                <th className="text-left p-2">Select</th>
                <th className="text-left p-2">ID</th>
                <th className="text-left p-2">Type</th>
                <th className="text-left p-2">Status</th>
                <th className="text-left p-2">Battery</th>
                <th className="text-left p-2">Position</th>
                <th className="text-left p-2">Actions</th>
              </tr>
            </thead>
            <tbody>
              {activeAgents.map(agent => (
                <tr key={agent.id} className="border-b border-gray-700 hover:bg-gray-750">
                  <td className="p-2">
                    <input
                      type="checkbox"
                      checked={selectedAgents.includes(agent.id)}
                      onChange={(e) => {
                        if (e.target.checked) {
                          selectAgent(agent.id)
                        } else {
                          deselectAgent(agent.id)
                        }
                      }}
                      className="rounded"
                    />
                  </td>
                  <td className="p-2 font-mono">{agent.id}</td>
                  <td className="p-2 capitalize">{agent.type}</td>
                  <td className="p-2">
                    <span className={`px-2 py-1 rounded text-xs ${getStatusColor(agent.status)}`}>
                      {agent.status}
                    </span>
                  </td>
                  <td className="p-2">
                    <span className={getBatteryColor(agent.battery)}>
                      {agent.battery.toFixed(0)}%
                    </span>
                  </td>
                  <td className="p-2 font-mono text-xs">
                    [{agent.position.map(p => p.toFixed(1)).join(', ')}]
                  </td>
                  <td className="p-2">
                    <div className="flex space-x-2">
                      <button
                        onClick={() => sendAgentCommand(agent.id, { type: 'return_to_base' })}
                        className="px-2 py-1 bg-orange-600 hover:bg-orange-700 rounded text-xs"
                      >
                        RTB
                      </button>
                      <button
                        onClick={() => sendAgentCommand(agent.id, { type: 'emergency_stop' })}
                        className="px-2 py-1 bg-red-600 hover:bg-red-700 rounded text-xs"
                      >
                        Stop
                      </button>
                    </div>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  )
}

function MissionsTab() {
  const { currentMission, missionHistory, startMission, stopMission } = useSwarmStore()
  const [newMissionName, setNewMissionName] = useState('')
  const [newMissionDescription, setNewMissionDescription] = useState('')

  const handleStartMission = () => {
    if (newMissionName.trim()) {
      const mission: Mission = {
        id: `mission_${Date.now()}`,
        name: newMissionName,
        description: newMissionDescription,
        phases: [],
        status: 'pending'
      }
      startMission(mission)
      setNewMissionName('')
      setNewMissionDescription('')
    }
  }

  return (
    <div className="space-y-6">
      {/* Create New Mission */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Create Mission</h3>
        <div className="space-y-4">
          <input
            type="text"
            placeholder="Mission Name"
            value={newMissionName}
            onChange={(e) => setNewMissionName(e.target.value)}
            className="w-full p-3 bg-gray-700 rounded border border-gray-600 focus:border-blue-500"
          />
          <textarea
            placeholder="Mission Description"
            value={newMissionDescription}
            onChange={(e) => setNewMissionDescription(e.target.value)}
            className="w-full p-3 bg-gray-700 rounded border border-gray-600 focus:border-blue-500 h-24"
          />
          <button
            onClick={handleStartMission}
            disabled={!newMissionName.trim()}
            className="px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 rounded"
          >
            Start Mission
          </button>
        </div>
      </div>

      {/* Current Mission */}
      {currentMission && (
        <div className="bg-gray-800 rounded-lg p-6">
          <h3 className="text-lg font-semibold mb-4">Current Mission</h3>
          <div className="space-y-2">
            <h4 className="font-medium text-xl">{currentMission.name}</h4>
            <p className="text-gray-400">{currentMission.description}</p>
            <div className="flex justify-between items-center">
              <span className={`px-3 py-1 rounded capitalize ${
                currentMission.status === 'active' ? 'bg-green-600' :
                currentMission.status === 'completed' ? 'bg-blue-600' :
                currentMission.status === 'failed' ? 'bg-red-600' :
                'bg-yellow-600'
              }`}>
                {currentMission.status}
              </span>
              {currentMission.status === 'active' && (
                <button
                  onClick={() => stopMission()}
                  className="px-4 py-2 bg-red-600 hover:bg-red-700 rounded"
                >
                  Stop Mission
                </button>
              )}
            </div>
          </div>
        </div>
      )}

      {/* Mission History */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Mission History</h3>
        {missionHistory.length > 0 ? (
          <div className="space-y-3">
            {missionHistory.slice(-5).reverse().map(mission => (
              <div key={mission.id} className="p-3 bg-gray-700 rounded">
                <div className="flex justify-between items-start">
                  <div>
                    <h4 className="font-medium">{mission.name}</h4>
                    <p className="text-sm text-gray-400">{mission.description}</p>
                  </div>
                  <span className={`px-2 py-1 rounded text-xs capitalize ${
                    mission.status === 'completed' ? 'bg-green-600' :
                    mission.status === 'failed' ? 'bg-red-600' :
                    'bg-yellow-600'
                  }`}>
                    {mission.status}
                  </span>
                </div>
              </div>
            ))}
          </div>
        ) : (
          <p className="text-gray-400">No missions in history</p>
        )}
      </div>
    </div>
  )
}

function AICommanderTab() {
  const { generateCommands, generateMissionPlan, isLoading } = useGPTIntegration()
  const { sendGlobalCommand, startMission } = useSwarmConnection()
  const activeAgents = useActiveAgents()
  
  const [commandInput, setCommandInput] = useState('')
  const [missionObjective, setMissionObjective] = useState('')
  const [aiResponse, setAiResponse] = useState<string>('')

  const handleNaturalLanguageCommand = async () => {
    if (!commandInput.trim()) return

    const result = await generateCommands(commandInput, {
      agents: activeAgents
    })

    if (result.error) {
      setAiResponse(`Error: ${result.error}`)
    } else if (result.commands) {
      setAiResponse(`Generated ${result.commands.length} commands`)
      
      // Execute the commands
      result.commands.forEach(command => {
        sendGlobalCommand(command.parameters)
      })
    }

    setCommandInput('')
  }

  const handleGenerateMission = async () => {
    if (!missionObjective.trim()) return

    const result = await generateMissionPlan(missionObjective, {
      agents: activeAgents
    })

    if (result.error) {
      setAiResponse(`Error: ${result.error}`)
    } else if (result.plan) {
      setAiResponse(`Generated mission plan: ${result.plan.name}`)
      startMission(result.plan)
    }

    setMissionObjective('')
  }

  return (
    <div className="space-y-6">
      {/* Natural Language Commands */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">AI Command Interface</h3>
        <div className="space-y-4">
          <textarea
            placeholder="Enter natural language command (e.g., 'Form a circle around the building', 'Search the area in grid pattern')"
            value={commandInput}
            onChange={(e) => setCommandInput(e.target.value)}
            className="w-full p-3 bg-gray-700 rounded border border-gray-600 focus:border-blue-500 h-24"
          />
          <button
            onClick={handleNaturalLanguageCommand}
            disabled={!commandInput.trim() || isLoading}
            className="px-4 py-2 bg-green-600 hover:bg-green-700 disabled:bg-gray-600 rounded"
          >
            {isLoading ? 'Processing...' : 'Execute Command'}
          </button>
        </div>
      </div>

      {/* Mission Generation */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">AI Mission Planner</h3>
        <div className="space-y-4">
          <textarea
            placeholder="Describe mission objective (e.g., 'Search and rescue in collapsed building', 'Perimeter security patrol')"
            value={missionObjective}
            onChange={(e) => setMissionObjective(e.target.value)}
            className="w-full p-3 bg-gray-700 rounded border border-gray-600 focus:border-blue-500 h-24"
          />
          <button
            onClick={handleGenerateMission}
            disabled={!missionObjective.trim() || isLoading}
            className="px-4 py-2 bg-purple-600 hover:bg-purple-700 disabled:bg-gray-600 rounded"
          >
            {isLoading ? 'Generating...' : 'Generate Mission Plan'}
          </button>
        </div>
      </div>

      {/* AI Response */}
      {aiResponse && (
        <div className="bg-gray-800 rounded-lg p-6">
          <h3 className="text-lg font-semibold mb-4">AI Response</h3>
          <div className="p-4 bg-gray-700 rounded">
            <p className="whitespace-pre-wrap">{aiResponse}</p>
          </div>
        </div>
      )}
    </div>
  )
}

function QuickActionButton({ 
  label, 
  color, 
  onClick 
}: { 
  label: string
  color: 'red' | 'orange' | 'blue' | 'green'
  onClick: () => void 
}) {
  const colorClasses = {
    red: 'bg-red-600 hover:bg-red-700',
    orange: 'bg-orange-600 hover:bg-orange-700',
    blue: 'bg-blue-600 hover:bg-blue-700',
    green: 'bg-green-600 hover:bg-green-700'
  }

  return (
    <button
      onClick={onClick}
      className={`px-4 py-2 rounded text-sm font-medium transition-colors ${colorClasses[color]}`}
    >
      {label}
    </button>
  )
}

function getStatusColor(status: string): string {
  switch (status) {
    case 'active': return 'bg-green-600'
    case 'idle': return 'bg-blue-600'
    case 'error': return 'bg-red-600'
    case 'emergency_stop': return 'bg-orange-600'
    case 'timeout': return 'bg-gray-600'
    default: return 'bg-gray-600'
  }
}

function getBatteryColor(battery: number): string {
  if (battery > 60) return 'text-green-400'
  if (battery > 30) return 'text-yellow-400'
  return 'text-red-400'
}