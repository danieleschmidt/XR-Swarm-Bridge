import React, { useState } from 'react'
import { useSwarmStore, useActiveAgents } from '../store/swarmStore'

export default function Dashboard() {
  const [activeTab, setActiveTab] = useState('overview')
  const { isConnected, connectionStatus, webrtcStats, currentMission } = useSwarmStore()
  const activeAgents = useActiveAgents()

  return (
    <div className="w-full h-screen bg-gray-900 text-white overflow-hidden">
      {/* Header */}
      <header className="bg-gray-800 border-b border-gray-700 p-4">
        <div className="flex items-center justify-between">
          <h1 className="text-2xl font-bold text-white">XR-Swarm-Bridge Dashboard</h1>
          <div className="flex items-center space-x-4">
            <ConnectionStatus isConnected={isConnected} status={connectionStatus} />
            <NetworkMetrics stats={webrtcStats} />
          </div>
        </div>
      </header>

      {/* Navigation */}
      <nav className="bg-gray-800 border-b border-gray-700">
        <div className="flex space-x-1">
          {['overview', 'agents', 'missions', 'telemetry', 'logs'].map((tab) => (
            <button
              key={tab}
              onClick={() => setActiveTab(tab)}
              className={`px-6 py-3 text-sm font-medium capitalize transition-colors ${
                activeTab === tab
                  ? 'bg-blue-600 text-white border-b-2 border-blue-400'
                  : 'text-gray-300 hover:text-white hover:bg-gray-700'
              }`}
            >
              {tab}
            </button>
          ))}
        </div>
      </nav>

      {/* Content */}
      <main className="flex-1 overflow-auto p-6">
        {activeTab === 'overview' && <OverviewTab />}
        {activeTab === 'agents' && <AgentsTab />}
        {activeTab === 'missions' && <MissionsTab />}
        {activeTab === 'telemetry' && <TelemetryTab />}
        {activeTab === 'logs' && <LogsTab />}
      </main>
    </div>
  )
}

function ConnectionStatus({ isConnected, status }: { isConnected: boolean; status: string }) {
  return (
    <div className="flex items-center space-x-2">
      <div className={`w-3 h-3 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`} />
      <span className="text-sm">{status}</span>
    </div>
  )
}

function NetworkMetrics({ stats }: { stats: any }) {
  if (!stats) return null

  return (
    <div className="flex items-center space-x-4 text-sm">
      <div>Latency: {Math.round(stats.latency || 0)}ms</div>
      <div>Loss: {Math.round((stats.packetLoss || 0) * 100)}%</div>
      <div>BW: {Math.round((stats.bandwidth || 0) / 1000)}kbps</div>
    </div>
  )
}

function OverviewTab() {
  const activeAgents = useActiveAgents()
  const { selectedAgents, currentMission } = useSwarmStore()

  const agentsByStatus = activeAgents.reduce((acc, agent) => {
    acc[agent.status] = (acc[agent.status] || 0) + 1
    return acc
  }, {} as Record<string, number>)

  return (
    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6">
      {/* Agent Summary */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Agent Status</h3>
        <div className="space-y-2">
          <div className="flex justify-between">
            <span>Total Agents:</span>
            <span className="font-mono">{activeAgents.length}</span>
          </div>
          <div className="flex justify-between">
            <span>Selected:</span>
            <span className="font-mono">{selectedAgents.length}</span>
          </div>
          {Object.entries(agentsByStatus).map(([status, count]) => (
            <div key={status} className="flex justify-between">
              <span className="capitalize">{status.replace('_', ' ')}:</span>
              <span className="font-mono">{count}</span>
            </div>
          ))}
        </div>
      </div>

      {/* Mission Status */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Current Mission</h3>
        {currentMission ? (
          <div className="space-y-2">
            <div><strong>Name:</strong> {currentMission.name}</div>
            <div><strong>Status:</strong> <span className="capitalize">{currentMission.status}</span></div>
            <div><strong>Phases:</strong> {currentMission.phases.length}</div>
            {currentMission.startTime && (
              <div><strong>Duration:</strong> {Math.round((Date.now() - currentMission.startTime) / 1000)}s</div>
            )}
          </div>
        ) : (
          <div className="text-gray-400">No active mission</div>
        )}
      </div>

      {/* Quick Actions */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Quick Actions</h3>
        <div className="space-y-2">
          <button className="w-full bg-green-600 hover:bg-green-700 px-4 py-2 rounded">
            Start Mission
          </button>
          <button className="w-full bg-blue-600 hover:bg-blue-700 px-4 py-2 rounded">
            Formation Control
          </button>
          <button className="w-full bg-red-600 hover:bg-red-700 px-4 py-2 rounded">
            Emergency Stop
          </button>
        </div>
      </div>
    </div>
  )
}

function AgentsTab() {
  const activeAgents = useActiveAgents()
  const { selectedAgents, selectAgent, deselectAgent } = useSwarmStore()

  return (
    <div className="space-y-6">
      <div className="flex justify-between items-center">
        <h2 className="text-xl font-semibold">Active Agents ({activeAgents.length})</h2>
        <button className="bg-blue-600 hover:bg-blue-700 px-4 py-2 rounded">
          Refresh
        </button>
      </div>

      <div className="bg-gray-800 rounded-lg overflow-hidden">
        <table className="w-full">
          <thead className="bg-gray-700">
            <tr>
              <th className="text-left p-4">Select</th>
              <th className="text-left p-4">ID</th>
              <th className="text-left p-4">Type</th>
              <th className="text-left p-4">Status</th>
              <th className="text-left p-4">Battery</th>
              <th className="text-left p-4">Position</th>
              <th className="text-left p-4">Last Seen</th>
            </tr>
          </thead>
          <tbody>
            {activeAgents.map((agent) => (
              <tr key={agent.id} className="border-t border-gray-700 hover:bg-gray-750">
                <td className="p-4">
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
                <td className="p-4 font-mono">{agent.id}</td>
                <td className="p-4 capitalize">{agent.type}</td>
                <td className="p-4">
                  <span className={`px-2 py-1 rounded text-xs ${getStatusColor(agent.status)}`}>
                    {agent.status}
                  </span>
                </td>
                <td className="p-4">
                  <div className="flex items-center space-x-2">
                    <div className="w-20 bg-gray-600 rounded-full h-2">
                      <div
                        className={`h-2 rounded-full ${getBatteryColor(agent.battery)}`}
                        style={{ width: `${agent.battery}%` }}
                      />
                    </div>
                    <span className="text-sm">{agent.battery}%</span>
                  </div>
                </td>
                <td className="p-4 font-mono text-sm">
                  [{agent.position.map(p => p.toFixed(1)).join(', ')}]
                </td>
                <td className="p-4 text-sm">
                  {Math.round((Date.now() - agent.lastSeen) / 1000)}s ago
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  )
}

function MissionsTab() {
  const { currentMission, missionHistory } = useSwarmStore()

  return (
    <div className="space-y-6">
      <h2 className="text-xl font-semibold">Mission Control</h2>
      
      {currentMission && (
        <div className="bg-gray-800 rounded-lg p-6">
          <h3 className="text-lg font-semibold mb-4">Active Mission: {currentMission.name}</h3>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
            <div>
              <p className="text-gray-300 mb-2">{currentMission.description}</p>
              <div className="space-y-1">
                <div>Status: <span className="capitalize">{currentMission.status}</span></div>
                <div>Phases: {currentMission.phases.length}</div>
              </div>
            </div>
            <div>
              <h4 className="font-semibold mb-2">Mission Phases:</h4>
              <div className="space-y-2">
                {currentMission.phases.map((phase, index) => (
                  <div key={index} className="bg-gray-700 p-3 rounded">
                    <div className="font-medium">Phase {phase.phase}: {phase.description}</div>
                    <div className="text-sm text-gray-300 mt-1">
                      Assignments: {Object.keys(phase.assignments).length}
                    </div>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      )}

      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Mission History</h3>
        {missionHistory.length === 0 ? (
          <div className="text-gray-400">No completed missions</div>
        ) : (
          <div className="space-y-2">
            {missionHistory.slice(-10).map((mission, index) => (
              <div key={index} className="flex justify-between items-center p-3 bg-gray-700 rounded">
                <div>
                  <div className="font-medium">{mission.name}</div>
                  <div className="text-sm text-gray-300">{mission.description}</div>
                </div>
                <div className="text-right">
                  <div className={`text-sm px-2 py-1 rounded ${
                    mission.status === 'completed' ? 'bg-green-600' : 'bg-red-600'
                  }`}>
                    {mission.status}
                  </div>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  )
}

function TelemetryTab() {
  const activeAgents = useActiveAgents()
  const { webrtcStats } = useSwarmStore()

  return (
    <div className="space-y-6">
      <h2 className="text-xl font-semibold">Real-time Telemetry</h2>
      
      {/* Network Stats */}
      {webrtcStats && (
        <div className="bg-gray-800 rounded-lg p-6">
          <h3 className="text-lg font-semibold mb-4">Network Performance</h3>
          <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
            <div>
              <div className="text-2xl font-mono">{Math.round(webrtcStats.latency || 0)}ms</div>
              <div className="text-sm text-gray-400">Latency</div>
            </div>
            <div>
              <div className="text-2xl font-mono">{Math.round((webrtcStats.packetLoss || 0) * 100)}%</div>
              <div className="text-sm text-gray-400">Packet Loss</div>
            </div>
            <div>
              <div className="text-2xl font-mono">{Math.round((webrtcStats.bandwidth || 0) / 1000)}kbps</div>
              <div className="text-sm text-gray-400">Bandwidth</div>
            </div>
            <div>
              <div className="text-2xl font-mono">{webrtcStats.connectionState}</div>
              <div className="text-sm text-gray-400">Connection</div>
            </div>
          </div>
        </div>
      )}

      {/* Agent Telemetry */}
      <div className="bg-gray-800 rounded-lg p-6">
        <h3 className="text-lg font-semibold mb-4">Agent Telemetry</h3>
        <div className="space-y-4">
          {activeAgents.map((agent) => (
            <div key={agent.id} className="bg-gray-700 rounded p-4">
              <div className="flex justify-between items-center mb-2">
                <h4 className="font-medium">{agent.id}</h4>
                <span className={`px-2 py-1 rounded text-xs ${getStatusColor(agent.status)}`}>
                  {agent.status}
                </span>
              </div>
              <div className="grid grid-cols-2 md:grid-cols-4 gap-4 text-sm">
                <div>
                  <div className="text-gray-400">Position</div>
                  <div className="font-mono">[{agent.position.map(p => p.toFixed(2)).join(', ')}]</div>
                </div>
                <div>
                  <div className="text-gray-400">Battery</div>
                  <div className={`font-mono ${getBatteryTextColor(agent.battery)}`}>{agent.battery}%</div>
                </div>
                <div>
                  <div className="text-gray-400">Type</div>
                  <div className="capitalize">{agent.type}</div>
                </div>
                <div>
                  <div className="text-gray-400">Last Seen</div>
                  <div>{Math.round((Date.now() - agent.lastSeen) / 1000)}s ago</div>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  )
}

function LogsTab() {
  return (
    <div className="space-y-6">
      <h2 className="text-xl font-semibold">System Logs</h2>
      <div className="bg-gray-800 rounded-lg p-6">
        <div className="bg-black rounded p-4 font-mono text-sm h-96 overflow-y-auto">
          <div className="text-green-400">[2024-01-01 12:00:00] WebRTC connection established</div>
          <div className="text-blue-400">[2024-01-01 12:00:01] Agent drone_01 connected</div>
          <div className="text-blue-400">[2024-01-01 12:00:02] Agent ugv_01 connected</div>
          <div className="text-yellow-400">[2024-01-01 12:00:05] Warning: High latency detected (&gt;500ms)</div>
          <div className="text-green-400">[2024-01-01 12:00:10] Mission "Search Pattern Alpha" started</div>
          <div className="text-white">[2024-01-01 12:00:15] Formation command sent to 5 agents</div>
        </div>
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

function getBatteryColor(level: number): string {
  if (level > 60) return 'bg-green-500'
  if (level > 30) return 'bg-yellow-500'
  return 'bg-red-500'
}

function getBatteryTextColor(level: number): string {
  if (level > 60) return 'text-green-400'
  if (level > 30) return 'text-yellow-400'
  return 'text-red-400'
}