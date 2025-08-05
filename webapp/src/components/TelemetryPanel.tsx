```tsx
import React, { useState, useEffect } from 'react'
import { useSwarmStore, useSelectedAgents } from '../store/swarmStore'

export default function TelemetryPanel() {
  const selectedAgents = useSelectedAgents()
  const { latency, packetLoss, bandwidth, webrtcStats } = useSwarmStore()
  const [selectedAgentId, setSelectedAgentId] = useState<string>('')
  const [telemetryHistory, setTelemetryHistory] = useState<any[]>([])

  // Auto-select first agent if none selected
  useEffect(() => {
    if (selectedAgents.length > 0 && !selectedAgentId) {
      setSelectedAgentId(selectedAgents[0].id)
    } else if (selectedAgents.length === 0) {
      setSelectedAgentId('')
    }
  }, [selectedAgents, selectedAgentId])

  const selectedAgent = selectedAgents.find(a => a.id === selectedAgentId)

  // Mock telemetry history (in real implementation, this would come from the store)
  useEffect(() => {
    const interval = setInterval(() => {
      if (selectedAgent) {
        setTelemetryHistory(prev => [
          ...prev.slice(-49), // Keep last 50 entries
          {
            timestamp: Date.now(),
            battery: selectedAgent.battery,
            position: [...selectedAgent.position],
            velocity: selectedAgent.telemetry?.velocity || [0, 0, 0],
            temperature: Math.round(25 + Math.random() * 10),
            cpu_usage: Math.round(20 + Math.random() * 60),
            memory_usage: Math.round(30 + Math.random() * 40),
            signal_strength: Math.round(70 + Math.random() * 30)
          }
        ])
      }
    }, 1000)

    return () => clearInterval(interval)
  }, [selectedAgent])

  if (selectedAgents.length === 0) {
    return (
      <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg p-6">
        <h3 className="text-lg font-semibold text-white mb-4">Telemetry</h3>
        <div className="text-center text-gray-400">
          <p>Select a robot to view telemetry</p>
        </div>
      </div>
    )
  }

  // Use webrtcStats if available, otherwise fall back to individual values
  const networkStats = webrtcStats || { latency, packetLoss, bandwidth }

  return (
    <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg h-full flex flex-col">
      {/* Header */}
      <div className="p-4 border-b border-gray-700">
        <h3 className="text-lg font-semibold text-white mb-3">Telemetry</h3>
        
        {/* Agent Selector */}
        {selectedAgents.length > 1 && (
          <select
            value={selectedAgentId}
            onChange={(e) => setSelectedAgentId(e.target.value)}
            className="w-full px-3 py-2 bg-gray-800 border border-gray-600 rounded-md text-white text-sm focus:border-blue-500 focus:outline-none"
          >
            {selectedAgents.map(agent => (
              <option key={agent.id} value={agent.id}>
                {agent.id} ({agent.type})
              </option>
            ))}
          </select>
        )}
      </div>

      <div className="flex-1 overflow-y-auto p-4 space-y-4">
        {selectedAgent && (
          <>
            {/* Basic Info */}
            <div className="bg-gray-800/50 rounded-lg p-4">
              <h4 className="text-sm font-medium text-white mb-3">Basic Info</h4>
              <div className="grid grid-cols-2 gap-3 text-sm">
                <div>
                  <span className="text-gray-400">Type:</span>
                  <span className="ml-2 text-white capitalize">{selectedAgent.type}</span>
                </div>
                <div>
                  <span className="text-gray-400">Status:</span>
                  <span className={`ml-2 capitalize ${getStatusColor(selectedAgent.status)}`}>
                    {selectedAgent.status}
                  </span>
                </div>
                <div>
                  <span className="text-gray-400">Battery:</span>
                  <span className={`ml-2 ${getBatteryColor(selectedAgent.battery)}`}>
                    {Math.round(selectedAgent.battery)}%
                  </span>
                </div>
                <div>
                  <span className="text-gray-400">Last Seen:</span>
                  <span className="ml-2 text-white">
                    {Math.round((Date.now() - selectedAgent.lastSeen) / 1000)}s ago
                  </span>
                </div>
              </div>
            </div>

            {/* Position & Movement */}
            <div className="bg-gray-800/50 rounded-lg p-4">
              <h4 className="text-sm font-medium text-white mb-3">Position & Movement</h4>
              <div className="space-y-2 text-sm">
                <div>
                  <span className="text-gray-400">Position:</span>
                  <div className="font-mono text-white">
                    X: {selectedAgent.position[0].toFixed(2)}m<br/>
                    Y: {selectedAgent.position[1].toFixed(2)}m<br/>
                    Z: {selectedAgent.position[2].toFixed(2)}m
                  </div>
                </div>
                {selectedAgent.telemetry?.velocity && (
                  <div>
                    <span className="text-gray-400">Velocity:</span>
                    <div className="font-mono text-white">
                      {selectedAgent.telemetry.velocity.map((v: number) => v.toFixed(2)).join(', ')} m/s
                    </div>
                  </div>
                )}
                {selectedAgent.telemetry?.altitude !== undefined && (
                  <div>
                    <span className="text-gray-400">Altitude:</span>
                    <span className="ml-2 font-mono text-white">{selectedAgent.telemetry.altitude.toFixed(1)}m</span>
                  </div>
                )}
                {selectedAgent.telemetry?.waypoints_remaining !== undefined && (
                  <div>
                    <span className="text-gray-400">Waypoints:</span>
                    <span className="ml-2 font-mono text-white">{selectedAgent.telemetry.waypoints_remaining}</span>
                  </div>
                )}
              </div>
            </div>

            {/* System Status */}
            <div className="bg-gray-800/50 rounded-lg p-4">
              <h4 className="text-sm font-medium text-white mb-3">System Status</h4>
              <div className="space-y-3">
                {telemetryHistory.length > 0 && (
                  <>
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400 text-sm">Temperature:</span>
                      <span className="text-white">
                        {telemetryHistory[telemetryHistory.length - 1]?.temperature}°C
                      </span>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400 text-sm">CPU Usage:</span>
                      <div className="flex items-center space-x-2">
                        <div className="w-16 h-2 bg-gray-700 rounded-full">
                          <div 
                            className="h-2 bg-blue-500 rounded-full transition-all"
                            style={{ width: `${telemetryHistory[telemetryHistory.length - 1]?.cpu_usage}%` }}
                          />
                        </div>
                        <span className="text-white text-sm">
                          {telemetryHistory[telemetryHistory.length - 1]?.cpu_usage}%
                        </span>
                      </div>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400 text-sm">Memory:</span>
                      <div className="flex items-center space-x-2">
                        <div className="w-16 h-2 bg-gray-700 rounded-full">
                          <div 
                            className="h-2 bg-green-500 rounded-full transition-all"
                            style={{ width: `${telemetryHistory[telemetryHistory.length - 1]?.memory_usage}%` }}
                          />
                        </div>
                        <span className="text-white text-sm">
                          {telemetryHistory[telemetryHistory.length - 1]?.memory_usage}%
                        </span>
                      </div>
                    </div>
                    <div className="flex items-center justify-between">
                      <span className="text-gray-400 text-sm">Signal:</span>
                      <div className="flex items-center space-x-2">
                        <div className="w-16 h-2 bg-gray-700 rounded-full">
                          <div 
                            className="h-2 bg-yellow-500 rounded-full transition-all"
                            style={{ width: `${telemetryHistory[telemetryHistory.length - 1]?.signal_strength}%` }}
                          />
                        </div>
                        <span className="text-white text-sm">
                          {telemetryHistory[telemetryHistory.length - 1]?.signal_strength}%
                        </span>
                      </div>
                    </div>
                  </>
                )}
              </div>
            </div>

            {/* Capabilities */}
            <div className="bg-gray-800/50 rounded-lg p-4">
              <h4 className="text-sm font-medium text-white mb-3">Capabilities</h4>
              <div className="flex flex-wrap gap-2">
                {selectedAgent.capabilities.length > 0 ? (
                  selectedAgent.capabilities.map(capability => (
                    <span
                      key={capability}
                      className="px-2 py-1 bg-blue-600 text-white text-xs rounded-md"
                    >
                      {capability}
                    </span>
                  ))
                ) : (
                  <span className="text-gray-400 text-sm">No capabilities defined</span>
                )}
              </div>
            </div>

            {/* Network Stats */}
            <div className="bg-gray-800/50 rounded-lg p-4">
              <h4 className="text-sm font-medium text-white mb-3">Network</h4>
              <div className="space-y-2 text-sm">
                <div className="flex items-center justify-between">
                  <span className="text-gray-400">Latency:</span>
                  <span className={networkStats.latency > 200 ? 'text-red-400' : networkStats.latency > 100 ? 'text-yellow-400' : 'text-green-400'}>
                    {Math.round(networkStats.latency || 0)}ms
                  </span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-gray-400">Packet Loss:</span>
                  <span className={networkStats.packetLoss > 0.05 ? 'text-red-400' : 'text-green-400'}>
                    {(networkStats.packetLoss * 100).toFixed(1)}%
                  </span>
                </div>
                <div className="flex items-center justify-between">
                  <span className="text-gray-400">Bandwidth:</span>
                  <span className="text-blue-400">
                    {(networkStats.bandwidth / 1000).toFixed(1)} Mbps
                  </span>
                </div>
              </div>
            </div>

            {/* Battery History Chart */}
            {telemetryHistory.length > 1 && (
              <div className="bg-gray-800/50 rounded-lg p-4">
                <h4 className="text-sm font-medium text-white mb-3">Battery History</h4>
                <div className="relative h-16 bg-gray-700 rounded">
                  <svg className="w-full h-full">
                    <polyline
                      fill="none"
                      stroke="#10b981"
                      strokeWidth="2"
                      points={telemetryHistory
                        .slice(-20) // Last 20 data points
                        .map((data, index) => {
                          const x = (index / 19) * 100
                          const y = 100 - data.battery
                          return `${x},${y}`
                        })
                        .join(' ')}
                      vectorEffect="non-scaling-stroke"
                      transform="scale(1,0.64)"
                    />
                  </svg>
                  <div className="absolute inset-0 flex items-center justify-center text-xs text-gray-400">
                    {Math.round(selectedAgent.battery)}%
                  </div>
                </div>
              </div>
            )}
          </>
        )}
      </div>
    </div>
  )
}

function getStatusColor(status: string): string {
  switch (status) {
    case 'active': return 'text-green-400'
    case 'idle': return 'text-blue-400'
    case 'error': return 'text-red-400'
    case 'emergency_stop': return 'text-orange-400'
    case 'timeout': return 'text-gray-400'
    default: return 'text-gray-400'
  }
}

function getBatteryColor(battery: number): string {
  if (battery > 60) return 'text-green-400'
  if (battery > 30) return 'text-yellow-400'
  return 'text-red-400'
}
```
