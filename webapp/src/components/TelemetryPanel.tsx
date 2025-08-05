import React from 'react'
import { useSwarmStore, useActiveAgents } from '../store/swarmStore'

export default function TelemetryPanel() {
  const { selectedAgents, webrtcStats } = useSwarmStore()
  const activeAgents = useActiveAgents()
  
  const selectedAgentData = activeAgents.filter(agent => 
    selectedAgents.includes(agent.id)
  )

  return (
    <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg p-4 max-h-96 overflow-y-auto">
      <h3 className="text-lg font-semibold text-white mb-4">Telemetry</h3>
      
      {/* Network Stats */}
      {webrtcStats && (
        <div className="mb-4 p-3 bg-gray-900 rounded">
          <h4 className="text-sm font-semibold text-gray-300 mb-2">Network</h4>
          <div className="text-xs space-y-1">
            <div className="flex justify-between">
              <span>Latency:</span>
              <span className="font-mono">{Math.round(webrtcStats.latency || 0)}ms</span>
            </div>
            <div className="flex justify-between">
              <span>Packet Loss:</span>
              <span className="font-mono">{Math.round((webrtcStats.packetLoss || 0) * 100)}%</span>
            </div>
            <div className="flex justify-between">
              <span>Bandwidth:</span>
              <span className="font-mono">{Math.round((webrtcStats.bandwidth || 0) / 1000)}kbps</span>
            </div>
          </div>
        </div>
      )}
      
      {/* Selected Agents Telemetry */}
      <div className="space-y-3">
        {selectedAgentData.length === 0 ? (
          <div className="text-gray-400 text-sm">Select agents to view telemetry</div>
        ) : (
          selectedAgentData.map((agent) => (
            <div key={agent.id} className="p-3 bg-gray-900 rounded">
              <div className="flex justify-between items-center mb-2">
                <h4 className="text-sm font-semibold text-white">{agent.id}</h4>
                <span className={`text-xs px-2 py-1 rounded ${getStatusColor(agent.status)}`}>
                  {agent.status}
                </span>
              </div>
              
              <div className="text-xs space-y-1">
                <div className="flex justify-between">
                  <span>Position:</span>
                  <span className="font-mono">[{agent.position.map(p => p.toFixed(2)).join(', ')}]</span>
                </div>
                
                <div className="flex justify-between">
                  <span>Battery:</span>
                  <div className="flex items-center space-x-2">
                    <div className="w-16 bg-gray-600 rounded-full h-1">
                      <div
                        className={`h-1 rounded-full ${getBatteryColor(agent.battery)}`}
                        style={{ width: `${agent.battery}%` }}
                      />
                    </div>
                    <span className="font-mono">{agent.battery}%</span>
                  </div>
                </div>
                
                <div className="flex justify-between">
                  <span>Last Seen:</span>
                  <span className="font-mono">{Math.round((Date.now() - agent.lastSeen) / 1000)}s</span>
                </div>
                
                {agent.telemetry?.altitude !== undefined && (
                  <div className="flex justify-between">
                    <span>Altitude:</span>
                    <span className="font-mono">{agent.telemetry.altitude.toFixed(1)}m</span>
                  </div>
                )}
                
                {agent.telemetry?.waypoints_remaining !== undefined && (
                  <div className="flex justify-between">
                    <span>Waypoints:</span>
                    <span className="font-mono">{agent.telemetry.waypoints_remaining}</span>
                  </div>
                )}
              </div>
            </div>
          ))
        )}
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