import React, { useState } from 'react'
import { useSwarmStore, useActiveAgents } from '../store/swarmStore'
import RobotPanel from './RobotPanel'
import ControlPanel from './ControlPanel'
import Minimap from './Minimap'
import TelemetryPanel from './TelemetryPanel'
import CommandInterface from './CommandInterface'
import { AutonomousPlanningPanel } from './AutonomousPlanningPanel'
import { QuantumDashboard } from './QuantumDashboard'

export default function UI() {
  const { showTelemetry, showMinimap, viewMode } = useSwarmStore()
  const activeAgents = useActiveAgents()
  const [showAutonomousPlanning, setShowAutonomousPlanning] = useState(false)
  const [showQuantumDashboard, setShowQuantumDashboard] = useState(false)

  // Don't show 2D UI in VR/AR mode
  if (viewMode === 'vr' || viewMode === 'ar') {
    return null
  }

  return (
    <div className="fixed inset-0 pointer-events-none z-10">
      {/* Robot Panel - Left Side */}
      <div className="absolute left-4 top-20 bottom-20 w-80 pointer-events-auto">
        <RobotPanel />
      </div>

      {/* Telemetry Panel - Right Side */}
      {showTelemetry && (
        <div className="absolute right-4 top-20 w-80 pointer-events-auto">
          <TelemetryPanel />
        </div>
      )}

      {/* Minimap - Top Right */}
      {showMinimap && (
        <div className="absolute top-16 right-4 pointer-events-auto">
          <Minimap />
        </div>
      )}

      {/* Control Panel - Bottom */}
      <div className="absolute bottom-4 left-4 right-4 pointer-events-auto">
        <ControlPanel />
      </div>

      {/* Command Interface - Top Center */}
      <div className="absolute top-4 left-1/2 transform -translate-x-1/2 pointer-events-auto">
        <CommandInterface />
      </div>

      {/* Stats Display - Top Left */}
      <div className="absolute top-16 left-4 pointer-events-auto">
        <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg p-3 text-sm">
          <div className="text-green-400 font-mono">
            Active Agents: {activeAgents.length}
          </div>
          <div className="text-blue-400 font-mono">
            View: {viewMode.toUpperCase()}
          </div>
        </div>
      </div>

      {/* Advanced Controls - Top Right */}
      <div className="absolute top-4 right-4 pointer-events-auto flex gap-2">
        <button
          onClick={() => setShowQuantumDashboard(true)}
          className="bg-purple-600 hover:bg-purple-700 px-3 py-2 rounded text-sm font-medium transition-colors"
          title="Open Quantum Optimization Dashboard"
        >
          🔬 Quantum
        </button>
        <button
          onClick={() => setShowAutonomousPlanning(true)}
          className="bg-blue-600 hover:bg-blue-700 px-3 py-2 rounded text-sm font-medium transition-colors"
          title="Open Autonomous Planning Interface"
        >
          🧠 Autonomous
        </button>
      </div>

      {/* Autonomous Planning Panel */}
      <AutonomousPlanningPanel 
        isVisible={showAutonomousPlanning}
        onClose={() => setShowAutonomousPlanning(false)}
      />

      {/* Quantum Dashboard */}
      <QuantumDashboard 
        isVisible={showQuantumDashboard}
        onClose={() => setShowQuantumDashboard(false)}
      />
    </div>
  )
}