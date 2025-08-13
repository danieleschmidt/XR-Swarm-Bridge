import React from 'react'
import { Link } from 'react-router-dom'
import { useSwarmStore } from '../store/swarmStore'
import { useTranslation } from 'react-i18next'
import TelemetryPanel from './TelemetryPanel'
import RobotPanel from './RobotPanel'
import { QuantumDashboard } from './QuantumDashboard'
import { AutonomousPlanningPanel } from './AutonomousPlanningPanel'

export default function Dashboard() {
  const { t } = useTranslation()
  const { agents, isConnected } = useSwarmStore()

  return (
    <div className="min-h-screen bg-gray-900 text-white">
      {/* Header */}
      <header className="bg-black/50 backdrop-blur border-b border-gray-800">
        <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
          <div className="flex justify-between items-center py-4">
            <div className="flex items-center space-x-4">
              <Link to="/" className="text-blue-400 hover:text-blue-300">
                ← Back to XR View
              </Link>
              <h1 className="text-2xl font-bold">XR-Swarm-Bridge Dashboard</h1>
            </div>
            
            <div className="flex items-center space-x-4">
              <div className={`flex items-center space-x-2 ${
                isConnected ? 'text-green-400' : 'text-red-400'
              }`}>
                <div className={`w-2 h-2 rounded-full ${
                  isConnected ? 'bg-green-400' : 'bg-red-400'
                }`} />
                <span className="text-sm">
                  {isConnected ? 'Connected' : 'Disconnected'}
                </span>
              </div>
              
              <div className="text-sm text-gray-400">
                {agents.length} robots active
              </div>
            </div>
          </div>
        </div>
      </header>

      {/* Main Content */}
      <main className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-8">
        <div className="grid grid-cols-1 lg:grid-cols-2 xl:grid-cols-3 gap-6">
          {/* System Status */}
          <div className="lg:col-span-1 xl:col-span-2">
            <TelemetryPanel />
          </div>

          {/* Robot Management */}
          <div className="lg:col-span-1">
            <RobotPanel />
          </div>

          {/* Quantum Optimization */}
          <div className="lg:col-span-1 xl:col-span-2">
            <QuantumDashboard />
          </div>

          {/* Autonomous Planning */}
          <div className="lg:col-span-1">
            <AutonomousPlanningPanel />
          </div>
        </div>

        {/* Quick Actions */}
        <div className="mt-8">
          <div className="bg-gray-800 rounded-lg p-6">
            <h3 className="text-lg font-semibold mb-4">Quick Actions</h3>
            <div className="grid grid-cols-2 md:grid-cols-4 gap-4">
              <button className="p-4 bg-blue-600 hover:bg-blue-700 rounded-lg text-center transition-colors">
                <div className="text-2xl mb-2">🛫</div>
                <div className="text-sm">Takeoff All</div>
              </button>
              
              <button className="p-4 bg-red-600 hover:bg-red-700 rounded-lg text-center transition-colors">
                <div className="text-2xl mb-2">🛑</div>
                <div className="text-sm">Emergency Stop</div>
              </button>
              
              <button className="p-4 bg-green-600 hover:bg-green-700 rounded-lg text-center transition-colors">
                <div className="text-2xl mb-2">🔄</div>
                <div className="text-sm">Return to Base</div>
              </button>
              
              <button className="p-4 bg-purple-600 hover:bg-purple-700 rounded-lg text-center transition-colors">
                <div className="text-2xl mb-2">🤖</div>
                <div className="text-sm">AI Mode</div>
              </button>
            </div>
          </div>
        </div>
      </main>
    </div>
  )
}