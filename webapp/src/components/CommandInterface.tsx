import React, { useState } from 'react'
import { useSwarmStore } from '../store/swarmStore'

export default function CommandInterface() {
  const [command, setCommand] = useState('')
  const { isConnected, sendCommand } = useSwarmStore()

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault()
    if (command.trim()) {
      // Parse simple commands
      const parts = command.toLowerCase().trim().split(' ')
      const cmd = parts[0]
      
      switch (cmd) {
        case 'takeoff':
          sendCommand('takeoff', { altitude: parseFloat(parts[1]) || 5.0 })
          break
        case 'land':
          sendCommand('land')
          break
        case 'hover':
          sendCommand('hover')
          break
        case 'stop':
          sendCommand('emergency_stop')
          break
        case 'rtb':
        case 'return':
          sendCommand('return_to_base')
          break
        default:
          sendCommand('custom', { command })
      }
      
      setCommand('')
    }
  }

  return (
    <form onSubmit={handleSubmit} className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg p-2">
      <div className="flex items-center space-x-2">
        <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`} />
        <input
          type="text"
          value={command}
          onChange={(e) => setCommand(e.target.value)}
          placeholder="Enter command..."
          className="bg-transparent text-white placeholder-gray-400 text-sm flex-1 outline-none"
          disabled={!isConnected}
        />
        <button
          type="submit"
          disabled={!isConnected || !command.trim()}
          className="text-blue-400 hover:text-blue-300 disabled:text-gray-600 text-sm px-2"
        >
          Send
        </button>
      </div>
    </form>
  )
}