import React, { useState, useRef, useEffect } from 'react'
import { useGPTIntegration } from '../ai/gptIntegration'
import { useSwarmConnection } from '../hooks/useSwarmConnection'
import { useSelectedAgents, useSwarmStore } from '../store/swarmStore'

export default function CommandInterface() {
  const [isVisible, setIsVisible] = useState(false)
  const [command, setCommand] = useState('')
  const [isListening, setIsListening] = useState(false)
  const [commandHistory, setCommandHistory] = useState<string[]>([])
  const [historyIndex, setHistoryIndex] = useState(-1)
  
  const inputRef = useRef<HTMLInputElement>(null)
  const selectedAgents = useSelectedAgents()
  const { generateCommands, isLoading } = useGPTIntegration()
  const { sendGlobalCommand, sendAgentCommand } = useSwarmConnection()
  const { isConnected, sendCommand } = useSwarmStore()

  // Toggle visibility with keyboard shortcut
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === '/' && !isVisible) {
        e.preventDefault()
        setIsVisible(true)
        setTimeout(() => inputRef.current?.focus(), 100)
      } else if (e.key === 'Escape' && isVisible) {
        setIsVisible(false)
        setCommand('')
        setHistoryIndex(-1)
      }
    }

    window.addEventListener('keydown', handleKeyDown)
    return () => window.removeEventListener('keydown', handleKeyDown)
  }, [isVisible])

  // Handle command history navigation
  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'ArrowUp') {
      e.preventDefault()
      const newIndex = Math.min(historyIndex + 1, commandHistory.length - 1)
      setHistoryIndex(newIndex)
      setCommand(commandHistory[commandHistory.length - 1 - newIndex] || '')
    } else if (e.key === 'ArrowDown') {
      e.preventDefault()
      const newIndex = Math.max(historyIndex - 1, -1)
      setHistoryIndex(newIndex)
      setCommand(newIndex === -1 ? '' : commandHistory[commandHistory.length - 1 - newIndex] || '')
    } else if (e.key === 'Enter') {
      handleSubmit()
    }
  }

  const parseSimpleCommand = (commandText: string) => {
    const parts = commandText.toLowerCase().trim().split(' ')
    const cmd = parts[0]
    
    switch (cmd) {
      case 'takeoff':
        return { type: 'takeoff', params: { altitude: parseFloat(parts[1]) || 5.0 } }
      case 'land':
        return { type: 'land', params: {} }
      case 'hover':
        return { type: 'hover', params: {} }
      case 'stop':
        return { type: 'emergency_stop', params: {} }
      case 'rtb':
      case 'return':
        return { type: 'return_to_base', params: {} }
      default:
        return null
    }
  }

  const handleSubmit = async () => {
    if (!command.trim() || isLoading) return

    // Add to history
    const newHistory = [command, ...commandHistory.slice(0, 9)] // Keep last 10
    setCommandHistory(newHistory)
    setHistoryIndex(-1)

    // First try to parse as simple command
    const simpleCmd = parseSimpleCommand(command)
    if (simpleCmd) {
      sendCommand(simpleCmd.type, simpleCmd.params)
    } else {
      // Process with AI for complex commands
      const result = await generateCommands(command, {
        agents: selectedAgents
      })

      if (result.commands) {
        result.commands.forEach(cmd => {
          if (cmd.target === 'all') {
            sendGlobalCommand(cmd.parameters)
          } else if (Array.isArray(cmd.target)) {
            cmd.target.forEach(agentId => {
              sendAgentCommand(agentId, cmd.parameters)
            })
          } else if (cmd.target === 'selected' && selectedAgents.length > 0) {
            selectedAgents.forEach(agent => {
              sendAgentCommand(agent.id, cmd.parameters)
            })
          }
        })
      }
    }

    setCommand('')
    setIsVisible(false)
  }

  // Speech recognition setup
  useEffect(() => {
    if ('webkitSpeechRecognition' in window) {
      const recognition = new (window as any).webkitSpeechRecognition()
      recognition.continuous = false
      recognition.interimResults = false
      recognition.lang = 'en-US'

      recognition.onresult = (event: any) => {
        const transcript = event.results[0][0].transcript
        setCommand(transcript)
        setIsListening(false)
      }

      recognition.onerror = () => {
        setIsListening(false)
      }

      recognition.onend = () => {
        setIsListening(false)
      }

      if (isListening) {
        recognition.start()
      }

      return () => {
        if (isListening) {
          recognition.stop()
        }
      }
    }
  }, [isListening])

  const startListening = () => {
    if ('webkitSpeechRecognition' in window) {
      setIsListening(true)
    }
  }

  if (!isVisible) {
    return (
      <div className="flex items-center space-x-2">
        <button
          onClick={() => setIsVisible(true)}
          className="px-3 py-2 bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg text-white hover:border-blue-500 transition-colors"
        >
          <span className="mr-2">🤖</span>
          AI Command
          <span className="ml-2 text-xs text-gray-400">/</span>
        </button>
        
        <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`} />
        
        {selectedAgents.length > 0 && (
          <div className="px-2 py-1 bg-blue-600/80 backdrop-blur-sm rounded text-xs text-white">
            {selectedAgents.length} selected
          </div>
        )}
      </div>
    )
  }

  return (
    <div className="bg-black/90 backdrop-blur-sm border border-gray-700 rounded-lg p-4 min-w-96 max-w-2xl">
      <div className="flex items-center justify-between mb-3">
        <h4 className="text-sm font-medium text-white flex items-center">
          <span className="mr-2">🤖</span>
          AI Command Interface
          <div className={`ml-2 w-2 h-2 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`} />
        </h4>
        <button
          onClick={() => setIsVisible(false)}
          className="text-gray-400 hover:text-white"
        >
          ✕
        </button>
      </div>

      <div className="space-y-3">
        {/* Target Info */}
        <div className="text-xs text-gray-400">
          Target: {selectedAgents.length > 0 
            ? `${selectedAgents.length} selected robot${selectedAgents.length !== 1 ? 's' : ''}`
            : 'All robots'
          }
        </div>

        {/* Command Input */}
        <div className="flex space-x-2">
          <input
            ref={inputRef}
            type="text"
            value={command}
            onChange={(e) => setCommand(e.target.value)}
            onKeyDown={handleKeyDown}
            placeholder="Enter natural language command..."
            className="flex-1 px-3 py-2 bg-gray-800 border border-gray-600 rounded-md text-white text-sm focus:border-blue-500 focus:outline-none"
            disabled={isLoading || !isConnected}
          />
          
          {'webkitSpeechRecognition' in window && (
            <button
              onClick={startListening}
              disabled={isListening || isLoading || !isConnected}
              className={`px-3 py-2 rounded-md text-sm transition-colors ${
                isListening 
                  ? 'bg-red-600 text-white' 
                  : 'bg-gray-700 hover:bg-gray-600 text-gray-300'
              }`}
            >
              {isListening ? '🎤' : '🎙️'}
            </button>
          )}
          
          <button
            onClick={handleSubmit}
            disabled={!command.trim() || isLoading || !isConnected}
            className="px-4 py-2 bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 rounded-md text-sm font-medium transition-colors"
          >
            {isLoading ? 'Processing...' : 'Send'}
          </button>
        </div>

        {/* Quick Commands */}
        <div className="grid grid-cols-2 gap-2">
          {[
            'Form a circle formation',
            'Return to base',
            'Search the area',
            'Emergency stop all',
            'Create perimeter',
            'Stack vertically'
          ].map((quickCommand, index) => (
            <button
              key={index}
              onClick={() => setCommand(quickCommand)}
              className="text-left px-2 py-1 bg-gray-800 hover:bg-gray-700 rounded text-xs text-gray-300 transition-colors"
            >
              "{quickCommand}"
            </button>
          ))}
        </div>

        {/* Command History */}
        {commandHistory.length > 0 && (
          <div className="border-t border-gray-700 pt-3">
            <div className="text-xs text-gray-400 mb-2">Recent Commands:</div>
            <div className="space-y-1 max-h-20 overflow-y-auto">
              {commandHistory.slice(0, 3).map((histCmd, index) => (
                <button
                  key={index}
                  onClick={() => setCommand(histCmd)}
                  className="block w-full text-left px-2 py-1 bg-gray-800 hover:bg-gray-700 rounded text-xs text-gray-300 truncate"
                >
                  {histCmd}
                </button>
              ))}
            </div>
          </div>
        )}

        {/* Help */}
        <div className="border-t border-gray-700 pt-3 text-xs text-gray-400">
          <div className="flex items-center justify-between">
            <span>Press <kbd className="px-1 bg-gray-700 rounded">ESC</kbd> to close</span>
            <span>Use <kbd className="px-1 bg-gray-700 rounded">↑</kbd><kbd className="px-1 bg-gray-700 rounded">↓</kbd> for history</span>
          </div>
        </div>
      </div>
    </div>
  )
}
