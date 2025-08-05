import React, { useState, useRef, useEffect } from 'react'
import { useSwarmStore, useActiveAgents } from '../store/swarmStore'
import { useGPTIntegration } from '../ai/gptIntegration'

interface AICommandInterfaceProps {
  isOpen: boolean
  onClose: () => void
}

export default function AICommandInterface({ isOpen, onClose }: AICommandInterfaceProps) {
  const [mode, setMode] = useState<'command' | 'mission' | 'analysis'>('command')
  const [input, setInput] = useState('')
  const [output, setOutput] = useState('')
  const [history, setHistory] = useState<Array<{ input: string; output: string; timestamp: number }>>([])
  
  const { generateMissionPlan, generateCommands, analyzePerformance, isLoading, error } = useGPTIntegration()
  const { selectedAgents, sendCommand, webrtcStats } = useSwarmStore()
  const activeAgents = useActiveAgents()
  
  const inputRef = useRef<HTMLTextAreaElement>(null)

  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus()
    }
  }, [isOpen])

  const handleSubmit = async () => {
    if (!input.trim() || isLoading) return

    const context = {
      agents: activeAgents.map(agent => ({
        ...agent,
        selected: selectedAgents.includes(agent.id)
      })),
      stats: webrtcStats
    }

    let result: any = {}
    
    try {
      switch (mode) {
        case 'command':
          result = await generateCommands(input, context)
          if (result.commands) {
            // Execute the generated commands
            result.commands.forEach((cmd: any) => {
              sendCommand(cmd.type, cmd.parameters)
            })
            setOutput(`Generated and executed ${result.commands.length} commands:\n${JSON.stringify(result.commands, null, 2)}`)
          } else if (result.error) {
            setOutput(`Error: ${result.error}`)
          }
          break
          
        case 'mission':
          result = await generateMissionPlan(input, context)
          if (result.plan) {
            setOutput(`Mission Plan Generated:\n\nName: ${result.plan.name}\nDescription: ${result.plan.description}\nPhases: ${result.plan.phases.length}\nEstimated Duration: ${result.plan.estimatedDuration} minutes\n\nDetailed Plan:\n${JSON.stringify(result.plan, null, 2)}`)
          } else if (result.error) {
            setOutput(`Error: ${result.error}`)
          }
          break
          
        case 'analysis':
          result = await analyzePerformance(context)
          if (result.analysis) {
            setOutput(result.analysis)
          } else if (result.error) {
            setOutput(`Error: ${result.error}`)
          }
          break
      }
      
      // Add to history
      setHistory(prev => [...prev, {
        input,
        output: result.error || output,
        timestamp: Date.now()
      }])
      
      setInput('')
    } catch (err: any) {
      setOutput(`Error: ${err.message}`)
    }
  }

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && (e.ctrlKey || e.metaKey)) {
      handleSubmit()
    }
  }

  const getExamplePrompts = () => {
    switch (mode) {
      case 'command':
        return [
          'Form a search grid over the warehouse',
          'Move selected drones to defensive positions',
          'Create a perimeter patrol around the building',
          'Return all agents to base immediately',
          'Form a circular formation around coordinates 50,50'
        ]
      case 'mission':
        return [
          'Search and rescue mission in a collapsed building',
          'Surveillance of a 100x100 meter area',
          'Package delivery to multiple locations',
          'Environmental monitoring of forest area',
          'Security patrol of industrial facility'
        ]
      case 'analysis':
        return [
          'Analyze current swarm performance',
          'Identify bottlenecks in formation efficiency',
          'Evaluate battery consumption patterns',
          'Assess communication latency issues',
          'Review mission completion metrics'
        ]
    }
  }

  if (!isOpen) return null

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-gray-800 rounded-lg w-4/5 h-4/5 max-w-6xl max-h-screen flex flex-col">
        {/* Header */}
        <div className="flex items-center justify-between p-4 border-b border-gray-700">
          <h2 className="text-xl font-bold text-white flex items-center">
            <span className="mr-2">🤖</span>
            AI Command Interface - GPT-4o
          </h2>
          <button
            onClick={onClose}
            className="text-gray-400 hover:text-white text-2xl"
          >
            ×
          </button>
        </div>

        {/* Mode Selector */}
        <div className="flex border-b border-gray-700">
          {['command', 'mission', 'analysis'].map((modeOption) => (
            <button
              key={modeOption}
              onClick={() => setMode(modeOption as any)}
              className={`px-6 py-3 capitalize transition-colors ${
                mode === modeOption
                  ? 'bg-blue-600 text-white'
                  : 'text-gray-300 hover:text-white hover:bg-gray-700'
              }`}
            >
              {modeOption}
            </button>
          ))}
        </div>

        <div className="flex-1 flex">
          {/* Left Panel - Input and Examples */}
          <div className="w-1/2 p-4 border-r border-gray-700">
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-300 mb-2">
                {mode === 'command' && 'Natural Language Command'}
                {mode === 'mission' && 'Mission Objective'}
                {mode === 'analysis' && 'Analysis Request'}
              </label>
              <textarea
                ref={inputRef}
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder={`Enter your ${mode} here... (Ctrl+Enter to submit)`}
                className="w-full h-32 bg-gray-700 border border-gray-600 rounded px-3 py-2 text-white placeholder-gray-400 resize-none"
              />
            </div>

            <button
              onClick={handleSubmit}
              disabled={!input.trim() || isLoading}
              className="w-full bg-blue-600 hover:bg-blue-700 disabled:bg-gray-600 text-white px-4 py-2 rounded mb-4"
            >
              {isLoading ? 'Processing...' : `Generate ${mode.charAt(0).toUpperCase() + mode.slice(1)}`}
            </button>

            {/* Context Info */}
            <div className="bg-gray-900 rounded p-3 mb-4">
              <h4 className="text-sm font-semibold text-gray-300 mb-2">Current Context</h4>
              <div className="text-xs text-gray-400 space-y-1">
                <div>Active Agents: {activeAgents.length}</div>
                <div>Selected Agents: {selectedAgents.length}</div>
                <div>Connection: {webrtcStats?.connectionState || 'Unknown'}</div>
                {webrtcStats && (
                  <div>Latency: {Math.round(webrtcStats.latency || 0)}ms</div>
                )}
              </div>
            </div>

            {/* Example Prompts */}
            <div>
              <h4 className="text-sm font-semibold text-gray-300 mb-2">Example Prompts</h4>
              <div className="space-y-2">
                {getExamplePrompts().map((example, index) => (
                  <button
                    key={index}
                    onClick={() => setInput(example)}
                    className="w-full text-left text-xs text-gray-400 hover:text-white hover:bg-gray-700 p-2 rounded"
                  >
                    {example}
                  </button>
                ))}
              </div>
            </div>
          </div>

          {/* Right Panel - Output and History */}
          <div className="w-1/2 p-4 flex flex-col">
            {/* Current Output */}
            <div className="mb-4">
              <label className="block text-sm font-medium text-gray-300 mb-2">
                AI Response
              </label>
              <div className="bg-black rounded p-3 h-64 overflow-y-auto font-mono text-sm">
                {error && (
                  <div className="text-red-400 mb-2">Error: {error}</div>
                )}
                {output ? (
                  <pre className="text-green-400 whitespace-pre-wrap">{output}</pre>
                ) : (
                  <div className="text-gray-500">AI response will appear here...</div>
                )}
              </div>
            </div>

            {/* History */}
            <div className="flex-1">
              <h4 className="text-sm font-semibold text-gray-300 mb-2">Session History</h4>
              <div className="bg-gray-900 rounded p-3 h-full overflow-y-auto">
                {history.length === 0 ? (
                  <div className="text-gray-500 text-sm">No commands in history</div>
                ) : (
                  <div className="space-y-3">
                    {history.slice(-10).reverse().map((item, index) => (
                      <div key={index} className="border-b border-gray-700 pb-2 last:border-b-0">
                        <div className="text-xs text-gray-400 mb-1">
                          {new Date(item.timestamp).toLocaleTimeString()}
                        </div>
                        <div className="text-sm text-blue-300 mb-1">{item.input}</div>
                        <div className="text-xs text-gray-300">{item.output.slice(0, 100)}...</div>
                      </div>
                    ))}
                  </div>
                )}
              </div>
            </div>
          </div>
        </div>

        {/* Footer */}
        <div className="border-t border-gray-700 p-4">
          <div className="flex justify-between items-center text-sm text-gray-400">
            <div>
              Powered by GPT-4o • {activeAgents.length} agents connected
            </div>
            <div className="flex space-x-4">
              <kbd className="bg-gray-700 px-2 py-1 rounded text-xs">Ctrl+Enter</kbd>
              <span>to submit</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}