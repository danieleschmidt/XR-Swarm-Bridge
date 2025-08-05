```tsx
import React, { useRef, useEffect } from 'react'
import { useSwarmStore, useActiveAgents, useSelectedAgents } from '../store/swarmStore'

export default function Minimap() {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const activeAgents = useActiveAgents()
  const selectedAgents = useSelectedAgents()
  const { viewMode } = useSwarmStore()

  const MAP_SIZE = 200 // Canvas size
  const WORLD_SIZE = 200 // World coordinate range (-100 to +100)
  const SCALE = MAP_SIZE / WORLD_SIZE

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const ctx = canvas.getContext('2d')
    if (!ctx) return

    // Clear canvas
    ctx.clearRect(0, 0, MAP_SIZE, MAP_SIZE)

    // Set up canvas
    ctx.fillStyle = '#1f2937' // Gray-800
    ctx.fillRect(0, 0, MAP_SIZE, MAP_SIZE)

    // Draw grid
    ctx.strokeStyle = '#374151' // Gray-700
    ctx.lineWidth = 1
    const gridSpacing = 20 * SCALE // 20m grid
    
    for (let i = 0; i <= MAP_SIZE; i += gridSpacing) {
      ctx.beginPath()
      ctx.moveTo(i, 0)
      ctx.lineTo(i, MAP_SIZE)
      ctx.stroke()
      
      ctx.beginPath()
      ctx.moveTo(0, i)
      ctx.lineTo(MAP_SIZE, i)
      ctx.stroke()
    }

    // Draw origin
    const centerX = MAP_SIZE / 2
    const centerY = MAP_SIZE / 2
    ctx.fillStyle = '#fbbf24' // Yellow-400
    ctx.beginPath()
    ctx.arc(centerX, centerY, 3, 0, 2 * Math.PI)
    ctx.fill()

    // Draw compass
    drawCompass(ctx, centerX + 70, centerY - 70)

    // Draw agents
    activeAgents.forEach(agent => {
      const isSelected = selectedAgents.map(a => a.id).includes(agent.id)
      
      // Convert world coordinates to canvas coordinates
      const x = centerX + (agent.position[0] * SCALE)
      const y = centerY - (agent.position[2] * SCALE) // Flip Y axis
      
      // Skip if outside bounds
      if (x < 0 || x > MAP_SIZE || y < 0 || y > MAP_SIZE) return

      // Agent color based on type and status
      let color = '#6b7280' // Gray-500 default
      switch (agent.type) {
        case 'drone':
          color = agent.status === 'active' ? '#10b981' : '#3b82f6' // Green or Blue
          break
        case 'ugv':
          color = agent.status === 'active' ? '#f59e0b' : '#8b5cf6' // Orange or Purple
          break
        default:
          color = agent.status === 'active' ? '#ef4444' : '#6b7280' // Red or Gray
      }

      // Draw agent
      ctx.fillStyle = color
      ctx.beginPath()
      
      if (agent.type === 'drone') {
        // Draw drone as diamond
        ctx.save()
        ctx.translate(x, y)
        ctx.rotate(Math.PI / 4) // 45 degrees
        ctx.fillRect(-4, -4, 8, 8)
        ctx.restore()
      } else {
        // Draw UGV/other as circle
        ctx.arc(x, y, 4, 0, 2 * Math.PI)
        ctx.fill()
      }

      // Draw selection indicator
      if (isSelected) {
        ctx.strokeStyle = '#fbbf24' // Yellow-400
        ctx.lineWidth = 2
        ctx.beginPath()
        ctx.arc(x, y, 8, 0, 2 * Math.PI)
        ctx.stroke()
      }

      // Draw agent ID
      ctx.fillStyle = '#ffffff'
      ctx.font = '8px monospace'
      ctx.textAlign = 'center'
      ctx.fillText(agent.id.slice(-2), x, y - 8)

      // Draw battery indicator
      const batteryWidth = 8
      const batteryHeight = 2
      const batteryLevel = agent.battery / 100
      
      ctx.fillStyle = agent.battery > 30 ? '#10b981' : '#ef4444'
      ctx.fillRect(x - batteryWidth/2, y + 6, batteryWidth * batteryLevel, batteryHeight)
      
      ctx.strokeStyle = '#ffffff'
      ctx.lineWidth = 1
      ctx.strokeRect(x - batteryWidth/2, y + 6, batteryWidth, batteryHeight)
    })

    // Draw scale indicator
    drawScale(ctx, 10, MAP_SIZE - 30)

  }, [activeAgents, selectedAgents])

  const drawCompass = (ctx: CanvasRenderingContext2D, x: number, y: number) => {
    const radius = 15

    // Background circle
    ctx.fillStyle = '#374151' // Gray-700
    ctx.beginPath()
    ctx.arc(x, y, radius, 0, 2 * Math.PI)
    ctx.fill()

    ctx.strokeStyle = '#9ca3af' // Gray-400
    ctx.lineWidth = 1
    ctx.stroke()

    // North arrow
    ctx.fillStyle = '#ef4444' // Red-500
    ctx.beginPath()
    ctx.moveTo(x, y - radius + 3)
    ctx.lineTo(x - 4, y - 3)
    ctx.lineTo(x + 4, y - 3)
    ctx.closePath()
    ctx.fill()

    // N label
    ctx.fillStyle = '#ffffff'
    ctx.font = '10px sans-serif'
    ctx.textAlign = 'center'
    ctx.fillText('N', x, y - radius - 5)
  }

  const drawScale = (ctx: CanvasRenderingContext2D, x: number, y: number) => {
    const scaleLength = 50 * SCALE // 50m in canvas pixels
    
    // Scale line
    ctx.strokeStyle = '#ffffff'
    ctx.lineWidth = 2
    ctx.beginPath()
    ctx.moveTo(x, y)
    ctx.lineTo(x + scaleLength, y)
    ctx.stroke()

    // Scale ticks
    ctx.beginPath()
    ctx.moveTo(x, y - 3)
    ctx.lineTo(x, y + 3)
    ctx.moveTo(x + scaleLength, y - 3)
    ctx.lineTo(x + scaleLength, y + 3)
    ctx.stroke()

    // Scale label
    ctx.fillStyle = '#ffffff'
    ctx.font = '10px sans-serif'
    ctx.textAlign = 'center'
    ctx.fillText('50m', x + scaleLength / 2, y + 15)
  }

  return (
    <div className="bg-black/80 backdrop-blur-sm border border-gray-700 rounded-lg p-3">
      <div className="flex items-center justify-between mb-2">
        <h4 className="text-sm font-medium text-white">Minimap</h4>
        <div className="text-xs text-gray-400">
          {activeAgents.length} robots
        </div>
      </div>
      
      <canvas
        ref={canvasRef}
        width={MAP_SIZE}
        height={MAP_SIZE}
        className="border border-gray-600 rounded"
      />

      <div className="mt-2 flex items-center justify-between text-xs text-gray-400">
        <div className="flex items-center space-x-3">
          <div className="flex items-center space-x-1">
            <div className="w-2 h-2 bg-green-500 rounded-full"></div>
            <span>Drone</span>
          </div>
          <div className="flex items-center space-x-1">
            <div className="w-2 h-2 bg-orange-500 rounded-full"></div>
            <span>UGV</span>
          </div>
        </div>
        <div className="text-yellow-400">
          {selectedAgents.length} selected
        </div>
      </div>
    </div>
  )
}
```
