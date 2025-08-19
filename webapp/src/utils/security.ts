// Security configuration
export const SECURITY_CONFIG = {
  MAX_COMMAND_LENGTH: 1000,
  MAX_WAYPOINTS: 100,
  MAX_AGENTS_PER_COMMAND: 50,
  ALLOWED_COMMAND_TYPES: [
    'navigate', 'takeoff', 'land', 'hover', 'return_to_base',
    'emergency_stop', 'formation', 'search', 'arm', 'disarm'
  ],
  ALLOWED_FORMATION_TYPES: ['line', 'grid', 'circle', 'diamond', 'triangle'],
  ALLOWED_SEARCH_PATTERNS: ['spiral', 'grid', 'perimeter', 'random'],
  MAX_POSITION_COORDINATE: 1000,
  MIN_POSITION_COORDINATE: -1000,
  MAX_ALTITUDE: 150, // meters
  MIN_ALTITUDE: 0
} as const

// Security validation functions
export class SecurityValidator {
  
  static validateAgentCommand(command: any): { valid: boolean; error?: string; data?: any } {
    try {
      // Validate command type
      if (!command.type || !SECURITY_CONFIG.ALLOWED_COMMAND_TYPES.includes(command.type)) {
        return { valid: false, error: 'Invalid or missing command type' }
      }
      
      // Validate agent ID
      if (!command.agent_id || typeof command.agent_id !== 'string') {
        return { valid: false, error: 'Invalid or missing agent_id' }
      }
      
      if (!/^[a-zA-Z0-9_-]+$/.test(command.agent_id) || command.agent_id.length > 50) {
        return { valid: false, error: 'Invalid agent_id format' }
      }
      
      // Validate position if provided
      if (command.position && !this.validatePosition(command.position)) {
        return { valid: false, error: 'Invalid position coordinates' }
      }
      
      // Additional business logic validation
      if (command.type === 'navigate' && !command.position && !command.waypoints) {
        return { valid: false, error: 'Navigate command requires position or waypoints' }
      }
      
      if (command.type === 'takeoff' && command.altitude && command.altitude > 50) {
        return { valid: false, error: 'Takeoff altitude exceeds safety limit (50m)' }
      }
      
      return { valid: true, data: command }
    } catch (error) {
      return { valid: false, error: 'Command validation failed' }
    }
  }
  
  static validateSwarmCommand(command: any): { valid: boolean; error?: string; data?: any } {
    try {
      if (!command.commands || !Array.isArray(command.commands)) {
        return { valid: false, error: 'Missing or invalid commands array' }
      }
      
      if (command.commands.length > SECURITY_CONFIG.MAX_AGENTS_PER_COMMAND) {
        return { valid: false, error: `Too many commands (max: ${SECURITY_CONFIG.MAX_AGENTS_PER_COMMAND})` }
      }
      
      // Validate each command
      for (const cmd of command.commands) {
        const validation = this.validateAgentCommand(cmd)
        if (!validation.valid) {
          return validation
        }
      }
      
      // Check for conflicting commands
      const emergencyStopCommands = command.commands.filter((c: any) => c.type === 'emergency_stop')
      if (emergencyStopCommands.length > 0 && command.commands.length > emergencyStopCommands.length) {
        return { valid: false, error: 'Emergency stop commands cannot be mixed with other commands' }
      }
      
      return { valid: true, data: command }
    } catch (error) {
      return { valid: false, error: 'Swarm command validation failed' }
    }
  }
  
  static validateGPTRequest(request: any): { valid: boolean; error?: string; data?: any } {
    try {
      if (!request.prompt || typeof request.prompt !== 'string') {
        return { valid: false, error: 'Missing or invalid prompt' }
      }
      
      if (request.prompt.length > SECURITY_CONFIG.MAX_COMMAND_LENGTH) {
        return { valid: false, error: 'Prompt too long' }
      }
      
      // Check for potential injection attempts
      const suspiciousPatterns = [
        /system\s*:/i,
        /ignore\s+previous/i,
        /forget\s+instructions/i,
        /<script/i,
        /javascript:/i,
        /eval\(/i,
        /function\s*\(/i
      ]
      
      for (const pattern of suspiciousPatterns) {
        if (pattern.test(request.prompt)) {
          return { valid: false, error: 'Prompt contains suspicious patterns' }
        }
      }
      
      return { valid: true, data: request }
    } catch (error) {
      return { valid: false, error: 'GPT request validation failed' }
    }
  }
  
  private static validatePosition(position: any): boolean {
    if (!Array.isArray(position) || position.length < 2) {
      return false
    }
    
    const [x, y, z = 0] = position
    
    return (
      typeof x === 'number' && isFinite(x) &&
      typeof y === 'number' && isFinite(y) &&
      typeof z === 'number' && isFinite(z) &&
      x >= SECURITY_CONFIG.MIN_POSITION_COORDINATE &&
      x <= SECURITY_CONFIG.MAX_POSITION_COORDINATE &&
      y >= SECURITY_CONFIG.MIN_POSITION_COORDINATE &&
      y <= SECURITY_CONFIG.MAX_POSITION_COORDINATE &&
      z >= SECURITY_CONFIG.MIN_ALTITUDE &&
      z <= SECURITY_CONFIG.MAX_ALTITUDE
    )
  }
}

// Input sanitization
export class InputSanitizer {
  
  static sanitizeString(input: string, maxLength: number = 255): string {
    if (typeof input !== 'string') {
      return ''
    }
    
    return input
      .trim()
      .substring(0, maxLength)
      .replace(/[<>\"']/g, '') // Remove potentially dangerous characters
      .replace(/\s+/g, ' ') // Normalize whitespace
  }
  
  static sanitizeNumber(input: unknown, min?: number, max?: number): number | null {
    const num = Number(input)
    
    if (isNaN(num) || !isFinite(num)) {
      return null
    }
    
    if (min !== undefined && num < min) {
      return min
    }
    
    if (max !== undefined && num > max) {
      return max
    }
    
    return num
  }
  
  static sanitizePosition(position: unknown): [number, number, number] | null {
    if (!Array.isArray(position) || position.length < 2) {
      return null
    }
    
    const [x, y, z = 0] = position
    
    const sanitizedX = this.sanitizeNumber(x, SECURITY_CONFIG.MIN_POSITION_COORDINATE, SECURITY_CONFIG.MAX_POSITION_COORDINATE)
    const sanitizedY = this.sanitizeNumber(y, SECURITY_CONFIG.MIN_POSITION_COORDINATE, SECURITY_CONFIG.MAX_POSITION_COORDINATE)
    const sanitizedZ = this.sanitizeNumber(z, SECURITY_CONFIG.MIN_ALTITUDE, SECURITY_CONFIG.MAX_ALTITUDE)
    
    if (sanitizedX === null || sanitizedY === null || sanitizedZ === null) {
      return null
    }
    
    return [sanitizedX, sanitizedY, sanitizedZ]
  }
  
  static sanitizeAgentId(agentId: unknown): string | null {
    if (typeof agentId !== 'string') {
      return null
    }
    
    const sanitized = agentId.replace(/[^a-zA-Z0-9_-]/g, '').substring(0, 50)
    
    return sanitized.length > 0 ? sanitized : null
  }
}

// Rate limiting
export class RateLimiter {
  private requests: Map<string, number[]> = new Map()
  
  constructor(
    private maxRequests: number = 10,
    private windowMs: number = 60000, // 1 minute
    private cleanupIntervalMs: number = 300000 // 5 minutes
  ) {
    // Periodically clean up old entries
    setInterval(() => this.cleanup(), cleanupIntervalMs)
  }
  
  isAllowed(identifier: string): boolean {
    const now = Date.now()
    const windowStart = now - this.windowMs
    
    if (!this.requests.has(identifier)) {
      this.requests.set(identifier, [])
    }
    
    const userRequests = this.requests.get(identifier)!
    
    // Remove old requests outside the window
    const recentRequests = userRequests.filter(timestamp => timestamp > windowStart)
    this.requests.set(identifier, recentRequests)
    
    if (recentRequests.length >= this.maxRequests) {
      return false
    }
    
    // Add current request
    recentRequests.push(now)
    
    return true
  }
  
  getRemainingRequests(identifier: string): number {
    const now = Date.now()
    const windowStart = now - this.windowMs
    
    if (!this.requests.has(identifier)) {
      return this.maxRequests
    }
    
    const recentRequests = this.requests.get(identifier)!
      .filter(timestamp => timestamp > windowStart)
    
    return Math.max(0, this.maxRequests - recentRequests.length)
  }
  
  private cleanup() {
    const now = Date.now()
    const windowStart = now - this.windowMs
    
    for (const [identifier, timestamps] of this.requests.entries()) {
      const recentRequests = timestamps.filter(timestamp => timestamp > windowStart)
      
      if (recentRequests.length === 0) {
        this.requests.delete(identifier)
      } else {
        this.requests.set(identifier, recentRequests)
      }
    }
  }
}

// Security event logging
export interface SecurityEvent {
  type: 'validation_failure' | 'rate_limit_exceeded' | 'suspicious_activity' | 'unauthorized_access'
  severity: 'low' | 'medium' | 'high' | 'critical'
  message: string
  timestamp: number
  userId?: string
  sessionId?: string
  data?: any
}

export class SecurityLogger {
  private static events: SecurityEvent[] = []
  private static maxEvents = 1000
  
  static logEvent(event: Omit<SecurityEvent, 'timestamp'>) {
    const fullEvent: SecurityEvent = {
      ...event,
      timestamp: Date.now()
    }
    
    this.events.push(fullEvent)
    
    // Keep only recent events
    if (this.events.length > this.maxEvents) {
      this.events = this.events.slice(-this.maxEvents)
    }
    
    // Log to console in development
    if (import.meta.env?.DEV) {
      console.warn('Security Event:', fullEvent)
    }
    
    // Send to monitoring service in production
    if (import.meta.env?.PROD && event.severity === 'critical') {
      this.reportCriticalEvent(fullEvent)
    }
  }
  
  private static reportCriticalEvent(event: SecurityEvent) {
    // Send to security monitoring service
    fetch('/api/security/events', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(event)
    }).catch(error => console.error('Failed to report security event:', error))
  }
  
  static getRecentEvents(limit: number = 50): SecurityEvent[] {
    return this.events.slice(-limit)
  }
  
  static getEventsByType(type: SecurityEvent['type']): SecurityEvent[] {
    return this.events.filter(event => event.type === type)
  }
}

// Singleton instances
export const commandRateLimiter = new RateLimiter(20, 60000) // 20 commands per minute
export const gptRateLimiter = new RateLimiter(5, 60000) // 5 GPT requests per minute
export const connectionRateLimiter = new RateLimiter(100, 60000) // 100 connections per minute

export default {
  SecurityValidator,
  InputSanitizer,
  RateLimiter,
  SecurityLogger,
  SECURITY_CONFIG,
  commandRateLimiter,
  gptRateLimiter,
  connectionRateLimiter
}