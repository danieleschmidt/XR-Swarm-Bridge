// Comprehensive input validation and sanitization for XR-Swarm-Bridge

import { errorHandler, ErrorCodes } from './errorHandler'

export interface ValidationResult {
  isValid: boolean
  errors: string[]
  sanitized?: any
}

export interface AgentPosition {
  x: number
  y: number
  z: number
}

export interface CommandPayload {
  type: string
  target?: string | string[]
  parameters?: Record<string, any>
  timeout?: number
}

class ValidationService {
  // General validation rules
  private readonly AGENT_ID_PATTERN = /^[a-zA-Z0-9_-]{1,32}$/
  private readonly MAX_STRING_LENGTH = 1000
  private readonly MAX_ARRAY_LENGTH = 1000
  private readonly MAX_OBJECT_DEPTH = 10
  
  // Position constraints
  private readonly MIN_POSITION = -10000
  private readonly MAX_POSITION = 10000
  private readonly MIN_ALTITUDE = -100
  private readonly MAX_ALTITUDE = 1000
  
  // Command constraints
  private readonly ALLOWED_COMMAND_TYPES = new Set([
    'takeoff', 'land', 'hover', 'navigate', 'search', 'formation_line',
    'formation_grid', 'formation_circle', 'return_to_base', 'emergency_stop',
    'follow', 'set_altitude', 'arm', 'disarm'
  ])
  
  private readonly MAX_WAYPOINTS = 100
  private readonly MAX_SEARCH_AREA = 10000 // square meters
  private readonly MIN_FORMATION_SPACING = 0.5 // meters
  private readonly MAX_FORMATION_SPACING = 50 // meters

  /**
   * Validate and sanitize agent ID
   */
  validateAgentId(id: any): ValidationResult {
    if (typeof id !== 'string') {
      return { isValid: false, errors: ['Agent ID must be a string'] }
    }
    
    if (!this.AGENT_ID_PATTERN.test(id)) {
      return { 
        isValid: false, 
        errors: ['Agent ID must contain only alphanumeric characters, underscores, and hyphens (max 32 chars)'] 
      }
    }
    
    return { isValid: true, errors: [], sanitized: id.trim() }
  }

  /**
   * Validate position coordinates
   */
  validatePosition(position: any): ValidationResult {
    if (!Array.isArray(position)) {
      return { isValid: false, errors: ['Position must be an array [x, y, z]'] }
    }
    
    if (position.length !== 3) {
      return { isValid: false, errors: ['Position must have exactly 3 coordinates [x, y, z]'] }
    }
    
    const errors: string[] = []
    const sanitized: AgentPosition = { x: 0, y: 0, z: 0 }
    
    // Validate X coordinate
    const x = this.sanitizeNumber(position[0])
    if (x === null || x < this.MIN_POSITION || x > this.MAX_POSITION) {
      errors.push(`X coordinate must be between ${this.MIN_POSITION} and ${this.MAX_POSITION}`)
    } else {
      sanitized.x = x
    }
    
    // Validate Y coordinate
    const y = this.sanitizeNumber(position[1])
    if (y === null || y < this.MIN_POSITION || y > this.MAX_POSITION) {
      errors.push(`Y coordinate must be between ${this.MIN_POSITION} and ${this.MAX_POSITION}`)
    } else {
      sanitized.y = y
    }
    
    // Validate Z coordinate (altitude)
    const z = this.sanitizeNumber(position[2])
    if (z === null || z < this.MIN_ALTITUDE || z > this.MAX_ALTITUDE) {
      errors.push(`Z coordinate (altitude) must be between ${this.MIN_ALTITUDE} and ${this.MAX_ALTITUDE}`)
    } else {
      sanitized.z = z
    }
    
    return {
      isValid: errors.length === 0,
      errors,
      sanitized: errors.length === 0 ? [sanitized.x, sanitized.y, sanitized.z] : undefined
    }
  }

  /**
   * Validate waypoint list
   */
  validateWaypoints(waypoints: any): ValidationResult {
    if (!Array.isArray(waypoints)) {
      return { isValid: false, errors: ['Waypoints must be an array'] }
    }
    
    if (waypoints.length === 0) {
      return { isValid: false, errors: ['At least one waypoint is required'] }
    }
    
    if (waypoints.length > this.MAX_WAYPOINTS) {
      return { isValid: false, errors: [`Maximum ${this.MAX_WAYPOINTS} waypoints allowed`] }
    }
    
    const errors: string[] = []
    const sanitized: number[][] = []
    
    waypoints.forEach((waypoint, index) => {
      const result = this.validatePosition(waypoint)
      if (!result.isValid) {
        errors.push(`Waypoint ${index + 1}: ${result.errors.join(', ')}`)
      } else if (result.sanitized) {
        sanitized.push(result.sanitized)
      }
    })
    
    return {
      isValid: errors.length === 0,
      errors,
      sanitized: errors.length === 0 ? sanitized : undefined
    }
  }

  /**
   * Validate command payload
   */
  validateCommand(command: any): ValidationResult {
    if (typeof command !== 'object' || command === null) {
      return { isValid: false, errors: ['Command must be an object'] }
    }
    
    const errors: string[] = []
    const sanitized: CommandPayload = {
      type: '',
      parameters: {}
    }
    
    // Validate command type
    if (typeof command.type !== 'string') {
      errors.push('Command type must be a string')
    } else if (!this.ALLOWED_COMMAND_TYPES.has(command.type)) {
      errors.push(`Invalid command type: ${command.type}`)
    } else {
      sanitized.type = command.type
    }
    
    // Validate target (optional)
    if (command.target !== undefined) {
      if (typeof command.target === 'string') {
        const targetResult = this.validateAgentId(command.target)
        if (!targetResult.isValid) {
          errors.push(`Invalid target: ${targetResult.errors.join(', ')}`)
        } else {
          sanitized.target = targetResult.sanitized
        }
      } else if (Array.isArray(command.target)) {
        const validTargets: string[] = []
        command.target.forEach((target: any, index: number) => {
          const targetResult = this.validateAgentId(target)
          if (!targetResult.isValid) {
            errors.push(`Invalid target ${index + 1}: ${targetResult.errors.join(', ')}`)
          } else {
            validTargets.push(targetResult.sanitized!)
          }
        })
        if (validTargets.length > 0) {
          sanitized.target = validTargets
        }
      } else {
        errors.push('Target must be a string or array of strings')
      }
    }
    
    // Validate parameters based on command type
    if (command.parameters) {
      const paramResult = this.validateCommandParameters(sanitized.type, command.parameters)
      if (!paramResult.isValid) {
        errors.push(...paramResult.errors)
      } else {
        sanitized.parameters = paramResult.sanitized
      }
    }
    
    // Validate timeout (optional)
    if (command.timeout !== undefined) {
      const timeout = this.sanitizeNumber(command.timeout)
      if (timeout === null || timeout < 0 || timeout > 300000) { // Max 5 minutes
        errors.push('Timeout must be between 0 and 300000 milliseconds')
      } else {
        sanitized.timeout = timeout
      }
    }
    
    return {
      isValid: errors.length === 0,
      errors,
      sanitized: errors.length === 0 ? sanitized : undefined
    }
  }

  /**
   * Validate command-specific parameters
   */
  private validateCommandParameters(commandType: string, parameters: any): ValidationResult {
    if (typeof parameters !== 'object' || parameters === null) {
      return { isValid: false, errors: ['Parameters must be an object'] }
    }
    
    const errors: string[] = []
    const sanitized: Record<string, any> = {}
    
    switch (commandType) {
      case 'takeoff':
        if (parameters.altitude !== undefined) {
          const altitude = this.sanitizeNumber(parameters.altitude)
          if (altitude === null || altitude < 0.5 || altitude > this.MAX_ALTITUDE) {
            errors.push(`Takeoff altitude must be between 0.5 and ${this.MAX_ALTITUDE} meters`)
          } else {
            sanitized.altitude = altitude
          }
        }
        break
        
      case 'navigate':
        if (parameters.position) {
          const posResult = this.validatePosition(parameters.position)
          if (!posResult.isValid) {
            errors.push(...posResult.errors)
          } else {
            sanitized.position = posResult.sanitized
          }
        }
        if (parameters.waypoints) {
          const waypointResult = this.validateWaypoints(parameters.waypoints)
          if (!waypointResult.isValid) {
            errors.push(...waypointResult.errors)
          } else {
            sanitized.waypoints = waypointResult.sanitized
          }
        }
        break
        
      case 'search':
        if (parameters.area) {
          const areaResult = this.validateSearchArea(parameters.area)
          if (!areaResult.isValid) {
            errors.push(...areaResult.errors)
          } else {
            sanitized.area = areaResult.sanitized
          }
        }
        if (parameters.pattern && typeof parameters.pattern === 'string') {
          const allowedPatterns = ['spiral', 'grid', 'perimeter', 'random']
          if (!allowedPatterns.includes(parameters.pattern)) {
            errors.push(`Search pattern must be one of: ${allowedPatterns.join(', ')}`)
          } else {
            sanitized.pattern = parameters.pattern
          }
        }
        break
        
      case 'formation_line':
      case 'formation_grid':
      case 'formation_circle':
        if (parameters.spacing !== undefined) {
          const spacing = this.sanitizeNumber(parameters.spacing)
          if (spacing === null || spacing < this.MIN_FORMATION_SPACING || spacing > this.MAX_FORMATION_SPACING) {
            errors.push(`Formation spacing must be between ${this.MIN_FORMATION_SPACING} and ${this.MAX_FORMATION_SPACING} meters`)
          } else {
            sanitized.spacing = spacing
          }
        }
        if (parameters.center) {
          const centerResult = this.validatePosition(parameters.center)
          if (!centerResult.isValid) {
            errors.push(...centerResult.errors)
          } else {
            sanitized.center = centerResult.sanitized
          }
        }
        break
        
      case 'set_altitude':
        if (parameters.altitude !== undefined) {
          const altitude = this.sanitizeNumber(parameters.altitude)
          if (altitude === null || altitude < this.MIN_ALTITUDE || altitude > this.MAX_ALTITUDE) {
            errors.push(`Altitude must be between ${this.MIN_ALTITUDE} and ${this.MAX_ALTITUDE} meters`)
          } else {
            sanitized.altitude = altitude
          }
        }
        break
    }
    
    return {
      isValid: errors.length === 0,
      errors,
      sanitized
    }
  }

  /**
   * Validate search area parameters
   */
  private validateSearchArea(area: any): ValidationResult {
    if (typeof area !== 'object' || area === null) {
      return { isValid: false, errors: ['Search area must be an object'] }
    }
    
    const errors: string[] = []
    const sanitized: any = {}
    
    // Validate center
    if (area.center) {
      const centerResult = this.validatePosition(area.center)
      if (!centerResult.isValid) {
        errors.push(`Search area center: ${centerResult.errors.join(', ')}`)
      } else {
        sanitized.center = centerResult.sanitized
      }
    }
    
    // Validate size
    if (area.size) {
      if (!Array.isArray(area.size) || area.size.length !== 2) {
        errors.push('Search area size must be [width, height]')
      } else {
        const width = this.sanitizeNumber(area.size[0])
        const height = this.sanitizeNumber(area.size[1])
        
        if (width === null || width <= 0 || width > Math.sqrt(this.MAX_SEARCH_AREA)) {
          errors.push(`Search area width must be between 0 and ${Math.sqrt(this.MAX_SEARCH_AREA)} meters`)
        }
        if (height === null || height <= 0 || height > Math.sqrt(this.MAX_SEARCH_AREA)) {
          errors.push(`Search area height must be between 0 and ${Math.sqrt(this.MAX_SEARCH_AREA)} meters`)
        }
        
        if (width && height && width * height > this.MAX_SEARCH_AREA) {
          errors.push(`Search area cannot exceed ${this.MAX_SEARCH_AREA} square meters`)
        }
        
        if (errors.length === 0) {
          sanitized.size = [width, height]
        }
      }
    }
    
    return {
      isValid: errors.length === 0,
      errors,
      sanitized
    }
  }

  /**
   * Validate GPT prompt input
   */
  validateGPTPrompt(prompt: any): ValidationResult {
    if (typeof prompt !== 'string') {
      return { isValid: false, errors: ['GPT prompt must be a string'] }
    }
    
    const trimmed = prompt.trim()
    
    if (trimmed.length === 0) {
      return { isValid: false, errors: ['GPT prompt cannot be empty'] }
    }
    
    if (trimmed.length > 4000) {
      return { isValid: false, errors: ['GPT prompt cannot exceed 4000 characters'] }
    }
    
    // Basic content filtering
    const forbiddenPatterns = [
      /\b(hack|exploit|attack|malware|virus)\b/i,
      /\b(password|credential|secret|token)\b/i
    ]
    
    for (const pattern of forbiddenPatterns) {
      if (pattern.test(trimmed)) {
        return { isValid: false, errors: ['GPT prompt contains potentially harmful content'] }
      }
    }
    
    return {
      isValid: true,
      errors: [],
      sanitized: this.sanitizeString(trimmed)
    }
  }

  /**
   * Sanitize and validate numeric input
   */
  private sanitizeNumber(value: any): number | null {
    if (typeof value === 'number' && !isNaN(value) && isFinite(value)) {
      return value
    }
    
    if (typeof value === 'string') {
      const parsed = parseFloat(value)
      if (!isNaN(parsed) && isFinite(parsed)) {
        return parsed
      }
    }
    
    return null
  }

  /**
   * Sanitize string input
   */
  private sanitizeString(value: string): string {
    return value
      .trim()
      .replace(/[\x01-\x1F\x7F]/g, '') // Remove control characters except null
      .replace(/[<>'"&]/g, (match) => { // Basic HTML entity encoding
        const entities: Record<string, string> = {
          '<': '&lt;',
          '>': '&gt;',
          '"': '&quot;',
          "'": '&#x27;',
          '&': '&amp;'
        }
        return entities[match] || match
      })
      .slice(0, this.MAX_STRING_LENGTH)
  }

  /**
   * Validate object depth to prevent deep nesting attacks
   */
  private validateObjectDepth(obj: any, maxDepth: number = this.MAX_OBJECT_DEPTH, currentDepth: number = 0): boolean {
    if (currentDepth > maxDepth) {
      return false
    }
    
    if (typeof obj === 'object' && obj !== null) {
      for (const key in obj) {
        if (Object.prototype.hasOwnProperty.call(obj, key)) {
          if (!this.validateObjectDepth(obj[key], maxDepth, currentDepth + 1)) {
            return false
          }
        }
      }
    }
    
    return true
  }

  /**
   * Comprehensive input sanitization
   */
  sanitizeInput(input: any): any {
    if (input === null || input === undefined) {
      return input
    }
    
    if (typeof input === 'string') {
      return this.sanitizeString(input)
    }
    
    if (typeof input === 'number') {
      return this.sanitizeNumber(input)
    }
    
    if (Array.isArray(input)) {
      if (input.length > this.MAX_ARRAY_LENGTH) {
        errorHandler.handleError({
          code: ErrorCodes.USER_INPUT_ERROR,
          message: `Array length exceeds maximum of ${this.MAX_ARRAY_LENGTH}`,
          component: 'validation'
        })
        return input.slice(0, this.MAX_ARRAY_LENGTH)
      }
      return input.map(item => this.sanitizeInput(item))
    }
    
    if (typeof input === 'object') {
      if (!this.validateObjectDepth(input)) {
        errorHandler.handleError({
          code: ErrorCodes.USER_INPUT_ERROR,
          message: `Object nesting exceeds maximum depth of ${this.MAX_OBJECT_DEPTH}`,
          component: 'validation'
        })
        return {}
      }
      
      const sanitized: any = {}
      for (const key in input) {
        if (Object.prototype.hasOwnProperty.call(input, key)) {
          const sanitizedKey = this.sanitizeString(key)
          sanitized[sanitizedKey] = this.sanitizeInput(input[key])
        }
      }
      return sanitized
    }
    
    return input
  }
}

// Singleton instance
export const validator = new ValidationService()

// React hook for validation
import { useCallback } from 'react'

export function useValidation() {
  const validateAgentId = useCallback((id: any) => validator.validateAgentId(id), [])
  const validatePosition = useCallback((position: any) => validator.validatePosition(position), [])
  const validateCommand = useCallback((command: any) => validator.validateCommand(command), [])
  const validateGPTPrompt = useCallback((prompt: any) => validator.validateGPTPrompt(prompt), [])
  const sanitizeInput = useCallback((input: any) => validator.sanitizeInput(input), [])

  return {
    validateAgentId,
    validatePosition,
    validateCommand,
    validateGPTPrompt,
    sanitizeInput
  }
}