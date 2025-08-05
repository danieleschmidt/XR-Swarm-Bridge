/**
 * Security utilities for XR-Swarm-Bridge
 * Provides input sanitization, validation, and security measures
 */

import DOMPurify from 'dompurify'

// Security configuration
const SECURITY_CONFIG = {
  MAX_COMMAND_LENGTH: 1000,
  MAX_MISSION_NAME_LENGTH: 100,
  MAX_AGENT_ID_LENGTH: 50,
  ALLOWED_PROTOCOLS: ['ws:', 'wss:', 'http:', 'https:'],
  RATE_LIMIT_WINDOW: 60000, // 1 minute
  MAX_REQUESTS_PER_WINDOW: 100,
  SESSION_TIMEOUT: 3600000, // 1 hour
}

// Rate limiting store
const rateLimitStore = new Map<string, { count: number; resetTime: number }>()

/**
 * Sanitize HTML content to prevent XSS attacks
 */
export function sanitizeHtml(input: string): string {
  return DOMPurify.sanitize(input, {
    ALLOWED_TAGS: [], // No HTML tags allowed
    ALLOWED_ATTR: [],
    KEEP_CONTENT: true
  })
}

/**
 * Sanitize and validate command input
 */
export function sanitizeCommand(command: string): { isValid: boolean; sanitized: string; errors: string[] } {
  const errors: string[] = []
  
  // Length check
  if (command.length > SECURITY_CONFIG.MAX_COMMAND_LENGTH) {
    errors.push(`Command too long (max ${SECURITY_CONFIG.MAX_COMMAND_LENGTH} characters)`)
  }
  
  // Sanitize HTML
  const sanitized = sanitizeHtml(command.trim())
  
  // Check for potentially dangerous patterns
  const dangerousPatterns = [
    /javascript:/i,
    /data:/i,
    /vbscript:/i,
    /on\w+\s*=/i, // Event handlers
    /<script/i,
    /<iframe/i,
    /<object/i,
    /<embed/i,
    /eval\s*\(/i,
    /Function\s*\(/i,
    /setTimeout\s*\(/i,
    /setInterval\s*\(/i
  ]
  
  for (const pattern of dangerousPatterns) {
    if (pattern.test(sanitized)) {
      errors.push('Command contains potentially dangerous content')
      break
    }
  }
  
  // Check for SQL injection patterns
  const sqlPatterns = [
    /(\b(SELECT|INSERT|UPDATE|DELETE|DROP|CREATE|ALTER|EXEC|UNION)\b)/i,
    /('|(--|\/\*))/,
    /(;|\||&)/
  ]
  
  for (const pattern of sqlPatterns) {
    if (pattern.test(sanitized)) {
      errors.push('Command contains potentially harmful SQL patterns')
      break
    }
  }
  
  return {
    isValid: errors.length === 0,
    sanitized,
    errors
  }
}

/**
 * Validate agent ID format
 */
export function validateAgentId(agentId: string): { isValid: boolean; errors: string[] } {
  const errors: string[] = []
  
  if (!agentId || typeof agentId !== 'string') {
    errors.push('Agent ID is required and must be a string')
    return { isValid: false, errors }
  }
  
  if (agentId.length > SECURITY_CONFIG.MAX_AGENT_ID_LENGTH) {
    errors.push(`Agent ID too long (max ${SECURITY_CONFIG.MAX_AGENT_ID_LENGTH} characters)`)
  }
  
  // Allow only alphanumeric characters, underscores, and hyphens
  if (!/^[a-zA-Z0-9_-]+$/.test(agentId)) {
    errors.push('Agent ID can only contain letters, numbers, underscores, and hyphens')
  }
  
  // Must start with a letter or number
  if (!/^[a-zA-Z0-9]/.test(agentId)) {
    errors.push('Agent ID must start with a letter or number')
  }
  
  return {
    isValid: errors.length === 0,
    errors
  }
}

/**
 * Validate mission data
 */
export function validateMission(mission: any): { isValid: boolean; sanitized: any; errors: string[] } {
  const errors: string[] = []
  
  if (!mission || typeof mission !== 'object') {
    errors.push('Mission data is required and must be an object')
    return { isValid: false, sanitized: null, errors }
  }
  
  const sanitized = { ...mission }
  
  // Validate mission name
  if (!mission.name || typeof mission.name !== 'string') {
    errors.push('Mission name is required')
  } else {
    if (mission.name.length > SECURITY_CONFIG.MAX_MISSION_NAME_LENGTH) {
      errors.push(`Mission name too long (max ${SECURITY_CONFIG.MAX_MISSION_NAME_LENGTH} characters)`)
    }
    sanitized.name = sanitizeHtml(mission.name.trim())
  }
  
  // Validate description
  if (mission.description && typeof mission.description === 'string') {
    sanitized.description = sanitizeHtml(mission.description.trim())
  }
  
  // Validate phases
  if (mission.phases && Array.isArray(mission.phases)) {
    sanitized.phases = mission.phases.map((phase: any) => ({
      ...phase,
      description: phase.description ? sanitizeHtml(phase.description) : phase.description
    }))
  }
  
  return {
    isValid: errors.length === 0,
    sanitized,
    errors
  }
}

/**
 * Validate URL for WebSocket/WebRTC connections
 */
export function validateConnectionUrl(url: string): { isValid: boolean; errors: string[] } {
  const errors: string[] = []
  
  if (!url || typeof url !== 'string') {
    errors.push('URL is required and must be a string')
    return { isValid: false, errors }
  }
  
  try {
    const parsed = new URL(url)
    
    // Check protocol
    if (!SECURITY_CONFIG.ALLOWED_PROTOCOLS.includes(parsed.protocol)) {
      errors.push(`Protocol ${parsed.protocol} is not allowed`)
    }
    
    // Prevent localhost access in production
    if (process.env.NODE_ENV === 'production' && (
      parsed.hostname === 'localhost' || 
      parsed.hostname === '127.0.0.1' ||
      parsed.hostname.startsWith('192.168.') ||
      parsed.hostname.startsWith('10.') ||
      parsed.hostname.startsWith('172.')
    )) {
      errors.push('Local/private IP addresses are not allowed in production')
    }
    
    // Validate port range
    if (parsed.port) {
      const port = parseInt(parsed.port, 10)
      if (port < 1 || port > 65535) {
        errors.push('Invalid port number')
      }
    }
    
  } catch (error) {
    errors.push('Invalid URL format')
  }
  
  return {
    isValid: errors.length === 0,
    errors
  }
}

/**
 * Rate limiting check
 */
export function checkRateLimit(identifier: string): { allowed: boolean; remaining: number; resetTime: number } {
  const now = Date.now()
  const key = identifier
  const existing = rateLimitStore.get(key)
  
  if (!existing || now > existing.resetTime) {
    // New window or expired window
    const resetTime = now + SECURITY_CONFIG.RATE_LIMIT_WINDOW
    rateLimitStore.set(key, { count: 1, resetTime })
    return {
      allowed: true,
      remaining: SECURITY_CONFIG.MAX_REQUESTS_PER_WINDOW - 1,
      resetTime
    }
  }
  
  if (existing.count >= SECURITY_CONFIG.MAX_REQUESTS_PER_WINDOW) {
    // Rate limit exceeded
    return {
      allowed: false,
      remaining: 0,
      resetTime: existing.resetTime
    }
  }
  
  // Increment count
  existing.count++
  rateLimitStore.set(key, existing)
  
  return {
    allowed: true,
    remaining: SECURITY_CONFIG.MAX_REQUESTS_PER_WINDOW - existing.count,
    resetTime: existing.resetTime
  }
}

/**
 * Generate secure session token
 */
export function generateSessionToken(): string {
  const array = new Uint8Array(32)
  crypto.getRandomValues(array)
  return Array.from(array, byte => byte.toString(16).padStart(2, '0')).join('')
}

/**
 * Validate session token format
 */
export function validateSessionToken(token: string): boolean {
  if (!token || typeof token !== 'string') {
    return false
  }
  
  // Should be 64 character hex string
  return /^[a-f0-9]{64}$/i.test(token)
}

/**
 * Content Security Policy headers
 */
export const CSP_DIRECTIVES = {
  'default-src': ["'self'"],
  'script-src': ["'self'", "'unsafe-eval'"], // unsafe-eval needed for WebRTC
  'style-src': ["'self'", "'unsafe-inline'"], // unsafe-inline needed for dynamic styles
  'img-src': ["'self'", 'data:', 'blob:'],
  'media-src': ["'self'", 'blob:'],
  'connect-src': ["'self'", 'ws:', 'wss:', 'https:'],
  'font-src': ["'self'"],
  'object-src': ["'none'"],
  'base-uri': ["'self'"],
  'form-action': ["'self'"],
  'frame-ancestors': ["'none'"],
  'upgrade-insecure-requests': []
}

/**
 * Generate CSP header string
 */
export function generateCSPHeader(): string {
  return Object.entries(CSP_DIRECTIVES)
    .map(([directive, values]) => `${directive} ${values.join(' ')}`)
    .join('; ')
}

/**
 * Validate telemetry data to prevent injection
 */
export function validateTelemetryData(data: any): { isValid: boolean; sanitized: any; errors: string[] } {
  const errors: string[] = []
  
  if (!data || typeof data !== 'object') {
    errors.push('Telemetry data must be an object')
    return { isValid: false, sanitized: null, errors }
  }
  
  const sanitized: any = {}
  
  // Validate agent_id
  if (data.agent_id) {
    const agentValidation = validateAgentId(data.agent_id)
    if (!agentValidation.isValid) {
      errors.push(...agentValidation.errors)
    } else {
      sanitized.agent_id = data.agent_id
    }
  }
  
  // Validate numeric fields
  const numericFields = ['battery_level', 'latitude', 'longitude', 'altitude', 'speed', 'heading']
  for (const field of numericFields) {
    if (data[field] !== undefined) {
      const value = parseFloat(data[field])
      if (isNaN(value)) {
        errors.push(`${field} must be a valid number`)
      } else {
        sanitized[field] = value
      }
    }
  }
  
  // Validate position array
  if (data.position && Array.isArray(data.position)) {
    if (data.position.length !== 3) {
      errors.push('Position must be an array of 3 numbers [x, y, z]')
    } else {
      const position = data.position.map(parseFloat)
      if (position.some(isNaN)) {
        errors.push('Position coordinates must be valid numbers')
      } else {
        sanitized.position = position
      }
    }
  }
  
  // Validate status
  const validStatuses = ['idle', 'active', 'error', 'emergency_stop', 'timeout', 'low_battery']
  if (data.status && !validStatuses.includes(data.status)) {
    errors.push(`Invalid status. Must be one of: ${validStatuses.join(', ')}`)
  } else if (data.status) {
    sanitized.status = data.status
  }
  
  // Validate timestamp
  if (data.timestamp) {
    const timestamp = parseInt(data.timestamp, 10)
    if (isNaN(timestamp) || timestamp < 0) {
      errors.push('Timestamp must be a valid positive integer')
    } else {
      sanitized.timestamp = timestamp
    }
  } else {
    sanitized.timestamp = Date.now()
  }
  
  return {
    isValid: errors.length === 0,
    sanitized,
    errors
  }
}

/**
 * Security audit log entry
 */
export interface SecurityEvent {
  type: 'validation_error' | 'rate_limit_exceeded' | 'suspicious_activity' | 'unauthorized_access'
  severity: 'low' | 'medium' | 'high' | 'critical'
  message: string
  data?: any
  timestamp: number
  userAgent?: string
  ip?: string
}

/**
 * Log security events (in production, this would send to a security monitoring service)
 */
export function logSecurityEvent(event: Omit<SecurityEvent, 'timestamp'>): void {
  const fullEvent: SecurityEvent = {
    ...event,
    timestamp: Date.now()
  }
  
  // In development, log to console
  if (process.env.NODE_ENV === 'development') {
    console.warn('Security Event:', fullEvent)
  }
  
  // In production, would send to security monitoring service
  // Example: sendToSecurityService(fullEvent)
}

/**
 * Validate and sanitize position coordinates
 */
export function validatePosition(position: any): { isValid: boolean; sanitized: [number, number, number]; errors: string[] } {
  const errors: string[] = []
  
  if (!Array.isArray(position)) {
    errors.push('Position must be an array')
    return { isValid: false, sanitized: [0, 0, 0], errors }
  }
  
  if (position.length !== 3) {
    errors.push('Position must have exactly 3 coordinates [x, y, z]')
    return { isValid: false, sanitized: [0, 0, 0], errors }
  }
  
  const [x, y, z] = position.map(Number)
  
  if ([x, y, z].some(coord => isNaN(coord) || !isFinite(coord))) {
    errors.push('All position coordinates must be valid finite numbers')
    return { isValid: false, sanitized: [0, 0, 0], errors }
  }
  
  // Reasonable bounds check (adjust based on your operational area)
  const MAX_COORDINATE = 100000 // 100km
  if (Math.abs(x) > MAX_COORDINATE || Math.abs(y) > MAX_COORDINATE || Math.abs(z) > MAX_COORDINATE) {
    errors.push(`Position coordinates must be within ±${MAX_COORDINATE} units`)
  }
  
  return {
    isValid: errors.length === 0,
    sanitized: [x, y, z],
    errors
  }
}

/**
 * Cleanup expired rate limit entries
 */
export function cleanupRateLimit(): number {
  const now = Date.now()
  let cleaned = 0
  
  for (const [key, data] of rateLimitStore.entries()) {
    if (now > data.resetTime) {
      rateLimitStore.delete(key)
      cleaned++
    }
  }
  
  return cleaned
}

// Periodic cleanup of rate limit store
if (typeof window !== 'undefined') {
  setInterval(cleanupRateLimit, SECURITY_CONFIG.RATE_LIMIT_WINDOW)
}