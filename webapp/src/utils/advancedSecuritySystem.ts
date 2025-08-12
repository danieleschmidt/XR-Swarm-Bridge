/**
 * Advanced Security System for XR-Swarm-Bridge
 * Zero-trust security, behavioral analysis, and threat detection
 */

import { logger } from './logger'
import { performanceMonitor } from './performance'

interface SecurityEvent {
  type: 'authentication' | 'authorization' | 'suspicious_activity' | 'rate_limit' | 'injection_attempt'
  severity: 'low' | 'medium' | 'high' | 'critical'
  userId?: string
  ip?: string
  userAgent?: string
  details: Record<string, any>
  timestamp: number
}

interface BehavioralProfile {
  userId: string
  commandFrequency: number
  typicalCommands: Set<string>
  accessPatterns: Array<{ endpoint: string; count: number }>
  timePatterns: Array<{ hour: number; activity: number }>
  suspiciousScore: number
  lastActivity: number
}

interface RateLimit {
  requests: number[]
  limit: number
  windowMs: number
}

interface ThreatDetectionRule {
  name: string
  pattern: RegExp | ((input: string) => boolean)
  severity: SecurityEvent['severity']
  action: 'log' | 'block' | 'quarantine'
}

class AdvancedSecuritySystem {
  private behavioralProfiles = new Map<string, BehavioralProfile>()
  private rateLimits = new Map<string, RateLimit>()
  private securityEvents: SecurityEvent[] = []
  private blockedIPs = new Set<string>()
  private suspiciousPatterns = new Map<string, number>()
  private threatRules: ThreatDetectionRule[] = []
  
  constructor() {
    this.initializeThreatRules()
    this.startSecurityMonitoring()
  }

  /**
   * Authenticate user with zero-trust principles
   */
  async authenticateUser(token: string, context: any): Promise<{ valid: boolean; userId?: string; risk: number }> {
    const startTime = performance.now()
    
    try {
      // Basic token validation (in production, use proper JWT verification)
      if (!token || token.length < 16) {
        this.logSecurityEvent({
          type: 'authentication',
          severity: 'medium',
          details: { reason: 'invalid_token_format', tokenLength: token?.length || 0 },
          timestamp: Date.now()
        })
        return { valid: false, risk: 0.8 }
      }

      // Extract user ID from token (simplified)
      const userId = this.extractUserIdFromToken(token)
      
      // Check if user is blocked
      if (this.isUserBlocked(userId, context.ip)) {
        this.logSecurityEvent({
          type: 'authentication',
          severity: 'high',
          userId,
          ip: context.ip,
          details: { reason: 'blocked_user_attempt' },
          timestamp: Date.now()
        })
        return { valid: false, risk: 1.0 }
      }

      // Calculate risk score
      const riskScore = this.calculateAuthenticationRisk(userId, context)
      
      // Behavioral analysis
      this.updateBehavioralProfile(userId, 'authentication', context)
      
      this.logSecurityEvent({
        type: 'authentication',
        severity: riskScore > 0.7 ? 'high' : riskScore > 0.4 ? 'medium' : 'low',
        userId,
        ip: context.ip,
        userAgent: context.userAgent,
        details: { riskScore, success: true },
        timestamp: Date.now()
      })

      return { valid: true, userId, risk: riskScore }
    } catch (error) {
      logger.error('Authentication error', { error: error instanceof Error ? error.message : error })
      return { valid: false, risk: 1.0 }
    } finally {
      performanceMonitor.recordMetric('authentication_duration', performance.now() - startTime)
    }
  }

  /**
   * Authorize user action with contextual analysis
   */
  async authorizeAction(
    userId: string, 
    action: string, 
    resource: string, 
    context: any
  ): Promise<{ authorized: boolean; reason?: string }> {
    
    // Check rate limits
    const rateLimitKey = `${userId}:${action}`
    if (!this.checkRateLimit(rateLimitKey, this.getRateLimitForAction(action))) {
      this.logSecurityEvent({
        type: 'rate_limit',
        severity: 'medium',
        userId,
        ip: context.ip,
        details: { action, resource, reason: 'rate_limit_exceeded' },
        timestamp: Date.now()
      })
      return { authorized: false, reason: 'Rate limit exceeded' }
    }

    // Behavioral analysis
    const profile = this.behavioralProfiles.get(userId)
    if (profile) {
      const isTypicalAction = this.isTypicalUserAction(profile, action, context)
      if (!isTypicalAction && profile.suspiciousScore > 0.6) {
        this.logSecurityEvent({
          type: 'suspicious_activity',
          severity: 'high',
          userId,
          ip: context.ip,
          details: { 
            action, 
            resource, 
            suspiciousScore: profile.suspiciousScore,
            reason: 'atypical_behavior_with_high_suspicion' 
          },
          timestamp: Date.now()
        })
        return { authorized: false, reason: 'Suspicious activity detected' }
      }
    }

    // Check for high-risk actions
    if (this.isHighRiskAction(action)) {
      const additionalVerification = await this.performAdditionalVerification(userId, action, context)
      if (!additionalVerification) {
        return { authorized: false, reason: 'Additional verification failed' }
      }
    }

    // Update behavioral profile
    this.updateBehavioralProfile(userId, action, context)

    this.logSecurityEvent({
      type: 'authorization',
      severity: 'low',
      userId,
      ip: context.ip,
      details: { action, resource, authorized: true },
      timestamp: Date.now()
    })

    return { authorized: true }
  }

  /**
   * Validate and sanitize user input
   */
  validateInput(input: string, inputType: string): { valid: boolean; sanitized?: string; threats?: string[] } {
    const threats: string[] = []
    let sanitized = input

    // Apply threat detection rules
    for (const rule of this.threatRules) {
      let matches = false
      
      if (rule.pattern instanceof RegExp) {
        matches = rule.pattern.test(input)
      } else if (typeof rule.pattern === 'function') {
        matches = rule.pattern(input)
      }

      if (matches) {
        threats.push(rule.name)
        
        this.logSecurityEvent({
          type: 'injection_attempt',
          severity: rule.severity,
          details: { 
            rule: rule.name, 
            inputType, 
            inputSample: input.substring(0, 100),
            action: rule.action 
          },
          timestamp: Date.now()
        })

        if (rule.action === 'block') {
          return { valid: false, threats }
        }
      }
    }

    // Basic sanitization
    sanitized = this.sanitizeInput(sanitized, inputType)

    return { valid: true, sanitized, threats }
  }

  /**
   * Monitor and detect anomalous patterns
   */
  detectAnomalies(): Array<{ type: string; severity: string; details: any }> {
    const anomalies: Array<{ type: string; severity: string; details: any }> = []

    // Check for repeated failed authentication attempts
    const recentFailures = this.securityEvents
      .filter(event => 
        event.type === 'authentication' && 
        event.details.success === false &&
        Date.now() - event.timestamp < 300000 // Last 5 minutes
      )

    if (recentFailures.length > 10) {
      anomalies.push({
        type: 'authentication_flood',
        severity: 'high',
        details: { 
          count: recentFailures.length,
          uniqueIPs: new Set(recentFailures.map(f => f.ip)).size
        }
      })
    }

    // Check for suspicious behavioral patterns
    for (const [userId, profile] of this.behavioralProfiles.entries()) {
      if (profile.suspiciousScore > 0.8) {
        anomalies.push({
          type: 'suspicious_user_behavior',
          severity: 'high',
          details: {
            userId,
            suspiciousScore: profile.suspiciousScore,
            lastActivity: profile.lastActivity
          }
        })
      }
    }

    // Check for unusual command patterns
    const recentCommands = this.securityEvents
      .filter(event => 
        event.type === 'authorization' && 
        Date.now() - event.timestamp < 600000 // Last 10 minutes
      )
      .map(event => event.details.action)

    const commandCounts = recentCommands.reduce((acc, cmd) => {
      acc[cmd] = (acc[cmd] || 0) + 1
      return acc
    }, {} as Record<string, number>)

    for (const [command, count] of Object.entries(commandCounts)) {
      if (count > 50 && this.isHighRiskAction(command)) {
        anomalies.push({
          type: 'command_flood',
          severity: 'medium',
          details: { command, count }
        })
      }
    }

    return anomalies
  }

  /**
   * Initialize threat detection rules
   */
  private initializeThreatRules(): void {
    this.threatRules = [
      {
        name: 'sql_injection',
        pattern: /('|(\\')|(;)|(\\)|(--)|(%3B)|(%27)|(%22)|(%5C)|(%2D%2D))/i,
        severity: 'high',
        action: 'block'
      },
      {
        name: 'xss_attempt',
        pattern: /<script|javascript:|onload=|onerror=|eval\(|expression\(/i,
        severity: 'high',
        action: 'block'
      },
      {
        name: 'command_injection',
        pattern: /(\||;|&|`|\$\(|>\s*\/)/,
        severity: 'high',
        action: 'block'
      },
      {
        name: 'path_traversal',
        pattern: /(\.\.\/|\.\.\\|%2e%2e%2f|%2e%2e%5c)/i,
        severity: 'medium',
        action: 'block'
      },
      {
        name: 'excessive_length',
        pattern: (input: string) => input.length > 10000,
        severity: 'low',
        action: 'log'
      },
      {
        name: 'suspicious_chars',
        pattern: /[\x00-\x08\x0b\x0c\x0e-\x1f\x7f-\x9f]/,
        severity: 'medium',
        action: 'log'
      }
    ]
  }

  /**
   * Calculate authentication risk score
   */
  private calculateAuthenticationRisk(userId: string, context: any): number {
    let risk = 0.0

    // IP-based risk
    if (this.blockedIPs.has(context.ip)) {
      risk += 0.8
    }

    // Behavioral risk
    const profile = this.behavioralProfiles.get(userId)
    if (profile) {
      risk += profile.suspiciousScore * 0.3

      // Time-based risk (unusual hours)
      const hour = new Date().getHours()
      const isTypicalHour = profile.timePatterns.some(p => 
        Math.abs(p.hour - hour) <= 2 && p.activity > 5
      )
      if (!isTypicalHour) {
        risk += 0.2
      }
    } else {
      // New user - moderate risk
      risk += 0.3
    }

    // Geographic risk (simplified)
    if (context.country && this.isHighRiskCountry(context.country)) {
      risk += 0.4
    }

    // User agent risk
    if (this.isSuspiciousUserAgent(context.userAgent)) {
      risk += 0.3
    }

    return Math.min(risk, 1.0)
  }

  /**
   * Update behavioral profile for user
   */
  private updateBehavioralProfile(userId: string, action: string, context: any): void {
    let profile = this.behavioralProfiles.get(userId)
    
    if (!profile) {
      profile = {
        userId,
        commandFrequency: 0,
        typicalCommands: new Set(),
        accessPatterns: [],
        timePatterns: [],
        suspiciousScore: 0.0,
        lastActivity: Date.now()
      }
      this.behavioralProfiles.set(userId, profile)
    }

    // Update command frequency
    profile.commandFrequency++
    profile.typicalCommands.add(action)
    profile.lastActivity = Date.now()

    // Update time patterns
    const hour = new Date().getHours()
    const existingTimePattern = profile.timePatterns.find(p => p.hour === hour)
    if (existingTimePattern) {
      existingTimePattern.activity++
    } else {
      profile.timePatterns.push({ hour, activity: 1 })
    }

    // Update access patterns
    const endpoint = context.endpoint || 'unknown'
    const existingPattern = profile.accessPatterns.find(p => p.endpoint === endpoint)
    if (existingPattern) {
      existingPattern.count++
    } else {
      profile.accessPatterns.push({ endpoint, count: 1 })
    }

    // Calculate suspicion score
    profile.suspiciousScore = this.calculateSuspicionScore(profile, context)
  }

  /**
   * Check if action is typical for user
   */
  private isTypicalUserAction(profile: BehavioralProfile, action: string, context: any): boolean {
    // Check if command is in typical set
    if (!profile.typicalCommands.has(action)) {
      return false
    }

    // Check time pattern
    const hour = new Date().getHours()
    const hasTimePattern = profile.timePatterns.some(p => 
      Math.abs(p.hour - hour) <= 2 && p.activity > 2
    )

    return hasTimePattern
  }

  /**
   * Calculate user suspicion score
   */
  private calculateSuspicionScore(profile: BehavioralProfile, context: any): number {
    let suspicion = profile.suspiciousScore * 0.9 // Decay over time

    // High frequency in short time
    const timeSinceLastActivity = Date.now() - profile.lastActivity
    if (timeSinceLastActivity < 1000 && profile.commandFrequency > 10) {
      suspicion += 0.2
    }

    // Unusual time patterns
    const hour = new Date().getHours()
    const isNightTime = hour < 6 || hour > 22
    if (isNightTime) {
      suspicion += 0.1
    }

    // Diverse command patterns (might indicate automation)
    if (profile.typicalCommands.size > 20) {
      suspicion += 0.1
    }

    return Math.min(suspicion, 1.0)
  }

  /**
   * Check rate limits
   */
  private checkRateLimit(key: string, config: { limit: number; windowMs: number }): boolean {
    const now = Date.now()
    let rateLimit = this.rateLimits.get(key)

    if (!rateLimit) {
      rateLimit = { requests: [], limit: config.limit, windowMs: config.windowMs }
      this.rateLimits.set(key, rateLimit)
    }

    // Clean old requests outside window
    rateLimit.requests = rateLimit.requests.filter(timestamp => now - timestamp < rateLimit.windowMs)

    if (rateLimit.requests.length >= rateLimit.limit) {
      return false
    }

    rateLimit.requests.push(now)
    return true
  }

  /**
   * Get rate limit configuration for action
   */
  private getRateLimitForAction(action: string): { limit: number; windowMs: number } {
    const rateLimits: Record<string, { limit: number; windowMs: number }> = {
      'emergency_stop': { limit: 5, windowMs: 60000 }, // 5 per minute
      'gpt_command': { limit: 10, windowMs: 60000 }, // 10 per minute
      'robot_command': { limit: 100, windowMs: 60000 }, // 100 per minute
      'authentication': { limit: 20, windowMs: 300000 }, // 20 per 5 minutes
      'default': { limit: 60, windowMs: 60000 } // 60 per minute default
    }

    return rateLimits[action] || rateLimits['default']
  }

  /**
   * Utility methods
   */
  private extractUserIdFromToken(token: string): string {
    // Simplified token parsing - in production use proper JWT
    return `user_${token.substring(0, 8)}`
  }

  private isUserBlocked(userId: string, ip: string): boolean {
    return this.blockedIPs.has(ip) // Simplified blocking logic
  }

  private isHighRiskAction(action: string): boolean {
    const highRiskActions = [
      'emergency_stop',
      'system_shutdown',
      'delete_mission',
      'override_safety',
      'admin_access'
    ]
    return highRiskActions.includes(action)
  }

  private isHighRiskCountry(country: string): boolean {
    // This is a simplified example - real implementation would be more nuanced
    const highRiskCountries = ['XX', 'YY'] // Placeholder country codes
    return highRiskCountries.includes(country)
  }

  private isSuspiciousUserAgent(userAgent?: string): boolean {
    if (!userAgent) return true
    
    const suspiciousPatterns = [
      /bot/i,
      /crawler/i,
      /scanner/i,
      /automated/i
    ]
    
    return suspiciousPatterns.some(pattern => pattern.test(userAgent))
  }

  private sanitizeInput(input: string, inputType: string): string {
    let sanitized = input

    // Basic HTML entity encoding
    sanitized = sanitized
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#x27;')

    // Type-specific sanitization
    switch (inputType) {
      case 'command':
        // Remove potentially dangerous characters from commands
        sanitized = sanitized.replace(/[;&|`$]/g, '')
        break
      case 'filename':
        // Sanitize filenames
        sanitized = sanitized.replace(/[^a-zA-Z0-9._-]/g, '')
        break
    }

    return sanitized.trim().substring(0, 1000) // Limit length
  }

  private async performAdditionalVerification(
    userId: string, 
    action: string, 
    context: any
  ): Promise<boolean> {
    // Simulate additional verification (biometrics, 2FA, etc.)
    await new Promise(resolve => setTimeout(resolve, 100))
    
    // For high-risk actions, require additional verification
    // This would integrate with actual verification systems
    return Math.random() > 0.1 // 90% success rate for simulation
  }

  /**
   * Security event logging
   */
  private logSecurityEvent(event: SecurityEvent): void {
    this.securityEvents.push(event)
    
    // Keep only recent events (last 24 hours)
    const dayAgo = Date.now() - 86400000
    this.securityEvents = this.securityEvents.filter(e => e.timestamp > dayAgo)

    // Log to system
    logger.info('Security event', event)

    // Alert on critical events
    if (event.severity === 'critical') {
      this.triggerSecurityAlert(event)
    }
  }

  /**
   * Trigger security alerts
   */
  private triggerSecurityAlert(event: SecurityEvent): void {
    logger.error('CRITICAL SECURITY EVENT', event)
    
    // In production, would integrate with alerting systems
    if (window.postMessage) {
      window.postMessage({
        type: 'SECURITY_ALERT',
        data: event
      }, '*')
    }
  }

  /**
   * Periodic security monitoring
   */
  private startSecurityMonitoring(): void {
    setInterval(() => {
      const anomalies = this.detectAnomalies()
      if (anomalies.length > 0) {
        logger.warn('Security anomalies detected', { anomalies })
      }

      this.cleanupOldData()
    }, 60000) // Every minute
  }

  private cleanupOldData(): void {
    const now = Date.now()
    const maxAge = 3600000 // 1 hour

    // Cleanup old rate limits
    for (const [key, rateLimit] of this.rateLimits.entries()) {
      rateLimit.requests = rateLimit.requests.filter(timestamp => 
        now - timestamp < rateLimit.windowMs
      )
      
      if (rateLimit.requests.length === 0) {
        this.rateLimits.delete(key)
      }
    }

    // Cleanup old behavioral profiles
    for (const [userId, profile] of this.behavioralProfiles.entries()) {
      if (now - profile.lastActivity > maxAge * 24) { // 24 hours inactive
        this.behavioralProfiles.delete(userId)
      }
    }
  }

  /**
   * Get security status report
   */
  getSecurityReport(): any {
    const recentEvents = this.securityEvents.filter(e => 
      Date.now() - e.timestamp < 3600000 // Last hour
    )

    const eventsByType = recentEvents.reduce((acc, event) => {
      acc[event.type] = (acc[event.type] || 0) + 1
      return acc
    }, {} as Record<string, number>)

    return {
      recentEvents: recentEvents.length,
      eventsByType,
      activeProfiles: this.behavioralProfiles.size,
      blockedIPs: this.blockedIPs.size,
      rateLimits: this.rateLimits.size,
      timestamp: Date.now()
    }
  }
}

// Export singleton instance
export const advancedSecuritySystem = new AdvancedSecuritySystem()

export default advancedSecuritySystem