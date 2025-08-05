// Security and authentication utilities for XR-Swarm-Bridge

import { errorHandler, ErrorCodes } from './errorHandler'
import { logger } from './logger'

export enum UserRole {
  VIEWER = 'viewer',
  OPERATOR = 'operator',
  COMMANDER = 'commander',
  ADMIN = 'admin'
}

export interface User {
  id: string
  username: string
  email: string
  role: UserRole
  permissions: string[]
  sessionId: string
  lastActivity: number
  isAuthenticated: boolean
}

export interface SecurityConfig {
  sessionTimeout: number // milliseconds
  maxFailedAttempts: number
  lockoutDuration: number // milliseconds
  requireStrongPasswords: boolean
  enableTwoFactor: boolean
  allowedOrigins: string[]
  rateLimitRequests: number // per minute
}

class SecurityManager {
  private currentUser: User | null = null
  private securityConfig: SecurityConfig
  private failedAttempts: Map<string, { count: number; lastAttempt: number }> = new Map()
  private rateLimitTracker: Map<string, number[]> = new Map()
  private sessionTimeoutId: NodeJS.Timeout | null = null

  constructor() {
    this.securityConfig = {
      sessionTimeout: 3600000, // 1 hour
      maxFailedAttempts: 5,
      lockoutDuration: 900000, // 15 minutes
      requireStrongPasswords: true,
      enableTwoFactor: false,
      allowedOrigins: ['https://localhost:3000', 'https://127.0.0.1:3000'],
      rateLimitRequests: 100 // per minute
    }

    this.setupSecurityHeaders()
    this.setupCSRFProtection()
    this.startSecurityMonitoring()
  }

  private setupSecurityHeaders() {
    // Add security headers to prevent common attacks
    const metaElements = [
      { name: 'X-Content-Type-Options', content: 'nosniff' },
      { name: 'X-Frame-Options', content: 'DENY' },
      { name: 'X-XSS-Protection', content: '1; mode=block' },
      { name: 'Referrer-Policy', content: 'strict-origin-when-cross-origin' },
      { name: 'Permissions-Policy', content: 'geolocation=(), microphone=(), camera=()' }
    ]

    metaElements.forEach(({ name, content }) => {
      const meta = document.createElement('meta')
      meta.httpEquiv = name
      meta.content = content
      document.head.appendChild(meta)
    })
  }

  private setupCSRFProtection() {
    // Generate CSRF token
    const csrfToken = this.generateSecureToken()
    sessionStorage.setItem('csrf_token', csrfToken)
    
    // Add CSRF token to all fetch requests
    const originalFetch = window.fetch
    window.fetch = function(...args: Parameters<typeof fetch>) {
      const [url, options = {}] = args
      
      if (options.method && ['POST', 'PUT', 'PATCH', 'DELETE'].includes(options.method.toUpperCase())) {
        options.headers = {
          ...options.headers,
          'X-CSRF-Token': csrfToken
        }
      }
      
      return originalFetch(url, options)
    }
  }

  private startSecurityMonitoring() {
    // Monitor for suspicious activity
    setInterval(() => {
      this.checkForSuspiciousActivity()
      this.cleanupExpiredSessions()
      this.updateRateLimits()
    }, 60000) // Every minute
  }

  private checkForSuspiciousActivity() {
    // Check for unusual patterns that might indicate an attack
    const currentTime = Date.now()
    const suspiciousPatterns = []

    // Check for rapid-fire requests
    for (const [identifier, timestamps] of this.rateLimitTracker) {
      const recentRequests = timestamps.filter(time => currentTime - time < 60000)
      if (recentRequests.length > this.securityConfig.rateLimitRequests * 2) {
        suspiciousPatterns.push(`High request rate from ${identifier}`)
      }
    }

    // Check for failed authentication attempts
    for (const [identifier, attempts] of this.failedAttempts) {
      if (attempts.count > this.securityConfig.maxFailedAttempts * 2) {
        suspiciousPatterns.push(`Excessive failed logins from ${identifier}`)
      }
    }

    if (suspiciousPatterns.length > 0) {
      logger.warn('Suspicious activity detected', 'security', { patterns: suspiciousPatterns })
      this.triggerSecurityAlert('SUSPICIOUS_ACTIVITY', { patterns: suspiciousPatterns })
    }
  }

  private cleanupExpiredSessions() {
    const currentTime = Date.now()
    
    // Clean up failed attempts
    for (const [identifier, attempts] of this.failedAttempts) {
      if (currentTime - attempts.lastAttempt > this.securityConfig.lockoutDuration) {
        this.failedAttempts.delete(identifier)
      }
    }
  }

  private updateRateLimits() {
    const currentTime = Date.now()
    const oneMinuteAgo = currentTime - 60000

    // Clean up old timestamps
    for (const [identifier, timestamps] of this.rateLimitTracker) {
      const recentTimestamps = timestamps.filter(time => time > oneMinuteAgo)
      if (recentTimestamps.length === 0) {
        this.rateLimitTracker.delete(identifier)
      } else {
        this.rateLimitTracker.set(identifier, recentTimestamps)
      }
    }
  }

  async authenticate(credentials: { username: string; password: string; totpCode?: string }): Promise<{ success: boolean; user?: User; error?: string }> {
    const { username, password, totpCode } = credentials
    const clientIdentifier = this.getClientIdentifier()

    try {
      // Check rate limiting
      if (!this.checkRateLimit(clientIdentifier)) {
        logger.warn('Rate limit exceeded for authentication', 'security', { username })
        return { success: false, error: 'Too many requests. Please try again later.' }
      }

      // Check if account is locked
      if (this.isAccountLocked(clientIdentifier)) {
        logger.warn('Authentication attempt on locked account', 'security', { username })
        return { success: false, error: 'Account temporarily locked due to failed attempts.' }
      }

      // Validate input
      if (!this.validateCredentials(username, password)) {
        this.recordFailedAttempt(clientIdentifier)
        return { success: false, error: 'Invalid credentials.' }
      }

      // Simulate authentication (in real implementation, this would call an API)
      const authResult = await this.performAuthentication(username, password, totpCode)
      
      if (authResult.success && authResult.user) {
        // Reset failed attempts on successful login
        this.failedAttempts.delete(clientIdentifier)
        
        // Set up user session
        this.setupUserSession(authResult.user)
        
        logger.info('User authenticated successfully', 'security', { 
          username, 
          role: authResult.user.role 
        })
        
        return { success: true, user: authResult.user }
      } else {
        this.recordFailedAttempt(clientIdentifier)
        logger.warn('Authentication failed', 'security', { username, error: authResult.error })
        return { success: false, error: authResult.error || 'Authentication failed.' }
      }

    } catch (error: any) {
      logger.error('Authentication error', 'security', { username, error: error.message })
      errorHandler.handleError({
        code: ErrorCodes.AUTHENTICATION_ERROR,
        message: 'Authentication system error',
        component: 'security',
        context: { username, error: error.message }
      })
      return { success: false, error: 'Authentication system error. Please try again.' }
    }
  }

  private async performAuthentication(username: string, password: string, totpCode?: string): Promise<{ success: boolean; user?: User; error?: string }> {
    // This is a simplified mock implementation
    // In production, this would authenticate against a secure backend

    // Mock users for demonstration
    const mockUsers: Record<string, { password: string; user: Omit<User, 'sessionId' | 'lastActivity' | 'isAuthenticated'> }> = {
      'admin': {
        password: 'SecurePassword123!',
        user: {
          id: 'admin_001',
          username: 'admin',
          email: 'admin@xr-swarm.com',
          role: UserRole.ADMIN,
          permissions: ['*'] // All permissions
        }
      },
      'commander': {
        password: 'Commander123!',
        user: {
          id: 'cmd_001',
          username: 'commander',
          email: 'commander@xr-swarm.com',
          role: UserRole.COMMANDER,
          permissions: ['command_swarm', 'view_all_agents', 'emergency_stop', 'mission_control']
        }
      },
      'operator': {
        password: 'Operator123!',
        user: {
          id: 'op_001',
          username: 'operator',
          email: 'operator@xr-swarm.com',
          role: UserRole.OPERATOR,
          permissions: ['control_agents', 'view_telemetry', 'basic_commands']
        }
      }
    }

    const userData = mockUsers[username]
    if (!userData) {
      return { success: false, error: 'Invalid username or password.' }
    }

    // Simple password check (in production, use proper password hashing)
    if (userData.password !== password) {
      return { success: false, error: 'Invalid username or password.' }
    }

    // Two-factor authentication check
    if (this.securityConfig.enableTwoFactor && !totpCode) {
      return { success: false, error: 'Two-factor authentication code required.' }
    }

    if (this.securityConfig.enableTwoFactor && totpCode) {
      const isValidTOTP = this.validateTOTP(totpCode, userData.user.id)
      if (!isValidTOTP) {
        return { success: false, error: 'Invalid two-factor authentication code.' }
      }
    }

    const user: User = {
      ...userData.user,
      sessionId: this.generateSecureToken(),
      lastActivity: Date.now(),
      isAuthenticated: true
    }

    return { success: true, user }
  }

  private setupUserSession(user: User) {
    this.currentUser = user
    
    // Store session info securely
    sessionStorage.setItem('user_session', JSON.stringify({
      userId: user.id,
      sessionId: user.sessionId,
      role: user.role,
      permissions: user.permissions
    }))

    // Set session timeout
    this.resetSessionTimeout()

    // Log user in monitoring systems
    logger.setUserId(user.id)
  }

  private resetSessionTimeout() {
    if (this.sessionTimeoutId) {
      clearTimeout(this.sessionTimeoutId)
    }

    this.sessionTimeoutId = setTimeout(() => {
      this.logout('session_timeout')
    }, this.securityConfig.sessionTimeout)
  }

  logout(reason: string = 'user_initiated') {
    if (this.currentUser) {
      logger.info('User logged out', 'security', { 
        userId: this.currentUser.id,
        reason 
      })
    }

    this.currentUser = null
    sessionStorage.removeItem('user_session')
    sessionStorage.removeItem('csrf_token')
    
    if (this.sessionTimeoutId) {
      clearTimeout(this.sessionTimeoutId)
      this.sessionTimeoutId = null
    }

    // Redirect to login page or show login modal
    window.dispatchEvent(new CustomEvent('user-logout', { detail: { reason } }))
  }

  hasPermission(permission: string): boolean {
    if (!this.currentUser || !this.currentUser.isAuthenticated) {
      return false
    }

    // Admin has all permissions
    if (this.currentUser.permissions.includes('*')) {
      return true
    }

    return this.currentUser.permissions.includes(permission)
  }

  requirePermission(permission: string): boolean {
    if (!this.hasPermission(permission)) {
      logger.warn('Unauthorized access attempt', 'security', { 
        userId: this.currentUser?.id,
        permission,
        userPermissions: this.currentUser?.permissions
      })
      
      this.triggerSecurityAlert('UNAUTHORIZED_ACCESS', { permission })
      throw new Error(`Access denied. Required permission: ${permission}`)
    }
    return true
  }

  private validateCredentials(username: string, password: string): boolean {
    // Basic validation
    if (!username || !password) {
      return false
    }

    if (username.length < 3 || username.length > 50) {
      return false
    }

    if (this.securityConfig.requireStrongPasswords) {
      return this.isStrongPassword(password)
    }

    return password.length >= 8
  }

  private isStrongPassword(password: string): boolean {
    // Strong password requirements
    const minLength = 12
    const hasUpperCase = /[A-Z]/.test(password)
    const hasLowerCase = /[a-z]/.test(password)
    const hasNumbers = /\d/.test(password)
    const hasSpecialChars = /[!@#$%^&*(),.?":{}|<>]/.test(password)

    return (
      password.length >= minLength &&
      hasUpperCase &&
      hasLowerCase &&
      hasNumbers &&
      hasSpecialChars
    )
  }

  private validateTOTP(code: string, userId: string): boolean {
    // This is a simplified implementation
    // In production, use a proper TOTP library like speakeasy
    
    // Mock validation for demonstration
    const validCodes = ['123456', '654321'] // Mock codes
    return validCodes.includes(code)
  }

  private checkRateLimit(identifier: string): boolean {
    const currentTime = Date.now()
    const requests = this.rateLimitTracker.get(identifier) || []
    
    // Filter to only recent requests (last minute)
    const recentRequests = requests.filter(time => currentTime - time < 60000)
    
    if (recentRequests.length >= this.securityConfig.rateLimitRequests) {
      return false
    }

    // Add current request
    recentRequests.push(currentTime)
    this.rateLimitTracker.set(identifier, recentRequests)
    
    return true
  }

  private isAccountLocked(identifier: string): boolean {
    const attempts = this.failedAttempts.get(identifier)
    if (!attempts) return false

    const timeSinceLastAttempt = Date.now() - attempts.lastAttempt
    const isInLockout = timeSinceLastAttempt < this.securityConfig.lockoutDuration
    
    return attempts.count >= this.securityConfig.maxFailedAttempts && isInLockout
  }

  private recordFailedAttempt(identifier: string) {
    const currentAttempts = this.failedAttempts.get(identifier) || { count: 0, lastAttempt: 0 }
    
    this.failedAttempts.set(identifier, {
      count: currentAttempts.count + 1,
      lastAttempt: Date.now()
    })
  }

  private getClientIdentifier(): string {
    // Generate a client identifier based on various factors
    const userAgent = navigator.userAgent
    const language = navigator.language
    const platform = navigator.platform
    const screenResolution = `${screen.width}x${screen.height}`
    
    // Simple hash function for identifier
    let hash = 0
    const str = `${userAgent}${language}${platform}${screenResolution}`
    for (let i = 0; i < str.length; i++) {
      const char = str.charCodeAt(i)
      hash = ((hash << 5) - hash) + char
      hash = hash & hash // Convert to 32-bit integer
    }
    
    return `client_${Math.abs(hash)}`
  }

  private generateSecureToken(): string {
    const array = new Uint8Array(32)
    crypto.getRandomValues(array)
    return Array.from(array, byte => byte.toString(16).padStart(2, '0')).join('')
  }

  private triggerSecurityAlert(alertType: string, data: any) {
    errorHandler.handleError({
      code: 'SECURITY_ALERT',
      message: `Security alert: ${alertType}`,
      severity: 'critical' as any,
      component: 'security',
      context: { alertType, data }
    })

    // In production, this would also send to security monitoring systems
    console.warn(`🚨 SECURITY ALERT: ${alertType}`, data)
  }

  // Public getters
  getCurrentUser(): User | null {
    return this.currentUser
  }

  isAuthenticated(): boolean {
    return this.currentUser?.isAuthenticated || false
  }

  updateActivity() {
    if (this.currentUser) {
      this.currentUser.lastActivity = Date.now()
      this.resetSessionTimeout()
    }
  }

  // Command validation with security checks
  validateCommand(command: any): { isValid: boolean; errors: string[] } {
    const errors: string[] = []

    if (!this.isAuthenticated()) {
      errors.push('Authentication required')
      return { isValid: false, errors }
    }

    // Check command permissions
    const commandPermissions: Record<string, string> = {
      'emergency_stop': 'emergency_stop',
      'takeoff': 'control_agents',
      'land': 'control_agents',
      'formation_line': 'command_swarm',
      'formation_grid': 'command_swarm',
      'formation_circle': 'command_swarm',
      'search_spiral': 'command_swarm',
      'return_to_base': 'control_agents'
    }

    const requiredPermission = commandPermissions[command.type]
    if (requiredPermission && !this.hasPermission(requiredPermission)) {
      errors.push(`Insufficient permissions for command: ${command.type}`)
    }

    // Rate limit commands
    const commandIdentifier = `cmd_${this.currentUser!.id}`
    if (!this.checkRateLimit(commandIdentifier)) {
      errors.push('Command rate limit exceeded')
    }

    return { isValid: errors.length === 0, errors }
  }
}

// Singleton instance
export const securityManager = new SecurityManager()

// React hook for authentication
import { useState, useEffect } from 'react'

export function useAuth() {
  const [user, setUser] = useState<User | null>(securityManager.getCurrentUser())
  const [isLoading, setIsLoading] = useState(false)

  useEffect(() => {
    // Listen for logout events
    const handleLogout = () => {
      setUser(null)
    }

    window.addEventListener('user-logout', handleLogout)
    
    return () => {
      window.removeEventListener('user-logout', handleLogout)
    }
  }, [])

  const login = async (credentials: { username: string; password: string; totpCode?: string }) => {
    setIsLoading(true)
    try {
      const result = await securityManager.authenticate(credentials)
      if (result.success && result.user) {
        setUser(result.user)
      }
      return result
    } finally {
      setIsLoading(false)
    }
  }

  const logout = () => {
    securityManager.logout()
    setUser(null)
  }

  const hasPermission = (permission: string) => {
    return securityManager.hasPermission(permission)
  }

  const updateActivity = () => {
    securityManager.updateActivity()
  }

  return {
    user,
    isAuthenticated: securityManager.isAuthenticated(),
    isLoading,
    login,
    logout,
    hasPermission,
    updateActivity
  }
}