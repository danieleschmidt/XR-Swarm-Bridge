/**
 * Advanced Security Monitor
 * Generation 2: Zero-trust architecture with behavioral anomaly detection
 */

export interface SecurityEvent {
  id: string;
  type: 'authentication' | 'authorization' | 'data_access' | 'network' | 'behavioral_anomaly' | 'injection_attempt';
  severity: 'low' | 'medium' | 'high' | 'critical';
  source: string;
  target: string;
  timestamp: Date;
  details: Record<string, any>;
  riskScore: number;
  blocked: boolean;
  userAgent?: string;
  ipAddress?: string;
}

export interface BehavioralProfile {
  userId: string;
  sessionId: string;
  normalPatterns: {
    avgSessionDuration: number;
    avgActionsPerMinute: number;
    commonCommands: string[];
    typicalLatency: number;
    usualTimeRanges: Array<{ start: number; end: number }>;
  };
  deviationThresholds: {
    sessionDuration: number;
    actionsPerMinute: number;
    commandDeviation: number;
    latencyDeviation: number;
  };
  lastUpdated: Date;
  trustScore: number;
}

export interface SecurityRule {
  id: string;
  name: string;
  type: 'rate_limit' | 'pattern_match' | 'behavioral' | 'geographic' | 'data_protection';
  condition: (event: SecurityEvent, context: SecurityContext) => boolean;
  action: 'log' | 'warn' | 'block' | 'escalate' | 'quarantine';
  enabled: boolean;
  priority: number;
  metadata: Record<string, any>;
}

export interface SecurityContext {
  currentUser?: string;
  sessionId: string;
  requestCount: number;
  lastRequestTime: Date;
  geolocation?: { country: string; region: string };
  deviceFingerprint: string;
  threatIntelligence: Map<string, number>;
}

export interface ThreatIndicator {
  type: 'ip' | 'domain' | 'hash' | 'pattern';
  value: string;
  severity: number;
  source: string;
  lastSeen: Date;
  confidence: number;
}

export class AdvancedSecurityMonitor {
  private securityEvents: SecurityEvent[] = [];
  private behavioralProfiles: Map<string, BehavioralProfile> = new Map();
  private securityRules: Map<string, SecurityRule> = new Map();
  private threatIndicators: Map<string, ThreatIndicator> = new Map();
  private rateLimits: Map<string, { count: number; lastReset: Date }> = new Map();
  private quarantinedSessions: Set<string> = new Set();
  private monitoringActive = false;

  constructor() {
    this.initializeSecurityRules();
    this.loadThreatIntelligence();
    this.startMonitoring();
  }

  private initializeSecurityRules(): void {
    // Rate limiting rule
    this.addSecurityRule({
      id: 'command_rate_limit',
      name: 'Command Rate Limiting',
      type: 'rate_limit',
      condition: (event, context) => {
        if (event.type !== 'data_access') return false;
        
        const key = `${event.source}_commands`;
        const limit = this.rateLimits.get(key) || { count: 0, lastReset: new Date() };
        
        // Reset counter every minute
        if (Date.now() - limit.lastReset.getTime() > 60000) {
          limit.count = 0;
          limit.lastReset = new Date();
        }
        
        limit.count++;
        this.rateLimits.set(key, limit);
        
        return limit.count > 100; // Max 100 commands per minute
      },
      action: 'block',
      enabled: true,
      priority: 1,
      metadata: { maxCommands: 100, windowMinutes: 1 }
    });

    // SQL Injection detection
    this.addSecurityRule({
      id: 'sql_injection_detection',
      name: 'SQL Injection Detection',
      type: 'pattern_match',
      condition: (event) => {
        const sqlPatterns = [
          /(\b(SELECT|INSERT|UPDATE|DELETE|DROP|CREATE|ALTER)\b)/i,
          /'.*(\s*(OR|AND)\s*'.*=.*'|;\s*(DROP|DELETE|INSERT))/i,
          /(\bunion\b.*\bselect\b|\bor\b\s+\d+\s*=\s*\d+)/i
        ];
        
        const payload = JSON.stringify(event.details);
        return sqlPatterns.some(pattern => pattern.test(payload));
      },
      action: 'block',
      enabled: true,
      priority: 1,
      metadata: { patterns: 'SQL injection patterns' }
    });

    // XSS detection
    this.addSecurityRule({
      id: 'xss_detection',
      name: 'Cross-Site Scripting Detection',
      type: 'pattern_match',
      condition: (event) => {
        const xssPatterns = [
          /<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi,
          /javascript:/i,
          /on\w+\s*=/i,
          /<iframe\b/i,
          /expression\s*\(/i
        ];
        
        const payload = JSON.stringify(event.details);
        return xssPatterns.some(pattern => pattern.test(payload));
      },
      action: 'block',
      enabled: true,
      priority: 1,
      metadata: { patterns: 'XSS patterns' }
    });

    // Behavioral anomaly detection
    this.addSecurityRule({
      id: 'behavioral_anomaly',
      name: 'Behavioral Anomaly Detection',
      type: 'behavioral',
      condition: (event, context) => {
        if (!context.currentUser) return false;
        
        const profile = this.behavioralProfiles.get(context.currentUser);
        if (!profile) return false;
        
        return this.detectBehavioralAnomaly(event, profile, context);
      },
      action: 'warn',
      enabled: true,
      priority: 2,
      metadata: { sensitivity: 'medium' }
    });

    // Geolocation anomaly
    this.addSecurityRule({
      id: 'geo_anomaly',
      name: 'Geographic Anomaly Detection',
      type: 'geographic',
      condition: (event, context) => {
        if (!context.geolocation || !context.currentUser) return false;
        
        const profile = this.behavioralProfiles.get(context.currentUser);
        if (!profile || !profile.normalPatterns) return false;
        
        // Check if accessing from unusual location
        const knownLocations = profile.metadata?.knownCountries || [];
        return !knownLocations.includes(context.geolocation.country);
      },
      action: 'warn',
      enabled: true,
      priority: 3,
      metadata: { maxDistance: 1000 }
    });

    // Data exfiltration detection
    this.addSecurityRule({
      id: 'data_exfiltration',
      name: 'Data Exfiltration Detection',
      type: 'data_protection',
      condition: (event, context) => {
        if (event.type !== 'data_access') return false;
        
        const dataSize = event.details.dataSize || 0;
        const threshold = 10 * 1024 * 1024; // 10MB threshold
        
        return dataSize > threshold;
      },
      action: 'escalate',
      enabled: true,
      priority: 1,
      metadata: { maxDataSize: 10485760 }
    });
  }

  private loadThreatIntelligence(): void {
    // Simulated threat intelligence data
    const threats = [
      { type: 'ip', value: '192.168.1.100', severity: 8, source: 'internal_blacklist' },
      { type: 'pattern', value: 'admin123', severity: 6, source: 'weak_credentials' },
      { type: 'domain', value: 'malicious-site.com', severity: 9, source: 'reputation_db' }
    ];

    threats.forEach(threat => {
      this.addThreatIndicator({
        ...threat,
        lastSeen: new Date(),
        confidence: 0.85
      } as ThreatIndicator);
    });
  }

  private startMonitoring(): void {
    if (this.monitoringActive) return;
    
    this.monitoringActive = true;
    
    // Monitor WebRTC data channels for security events
    this.interceptWebRTCMessages();
    
    // Monitor DOM mutations for XSS attempts
    this.monitorDOMChanges();
    
    // Monitor network requests
    this.interceptNetworkRequests();
    
    console.log('Advanced security monitoring started');
  }

  private interceptWebRTCMessages(): void {
    // Hook into WebRTC message handling
    const originalSend = RTCDataChannel.prototype.send;
    RTCDataChannel.prototype.send = (data: string | ArrayBuffer | Blob) => {
      try {
        let payload = '';
        if (typeof data === 'string') {
          payload = data;
        } else if (data instanceof ArrayBuffer) {
          payload = new TextDecoder().decode(data);
        }
        
        const event: SecurityEvent = {
          id: `webrtc_${Date.now()}_${Math.random()}`,
          type: 'data_access',
          severity: 'low',
          source: 'webrtc_datachannel',
          target: 'robot_swarm',
          timestamp: new Date(),
          details: { payload, size: payload.length },
          riskScore: 0,
          blocked: false
        };
        
        this.processSecurityEvent(event);
        
      } catch (error) {
        console.error('Security monitoring error in WebRTC interception:', error);
      }
      
      return originalSend.call(this, data);
    };
  }

  private monitorDOMChanges(): void {
    if (typeof MutationObserver === 'undefined') return;
    
    const observer = new MutationObserver((mutations) => {
      mutations.forEach((mutation) => {
        if (mutation.type === 'childList') {
          mutation.addedNodes.forEach((node) => {
            if (node.nodeType === Node.ELEMENT_NODE) {
              const element = node as Element;
              
              // Check for potentially malicious elements
              if (element.tagName === 'SCRIPT' || element.tagName === 'IFRAME') {
                const event: SecurityEvent = {
                  id: `dom_${Date.now()}_${Math.random()}`,
                  type: 'injection_attempt',
                  severity: 'high',
                  source: 'dom_mutation',
                  target: 'application',
                  timestamp: new Date(),
                  details: { 
                    tagName: element.tagName,
                    innerHTML: element.innerHTML,
                    attributes: Array.from(element.attributes).map(attr => `${attr.name}="${attr.value}"`)
                  },
                  riskScore: 8,
                  blocked: false
                };
                
                this.processSecurityEvent(event);
              }
            }
          });
        }
      });
    });

    observer.observe(document.body, {
      childList: true,
      subtree: true
    });
  }

  private interceptNetworkRequests(): void {
    // Intercept fetch requests
    const originalFetch = window.fetch;
    window.fetch = async (input: RequestInfo | URL, init?: RequestInit) => {
      try {
        const url = typeof input === 'string' ? input : input.toString();
        
        const event: SecurityEvent = {
          id: `network_${Date.now()}_${Math.random()}`,
          type: 'network',
          severity: 'low',
          source: 'application',
          target: url,
          timestamp: new Date(),
          details: { 
            method: init?.method || 'GET',
            headers: init?.headers,
            body: init?.body
          },
          riskScore: 0,
          blocked: false
        };
        
        await this.processSecurityEvent(event);
        
      } catch (error) {
        console.error('Security monitoring error in fetch interception:', error);
      }
      
      return originalFetch.call(window, input, init);
    };
  }

  async processSecurityEvent(event: SecurityEvent): Promise<void> {
    // Calculate risk score
    event.riskScore = this.calculateRiskScore(event);
    
    // Check against threat intelligence
    this.checkThreatIntelligence(event);
    
    // Apply security rules
    const context = this.buildSecurityContext(event);
    await this.applySecurity(event, context);
    
    // Store event
    this.securityEvents.push(event);
    
    // Keep only last 10000 events
    if (this.securityEvents.length > 10000) {
      this.securityEvents = this.securityEvents.slice(-10000);
    }
    
    // Update behavioral profiles
    if (context.currentUser) {
      this.updateBehavioralProfile(context.currentUser, event, context);
    }
    
    // Log high-severity events
    if (event.severity === 'high' || event.severity === 'critical') {
      console.warn('High-severity security event detected:', event);
    }
  }

  private calculateRiskScore(event: SecurityEvent): number {
    let score = 0;
    
    // Base score by type
    const typeScores = {
      'authentication': 3,
      'authorization': 4,
      'data_access': 2,
      'network': 1,
      'behavioral_anomaly': 5,
      'injection_attempt': 8
    };
    
    score += typeScores[event.type] || 1;
    
    // Adjust for payload size
    if (event.details.size && event.details.size > 1024) {
      score += Math.min(3, Math.floor(event.details.size / 1024));
    }
    
    // Check for suspicious patterns
    const payload = JSON.stringify(event.details).toLowerCase();
    const suspiciousPatterns = ['password', 'token', 'secret', 'admin', 'root', 'eval'];
    
    suspiciousPatterns.forEach(pattern => {
      if (payload.includes(pattern)) {
        score += 2;
      }
    });
    
    return Math.min(10, score);
  }

  private checkThreatIntelligence(event: SecurityEvent): void {
    const payload = JSON.stringify(event.details);
    
    for (const [key, indicator] of this.threatIndicators.entries()) {
      if (indicator.type === 'pattern' && payload.includes(indicator.value)) {
        event.riskScore += indicator.severity;
        event.details.threatMatches = event.details.threatMatches || [];
        event.details.threatMatches.push({
          type: indicator.type,
          value: indicator.value,
          source: indicator.source
        });
      }
    }
  }

  private buildSecurityContext(event: SecurityEvent): SecurityContext {
    return {
      sessionId: this.generateSessionId(),
      requestCount: this.getRequestCount(event.source),
      lastRequestTime: new Date(),
      deviceFingerprint: this.generateDeviceFingerprint(),
      threatIntelligence: this.threatIndicators
    };
  }

  private async applySecurity(event: SecurityEvent, context: SecurityContext): Promise<void> {
    const applicableRules = Array.from(this.securityRules.values())
      .filter(rule => rule.enabled)
      .sort((a, b) => a.priority - b.priority);

    for (const rule of applicableRules) {
      try {
        if (rule.condition(event, context)) {
          await this.executeSecurityAction(rule.action, event, context, rule);
          
          // Log rule execution
          console.log(`Security rule "${rule.name}" triggered for event ${event.id}`);
          
          if (rule.action === 'block' || rule.action === 'quarantine') {
            event.blocked = true;
            break; // Stop processing further rules if blocked
          }
        }
      } catch (error) {
        console.error(`Error executing security rule ${rule.id}:`, error);
      }
    }
  }

  private async executeSecurityAction(
    action: string, 
    event: SecurityEvent, 
    context: SecurityContext,
    rule: SecurityRule
  ): Promise<void> {
    switch (action) {
      case 'log':
        console.log('Security event logged:', event);
        break;
        
      case 'warn':
        console.warn('Security warning:', event);
        this.sendSecurityAlert(event, 'warning');
        break;
        
      case 'block':
        console.error('Security event blocked:', event);
        this.sendSecurityAlert(event, 'blocked');
        
        // Additional blocking logic
        if (event.type === 'data_access') {
          // Block the operation (implementation would depend on context)
          throw new Error('Operation blocked by security policy');
        }
        break;
        
      case 'escalate':
        console.error('Security event escalated:', event);
        this.sendSecurityAlert(event, 'escalated');
        await this.escalateToAdmin(event, context);
        break;
        
      case 'quarantine':
        console.error('Session quarantined:', context.sessionId);
        this.quarantinedSessions.add(context.sessionId);
        this.sendSecurityAlert(event, 'quarantined');
        break;
    }
  }

  private sendSecurityAlert(event: SecurityEvent, status: string): void {
    // In a real implementation, this would send alerts to a SIEM system
    const alert = {
      timestamp: new Date().toISOString(),
      eventId: event.id,
      status,
      severity: event.severity,
      summary: `${event.type} event ${status}`,
      details: event
    };
    
    console.warn('Security Alert:', alert);
  }

  private async escalateToAdmin(event: SecurityEvent, context: SecurityContext): Promise<void> {
    // Simulate admin notification
    const escalation = {
      timestamp: new Date().toISOString(),
      event,
      context,
      recommendedActions: this.getRecommendedActions(event),
      priority: event.severity === 'critical' ? 'immediate' : 'high'
    };
    
    console.error('Security Escalation:', escalation);
  }

  private getRecommendedActions(event: SecurityEvent): string[] {
    const actions = [];
    
    switch (event.type) {
      case 'injection_attempt':
        actions.push('Review input validation', 'Check for application vulnerabilities', 'Monitor for additional attempts');
        break;
      case 'behavioral_anomaly':
        actions.push('Verify user identity', 'Review recent actions', 'Consider account lockout');
        break;
      case 'data_access':
        actions.push('Review data access permissions', 'Audit data usage', 'Check for data exfiltration');
        break;
      default:
        actions.push('Investigate event context', 'Review security policies', 'Monitor for patterns');
    }
    
    return actions;
  }

  private detectBehavioralAnomaly(event: SecurityEvent, profile: BehavioralProfile, context: SecurityContext): boolean {
    const now = new Date();
    const currentHour = now.getHours();
    
    // Check time-based anomalies
    const isUsualTime = profile.normalPatterns.usualTimeRanges.some(range => 
      currentHour >= range.start && currentHour <= range.end
    );
    
    if (!isUsualTime) {
      return true;
    }
    
    // Check command pattern anomalies
    if (event.details.command && event.type === 'data_access') {
      const command = event.details.command;
      const isCommonCommand = profile.normalPatterns.commonCommands.includes(command);
      
      if (!isCommonCommand && profile.normalPatterns.commonCommands.length > 10) {
        return true;
      }
    }
    
    // Check frequency anomalies
    const recentActions = this.securityEvents.filter(e => 
      e.source === event.source && 
      Date.now() - e.timestamp.getTime() < 60000 // Last minute
    ).length;
    
    if (recentActions > profile.normalPatterns.avgActionsPerMinute * 2) {
      return true;
    }
    
    return false;
  }

  private updateBehavioralProfile(userId: string, event: SecurityEvent, context: SecurityContext): void {
    let profile = this.behavioralProfiles.get(userId);
    
    if (!profile) {
      profile = {
        userId,
        sessionId: context.sessionId,
        normalPatterns: {
          avgSessionDuration: 30 * 60 * 1000, // 30 minutes
          avgActionsPerMinute: 10,
          commonCommands: [],
          typicalLatency: 100,
          usualTimeRanges: [{ start: 9, end: 17 }] // 9 AM to 5 PM
        },
        deviationThresholds: {
          sessionDuration: 0.5,
          actionsPerMinute: 0.8,
          commandDeviation: 0.3,
          latencyDeviation: 0.6
        },
        lastUpdated: new Date(),
        trustScore: 0.5
      };
      
      this.behavioralProfiles.set(userId, profile);
    }
    
    // Update patterns based on current event
    if (event.details.command && event.type === 'data_access') {
      const commands = profile.normalPatterns.commonCommands;
      if (!commands.includes(event.details.command)) {
        commands.push(event.details.command);
        // Keep only most common 20 commands
        if (commands.length > 20) {
          commands.splice(0, 1);
        }
      }
    }
    
    // Update trust score
    if (event.blocked || event.riskScore > 5) {
      profile.trustScore = Math.max(0, profile.trustScore - 0.1);
    } else {
      profile.trustScore = Math.min(1, profile.trustScore + 0.01);
    }
    
    profile.lastUpdated = new Date();
  }

  private generateSessionId(): string {
    return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  private generateDeviceFingerprint(): string {
    const canvas = document.createElement('canvas');
    const ctx = canvas.getContext('2d');
    ctx!.textBaseline = 'top';
    ctx!.font = '14px Arial';
    ctx!.fillText('Device fingerprint', 2, 2);
    
    const fingerprint = [
      navigator.userAgent,
      navigator.language,
      screen.width + 'x' + screen.height,
      new Date().getTimezoneOffset(),
      canvas.toDataURL()
    ].join('|');
    
    // Simple hash
    let hash = 0;
    for (let i = 0; i < fingerprint.length; i++) {
      const char = fingerprint.charCodeAt(i);
      hash = ((hash << 5) - hash) + char;
      hash = hash & hash; // Convert to 32-bit integer
    }
    
    return hash.toString(36);
  }

  private getRequestCount(source: string): number {
    return this.securityEvents.filter(e => e.source === source).length;
  }

  // Public API methods
  addSecurityRule(rule: SecurityRule): void {
    this.securityRules.set(rule.id, rule);
  }

  addThreatIndicator(indicator: ThreatIndicator): void {
    this.threatIndicators.set(indicator.value, indicator);
  }

  getSecurityEvents(limit: number = 100): SecurityEvent[] {
    return this.securityEvents.slice(-limit);
  }

  getSecurityMetrics(): {
    totalEvents: number;
    blockedEvents: number;
    highRiskEvents: number;
    quarantinedSessions: number;
    threatDetections: number;
  } {
    const totalEvents = this.securityEvents.length;
    const blockedEvents = this.securityEvents.filter(e => e.blocked).length;
    const highRiskEvents = this.securityEvents.filter(e => e.riskScore >= 7).length;
    const threatDetections = this.securityEvents.filter(e => e.details.threatMatches).length;
    
    return {
      totalEvents,
      blockedEvents,
      highRiskEvents,
      quarantinedSessions: this.quarantinedSessions.size,
      threatDetections
    };
  }

  generateSecurityReport(): string {
    const metrics = this.getSecurityMetrics();
    const recentEvents = this.getSecurityEvents(50);
    const topThreats = Array.from(this.threatIndicators.values())
      .sort((a, b) => b.severity - a.severity)
      .slice(0, 10);
    
    const report = {
      timestamp: new Date().toISOString(),
      securityMetrics: metrics,
      detectionRate: metrics.totalEvents > 0 ? 
        `${((metrics.blockedEvents / metrics.totalEvents) * 100).toFixed(1)}%` : '0%',
      topEventTypes: this.getTopEventTypes(),
      recentHighRiskEvents: recentEvents.filter(e => e.riskScore >= 7),
      topThreatIndicators: topThreats,
      activeBehavioralProfiles: this.behavioralProfiles.size,
      activeSecurityRules: Array.from(this.securityRules.values()).filter(r => r.enabled).length
    };
    
    return JSON.stringify(report, null, 2);
  }

  private getTopEventTypes(): Array<{ type: string; count: number }> {
    const typeCounts = new Map<string, number>();
    
    this.securityEvents.forEach(event => {
      typeCounts.set(event.type, (typeCounts.get(event.type) || 0) + 1);
    });
    
    return Array.from(typeCounts.entries())
      .map(([type, count]) => ({ type, count }))
      .sort((a, b) => b.count - a.count)
      .slice(0, 5);
  }

  stopMonitoring(): void {
    this.monitoringActive = false;
    console.log('Advanced security monitoring stopped');
  }
}

export const advancedSecurityMonitor = new AdvancedSecurityMonitor();