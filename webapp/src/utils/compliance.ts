/**
 * Global Compliance Management System
 * Handles GDPR, CCPA, and other privacy regulations
 */

export interface ComplianceConfig {
  gdprEnabled: boolean
  ccpaEnabled: boolean
  cookiesEnabled: boolean
  analyticsEnabled: boolean
  dataRetentionDays: number
  region: string
  consentVersion: string
}

export interface UserConsent {
  userId?: string
  sessionId: string
  timestamp: number
  version: string
  consents: {
    necessary: boolean
    analytics: boolean
    marketing: boolean
    personalization: boolean
    functional: boolean
  }
  ipAddress?: string
  userAgent?: string
  geoLocation?: {
    country: string
    region: string
    city?: string
  }
}

export interface DataProcessingRecord {
  id: string
  userId?: string
  sessionId: string
  dataType: 'personal' | 'telemetry' | 'analytics' | 'functional'
  purpose: string
  legalBasis: 'consent' | 'legitimate_interest' | 'contract' | 'legal_obligation'
  timestamp: number
  retentionPeriod: number
  isAnonymized: boolean
  processingLocation: string
}

class ComplianceManager {
  private config: ComplianceConfig
  private consentData: Map<string, UserConsent> = new Map()
  private processingRecords: DataProcessingRecord[] = []

  constructor(config: ComplianceConfig) {
    this.config = config
    this.loadStoredConsents()
  }

  // Consent Management
  async requestConsent(sessionId: string, options: {
    showBanner?: boolean
    forceModal?: boolean
    categories?: string[]
  } = {}): Promise<UserConsent | null> {
    // Check if consent already exists and is current
    const existingConsent = this.getConsent(sessionId)
    if (existingConsent && existingConsent.version === this.config.consentVersion) {
      return existingConsent
    }

    // Show consent UI (would integrate with your UI framework)
    return new Promise((resolve) => {
      this.showConsentUI(sessionId, resolve, options)
    })
  }

  recordConsent(consent: UserConsent): void {
    // Validate consent data
    if (!this.validateConsent(consent)) {
      throw new Error('Invalid consent data')
    }

    // Store consent
    this.consentData.set(consent.sessionId, consent)
    
    // Persist to storage
    this.persistConsent(consent)

    // Log compliance event
    this.logComplianceEvent('consent_recorded', {
      sessionId: consent.sessionId,
      version: consent.version,
      timestamp: consent.timestamp
    })

    // Configure services based on consent
    this.configureServices(consent)
  }

  getConsent(sessionId: string): UserConsent | null {
    return this.consentData.get(sessionId) || null
  }

  updateConsent(sessionId: string, updates: Partial<UserConsent['consents']>): void {
    const existing = this.getConsent(sessionId)
    if (!existing) {
      throw new Error('No existing consent found')
    }

    const updated: UserConsent = {
      ...existing,
      consents: { ...existing.consents, ...updates },
      timestamp: Date.now(),
      version: this.config.consentVersion
    }

    this.recordConsent(updated)
  }

  withdrawConsent(sessionId: string, categories?: string[]): void {
    const existing = this.getConsent(sessionId)
    if (!existing) return

    const withdrawnConsents = categories 
      ? Object.fromEntries(
          Object.entries(existing.consents).map(([key, value]) =>
            categories.includes(key) ? [key, false] : [key, value]
          )
        )
      : {
          necessary: true, // Always required
          analytics: false,
          marketing: false,
          personalization: false,
          functional: false
        }

    this.updateConsent(sessionId, withdrawnConsents)
    
    // Trigger data cleanup for withdrawn categories
    this.handleConsentWithdrawal(sessionId, categories || Object.keys(existing.consents))
  }

  // Data Processing Records
  recordDataProcessing(record: Omit<DataProcessingRecord, 'id' | 'timestamp'>): string {
    const fullRecord: DataProcessingRecord = {
      ...record,
      id: this.generateId(),
      timestamp: Date.now()
    }

    this.processingRecords.push(fullRecord)
    
    // Persist record
    this.persistProcessingRecord(fullRecord)

    return fullRecord.id
  }

  getDataProcessingRecords(filters: {
    userId?: string
    sessionId?: string
    dataType?: string
    startDate?: number
    endDate?: number
  } = {}): DataProcessingRecord[] {
    return this.processingRecords.filter(record => {
      if (filters.userId && record.userId !== filters.userId) return false
      if (filters.sessionId && record.sessionId !== filters.sessionId) return false
      if (filters.dataType && record.dataType !== filters.dataType) return false
      if (filters.startDate && record.timestamp < filters.startDate) return false
      if (filters.endDate && record.timestamp > filters.endDate) return false
      return true
    })
  }

  // Right to Erasure (GDPR Article 17)
  async handleDataErasureRequest(identifier: { userId?: string; sessionId?: string }): Promise<{
    recordsDeleted: number
    consentWithdrawn: boolean
    dataAnonymized: number
  }> {
    const result = {
      recordsDeleted: 0,
      consentWithdrawn: false,
      dataAnonymized: 0
    }

    // Remove consent data
    if (identifier.sessionId) {
      const consent = this.consentData.get(identifier.sessionId)
      if (consent) {
        this.consentData.delete(identifier.sessionId)
        this.removeStoredConsent(identifier.sessionId)
        result.consentWithdrawn = true
      }
    }

    // Handle processing records
    const recordsToProcess = this.getDataProcessingRecords(identifier)
    
    for (const record of recordsToProcess) {
      if (record.legalBasis === 'consent' || record.dataType === 'personal') {
        // Remove or anonymize based on legal requirements
        if (await this.canDeleteRecord(record)) {
          this.removeProcessingRecord(record.id)
          result.recordsDeleted++
        } else {
          await this.anonymizeRecord(record)
          result.dataAnonymized++
        }
      }
    }

    // Log erasure request
    this.logComplianceEvent('data_erasure_processed', {
      identifier,
      result,
      timestamp: Date.now()
    })

    return result
  }

  // Data Portability (GDPR Article 20)
  async exportUserData(identifier: { userId?: string; sessionId?: string }): Promise<{
    consent: UserConsent | null
    processingRecords: DataProcessingRecord[]
    personalData: any
    exportDate: number
    format: 'json'
  }> {
    const consent = identifier.sessionId ? this.getConsent(identifier.sessionId) : null
    const processingRecords = this.getDataProcessingRecords(identifier)
    
    // Collect personal data from various sources
    const personalData = await this.collectPersonalData(identifier)

    const exportData = {
      consent,
      processingRecords,
      personalData,
      exportDate: Date.now(),
      format: 'json' as const
    }

    // Log export request
    this.logComplianceEvent('data_export_processed', {
      identifier,
      recordCount: processingRecords.length,
      timestamp: Date.now()
    })

    return exportData
  }

  // Data Retention Management
  async performDataRetentionCleanup(): Promise<{
    recordsDeleted: number
    recordsAnonymized: number
    errors: string[]
  }> {
    const result = {
      recordsDeleted: 0,
      recordsAnonymized: 0,
      errors: []
    }

    const cutoffDate = Date.now() - (this.config.dataRetentionDays * 24 * 60 * 60 * 1000)
    const expiredRecords = this.processingRecords.filter(record => 
      record.timestamp < cutoffDate && 
      record.timestamp + record.retentionPeriod < Date.now()
    )

    for (const record of expiredRecords) {
      try {
        if (record.dataType === 'personal' || record.legalBasis === 'consent') {
          if (await this.canDeleteRecord(record)) {
            this.removeProcessingRecord(record.id)
            result.recordsDeleted++
          } else {
            await this.anonymizeRecord(record)
            result.recordsAnonymized++
          }
        }
      } catch (error) {
        result.errors.push(`Failed to process record ${record.id}: ${error}`)
      }
    }

    // Clean up old consent records
    for (const [sessionId, consent] of this.consentData.entries()) {
      if (consent.timestamp < cutoffDate) {
        this.consentData.delete(sessionId)
        this.removeStoredConsent(sessionId)
      }
    }

    this.logComplianceEvent('retention_cleanup_completed', result)

    return result
  }

  // Compliance Validation
  validateCompliance(): {
    isCompliant: boolean
    issues: string[]
    recommendations: string[]
  } {
    const issues: string[] = []
    const recommendations: string[] = []

    // Check consent version currency
    const outdatedConsents = Array.from(this.consentData.values())
      .filter(consent => consent.version !== this.config.consentVersion)
    
    if (outdatedConsents.length > 0) {
      issues.push(`${outdatedConsents.length} consent records are outdated`)
      recommendations.push('Request updated consent from users with outdated records')
    }

    // Check data retention compliance
    const expiredRecords = this.getExpiredRecords()
    if (expiredRecords.length > 0) {
      issues.push(`${expiredRecords.length} records exceed retention period`)
      recommendations.push('Run data retention cleanup process')
    }

    // Check legal basis for processing
    const recordsWithoutLegalBasis = this.processingRecords.filter(record => 
      !record.legalBasis || record.legalBasis === 'consent' && !this.hasValidConsent(record.sessionId)
    )
    
    if (recordsWithoutLegalBasis.length > 0) {
      issues.push(`${recordsWithoutLegalBasis.length} records lack valid legal basis`)
      recommendations.push('Review and update legal basis for processing')
    }

    return {
      isCompliant: issues.length === 0,
      issues,
      recommendations
    }
  }

  // Private Methods
  private validateConsent(consent: UserConsent): boolean {
    return !!(
      consent.sessionId &&
      consent.timestamp &&
      consent.version &&
      consent.consents &&
      typeof consent.consents.necessary === 'boolean'
    )
  }

  private showConsentUI(sessionId: string, resolve: (consent: UserConsent | null) => void, options: any): void {
    // This would integrate with your UI framework to show consent banner/modal
    // For now, we'll simulate with a basic implementation
    console.log('Showing consent UI for session:', sessionId, options)
    
    // Simulate user providing consent
    setTimeout(() => {
      const consent: UserConsent = {
        sessionId,
        timestamp: Date.now(),
        version: this.config.consentVersion,
        consents: {
          necessary: true,
          analytics: false,
          marketing: false,
          personalization: false,
          functional: false
        }
      }
      resolve(consent)
    }, 100)
  }

  private configureServices(consent: UserConsent): void {
    // Configure analytics
    if (consent.consents.analytics && this.config.analyticsEnabled) {
      this.enableAnalytics()
    } else {
      this.disableAnalytics()
    }

    // Configure cookies
    if (!consent.consents.functional) {
      this.clearNonEssentialCookies()
    }
  }

  private handleConsentWithdrawal(sessionId: string, categories: string[]): void {
    // Clean up data for withdrawn categories
    categories.forEach(category => {
      switch (category) {
        case 'analytics':
          this.disableAnalytics()
          this.deleteAnalyticsData(sessionId)
          break
        case 'marketing':
          this.deleteMarketingData(sessionId)
          break
        case 'personalization':
          this.deletePersonalizationData(sessionId)
          break
      }
    })
  }

  private enableAnalytics(): void {
    // Enable analytics tracking
    if (typeof window !== 'undefined' && (window as any).gtag) {
      (window as any).gtag('consent', 'update', {
        analytics_storage: 'granted'
      })
    }
  }

  private disableAnalytics(): void {
    // Disable analytics tracking
    if (typeof window !== 'undefined' && (window as any).gtag) {
      (window as any).gtag('consent', 'update', {
        analytics_storage: 'denied'
      })
    }
  }

  private clearNonEssentialCookies(): void {
    if (typeof document !== 'undefined') {
      // Clear non-essential cookies
      document.cookie.split(';').forEach(cookie => {
        const [name] = cookie.split('=')
        if (name.trim() && !this.isEssentialCookie(name.trim())) {
          document.cookie = `${name}=; expires=Thu, 01 Jan 1970 00:00:00 UTC; path=/;`
        }
      })
    }
  }

  private isEssentialCookie(name: string): boolean {
    const essentialCookies = ['session', 'auth', 'csrf', 'consent']
    return essentialCookies.some(essential => name.toLowerCase().includes(essential))
  }

  private deleteAnalyticsData(sessionId: string): void {
    // Remove analytics data for this session
    this.processingRecords = this.processingRecords.filter(record => 
      !(record.sessionId === sessionId && record.dataType === 'analytics')
    )
  }

  private deleteMarketingData(sessionId: string): void {
    // Remove marketing data for this session
    this.processingRecords = this.processingRecords.filter(record => 
      !(record.sessionId === sessionId && record.purpose.includes('marketing'))
    )
  }

  private deletePersonalizationData(sessionId: string): void {
    // Remove personalization data for this session
    this.processingRecords = this.processingRecords.filter(record => 
      !(record.sessionId === sessionId && record.purpose.includes('personalization'))
    )
  }

  private getExpiredRecords(): DataProcessingRecord[] {
    const now = Date.now()
    return this.processingRecords.filter(record => 
      record.timestamp + record.retentionPeriod < now
    )
  }

  private hasValidConsent(sessionId: string): boolean {
    const consent = this.getConsent(sessionId)
    return !!(consent && consent.version === this.config.consentVersion)
  }

  private async canDeleteRecord(record: DataProcessingRecord): Promise<boolean> {
    // Check if record can be safely deleted based on legal requirements
    return record.legalBasis === 'consent' || record.dataType === 'personal'
  }

  private async anonymizeRecord(record: DataProcessingRecord): Promise<void> {
    // Anonymize the record data
    const anonymizedRecord = {
      ...record,
      userId: undefined,
      isAnonymized: true
    }
    
    const index = this.processingRecords.findIndex(r => r.id === record.id)
    if (index >= 0) {
      this.processingRecords[index] = anonymizedRecord
    }
  }

  private removeProcessingRecord(id: string): void {
    this.processingRecords = this.processingRecords.filter(record => record.id !== id)
  }

  private async collectPersonalData(identifier: { userId?: string; sessionId?: string }): Promise<any> {
    // Collect personal data from various sources in your application
    return {
      identifier,
      collectDate: Date.now(),
      sources: ['consent', 'telemetry', 'user_preferences']
    }
  }

  private generateId(): string {
    return `${Date.now()}-${Math.random().toString(36).substr(2, 9)}`
  }

  private loadStoredConsents(): void {
    if (typeof localStorage !== 'undefined') {
      try {
        const stored = localStorage.getItem('compliance_consents')
        if (stored) {
          const consents = JSON.parse(stored)
          Object.entries(consents).forEach(([sessionId, consent]) => {
            this.consentData.set(sessionId, consent as UserConsent)
          })
        }
      } catch (error) {
        console.warn('Failed to load stored consents:', error)
      }
    }
  }

  private persistConsent(consent: UserConsent): void {
    if (typeof localStorage !== 'undefined') {
      try {
        const existing = localStorage.getItem('compliance_consents')
        const consents = existing ? JSON.parse(existing) : {}
        consents[consent.sessionId] = consent
        localStorage.setItem('compliance_consents', JSON.stringify(consents))
      } catch (error) {
        const errorMessage = error instanceof Error ? error.message : 'Unknown error'
        this.logComplianceEvent('persist_error', { error: errorMessage })
        // Don't throw error to prevent UI disruption
        console.warn('Failed to persist consent:', error)
      }
    }
  }

  private removeStoredConsent(sessionId: string): void {
    if (typeof localStorage !== 'undefined') {
      try {
        const existing = localStorage.getItem('compliance_consents')
        if (existing) {
          const consents = JSON.parse(existing)
          delete consents[sessionId]
          localStorage.setItem('compliance_consents', JSON.stringify(consents))
        }
      } catch (error) {
        console.warn('Failed to remove stored consent:', error)
      }
    }
  }

  private persistProcessingRecord(record: DataProcessingRecord): void {
    // In a real implementation, this would persist to a secure backend
    console.log('Persisting processing record:', record.id)
  }

  private logComplianceEvent(event: string, data: any): void {
    console.log(`[Compliance] ${event}:`, data)
    // In production, send to compliance audit log
  }
}

// Default configuration
const defaultConfig: ComplianceConfig = {
  gdprEnabled: true,
  ccpaEnabled: true,
  cookiesEnabled: true,
  analyticsEnabled: true,
  dataRetentionDays: 365,
  region: 'EU',
  consentVersion: '1.0.0'
}

// Global compliance manager instance
export const complianceManager = new ComplianceManager(defaultConfig)

// Utility functions
export function initializeCompliance(config?: Partial<ComplianceConfig>): ComplianceManager {
  return new ComplianceManager({ ...defaultConfig, ...config })
}

export function getSessionId(): string {
  if (typeof window !== 'undefined') {
    let sessionId = localStorage.getItem('session_id')
    if (!sessionId) {
      sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
      localStorage.setItem('session_id', sessionId)
    }
    return sessionId
  }
  return `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`
}

// React hooks for compliance
export function useCompliance() {
  const sessionId = getSessionId()
  
  return {
    sessionId,
    requestConsent: (options?: any) => complianceManager.requestConsent(sessionId, options),
    getConsent: () => complianceManager.getConsent(sessionId),
    updateConsent: (updates: any) => complianceManager.updateConsent(sessionId, updates),
    withdrawConsent: (categories?: string[]) => complianceManager.withdrawConsent(sessionId, categories),
    recordDataProcessing: (record: any) => complianceManager.recordDataProcessing({
      ...record,
      sessionId
    }),
    exportData: () => complianceManager.exportUserData({ sessionId }),
    requestErasure: () => complianceManager.handleDataErasureRequest({ sessionId })
  }
}