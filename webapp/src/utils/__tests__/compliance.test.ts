import { describe, it, expect, beforeEach, vi, afterEach } from 'vitest'
import { complianceManager, initializeCompliance, getSessionId, UserConsent, DataProcessingRecord } from '../compliance'

// Mock localStorage
const localStorageMock = {
  getItem: vi.fn(),
  setItem: vi.fn(),
  removeItem: vi.fn(),
  clear: vi.fn(),
}

Object.defineProperty(window, 'localStorage', {
  value: localStorageMock
})

describe('Compliance Manager', () => {
  beforeEach(() => {
    vi.clearAllMocks()
    localStorageMock.getItem.mockReturnValue(null)
  })

  afterEach(() => {
    vi.restoreAllMocks()
  })

  describe('Consent Management', () => {
    it('should request consent for new session', async () => {
      const sessionId = 'test-session-123'
      
      const consent = await complianceManager.requestConsent(sessionId)
      
      expect(consent).toBeDefined()
      expect(consent?.sessionId).toBe(sessionId)
      expect(consent?.consents.necessary).toBe(true)
    })

    it('should return existing valid consent', async () => {
      const sessionId = 'test-session-123'
      const existingConsent: UserConsent = {
        sessionId,
        timestamp: Date.now(),
        version: '1.0.0',
        consents: {
          necessary: true,
          analytics: true,
          marketing: false,
          personalization: false,
          functional: true
        }
      }

      complianceManager.recordConsent(existingConsent)
      
      const consent = await complianceManager.requestConsent(sessionId)
      
      expect(consent).toEqual(existingConsent)
    })

    it('should update consent preferences', () => {
      const sessionId = 'test-session-123'
      const initialConsent: UserConsent = {
        sessionId,
        timestamp: Date.now(),
        version: '1.0.0',
        consents: {
          necessary: true,
          analytics: false,
          marketing: false,
          personalization: false,
          functional: false
        }
      }

      complianceManager.recordConsent(initialConsent)
      
      complianceManager.updateConsent(sessionId, {
        analytics: true,
        marketing: true
      })

      const updatedConsent = complianceManager.getConsent(sessionId)
      
      expect(updatedConsent?.consents.analytics).toBe(true)
      expect(updatedConsent?.consents.marketing).toBe(true)
      expect(updatedConsent?.consents.necessary).toBe(true)
      expect(updatedConsent?.consents.functional).toBe(false)
    })

    it('should withdraw consent for specific categories', () => {
      const sessionId = 'test-session-123'
      const initialConsent: UserConsent = {
        sessionId,
        timestamp: Date.now(),
        version: '1.0.0',
        consents: {
          necessary: true,
          analytics: true,
          marketing: true,
          personalization: true,
          functional: true
        }
      }

      complianceManager.recordConsent(initialConsent)
      
      complianceManager.withdrawConsent(sessionId, ['analytics', 'marketing'])

      const updatedConsent = complianceManager.getConsent(sessionId)
      
      expect(updatedConsent?.consents.analytics).toBe(false)
      expect(updatedConsent?.consents.marketing).toBe(false)
      expect(updatedConsent?.consents.necessary).toBe(true)
      expect(updatedConsent?.consents.functional).toBe(true)
    })

    it('should always keep necessary consent as true', () => {
      const sessionId = 'test-session-123'
      const initialConsent: UserConsent = {
        sessionId,
        timestamp: Date.now(),
        version: '1.0.0',
        consents: {
          necessary: true,
          analytics: true,
          marketing: true,
          personalization: true,
          functional: true
        }
      }

      complianceManager.recordConsent(initialConsent)
      
      complianceManager.withdrawConsent(sessionId)

      const updatedConsent = complianceManager.getConsent(sessionId)
      
      expect(updatedConsent?.consents.necessary).toBe(true)
      expect(updatedConsent?.consents.analytics).toBe(false)
      expect(updatedConsent?.consents.marketing).toBe(false)
    })
  })

  describe('Data Processing Records', () => {
    it('should record data processing activity', () => {
      const recordData = {
        userId: 'user-123',
        sessionId: 'session-123',
        dataType: 'personal' as const,
        purpose: 'user_authentication',
        legalBasis: 'contract' as const,
        retentionPeriod: 365 * 24 * 60 * 60 * 1000, // 1 year
        isAnonymized: false,
        processingLocation: 'EU'
      }

      const recordId = complianceManager.recordDataProcessing(recordData)
      
      expect(recordId).toBeDefined()
      expect(typeof recordId).toBe('string')

      const records = complianceManager.getDataProcessingRecords({
        userId: 'user-123'
      })
      
      expect(records).toHaveLength(1)
      expect(records[0].purpose).toBe('user_authentication')
      expect(records[0].dataType).toBe('personal')
    })

    it('should filter data processing records by criteria', () => {
      const baseRecord = {
        sessionId: 'session-123',
        retentionPeriod: 365 * 24 * 60 * 60 * 1000,
        isAnonymized: false,
        processingLocation: 'EU'
      }

      complianceManager.recordDataProcessing({
        ...baseRecord,
        userId: 'user-1',
        dataType: 'personal',
        purpose: 'authentication',
        legalBasis: 'contract'
      })

      complianceManager.recordDataProcessing({
        ...baseRecord,
        userId: 'user-2',
        dataType: 'analytics',
        purpose: 'usage_tracking',
        legalBasis: 'consent'
      })

      const personalRecords = complianceManager.getDataProcessingRecords({
        dataType: 'personal'
      })
      
      const user1Records = complianceManager.getDataProcessingRecords({
        userId: 'user-1'
      })

      expect(personalRecords).toHaveLength(1)
      expect(personalRecords[0].dataType).toBe('personal')
      
      expect(user1Records).toHaveLength(1)
      expect(user1Records[0].userId).toBe('user-1')
    })
  })

  describe('Data Erasure (Right to be Forgotten)', () => {
    it('should handle data erasure request', async () => {
      const sessionId = 'test-session-123'
      
      // Record consent
      const consent: UserConsent = {
        sessionId,
        timestamp: Date.now(),
        version: '1.0.0',
        consents: {
          necessary: true,
          analytics: true,
          marketing: true,
          personalization: true,
          functional: true
        }
      }
      complianceManager.recordConsent(consent)

      // Record some data processing
      complianceManager.recordDataProcessing({
        sessionId,
        dataType: 'personal',
        purpose: 'user_profile',
        legalBasis: 'consent',
        retentionPeriod: 365 * 24 * 60 * 60 * 1000,
        isAnonymized: false,
        processingLocation: 'EU'
      })

      const result = await complianceManager.handleDataErasureRequest({
        sessionId
      })

      expect(result.consentWithdrawn).toBe(true)
      expect(result.recordsDeleted).toBeGreaterThan(0)
      
      // Verify consent is removed
      const remainingConsent = complianceManager.getConsent(sessionId)
      expect(remainingConsent).toBeNull()
    })
  })

  describe('Data Export (Portability)', () => {
    it('should export user data in structured format', async () => {
      const sessionId = 'test-session-123'
      
      // Record consent
      const consent: UserConsent = {
        sessionId,
        timestamp: Date.now(),
        version: '1.0.0',
        consents: {
          necessary: true,
          analytics: true,
          marketing: false,
          personalization: true,
          functional: true
        }
      }
      complianceManager.recordConsent(consent)

      // Record some data processing
      complianceManager.recordDataProcessing({
        sessionId,
        dataType: 'analytics',
        purpose: 'usage_tracking',
        legalBasis: 'consent',
        retentionPeriod: 180 * 24 * 60 * 60 * 1000,
        isAnonymized: false,
        processingLocation: 'EU'
      })

      const exportData = await complianceManager.exportUserData({
        sessionId
      })

      expect(exportData.format).toBe('json')
      expect(exportData.consent).toEqual(consent)
      expect(exportData.processingRecords).toHaveLength(1)
      expect(exportData.exportDate).toBeDefined()
      expect(exportData.personalData).toBeDefined()
    })
  })

  describe('Data Retention', () => {
    it('should clean up expired data', async () => {
      const sessionId = 'test-session-123'
      const pastTimestamp = Date.now() - (400 * 24 * 60 * 60 * 1000) // 400 days ago
      
      // Record old consent
      const oldConsent: UserConsent = {
        sessionId,
        timestamp: pastTimestamp,
        version: '1.0.0',
        consents: {
          necessary: true,
          analytics: true,
          marketing: true,
          personalization: true,
          functional: true
        }
      }
      complianceManager.recordConsent(oldConsent)

      // Record expired data processing
      complianceManager.recordDataProcessing({
        sessionId,
        dataType: 'personal',
        purpose: 'expired_purpose',
        legalBasis: 'consent',
        retentionPeriod: 30 * 24 * 60 * 60 * 1000, // 30 days
        isAnonymized: false,
        processingLocation: 'EU'
      })

      const cleanupResult = await complianceManager.performDataRetentionCleanup()

      expect(cleanupResult.recordsDeleted + cleanupResult.recordsAnonymized).toBeGreaterThan(0)
      expect(cleanupResult.errors).toEqual([])
    })
  })

  describe('Compliance Validation', () => {
    it('should validate compliance status', () => {
      const validation = complianceManager.validateCompliance()

      expect(validation).toHaveProperty('isCompliant')
      expect(validation).toHaveProperty('issues')
      expect(validation).toHaveProperty('recommendations')
      expect(Array.isArray(validation.issues)).toBe(true)
      expect(Array.isArray(validation.recommendations)).toBe(true)
    })

    it('should detect outdated consent versions', () => {
      const sessionId = 'test-session-123'
      
      // Record consent with old version
      const oldConsent: UserConsent = {
        sessionId,
        timestamp: Date.now(),
        version: '0.9.0', // Old version
        consents: {
          necessary: true,
          analytics: true,
          marketing: false,
          personalization: false,
          functional: true
        }
      }
      complianceManager.recordConsent(oldConsent)

      const validation = complianceManager.validateCompliance()

      expect(validation.isCompliant).toBe(false)
      expect(validation.issues.some(issue => 
        issue.includes('consent records are outdated')
      )).toBe(true)
    })
  })

  describe('Session Management', () => {
    it('should generate unique session ID', () => {
      // Mock localStorage to return null initially
      localStorageMock.getItem.mockReturnValue(null)
      
      const sessionId1 = getSessionId()
      const sessionId2 = getSessionId()
      
      expect(sessionId1).toBeDefined()
      expect(typeof sessionId1).toBe('string')
      expect(sessionId1.startsWith('session_')).toBe(true)
      
      // Second call should return the same ID (cached)
      expect(sessionId1).toBe(sessionId2)
    })

    it('should use existing session ID from localStorage', () => {
      const existingSessionId = 'existing-session-456'
      localStorageMock.getItem.mockReturnValue(existingSessionId)
      
      const sessionId = getSessionId()
      
      expect(sessionId).toBe(existingSessionId)
    })
  })

  describe('Configuration', () => {
    it('should initialize with custom configuration', () => {
      const customConfig = {
        gdprEnabled: true,
        ccpaEnabled: false,
        cookiesEnabled: true,
        analyticsEnabled: false,
        dataRetentionDays: 180,
        region: 'US',
        consentVersion: '2.0.0'
      }

      const customManager = initializeCompliance(customConfig)
      
      expect(customManager).toBeDefined()
      
      // Verify that custom config affects behavior
      const validation = customManager.validateCompliance()
      expect(validation).toBeDefined()
    })
  })

  describe('Error Handling', () => {
    it('should handle invalid consent data gracefully', () => {
      const invalidConsent = {
        // Missing required fields
        timestamp: Date.now()
      } as any

      expect(() => {
        complianceManager.recordConsent(invalidConsent)
      }).toThrow('Invalid consent data')
    })

    it('should handle consent updates for non-existent sessions', () => {
      expect(() => {
        complianceManager.updateConsent('non-existent-session', {
          analytics: true
        })
      }).toThrow('No existing consent found')
    })

    it('should handle localStorage errors gracefully', () => {
      // Mock localStorage to throw error
      localStorageMock.setItem.mockImplementation(() => {
        throw new Error('localStorage error')
      })

      const sessionId = 'test-session-123'
      const consent: UserConsent = {
        sessionId,
        timestamp: Date.now(),
        version: '1.0.0',
        consents: {
          necessary: true,
          analytics: false,
          marketing: false,
          personalization: false,
          functional: false
        }
      }

      // Should not throw error, but handle gracefully
      expect(() => {
        complianceManager.recordConsent(consent)
      }).not.toThrow()
    })
  })
})

describe('Integration Tests', () => {
  it('should handle complete user journey', async () => {
    const sessionId = 'integration-test-session'
    
    // 1. User visits site - request consent
    const initialConsent = await complianceManager.requestConsent(sessionId)
    expect(initialConsent).toBeDefined()
    
    // 2. User accepts analytics
    complianceManager.updateConsent(sessionId, {
      analytics: true,
      functional: true
    })
    
    // 3. Record some data processing
    const recordId = complianceManager.recordDataProcessing({
      sessionId,
      dataType: 'analytics',
      purpose: 'user_behavior_analysis',
      legalBasis: 'consent',
      retentionPeriod: 365 * 24 * 60 * 60 * 1000,
      isAnonymized: false,
      processingLocation: 'EU'
    })
    
    expect(recordId).toBeDefined()
    
    // 4. User requests data export
    const exportData = await complianceManager.exportUserData({ sessionId })
    expect(exportData.consent?.consents.analytics).toBe(true)
    expect(exportData.processingRecords).toHaveLength(1)
    
    // 5. User withdraws analytics consent
    complianceManager.withdrawConsent(sessionId, ['analytics'])
    
    const updatedConsent = complianceManager.getConsent(sessionId)
    expect(updatedConsent?.consents.analytics).toBe(false)
    
    // 6. User requests data deletion
    const erasureResult = await complianceManager.handleDataErasureRequest({
      sessionId
    })
    
    expect(erasureResult.consentWithdrawn).toBe(true)
    
    // 7. Verify data is removed
    const finalConsent = complianceManager.getConsent(sessionId)
    expect(finalConsent).toBeNull()
  })
})