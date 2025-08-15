import React from 'react'
import { render, screen, fireEvent, waitFor } from '@testing-library/react'
import { BrowserRouter } from 'react-router-dom'
import { describe, it, expect, beforeEach, vi } from 'vitest'
import App from '../App'
import i18n from '../i18n'
import { complianceManager } from '../utils/compliance'

// Mock Three.js and WebGL context
vi.mock('@react-three/fiber', () => ({
  Canvas: ({ children }: any) => <div data-testid="canvas">{children}</div>,
  useFrame: () => {},
  useThree: () => ({
    camera: { position: { set: vi.fn() } },
    scene: { add: vi.fn(), remove: vi.fn() }
  })
}))

vi.mock('@react-three/drei', () => ({
  OrbitControls: () => <div data-testid="orbit-controls" />,
  Stats: () => <div data-testid="stats" />,
  Environment: () => <div data-testid="environment" />
}))

vi.mock('@react-three/xr', () => ({
  VRButton: () => <button data-testid="vr-button">Enter VR</button>,
  ARButton: () => <button data-testid="ar-button">Enter AR</button>,
  XR: ({ children }: any) => <div data-testid="xr">{children}</div>,
  useXR: () => ({
    isPresenting: false,
    player: { position: [0, 0, 0], rotation: [0, 0, 0] },
    session: null,
    visibilityState: 'visible'
  }),
  useController: () => ({
    controller: null,
    grip: null,
    inputSource: null
  }),
  useHitTest: () => [],
  Hands: ({ children }: any) => <div data-testid="hands">{children}</div>,
  Controllers: ({ children }: any) => <div data-testid="controllers">{children}</div>
}))

vi.mock('leva', () => ({
  Leva: () => <div data-testid="leva" />
}))

// Mock WebRTC and Socket.IO
vi.mock('../hooks/useWebRTC', () => ({
  useWebRTC: () => ({
    isConnected: false,
    connect: vi.fn(),
    disconnect: vi.fn()
  })
}))

vi.mock('../hooks/useSwarmConnection', () => ({
  useSwarmConnection: () => ({
    isConnected: false,
    sendGlobalCommand: vi.fn(),
    sendAgentCommand: vi.fn(),
    startMission: vi.fn(),
    stopMission: vi.fn()
  })
}))

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

// Test wrapper component
function TestWrapper({ children }: { children: React.ReactNode }) {
  return (
    <BrowserRouter>
      {children}
    </BrowserRouter>
  )
}

describe('XR-Swarm-Bridge Integration Tests', () => {
  beforeEach(() => {
    vi.clearAllMocks()
    localStorageMock.getItem.mockReturnValue(null)
  })

  describe('Application Initialization', () => {
    it('should render main application components', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Check for main UI elements
      expect(screen.getByTestId('canvas')).toBeInTheDocument()
      expect(screen.getByTestId('vr-button')).toBeInTheDocument()
      expect(screen.getByTestId('ar-button')).toBeInTheDocument()

      // Check for language selector
      expect(screen.getByRole('button', { name: /select language/i })).toBeInTheDocument()
    })

    it('should initialize with default language settings', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Should default to English
      expect(i18n.language).toBe('en')
      
      // Document should have correct language attributes
      expect(document.documentElement.lang).toBe('en')
      expect(document.documentElement.dir).toBe('ltr')
    })
  })

  describe('Internationalization Integration', () => {
    it('should change language dynamically', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Find and click language selector
      const languageSelector = screen.getByRole('button', { name: /select language/i })
      fireEvent.click(languageSelector)

      // Should show language options
      await waitFor(() => {
        expect(screen.getByText('Español')).toBeInTheDocument()
      })

      // Click Spanish option
      fireEvent.click(screen.getByText('Español'))

      // Language should change
      await waitFor(() => {
        expect(i18n.language).toBe('es')
      })
    })

    it('should handle RTL language switching', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Change to Arabic (RTL language)
      await i18n.changeLanguage('ar')

      await waitFor(() => {
        expect(document.documentElement.dir).toBe('rtl')
        expect(document.documentElement.lang).toBe('ar')
      })

      // Change back to English (LTR)
      await i18n.changeLanguage('en')

      await waitFor(() => {
        expect(document.documentElement.dir).toBe('ltr')
        expect(document.documentElement.lang).toBe('en')
      })
    })

    it('should persist language choice', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      await i18n.changeLanguage('de')

      // Should save to localStorage
      expect(localStorageMock.setItem).toHaveBeenCalledWith(
        expect.stringContaining('language'),
        'de'
      )
    })

    it('should translate UI elements correctly', async () => {
      const { rerender } = render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Navigate to dashboard
      fireEvent.click(screen.getByRole('link', { name: /dashboard/i }))

      // Should show English dashboard
      await waitFor(() => {
        expect(screen.getByText(/dashboard/i)).toBeInTheDocument()
      })

      // Change to Spanish
      await i18n.changeLanguage('es')
      
      rerender(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Should show Spanish dashboard
      await waitFor(() => {
        expect(screen.getByText(/panel de control/i)).toBeInTheDocument()
      })
    })
  })

  describe('Accessibility Integration', () => {
    it('should provide skip links for keyboard navigation', () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Should have skip to main content link
      expect(screen.getByText(/skip to main content/i)).toBeInTheDocument()
    })

    it('should support keyboard navigation', () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Tab to first focusable element
      const firstButton = screen.getByTestId('vr-button')
      firstButton.focus()

      expect(document.activeElement).toBe(firstButton)

      // Should show focus styles for keyboard users
      fireEvent.keyDown(document.body, { key: 'Tab' })
      expect(document.body).toHaveClass('keyboard-user')
    })

    it('should announce screen reader messages', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Check for screen reader announcement region
      expect(screen.getByRole('status')).toBeInTheDocument()
    })

    it('should handle high contrast mode', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Simulate high contrast preference
      Object.defineProperty(window, 'matchMedia', {
        value: vi.fn().mockImplementation((query) => ({
          matches: query === '(prefers-contrast: high)',
          media: query,
          onchange: null,
          addListener: vi.fn(),
          removeListener: vi.fn(),
        })),
      })

      // Should apply high contrast styles
      const root = document.documentElement
      expect(root.classList.contains('high-contrast') || 
             getComputedStyle(root).getPropertyValue('--bg-primary')).toBeTruthy()
    })
  })

  describe('Compliance Integration', () => {
    it('should show consent banner for new users', async () => {
      localStorageMock.getItem.mockReturnValue(null) // No stored consent

      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Should show consent banner
      await waitFor(() => {
        expect(screen.getByRole('dialog', { name: /consent/i })).toBeInTheDocument()
      })

      expect(screen.getByText(/accept all/i)).toBeInTheDocument()
      expect(screen.getByText(/necessary only/i)).toBeInTheDocument()
      expect(screen.getByText(/customize/i)).toBeInTheDocument()
    })

    it('should handle consent acceptance', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Wait for consent banner and accept all
      await waitFor(() => {
        expect(screen.getByText(/accept all/i)).toBeInTheDocument()
      })

      fireEvent.click(screen.getByText(/accept all/i))

      // Banner should disappear
      await waitFor(() => {
        expect(screen.queryByRole('dialog', { name: /consent/i })).not.toBeInTheDocument()
      })

      // Should show consent management button
      expect(screen.getByRole('button', { name: /consent/i })).toBeInTheDocument()
    })

    it('should handle consent customization', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Wait for consent banner and click customize
      await waitFor(() => {
        expect(screen.getByText(/customize/i)).toBeInTheDocument()
      })

      fireEvent.click(screen.getByText(/customize/i))

      // Should show detailed consent modal
      await waitFor(() => {
        expect(screen.getByText(/strictly necessary cookies/i)).toBeInTheDocument()
        expect(screen.getByText(/analytics cookies/i)).toBeInTheDocument()
        expect(screen.getByText(/marketing cookies/i)).toBeInTheDocument()
      })

      // Should have checkboxes for each category
      const analyticsCheckbox = screen.getByLabelText(/analytics/i)
      expect(analyticsCheckbox).toBeInTheDocument()
      expect(analyticsCheckbox).toHaveProperty('type', 'checkbox')
    })

    it('should record data processing activities', async () => {
      const sessionId = 'test-session-123'
      
      // Mock session ID
      localStorageMock.getItem.mockImplementation((key) => {
        if (key === 'session_id') return sessionId
        return null
      })

      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Record some data processing
      const recordId = complianceManager.recordDataProcessing({
        sessionId,
        dataType: 'functional',
        purpose: 'ui_preferences',
        legalBasis: 'legitimate_interest',
        retentionPeriod: 30 * 24 * 60 * 60 * 1000,
        isAnonymized: false,
        processingLocation: 'EU'
      })

      expect(recordId).toBeDefined()
      expect(typeof recordId).toBe('string')

      // Should be able to retrieve the record
      const records = complianceManager.getDataProcessingRecords({ sessionId })
      expect(records).toHaveLength(1)
      expect(records[0].purpose).toBe('ui_preferences')
    })

    it('should handle GDPR data export request', async () => {
      const sessionId = 'test-session-123'
      
      localStorageMock.getItem.mockImplementation((key) => {
        if (key === 'session_id') return sessionId
        return null
      })

      // Record consent and data processing
      const consent = await complianceManager.requestConsent(sessionId)
      complianceManager.recordDataProcessing({
        sessionId,
        dataType: 'personal',
        purpose: 'user_profile',
        legalBasis: 'consent',
        retentionPeriod: 365 * 24 * 60 * 60 * 1000,
        isAnonymized: false,
        processingLocation: 'EU'
      })

      // Export user data
      const exportData = await complianceManager.exportUserData({ sessionId })

      expect(exportData.format).toBe('json')
      expect(exportData.consent).toBeDefined()
      expect(exportData.processingRecords).toHaveLength(1)
      expect(exportData.personalData).toBeDefined()
      expect(exportData.exportDate).toBeDefined()
    })

    it('should handle GDPR data erasure request', async () => {
      const sessionId = 'test-session-123'
      
      localStorageMock.getItem.mockImplementation((key) => {
        if (key === 'session_id') return sessionId
        return null
      })

      // Record consent and data processing
      const consent = await complianceManager.requestConsent(sessionId)
      complianceManager.recordDataProcessing({
        sessionId,
        dataType: 'personal',
        purpose: 'user_profile',
        legalBasis: 'consent',
        retentionPeriod: 365 * 24 * 60 * 60 * 1000,
        isAnonymized: false,
        processingLocation: 'EU'
      })

      // Request data erasure
      const result = await complianceManager.handleDataErasureRequest({ sessionId })

      expect(result.consentWithdrawn).toBe(true)
      expect(result.recordsDeleted).toBeGreaterThan(0)

      // Verify data is removed
      const remainingConsent = complianceManager.getConsent(sessionId)
      expect(remainingConsent).toBeNull()
    })
  })

  describe('Performance and Error Handling', () => {
    it('should handle initialization errors gracefully', () => {
      // Mock console.error to track errors
      const consoleError = vi.spyOn(console, 'error').mockImplementation(() => {})

      // Simulate localStorage error
      localStorageMock.getItem.mockImplementation(() => {
        throw new Error('localStorage error')
      })

      expect(() => {
        render(
          <TestWrapper>
            <App />
          </TestWrapper>
        )
      }).not.toThrow()

      consoleError.mockRestore()
    })

    it('should handle language switching errors gracefully', async () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Try to switch to an invalid language
      await expect(i18n.changeLanguage('invalid-lang')).resolves.not.toThrow()

      // Should fallback to a valid language
      expect(['en', 'es', 'fr', 'de', 'zh', 'ja', 'ar']).toContain(i18n.language)
    })

    it('should handle compliance errors gracefully', () => {
      const sessionId = 'test-session-123'

      // Try to update consent for non-existent session
      expect(() => {
        complianceManager.updateConsent('non-existent', { analytics: true })
      }).toThrow('No existing consent found')

      // Try to record invalid consent
      expect(() => {
        complianceManager.recordConsent({} as any)
      }).toThrow('Invalid consent data')
    })

    it('should perform well under normal usage', async () => {
      const startTime = performance.now()

      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Simulate typical user interactions
      await i18n.changeLanguage('es')
      await i18n.changeLanguage('en')

      const sessionId = 'perf-test-session'
      await complianceManager.requestConsent(sessionId)

      const endTime = performance.now()
      const totalTime = endTime - startTime

      // Should complete within reasonable time
      expect(totalTime).toBeLessThan(1000) // 1 second
    })
  })

  describe('Cross-Browser Compatibility', () => {
    it('should handle missing Web APIs gracefully', () => {
      // Mock missing Intl support
      const originalIntl = global.Intl
      delete (global as any).Intl

      expect(() => {
        render(
          <TestWrapper>
            <App />
          </TestWrapper>
        )
      }).not.toThrow()

      // Restore Intl
      global.Intl = originalIntl
    })

    it('should handle missing localStorage gracefully', () => {
      // Mock missing localStorage
      const originalLocalStorage = window.localStorage
      delete (window as any).localStorage

      expect(() => {
        render(
          <TestWrapper>
            <App />
          </TestWrapper>
        )
      }).not.toThrow()

      // Restore localStorage
      window.localStorage = originalLocalStorage
    })
  })

  describe('Mobile and Touch Support', () => {
    it('should adapt to mobile viewport', () => {
      // Mock mobile viewport
      Object.defineProperty(window, 'innerWidth', { value: 375 })
      Object.defineProperty(window, 'innerHeight', { value: 667 })

      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      // Should render mobile-friendly interface
      expect(screen.getByTestId('canvas')).toBeInTheDocument()
    })

    it('should handle touch interactions', () => {
      render(
        <TestWrapper>
          <App />
        </TestWrapper>
      )

      const button = screen.getByTestId('vr-button')

      // Simulate touch events
      fireEvent.touchStart(button)
      fireEvent.touchEnd(button)

      expect(button).toBeInTheDocument()
    })
  })
})

describe('End-to-End User Scenarios', () => {
  beforeEach(() => {
    vi.clearAllMocks()
    localStorageMock.getItem.mockReturnValue(null)
  })

  it('should handle complete first-time user journey', async () => {
    const { rerender } = render(
      <TestWrapper>
        <App />
      </TestWrapper>
    )

    // 1. User lands on site - sees consent banner
    await waitFor(() => {
      expect(screen.getByRole('dialog', { name: /consent/i })).toBeInTheDocument()
    })

    // 2. User customizes language to Spanish
    const languageSelector = screen.getByRole('button', { name: /select language/i })
    fireEvent.click(languageSelector)
    
    await waitFor(() => {
      expect(screen.getByText('Español')).toBeInTheDocument()
    })
    
    fireEvent.click(screen.getByText('Español'))

    // 3. User accepts necessary cookies only
    fireEvent.click(screen.getByText(/necessary only/i))

    // 4. User navigates to dashboard
    await waitFor(() => {
      expect(screen.queryByRole('dialog', { name: /consent/i })).not.toBeInTheDocument()
    })

    // Should show Spanish interface
    rerender(
      <TestWrapper>
        <App />
      </TestWrapper>
    )

    // 5. User enables accessibility features
    const body = document.body
    fireEvent.keyDown(body, { key: 'Tab' })
    expect(body).toHaveClass('keyboard-user')

    // 6. User manages consent preferences
    const consentButton = screen.getByRole('button', { name: /consent/i })
    fireEvent.click(consentButton)

    await waitFor(() => {
      expect(screen.getByText(/strictly necessary cookies/i)).toBeInTheDocument()
    })
  })

  it('should handle returning user with stored preferences', async () => {
    // Mock stored preferences
    localStorageMock.getItem.mockImplementation((key) => {
      switch (key) {
        case 'session_id':
          return 'returning-user-session'
        case 'xr-swarm-language':
          return 'de'
        case 'compliance_consents':
          return JSON.stringify({
            'returning-user-session': {
              sessionId: 'returning-user-session',
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
          })
        case 'accessibility-settings':
          return JSON.stringify({
            highContrast: true,
            fontSize: 'large'
          })
        default:
          return null
      }
    })

    render(
      <TestWrapper>
        <App />
      </TestWrapper>
    )

    // Should not show consent banner (already consented)
    expect(screen.queryByRole('dialog', { name: /consent/i })).not.toBeInTheDocument()

    // Should show consent management button
    expect(screen.getByRole('button', { name: /consent/i })).toBeInTheDocument()

    // Should have German language
    await waitFor(() => {
      expect(i18n.language).toBe('de')
    })

    // Should have accessibility preferences applied
    expect(document.documentElement).toHaveClass('high-contrast')
  })

  it('should handle enterprise compliance audit scenario', async () => {
    const auditSessionId = 'audit-session-123'
    
    localStorageMock.getItem.mockImplementation((key) => {
      if (key === 'session_id') return auditSessionId
      return null
    })

    render(
      <TestWrapper>
        <App />
      </TestWrapper>
    )

    // 1. Record various data processing activities
    complianceManager.recordDataProcessing({
      sessionId: auditSessionId,
      dataType: 'personal',
      purpose: 'authentication',
      legalBasis: 'contract',
      retentionPeriod: 365 * 24 * 60 * 60 * 1000,
      isAnonymized: false,
      processingLocation: 'EU'
    })

    complianceManager.recordDataProcessing({
      sessionId: auditSessionId,
      dataType: 'analytics',
      purpose: 'performance_monitoring',
      legalBasis: 'legitimate_interest',
      retentionPeriod: 180 * 24 * 60 * 60 * 1000,
      isAnonymized: true,
      processingLocation: 'EU'
    })

    // 2. Run compliance validation
    const validation = complianceManager.validateCompliance()
    expect(validation).toHaveProperty('isCompliant')
    expect(validation).toHaveProperty('issues')
    expect(validation).toHaveProperty('recommendations')

    // 3. Generate audit report
    const records = complianceManager.getDataProcessingRecords({
      sessionId: auditSessionId
    })
    
    expect(records).toHaveLength(2)
    expect(records.every(record => record.processingLocation === 'EU')).toBe(true)

    // 4. Export data for compliance review
    const exportData = await complianceManager.exportUserData({
      sessionId: auditSessionId
    })
    
    expect(exportData.processingRecords).toHaveLength(2)
    expect(exportData.format).toBe('json')
  })
})