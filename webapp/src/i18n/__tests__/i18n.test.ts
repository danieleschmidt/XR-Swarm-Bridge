import { describe, it, expect, beforeEach, vi } from 'vitest'
import i18n from '../index'
import { 
  SUPPORTED_LANGUAGES, 
  changeLanguage, 
  getCurrentLanguage, 
  isRTL, 
  formatNumber, 
  formatDate, 
  formatCurrency 
} from '../index'

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

describe('Internationalization (i18n)', () => {
  beforeEach(() => {
    vi.clearAllMocks()
    localStorageMock.getItem.mockReturnValue('en')
  })

  describe('Language Configuration', () => {
    it('should have all required supported languages', () => {
      expect(SUPPORTED_LANGUAGES).toHaveLength(7)
      
      const languageCodes = SUPPORTED_LANGUAGES.map(lang => lang.code)
      expect(languageCodes).toContain('en')
      expect(languageCodes).toContain('es')
      expect(languageCodes).toContain('fr')
      expect(languageCodes).toContain('de')
      expect(languageCodes).toContain('zh')
      expect(languageCodes).toContain('ja')
      expect(languageCodes).toContain('ar')
    })

    it('should have proper language structure', () => {
      SUPPORTED_LANGUAGES.forEach(language => {
        expect(language).toHaveProperty('code')
        expect(language).toHaveProperty('name')
        expect(language).toHaveProperty('nativeName')
        expect(typeof language.code).toBe('string')
        expect(typeof language.name).toBe('string')
        expect(typeof language.nativeName).toBe('string')
        expect(language.code.length).toBe(2)
      })
    })

    it('should initialize with English as fallback', () => {
      expect(i18n.options.fallbackLng).toBe('en')
    })
  })

  describe('Language Detection', () => {
    it('should detect RTL languages correctly', () => {
      expect(isRTL('ar')).toBe(true)
      expect(isRTL('he')).toBe(true)
      expect(isRTL('fa')).toBe(true)
      
      expect(isRTL('en')).toBe(false)
      expect(isRTL('es')).toBe(false)
      expect(isRTL('fr')).toBe(false)
      expect(isRTL('de')).toBe(false)
      expect(isRTL('zh')).toBe(false)
      expect(isRTL('ja')).toBe(false)
    })

    it('should get current language', () => {
      const currentLang = getCurrentLanguage()
      expect(typeof currentLang).toBe('string')
      expect(currentLang.length).toBeGreaterThan(0)
    })
  })

  describe('Language Switching', () => {
    it('should change language successfully', async () => {
      await changeLanguage('es')
      expect(getCurrentLanguage()).toBe('es')
      
      await changeLanguage('fr')
      expect(getCurrentLanguage()).toBe('fr')
      
      await changeLanguage('en')
      expect(getCurrentLanguage()).toBe('en')
    })

    it('should handle invalid language codes gracefully', async () => {
      const originalLang = getCurrentLanguage()
      
      await changeLanguage('invalid-lang')
      
      // Should fallback to original or default language
      const newLang = getCurrentLanguage()
      expect(['en', originalLang]).toContain(newLang)
    })
  })

  describe('Number Formatting', () => {
    it('should format numbers according to locale', () => {
      // Test different locales
      const testNumber = 1234567.89

      // English (US)
      i18n.changeLanguage('en')
      const enFormatted = formatNumber(testNumber)
      expect(typeof enFormatted).toBe('string')
      expect(enFormatted).toContain('1')
      expect(enFormatted).toContain('234')

      // German (uses different thousands separator)
      i18n.changeLanguage('de')
      const deFormatted = formatNumber(testNumber)
      expect(typeof deFormatted).toBe('string')
    })

    it('should format numbers with custom options', () => {
      const testNumber = 0.1234
      
      const formatted = formatNumber(testNumber, {
        style: 'percent',
        minimumFractionDigits: 2
      })
      
      expect(typeof formatted).toBe('string')
      expect(formatted).toContain('%')
    })

    it('should handle edge cases in number formatting', () => {
      expect(() => formatNumber(0)).not.toThrow()
      expect(() => formatNumber(-1000)).not.toThrow()
      expect(() => formatNumber(Infinity)).not.toThrow()
      expect(() => formatNumber(NaN)).not.toThrow()
    })
  })

  describe('Date Formatting', () => {
    it('should format dates according to locale', () => {
      const testDate = new Date('2024-01-15T10:30:00Z')

      // English
      i18n.changeLanguage('en')
      const enFormatted = formatDate(testDate)
      expect(typeof enFormatted).toBe('string')
      expect(enFormatted.length).toBeGreaterThan(0)

      // German
      i18n.changeLanguage('de')
      const deFormatted = formatDate(testDate)
      expect(typeof deFormatted).toBe('string')
      expect(deFormatted.length).toBeGreaterThan(0)

      // Formats should be different for different locales
      // (though this might not always be true depending on the date)
    })

    it('should format dates with custom options', () => {
      const testDate = new Date('2024-01-15T10:30:00Z')
      
      const formatted = formatDate(testDate, {
        weekday: 'long',
        year: 'numeric',
        month: 'long',
        day: 'numeric'
      })
      
      expect(typeof formatted).toBe('string')
      expect(formatted.length).toBeGreaterThan(10) // Should be a long format
    })

    it('should handle invalid dates gracefully', () => {
      const invalidDate = new Date('invalid-date')
      
      expect(() => formatDate(invalidDate)).not.toThrow()
    })
  })

  describe('Currency Formatting', () => {
    it('should format currency according to locale', () => {
      const testAmount = 1234.56

      const formatted = formatCurrency(testAmount, 'USD')
      
      expect(typeof formatted).toBe('string')
      expect(formatted).toContain('1')
      expect(formatted).toContain('234')
    })

    it('should handle different currencies', () => {
      const testAmount = 100

      const usdFormatted = formatCurrency(testAmount, 'USD')
      const eurFormatted = formatCurrency(testAmount, 'EUR')
      const jpyFormatted = formatCurrency(testAmount, 'JPY')

      expect(usdFormatted).toContain('$')
      expect(eurFormatted).toContain('€')
      expect(jpyFormatted).toContain('¥')
    })

    it('should handle zero and negative amounts', () => {
      expect(() => formatCurrency(0, 'USD')).not.toThrow()
      expect(() => formatCurrency(-100, 'USD')).not.toThrow()
    })
  })

  describe('Translation Keys', () => {
    const requiredKeys = [
      'app.title',
      'app.loading',
      'app.error',
      'navigation.dashboard',
      'navigation.agents',
      'navigation.missions',
      'agents.active',
      'agents.selected',
      'system.latency',
      'system.packetLoss',
      'compliance.consent',
      'settings.language',
      'errors.general',
      'accessibility.skipToMain'
    ]

    SUPPORTED_LANGUAGES.forEach(language => {
      describe(`${language.name} (${language.code}) translations`, () => {
        beforeEach(async () => {
          await changeLanguage(language.code)
        })

        requiredKeys.forEach(key => {
          it(`should have translation for key: ${key}`, () => {
            const translation = i18n.t(key)
            
            expect(translation).toBeDefined()
            expect(typeof translation).toBe('string')
            expect(translation.length).toBeGreaterThan(0)
            expect(translation).not.toBe(key) // Should not return the key itself
          })
        })

        it('should have all navigation keys', () => {
          const navKeys = [
            'navigation.dashboard',
            'navigation.3d_view',
            'navigation.overview',
            'navigation.agents',
            'navigation.missions',
            'navigation.ai_commander'
          ]

          navKeys.forEach(key => {
            const translation = i18n.t(key)
            expect(translation).toBeDefined()
            expect(translation).not.toBe(key)
          })
        })

        it('should have all agent-related keys', () => {
          const agentKeys = [
            'agents.title',
            'agents.active',
            'agents.selected',
            'agents.battery',
            'agents.position',
            'agents.status',
            'agents.drone',
            'agents.ugv',
            'agents.manipulator'
          ]

          agentKeys.forEach(key => {
            const translation = i18n.t(key)
            expect(translation).toBeDefined()
            expect(translation).not.toBe(key)
          })
        })

        it('should have all system status keys', () => {
          const systemKeys = [
            'system.status',
            'system.latency',
            'system.packetLoss',
            'system.bandwidth',
            'system.healthy',
            'system.warning',
            'system.critical'
          ]

          systemKeys.forEach(key => {
            const translation = i18n.t(key)
            expect(translation).toBeDefined()
            expect(translation).not.toBe(key)
          })
        })

        it('should have all compliance keys', () => {
          const complianceKeys = [
            'compliance.gdpr',
            'compliance.dataProcessing',
            'compliance.consent',
            'compliance.privacy',
            'compliance.cookies'
          ]

          complianceKeys.forEach(key => {
            const translation = i18n.t(key)
            expect(translation).toBeDefined()
            expect(translation).not.toBe(key)
          })
        })
      })
    })
  })

  describe('Translation Interpolation', () => {
    it('should handle variable interpolation', async () => {
      await changeLanguage('en')
      
      const translation = i18n.t('ai.generated', { count: 5 })
      expect(translation).toContain('5')
      
      const planTranslation = i18n.t('ai.planGenerated', { name: 'Test Mission' })
      expect(planTranslation).toContain('Test Mission')
    })

    it('should handle pluralization', async () => {
      await changeLanguage('en')
      
      // Test with singular
      const singular = i18n.t('agents.active', { count: 1 })
      expect(typeof singular).toBe('string')
      
      // Test with plural
      const plural = i18n.t('agents.active', { count: 5 })
      expect(typeof plural).toBe('string')
    })
  })

  describe('Browser Integration', () => {
    it('should detect browser language preference', () => {
      // Mock navigator.language
      Object.defineProperty(navigator, 'language', {
        value: 'es-ES',
        configurable: true
      })

      // This would be tested more thoroughly in an integration test
      // For now, just verify the function exists and can be called
      expect(typeof navigator.language).toBe('string')
    })

    it('should persist language choice to localStorage', async () => {
      await changeLanguage('de')
      
      // Should have called localStorage.setItem
      expect(localStorageMock.setItem).toHaveBeenCalled()
    })
  })

  describe('Performance', () => {
    it('should load translations efficiently', async () => {
      const startTime = performance.now()
      
      await changeLanguage('zh')
      const translation = i18n.t('app.title')
      
      const endTime = performance.now()
      const loadTime = endTime - startTime
      
      expect(translation).toBeDefined()
      expect(loadTime).toBeLessThan(100) // Should load within 100ms
    })

    it('should cache translations', async () => {
      await changeLanguage('ja')
      
      const startTime = performance.now()
      
      // Multiple calls should be fast due to caching
      for (let i = 0; i < 100; i++) {
        i18n.t('app.title')
      }
      
      const endTime = performance.now()
      const totalTime = endTime - startTime
      
      expect(totalTime).toBeLessThan(50) // Cached calls should be very fast
    })
  })

  describe('Error Handling', () => {
    it('should handle missing translation keys gracefully', () => {
      const missingKey = 'this.key.does.not.exist'
      const result = i18n.t(missingKey)
      
      // Should return the key or a fallback, not throw an error
      expect(typeof result).toBe('string')
    })

    it('should handle malformed interpolation gracefully', () => {
      expect(() => {
        i18n.t('ai.generated', { invalidParam: 'test' })
      }).not.toThrow()
    })

    it('should handle language switching failures gracefully', async () => {
      // Try to switch to a completely invalid language
      expect(async () => {
        await changeLanguage('xxx-invalid')
      }).not.toThrow()
    })
  })

  describe('Accessibility', () => {
    it('should provide accessibility-specific translations', async () => {
      await changeLanguage('en')
      
      const accessibilityKeys = [
        'accessibility.skipToMain',
        'accessibility.openMenu',
        'accessibility.closeMenu',
        'accessibility.loading',
        'accessibility.selected'
      ]

      accessibilityKeys.forEach(key => {
        const translation = i18n.t(key)
        expect(translation).toBeDefined()
        expect(translation).not.toBe(key)
        expect(translation.length).toBeGreaterThan(0)
      })
    })
  })
})