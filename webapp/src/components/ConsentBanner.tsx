import React, { useState, useEffect } from 'react'
import { useTranslation } from 'react-i18next'
import { useCompliance } from '../utils/compliance'

interface ConsentBannerProps {
  className?: string
  position?: 'top' | 'bottom'
  theme?: 'light' | 'dark'
}

export default function ConsentBanner({ 
  className = '', 
  position = 'bottom',
  theme = 'dark'
}: ConsentBannerProps) {
  const { t } = useTranslation()
  const { requestConsent, getConsent, updateConsent, withdrawConsent } = useCompliance()
  
  const [showBanner, setShowBanner] = useState(false)
  const [showDetails, setShowDetails] = useState(false)
  const [consent, setConsent] = useState(getConsent())
  const [isLoading, setIsLoading] = useState(false)

  useEffect(() => {
    // Check if we need to show consent banner
    const checkConsentStatus = async () => {
      const currentConsent = getConsent()
      if (!currentConsent) {
        setShowBanner(true)
      } else {
        setConsent(currentConsent)
      }
    }

    checkConsentStatus()
  }, [getConsent])

  const handleAcceptAll = async () => {
    setIsLoading(true)
    try {
      const newConsent = await requestConsent({ 
        forceModal: false,
        categories: ['necessary', 'analytics', 'marketing', 'personalization', 'functional']
      })
      
      if (newConsent) {
        const updatedConsent = {
          ...newConsent,
          consents: {
            necessary: true,
            analytics: true,
            marketing: true,
            personalization: true,
            functional: true
          }
        }
        
        updateConsent(updatedConsent.consents)
        setConsent(updatedConsent)
        setShowBanner(false)
        setShowDetails(false)
      }
    } catch (error) {
      console.error('Failed to update consent:', error)
    } finally {
      setIsLoading(false)
    }
  }

  const handleAcceptNecessary = async () => {
    setIsLoading(true)
    try {
      const newConsent = await requestConsent({ 
        forceModal: false,
        categories: ['necessary']
      })
      
      if (newConsent) {
        const updatedConsent = {
          ...newConsent,
          consents: {
            necessary: true,
            analytics: false,
            marketing: false,
            personalization: false,
            functional: false
          }
        }
        
        updateConsent(updatedConsent.consents)
        setConsent(updatedConsent)
        setShowBanner(false)
        setShowDetails(false)
      }
    } catch (error) {
      console.error('Failed to update consent:', error)
    } finally {
      setIsLoading(false)
    }
  }

  const handleCustomizeConsent = () => {
    setShowDetails(true)
  }

  const handleSaveCustomConsent = async (customConsents: any) => {
    setIsLoading(true)
    try {
      updateConsent(customConsents)
      setShowBanner(false)
      setShowDetails(false)
    } catch (error) {
      console.error('Failed to save custom consent:', error)
    } finally {
      setIsLoading(false)
    }
  }

  const handleManageConsent = () => {
    setShowDetails(true)
    setShowBanner(true)
  }

  const themeClasses = {
    light: 'bg-white text-gray-900 border-gray-200',
    dark: 'bg-gray-900 text-white border-gray-700'
  }

  const positionClasses = {
    top: 'top-0',
    bottom: 'bottom-0'
  }

  if (!showBanner && !showDetails) {
    // Show manage consent button if consent has been given
    return consent ? (
      <button
        onClick={handleManageConsent}
        className="fixed bottom-4 right-4 z-40 px-3 py-2 bg-blue-600 text-white text-sm rounded-lg hover:bg-blue-700 transition-colors"
        aria-label={t('compliance.consent')}
      >
        🔒 {t('compliance.consent')}
      </button>
    ) : null
  }

  return (
    <>
      {/* Main Consent Banner */}
      {showBanner && !showDetails && (
        <div 
          className={`fixed left-0 right-0 z-50 p-4 border-t ${positionClasses[position]} ${themeClasses[theme]} ${className}`}
          role="dialog"
          aria-label={t('compliance.consent')}
          aria-describedby="consent-description"
        >
          <div className="max-w-6xl mx-auto">
            <div className="flex flex-col lg:flex-row lg:items-center lg:justify-between gap-4">
              <div className="flex-1">
                <p id="consent-description" className="text-sm">
                  {t('app.title')} uses cookies and similar technologies to provide you with the best experience, 
                  analyze site usage, and assist with marketing efforts. By clicking "Accept All", 
                  you consent to the use of all cookies. You can customize your preferences or decline non-essential cookies.
                </p>
                <div className="flex gap-2 mt-2 text-xs">
                  <a href="/privacy" className="text-blue-400 hover:text-blue-300 underline">
                    {t('compliance.privacy')}
                  </a>
                  <a href="/cookies" className="text-blue-400 hover:text-blue-300 underline">
                    {t('compliance.cookies')}
                  </a>
                </div>
              </div>
              
              <div className="flex flex-col sm:flex-row gap-2 lg:ml-4">
                <button
                  onClick={handleCustomizeConsent}
                  className="px-4 py-2 text-sm border border-gray-400 rounded hover:bg-gray-100 hover:text-gray-900 transition-colors"
                  disabled={isLoading}
                >
                  Customize
                </button>
                <button
                  onClick={handleAcceptNecessary}
                  className="px-4 py-2 text-sm bg-gray-600 text-white rounded hover:bg-gray-700 transition-colors"
                  disabled={isLoading}
                >
                  {isLoading ? 'Processing...' : 'Necessary Only'}
                </button>
                <button
                  onClick={handleAcceptAll}
                  className="px-4 py-2 text-sm bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors"
                  disabled={isLoading}
                >
                  {isLoading ? 'Processing...' : 'Accept All'}
                </button>
              </div>
            </div>
          </div>
        </div>
      )}

      {/* Detailed Consent Modal */}
      {showDetails && (
        <ConsentDetailsModal
          currentConsent={consent}
          onSave={handleSaveCustomConsent}
          onClose={() => {
            setShowDetails(false)
            if (!consent) setShowBanner(true)
          }}
          isLoading={isLoading}
        />
      )}
    </>
  )
}

interface ConsentDetailsModalProps {
  currentConsent: any
  onSave: (consents: any) => void
  onClose: () => void
  isLoading: boolean
}

function ConsentDetailsModal({ currentConsent, onSave, onClose, isLoading }: ConsentDetailsModalProps) {
  const { t } = useTranslation()
  const [consents, setConsents] = useState({
    necessary: true,
    analytics: currentConsent?.consents?.analytics || false,
    marketing: currentConsent?.consents?.marketing || false,
    personalization: currentConsent?.consents?.personalization || false,
    functional: currentConsent?.consents?.functional || false
  })

  const consentCategories = [
    {
      key: 'necessary',
      title: 'Strictly Necessary Cookies',
      description: 'These cookies are necessary for the website to function and cannot be disabled.',
      required: true,
      examples: 'Session management, security, load balancing'
    },
    {
      key: 'functional',
      title: 'Functional Cookies',
      description: 'These cookies enable enhanced functionality and personalization.',
      required: false,
      examples: 'Language preferences, region settings, accessibility options'
    },
    {
      key: 'analytics',
      title: 'Analytics Cookies',
      description: 'These cookies help us understand how visitors interact with our website.',
      required: false,
      examples: 'Google Analytics, usage statistics, performance monitoring'
    },
    {
      key: 'personalization',
      title: 'Personalization Cookies',
      description: 'These cookies allow us to personalize content and offers.',
      required: false,
      examples: 'Content recommendations, user preferences, customized experience'
    },
    {
      key: 'marketing',
      title: 'Marketing Cookies',
      description: 'These cookies are used to track visitors across websites for marketing purposes.',
      required: false,
      examples: 'Ad targeting, social media integration, conversion tracking'
    }
  ]

  const handleConsentChange = (category: string, value: boolean) => {
    if (category === 'necessary') return // Cannot disable necessary cookies
    setConsents(prev => ({ ...prev, [category]: value }))
  }

  const handleSave = () => {
    onSave(consents)
  }

  const handleAcceptAll = () => {
    const allConsents = Object.fromEntries(
      consentCategories.map(cat => [cat.key, true])
    )
    setConsents(allConsents)
    onSave(allConsents)
  }

  const handleRejectAll = () => {
    const necessaryOnly = Object.fromEntries(
      consentCategories.map(cat => [cat.key, cat.required])
    )
    setConsents(necessaryOnly)
    onSave(necessaryOnly)
  }

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 z-50 flex items-center justify-center p-4">
      <div className="bg-white rounded-lg max-w-2xl w-full max-h-[90vh] overflow-y-auto">
        <div className="p-6">
          <div className="flex justify-between items-center mb-6">
            <h2 className="text-2xl font-bold text-gray-900">{t('compliance.consent')}</h2>
            <button
              onClick={onClose}
              className="text-gray-400 hover:text-gray-600"
              aria-label={t('app.close')}
            >
              <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
              </svg>
            </button>
          </div>

          <p className="text-gray-600 mb-6">
            We use cookies and similar technologies to provide, protect and improve our services. 
            You can choose which categories of cookies you allow:
          </p>

          <div className="space-y-4 mb-6">
            {consentCategories.map(category => (
              <div key={category.key} className="border border-gray-200 rounded-lg p-4">
                <div className="flex items-start justify-between">
                  <div className="flex-1">
                    <div className="flex items-center space-x-3 mb-2">
                      <h3 className="font-semibold text-gray-900">{category.title}</h3>
                      {category.required && (
                        <span className="px-2 py-1 text-xs bg-red-100 text-red-800 rounded">
                          Required
                        </span>
                      )}
                    </div>
                    <p className="text-sm text-gray-600 mb-2">{category.description}</p>
                    <p className="text-xs text-gray-500">
                      <strong>Examples:</strong> {category.examples}
                    </p>
                  </div>
                  <div className="ml-4">
                    <label className="flex items-center">
                      <input
                        type="checkbox"
                        checked={consents[category.key as keyof typeof consents]}
                        onChange={(e) => handleConsentChange(category.key, e.target.checked)}
                        disabled={category.required}
                        className="w-5 h-5 text-blue-600 rounded border-gray-300 focus:ring-blue-500"
                      />
                      <span className="sr-only">
                        {category.required ? 'Required' : `Allow ${category.title}`}
                      </span>
                    </label>
                  </div>
                </div>
              </div>
            ))}
          </div>

          <div className="flex flex-col sm:flex-row gap-3 justify-end">
            <button
              onClick={handleRejectAll}
              className="px-4 py-2 text-sm border border-gray-300 text-gray-700 rounded hover:bg-gray-50 transition-colors"
              disabled={isLoading}
            >
              Necessary Only
            </button>
            <button
              onClick={handleAcceptAll}
              className="px-4 py-2 text-sm bg-gray-600 text-white rounded hover:bg-gray-700 transition-colors"
              disabled={isLoading}
            >
              Accept All
            </button>
            <button
              onClick={handleSave}
              className="px-4 py-2 text-sm bg-blue-600 text-white rounded hover:bg-blue-700 transition-colors"
              disabled={isLoading}
            >
              {isLoading ? 'Saving...' : 'Save Preferences'}
            </button>
          </div>

          <div className="mt-6 pt-4 border-t border-gray-200">
            <div className="flex flex-wrap gap-4 text-sm text-gray-500">
              <a href="/privacy" className="hover:text-blue-600 underline">
                Privacy Policy
              </a>
              <a href="/cookies" className="hover:text-blue-600 underline">
                Cookie Policy
              </a>
              <a href="/terms" className="hover:text-blue-600 underline">
                Terms of Service
              </a>
              <button 
                onClick={() => window.open('/data-export', '_blank')}
                className="hover:text-blue-600 underline"
              >
                Export My Data
              </button>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

// Consent status indicator component
export function ConsentStatus() {
  const { getConsent } = useCompliance()
  const consent = getConsent()
  const { t } = useTranslation()

  if (!consent) return null

  const activeCategories = Object.entries(consent.consents)
    .filter(([_, enabled]) => enabled)
    .length

  return (
    <div className="flex items-center space-x-2 text-sm text-gray-600">
      <svg className="w-4 h-4 text-green-600" fill="currentColor" viewBox="0 0 20 20">
        <path fillRule="evenodd" d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z" clipRule="evenodd" />
      </svg>
      <span>
        {activeCategories} cookie categories active
      </span>
    </div>
  )
}