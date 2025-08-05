import React, { useState } from 'react'
import { useTranslation } from 'react-i18next'
import { SUPPORTED_LANGUAGES, changeLanguage, getCurrentLanguage, isRTL } from '../i18n'

interface LanguageSelectorProps {
  className?: string
  showFlags?: boolean
  showNativeNames?: boolean
  variant?: 'dropdown' | 'buttons' | 'modal'
}

export default function LanguageSelector({
  className = '',
  showFlags = true,
  showNativeNames = true,
  variant = 'dropdown'
}: LanguageSelectorProps) {
  const { t } = useTranslation()
  const [isOpen, setIsOpen] = useState(false)
  const currentLang = getCurrentLanguage()

  const handleLanguageChange = async (langCode: string) => {
    try {
      await changeLanguage(langCode)
      
      // Update document direction for RTL languages
      document.documentElement.dir = isRTL(langCode) ? 'rtl' : 'ltr'
      document.documentElement.lang = langCode
      
      // Close dropdown
      setIsOpen(false)
      
      // Optional: Reload page to ensure all components update
      // window.location.reload()
    } catch (error) {
      console.error('Failed to change language:', error)
    }
  }

  const currentLanguage = SUPPORTED_LANGUAGES.find(lang => lang.code === currentLang)

  if (variant === 'buttons') {
    return (
      <div className={`flex flex-wrap gap-2 ${className}`}>
        {SUPPORTED_LANGUAGES.map(language => (
          <button
            key={language.code}
            onClick={() => handleLanguageChange(language.code)}
            className={`px-3 py-1 rounded text-sm transition-colors ${
              currentLang === language.code
                ? 'bg-blue-600 text-white'
                : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
            }`}
            aria-label={`${t('settings.language')}: ${language.name}`}
            title={language.nativeName}
          >
            {showFlags && <span className="mr-1">{getFlagEmoji(language.code)}</span>}
            {showNativeNames ? language.nativeName : language.name}
          </button>
        ))}
      </div>
    )
  }

  if (variant === 'modal') {
    return (
      <>
        <button
          onClick={() => setIsOpen(true)}
          className={`flex items-center space-x-2 px-3 py-2 rounded hover:bg-gray-100 ${className}`}
          aria-label={t('settings.language')}
        >
          {showFlags && <span>{getFlagEmoji(currentLang)}</span>}
          <span>{showNativeNames ? currentLanguage?.nativeName : currentLanguage?.name}</span>
          <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
          </svg>
        </button>

        {isOpen && (
          <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
            <div className="bg-white rounded-lg p-6 max-w-md w-full mx-4 max-h-96 overflow-y-auto">
              <div className="flex justify-between items-center mb-4">
                <h3 className="text-lg font-semibold">{t('settings.language')}</h3>
                <button
                  onClick={() => setIsOpen(false)}
                  className="text-gray-400 hover:text-gray-600"
                  aria-label={t('app.close')}
                >
                  <svg className="w-6 h-6" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                  </svg>
                </button>
              </div>
              
              <div className="space-y-2">
                {SUPPORTED_LANGUAGES.map(language => (
                  <button
                    key={language.code}
                    onClick={() => handleLanguageChange(language.code)}
                    className={`w-full flex items-center space-x-3 px-3 py-2 rounded hover:bg-gray-100 text-left ${
                      currentLang === language.code ? 'bg-blue-50 text-blue-700' : ''
                    }`}
                  >
                    {showFlags && <span className="text-xl">{getFlagEmoji(language.code)}</span>}
                    <div>
                      <div className="font-medium">{language.nativeName}</div>
                      <div className="text-sm text-gray-500">{language.name}</div>
                    </div>
                    {currentLang === language.code && (
                      <div className="ml-auto">
                        <svg className="w-5 h-5 text-blue-600" fill="currentColor" viewBox="0 0 20 20">
                          <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                        </svg>
                      </div>
                    )}
                  </button>
                ))}
              </div>
            </div>
          </div>
        )}
      </>
    )
  }

  // Default dropdown variant
  return (
    <div className={`relative ${className}`}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="flex items-center space-x-2 px-3 py-2 border border-gray-300 rounded-md hover:border-gray-400 focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-transparent"
        aria-label={t('settings.language')}
        aria-expanded={isOpen}
        aria-haspopup="listbox"
      >
        {showFlags && <span>{getFlagEmoji(currentLang)}</span>}
        <span>{showNativeNames ? currentLanguage?.nativeName : currentLanguage?.name}</span>
        <svg 
          className={`w-4 h-4 transition-transform ${isOpen ? 'rotate-180' : ''}`} 
          fill="none" 
          stroke="currentColor" 
          viewBox="0 0 24 24"
        >
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
        </svg>
      </button>

      {isOpen && (
        <div className="absolute top-full mt-1 w-64 bg-white border border-gray-300 rounded-md shadow-lg z-50 max-h-64 overflow-y-auto">
          {SUPPORTED_LANGUAGES.map(language => (
            <button
              key={language.code}
              onClick={() => handleLanguageChange(language.code)}
              className={`w-full flex items-center space-x-3 px-4 py-2 hover:bg-gray-100 text-left ${
                currentLang === language.code ? 'bg-blue-50 text-blue-700' : ''
              }`}
              role="option"
              aria-selected={currentLang === language.code}
            >
              {showFlags && <span className="text-lg">{getFlagEmoji(language.code)}</span>}
              <div className="flex-1">
                <div className="font-medium">{language.nativeName}</div>
                <div className="text-sm text-gray-500">{language.name}</div>
              </div>
              {currentLang === language.code && (
                <svg className="w-5 h-5 text-blue-600" fill="currentColor" viewBox="0 0 20 20">
                  <path fillRule="evenodd" d="M16.707 5.293a1 1 0 010 1.414l-8 8a1 1 0 01-1.414 0l-4-4a1 1 0 011.414-1.414L8 12.586l7.293-7.293a1 1 0 011.414 0z" clipRule="evenodd" />
                </svg>
              )}
            </button>
          ))}
        </div>
      )}

      {/* Click outside to close */}
      {isOpen && (
        <div 
          className="fixed inset-0 z-40"
          onClick={() => setIsOpen(false)}
          aria-hidden="true"
        />
      )}
    </div>
  )
}

// Helper function to get flag emoji for language
function getFlagEmoji(langCode: string): string {
  const flagMap: Record<string, string> = {
    en: '🇺🇸', // English (US)
    es: '🇪🇸', // Spanish (Spain)
    fr: '🇫🇷', // French (France)
    de: '🇩🇪', // German (Germany)
    zh: '🇨🇳', // Chinese (China)
    ja: '🇯🇵', // Japanese (Japan)
    ar: '🇸🇦', // Arabic (Saudi Arabia)
  }
  
  return flagMap[langCode] || '🌐'
}

// Compact language selector for mobile/small spaces
export function CompactLanguageSelector({ className = '' }: { className?: string }) {
  const currentLang = getCurrentLanguage()
  const [isOpen, setIsOpen] = useState(false)

  return (
    <div className={`relative ${className}`}>
      <button
        onClick={() => setIsOpen(!isOpen)}
        className="w-10 h-10 rounded-full bg-gray-100 hover:bg-gray-200 flex items-center justify-center"
        aria-label="Select language"
      >
        {getFlagEmoji(currentLang)}
      </button>

      {isOpen && (
        <div className="absolute top-full mt-2 right-0 bg-white border border-gray-300 rounded-md shadow-lg z-50 py-1">
          {SUPPORTED_LANGUAGES.map(language => (
            <button
              key={language.code}
              onClick={() => {
                changeLanguage(language.code)
                setIsOpen(false)
              }}
              className={`flex items-center space-x-2 px-3 py-2 hover:bg-gray-100 whitespace-nowrap ${
                currentLang === language.code ? 'bg-blue-50' : ''
              }`}
            >
              <span>{getFlagEmoji(language.code)}</span>
              <span className="text-sm">{language.nativeName}</span>
            </button>
          ))}
        </div>
      )}

      {isOpen && (
        <div 
          className="fixed inset-0 z-40"
          onClick={() => setIsOpen(false)}
          aria-hidden="true"
        />
      )}
    </div>
  )
}

// Hook for using language in components
export function useLanguage() {
  const { i18n } = useTranslation()
  
  return {
    currentLanguage: getCurrentLanguage(),
    changeLanguage,
    isRTL: isRTL(),
    supportedLanguages: SUPPORTED_LANGUAGES,
    direction: isRTL() ? 'rtl' : 'ltr'
  }
}