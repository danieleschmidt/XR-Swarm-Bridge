import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react'
import { useTranslation } from 'react-i18next'

interface AccessibilitySettings {
  // Visual
  highContrast: boolean
  reducedMotion: boolean
  fontSize: 'small' | 'medium' | 'large' | 'extra-large'
  colorBlindMode: 'none' | 'protanopia' | 'deuteranopia' | 'tritanopia'
  
  // Motor
  stickyKeys: boolean
  mouseKeys: boolean
  focusTraps: boolean
  
  // Cognitive
  reducedAnimations: boolean
  simplifiedUI: boolean
  autoplayDisabled: boolean
  
  // Audio
  soundEnabled: boolean
  screenReaderOptimized: boolean
  
  // Keyboard
  keyboardNavigation: boolean
  skipLinks: boolean
  tabIndex: boolean
}

interface AccessibilityContextType {
  settings: AccessibilitySettings
  updateSetting: <K extends keyof AccessibilitySettings>(
    key: K, 
    value: AccessibilitySettings[K]
  ) => void
  resetSettings: () => void
  announceToScreenReader: (message: string, priority?: 'polite' | 'assertive') => void
  focusElement: (selector: string) => void
  isReducedMotion: boolean
  isHighContrast: boolean
}

const defaultSettings: AccessibilitySettings = {
  highContrast: false,
  reducedMotion: false,
  fontSize: 'medium',
  colorBlindMode: 'none',
  stickyKeys: false,
  mouseKeys: false,
  focusTraps: true,
  reducedAnimations: false,
  simplifiedUI: false,
  autoplayDisabled: false,
  soundEnabled: true,
  screenReaderOptimized: false,
  keyboardNavigation: true,
  skipLinks: true,
  tabIndex: true
}

const AccessibilityContext = createContext<AccessibilityContextType | null>(null)

interface AccessibilityProviderProps {
  children: ReactNode
}

export function AccessibilityProvider({ children }: AccessibilityProviderProps) {
  const [settings, setSettings] = useState<AccessibilitySettings>(defaultSettings)
  const { t } = useTranslation()

  // Load settings from localStorage on mount
  useEffect(() => {
    const stored = localStorage.getItem('accessibility-settings')
    if (stored) {
      try {
        const parsedSettings = JSON.parse(stored)
        setSettings({ ...defaultSettings, ...parsedSettings })
      } catch (error) {
        console.warn('Failed to parse accessibility settings:', error)
      }
    }

    // Detect system preferences
    detectSystemPreferences()
  }, [])

  // Save settings to localStorage whenever they change
  useEffect(() => {
    localStorage.setItem('accessibility-settings', JSON.stringify(settings))
    applyAccessibilitySettings(settings)
  }, [settings])

  const detectSystemPreferences = () => {
    // Detect reduced motion preference
    if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
      setSettings(prev => ({ ...prev, reducedMotion: true, reducedAnimations: true }))
    }

    // Detect high contrast preference
    if (window.matchMedia('(prefers-contrast: high)').matches) {
      setSettings(prev => ({ ...prev, highContrast: true }))
    }

    // Detect color scheme preference
    if (window.matchMedia('(prefers-color-scheme: dark)').matches) {
      document.documentElement.classList.add('dark')
    }
  }

  const updateSetting = <K extends keyof AccessibilitySettings>(
    key: K, 
    value: AccessibilitySettings[K]
  ) => {
    setSettings(prev => ({ ...prev, [key]: value }))
  }

  const resetSettings = () => {
    setSettings(defaultSettings)
  }

  const announceToScreenReader = (message: string, priority: 'polite' | 'assertive' = 'polite') => {
    const announcement = document.createElement('div')
    announcement.setAttribute('aria-live', priority)
    announcement.setAttribute('aria-atomic', 'true')
    announcement.className = 'sr-only'
    announcement.textContent = message
    
    document.body.appendChild(announcement)
    
    setTimeout(() => {
      document.body.removeChild(announcement)
    }, 1000)
  }

  const focusElement = (selector: string) => {
    const element = document.querySelector(selector) as HTMLElement
    if (element) {
      element.focus()
    }
  }

  const value: AccessibilityContextType = {
    settings,
    updateSetting,
    resetSettings,
    announceToScreenReader,
    focusElement,
    isReducedMotion: settings.reducedMotion,
    isHighContrast: settings.highContrast
  }

  return (
    <AccessibilityContext.Provider value={value}>
      <AccessibilityCSS settings={settings} />
      <ScreenReaderAnnouncements />
      <SkipLinks />
      <KeyboardNavigation />
      {children}
    </AccessibilityContext.Provider>
  )
}

// Apply accessibility settings to DOM
function applyAccessibilitySettings(settings: AccessibilitySettings) {
  const root = document.documentElement

  // Font size
  const fontSizeMap = {
    small: '14px',
    medium: '16px',
    large: '18px',
    'extra-large': '20px'
  }
  root.style.fontSize = fontSizeMap[settings.fontSize]

  // High contrast
  if (settings.highContrast) {
    root.classList.add('high-contrast')
  } else {
    root.classList.remove('high-contrast')
  }

  // Reduced motion
  if (settings.reducedMotion) {
    root.classList.add('reduced-motion')
  } else {
    root.classList.remove('reduced-motion')
  }

  // Color blind mode
  root.className = root.className.replace(/colorblind-\w+/g, '')
  if (settings.colorBlindMode !== 'none') {
    root.classList.add(`colorblind-${settings.colorBlindMode}`)
  }

  // Simplified UI
  if (settings.simplifiedUI) {
    root.classList.add('simplified-ui')
  } else {
    root.classList.remove('simplified-ui')
  }
}

// CSS styles for accessibility
function AccessibilityCSS({ settings }: { settings: AccessibilitySettings }) {
  return (
    <style>
      {`
        /* High Contrast Mode */
        .high-contrast {
          --bg-primary: #000000;
          --text-primary: #ffffff;
          --bg-secondary: #ffffff;
          --text-secondary: #000000;
          --border-color: #ffffff;
          --focus-color: #ffff00;
        }

        .high-contrast * {
          background-color: var(--bg-primary) !important;
          color: var(--text-primary) !important;
          border-color: var(--border-color) !important;
        }

        .high-contrast button,
        .high-contrast input,
        .high-contrast select,
        .high-contrast textarea {
          background-color: var(--bg-secondary) !important;
          color: var(--text-secondary) !important;
          border: 2px solid var(--border-color) !important;
        }

        .high-contrast *:focus {
          outline: 3px solid var(--focus-color) !important;
          outline-offset: 2px !important;
        }

        /* Reduced Motion */
        .reduced-motion *,
        .reduced-motion *::before,
        .reduced-motion *::after {
          animation-duration: 0.01ms !important;
          animation-iteration-count: 1 !important;
          transition-duration: 0.01ms !important;
          scroll-behavior: auto !important;
        }

        /* Color Blind Support */
        .colorblind-protanopia {
          filter: url('#protanopia-filter');
        }

        .colorblind-deuteranopia {
          filter: url('#deuteranopia-filter');
        }

        .colorblind-tritanopia {
          filter: url('#tritanopia-filter');
        }

        /* Simplified UI */
        .simplified-ui .shadow,
        .simplified-ui .rounded,
        .simplified-ui .gradient {
          box-shadow: none !important;
          border-radius: 0 !important;
          background-image: none !important;
        }

        .simplified-ui .animation,
        .simplified-ui .transition {
          animation: none !important;
          transition: none !important;
        }

        /* Screen Reader Only */
        .sr-only {
          position: absolute;
          width: 1px;
          height: 1px;
          padding: 0;
          margin: -1px;
          overflow: hidden;
          clip: rect(0, 0, 0, 0);
          white-space: nowrap;
          border: 0;
        }

        /* Focus Management */
        .focus-trap:focus {
          outline: 3px solid #0066cc !important;
          outline-offset: 2px !important;
        }

        /* Skip Links */
        .skip-link {
          position: absolute;
          top: -40px;
          left: 6px;
          background: #000;
          color: #fff;
          padding: 8px;
          z-index: 10000;
          text-decoration: none;
          border-radius: 0 0 4px 4px;
        }

        .skip-link:focus {
          top: 0;
        }

        /* Keyboard Navigation */
        .keyboard-user *:focus {
          outline: 2px solid #0066cc !important;
          outline-offset: 2px !important;
        }

        /* Color Blind SVG Filters */
        svg.accessibility-filters {
          position: absolute;
          width: 0;
          height: 0;
        }
      `}
    </style>
  )
}

// Screen reader announcements
function ScreenReaderAnnouncements() {
  return (
    <div aria-live="polite" aria-atomic="true" className="sr-only" id="screen-reader-announcements" />
  )
}

// Skip links for keyboard navigation
function SkipLinks() {
  const { t } = useTranslation()

  return (
    <>
      <a href="#main-content" className="skip-link">
        {t('accessibility.skipToMain')}
      </a>
      <a href="#navigation" className="skip-link">
        Skip to navigation
      </a>
    </>
  )
}

// Keyboard navigation enhancement
function KeyboardNavigation() {
  useEffect(() => {
    let isKeyboardUser = false

    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Tab') {
        isKeyboardUser = true
        document.body.classList.add('keyboard-user')
      }
    }

    const handleMouseDown = () => {
      isKeyboardUser = false
      document.body.classList.remove('keyboard-user')
    }

    document.addEventListener('keydown', handleKeyDown)
    document.addEventListener('mousedown', handleMouseDown)

    return () => {
      document.removeEventListener('keydown', handleKeyDown)
      document.removeEventListener('mousedown', handleMouseDown)
    }
  }, [])

  return (
    <svg className="accessibility-filters">
      <defs>
        {/* Protanopia Filter */}
        <filter id="protanopia-filter">
          <feColorMatrix values="0.567, 0.433, 0,     0, 0
                                 0.558, 0.442, 0,     0, 0
                                 0,     0.242, 0.758, 0, 0
                                 0,     0,     0,     1, 0" />
        </filter>

        {/* Deuteranopia Filter */}
        <filter id="deuteranopia-filter">
          <feColorMatrix values="0.625, 0.375, 0,   0, 0
                                 0.7,   0.3,   0,   0, 0
                                 0,     0.3,   0.7, 0, 0
                                 0,     0,     0,   1, 0" />
        </filter>

        {/* Tritanopia Filter */}
        <filter id="tritanopia-filter">
          <feColorMatrix values="0.95, 0.05,  0,     0, 0
                                 0,    0.433, 0.567, 0, 0
                                 0,    0.475, 0.525, 0, 0
                                 0,    0,     0,     1, 0" />
        </filter>
      </defs>
    </svg>
  )
}

// Hook to use accessibility context
export function useAccessibility() {
  const context = useContext(AccessibilityContext)
  if (!context) {
    throw new Error('useAccessibility must be used within an AccessibilityProvider')
  }
  return context
}

// Accessibility settings panel component
export function AccessibilitySettings() {
  const { settings, updateSetting, resetSettings } = useAccessibility()
  const { t } = useTranslation()

  return (
    <div className="space-y-6">
      <h2 className="text-2xl font-bold">{t('settings.accessibility')}</h2>

      {/* Visual Settings */}
      <div className="space-y-4">
        <h3 className="text-lg font-semibold">Visual</h3>
        
        <label className="flex items-center space-x-3">
          <input
            type="checkbox"
            checked={settings.highContrast}
            onChange={(e) => updateSetting('highContrast', e.target.checked)}
            className="w-4 h-4"
          />
          <span>High Contrast Mode</span>
        </label>

        <label className="flex items-center space-x-3">
          <input
            type="checkbox"
            checked={settings.reducedMotion}
            onChange={(e) => updateSetting('reducedMotion', e.target.checked)}
            className="w-4 h-4"
          />
          <span>Reduce Motion</span>
        </label>

        <div className="space-y-2">
          <label className="block text-sm font-medium">Font Size</label>
          <select
            value={settings.fontSize}
            onChange={(e) => updateSetting('fontSize', e.target.value as any)}
            className="w-full p-2 border border-gray-300 rounded"
          >
            <option value="small">Small</option>
            <option value="medium">Medium</option>
            <option value="large">Large</option>
            <option value="extra-large">Extra Large</option>
          </select>
        </div>

        <div className="space-y-2">
          <label className="block text-sm font-medium">Color Blind Support</label>
          <select
            value={settings.colorBlindMode}
            onChange={(e) => updateSetting('colorBlindMode', e.target.value as any)}
            className="w-full p-2 border border-gray-300 rounded"
          >
            <option value="none">None</option>
            <option value="protanopia">Protanopia</option>
            <option value="deuteranopia">Deuteranopia</option>
            <option value="tritanopia">Tritanopia</option>
          </select>
        </div>
      </div>

      {/* Cognitive Settings */}
      <div className="space-y-4">
        <h3 className="text-lg font-semibold">Cognitive</h3>
        
        <label className="flex items-center space-x-3">
          <input
            type="checkbox"
            checked={settings.simplifiedUI}
            onChange={(e) => updateSetting('simplifiedUI', e.target.checked)}
            className="w-4 h-4"
          />
          <span>Simplified Interface</span>
        </label>

        <label className="flex items-center space-x-3">
          <input
            type="checkbox"
            checked={settings.autoplayDisabled}
            onChange={(e) => updateSetting('autoplayDisabled', e.target.checked)}
            className="w-4 h-4"
          />
          <span>Disable Autoplay</span>
        </label>
      </div>

      {/* Keyboard Settings */}
      <div className="space-y-4">
        <h3 className="text-lg font-semibold">Keyboard & Navigation</h3>
        
        <label className="flex items-center space-x-3">
          <input
            type="checkbox"
            checked={settings.skipLinks}
            onChange={(e) => updateSetting('skipLinks', e.target.checked)}
            className="w-4 h-4"
          />
          <span>Show Skip Links</span>
        </label>

        <label className="flex items-center space-x-3">
          <input
            type="checkbox"
            checked={settings.screenReaderOptimized}
            onChange={(e) => updateSetting('screenReaderOptimized', e.target.checked)}
            className="w-4 h-4"
          />
          <span>Screen Reader Optimized</span>
        </label>
      </div>

      <button
        onClick={resetSettings}
        className="px-4 py-2 bg-gray-600 text-white rounded hover:bg-gray-700"
      >
        {t('settings.reset')}
      </button>
    </div>
  )
}