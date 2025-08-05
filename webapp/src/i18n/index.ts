import i18n from 'i18next'
import { initReactI18next } from 'react-i18next'
import Backend from 'i18next-http-backend'
import LanguageDetector from 'i18next-browser-languagedetector'

// Import language resources
import en from './locales/en/common.json'
import es from './locales/es/common.json'
import fr from './locales/fr/common.json'
import de from './locales/de/common.json'
import zh from './locales/zh/common.json'
import ja from './locales/ja/common.json'
import ar from './locales/ar/common.json'

// Define supported languages
export const SUPPORTED_LANGUAGES = [
  { code: 'en', name: 'English', nativeName: 'English' },
  { code: 'es', name: 'Spanish', nativeName: 'Español' },
  { code: 'fr', name: 'French', nativeName: 'Français' },
  { code: 'de', name: 'German', nativeName: 'Deutsch' },
  { code: 'zh', name: 'Chinese', nativeName: '中文' },
  { code: 'ja', name: 'Japanese', nativeName: '日本語' },
  { code: 'ar', name: 'Arabic', nativeName: 'العربية' }
]

// Resources object
const resources = {
  en: { common: en },
  es: { common: es },
  fr: { common: fr },
  de: { common: de },
  zh: { common: zh },
  ja: { common: ja },
  ar: { common: ar }
}

// Initialize i18next
i18n
  .use(Backend)
  .use(LanguageDetector)
  .use(initReactI18next)
  .init({
    resources,
    fallbackLng: 'en',
    debug: process.env.NODE_ENV === 'development',
    
    interpolation: {
      escapeValue: false // React already does escaping
    },
    
    // Detection options
    detection: {
      order: ['localStorage', 'navigator', 'htmlTag'],
      caches: ['localStorage'],
      lookupLocalStorage: 'xr-swarm-language'
    },
    
    // Backend options (for loading translations)
    backend: {
      loadPath: '/locales/{{lng}}/{{ns}}.json',
      allowMultiLoading: false
    },
    
    // React options
    react: {
      useSuspense: false,
      bindI18n: 'languageChanged loaded',
      bindI18nStore: 'added removed',
      transEmptyNodeValue: '',
      transSupportBasicHtmlNodes: true,
      transKeepBasicHtmlNodesFor: ['br', 'strong', 'i', 'em', 'b']
    },
    
    // Default namespace
    defaultNS: 'common',
    ns: ['common']
  })

export default i18n

// Utility functions
export const getCurrentLanguage = () => i18n.language
export const changeLanguage = (lang: string) => i18n.changeLanguage(lang)
export const isRTL = (lang: string = i18n.language) => ['ar', 'he', 'fa'].includes(lang)

// Format numbers/dates for current locale
export const formatNumber = (num: number, options?: Intl.NumberFormatOptions) => {
  return new Intl.NumberFormat(i18n.language, options).format(num)
}

export const formatDate = (date: Date, options?: Intl.DateTimeFormatOptions) => {
  return new Intl.DateTimeFormat(i18n.language, options).format(date)
}

export const formatCurrency = (amount: number, currency: string = 'USD') => {
  return new Intl.NumberFormat(i18n.language, {
    style: 'currency',
    currency
  }).format(amount)
}