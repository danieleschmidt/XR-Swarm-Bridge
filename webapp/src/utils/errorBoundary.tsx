import React from 'react'
import { useTranslation } from 'react-i18next'

interface ErrorBoundaryState {
  hasError: boolean
  error?: Error
  errorInfo?: React.ErrorInfo
}

interface ErrorBoundaryProps {
  children: React.ReactNode
  fallback?: React.ComponentType<{ error: Error; resetError: () => void }>
}

export class ErrorBoundary extends React.Component<ErrorBoundaryProps, ErrorBoundaryState> {
  constructor(props: ErrorBoundaryProps) {
    super(props)
    this.state = { hasError: false }
  }

  static getDerivedStateFromError(error: Error): ErrorBoundaryState {
    return { hasError: true, error }
  }

  componentDidCatch(error: Error, errorInfo: React.ErrorInfo) {
    this.setState({
      error,
      errorInfo
    })
    
    // Log error to monitoring service
    console.error('XR-Swarm-Bridge Error:', error)
    console.error('Component stack:', errorInfo.componentStack)
    
    // Send to error tracking service
    if (process.env.NODE_ENV === 'production') {
      this.reportError(error, errorInfo)
    }
  }

  private reportError = (error: Error, errorInfo: React.ErrorInfo) => {
    // Implementation for error reporting service (e.g., Sentry)
    const errorReport = {
      message: error.message,
      stack: error.stack,
      componentStack: errorInfo.componentStack,
      url: window.location.href,
      userAgent: navigator.userAgent,
      timestamp: new Date().toISOString(),
      userId: localStorage.getItem('userId') || 'anonymous'
    }
    
    // Send to error tracking endpoint
    fetch('/api/errors', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(errorReport)
    }).catch(err => console.error('Failed to report error:', err))
  }

  private resetError = () => {
    this.setState({ hasError: false, error: undefined, errorInfo: undefined })
  }

  render() {
    if (this.state.hasError && this.state.error) {
      const FallbackComponent = this.props.fallback || DefaultErrorFallback
      return <FallbackComponent error={this.state.error} resetError={this.resetError} />
    }

    return this.props.children
  }
}

interface ErrorFallbackProps {
  error: Error
  resetError: () => void
}

const DefaultErrorFallback: React.FC<ErrorFallbackProps> = ({ error, resetError }) => {
  const { t } = useTranslation()
  
  const isDevelopment = process.env.NODE_ENV === 'development'
  
  return (
    <div className="min-h-screen bg-gray-900 text-white flex items-center justify-center p-4">
      <div className="max-w-md w-full bg-gray-800 rounded-lg p-6 shadow-xl border border-red-500/20">
        <div className="text-center mb-6">
          <div className="text-red-400 text-6xl mb-4">⚠️</div>
          <h1 className="text-2xl font-bold text-red-400 mb-2">
            {t('error.system_error', 'System Error')}
          </h1>
          <p className="text-gray-300 mb-4">
            {t('error.unexpected_error', 'An unexpected error occurred in the XR-Swarm-Bridge interface.')}
          </p>
        </div>
        
        {isDevelopment && (
          <div className="mb-6 p-4 bg-gray-700 rounded border-l-4 border-red-400">
            <h3 className="font-semibold text-red-400 mb-2">Development Error Details:</h3>
            <p className="text-sm font-mono text-gray-300 break-all">
              {error.message}
            </p>
            {error.stack && (
              <details className="mt-2">
                <summary className="cursor-pointer text-sm text-gray-400">
                  Stack Trace
                </summary>
                <pre className="mt-2 text-xs text-gray-400 overflow-auto max-h-32">
                  {error.stack}
                </pre>
              </details>
            )}
          </div>
        )}
        
        <div className="space-y-3">
          <button
            onClick={resetError}
            className="w-full bg-blue-600 hover:bg-blue-700 text-white py-2 px-4 rounded transition-colors"
          >
            {t('error.try_again', 'Try Again')}
          </button>
          
          <button
            onClick={() => window.location.reload()}
            className="w-full bg-gray-600 hover:bg-gray-700 text-white py-2 px-4 rounded transition-colors"
          >
            {t('error.reload_page', 'Reload Page')}
          </button>
          
          <div className="text-center pt-4 border-t border-gray-600">
            <p className="text-sm text-gray-400 mb-2">
              {t('error.persist_help', 'If this error persists:')}
            </p>
            <div className="text-xs text-gray-500 space-y-1">
              <p>• Check your network connection</p>
              <p>• Clear browser cache and cookies</p>
              <p>• Try a different browser</p>
              <p>• Contact system administrator</p>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}

// Hook for handling async errors
export const useErrorHandler = () => {
  const [error, setError] = React.useState<Error | null>(null)
  
  const handleError = React.useCallback((error: Error | unknown) => {
    const err = error instanceof Error ? error : new Error(String(error))
    setError(err)
    
    // Log the error
    console.error('Async error caught:', err)
    
    // Report in production
    if (process.env.NODE_ENV === 'production') {
      fetch('/api/errors', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: err.message,
          stack: err.stack,
          type: 'async_error',
          timestamp: new Date().toISOString()
        })
      }).catch(console.error)
    }
  }, [])
  
  const clearError = React.useCallback(() => setError(null), [])
  
  // Throw error in next render to trigger ErrorBoundary
  if (error) {
    throw error
  }
  
  return { handleError, clearError }
}

export default ErrorBoundary