import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import { resolve } from 'path';

export default defineConfig({
  plugins: [react()],
  
  // Production optimizations
  build: {
    target: 'esnext',
    minify: 'terser',
    sourcemap: true,
    
    // Code splitting and chunk optimization
    rollupOptions: {
      output: {
        manualChunks: {
          // Vendor chunks
          'vendor-react': ['react', 'react-dom'],
          'vendor-three': ['three'],
          'vendor-react-three': ['@react-three/fiber', '@react-three/drei', '@react-three/xr'],
          
          // Feature chunks
          'xr-core': [
            'src/xr/XRInterface.tsx',
            'src/xr/XRControlPanel.tsx',
            'src/xr/XRGestureHandler.tsx',
            'src/xr/XRMiniMap.tsx'
          ],
          'ai-ml': [
            'src/ai/gptIntegration.ts',
            'src/ai/mlIntegration.ts',
            'src/ai/autonomousPlanning.ts'
          ],
          'optimization': [
            'src/utils/quantumHybridOptimizer.ts',
            'src/utils/quantumPerformanceOptimizer.ts',
            'src/utils/massiveScaleOptimizer.ts',
            'src/utils/advancedCaching.ts',
            'src/utils/concurrentProcessing.ts'
          ],
          'monitoring': [
            'src/utils/advancedMonitoring.ts',
            'src/utils/predictiveHealthMonitor.ts',
            'src/utils/performance.ts'
          ],
          'security': [
            'src/utils/security.ts',
            'src/utils/advancedSecurity.ts',
            'src/utils/compliance.ts'
          ]
        },
        
        // Optimize chunk sizes
        chunkFileNames: (chunkInfo) => {
          const facadeModuleId = chunkInfo.facadeModuleId 
            ? chunkInfo.facadeModuleId.split('/').pop()?.replace('.tsx', '').replace('.ts', '') 
            : 'chunk';
          return `assets/${facadeModuleId}-[hash].js`;
        }
      }
    },
    
    // Compression and optimization
    terserOptions: {
      compress: {
        drop_console: true, // Remove console.log in production
        drop_debugger: true,
        passes: 2
      },
      mangle: {
        safari10: true
      }
    },
    
    // Size limits
    chunkSizeWarningLimit: 1000, // 1MB chunks
    assetsInlineLimit: 8192, // 8KB inline limit
  },
  
  // Advanced optimizations
  esbuild: {
    target: 'esnext',
    treeShaking: true,
    minifyIdentifiers: true,
    minifySyntax: true,
    minifyWhitespace: true
  },
  
  // Resolve optimization
  resolve: {
    alias: {
      '@': resolve(__dirname, 'src'),
      '@components': resolve(__dirname, 'src/components'),
      '@utils': resolve(__dirname, 'src/utils'),
      '@ai': resolve(__dirname, 'src/ai'),
      '@xr': resolve(__dirname, 'src/xr'),
      '@hooks': resolve(__dirname, 'src/hooks'),
      '@store': resolve(__dirname, 'src/store')
    }
  },
  
  // CSS optimization
  css: {
    postcss: {
      plugins: [
        require('autoprefixer'),
        require('cssnano')({
          preset: ['default', {
            discardComments: { removeAll: true },
            normalizeWhitespace: false
          }]
        })
      ]
    }
  },
  
  // Server configuration for production
  server: {
    port: 3000,
    host: '0.0.0.0',
    https: false // Enable HTTPS in production reverse proxy
  },
  
  preview: {
    port: 4173,
    host: '0.0.0.0'
  },
  
  // Environment variables
  envPrefix: ['VITE_'],
  
  // Advanced build features
  experimental: {
    renderBuiltUrl(filename) {
      // CDN support for production assets
      return `https://cdn.xr-swarm-bridge.com/assets/${filename}`;
    }
  }
});