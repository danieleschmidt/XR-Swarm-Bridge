import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { resolve } from 'path'

export default defineConfig({
  plugins: [react()],
  root: '.',
  publicDir: 'public',
  resolve: {
    alias: {
      '@': resolve(__dirname, './src'),
    },
  },
  server: {
    port: 3000,
    host: '0.0.0.0',
    https: {
      // Self-signed certificates for development
      // In production, use proper SSL certificates
    }
  },
  build: {
    outDir: 'dist',
    sourcemap: true,
    chunkSizeWarningLimit: 1000, // Increase to 1000kb for advanced robotics features
    rollupOptions: {
      input: {
        main: resolve(__dirname, 'public/index.html')
      },
      output: {
        manualChunks(id) {
          if (id.includes('node_modules/three')) {
            return 'three';
          }
          if (id.includes('node_modules/@react-three')) {
            return 'react-three';
          }
          if (id.includes('node_modules/react') || id.includes('node_modules/zustand') || id.includes('node_modules/socket.io-client')) {
            return 'vendor';
          }
          if (id.includes('/src/consciousness/')) {
            return 'consciousness';
          }
          if (id.includes('/src/quantum/') || id.includes('/src/multiverse/') || id.includes('/src/temporal/')) {
            return 'quantum';
          }
          if (id.includes('/src/ai/')) {
            return 'ai';
          }
          if (id.includes('/src/utils/')) {
            return 'utils';
          }
        }
      }
    }
  },
  optimizeDeps: {
    include: ['three', '@react-three/fiber', '@react-three/drei', '@react-three/xr']
  },
  define: {
    // Enable WebXR in development
    global: 'globalThis',
  }
})