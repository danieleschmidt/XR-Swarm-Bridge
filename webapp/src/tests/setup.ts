import { vi } from 'vitest'

// Mock WebGL context for Three.js tests
const mockWebGLContext = {
  canvas: {},
  drawingBufferWidth: 1024,
  drawingBufferHeight: 768,
  getExtension: vi.fn(),
  getParameter: vi.fn(),
  getShaderParameter: vi.fn(),
  getProgramParameter: vi.fn(),
  createShader: vi.fn(),
  shaderSource: vi.fn(),
  compileShader: vi.fn(),
  createProgram: vi.fn(),
  attachShader: vi.fn(),
  linkProgram: vi.fn(),
  useProgram: vi.fn(),
  getAttribLocation: vi.fn(),
  getUniformLocation: vi.fn(),
  enableVertexAttribArray: vi.fn(),
  vertexAttribPointer: vi.fn(),
  uniform1f: vi.fn(),
  uniform2f: vi.fn(),
  uniform3f: vi.fn(),
  uniform4f: vi.fn(),
  uniformMatrix4fv: vi.fn(),
  createBuffer: vi.fn(),
  bindBuffer: vi.fn(),
  bufferData: vi.fn(),
  createTexture: vi.fn(),
  bindTexture: vi.fn(),
  texImage2D: vi.fn(),
  texParameteri: vi.fn(),
  generateMipmap: vi.fn(),
  createFramebuffer: vi.fn(),
  bindFramebuffer: vi.fn(),
  framebufferTexture2D: vi.fn(),
  createRenderbuffer: vi.fn(),
  bindRenderbuffer: vi.fn(),
  renderbufferStorage: vi.fn(),
  framebufferRenderbuffer: vi.fn(),
  viewport: vi.fn(),
  clearColor: vi.fn(),
  clear: vi.fn(),
  drawElements: vi.fn(),
  drawArrays: vi.fn(),
  enable: vi.fn(),
  disable: vi.fn(),
  blendFunc: vi.fn(),
  depthFunc: vi.fn(),
  cullFace: vi.fn(),
  // WebGL constants
  VERTEX_SHADER: 0x8B31,
  FRAGMENT_SHADER: 0x8B30,
  ARRAY_BUFFER: 0x8892,
  ELEMENT_ARRAY_BUFFER: 0x8893,
  STATIC_DRAW: 0x88E4,
  TEXTURE_2D: 0x0DE1,
  RGBA: 0x1908,
  UNSIGNED_BYTE: 0x1401,
  TEXTURE_MAG_FILTER: 0x2800,
  TEXTURE_MIN_FILTER: 0x2801,
  LINEAR: 0x2601,
  CLAMP_TO_EDGE: 0x812F,
  TEXTURE_WRAP_S: 0x2802,
  TEXTURE_WRAP_T: 0x2803,
  COLOR_BUFFER_BIT: 0x00004000,
  DEPTH_BUFFER_BIT: 0x00000100,
  DEPTH_TEST: 0x0B71,
  BLEND: 0x0BE2,
  SRC_ALPHA: 0x0302,
  ONE_MINUS_SRC_ALPHA: 0x0303,
  BACK: 0x0405,
  CULL_FACE: 0x0B44,
  LEQUAL: 0x0203,
  FLOAT: 0x1406
}

// Mock HTMLCanvasElement.getContext
HTMLCanvasElement.prototype.getContext = vi.fn((contextType) => {
  if (contextType === 'webgl' || contextType === 'webgl2') {
    return mockWebGLContext
  }
  return null
})

// Mock WebXR
Object.defineProperty(navigator, 'xr', {
  value: {
    isSessionSupported: vi.fn().mockResolvedValue(false),
    requestSession: vi.fn().mockRejectedValue(new Error('WebXR not supported in tests'))
  },
  writable: true
})

// Mock WebRTC
global.RTCPeerConnection = vi.fn().mockImplementation(() => ({
  createOffer: vi.fn().mockResolvedValue({}),
  createAnswer: vi.fn().mockResolvedValue({}),
  setLocalDescription: vi.fn().mockResolvedValue(undefined),
  setRemoteDescription: vi.fn().mockResolvedValue(undefined),
  addIceCandidate: vi.fn().mockResolvedValue(undefined),
  close: vi.fn(),
  addEventListener: vi.fn(),
  removeEventListener: vi.fn()
}))

// Mock WebSocket
global.WebSocket = vi.fn().mockImplementation((url) => ({
  url,
  readyState: 1, // OPEN
  send: vi.fn(),
  close: vi.fn(),
  addEventListener: vi.fn(),
  removeEventListener: vi.fn()
}))

// Mock MediaDevices
Object.defineProperty(navigator, 'mediaDevices', {
  value: {
    getUserMedia: vi.fn().mockResolvedValue({
      getTracks: vi.fn().mockReturnValue([])
    }),
    enumerateDevices: vi.fn().mockResolvedValue([])
  },
  writable: true
})

// Mock requestAnimationFrame
global.requestAnimationFrame = vi.fn((callback) => {
  setTimeout(callback, 16) // ~60fps
  return 1
})

global.cancelAnimationFrame = vi.fn()

// Mock performance
Object.defineProperty(global, 'performance', {
  value: {
    now: vi.fn(() => Date.now()),
    mark: vi.fn(),
    measure: vi.fn(),
    getEntriesByName: vi.fn().mockReturnValue([]),
    getEntriesByType: vi.fn().mockReturnValue([])
  }
})

// Mock ResizeObserver
global.ResizeObserver = vi.fn().mockImplementation((callback) => ({
  observe: vi.fn(),
  unobserve: vi.fn(),
  disconnect: vi.fn()
}))

// Mock IntersectionObserver
global.IntersectionObserver = vi.fn().mockImplementation((callback) => ({
  observe: vi.fn(),
  unobserve: vi.fn(),
  disconnect: vi.fn()
}))

// Suppress console warnings during tests
const originalConsoleWarn = console.warn
console.warn = (...args) => {
  // Filter out Three.js WebGL warnings during tests
  const message = args[0]
  if (typeof message === 'string' && (
    message.includes('WebGL') ||
    message.includes('THREE.') ||
    message.includes('WebXR')
  )) {
    return
  }
  originalConsoleWarn(...args)
}

// Mock localStorage
const localStorageMock = {
  getItem: vi.fn(),
  setItem: vi.fn(),
  removeItem: vi.fn(),
  clear: vi.fn(),
  length: 0,
  key: vi.fn()
}

Object.defineProperty(window, 'localStorage', {
  value: localStorageMock
})

// Mock sessionStorage
Object.defineProperty(window, 'sessionStorage', {
  value: localStorageMock
})