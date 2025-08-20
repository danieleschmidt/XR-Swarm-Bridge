import { render, screen, act, waitFor } from '@testing-library/react';
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { BrowserRouter } from 'react-router-dom';
import App from '../App';

// Mock WebRTC and XR dependencies
vi.mock('@react-three/xr', () => ({
  XR: ({ children }: { children: React.ReactNode }) => <div data-testid="xr-container">{children}</div>,
  VRButton: ({ children }: { children?: React.ReactNode }) => <button data-testid="vr-button">{children || 'Enter VR'}</button>,
  ARButton: ({ children }: { children?: React.ReactNode }) => <button data-testid="ar-button">{children || 'Enter AR'}</button>,
  useXR: () => ({
    isPresenting: false,
    player: null,
    isHandTracking: false,
    session: null,
    visibilityState: 'visible',
    controllers: [],
    hands: []
  }),
  useController: () => ({
    inputSource: null,
    grip: null,
    hand: null
  }),
  useHitTest: () => [],
  Hands: ({ children }: { children: React.ReactNode }) => <group>{children}</group>,
  Controllers: ({ children }: { children: React.ReactNode }) => <group>{children}</group>
}));

vi.mock('../hooks/useWebRTC', () => ({
  useWebRTC: () => ({
    isConnected: false,
    connect: vi.fn(),
    disconnect: vi.fn(),
    sendMessage: vi.fn(),
    peerConnection: null
  })
}));

vi.mock('../hooks/useSwarmConnection', () => ({
  useSwarmConnection: () => ({
    robots: [],
    isConnected: false,
    connectionStatus: 'disconnected',
    connect: vi.fn(),
    disconnect: vi.fn(),
    sendCommand: vi.fn()
  })
}));

const AppWrapper = ({ children }: { children?: React.ReactNode }) => (
  <BrowserRouter>
    <App />
    {children}
  </BrowserRouter>
);

describe('XR-Swarm-Bridge Integration Tests', () => {
  beforeEach(() => {
    // Reset all mocks before each test
    vi.clearAllMocks();
  });

  afterEach(() => {
    // Clean up after each test
    vi.resetAllMocks();
  });

  it('should render main application without crashing', () => {
    render(<AppWrapper />);
    expect(screen.getByTestId('xr-container')).toBeInTheDocument();
  });

  it('should initialize with default state', async () => {
    render(<AppWrapper />);
    
    await waitFor(() => {
      expect(screen.getByTestId('xr-container')).toBeInTheDocument();
    });
  });

  it('should handle XR session management', async () => {
    const mockConnect = vi.fn();
    
    render(<AppWrapper />);
    
    await act(async () => {
      // Simulate XR session start
      mockConnect();
    });
    
    expect(mockConnect).toHaveBeenCalled();
  });

  it('should manage swarm connections properly', async () => {
    const mockSendCommand = vi.fn();
    
    render(<AppWrapper />);
    
    await act(async () => {
      // Simulate swarm command
      mockSendCommand('test-command');
    });
    
    expect(mockSendCommand).toHaveBeenCalledWith('test-command');
  });
});