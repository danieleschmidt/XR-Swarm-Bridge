# XR-Swarm-Bridge API Reference

## Overview

XR-Swarm-Bridge provides multiple API interfaces for controlling robot swarms, managing missions, and integrating with external systems. This document covers all available APIs including WebSocket, WebRTC, ROS topics, and HTTP endpoints.

## Authentication

All API endpoints require authentication using JWT tokens or API keys.

```bash
# Get authentication token
curl -X POST https://api.xr-swarm-bridge.com/auth/login \
  -H "Content-Type: application/json" \
  -d '{"username": "operator", "password": "secure_password"}'

# Use token in subsequent requests
curl -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  https://api.xr-swarm-bridge.com/api/v1/agents
```

## WebSocket API

### Connection

Connect to the swarm coordinator via WebSocket for real-time communication:

```javascript
const ws = new WebSocket('wss://your-domain.com:8443');

ws.onopen = () => {
  console.log('Connected to swarm coordinator');
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  handleSwarmMessage(message);
};
```

### Message Format

All WebSocket messages use a standardized JSON format:

```json
{
  "type": "message_type",
  "timestamp": 1640995200000,
  "id": "msg_12345",
  "data": {
    // Message-specific data
  }
}
```

### Swarm Control Messages

#### Get Swarm Status

```json
{
  "type": "get_swarm_status",
  "timestamp": 1640995200000,
  "id": "req_001"
}
```

**Response**:
```json
{
  "type": "swarm_status",
  "timestamp": 1640995200000,
  "id": "req_001",
  "data": {
    "total_agents": 25,
    "active_agents": 23,
    "agents": {
      "drone_01": {
        "type": "drone",
        "status": "active",
        "position": [10.5, 20.3, 15.0],
        "battery": 85,
        "last_seen": 1640995195000
      }
    },
    "current_mission": {
      "id": "mission_001",
      "name": "Area Survey",
      "status": "active",
      "progress": 65
    }
  }
}
```

#### Send Global Command

```json
{
  "type": "global_command",
  "timestamp": 1640995200000,
  "id": "cmd_001",
  "data": {
    "command": {
      "type": "formation",
      "formation": "line",
      "spacing": 5.0,
      "target_agents": ["drone_01", "drone_02", "drone_03"]
    }
  }
}
```

#### Send Agent-Specific Command

```json
{
  "type": "agent_command",
  "timestamp": 1640995200000,
  "id": "cmd_002",
  "data": {
    "agent_id": "drone_01",
    "command": {
      "type": "navigate",
      "position": [25.0, 30.0, 10.0],
      "speed": 2.5
    }
  }
}
```

### Mission Management

#### Start Mission

```json
{
  "type": "start_mission",
  "timestamp": 1640995200000,
  "id": "mission_start_001",
  "data": {
    "mission": {
      "id": "mission_002",
      "name": "Search and Rescue",
      "description": "Search for survivors in collapsed building",
      "phases": [
        {
          "phase": 1,
          "description": "Perimeter establishment",
          "assignments": {
            "drone_01": "North perimeter patrol",
            "drone_02": "South perimeter patrol",
            "ugv_01": "Equipment deployment"
          }
        }
      ]
    }
  }
}
```

#### Stop Mission

```json
{
  "type": "stop_mission",
  "timestamp": 1640995200000,
  "id": "mission_stop_001"
}
```

### Telemetry Streaming

#### Agent Telemetry Updates

Automatic telemetry updates are sent for all active agents:

```json
{
  "type": "agent_telemetry",
  "timestamp": 1640995200000,
  "data": {
    "agent_id": "drone_01",
    "position": [10.5, 20.3, 15.0],
    "velocity": [1.2, 0.8, 0.0],
    "battery": 85,
    "status": "navigating",
    "sensors": {
      "camera": {
        "resolution": "1920x1080",
        "fps": 30,
        "stream_url": "webrtc://stream_001"
      },
      "lidar": {
        "range": 50.0,
        "points": 12000
      }
    }
  }
}
```

#### System Health Updates

```json
{
  "type": "system_health",
  "timestamp": 1640995200000,
  "data": {
    "coordinator": {
      "status": "healthy",
      "cpu_usage": 45.2,
      "memory_usage": 62.8,
      "uptime": 86400
    },
    "webrtc_bridge": {
      "status": "healthy",
      "active_connections": 15,
      "bandwidth_usage": 125.6
    },
    "network": {
      "latency": 45,
      "packet_loss": 0.02,
      "bandwidth": 1000
    }
  }
}
```

## WebRTC Data Channels

For ultra-low latency communication, WebRTC data channels are used for critical telemetry and commands.

### Connection Setup

```javascript
const pc = new RTCPeerConnection({
  iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
});

// Create data channel
const dataChannel = pc.createDataChannel('swarm_control', {
  ordered: false,  // Allow out-of-order delivery for speed
  maxRetransmits: 0  // No retransmission for real-time data
});

dataChannel.onopen = () => {
  console.log('WebRTC data channel open');
};

dataChannel.onmessage = (event) => {
  const data = JSON.parse(event.data);
  handleRealTimeData(data);
};
```

### Data Channel Messages

#### Critical Telemetry (50Hz updates)

```json
{
  "t": "pos",  // Abbreviated for speed
  "id": "drone_01",
  "p": [10.5, 20.3, 15.0],  // position
  "v": [1.2, 0.8, 0.0],     // velocity  
  "b": 85,                  // battery
  "ts": 1640995200123       // timestamp
}
```

#### Emergency Commands

```json
{
  "t": "emergency",
  "cmd": "stop",
  "all": true,
  "ts": 1640995200123
}
```

#### Hand Tracking Data (VR)

```json
{
  "t": "hand",
  "left": {
    "pos": [0.1, 0.8, -0.3],
    "rot": [0, 0, 0, 1],
    "gesture": "pointing"
  },
  "right": {
    "pos": [0.2, 0.9, -0.2],
    "rot": [0, 0, 0, 1],
    "gesture": "grasping"
  },
  "ts": 1640995200123
}
```

## ROS 2 Topics

### Command Topics

#### Global Commands
- **Topic**: `/swarm/global/command`
- **Type**: `std_msgs/String`
- **Description**: Broadcast commands to all agents

```bash
# Command all drones to return to base
ros2 topic pub /swarm/global/command std_msgs/String \
  '{"data": "{\"type\": \"return_to_base\", \"timestamp\": 1640995200}"}'
```

#### Agent-Specific Commands
- **Topic**: `/swarm/{agent_id}/command`
- **Type**: `std_msgs/String`
- **Description**: Send commands to specific agent

```bash
# Command drone_01 to hover
ros2 topic pub /swarm/drone_01/command std_msgs/String \
  '{"data": "{\"type\": \"hover\", \"altitude\": 10.0}"}'
```

### Telemetry Topics

#### Agent Status
- **Topic**: `/swarm/{agent_id}/status`
- **Type**: `std_msgs/String`
- **Publisher**: Individual agents
- **Rate**: 1 Hz

#### Agent Pose
- **Topic**: `/swarm/{agent_id}/pose`
- **Type**: `geometry_msgs/PoseStamped`
- **Publisher**: Individual agents
- **Rate**: 10 Hz

#### Agent Telemetry
- **Topic**: `/swarm/{agent_id}/telemetry`
- **Type**: `std_msgs/String`
- **Publisher**: Individual agents
- **Rate**: 5 Hz

### Video Streaming Topics

#### Agent Camera
- **Topic**: `/swarm/{agent_id}/camera/image`
- **Type**: `sensor_msgs/Image`
- **Publisher**: Individual agents
- **Rate**: 30 Hz

#### Compressed Images
- **Topic**: `/swarm/{agent_id}/camera/image/compressed`
- **Type**: `sensor_msgs/CompressedImage`
- **Publisher**: Individual agents
- **Rate**: 30 Hz

### Point Cloud Topics

#### Agent LiDAR
- **Topic**: `/swarm/{agent_id}/pointcloud`
- **Type**: `sensor_msgs/PointCloud2`
- **Publisher**: Individual agents
- **Rate**: 10 Hz

## HTTP REST API

### Base URL
```
https://api.xr-swarm-bridge.com/api/v1
```

### Authentication
```http
Authorization: Bearer YOUR_JWT_TOKEN
```

### Agents Endpoint

#### Get All Agents
```http
GET /agents
```

**Response**:
```json
{
  "agents": [
    {
      "id": "drone_01",
      "type": "drone",
      "status": "active",
      "position": [10.5, 20.3, 15.0],
      "battery": 85,
      "capabilities": ["navigate", "hover", "stream_video"],
      "last_seen": "2023-01-01T12:00:00Z"
    }
  ],
  "total": 25,
  "active": 23
}
```

#### Get Specific Agent
```http
GET /agents/{agent_id}
```

**Response**:
```json
{
  "id": "drone_01",
  "type": "drone",
  "status": "active",
  "position": [10.5, 20.3, 15.0],
  "battery": 85,
  "telemetry": {
    "altitude": 15.0,
    "speed": 2.5,
    "heading": 90.0
  },
  "sensors": [
    {
      "type": "camera",
      "status": "active",
      "stream_url": "webrtc://stream_001"
    }
  ]
}
```

#### Send Command to Agent
```http
POST /agents/{agent_id}/commands
Content-Type: application/json

{
  "type": "navigate",
  "position": [25.0, 30.0, 10.0],
  "speed": 2.5
}
```

**Response**:
```json
{
  "command_id": "cmd_12345",
  "status": "accepted",
  "estimated_completion": "2023-01-01T12:05:00Z"
}
```

### Missions Endpoint

#### Get All Missions
```http
GET /missions
```

**Response**:
```json
{
  "missions": [
    {
      "id": "mission_001",
      "name": "Area Survey",
      "status": "active",
      "progress": 65,
      "start_time": "2023-01-01T12:00:00Z",
      "estimated_completion": "2023-01-01T14:00:00Z"
    }
  ]
}
```

#### Create Mission
```http
POST /missions
Content-Type: application/json

{
  "name": "Search and Rescue",
  "description": "Search for survivors in collapsed building",
  "phases": [
    {
      "phase": 1,
      "description": "Perimeter establishment",
      "assignments": {
        "drone_01": "North perimeter patrol",
        "drone_02": "South perimeter patrol"
      }
    }
  ]
}
```

**Response**:
```json
{
  "id": "mission_002",
  "status": "created",
  "message": "Mission created successfully"
}
```

#### Start Mission
```http
POST /missions/{mission_id}/start
```

#### Stop Mission
```http
POST /missions/{mission_id}/stop
```

### System Status Endpoint

#### Get System Health
```http
GET /system/health
```

**Response**:
```json
{
  "status": "healthy",
  "components": {
    "swarm_coordinator": {
      "status": "healthy",
      "uptime": 86400,
      "memory_usage": 62.8
    },
    "webrtc_bridge": {
      "status": "healthy",
      "active_connections": 15
    },
    "database": {
      "status": "healthy",
      "response_time": 5
    }
  },
  "metrics": {
    "total_agents": 25,
    "active_missions": 1,
    "average_latency": 45
  }
}
```

#### Get Performance Metrics
```http
GET /system/metrics
```

**Response**:
```json
{
  "timestamp": "2023-01-01T12:00:00Z",
  "metrics": {
    "latency": {
      "p50": 35,
      "p95": 65,
      "p99": 95
    },
    "throughput": {
      "commands_per_second": 150,
      "messages_per_second": 2500
    },
    "resources": {
      "cpu_usage": 45.2,
      "memory_usage": 62.8,
      "network_bandwidth": 125.6
    },
    "agents": {
      "total": 25,
      "active": 23,
      "average_battery": 78.5
    }
  }
}
```

## GPT-4o Integration API

### Natural Language Commands

#### Send Natural Language Command
```http
POST /ai/commands
Content-Type: application/json

{
  "command": "Have all drones form a line formation and search the northern sector",
  "context": {
    "available_agents": ["drone_01", "drone_02", "drone_03"],
    "current_location": [0, 0, 0],
    "mission_objective": "area_search"
  }
}
```

**Response**:
```json
{
  "interpretation": {
    "command_type": "formation_and_search",
    "confidence": 0.95,
    "parameters": {
      "formation": "line",
      "target_agents": ["drone_01", "drone_02", "drone_03"],
      "search_area": "northern_sector"
    }
  },
  "execution_plan": [
    {
      "step": 1,
      "action": "form_line_formation",
      "agents": ["drone_01", "drone_02", "drone_03"],
      "parameters": {"spacing": 10.0}
    },
    {
      "step": 2,
      "action": "search_pattern",
      "pattern": "grid",
      "area": {"bounds": [[0, 50], [0, 100]]}
    }
  ],
  "estimated_duration": 1800,
  "command_id": "ai_cmd_001"
}
```

#### Generate Mission Plan
```http
POST /ai/missions
Content-Type: application/json

{
  "objective": "Search and rescue in collapsed building with 20 drones and 5 ground vehicles",
  "constraints": {
    "time_limit": 7200,
    "safety_level": "high",
    "available_agents": {
      "drones": 20,
      "ugvs": 5
    }
  }
}
```

**Response**:
```json
{
  "mission": {
    "id": "ai_mission_001",
    "name": "AI-Generated Search and Rescue",
    "description": "Comprehensive search and rescue operation using coordinated drone and ground vehicle teams",
    "estimated_duration": 6800,
    "phases": [
      {
        "phase": 1,
        "description": "Perimeter establishment and initial reconnaissance",
        "duration": 1200,
        "assignments": {
          "drones_1-8": "Establish aerial perimeter at 20m altitude",
          "drones_9-12": "High-altitude overview and mapping",
          "ugvs_1-2": "Ground-level perimeter establishment"
        }
      }
    ],
    "contingencies": [
      {
        "trigger": "Structural instability detected",
        "action": "All units retreat 50m and reassess",
        "priority": "high"
      }
    ],
    "success_criteria": [
      "100% area coverage achieved",
      "All survivors located and marked",
      "No agent losses or damage"
    ]
  },
  "confidence": 0.88,
  "alternative_plans": 2
}
```

## Error Handling

### Standard Error Response

```json
{
  "error": {
    "code": "INVALID_COMMAND",
    "message": "The specified command is not supported by this agent type",
    "details": {
      "agent_id": "drone_01",
      "command_type": "excavate",
      "supported_commands": ["navigate", "hover", "takeoff", "land"]
    },
    "timestamp": "2023-01-01T12:00:00Z"
  }
}
```

### Common Error Codes

- `AUTHENTICATION_FAILED`: Invalid or expired token
- `AGENT_NOT_FOUND`: Specified agent ID does not exist
- `AGENT_OFFLINE`: Agent is not currently connected
- `INVALID_COMMAND`: Command not supported or malformed
- `MISSION_NOT_FOUND`: Specified mission ID does not exist
- `SYSTEM_OVERLOAD`: System is currently at capacity
- `EMERGENCY_STOP_ACTIVE`: System is in emergency stop mode
- `GEOFENCE_VIOLATION`: Requested action would violate safety boundaries

## Rate Limits

### WebSocket API
- **Connection limit**: 100 concurrent connections per user
- **Message rate**: 1000 messages per minute per connection

### HTTP REST API
- **Request rate**: 10,000 requests per hour per API key
- **Burst limit**: 100 requests per minute

### WebRTC Data Channels
- **Data rate**: Unlimited (hardware/network constrained)
- **Connection limit**: 50 concurrent data channels per user

## SDK Examples

### JavaScript/TypeScript

```javascript
import { XRSwarmBridge } from '@xr-swarm-bridge/sdk';

const swarm = new XRSwarmBridge({
  serverUrl: 'wss://your-domain.com:8443',
  apiKey: 'your-api-key'
});

// Connect to swarm
await swarm.connect();

// Send natural language command
const result = await swarm.command("Form a search grid with all available drones");

// Listen for agent updates
swarm.onAgentUpdate((agent) => {
  console.log(`Agent ${agent.id} is now at position:`, agent.position);
});

// Start a mission
const mission = await swarm.startMission({
  name: "Area Survey",
  objective: "Survey the designated area for anomalies"
});
```

### Python

```python
from xr_swarm_bridge import SwarmClient
import asyncio

async def main():
    client = SwarmClient(
        server_url="wss://your-domain.com:8443",
        api_key="your-api-key"
    )
    
    # Connect to swarm
    await client.connect()
    
    # Send command to specific agent
    await client.send_agent_command("drone_01", {
        "type": "navigate",
        "position": [10, 20, 15]
    })
    
    # Listen for telemetry updates
    async for telemetry in client.telemetry_stream():
        print(f"Agent {telemetry.agent_id}: {telemetry.data}")

if __name__ == "__main__":
    asyncio.run(main())
```

### ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')
        
        # Publisher for global commands
        self.command_pub = self.create_publisher(
            String, '/swarm/global/command', 10
        )
        
        # Subscriber for agent telemetry
        self.telemetry_sub = self.create_subscription(
            String, '/swarm/+/telemetry',
            self.telemetry_callback, 10
        )
    
    def send_formation_command(self, formation_type, agents):
        command = {
            "type": "formation",
            "formation": formation_type,
            "target_agents": agents
        }
        
        msg = String()
        msg.data = json.dumps(command)
        self.command_pub.publish(msg)
    
    def telemetry_callback(self, msg):
        data = json.loads(msg.data)
        self.get_logger().info(f"Received telemetry: {data}")

def main():
    rclpy.init()
    controller = SwarmController()
    
    # Send formation command
    controller.send_formation_command("line", ["drone_01", "drone_02"])
    
    rclpy.spin(controller)
    rclpy.shutdown()
```

This API reference provides comprehensive coverage of all available interfaces for interacting with the XR-Swarm-Bridge system. Use the appropriate API based on your specific requirements for latency, reliability, and integration needs.