# XR-Swarm-Bridge Deployment Guide

## Overview

This guide covers deployment options for XR-Swarm-Bridge, from development environments to production-scale deployments supporting 100+ robots and multiple operators.

## Quick Start

### Prerequisites

- Ubuntu 22.04 or Windows 11 with WSL2
- ROS 2 Humble
- Node.js 18+
- Docker and Docker Compose (optional)
- Python 3.10+

### Local Development Setup

```bash
# Clone repository
git clone https://github.com/danieleschmidt/xr-swarm-bridge.git
cd xr-swarm-bridge

# Run setup script
./launch_xr_swarm.sh dev
```

This will:
- Install all dependencies
- Build ROS workspace
- Start all services in development mode
- Open webapp at http://localhost:3000

## Deployment Options

### 1. Development Environment

**Use Case**: Local development, testing, small-scale demos

```bash
# Development mode with hot reload
./launch_xr_swarm.sh dev

# Services started:
# - Swarm Coordinator (ROS)
# - WebRTC Bridge (port 8443)
# - Webapp Dev Server (port 3000)
# - 3x Example Drone Agents
```

**Features**:
- Hot reload for code changes
- Development tools enabled
- Mock data for offline development
- Reduced security (for easier debugging)

### 2. Docker Compose (Recommended for Testing)

**Use Case**: Consistent environments, CI/CD, integration testing

```bash
# Build and start all services
docker-compose up -d

# Scale specific services
docker-compose up -d --scale drone-agent=10

# Monitor logs
docker-compose logs -f swarm-coordinator
```

**Configuration**: `docker-compose.yml`

```yaml
version: '3.8'
services:
  swarm-coordinator:
    build: ./ros2_ws
    environment:
      - ROS_DOMAIN_ID=42
    ports:
      - "11311:11311"
    volumes:
      - ./config:/app/config
    restart: unless-stopped

  webrtc-bridge:
    build: ./ros2_ws
    environment:
      - BRIDGE_PORT=8443
      - SSL_ENABLED=false
    ports:
      - "8443:8443"
    depends_on:
      - swarm-coordinator
    restart: unless-stopped

  webapp:
    build: ./webapp
    environment:
      - NODE_ENV=production
      - REACT_APP_SWARM_WS_URL=ws://localhost:8443
    ports:
      - "3000:3000"
    depends_on:
      - webrtc-bridge
    restart: unless-stopped

  drone-agent:
    build: ./ros2_ws
    command: ros2 run robot_agents drone_agent.py
    environment:
      - ROS_DOMAIN_ID=42
      - AGENT_ID=drone_${HOSTNAME}
    depends_on:
      - swarm-coordinator
    restart: unless-stopped
```

### 3. Kubernetes Production Deployment

**Use Case**: Production deployments, high availability, auto-scaling

#### Prerequisites

```bash
# Install required tools
kubectl version --client
helm version

# Create namespace
kubectl create namespace xr-swarm-bridge
```

#### Core Services

**Swarm Coordinator Deployment**:

```yaml
# k8s/swarm-coordinator.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: swarm-coordinator
  namespace: xr-swarm-bridge
spec:
  replicas: 2  # HA setup
  selector:
    matchLabels:
      app: swarm-coordinator
  template:
    metadata:
      labels:
        app: swarm-coordinator
    spec:
      containers:
      - name: swarm-coordinator
        image: xr-swarm-bridge/swarm-coordinator:latest
        env:
        - name: ROS_DOMAIN_ID
          value: "42"
        ports:
        - containerPort: 11311
        resources:
          requests:
            memory: "512Mi"
            cpu: "500m"
          limits:
            memory: "1Gi"
            cpu: "1000m"
        livenessProbe:
          httpGet:
            path: /health
            port: 8080
          initialDelaySeconds: 30
          periodSeconds: 10
---
apiVersion: v1
kind: Service
metadata:
  name: swarm-coordinator
  namespace: xr-swarm-bridge
spec:
  selector:
    app: swarm-coordinator
  ports:
  - port: 11311
    targetPort: 11311
  type: ClusterIP
```

**WebRTC Bridge with Load Balancer**:

```yaml
# k8s/webrtc-bridge.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: webrtc-bridge
  namespace: xr-swarm-bridge
spec:
  replicas: 3  # Scale based on concurrent connections
  selector:
    matchLabels:
      app: webrtc-bridge
  template:
    metadata:
      labels:
        app: webrtc-bridge
    spec:
      containers:
      - name: webrtc-bridge
        image: xr-swarm-bridge/webrtc-bridge:latest
        env:
        - name: BRIDGE_PORT
          value: "8443"
        - name: SSL_CERT_PATH
          value: "/certs/tls.crt"
        - name: SSL_KEY_PATH
          value: "/certs/tls.key"
        ports:
        - containerPort: 8443
        volumeMounts:
        - name: tls-certs
          mountPath: "/certs"
          readOnly: true
        resources:
          requests:
            memory: "1Gi"
            cpu: "1000m"
          limits:
            memory: "2Gi"
            cpu: "2000m"
      volumes:
      - name: tls-certs
        secret:
          secretName: webrtc-bridge-tls
---
apiVersion: v1
kind: Service
metadata:
  name: webrtc-bridge
  namespace: xr-swarm-bridge
spec:
  selector:
    app: webrtc-bridge
  ports:
  - port: 8443
    targetPort: 8443
  type: LoadBalancer
  sessionAffinity: ClientIP  # Sticky sessions for WebRTC
```

**Webapp Frontend**:

```yaml
# k8s/webapp.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: webapp
  namespace: xr-swarm-bridge
spec:
  replicas: 3
  selector:
    matchLabels:
      app: webapp
  template:
    metadata:
      labels:
        app: webapp
    spec:
      containers:
      - name: webapp
        image: xr-swarm-bridge/webapp:latest
        env:
        - name: NODE_ENV
          value: "production"
        - name: REACT_APP_SWARM_WS_URL
          value: "wss://webrtc-bridge.xr-swarm-bridge.svc.cluster.local:8443"
        ports:
        - containerPort: 3000
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
---
apiVersion: v1
kind: Service
metadata:
  name: webapp
  namespace: xr-swarm-bridge
spec:
  selector:
    app: webapp
  ports:
  - port: 80
    targetPort: 3000
  type: LoadBalancer
```

**Auto-scaling Robot Agents**:

```yaml
# k8s/robot-agents.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: drone-agents
  namespace: xr-swarm-bridge
spec:
  replicas: 10  # Initial drone count
  selector:
    matchLabels:
      app: drone-agent
  template:
    metadata:
      labels:
        app: drone-agent
    spec:
      containers:
      - name: drone-agent
        image: xr-swarm-bridge/drone-agent:latest
        env:
        - name: ROS_DOMAIN_ID
          value: "42"
        - name: AGENT_ID
          valueFrom:
            fieldRef:
              fieldPath: metadata.name
        resources:
          requests:
            memory: "256Mi"
            cpu: "250m"
          limits:
            memory: "512Mi"
            cpu: "500m"
---
apiVersion: autoscaling/v2
kind: HorizontalPodAutoscaler
metadata:
  name: drone-agents-hpa
  namespace: xr-swarm-bridge
spec:
  scaleTargetRef:
    apiVersion: apps/v1
    kind: Deployment
    name: drone-agents
  minReplicas: 5
  maxReplicas: 100
  metrics:
  - type: Resource
    resource:
      name: cpu
      target:
        type: Utilization
        averageUtilization: 70
  - type: Resource
    resource:
      name: memory
      target:
        type: Utilization
        averageUtilization: 80
```

#### Deploy to Kubernetes

```bash
# Apply all configurations
kubectl apply -f k8s/

# Check deployment status
kubectl get pods -n xr-swarm-bridge
kubectl get services -n xr-swarm-bridge

# Scale drone agents
kubectl scale deployment drone-agents --replicas=50 -n xr-swarm-bridge

# View logs
kubectl logs -f deployment/swarm-coordinator -n xr-swarm-bridge
```

### 4. Cloud Provider Deployments

#### AWS EKS

```bash
# Create EKS cluster
eksctl create cluster \
  --name xr-swarm-bridge \
  --region us-west-2 \
  --nodegroup-name workers \
  --node-type m5.large \
  --nodes 3 \
  --nodes-min 1 \
  --nodes-max 10

# Deploy application
kubectl apply -f k8s/

# Configure load balancer
kubectl apply -f k8s/aws-load-balancer.yaml
```

#### Google GKE

```bash
# Create GKE cluster
gcloud container clusters create xr-swarm-bridge \
  --zone us-central1-a \
  --num-nodes 3 \
  --enable-autoscaling \
  --min-nodes 1 \
  --max-nodes 20

# Deploy application
kubectl apply -f k8s/

# Configure ingress
kubectl apply -f k8s/gcp-ingress.yaml
```

#### Microsoft AKS

```bash
# Create AKS cluster
az aks create \
  --resource-group xr-swarm-rg \
  --name xr-swarm-bridge \
  --node-count 3 \
  --enable-addons monitoring \
  --generate-ssh-keys

# Deploy application
kubectl apply -f k8s/

# Configure ingress
kubectl apply -f k8s/azure-ingress.yaml
```

## Configuration Management

### Environment Variables

**Core Services**:

```bash
# ROS Configuration
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=/app/config/fastrtps.xml

# WebRTC Bridge
export BRIDGE_PORT=8443
export SSL_ENABLED=true
export SSL_CERT_PATH=/certs/tls.crt
export SSL_KEY_PATH=/certs/tls.key
export MAX_CONNECTIONS=1000

# GPT-4o Integration
export OPENAI_API_KEY=your_api_key_here
export GPT_MODEL=gpt-4o
export GPT_MAX_TOKENS=2000

# Performance Tuning
export MAX_ROBOTS=100
export UPDATE_RATE=50
export VIDEO_QUALITY=high
export ENABLE_COMPRESSION=true
```

**Webapp Configuration**:

```bash
# API Endpoints
export REACT_APP_SWARM_WS_URL=wss://your-domain.com:8443
export REACT_APP_GPT_API_ENDPOINT=https://api.openai.com/v1

# Feature Flags
export REACT_APP_VR_ENABLED=true
export REACT_APP_AR_ENABLED=true
export REACT_APP_DEBUG_MODE=false

# Performance Settings
export REACT_APP_MAX_CONCURRENT_STREAMS=25
export REACT_APP_RENDER_QUALITY=high
export REACT_APP_PHYSICS_ENABLED=true
```

### Configuration Files

**ROS DDS Configuration** (`config/fastrtps.xml`):

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>16777216</sendBufferSize>
            <receiveBufferSize>16777216</receiveBufferSize>
        </transport_descriptor>
    </transport_descriptors>
    
    <participant profile_name="high_performance_participant">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <use_SIMPLE_EndpointDiscoveryProtocol>true</use_SIMPLE_EndpointDiscoveryProtocol>
                    <use_SIMPLE_ParticipantDiscoveryProtocol>true</use_SIMPLE_ParticipantDiscoveryProtocol>
                    <leaseDuration>
                        <sec>20</sec>
                        <nanosec>0</nanosec>
                    </leaseDuration>
                </discovery_config>
            </builtin>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
```

**System Limits** (`config/system-limits.conf`):

```ini
# Network Configuration
MAX_CONNECTIONS=1000
CONNECTION_TIMEOUT=30
KEEPALIVE_INTERVAL=5

# Performance Limits
MAX_CPU_USAGE=80
MAX_MEMORY_USAGE=85
MAX_DISK_USAGE=90

# Robot Limits
MAX_ROBOTS_PER_COORDINATOR=100
MAX_OPERATORS_PER_SESSION=10
MAX_CONCURRENT_MISSIONS=5

# Safety Limits
EMERGENCY_STOP_TIMEOUT=1000  # milliseconds
GEOFENCE_ENABLED=true
COLLISION_DETECTION_ENABLED=true
```

## Security Configuration

### SSL/TLS Setup

**Generate Certificates**:

```bash
# Self-signed for development
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes

# Let's Encrypt for production
certbot certonly --standalone -d your-domain.com
```

**Configure WebRTC Bridge**:

```bash
# Set certificate paths
export SSL_CERT_PATH=/etc/ssl/certs/cert.pem
export SSL_KEY_PATH=/etc/ssl/private/key.pem
export SSL_ENABLED=true
```

### Authentication Setup

**JWT Configuration**:

```bash
# Generate secret key
export JWT_SECRET=$(openssl rand -base64 32)
export JWT_EXPIRY=3600  # 1 hour

# Configure OAuth (optional)
export OAUTH_CLIENT_ID=your_client_id
export OAUTH_CLIENT_SECRET=your_client_secret
export OAUTH_REDIRECT_URI=https://your-domain.com/auth/callback
```

### Network Security

**Firewall Rules**:

```bash
# Allow required ports
sudo ufw allow 3000   # Webapp
sudo ufw allow 8443   # WebRTC Bridge
sudo ufw allow 11311  # ROS Master

# Block all other traffic
sudo ufw --force enable
```

**VPN Configuration** (Production):

```bash
# Configure WireGuard VPN
wg genkey | tee privatekey | wg pubkey > publickey

# Server configuration
cat > /etc/wireguard/wg0.conf << EOF
[Interface]
PrivateKey = $(cat privatekey)
Address = 10.0.0.1/24
ListenPort = 51820

[Peer]
PublicKey = client_public_key
AllowedIPs = 10.0.0.2/32
EOF

# Start VPN
systemctl enable wg-quick@wg0
systemctl start wg-quick@wg0
```

## Monitoring & Logging

### Prometheus Metrics

**Deploy Monitoring Stack**:

```bash
# Add Prometheus Helm repo
helm repo add prometheus-community https://prometheus-community.github.io/helm-charts
helm repo update

# Install Prometheus stack
helm install monitoring prometheus-community/kube-prometheus-stack \
  --namespace monitoring --create-namespace \
  --set grafana.adminPassword=admin123

# Configure ServiceMonitor for XR-Swarm-Bridge
kubectl apply -f k8s/monitoring/
```

**Key Metrics to Monitor**:

- Command latency (p50, p95, p99)
- WebRTC connection count and quality
- Robot agent health and battery levels
- System resource usage (CPU, memory, network)
- Error rates and recovery success rates

### Centralized Logging

**ELK Stack Setup**:

```bash
# Deploy Elasticsearch
helm install elasticsearch elastic/elasticsearch \
  --set replicas=3 \
  --set minimumMasterNodes=2

# Deploy Logstash
kubectl apply -f k8s/logging/logstash-config.yaml
helm install logstash elastic/logstash

# Deploy Kibana
helm install kibana elastic/kibana
```

**Log Configuration**:

```yaml
# logstash-config.yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: logstash-config
data:
  logstash.conf: |
    input {
      beats {
        port => 5044
      }
    }
    
    filter {
      if [kubernetes][container][name] == "swarm-coordinator" {
        grok {
          match => { "message" => "%{TIMESTAMP_ISO8601:timestamp} %{LOGLEVEL:level} %{GREEDYDATA:msg}" }
        }
      }
    }
    
    output {
      elasticsearch {
        hosts => ["elasticsearch:9200"]
        index => "xr-swarm-bridge-%{+YYYY.MM.dd}"
      }
    }
```

## Backup & Disaster Recovery

### Data Backup Strategy

```bash
# Database backups
kubectl create cronjob postgres-backup --image=postgres:13 \
  --schedule="0 2 * * *" \
  --restart=OnFailure \
  -- /bin/bash -c "pg_dump -h postgres -U user db > /backup/db-$(date +%Y%m%d).sql"

# Configuration backups
kubectl get configmaps -o yaml > configs-backup.yaml
kubectl get secrets -o yaml > secrets-backup.yaml
```

### Disaster Recovery Plan

1. **Automated Backups**
   - Database: Daily full, hourly incremental
   - Configuration: Version controlled
   - Application state: Event sourcing

2. **Multi-Region Setup**
   - Primary region: Active
   - Secondary region: Warm standby
   - Failover time: <5 minutes

3. **Recovery Procedures**
   - RTO (Recovery Time Objective): 15 minutes
   - RPO (Recovery Point Objective): 1 hour
   - Automated failover triggers

## Performance Tuning

### System Optimization

**Kernel Parameters**:

```bash
# /etc/sysctl.conf
net.core.rmem_max = 67108864
net.core.wmem_max = 67108864
net.ipv4.tcp_rmem = 4096 65536 67108864
net.ipv4.tcp_wmem = 4096 65536 67108864
net.core.netdev_max_backlog = 5000
```

**Docker Optimization**:

```json
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  },
  "default-ulimits": {
    "nofile": {
      "Name": "nofile",
      "Hard": 65536,
      "Soft": 65536
    }
  }
}
```

### Application Tuning

**ROS Performance**:

```bash
# Set CPU affinity for real-time threads
taskset -c 0,1 ros2 run xr_swarm_core swarm_coordinator.py

# Increase DDS limits
export FASTRTPS_DEFAULT_PROFILES_FILE=/app/config/fastrtps-performance.xml
```

**WebRTC Optimization**:

```bash
# Increase connection limits
export MAX_WEBRTC_CONNECTIONS=1000
export WEBRTC_BUFFER_SIZE=1048576

# Enable hardware acceleration
export ENABLE_HARDWARE_ACCELERATION=true
```

## Troubleshooting

### Common Issues

1. **High Latency**
   - Check network configuration
   - Verify WebRTC connection quality
   - Monitor system resource usage

2. **Connection Failures**
   - Verify firewall rules
   - Check SSL certificate validity
   - Validate DNS resolution

3. **Performance Degradation**
   - Scale up resources
   - Check for memory leaks
   - Optimize database queries

### Diagnostic Commands

```bash
# Check system status
kubectl get pods -n xr-swarm-bridge
kubectl top nodes
kubectl top pods -n xr-swarm-bridge

# Check logs
kubectl logs -f deployment/swarm-coordinator -n xr-swarm-bridge
kubectl logs -f deployment/webrtc-bridge -n xr-swarm-bridge

# Network diagnostics
kubectl exec -it pod-name -- netstat -tulpn
kubectl exec -it pod-name -- ss -tulpn

# Performance monitoring
kubectl exec -it pod-name -- top
kubectl exec -it pod-name -- iostat -x 1
```

## Maintenance

### Update Procedures

```bash
# Rolling update
kubectl set image deployment/swarm-coordinator \
  swarm-coordinator=xr-swarm-bridge/swarm-coordinator:v2.0 \
  -n xr-swarm-bridge

# Rollback if needed
kubectl rollout undo deployment/swarm-coordinator -n xr-swarm-bridge

# Monitor rollout
kubectl rollout status deployment/swarm-coordinator -n xr-swarm-bridge
```

### Scaling Operations

```bash
# Scale up for high load
kubectl scale deployment drone-agents --replicas=100 -n xr-swarm-bridge

# Scale down during maintenance
kubectl scale deployment drone-agents --replicas=5 -n xr-swarm-bridge

# Auto-scaling configuration
kubectl autoscale deployment drone-agents \
  --cpu-percent=70 --min=10 --max=100 \
  -n xr-swarm-bridge
```

This deployment guide provides comprehensive coverage for all deployment scenarios from development to production-scale operations. Choose the appropriate deployment method based on your specific requirements and scale.