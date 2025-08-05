# XR-Swarm-Bridge Production Deployment Guide

## 🚀 Overview

This guide covers the production deployment of XR-Swarm-Bridge, an immersive multi-agent telepresence platform for controlling 100+ robots through VR/AR interfaces with GPT-4o integration.

## 📋 Prerequisites

### System Requirements

- **CPU**: 16+ cores (Intel Xeon or AMD EPYC recommended)
- **RAM**: 32GB+ (64GB recommended for 100+ agents)
- **Storage**: 500GB+ SSD (NVMe preferred)
- **Network**: 10Gbps+ with low latency (<10ms to agents)
- **GPU**: NVIDIA RTX 4090 or better (for XR processing)

### Software Requirements

- **OS**: Ubuntu 22.04 LTS or RHEL 8+
- **Docker**: 24.0+ with Docker Compose v2
- **ROS 2**: Humble Hawksbill
- **SSL Certificates**: Valid certificates for HTTPS/WSS
- **Domain**: Configured DNS for your deployment

## 🔧 Pre-Deployment Setup

### 1. Clone and Configure

```bash
git clone https://github.com/danieleschmidt/xr-swarm-bridge.git
cd xr-swarm-bridge

# Copy environment template
cp .env.example .env
```

### 2. SSL Certificates

```bash
# Create SSL directory
mkdir -p ssl

# Generate self-signed certificates (development)
openssl req -x509 -nodes -days 365 -newkey rsa:4096 \
  -keyout ssl/server.key \
  -out ssl/server.crt \
  -subj "/C=US/ST=CA/L=SF/O=XR-Swarm/CN=*.xr-swarm.local"

# For production, use Let's Encrypt or your CA certificates
# Place your certificates as:
# ssl/server.crt
# ssl/server.key
```

### 3. Secrets Configuration

```bash
# Create secrets directory
mkdir -p secrets

# Generate secure passwords
openssl rand -base64 32 > secrets/db_password.txt
openssl rand -base64 32 > secrets/grafana_password.txt
openssl rand -base64 32 > secrets/redis_password.txt

# Set proper permissions
chmod 600 secrets/*.txt
```

### 4. Configuration Files

Create the following configuration files:

#### `config/production.env`
```bash
# System Configuration
MAX_AGENTS=1000
DEFAULT_FORMATION=grid
SAFETY_TIMEOUT=30000
EMERGENCY_STOP_TIMEOUT=5000

# GPT-4o Integration
OPENAI_API_KEY=your_api_key_here
OPENAI_MODEL=gpt-4o
OPENAI_MAX_TOKENS=2000

# WebRTC Configuration
WEBRTC_SSL_ENABLED=true
STUN_SERVERS=stun:stun.l.google.com:19302,stun:stun1.l.google.com:19302
TURN_SERVER=turn:your-turn-server.com:3478
TURN_USERNAME=your_username
TURN_PASSWORD=your_password

# Performance Tuning
CACHE_SIZE_MB=1024
LOG_LEVEL=info
METRICS_ENABLED=true
TELEMETRY_BUFFER_SIZE=10000

# Security
SESSION_TIMEOUT=3600000
MAX_FAILED_ATTEMPTS=5
RATE_LIMIT_REQUESTS=100
CORS_ORIGINS=https://your-domain.com
```

#### `nginx/nginx.conf`
```nginx
events {
    worker_connections 1024;
}

http {
    upstream frontend {
        server frontend:3000;
    }
    
    upstream webrtc {
        server webrtc-bridge:8443;
    }

    server {
        listen 443 ssl http2;
        server_name your-domain.com;
        
        ssl_certificate /etc/ssl/certs/server.crt;
        ssl_certificate_key /etc/ssl/certs/server.key;
        ssl_protocols TLSv1.2 TLSv1.3;
        ssl_ciphers HIGH:!aNULL:!MD5;
        
        # Frontend
        location / {
            proxy_pass http://frontend;
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
            proxy_set_header X-Forwarded-Proto $scheme;
        }
        
        # WebRTC WebSocket
        location /ws {
            proxy_pass http://webrtc;
            proxy_http_version 1.1;
            proxy_set_header Upgrade $http_upgrade;
            proxy_set_header Connection "upgrade";
            proxy_set_header Host $host;
            proxy_set_header X-Real-IP $remote_addr;
            proxy_read_timeout 86400;
        }
        
        # Health check
        location /health {
            access_log off;
            return 200 "healthy\n";
            add_header Content-Type text/plain;
        }
    }
    
    # HTTP to HTTPS redirect
    server {
        listen 80;
        server_name your-domain.com;
        return 301 https://$server_name$request_uri;
    }
}
```

### 5. Monitoring Configuration

#### `monitoring/prometheus.yml`
```yaml
global:
  scrape_interval: 15s
  evaluation_interval: 15s

rule_files:
  - "rules/*.yml"

scrape_configs:
  - job_name: 'xr-swarm-frontend'
    static_configs:
      - targets: ['frontend:3000']
    metrics_path: /metrics
    scrape_interval: 10s
    
  - job_name: 'xr-swarm-backend'
    static_configs:
      - targets: ['ros2-coordinator:8080']
    metrics_path: /metrics
    scrape_interval: 5s
    
  - job_name: 'webrtc-bridge'
    static_configs:
      - targets: ['webrtc-bridge:9090']
    metrics_path: /metrics
    scrape_interval: 5s
    
  - job_name: 'system-metrics'
    static_configs:
      - targets: ['health-monitor:9100']
    scrape_interval: 10s

alerting:
  alertmanagers:
    - static_configs:
        - targets:
          - alertmanager:9093
```

## 🚀 Deployment Steps

### 1. Build and Deploy

```bash
# Set environment variables
export COMPOSE_FILE=docker-compose.production.yml
export COMPOSE_PROJECT_NAME=xr-swarm-prod

# Build and start services
docker-compose -f docker-compose.production.yml build
docker-compose -f docker-compose.production.yml up -d

# Check service status
docker-compose -f docker-compose.production.yml ps
```

### 2. Initialize Database

```bash
# Wait for database to be ready
docker-compose -f docker-compose.production.yml exec database pg_isready -U swarm_user

# Run migrations
docker-compose -f docker-compose.production.yml exec database psql -U swarm_user -d xr_swarm -f /docker-entrypoint-initdb.d/init.sql
```

### 3. Verify Deployment

```bash
# Check all services are healthy
docker-compose -f docker-compose.production.yml ps

# Test endpoints
curl -k https://localhost/health
curl -k https://localhost:8443/ws

# Check logs
docker-compose -f docker-compose.production.yml logs -f
```

### 4. Performance Tuning

```bash
# System-level optimizations
echo 'net.core.rmem_max = 268435456' >> /etc/sysctl.conf
echo 'net.core.wmem_max = 268435456' >> /etc/sysctl.conf
echo 'fs.file-max = 1000000' >> /etc/sysctl.conf
sysctl -p

# Docker optimizations
echo '{"log-driver": "json-file", "log-opts": {"max-size": "10m", "max-file": "3"}}' > /etc/docker/daemon.json
systemctl restart docker
```

## 📊 Monitoring and Observability

### Dashboards

- **Grafana**: https://your-domain.com:3001
- **Prometheus**: https://your-domain.com:9090
- **Application**: https://your-domain.com

### Key Metrics to Monitor

1. **System Performance**
   - CPU usage per service
   - Memory consumption
   - Network throughput
   - Disk I/O

2. **Application Metrics**
   - Active agent count
   - WebRTC connection quality
   - Command response times
   - Error rates

3. **Network Performance**
   - Latency to agents
   - Packet loss rates
   - Bandwidth utilization
   - Connection stability

### Alerting Rules

Critical alerts are configured for:
- Service downtime
- High error rates (>5%)
- Memory usage >85%
- CPU usage >90%
- Network latency >200ms
- Agent connection failures

## 🔒 Security Considerations

### Network Security

```bash
# Firewall configuration
ufw allow 22/tcp      # SSH
ufw allow 80/tcp      # HTTP
ufw allow 443/tcp     # HTTPS
ufw allow 8443/tcp    # WebRTC
ufw allow 3478/udp    # STUN
ufw allow 5349/tcp    # TURN
ufw allow 49152:49252/udp  # RTP
ufw enable
```

### SSL/TLS Configuration

- Use TLS 1.2+ only
- Strong cipher suites
- HSTS headers
- Certificate pinning for critical connections

### Authentication & Authorization

- Multi-factor authentication enabled
- Role-based access control (RBAC)
- API key rotation every 90 days
- Session timeout enforcement

## 🔧 Maintenance

### Regular Tasks

1. **Daily**
   - Check service health
   - Monitor resource usage
   - Review error logs

2. **Weekly**
   - Update security patches
   - Rotate log files
   - Backup database

3. **Monthly**
   - Update dependencies
   - Performance review
   - Security audit

### Backup Strategy

```bash
# Database backup
docker-compose -f docker-compose.production.yml exec database pg_dump -U swarm_user xr_swarm > backup_$(date +%Y%m%d).sql

# Configuration backup
tar -czf config_backup_$(date +%Y%m%d).tar.gz config/ secrets/ ssl/

# Automated backup script
cat > backup.sh << 'EOF'
#!/bin/bash
DATE=$(date +%Y%m%d_%H%M%S)
BACKUP_DIR="/backup/xr-swarm"

mkdir -p $BACKUP_DIR

# Database
docker-compose -f docker-compose.production.yml exec -T database pg_dump -U swarm_user xr_swarm > $BACKUP_DIR/db_$DATE.sql

# Configuration
tar -czf $BACKUP_DIR/config_$DATE.tar.gz config/ secrets/ ssl/

# Cleanup old backups (keep 30 days)
find $BACKUP_DIR -name "*.sql" -mtime +30 -delete
find $BACKUP_DIR -name "*.tar.gz" -mtime +30 -delete
EOF

chmod +x backup.sh
```

### Scaling

For scaling beyond 1000 agents:

1. **Horizontal Scaling**
   - Deploy multiple coordinator instances
   - Use load balancer for WebRTC connections
   - Shard database by agent groups

2. **Resource Scaling**
   - Increase container resources
   - Add more worker nodes
   - Optimize network topology

### Troubleshooting

Common issues and solutions:

1. **High Latency**
   - Check network configuration
   - Optimize WebRTC settings
   - Review server load

2. **Connection Drops**
   - Verify SSL certificates
   - Check firewall rules
   - Monitor resource usage

3. **Performance Issues**
   - Review logs for errors
   - Check database queries
   - Monitor cache hit rates

## 📞 Support

For production support:
- Documentation: See `/docs` directory
- Monitoring: Grafana dashboards
- Logs: Centralized in Loki
- Metrics: Prometheus endpoints

## 🔄 Updates

To update the deployment:

```bash
# Pull latest changes
git pull origin main

# Rebuild and restart services
docker-compose -f docker-compose.production.yml build
docker-compose -f docker-compose.production.yml up -d --remove-orphans

# Verify update
docker-compose -f docker-compose.production.yml ps
```

---

**Note**: This deployment supports up to 1000 concurrent robot agents with sub-200ms latency. For larger deployments, contact the development team for enterprise scaling solutions.