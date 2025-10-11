#!/bin/bash
# ===================================
# Tempo Insights Instance Bootstrap
# ===================================
# Configures a fresh Ubuntu 22.04 server for Tempo Insights deployment
# 
# Usage: 
#   wget https://raw.githubusercontent.com/rrainey/tempo/refs/heads/main/desktop/tempo-insights/scripts/configure-instance.sh
#   chmod +x configure-instance.sh
#   sudo ./configure-instance.sh
#
# This script:
# - Installs required system packages
# - Configures Docker and BlueZ
# - Clones the tempo-insights repository
# - Sets up environment-specific configuration
# - Initializes Supabase stack
# - Deploys database schema
# - Starts all services

set -e

# Check if running as root/sudo
if [ "$EUID" -ne 0 ]; then 
    echo "ERROR: This script must be run with sudo"
    echo "Usage: sudo ./configure-instance.sh"
    exit 1
fi

# Get the actual user (not root when using sudo)
ACTUAL_USER="${SUDO_USER:-$USER}"
ACTUAL_HOME=$(eval echo ~$ACTUAL_USER)

echo "=========================================="
echo "Tempo Insights Instance Bootstrap"
echo "=========================================="
echo "Running as: $ACTUAL_USER"
echo "Install directory: $ACTUAL_HOME/tempo-insights"
echo ""

# ===================================
# STEP 1: Environment Selection
# ===================================
echo "Step 1: Select Environment Type"
echo "-----------------------------------"
echo "1) Development (local testing, weak secrets OK)"
echo "2) QA/Staging (team testing, documented secrets)"
echo "3) Production (live system, maximum security)"
echo ""
read -p "Select environment [1-3]: " ENV_CHOICE

case $ENV_CHOICE in
    1)
        ENVIRONMENT="development"
        ENV_SHORT="dev"
        ;;
    2)
        ENVIRONMENT="qa"
        ENV_SHORT="qa"
        ;;
    3)
        ENVIRONMENT="production"
        ENV_SHORT="prod"
        ;;
    *)
        echo "Invalid choice. Exiting."
        exit 1
        ;;
esac

echo "✓ Selected: $ENVIRONMENT"
echo ""

# ===================================
# STEP 2: System Package Installation
# ===================================
echo "Step 2: Installing System Dependencies"
echo "-----------------------------------"

apt-get update

echo "Installing core packages..."
apt-get install -y \
    curl \
    wget \
    git \
    ca-certificates \
    gnupg \
    lsb-release \
    ufw \
    bluez \
    libbluetooth-dev \
    bluetooth

echo "✓ Core packages installed"

# Install Docker
if ! command -v docker &> /dev/null; then
    echo "Installing Docker..."
    
    # Add Docker's official GPG key
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    chmod a+r /etc/apt/keyrings/docker.gpg
    
    # Add Docker repository
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    apt-get update
    apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    
    # Add user to docker group
    usermod -aG docker $ACTUAL_USER
    
    echo "✓ Docker installed"
else
    echo "✓ Docker already installed"
fi

# Install Node.js 20
if ! command -v node &> /dev/null || [ "$(node -v | cut -d'.' -f1 | sed 's/v//')" -lt 20 ]; then
    echo "Installing Node.js 20..."
    curl -fsSL https://deb.nodesource.com/setup_20.x | bash -
    apt-get install -y nodejs
    echo "✓ Node.js installed"
else
    echo "✓ Node.js 20+ already installed"
fi

# Install pnpm
if ! command -v pnpm &> /dev/null; then
    echo "Installing pnpm..."
    npm install -g pnpm
    echo "✓ pnpm installed"
else
    echo "✓ pnpm already installed"
fi

# Enable and start Bluetooth
systemctl enable bluetooth
systemctl start bluetooth
echo "✓ Bluetooth service enabled"

echo ""

# ===================================
# STEP 3: Clone Repository
# ===================================
echo "Step 3: Cloning Repository"
echo "-----------------------------------"

REPO_URL="https://github.com/rrainey/tempo-insights.git"
INSTALL_DIR="$ACTUAL_HOME/tempo-insights"

read -p "Repository URL [$REPO_URL]: " INPUT_REPO
REPO_URL="${INPUT_REPO:-$REPO_URL}"

if [ -d "$INSTALL_DIR" ]; then
    echo "⚠️  Directory $INSTALL_DIR already exists"
    read -p "Remove and re-clone? (y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$INSTALL_DIR"
    else
        echo "Using existing directory"
    fi
fi

if [ ! -d "$INSTALL_DIR" ]; then
    echo "Cloning repository..."
    su - $ACTUAL_USER -c "git clone $REPO_URL $INSTALL_DIR"
    echo "✓ Repository cloned"
else
    echo "✓ Using existing repository"
fi

cd "$INSTALL_DIR"
echo ""

# ===================================
# STEP 4: Generate Secrets (Tasks 142-143)
# ===================================
echo "Step 4: Generating Security Secrets"
echo "-----------------------------------"

# Generate strong secrets
POSTGRES_PASSWORD=$(openssl rand -base64 32 | tr -d "=+/" | cut -c1-32)
JWT_SECRET=$(openssl rand -base64 48 | tr -d "=+/" | cut -c1-64)

echo "Generated secrets:"
echo "  - POSTGRES_PASSWORD: ***REDACTED***"
echo "  - JWT_SECRET: ***REDACTED***"
echo ""

# For production, prompt for dashboard credentials
if [ "$ENV_SHORT" = "prod" ]; then
    echo "Enter Supabase Studio dashboard credentials:"
    read -p "Dashboard Username: " DASHBOARD_USER
    read -sp "Dashboard Password: " DASHBOARD_PASS
    echo ""
else
    DASHBOARD_USER="admin"
    DASHBOARD_PASS="admin123"
    echo "Using default dashboard credentials (admin/admin123)"
fi

# Save secrets to secure location
SECRETS_FILE="$INSTALL_DIR/.secrets.$ENV_SHORT"
cat > "$SECRETS_FILE" <<EOF
# Tempo Insights Secrets - $ENVIRONMENT Environment
# Generated: $(date)
# KEEP THIS FILE SECURE - DO NOT COMMIT TO GIT

POSTGRES_PASSWORD=$POSTGRES_PASSWORD
JWT_SECRET=$JWT_SECRET
DASHBOARD_USERNAME=$DASHBOARD_USER
DASHBOARD_PASSWORD=$DASHBOARD_PASS

# These will be generated from JWT_SECRET:
# ANON_KEY=(see supabase-stack/.env)
# SERVICE_ROLE_KEY=(see supabase-stack/.env)
EOF

chmod 600 "$SECRETS_FILE"
chown $ACTUAL_USER:$ACTUAL_USER "$SECRETS_FILE"

echo "✓ Secrets saved to $SECRETS_FILE"
echo "⚠️  IMPORTANT: Backup this file securely!"
echo ""

# ===================================
# STEP 5: Setup Supabase Stack
# ===================================
echo "Step 5: Setting Up Supabase Stack"
echo "-----------------------------------"

# Clone Supabase Docker config
SUPABASE_STACK_DIR="$INSTALL_DIR/supabase-stack"

if [ ! -d "$SUPABASE_STACK_DIR" ]; then
    echo "Downloading Supabase Docker configuration..."
    su - $ACTUAL_USER -c "cd $INSTALL_DIR && git clone --depth 1 https://github.com/supabase/supabase supabase-temp"
    su - $ACTUAL_USER -c "cd $INSTALL_DIR && mkdir -p supabase-stack && cp -r supabase-temp/docker/* supabase-stack/"
    su - $ACTUAL_USER -c "cd $INSTALL_DIR && rm -rf supabase-temp"
    echo "✓ Supabase configuration downloaded"
fi

# Configure Supabase .env
if [ ! -f "$SUPABASE_STACK_DIR/.env" ]; then
    cp "$SUPABASE_STACK_DIR/.env.example" "$SUPABASE_STACK_DIR/.env"
    
    # Update with generated secrets
    sed -i "s|your-super-secret-and-long-postgres-password|$POSTGRES_PASSWORD|g" "$SUPABASE_STACK_DIR/.env"
    sed -i "s|your-super-secret-jwt-token-with-at-least-32-characters-long|$JWT_SECRET|g" "$SUPABASE_STACK_DIR/.env"
    sed -i "s|DASHBOARD_USERNAME=.*|DASHBOARD_USERNAME=$DASHBOARD_USER|g" "$SUPABASE_STACK_DIR/.env"
    sed -i "s|DASHBOARD_PASSWORD=.*|DASHBOARD_PASSWORD=$DASHBOARD_PASS|g" "$SUPABASE_STACK_DIR/.env"
    
    echo "✓ Supabase .env configured"
    echo "⚠️  Note: You must still generate ANON_KEY and SERVICE_ROLE_KEY"
    echo "   Visit: https://supabase.com/docs/guides/self-hosting/docker#generate-api-keys"
fi

# ===================================
# STEP 6: Create Volumes (Task 144)
# ===================================
echo ""
echo "Step 6: Creating Persistent Volumes"
echo "-----------------------------------"

VOLUMES_DIR="$SUPABASE_STACK_DIR/volumes"
mkdir -p "$VOLUMES_DIR"/{db,storage,functions,api,logs}
chown -R $ACTUAL_USER:$ACTUAL_USER "$VOLUMES_DIR"
chmod -R 755 "$VOLUMES_DIR"

echo "✓ Volume directories created at $VOLUMES_DIR"
echo ""

# ===================================
# STEP 7: Firewall Configuration
# ===================================
echo "Step 7: Configuring Firewall"
echo "-----------------------------------"

if command -v ufw &> /dev/null; then
    # Allow SSH
    ufw allow 22/tcp
    
    # Allow web traffic
    ufw allow 3000/tcp  # Next.js app
    ufw allow 8000/tcp  # Supabase Kong gateway
    
    # Enable firewall
    echo "y" | ufw enable
    
    echo "✓ Firewall configured"
    ufw status
else
    echo "⚠️  UFW not available, skipping firewall setup"
fi

echo ""

# ===================================
# STEP 8: Install Application Dependencies
# ===================================
echo "Step 8: Installing Application Dependencies"
echo "-----------------------------------"

cd "$INSTALL_DIR"
su - $ACTUAL_USER -c "cd $INSTALL_DIR && pnpm install"

echo "✓ Application dependencies installed"
echo ""

# ===================================
# STEP 9: Start Supabase Stack
# ===================================
echo "Step 9: Starting Supabase Stack"
echo "-----------------------------------"

cd "$SUPABASE_STACK_DIR"
su - $ACTUAL_USER -c "cd $SUPABASE_STACK_DIR && docker compose up -d"

echo "Waiting for services to start (60 seconds)..."
sleep 60

echo "✓ Supabase stack started"
docker compose ps
echo ""

# ===================================
# STEP 10: Database Setup (Tasks 147-148)
# ===================================
echo "Step 10: Setting Up Database Schema"
echo "-----------------------------------"

# Note: Supabase exposes Postgres on port 54322 by default
DATABASE_URL="postgresql://postgres:$POSTGRES_PASSWORD@localhost:54322/postgres"

echo "Running Prisma migrations..."
su - $ACTUAL_USER -c "cd $INSTALL_DIR && DATABASE_URL='$DATABASE_URL' pnpm prisma migrate deploy"

echo "✓ Database schema created"

# Seed data based on environment
echo "Seeding database..."
if [ "$ENV_SHORT" = "prod" ]; then
    # Production: minimal seed (admin user only)
    su - $ACTUAL_USER -c "cd $INSTALL_DIR && DATABASE_URL='$DATABASE_URL' NODE_ENV=production pnpm prisma db seed"
else
    # Dev/QA: full test data
    su - $ACTUAL_USER -c "cd $INSTALL_DIR && DATABASE_URL='$DATABASE_URL' NODE_ENV=$ENV_SHORT pnpm prisma db seed"
fi

echo "✓ Database seeded for $ENVIRONMENT environment"
echo ""

# ===================================
# STEP 11: Build Docker Images
# ===================================
echo "Step 11: Building Application Docker Images"
echo "-----------------------------------"

cd "$INSTALL_DIR"

echo "Building web server image..."
su - $ACTUAL_USER -c "cd $INSTALL_DIR && docker build --platform linux/arm64 -f docker/web/Dockerfile -t tempo-web:latest ."

echo "Building bluetooth scanner image..."
su - $ACTUAL_USER -c "cd $INSTALL_DIR && docker build --platform linux/arm64 -f docker/bluetooth-scanner/Dockerfile -t tempo-bt-scanner:latest ."

echo "Building analysis worker image..."
su - $ACTUAL_USER -c "cd $INSTALL_DIR && docker build --platform linux/arm64 -f docker/analysis-worker/Dockerfile -t tempo-analysis:latest ."

echo "✓ Docker images built"
echo ""

# ===================================
# STEP 12: Configure Application Environment
# ===================================
echo "Step 12: Configuring Application Environment"
echo "-----------------------------------"

# Create .env.docker if it doesn't exist
if [ ! -f "$INSTALL_DIR/.env.docker" ]; then
    cp "$INSTALL_DIR/.env.example" "$INSTALL_DIR/.env.docker"
    
    # Update with actual values
    sed -i "s|JWT_SECRET=.*|JWT_SECRET=$JWT_SECRET|g" "$INSTALL_DIR/.env.docker"
    sed -i "s|DATABASE_URL=.*|DATABASE_URL=postgresql://postgres:$POSTGRES_PASSWORD@db:5432/postgres|g" "$INSTALL_DIR/.env.docker"
    
    echo "✓ .env.docker configured"
fi

echo ""

# ===================================
# FINAL: Summary
# ===================================
echo "=========================================="
echo "Instance Configuration Complete!"
echo "=========================================="
echo ""
echo "Environment: $ENVIRONMENT"
echo "Install Location: $INSTALL_DIR"
echo "Secrets File: $SECRETS_FILE"
echo ""
echo "Services Status:"
echo "-----------------------------------"
cd "$SUPABASE_STACK_DIR"
docker compose ps
echo ""
echo "Next Steps:"
echo "1. Complete Supabase JWT key generation:"
echo "   - Visit: https://supabase.com/docs/guides/self-hosting/docker#generate-api-keys"
echo "   - Update $SUPABASE_STACK_DIR/.env with ANON_KEY and SERVICE_ROLE_KEY"
echo ""
echo "2. Start application containers:"
echo "   cd $INSTALL_DIR"
echo "   docker compose up -d"
echo ""
echo "3. Access services:"
echo "   - Supabase Studio: http://$(hostname -I | awk '{print $1}'):8000"
echo "   - Tempo Insights: http://$(hostname -I | awk '{print $1}'):3000"
echo ""
echo "4. IMPORTANT: Backup the secrets file securely!"
echo "   cp $SECRETS_FILE /secure/backup/location/"
echo ""
echo "=========================================="