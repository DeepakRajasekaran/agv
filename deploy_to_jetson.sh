#!/bin/bash
# Helper script to build and deploy the AGV hardware Docker image to NVIDIA Jetson.
# Ensure you have SSH access to the Jetson before running.

set -e

JETSON_IP="192.168.1.83"
JETSON_USER="nvidia"
JETSON_DIR="/home/nvidia/agv"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== AGV Jetson Hardware Image Deployer ===${NC}"
echo "Target: $JETSON_USER@$JETSON_IP"
echo "Target Directory: $JETSON_DIR"
echo ""

# Function to check and wait for Jetson connection
check_and_wait_for_jetson() {
    echo -n "Checking network connection to Jetson... "
    while ! ping -c 1 -W 2 "$JETSON_IP" > /dev/null 2>&1; do
        echo -e "${RED}OFFLINE${NC}"
        echo -e "${YELLOW}Please connect to the robot network (IP: $JETSON_IP) and press Enter to continue...${NC}"
        read -r
        echo -n "Re-checking network connection... "
    done
    echo -e "${GREEN}ONLINE${NC}"
}

# Check for upstream Git changes
echo -n "Checking for updates from GitHub... "
git fetch origin > /dev/null 2>&1 || true
LOCAL=$(git rev-parse HEAD 2>/dev/null || echo "")
REMOTE=$(git rev-parse origin/main 2>/dev/null || echo "")

if [ -n "$LOCAL" ] && [ -n "$REMOTE" ]; then
    if [ "$LOCAL" != "$REMOTE" ]; then
        echo -e "${YELLOW}Changes detected! Pulling latest code...${NC}"
        git pull origin main
    else
        echo -e "${GREEN}Up to date.${NC}"
    fi
else
    echo -e "${YELLOW}Could not verify Git status. Skipping pull.${NC}"
fi

echo ""
# Sourced variables
WS_CHOICE="c"

echo ""
echo "Select Deployment Method:"
echo "1) Sync source and build natively on Jetson (Recommended, fast, simple)"
echo "2) Cross-compile on Host (Buildx arm64) and push image tarball"
echo "3) Cancel"
echo -n "Enter choice [1-3]: "
read -r CHOICE

case $CHOICE in
    1)
        echo -e "\n${YELLOW}Starting Native Build Method...${NC}"
        
        # 1. Sync source code
        echo "Step 1: Syncing workspace source files via rsync..."
        check_and_wait_for_jetson
        ssh "$JETSON_USER@$JETSON_IP" "mkdir -p $JETSON_DIR"
        rsync -avz --delete \
            --exclude 'build/' \
            --exclude 'install/' \
            --exclude 'log/' \
            --exclude '.git/' \
            --exclude 'test/' \
            --exclude '*.tar' \
            ./ "$JETSON_USER@$JETSON_IP:$JETSON_DIR"
            
        # 2. Build on target
        echo -e "\nStep 2: Triggering Docker build natively on the Jetson..."
        ssh -t "$JETSON_USER@$JETSON_IP" "cd $JETSON_DIR && source ./agv_env.bash && docker compose -f docker/docker-compose.yml build"
        
        # 3. Cleanup source code
        echo -e "\nStep 3: Cleaning up source code from Jetson host (leaving only the container)..."
        ssh -t "$JETSON_USER@$JETSON_IP" "mkdir -p $JETSON_DIR/robot_ws/config && mv $JETSON_DIR/robot_ws/config $JETSON_DIR/config_tmp && rm -rf $JETSON_DIR/robot_ws && mkdir -p $JETSON_DIR/robot_ws && mv $JETSON_DIR/config_tmp $JETSON_DIR/robot_ws/config"
        
        echo -e "\n${GREEN}Build completed successfully on Jetson!${NC}"
        echo "To run the container on Jetson, execute:"
        echo "  ssh $JETSON_USER@$JETSON_IP 'cd $JETSON_DIR && source ./agv_env.bash && docker compose -f docker/docker-compose.yml up -d'"
        ;;
        
    2)
        echo -e "\n${YELLOW}Starting Cross-compile Method...${NC}"
        
        # 1. Check/Configure buildx
        echo "Step 1: Checking docker buildx builder..."
        if ! docker buildx inspect jetson_builder > /dev/null 2>&1; then
            echo "Creating a new buildx builder instance: jetson_builder..."
            docker buildx create --name jetson_builder --driver-opt network=host --use
        else
            docker buildx use jetson_builder
        fi
        docker buildx inspect --bootstrap
        
        # 2. Build ARM64 image on host
        echo -e "\nStep 2: Building linux/arm64 Docker image on host..."
        source ./agv_env.bash
        docker buildx build \
            --platform linux/arm64 \
            -t agv:latest \
            -f docker/Dockerfile \
            --target ${BUILD_MODE:-deployment} \
            --load .
            
        echo "Exporting image using docker save..."
        docker save agv:latest -o agv.tar
            
        # 3. Package config files
        echo -e "\nStep 3: Packaging configuration files..."
        tar -czf config.tar.gz docker/docker-compose.yml agv_env.bash robot_ws/config/ 2>/dev/null || true
            
        # 4. Transfer files to target
        echo -e "\nStep 4: Transferring files to Jetson..."
        check_and_wait_for_jetson
        scp agv.tar config.tar.gz "$JETSON_USER@$JETSON_IP:/home/$JETSON_USER/"
        
        # 5. Load image and extract configs on target
        echo -e "\nStep 5: Loading image and extracting configurations on Jetson..."
        ssh "$JETSON_USER@$JETSON_IP" "docker load -i /home/$JETSON_USER/agv.tar && rm /home/$JETSON_USER/agv.tar && rm -rf $JETSON_DIR/* && mkdir -p $JETSON_DIR && tar -xzf /home/$JETSON_USER/config.tar.gz -C $JETSON_DIR && rm /home/$JETSON_USER/config.tar.gz"
        
        echo -e "\n${GREEN}Cross-build and transfer completed successfully!${NC}"
        echo "To run the container on Jetson, execute:"
        echo "  ssh $JETSON_USER@$JETSON_IP 'cd $JETSON_DIR && source ./agv_env.bash && docker compose -f docker/docker-compose.yml up -d'"
        
        # Clean up local files
        rm -f agv.tar config.tar.gz
        ;;
        
    3|*)
        echo "Deployment cancelled."
        exit 0
        ;;
esac
