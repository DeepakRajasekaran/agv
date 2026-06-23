#!/bin/bash
# Helper script to build and deploy the AGV hardware Docker image to NVIDIA Jetson.
# Ensure you have SSH access to the Jetson before running.

set -e

JETSON_IP="192.168.1.83"
JETSON_USER="nvidia"
JETSON_DIR="/home/nvidia/anscer_workspace/agv"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== AGV Jetson Hardware Image Deployer ===${NC}"
echo "Target: $JETSON_USER@$JETSON_IP"
echo "Target Directory: $JETSON_DIR"
echo ""

# Check if Jetson is online
echo -n "Checking network connection to Jetson... "
if ping -c 1 -W 2 "$JETSON_IP" > /dev/null 2>&1; then
    echo -e "${GREEN}ONLINE${NC}"
else
    echo -e "${RED}OFFLINE${NC}"
    echo "Please verify the Jetson is powered on, connected to the network, and the IP is correct ($JETSON_IP)."
    exit 1
fi

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
        ssh -t "$JETSON_USER@$JETSON_IP" "cd $JETSON_DIR && docker compose -f docker/docker-compose.yml build"
        
        echo -e "\n${GREEN}Build completed successfully on Jetson!${NC}"
        echo "To run the container on Jetson, execute:"
        echo "  ssh $JETSON_USER@$JETSON_IP 'cd $JETSON_DIR && docker compose -f docker/docker-compose.yml up -d'"
        ;;
        
    2)
        echo -e "\n${YELLOW}Starting Cross-compile Method...${NC}"
        
        # 1. Check/Configure buildx
        echo "Step 1: Checking docker buildx builder..."
        if ! docker buildx inspect jetson_builder > /dev/null 2>&1; then
            echo "Creating a new buildx builder instance: jetson_builder..."
            docker buildx create --name jetson_builder --use
        else
            docker buildx use jetson_builder
        fi
        docker buildx inspect --bootstrap
        
        # 2. Build ARM64 image on host
        echo -e "\nStep 2: Building linux/arm64 Docker image on host..."
        docker buildx build \
            --platform linux/arm64 \
            -t roboteq_driver_hardware:latest \
            -f docker/Dockerfile \
            --load .
            
        echo "Exporting image using docker save..."
        docker save roboteq_driver_hardware:latest -o roboteq_driver_hardware.tar
            
        # 3. Transfer image to target
        echo -e "\nStep 3: Transferring image tarball to Jetson..."
        scp roboteq_driver_hardware.tar "$JETSON_USER@$JETSON_IP:/home/$JETSON_USER/"
        
        # 4. Load image on target
        echo -e "\nStep 4: Loading image into Jetson's Docker daemon..."
        ssh "$JETSON_USER@$JETSON_IP" "docker load -i /home/$JETSON_USER/roboteq_driver_hardware.tar && rm /home/$JETSON_USER/roboteq_driver_hardware.tar"
        
        # 5. Sync Docker Compose config to run it easily
        echo -e "\nStep 5: Syncing Docker Compose configuration to Jetson..."
        ssh "$JETSON_USER@$JETSON_IP" "mkdir -p $JETSON_DIR/docker"
        scp docker/docker-compose.yml "$JETSON_USER@$JETSON_IP:$JETSON_DIR/docker/docker-compose.yml"
        
        echo -e "\n${GREEN}Cross-build and transfer completed successfully!${NC}"
        echo "To run the container on Jetson, execute:"
        echo "  ssh $JETSON_USER@$JETSON_IP 'cd $JETSON_DIR && docker compose -f docker/docker-compose.yml up -d'"
        
        # Clean up local tar file
        rm -f roboteq_driver_hardware.tar
        ;;
        
    3|*)
        echo "Deployment cancelled."
        exit 0
        ;;
esac
