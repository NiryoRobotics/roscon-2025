#!/bin/bash

# Internal Build Script for Niryo ROS2 Workshop Container
# Simple script for the team to build and push the workshop image

set -e 

# Configuration 
IMAGE_NAME="roscon-2025-workshop"
REGISTRY="ghcr.io/niryorobotics"
TAG="latest"
ROSCON_BRANCH="main"  # Default branch for roscon-2025 repo

# Parse arguments
NO_CACHE=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-cache)
            NO_CACHE="--no-cache"
            echo "🔄 Building without cache (will re-clone repositories)"
            shift
            ;;
        --branch)
            ROSCON_BRANCH="$2"
            echo "🌿 Using branch: $ROSCON_BRANCH for roscon-2025 repo"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--no-cache] [--branch <branch_name>]"
            exit 1
            ;;
    esac
done

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Building Niryo ROS2 Workshop Container${NC}"
echo "📦 Image: $REGISTRY/$IMAGE_NAME:$TAG"
echo ""

# Check Docker
if ! command -v docker &> /dev/null; then
    echo -e "${RED}❌ Docker not found. Please install Docker first.${NC}"
    exit 1
fi

# Build the image (path relative to project root when script is run from root)
echo -e "${BLUE}🔨 Building Docker image...${NC}"
docker build \
    $NO_CACHE \
    --network=host \
    --build-arg ROSCON_BRANCH=$ROSCON_BRANCH \
    --file .devcontainer/Dockerfile \
    --tag $IMAGE_NAME:$TAG \
    --tag $REGISTRY/$IMAGE_NAME:$TAG \
    .

echo -e "${GREEN}✅ Build completed successfully!${NC}"

# Ask if user wants to push
read -p "🚀 Push to registry? (y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${BLUE}📤 Pushing to registry...${NC}"
    docker push $REGISTRY/$IMAGE_NAME:$TAG
    echo -e "${GREEN}✅ Push completed successfully!${NC}"
    echo -e "${BLUE}📝 Update devcontainer.json to use: $REGISTRY/$IMAGE_NAME:$TAG${NC}"
else
    echo -e "${BLUE}📝 Local image built: $IMAGE_NAME:$TAG${NC}"
    echo -e "${BLUE}📝 To push later, run: docker push $REGISTRY/$IMAGE_NAME:$TAG${NC}"
fi

echo ""
echo -e "${GREEN}🎉 Done! Image ready for workshop participants.${NC}"
echo ""
echo -e "${BLUE}💡 Tip: Use --no-cache to force rebuild and re-clone repositories${NC}"
echo -e "${BLUE}   Example: ./build-and-push.sh --no-cache${NC}"
echo -e "${BLUE}💡 Tip: Use --branch to specify roscon-2025 branch to clone${NC}"
echo -e "${BLUE}   Example: ./build-and-push.sh --branch feature-xyz${NC}"
echo -e "${BLUE}   Example: ./build-and-push.sh --no-cache --branch develop${NC}"