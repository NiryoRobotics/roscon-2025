#!/bin/bash

# Internal Build Script for Niryo ROS2 Workshop Container
# Simple script for the team to build and push the workshop image

set -e 

# Configuration 
IMAGE_NAME="roscon-2025-workshop"
REGISTRY="ghcr.io/niryorobotics"
TAG="latest"

# Parse arguments
NO_CACHE=""
if [[ "$1" == "--no-cache" ]]; then
    NO_CACHE="--no-cache"
    echo "🔄 Building without cache (will re-clone repositories)"
fi

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