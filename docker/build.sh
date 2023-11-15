#!/bin/bash

# define a registry to push the images to
SCRIPT=$(readlink -f "$0")
SCRIPT_DIR=$(dirname "${SCRIPT}")

REGISTRY=jussikalliola
IMAGE=nano-ros2-orbslam3
VERSION=latest
USE_CACHE=1
SYSTEM_ARCH=$(uname -m)
DOCKERFILE=Dockerfile

# Help function
show_help() {
  echo "Usage: $0 [OPTIONS]"
  echo "Options:"
  echo "  -h    Show help information"
  echo "  -v    Version; amd64,arm64,..."
  echo "  -p    Platoforms; arm64,amd64,..."
  echo "  -c    Use Cache"
  echo "  -g    GPU; 1=True, 0=False"
  exit 0
}

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  show_help
fi



while getopts p:c:v: flag
do
    case "${flag}" in
        p) PLATFORMS=${OPTARG};;
        c) USE_CACHE=${OPTARG};;
        v) VERSION=${OPTARG};;
        g) GPU=${OPTARG};;
    esac
done

if [ -z "${PLATFORMS}" ]; 
then
  echo "No platforms defined. DEFAULT: linux/${SYSTEM_ARCH}"
  PLATFORMS="linux/${SYSTEM_ARCH}"  
else
  echo "Building for platforms: ${PLATFORMS}"
fi


# Check if NVIDIA GPU drivers are available
if [ nvidia-smi &> /dev/null ] && [ "$GPU" == "1" ]; then
  echo "Building a container with GPU support."
  VERSION+="_gpu"
  DOCKERFILE=gpu.Dockerfile
else
  echo "No GPU support."
  DOCKERFILE=Dockerfile
fi



echo "${SCRIPT_DIR}"
echo "Building with params: PLATFORMS=${PLATFORMS}, USE CACHE=${USE_CACHE}, VERSION=${VERSION}"


# create new buildx that support multiple platforms
docker buildx create --use  --driver-opt network=host --name MultiPlatform

# build the image for two different platforms and push the images
if [ $USE_CACHE -eq 1 ]; then
  docker buildx build \
    --cache-from=type=registery,ref=${REGISTRY}/${IMAGE}:${VERSION} \
    --force-rm \
    --progress=plain \
    --platform ${PLATFORMS} \
    -f ${DOCKERFILE} \
    -t ${REGISTRY}/${IMAGE}:${VERSION} \
    --push .

else
  docker buildx build \
    --no-cache \
    --force-rm \
    --progress=plain \
    --platform ${PLATFORMS} \
    -f ${DOCKERFILE} \
    -t ${REGISTRY}/${IMAGE}:${VERSION} \
    --push ${SCRIPT_DIR}/..
fi
