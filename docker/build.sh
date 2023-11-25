#!/bin/bash

# define a registry to push the images to
SCRIPT=$(readlink -f "$0")
SCRIPT_DIR=$(dirname "${SCRIPT}")

REGISTRY=jussikalliola
IMAGE=nano-ros2-orbslam3
VERSION=latest
USE_CACHE="1"
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
  echo "  -t    Target build; dev,prod,test... Default=dev"
  exit 0
}

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  show_help
fi



while getopts p:c:v:g:t: flag
do
    case "${flag}" in
        p) PLATFORMS=${OPTARG};;
        c) USE_CACHE=${OPTARG};;
        v) VERSION=${OPTARG};;
        g) GPU=${OPTARG};;
        t) TARGET_BUILD=${OPTARG};;
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

# Check target build, default dev
if [ -z "$TARGET_BUILD" ]; then
  ECHO "No target build given... Default: dev"
  TARGET_BUILD="dev"
fi

echo -e "\n============================"
echo -e "Building with params:\n - PLATFORMS=${PLATFORMS}\n - USE CACHE=${USE_CACHE}\n - VERSION=${VERSION}\n - TARGET_BUILD=${TARGET_BUILD}\n - GPU=${GPU}"
echo -e "============================\n"


# create new buildx that support multiple platforms
docker buildx create --use  --driver-opt network=host --name MultiPlatform

# build the image for two different platforms and push the images
if [ "$USE_CACHE" == "1" ]; then
  
  echo "Building with Cache."

  docker buildx build \
    --build-arg TARGET_BUILD=${TARGET_BUILD} \
    --cache-from=type=registery,ref=${REGISTRY}/${IMAGE}:${VERSION} \
    --force-rm \
    --progress=plain \
    --platform ${PLATFORMS} \
    -f ${DOCKERFILE} \
    -t ${REGISTRY}/${IMAGE}:${VERSION} \
    --push .

else

  echo "Building without Cache."

  docker buildx build \
    --build-arg TARGET_BUILD=${TARGET_BUILD} \
    --no-cache \
    --force-rm \
    --progress=plain \
    --platform ${PLATFORMS} \
    -f ${DOCKERFILE} \
    -t ${REGISTRY}/${IMAGE}:${VERSION} \
    --push .
fi
