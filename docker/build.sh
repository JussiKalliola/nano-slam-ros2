# define a registry to push the images to
REGISTRY=jussikalliola
IMAGE=nano-ros2-orbslam3
VERSION=latest
USE_CACHE=1
SYSTEM_ARCH=$(uname -m)

while getopts p:c:v: flag
do
    case "${flag}" in
        p) PLATFORMS=${OPTARG};;
        c) USE_CACHE=${OPTARG};;
        v) VERSION=${OPTARG};;
    esac
done

if [ -z "${PLATFORMS}" ]; 
then
  echo "No platforms defined. DEFAULT: linux/${SYSTEM_ARCH}"
  PLATFORMS="linux/${SYSTEM_ARCH}"  
else
  echo "Building for platforms: ${PLATFORMS}"
fi




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
    -f Dockerfile \
    -t ${REGISTRY}/${IMAGE}:${VERSION} \
    --push .

else
  docker buildx build \
    --no-cache \
    --force-rm \
    --progress=plain \
    --platform ${PLATFORMS} \
    -f Dockerfile \
    -t ${REGISTRY}/${IMAGE}:${VERSION} \
    --push .
fi
