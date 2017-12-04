#! /bin/bash
set -e

# Settings from environment
UDACITY_SOURCE=${UDACITY_SOURCE:-`pwd`}
UDACITY_IMAGE=${UDACITY_IMAGE:-capstone-gpu}
CONTAINER_NAME="capstone"

if [ "$(docker ps -a | grep ${CONTAINER_NAME})" ]; then
  echo "Attaching to running container..."
  nvidia-docker exec -it ${CONTAINER_NAME} bash $@
else
  nvidia-docker run --name ${CONTAINER_NAME} --rm -it -p 4567:4567 -v "${UDACITY_SOURCE}:/udacity" ${UDACITY_IMAGE} $@
fi
