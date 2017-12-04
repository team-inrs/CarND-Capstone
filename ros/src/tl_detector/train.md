
How to train

1. Get the data 
Enter email here 
   https://hci.iwr.uni-heidelberg.de/node/6132
get the email and download
   dataset_train_rgb.zip.001 to 004
Then:

cat dataset_train_rgb.zip.001 dataset_train_rgb.zip.002 dataset_train_rgb.zip.003 dataset_train_rgb.zip.004 > dataset_train_rgb.zip
unzip dataset_train_rgb.zip
mv rgb/ CarND-Capstone/ros/src/tl_detector/

2. Download faster-rcnn-inception checkpoint from the object detection model zoo

wget http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_v2_coco_2017_11_08.tar.gz
tar -xvzf faster_rcnn_inception_v2_coco_2017_11_08.tar.gz

3. Install NVidia docker https://github.com/NVIDIA/nvidia-docker

# Remove old
docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
sudo apt-get purge -y nvidia-docker

# Add the package repositories
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu16.04/amd64/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update

# Install nvidia-docker2 and reload the Docker daemon configuration
sudo apt-get install -y nvidia-docker2
sudo pkill -SIGHUP dockerd

# Build
sudo nvidia-docker build . -f Dockerfile.gpu -t capstone-gpu

4. Run it

# Build object detection outside of the docker container
cd CarND-Capstone/models/research
sudo apt install protobuf-compiler
protoc object_detection/protos/*.proto --python_out=.

# Run
sudo ./run-cuda.sh

# When in the docker, build
cd /udcity
cd ros
catkin_make

# Start
source devel/setup.sh
roslaunch launch/styx.launch

# Or, start training
cd /udacity/ros/src/tl_detector
python simulator_conversion.py
python train.py  -logtostderr --pipeline_config_path=./faster_rcnn_inceptionv2_bosch.config --train_dir=./
 
