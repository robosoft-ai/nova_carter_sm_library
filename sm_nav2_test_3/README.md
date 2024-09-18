 # State Machine Diagram

 ![sm_nav2_test_3](docs/SmNav2Test3_2024-9-7_222225.svg)

 ## Description 
 A state machine application for the NOVA Carter using Nav2 and the SMACC nav2z Client Behavior Library.


# Workflow
## Setting up an IsaacROSDev Container
[https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment](https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment)  
[https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)  
[https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup](https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup)  

## Simulation Environment
For simulation, we'll be using IsaacSim 4.1.
Under the IsaacAssets tab go to SAMPLES/NVBLOX/nvblox_sample_scene.

Once inside, you'll need to fing the NOVA_Carter_ROS_nvblox_setup Folder.
For caster_wheel_left and caster_wheel_right, set the Max Angular Velocity to 500.
Then set the Mass to 10.0 for each one.

Then find the wheel_left, wheel_right and change the Max Angular Velocity to 500.

Then, find the chassis_link Mass and change it to 30.0
Find wheel_material and set Dynamic Friction to 2.0, and Static Friction to 2.0.

Deactivate the Dynamics Xform.

## Let's Get Started
We begin by cloning isaac_ros_common and nova_carter repos to the src folder of our local workspace. My local workspace is ~/workspace/humble_ws
 ```
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/nova_carter.git  
 ```
## Start the IsaacROSDev Container (from the workspace...)
 ```
./src/isaac_ros_common/scripts/run_dev.sh  -d  ~/workspace/humble_ws/
 ```
Then run ls in the terminal to confirm that isaac_ros-dev is set to your host host workspace.    
  
Alternatively, you can run this one from inside workspace/src/isaac_ros_common/scripts. The -d command explicity sets the workspace to workspaces/isaac_ros-dev inside the IsaacROSDev container.
```
./run_dev.sh -d  ~/workspace/humble_ws/
```
## Installations onto the Container....
First, lets get updated... 
 ```
sudo apt-get update  
rosdep update
 ```
We'll need curl too to for later when we download assets into the isaac_ros_assets folder...
 ```
sudo apt-get install -y curl tar
 ```
### Install LTTng-UST...
We'll need this for SMACC later...  
 ```
sudo apt-get install -y lttng-tools  
sudo apt-get install -y lttng-modules-dkms  
sudo apt-get install -y liblttng-ust-dev  
 ```
### Install Jetson Stats
 ```
sudo apt-get install -y ros-humble-isaac-ros-jetson-stats
 ```
### Use rosdep to install Nova Carter bringup dependencies...
 ```
rosdep install -i -r --from-paths ${ISAAC_ROS_WS}/src/nova_carter/nova_carter_bringup/ --rosdistro humble -y
 ```
### Install Nvblox From Debian...
 ```
sudo apt-get install -y ros-humble-isaac-ros-nvblox && \
rosdep install isaac_ros_nvblox
 ```
#### Download the nvblox assets

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_assets"
NGC_VERSION="isaac_ros_nvblox"
NGC_FILENAME="quickstart.tar.gz"

REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions/$NGC_VERSION/files/$NGC_FILENAME"
 ```
Create isaac_ros_assets workspace folder...
 ```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    curl -LO --request GET "${REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    rm ${NGC_FILENAME}
 ```
#### Download the isacc_ros_ess and isaac_ros_peoplesemsegnet models into the isaac_ros_assets folder (takes a while)
Source setup.bash since the packages are already installed...   
```
source /opt/ros/humble/setup.bash
```
Set env variable so you don't have to manually accept every EULA...  
```   
export ISAAC_ROS_ACCEPT_EULA=1
 ```
Run the shell scripts  
```   
ros2 run isaac_ros_ess_models_install install_ess_models.sh
ros2 run isaac_ros_peoplesemseg_models_install install_peoplesemsegnet_vanilla.sh
ros2 run isaac_ros_peoplesemseg_models_install install_peoplesemsegnet_shuffleseg.sh
 ```
### Install isaac_ros_object_detection and other perception packages from Debian... (optional)
We'll start with pointcloud_to_laserscan...  
 ```
sudo apt-get install -y ros-humble-pointcloud-to-laserscan
 ```
#### Install isaac_ros_detectnet
Then we'll get into isaac_ros_object_detection, starting with detectnet
 ```
sudo apt-get install -y ros-humble-isaac-ros-detectnet 
 ```
#### Download the isaac_ros_detectnet assets

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_assets"
NGC_VERSION="isaac_ros_detectnet"
NGC_FILENAME="quickstart.tar.gz"

REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions/$NGC_VERSION/files/$NGC_FILENAME"
 ```
Create isaac_ros_assets workspace folder...
 ```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    curl -LO --request GET "${REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    rm ${NGC_FILENAME}
 ```
#### Install isaac_ros_rtdetr
 ```
sudo apt-get install -y ros-humble-isaac-ros-rtdetr
 ```
#### Download the isaac_ros_rtdetr assets

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
NGC_RESOURCE="isaac_ros_assets"
NGC_VERSION="isaac_ros_rtdetr"
NGC_FILENAME="quickstart.tar.gz"

REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/versions/$NGC_VERSION/files/$NGC_FILENAME"
 ```
Create isaac_ros_assets workspace folder...
 ```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    curl -LO --request GET "${REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets/${NGC_VERSION} && \
    rm ${NGC_FILENAME}
 ```
Now download the Nvidia SyntheitcaDETR model...
 ```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
cd ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
   wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/synthetica_detr/versions/1.0.0/files/sdetr_grasp.etlt'
```
Then we'll convert the encrypted model (.etlt) to a TensorRT engine plan and drop it in the isaac_ros_assets/models/synthetica_detr folder...
```
/opt/nvidia/tao/tao-converter -k sdetr -t fp16 -e ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan -p images,1x3x640x640,2x3x640x640,4x3x640x640 -p orig_target_sizes,1x2,2x2,4x2 ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.etlt
```
Then get back to the workspace...  
```
cd /workspaces/isaac_ros-dev/
```
and install this package so you can test your vision pipeline later...  
```
sudo apt-get install -y ros-humble-isaac-ros-examples
```
#### Install isaac_ros_image_pipeline
 ```
 sudo apt-get install -y ros-humble-isaac-ros-depth-image-proc
 sudo apt-get install -y ros-humble-isaac-ros-gxf-extensions
 sudo apt-get install -y ros-humble-isaac-ros-image-pipeline
 ```
These should already be installed but if you just want to make sure..  
 ```
 sudo apt-get install -y ros-humble-isaac-ros-image-proc
 sudo apt-get install -y ros-humble-isaac-ros-stereo-image-proc
 ```

## Assemble the Workspace

### Clone the Application Reposinto the src folder...  
 ```
git clone https://github.com/robosoft-ai/SMACC2.git  
git clone https://github.com/robosoft-ai/nova_carter_sm_library 
 ```

### Get a Nav2 release into the workspace src folder...
I like to work from Nav2 releases. Download and unzip, then drop into the src folder of your local workspace. This is currently 1.1.16
 https://github.com/ros-navigation/navigation2/releases/tag/1.1.16  

### Workspace Operations

Create a file called .isaac_ros_common-config with the following context:
 ```
cd src/isaac_ros_common/scripts
 ```
 ```
echo -e "CONFIG_IMAGE_KEY=ros2_humble.nova_carter\nCONFIG_DOCKER_SEARCH_DIRS=(../../nova_carter/docker ../docker)" > .isaac_ros_common-config
 ```
Create a file called .isaac_ros_dev-dockerargs with the following context:
 ```
  echo -e "-v /etc/nova/:/etc/nova/\n-v /opt/nvidia/nova/:/opt/nvidia/nova/" > .isaac_ros_dev-dockerargs
 ```

Clone the perceptor dependency repositories using the vcstool file in the nova_carter repository:
 ```
cd /workspaces/isaac_ros-dev
vcs import --recursive src < src/nova_carter/nova_carter.repos
 ```
A ton of beta ish dependencies are added in this step to make the original perceptor demo work.  

## Build Workspace
```
source /opt/ros/humble/setup.bash
```
```
rosdep update
rosdep install --ignore-src --from-paths src -y -r
 ```
 ```
colcon build --symlink-install
 ```

## Launch Application
Source the workspace...  
 ```
source install/setup.bash
 ```
```
ros2 run sm_nav2_test_3 lidar_completion.py --ros-args -r /scan_input:=/scan2 -r /scan_output:=/scan
```
 ```
ros2 launch sm_nav2_test_3 sm_nav2_test_3_launch.py 
 ```


