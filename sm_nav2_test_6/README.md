 # State Machine Diagram

 ![sm_nav2_test_6](docs/SmNav2Test6_2024-9-7_222225.svg)

 ## Description 
 A state machine application for the NOVA Carter using Nav2 and the SMACC nav2z Client Behavior Library.


# Workflow
## Setting up an IsaacROSDev Container
[https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment](https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment)  
[https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html](https://nvidia-isaac-ros.github.io/getting_started/dev_env_setup.html)  
[https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup](https://nvidia-isaac-ros.github.io/robots/nova_carter/getting_started.html#nova-carter-dev-setup)  

## Simulation Environment
For simulation, we'll be using IsaacSim 4.2.
Under the IsaacAssets tab go to SAMPLES/NVBLOX/nvblox_sample_scene.

Once inside, you'll need to fing the NOVA_Carter_ROS_nvblox_setup Folder.
For caster_wheel_left and caster_wheel_right, set the Max Angular Velocity to 500.
Then set the Mass to 10.0 for each one.

Then find the wheel_left, wheel_right and change the Max Angular Velocity to 500.

Then, find the chassis_link Mass and change it to 30.0
Find wheel_material and set Dynamic Friction to 2.0, and Static Friction to 2.0.

Deactivate the Dynamics Xform.

Go to PROPS/NVIDIA and frag the charger into the scene.
Set the position to... z= 0.00917, y= 0.59935, x=5.27575
Be sure to disable gravity and lock position or it will fall through the floor.

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
sudo apt-get install -y curl jq tar
 ```
### Install LTTng-UST...
We'll need this for SMACC later...  
 ```
sudo apt-get install -y lttng-tools  
sudo apt-get install -y lttng-modules-dkms  
sudo apt-get install -y liblttng-ust-dev  
 ```
### Install Jetson Stats  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_jetson/isaac_ros_jetson_stats/index.html#build-package-name)
 ```
sudo apt-get install -y ros-humble-isaac-ros-jetson-stats
 ```
### Use rosdep to install Nova Carter bringup dependencies...
 ```
rosdep install -i -r --from-paths ${ISAAC_ROS_WS}/src/nova_carter/nova_carter_bringup/ --rosdistro humble -y
 ```
### Install isaac_ros_ess  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/isaac_ros_ess/index.html#build-package-name)
```
sudo apt-get install -y ros-humble-isaac-ros-ess && \
   sudo apt-get install -y ros-humble-isaac-ros-ess-models-install
```
#### Download the isaac_ros_ess assets | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_dnn_stereo_depth/isaac_ros_ess/index.html#download-quickstart-assets)

Run these commands to download the asset from NGC...
```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_ess"
NGC_RESOURCE="isaac_ros_ess_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=1
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
```

Download and install the pre-trained ESS model files...
```
ros2 run isaac_ros_ess_models_install install_ess_models.sh --eula
```
Then get back to the workspace...  
```
cd /workspaces/isaac_ros-dev/
```
### Install Nvblox From Debian...  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#set-up-package-name)
 ```
sudo apt-get install -y ros-humble-isaac-ros-nvblox && \
rosdep install isaac_ros_nvblox
 ```
#### Download the nvblox assets  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#download-quickstart-assets)

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_nvblox"
NGC_RESOURCE="isaac_ros_nvblox_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=1
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
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
To test this section, use this [IsaacSim Tutorial](https://nvidia-isaac-ros.github.io/concepts/scene_reconstruction/nvblox/tutorials/tutorial_isaac_sim.html)

### Install isaac_ros_object_detection and other perception packages from Debian... (optional)
We'll start with pointcloud_to_laserscan...  
 ```
sudo apt-get install -y ros-humble-pointcloud-to-laserscan
 ```
### Install isaac_ros_detectnet  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/isaac_ros_detectnet/index.html#build-package-name)
Then we'll get into isaac_ros_object_detection, starting with required pkgs
 ```
sudo apt-get install -y ros-humble-isaac-ros-dnn-image-encoder ros-humble-isaac-ros-triton
 ```
and then with detectnet...
 ```
sudo apt-get install -y ros-humble-isaac-ros-detectnet
 ```
#### Download the isaac_ros_detectnet assets  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/isaac_ros_detectnet/index.html#download-quickstart-assets)

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_detectnet"
NGC_RESOURCE="isaac_ros_detectnet_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=1
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
 ```
Then Run the setup script to download the PeopleNet Model from NVIDIA GPU Cloud(NGC) and convert it to a .etlt file
 ```
 ros2 run isaac_ros_detectnet setup_model.sh --height 720 --width 1280 --config-file isaac_sim_config.pbtxt
 ```
 
To test this section, use this [IsaacSim Tutorial](https://nvidia-isaac-ros.github.io/concepts/object_detection/detectnet/tutorial_isaac_sim.html)
### Install isaac_ros_rtdetr  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/isaac_ros_rtdetr/index.html#build-package-name)
 ```
sudo apt-get install -y ros-humble-isaac-ros-rtdetr
 ```
#### Download the isaac_ros_rtdetr assets | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_object_detection/isaac_ros_rtdetr/index.html#download-quickstart-assets)

Set variables for isaac_ros_assets workspace folder...
 ```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_rtdetr"
NGC_RESOURCE="isaac_ros_rtdetr_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=1
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
 ```
Now download the pre-trained Nvidia SyntheitcaDETR models...
 ```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
cd ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
   wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/synthetica_detr/versions/1.0.0/files/sdetr_grasp.etlt'
```
 ```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
cd ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr && \
   wget 'https://api.ngc.nvidia.com/v2/models/nvidia/isaac/synthetica_detr/versions/1.0.0/files/sdetr_amr.etlt'
```
Then we'll convert the encrypted model (.etlt) to a TensorRT engine plan and drop it in the isaac_ros_assets/models/synthetica_detr folder...
```
/opt/nvidia/tao/tao-converter -k sdetr -t fp16 -e ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan -p images,1x3x640x640,2x3x640x640,4x3x640x640 -p orig_target_sizes,1x2,2x2,4x2 ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.etlt
```
```
/opt/nvidia/tao/tao-converter -k sdetr -t fp16 -e ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_amr.plan -p images,1x3x640x640,2x3x640x640,4x3x640x640 -p orig_target_sizes,1x2,2x2,4x2 ${ISAAC_ROS_WS}/isaac_ros_assets/models/synthetica_detr/sdetr_amr.etlt
```
Then get back to the workspace...  
```
cd /workspaces/isaac_ros-dev/
```
and install this package so you can test your vision pipeline later...  
```
sudo apt-get install -y ros-humble-isaac-ros-examples
```
To test this section use this [IsaacSim Tutorial](https://nvidia-isaac-ros.github.io/concepts/object_detection/rtdetr/tutorial_isaac_sim.html)

### Install isaac_ros_foundation_pose  | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html#build-package-name)
```
sudo apt-get install -y ros-humble-isaac-ros-foundationpose
```
Then download the assets from NGC...
```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_foundationpose"
NGC_RESOURCE="isaac_ros_foundationpose_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=1
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
```
#### Download the foundationpose assets | [Source](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_pose_estimation/isaac_ros_foundationpose/index.html#download-quickstart-assets)
```
wget --content-disposition https://api.ngc.nvidia.com/v2/models/nvidia/isaac/foundationpose/versions/1.0.0/zip -O foundationpose_1.0.0.zip
```
Copy the models to the required directory...
```
mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose && \
   unzip foundationpose_1.0.0.zip -d ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose
```
Then we'll convert the encrypted models (.etlt) for the isaac_ros_foundationpose package to TensorRT engine plans and drop it in the isaac_ros_assets/models/isaac_ros_foundationpose folder...
```
/opt/nvidia/tao/tao-converter -k foundationpose -t fp16 -e ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan -p input1,1x160x160x6,1x160x160x6,252x160x160x6 -p input2,1x160x160x6,1x160x160x6,252x160x160x6 -o output1,output2 ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/refine_model.etlt
```
```   
/opt/nvidia/tao/tao-converter -k foundationpose -t fp16 -e ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_trt_engine.plan -p input1,1x160x160x6,1x160x160x6,252x160x160x6 -p input2,1x160x160x6,1x160x160x6,252x160x160x6 -o output1 ${ISAAC_ROS_WS}/isaac_ros_assets/models/foundationpose/score_model.etlt
```   

Then get back to the workspace...  
```
cd /workspaces/isaac_ros-dev/
```
To test this section use this [IsaacSim Tutorial](https://nvidia-isaac-ros.github.io/concepts/pose_estimation/foundationpose/tutorial_isaac_sim.html)

### Install isaac_ros_image_pipeline (deprecated?)
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
This has now led to build problems involving building the following packages from source. So, after completing the previous vcs import command, remove the following packages form workspace/src

 ```
 REMOVE THESE PKGS FROM THE WORKSPACE/SRC FOLDER BEFORE COMPILING
isaac_ros_nitros
isaac_ros_image_segmentation
isaac_ros_dnn_inference
isaac_ros_image_pipeline
isaac_ros_freespace_segmentation
isaac_perceptor
isaac_ros_depth_segmentation
isaac_ros_dnn_stereo_depth
 ```
## Build Workspace
Ok, now you're ready to compile everything...
```
source /opt/ros/humble/setup.bash
```
```
rosdep update
rosdep install --ignore-src --from-paths src -y -r
 ```
 ```
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
 ```

## Launch Application
Source the workspace...  
 ```
source install/setup.bash
 ```
```
ros2 run sm_nav2_test_6 lidar_completion.py --ros-args -r /scan_input:=/scan2 -r /scan_output:=/scan
```
 ```
ros2 launch sm_nav2_test_6 sm_nav2_test_6_launch.py 
 ```


