# Udacity Self-Driving Car Nanodegree: Capstone

## Group Members
* [Kibaek Jeong](https://github.com/KibaekJeong)
* [Duke Noi](https://github.com/dawn360)
* [Hong Cho](https://github.com/hong9life)
* [Shayan Salehian](https://github.com/shayan72)
* [Shin-Ying Lu](https://github.com/shinyingl)

## ROS Nodes

### Traffic Light Detection
The traffic light detection node is the main part of the perception sub system for following project. Traffic light detection node detects incoming traffic lights and its state. It detects state of the upcoming traffic light so that waypoint updater decides whether to stop at the traffic light or pass the traffic light. 

For traffic light detector uses [Single Shot MultiBox Detector](https://arxiv.org/abs/1512.02325) with feature extractor of [MobileNetV2](https://arxiv.org/abs/1801.04381). Model pretraiined with [COCO Dataset](http://cocodataset.org/), which already includes traffic light category was used for following project. Tensorflow detection model is provided in following [link](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md).

In order to maximize detection capability, transfer learning was done with two different dataset. First dataset was from Udacity. Photos of Udacity simulator and [Udacity Training Bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) provided by Udacity was used for first set. In addition, [Bosch Small Traffic Lights Dataset](https://hci.iwr.uni-heidelberg.de/node/6132) was used for training and evaluation.

As result, model was able to detect all the traffic lights in Udacity simulator and Udacity on-site test images with no problem. Also, as detection timing is highly important as it is directly connected to passengers safety. Using MobileNetV2, we were able to achieve average detection time of 25ms.

### Waypoint updater

### Drive by  Wire (DBW)

## Running Simulation
Please use **one** of the two installation options, either native **or** docker installation.
### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).

* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
In order to run simulator, port 4567 has to be forwarded. For example, in Virtual box:
1. Open Oracle VM VirtualBox
2. Click on the default session and select settings.
3. Click on Network, Advanced, Port Forwarding.
4. Click on green plus sign, and add new port forwarding rule.
5. Add 4567 as both the host port and guest port.

### Running

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator
