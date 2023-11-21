# Gaze Dialogue Model
[![Python 3.5.5](https://img.shields.io/badge/python-3.5.5-blue.svg)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)
[![Python 3.8](https://img.shields.io/badge/python-3.8.12-blue.svg)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)
[![Tensorflow 1.9](https://img.shields.io/badge/tensorflow-v1.9-orange?style=flat&logo=app)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)
[![Tensorflow 2.7.0](https://img.shields.io/badge/tensorflow-v2.7.0-orange?style=flat&logo=app)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)
[![C++](https://img.shields.io/badge/cpp-5.5.0-blue?logo=cplusplus)](https://github.com/NunoDuarte/GazeDialogue/tree/master/controller)
[![Build Status](https://app.travis-ci.com/NunoDuarte/GazeDialogue.svg?branch=master)](https://app.travis-ci.com/NunoDuarte/GazeDialogue)
[![GitHub license](https://img.shields.io/github/license/Naereen/StrapDown.js.svg)](https://github.com/Naereen/StrapDown.js/blob/master/LICENSE)

Gaze Dialogue Model system for iCub Humanoid Robot 

<img src="doc/gif_g.gif" width="400" height="225" /> <img src="doc/gif_f.gif" width="400" height="225" />

# Table of Contents

- [Structure](#structure)
- [Dependencies](#dependencies)
- [Building](#building)
- [Instructions](#instructions-for-a-dual-computer-system)
- [Setup](#setup)
- [Extras](#extras)
- [Issues](#issues)
- [Citation](#citation)
- [Contributing](#contributing)
- [License](#license)


## Structure
``` text
.
├─── Controller
	├── CMakeLists.txt
	├── app
	│   ├── GazeDialogue_follower.xml
	|   ├── GazeDialogue_leader.xml
	|   └── iCub_startup.xml
	|   
	├── include
	│   ├── compute.h
	│   ├── configure.h
	|   ├── helpers.h
	|   └── init.h
	└── src
	    ├── icub_follower.cpp
	    ├── icub_leader.cpp
	    └── extras
		├── CvHMM.h
		├── CvMC.h
		├── compute.cpp
		├── configure.cpp
		├── detector.cpp
		└── helpers.cpp
├─── Detection
	├── main.py | main_offline.py
	├── face_detector.py | face_detector_gpu.py
	├── objt_tracking.py
	├── gaze_behaviour.py
	└── pupil_lsl_yarp.py

```

## Dependencies
### For controller App:
- OpenCV (tested on v3.4.1)
- YARP  (tested on v2.3.72)
- iCub (tested on v1.10)

	- OpenCV can be with or without CUDA, but we do recommend to install OpenCV with CUDA (tested on CUDA-8.0 and on CUDA-11.2). Please follow the official [OpenCV documentation](https://docs.opencv.org/4.5.2/d7/d9f/tutorial_linux_install.html). 
	- To install iCub simulator and drivers you need to follow the oficial [iCub documentation](https://icub-tech-iit.github.io/documentation/sw_installation/). 
	- To install the YARP middleware you need to follow the oficial [YARP documentation](https://www.yarp.it/latest/install_yarp_linux.html).

### For the detection App:
- OpenCV 
- pylsl
- numpy
- os
- math
- msgpack
- zmq
- Tensorflow with CUDA
	- CUDA 8.0; Tensorflow 1.9; Cudnn v7.1 (for GTX 1070)
	- CUDA 11.2; Tensorflow 2.7; Cudnn v8.1; nvidia driver 460.32 (for RTX 3090)
- utils (from Tensorflow Object Detection API)

We recommend installing [Anaconda](https://docs.anaconda.com/anaconda/install/linux/) virtual environment  
```bash
pip install numpy os math msgpath zmq pylsl cv2
```
to import utils you need to install tensorflow with gpu then get the models of tensorflow for object recognition to recognize the import 
```
from utils import label_map_util
from utils import visualization_utils as vis_util
```
you need the following (after you have followed the instructions on how to install tensorflow models) **[WARNING](#issues)** 
``` 
cd tensorflow/models/research
export PYTHONPATH=$PYTHONPATH:$(pwd)/slim
echo $PYTHONPATH 
export PYTHONPATH=$PYTHONPATH:$(pwd):$(pwd)/object_detection 
```
### For the connectivity App:
- LSL - [LabStreamingLayer](https://github.com/sccn/labstreaminglayer) (tested on 1.12)
- YARP  (tested on v2.3.72)
- PupilLabs - [Pupil Capture](https://github.com/pupil-labs/pupil) (tested on v1.7.42)
	- Pupil ROS [plugin](https://github.com/qian256/pupil_ros_plugin.git)

## Building

## Setup
### Robot as a Follower:
1. open YARP - yarpserver 
2. use yarpnamespace /icub (for more information check [link](https://github.com/NunoDuarte/gazePupil_iCub#run-yarp-from-a-different-computer))
3. open Pupil-Labs (Capture App)
4. open [detection](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection) project 
5. run [Pupil_Stream_to_Yarp](https://github.com/NunoDuarte/armCoupling_iCub/blob/master/lsl/pupil/README.md) to open LSL 
6. check /pupil_gaze_tracker is publishing gaze fixations 

### Robot as a Leader:
1. placing action is in the module simHHItoiCub-left 
- look_down
- grasp_it (/hardcoded)
- place_on_the_center
2. giving action it is icub_leader.cpp
- look_down is automatic
- grasp_it is when for the first time the iCub looks at the brick (red ball)
- giving action is automatic (deterministic controller with a pre-defined gaze behaviour)

Run on the real robot - without right arm (optional). Firstly, start iCubStartup from the yarpmotorgui in the real iCub and run the following packages:
- yarprobotinterface --from yarprobotinterface_noSkinNoRight.ini
- iKinCartesianSolver -part left_arm
- iKinGazeCtrl 
- wholeBodyDynamics     icubbrain1   --headV2 --autocorrect --no_right_arm
- gravityCompensator    icubbrain2   --headV2 --no_right_arm
- fingersTuner          icub-laptop
- imuFilter             pc104

## Instructions for a dual-computer system
In case you have the detection App and/or the connectivity App in a different computer do not forget to point YARP to where iCub is running:
- yarp namespace /icub (in case /icub is the name of the yarp network)
- yarp detect (to check you are connected)
- gedit /home/user/.config/yarp/_icub.conf
- 'ip of computer you wish to connect' 10000 yarp 

## Extras
Read camera output
- yarpdev --device grabber --name /test/video --subdevice usbCamera --d /dev/video0
- yarp connect /test/video /icubSim/texture/screen

## Issues
- To make it work on Ubuntu 16.04 with CUDA-11.2 and Tensorflow 2.7 you need to do the following:
1. install nvidia driver 460.32.03 (cuda-repo-ubuntu1604-11-2-local_11.2.1-460.32.03-1_amd64.deb)
2. wget https://developer.download.nvidia.com/compute/cuda/11.2.1/local_installers/cuda-repo-ubuntu1604-11-2-local_11.2.1-460.32.03-1_amd64.deb
3. sudo dpkg -i cuda-repo-ubuntu1604-11-2-local_11.2.1-460.32.03-1_amd64.deb 
4. sudo apt-key add /var/cuda-repo-ubuntu1604-11-2-local/7fa2af80.pub
5. sudo apt-get install cuda-11-2
6. check that apt-get is not removing any packages
7. install Cudnn 8.1 for CUDA-11.0, 11.1, and 11.2
8. test using deviceQuery on cuda-11.0 samples/1_Utilities
9. follow the guidelines of Building and Instructions
10. if after installing tensorflow, the system complains about missing cudart.so.11.0 then do this: (you can add this to ~/.bashrc)
```
export PATH=$PATH:/usr/local/cuda-11.2/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.2/lib64
```
- To make it work on tensorflow 2.7 I needed to alter the code in ~/software/tensorflow/models/research/object_detection/utils/label_map_utils.py (line 132)
```
with tf.io.gfile.GFile(path, 'r') as fid:
```
instead of 
```
with tf.gfile.GFile(path, 'r') as fid:
```

## Citation 
If you find this code useful in your research, please consider citing our [paper](https://ieeexplore.ieee.org/abstract/document/9965577):

	M. Raković, N. F. Duarte, J. Marques, A. Billard and J. Santos-Victor, "The Gaze Dialogue Model: Nonverbal Communication in HHI and HRI," in IEEE Transactions on Cybernetics, doi: 10.1109/TCYB.2022.3222077.


## Contributing

Nuno Ferreira Duarte

[![GitHub Badge](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/NunoDuarte)
[![Website Badge](https://camo.githubusercontent.com/42acc7ee3a18313a065e672e0835729edf3361dedb045d6c3cf8821fe30a1c2d/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d47697426636f6c6f723d463035303332266c6f676f3d476974266c6f676f436f6c6f723d464646464646266c6162656c3d)](https://nunoduarte.github.io/)
[![Google Badge](https://camo.githubusercontent.com/19402432392aa6c26fb154d597e9d809a69e7b6661219a70c732f60c8ccf87c6/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d476f6f676c652b5363686f6c617226636f6c6f723d343238354634266c6f676f3d476f6f676c652b5363686f6c6172266c6f676f436f6c6f723d464646464646266c6162656c3d)](https://scholar.google.ch/citations?user=HA_f9qsAAAAJ&hl=en)

## License

MIT © Nuno Duarte

