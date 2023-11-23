# Gaze Dialogue Model
[![Python 3.5.5](https://img.shields.io/badge/python-3.5.5-blue.svg)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)
[![Python 3.8](https://img.shields.io/badge/python-3.8.12-blue.svg)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)
[![Tensorflow 1.9](https://img.shields.io/badge/tensorflow-v1.9-orange?style=flat&logo=app)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)
[![Tensorflow 2.8.0](https://img.shields.io/badge/tensorflow-v2.8.0-orange?style=flat&logo=app)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)
[![C++](https://img.shields.io/badge/cpp-5.5.0-blue?logo=cplusplus)](https://github.com/NunoDuarte/GazeDialogue/tree/master/controller)
[![Build Status](https://app.travis-ci.com/NunoDuarte/GazeDialogue.svg?branch=master)](https://app.travis-ci.com/NunoDuarte/GazeDialogue)
[![GitHub license](https://img.shields.io/github/license/Naereen/StrapDown.js.svg)](https://github.com/Naereen/StrapDown.js/blob/master/LICENSE)

Gaze Dialogue Model system for iCub Humanoid Robot 

<img src="doc/gif_g.gif" width="400" height="225" /> <img src="doc/gif_f.gif" width="400" height="225" />

# Table of Contents

- [Building](#building)
- [Dependencies](#dependencies)
- [Setup](#setup)
- [Structure](#structure)
- [Instructions](#instructions-for-a-dual-computer-system)
- [Extras](#extras)
- [Issues](#issues)
- [Citation](#citation)
- [Contributing](#contributing)
- [License](#license)

## Building
1. clone repository
```
git clone git@github.com:NunoDuarte/GazeDialogue.git
```
2. start with the controller App (have a look at [Structure](#structure) to understand the pipeline of GazeDialogue)
```
cd controller
```
3. install dependencies for controller App in [Dependencies](#dependencies)
4. build
```
mkdir build
ccmake .
make -j
```
5. install the dependencies for detection App in [Dependencies](#dependencies)
6. install the dependencies for connectivity App in [Dependencies](#dependencies) (optional only for real iCub)
7. Jump to [Setup](#setup) for the first tests of the GazeDialogue pipeline

## Dependencies
### For controller App follow instructions in [icub website](https://icub-tech-iit.github.io/documentation/sw_installation/linux_from_sources_manual/):
- YARP  (tested on v2.3.72)
- iCub (tested on v1.10)
```
$ git clone https://github.com/robotology/ycm.git -b v0.11.3
$ git clone https://github.com/robotology/yarp.git -b v3.4.0
$ git clone https://github.com/robotology/icub-main.git -b v1.17.0
```
- OpenCV (tested on v3.4.1 and v3.4.17)
	- OpenCV can be with or without CUDA, but we do recommend to install OpenCV with CUDA (tested on CUDA-8.0, CUDA-11.2, and CUDA-11.4). Please follow the official [OpenCV documentation](https://docs.opencv.org/4.5.2/d7/d9f/tutorial_linux_install.html). 

### For the detection App
Install the requirements. We recommend installing [Anaconda](https://docs.anaconda.com/anaconda/install/linux/) virtual environment  
```
pip3 install -r requirements.txt
```
```utils``` package is from Tensorflow [Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) (follow the instructions to install it). Then add it to your path
```
cd tensorflow/models/research
export PYTHONPATH=$PYTHONPATH:$(pwd)/slim
echo $PYTHONPATH 
export PYTHONPATH=$PYTHONPATH:$(pwd):$(pwd)/object_detection 
```
``pylsl`` needs liblsl. Either install in /usr/ or add the filepath specified by an environment variable named PYLSL_LIB
```
export PYLSL_LIB=/path/to/liblsl.so
```

### For the connectivity App:
This is send the communication of PupilLabs to the detection App which then send to the iCub (YARP)
- LSL - [LabStreamingLayer](https://github.com/sccn/labstreaminglayer) (tested on 1.12)
- YARP  (tested on v2.3.72)
- PupilLabs - [Pupil Capture](https://github.com/pupil-labs/pupil) (tested on v1.7.42)
	- Pupil ROS [plugin](https://github.com/qian256/pupil_ros_plugin.git)


# Setup
**Test detection App** (pupil_data_test)
1. go to detection app
```
cd detection
```
2. run detection system offline
```
python3 main_offline.py
```
You should see a window of a video output appear. The detection system is running on the PupilLabs exported data (pupil_data_test) and the output are [timestep, gaze fixations label, pixel_x, pixel_y], for each detected gaze fixation. 

### Manual mode:
**Test controller App (iCubSIM)**. There are three modes: manual robot leader; gazedialogue robot leader; gazedialogue robot follower. manual robot leader does not need eye-tracker(PupilLabs) while gazedialogue modes require eye-tracker(PupilLabs) for it to work.

Open terminals:
```
yarpserver --write
yarpmanager
```
in yarpmanager do:
1. open controller/apps/iCub_startup.xml
2. open controller/apps/GazeDialogue_leader.xml
3. run all modules in iCub_startup
You should see the iCubSIM simulator open a window, and a second window. Open more terminals:
```
cd GazeDialogue/controller/build
./gazePupil-detector
```
4. connect all modules in iCub_startup. You should see the iCub's perspective in the second window now. 
```
./gazePupil-main-follower
```
5. connect all modules in GazeDialogue-Leader.
6. Press Enter - robot will look down
7. Press Enter - robot will find ball and grasp it (try to!)
8. Press Enter - robot will run GazeDialogue system for leader (needs PupilLabs to function properly)

### GazeDialogue mode - Robot as a Leader:
Open terminals:
```
yarpserver --write
yarpmanager
```
in yarpmanager do:
1. open controller/apps/iCub_startup.xml
2. open controller/apps/GazeDialogue_leader.xml
3. run all modules in iCub_startup
You should see the iCubSIM simulator open a window, and a second window. Open more terminals:
```
cd GazeDialogue/controller/build
./gazePupil-detector
```
4. connect all modules in iCub_startup. You should see the iCub's perspective in the second window now. 
```
./gazePupil-main-follower
```
5. connect all modules in GazeDialogue-Leader.
6. Press Enter - robot will look down
7. Press Enter - robot will find ball and grasp it (try to!)
8. Press Enter - robot will run GazeDialogue system for leader (needs PupilLabs to function properly)

1. placing action is in the module simHHItoiCub-left 
- look_down
- grasp_it (/hardcoded)
- place_on_the_center
2. giving action it is icub_leader.cpp
- look_down is automatic
- grasp_it is when for the first time the iCub looks at the brick (red ball)
- giving action is automatic (deterministic controller with a pre-defined gaze behaviour)

# Run in real robot (iCub)
You need to change robot name in the file ```src/extras/configure.cpp``` 
```cpp
        // Open cartesian solver for right and left arm
        string robot="icub";
```
from ```"icubSim"``` to ```"icub"```. Then recompile build.

## Robot as a Follower:
1. open YARP - yarpserver 
2. use yarpnamespace /icub (for more information check [link](https://github.com/NunoDuarte/gazePupil_iCub#run-yarp-from-a-different-computer))
3. open Pupil-Labs (Capture App)
4. open [detection](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection) project 
5. run [Pupil_Stream_to_Yarp](https://github.com/NunoDuarte/armCoupling_iCub/blob/master/lsl/pupil/README.md) to open LSL 
6. check /pupil_gaze_tracker is publishing gaze fixations 

Run on the real robot - without right arm (optional). Firstly, start iCubStartup from the yarpmotorgui in the real iCub and run the following packages:
- yarprobotinterface --from yarprobotinterface_noSkinNoRight.ini
- iKinCartesianSolver -part left_arm
- iKinGazeCtrl 
- wholeBodyDynamics     icubbrain1   --headV2 --autocorrect --no_right_arm
- gravityCompensator    icubbrain2   --headV2 --no_right_arm
- fingersTuner          icub-laptop
- imuFilter             pc104

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

