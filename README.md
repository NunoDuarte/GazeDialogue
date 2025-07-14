# Gaze Dialogue Model

[![Build Status](https://app.travis-ci.com/NunoDuarte/GazeDialogue.svg?branch=master)](https://app.travis-ci.com/NunoDuarte/GazeDialogue)
[![GitHub license](https://img.shields.io/github/license/Naereen/StrapDown.js.svg)](https://github.com/Naereen/StrapDown.js/blob/master/LICENSE)

Gaze Dialogue Model system for iCub Humanoid Robot

<p align="center">
  <img src="doc/gif_g.gif" width="400" height="225" />
  <img src="doc/gif_f.gif" width="400" height="225" />
</p>

## 🧪 Tested on
![Static Badge](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white&label=16.04)
![Static Badge](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white&label=20.04)

## 🔗 Dependencies
### For controller App
Follow instructions in [icub website](https://icub-tech-iit.github.io/documentation/sw_installation/linux_from_sources_manual/):
- **YCM**
- **YARP**
- **icub-main**
- **OpenCV** (optional)

#### Ubuntu 16.04
```bash
git clone https://github.com/robotology/ycm.git -b v0.11.3
git clone https://github.com/robotology/yarp.git -b v2.3.72
git clone https://github.com/robotology/icub-main.git -b v1.10.0
git clone https://github.com/robotology/icub-contrib-common -b 7d9b7e4
```
#### Ubuntu 20.04
```bash
git clone https://github.com/robotology/ycm.git -b v0.11.3
git clone https://github.com/robotology/yarp.git -b v3.4.0
git clone https://github.com/robotology/icub-main.git -b v1.17.0
```
#### OpenCV (tested on v3.4.1 and v3.4.17)
Recommended with **CUDA** (tested on CUDA-8.0, CUDA-11.2, and CUDA-11.4). Please follow the official [OpenCV documentation](https://docs.opencv.org/4.5.2/d7/d9f/tutorial_linux_install.html).

### For the detection App [![Python 3.5.5](https://img.shields.io/badge/python-3.5.5-blue.svg)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection) [![Python 3.9](https://img.shields.io/badge/python-3.9.19-blue.svg)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection) [![Tensorflow 1.9](https://img.shields.io/badge/tensorflow-v1.9-orange?style=flat&logo=app)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection) [![Tensorflow 2.15.0](https://img.shields.io/badge/tensorflow-v2.15.0-orange?style=flat&logo=app)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)

Install the requirements. We recommend installing in a virtual environment like [Anaconda](https://docs.anaconda.com/anaconda/install/linux/)
```bash
pip3 install -r requirements.txt
```
For our gaze fixations we use **Tensorflow models**
```bash
git clone https://github.com/tensorflow/models.git
```
`utils` package is from **Tensorflow** [Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) (follow the instructions to install it). Then add it to your path
```bash
cd models/research
export PYTHONPATH=$PYTHONPATH:$(pwd)/slim
echo $PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$(pwd):$(pwd)/object_detection
```
`pylsl` needs **liblsl** (v1.13.0). Either install in `/usr/` or add the filepath specified by an environment variable named `PYLSL_LIB`
```bash
cd liblsl && mkdir build && cmake ..
export PYLSL_LIB=/path/to/liblsl.so
```
You can test if the detection system is working by running `python main_offline.py`
### For the connectivity App: [![PupilLabs 3.6.7](https://img.shields.io/badge/PupilLabs-3.6.7-blue.svg)](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection)
This is to send the communication of **PupilLabs** to the detection App which then sends it to the **iCub** (through **YARP**)
- **YARP**
- **PupilLabs** [Capture](https://github.com/pupil-labs/pupil)
- **PupilLabs LSL** [plugin](https://github.com/labstreaminglayer/App-PupilLabs/tree/65f577a520a316ff955b7076150ffa1a61182748/pupil_capture)

Either install the **PupilLabs Capture** app or from source. We use **LabStreamingLayer(LSL)** to stream the data and convert to **YARP**. Alternatively to **LabStreamingLayer** is **ROS** (not yet tested)
- **ROS**
- **PupilLabs ROS** [plugin](https://github.com/qian256/pupil_ros_plugin.git)

## 🛠️ Building *controller app*
[![C++](https://img.shields.io/badge/cpp-5.5.0-blue?logo=cplusplus)](https://github.com/NunoDuarte/GazeDialogue/tree/master/controller)

1.  **Clone repository**
    ```bash
    git clone git@github.com:NunoDuarte/GazeDialogue.git
    ```
2.  **Start with the controller App**
    ```bash
    cd controller
    ```
3.  **Install controller App [Dependencies](#dependencies)**
4.  **Build**
    ```bash
    mkdir build
    ccmake .
    make -j
    ```
5.  **Install detection App [Dependencies](#dependencies)**
6.  **Install connectivity App [Dependencies](#dependencies)** (optional when using iCub)
7.  **Jump to [Setup](#setup)** for the first tests of the GazeDialogue pipeline

# 🚀 Demo
**Test detection App** (pupil_data_test)
1.  **Go to detection app**
    ```bash
    cd detection
    ```
2.  **Run detection system offline**
    ```bash
    python3 main_offline.py
    ```
    You should see a window of a video output appear. The detection system is running on the **PupilLabs** exported data (pupil_data_test) and the output are `[timestep, gaze fixations label, pixel_x, pixel_y]`, for each detected gaze fixation.

# ⚙️ Setup
### Manual mode:
**Test controller App (iCubSIM)**. There are three modes: *manual robot leader*; *gazedialogue robot leader*; *gazedialogue robot follower*. *manual robot leader* does not need eye-tracker(**PupilLabs**) while *gazedialogue modes* require eye-tracker(**PupilLabs**) for it to work.

Open terminals:
```bash
yarpserver --write
yarpmanager
```
In `yarpmanager` do:
1.  Open `controller/apps/iCub_startup.xml`
2.  Open `controller/apps/GazeDialogue_leader.xml`
3.  Run all modules in **iCub_startup**
    You should see the **iCubSIM** simulator open a window, and a second window. Open more terminals:
    ```bash
    cd GazeDialogue/controller/build
    ./gazePupil-detector
    ```
4.  Connect all modules in **iCub_startup**. You should see the iCub's perspective in the second window now.
    ```bash
    ./gazePupil-manual-leader
    ```
5.  Connect all modules in **GazeDialogue-Leader**. Open terminal:
    ```bash
    yarp rpc /service
    ```
6.  Write the following `>> help` this shows the available actions:
    ```
    >> look_down
    >> grasp_it
    >> pass or place
    ```

## GazeDialogue mode:
Open terminals:
```bash
yarpserver --write
yarpmanager
```
In `yarpmanager` do:
1.  Open `controller/apps/iCub_startup.xml`
2.  Open `controller/apps/GazeDialogue_leader.xml`
3.  Run all modules in **iCub_startup**
    You should see the **iCubSIM** simulator open a window, and a second window. Open more terminals:
    ```bash
    cd GazeDialogue/controller/build
    ./gazePupil-detector
    ```
4.  Connect all modules in **iCub_startup**. You should see the iCub's perspective in the second window now.
5.  Turn **PupilLabs Capture** on
6.  Make sure the streaming plugin is on
7.  Open a new terminal and open the *detection app*
    ```bash
    python3 main.py
    ```
    You should see a window open of the eye-tracker output. It should highlight the objects, faces, and gaze.

8.  Run `Pupil_Stream_to_Yarp` (pl1_yarp) to convert the message to **YARP** !!!! (this should be improved)

Now, depending on whether you want to interact with the **iCub** or **iCubSIM** as a **Leader** or **Follower** the instructions change slightly
### Robot as a Leader:
Open a new terminal to run main process for leader
```bash
./gazePupil-main-leader
```
Connect the **GazeDialogue-Leader** yarp port that receives the the gaze fixations.
Press `Enter` - robot will run **GazeDialogue** system for leader

### Robot as a Follower:
Open a new terminal to run main process for follower
```bash
./gazePupil-main-follower
```
Connect the **GazeDialogue-Follower** yarp port that receives the the gaze fixations.
Press `Enter` - robot will run **GazeDialogue** system for follower

# 🤖 Run in real robot (iCub)
You need to change robot name in the file `src/extras/configure.cpp`
```cpp
// Open cartesian solver for right and left arm
string robot="icub";
```
from `"icubSim"` to `"icub"`. Then recompile build.

## Robot as a Follower:
1.  Open **YARP** - `yarpserver`
2.  Use `yarpnamespace /icub` (for more information check [link](https://github.com/NunoDuarte/gazePupil_iCub#run-yarp-from-a-different-computer))
3.  Open **Pupil-Labs** (Capture App)
4.  Open [detection](https://github.com/NunoDuarte/GazeDialogue/tree/master/detection) project
5.  Run [Pupil_Stream_to_Yarp](https://github.com/NunoDuarte/armCoupling_iCub/blob/master/lsl/pupil/README.md) to open **LSL**
6.  Check `/pupil_gaze_tracker` is publishing gaze fixations

Run on the real robot - without right arm (optional). Firstly, start **iCubStartup** from the **yarpmotorgui** in the real **iCub** and run the following packages:
-   `yarprobotinterface --from yarprobotinterface_noSkinNoRight.ini`
-   `iKinCartesianSolver -part left_arm`
-   `iKinGazeCtrl`
-   `wholeBodyDynamics     icubbrain1   --headV2 --autocorrect --no_right_arm`
-   `gravityCompensator    icubbrain2   --headV2 --no_right_arm`
-   `fingersTuner          icub-laptop`
-   `imuFilter             pc104`

## 📂 Structure
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

## 💻 Instructions for a dual-computer system
In case you have the **detection App** and/or the **connectivity App** in a different computer do not forget to point **YARP** to where **iCub** is running:
-   `yarp namespace /icub` (in case `/icub` is the name of the yarp network)
-   `yarp detect` (to check you are connected)
-   `gedit /home/user/.config/yarp/_icub.conf`
-   `'ip of computer you wish to connect' 10000 yarp`

## ✨ Extras
Read camera output
-   `yarpdev --device grabber --name /test/video --subdevice usbCamera --d /dev/video0`
-   `yarp connect /test/video /icubSim/texture/screen`

## 🐛 Issues
-   To make it work on **Ubuntu 16.04** with **CUDA-11.2** and **Tensorflow 2.7** you need to do the following:
    1.  Install **nvidia driver 460.32.03** (`cuda-repo-ubuntu1604-11-2-local_11.2.1-460.32.03-1_amd64.deb`)
    2.  `wget https://developer.download.nvidia.com/compute/cuda/11.2.1/local_installers/cuda-repo-ubuntu1604-11-2-local_11.2.1-460.32.03-1_amd64.deb`
    3.  `sudo dpkg -i cuda-repo-ubuntu1604-11-2-local_11.2.1-460.32.03-1_amd64.deb`
    4.  `sudo apt-key add /var/cuda-repo-ubuntu1604-11-2-local/7fa2af80.pub`
    5.  `sudo apt-get install cuda-11-2`
    6.  Check that `apt-get` is not removing any packages
    7.  Install **Cudnn 8.1** for **CUDA-11.0, 11.1, and 11.2**
    8.  Test using `deviceQuery` on `cuda-11.0 samples/1_Utilities`
    9.  Follow the guidelines of **Building and Instructions**
    10. If after installing **tensorflow**, the system complains about missing `cudart.so.11.0` then do this: (you can add this to `~/.bashrc`)
        ```bash
        export PATH=$PATH:/usr/local/cuda-11.2/bin
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda-11.2/lib64
        ```
-   To make it work on **tensorflow 2.7** I needed to alter the code in `~/software/tensorflow/models/research/object_detection/utils/label_map_utils.py` (line 132)
    ```python
    with tf.io.gfile.GFile(path, 'r') as fid:
    ```
    instead of
    ```python
    with tf.gfile.GFile(path, 'r') as fid:
    ```

## 📜 Citation
If you find this code useful in your research, please consider citing our [paper](https://ieeexplore.ieee.org/abstract/document/9965577):

	M. Raković, N. F. Duarte, J. Marques, A. Billard and J. Santos-Victor, "The Gaze Dialogue Model: Nonverbal Communication in HHI and HRI," in IEEE Transactions on Cybernetics, doi: 10.1109/TCYB.2022.3222077.
