# Gaze Dialogue Model
[![Python 3.6](https://img.shields.io/badge/python-3.6-blue.svg)](https://www.python.org/downloads/release/python-360/)
[![C++](https://img.shields.io/badge/C%2B%2B-%3F%3F-blue?logo=cplusplus)](https://www.codecademy.com/catalog/language/c-plus-plus?g_network=g&g_device=c&g_adid=518718870684&g_keyword=c%2B%2B%20programming&g_acctid=243-039-7011&g_adtype=search&g_adgroupid=102650142713&g_keywordid=kwd-12432420&g_campaign=ROW+Language%3A+Basic+-+Exact&g_campaignid=10074200771&utm_id=t_kwd-12432420:ag_102650142713:cp_10074200771:n_g:d_c&utm_term=c%2B%2B%20programming&utm_campaign=ROW%20Language%3A%20Basic%20-%20Exact&utm_source=google&utm_medium=paid-search&utm_content=518718870684&hsa_acc=2430397011&hsa_cam=10074200771&hsa_grp=102650142713&hsa_ad=518718870684&hsa_src=g&hsa_tgt=kwd-12432420&hsa_kw=c%2B%2B%20programming&hsa_mt=e&hsa_net=adwords&hsa_ver=3&gclid=Cj0KCQjw5uWGBhCTARIsAL70sLKe0aAEMv2Io23zejU2ij5ElZ7_tqrInOxNVFF8Ra9WRHwyGMqGbvYaAn1REALw_wcB)
[![Build Status](https://travis-ci.com/NunoDuarte/GazeDialogue.svg?token=dpExjnDjRy1sV64P2psP&branch=master)](https://travis-ci.com/NunoDuarte/GazeDialogue)
[![GitHub license](https://img.shields.io/github/license/Naereen/StrapDown.js.svg)](https://github.com/Naereen/StrapDown.js/blob/master/LICENSE)

Gaze Dialogue Model controller for iCub Humanoid Robot 

<img src="gif_g.gif" width="400" height="225" /> <img src="gif_f.gif" width="400" height="225" />

# Table of Contents

- [Structure](#structure)
- [Building](#building)
- [Instructions](#instructions)
- [Setup](#setup)
- [Extras](#extras)
- [Citation](#citation)
- [Contributing](#contributing)
- [License](#license)

## Structure
``` text
.
├── CMakeLists.txt
├── app
│   ├── pupil_iCubSim-follower-nonmutual.xml
|   ├── pupil_iCubSim-leader-nonmutual.xml
|   ├── pupil_iCubSim-leader-mutual.xml
|   └── system.xml
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

```

## Building

## Instructions
Run yarp from a different computer
- yarp namespace /icub
- yarp detect (to check you are connected)
- gedit /home/nduarte/.config/yarp/_icub.conf
- 'ip of computer you wish to connect' 10000 yarp 

Run on the real robot - without right arm (optional)
- yarprobotinterface --from yarprobotinterface_noSkinNoRight.ini
- iCubStartup
- iKinCartesianSolver -part left_arm
- iKinGazeCtrl ...
- wholeBodyDynamics     icubbrain1   --headV2 --autocorrect --no_right_arm
- gravityCompensator    icubbrain2   --headV2 --no_right_arm
- fingersTuner          icub-laptop
- imuFilter             pc104

## Setup
Robot as a Follower:
1. open YARP - yarpserver 
2. use yarpnamespace /icub (for more information check [link](https://github.com/NunoDuarte/gazePupil_iCub#run-yarp-from-a-different-computer))
3. open PupilLabs (python3 main.py)
4. connect to [python](https://github.com/NunoDuarte/gazePupil_iCub/tree/master/python) project 
5. run [Pupil_Stream_to_Yarp](https://github.com/NunoDuarte/armCoupling_iCub/blob/master/lsl/pupil/README.md) to open LSL 
6. check /pupil_gaze_tracker is publishing gaze fixations 

Robot as a Leader:
1. placing action is in the module simHHItoiCub-left 
- look_down
- grasp_it (/hardcoded)
- place_on_the_center
2. giving action it is icub_leader.cpp
- look_down is automatic
- grasp_it is when for the first time the iCub looks at the brick (red ball)
- giving action is automatic (deterministic controller with a pre-defined gaze behaviour)

## Extras
Read camera output
- yarpdev --device grabber --name /test/video --subdevice usbCamera --d /dev/video0
- yarp connect /test/video /icubSim/texture/screen

## Citation 
(temporary)
If you find this code useful in your research, please consider citing our [paper](https://www.researchgate.net/publication/326346431_The_Gaze_Dialogue_Model_Non-verbal_communication_in_Human-Human_and_Human-Robot_Interaction):

    @inproceedings{rakovic2021gazedialogue,
	author = {Raković, Mirko and Ferreira Duarte, Nuno and Marques, Jorge and Billard, Aude and Santos-Victor, José},
	year = {2021},
	month = {},
	pages = {14},
	title = {The Gaze Dialogue Model: Non-verbal communication in Human-Human and Human-Robot Interaction}
	}

## Contributing

Nuno Ferreira Duarte

[![GitHub Badge](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/NunoDuarte)
[![Website Badge](https://camo.githubusercontent.com/42acc7ee3a18313a065e672e0835729edf3361dedb045d6c3cf8821fe30a1c2d/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d47697426636f6c6f723d463035303332266c6f676f3d476974266c6f676f436f6c6f723d464646464646266c6162656c3d)](https://nunoduarte.github.io/)
[![Google Badge](https://camo.githubusercontent.com/19402432392aa6c26fb154d597e9d809a69e7b6661219a70c732f60c8ccf87c6/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d476f6f676c652b5363686f6c617226636f6c6f723d343238354634266c6f676f3d476f6f676c652b5363686f6c6172266c6f676f436f6c6f723d464646464646266c6162656c3d)](https://scholar.google.ch/citations?user=HA_f9qsAAAAJ&hl=en)

## License

MIT © Nuno Duarte

