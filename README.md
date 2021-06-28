# Gaze Dialogue Model

Gaze Dialogue Model controller for iCub Humanoid Robot [![Build Status](https://travis-ci.com/NunoDuarte/gazePupil_iCub.svg?token=dpExjnDjRy1sV64P2psP&branch=master)](https://travis-ci.com/NunoDuarte/gazePupil_iCub)

## Table of Contents

- [Install](#install)
- [Usage](#usage)
- [Extras](#extras)
- [Contributing](#contributing)
- [License](#license)

## Install

```
```

## Steps to take before Usage
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

## Usage
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

## Contributing

Nuno Ferreira Duarte

[![GitHub Badge](https://img.shields.io/badge/GitHub-100000?style=for-the-badge&logo=github&logoColor=white)](https://github.com/NunoDuarte)
[![Website Badge](https://camo.githubusercontent.com/42acc7ee3a18313a065e672e0835729edf3361dedb045d6c3cf8821fe30a1c2d/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d47697426636f6c6f723d463035303332266c6f676f3d476974266c6f676f436f6c6f723d464646464646266c6162656c3d)](https://nunoduarte.github.io/)
[![Google Badge](https://camo.githubusercontent.com/19402432392aa6c26fb154d597e9d809a69e7b6661219a70c732f60c8ccf87c6/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d476f6f676c652b5363686f6c617226636f6c6f723d343238354634266c6f676f3d476f6f676c652b5363686f6c6172266c6f676f436f6c6f723d464646464646266c6162656c3d)](https://scholar.google.ch/citations?user=HA_f9qsAAAAJ&hl=en)

## License

MIT © Nuno Duarte

