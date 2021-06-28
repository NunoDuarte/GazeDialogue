# Gaze Dialogue Model

Gaze Dialogue Model controller for iCub Humanoid Robot [![Build Status](https://travis-ci.com/NunoDuarte/gazePupil_iCub.svg?token=dpExjnDjRy1sV64P2psP&branch=master)](https://travis-ci.com/NunoDuarte/gazePupil_iCub)

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
- 
## Contributing

@[NunoDuarte](https://github.com/NunoDuarte)

## License

MIT © Nuno Duarte

