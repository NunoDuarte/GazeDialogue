# gazePupil_iCub
Repository for the Modeling Human Gaze Behavior and Robot Application Project

## Robot as a Leader
1. placing action is in the module simHHItoiCub-left 
- look_down
- grasp_it (/hardcoded)
- place_on_the_center
2. giving action it is icub_leader.cpp
- look_down is automatic
- grasp_it is when for the first time the iCub looks at the brick (red ball)
- giving action is automatic (deterministic controller with a pre-defined gaze behaviour)

## Robot as a Follower

# read camera output
- yarpdev --device grabber --name /test/video --subdevice usbCamera --d /dev/video0
- yarp connect /test/video /icubSim/texture/screen

# to work paralleled with pupil labs
- sudo rmmod uvcvideo
- sudo modprobe uvcvideo quirks=128 
-> you can think of running a script instead of always needing to run this before turning it on

# run on the real robot (without right arm)
- yarprobotinterface --from yarprobotinterface_noSkinNoRight.ini
### iCubStartup:
- iKinCartesianSolver -part left_arm
- iKinGazeCtrl ...
- wholeBodyDynamics     icubbrain1   --headV2 --autocorrect --no_right_arm
- gravityCompensator    icubbrain2   --headV2 --no_right_arm
- fingersTuner          icub-laptop
- imuFilter             pc104

# run yarp from a different computer
- yarp namespace /icub
- yarp detect (to check you are connected)
- gedit /home/nduarte/.config/yarp/_icub.conf
- 'ip of computer you wish to connect' 10000 yarp 
