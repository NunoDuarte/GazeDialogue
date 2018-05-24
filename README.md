# gazePupil_iCubSIM
Repository for the Modeling Human Gaze Behavior and Robot Application Project

# read camera output
yarpdev --device grabber --name /test/video --subdevice usbCamera --d /dev/video0
yarp connect /test/video /icubSim/texture/screen

# to work paralleled with pupil labs
sudo rmmod uvcvideo
sudo modprobe uvcvideo quirks=128 
-> you can think of running a script instead of always needing to run this before turning it on

# run on the real robot (without right arm)
- yarprobotinterface --from yarprobotinterface_noSkinNoRight.ini
iCubStartup:
- iKinCartesianSolver -part left_arm
- iKinGazeCtrl ...
- wholeBodyDynamics     icubbrain1   -headV2 -autocorrect -no_right_arm
- gravityCompensator    icubbrain2   -headV2 -no_right_arm
- fingersTuner          icub-laptop
- imuFilter             pc104

