# gazePupil_iCubSIM
Repository for the Modeling Human Gaze Behavior and Robot Application Project

# read camera output
yarpdev --device grabber --name /test/video --subdevice usbCamera --d /dev/video0
yarp connect /test/video /icubSim/texture/screen
