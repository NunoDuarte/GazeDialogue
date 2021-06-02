# Python Project for PupilLabs interface for object and iCub detection using eye-tracker
Software for Identifying from Gaze Fixations the Recognized Object or Face

### Current version of onlineDetect uses tensorflow object detection network trained on icub faces
So you need to get the icub-face-detection repo to make it work

## Requirements to make it work
1. add anaconda
2. source activate pupilos
3. tensorflow models - object detection
4. pylsl
5. labstreaminglayer/liblsl (both work)
6.1. I got https://github.com/NunoDuarte/lsl_archived which has the version I need (because it was working)
7. Install, compile, and sudo make from the lsl_archived
8. yarp
9. pupil with zmq plugin
10. pupil LSL plugin (**pupil_lsl_relay.py**)
11. pupil with pupil remote and frame publisher activated
12. pupil-stream-lsl project (other repository)
13. check the requirements of the pupil-stream-lsl 
14. check that pupil-stream-lsl has the correct threads activated

## if you find this problem when using codeblocks 
- pthread_create@2.25 
The solution is to add the thread library to both the compiler and linker 
- source: https://askubuntu.com/questions/568068/multithreading-in-codeblocks

## issue with not recognizing libYARP_init.so.3 even though it recognizes libYARP_init.so
- quick workaround is to add the path to the yarp libs to the Search directories -> Linker 
- source: http://forums.codeblocks.org/index.php?topic=18661.0 
- e.g. project->Build options...->Search directories->Linker and add the directory there
- ../../../../middleware/yarp/build/lib

## cannot find shared library (e.g. liblsl64.so.1.2.0) what to do?
It works if I right click project->Build options...->Search directories->Linker and add the directory there.
The directory where liblsl64.so.1.2.0 is for example
