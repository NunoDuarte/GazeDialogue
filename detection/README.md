# Python Project for PupilLabs interface for object and iCub detection using eye-tracker
Software for Identifying from Gaze Fixations the Recognized Object or Face

<img src="../doc/python.gif" width="800" height="450" />

### Current version of onlineDetect uses tensorflow object detection network trained on icub faces
So you need to get the icub-face-detection repo to make it work

## Requirements to make it work
1. add anaconda
2. source activate pupilos
3. tensorflow models - object detection
4. import utils
###  to import utils you need to install tensorflow with gpu then get the models of tensorflow for object recognition to recognize the import 
```
from utils import label_map_util
from utils import visualization_utils as vis_util
```
you need the following (after you have followed the instructions on how to install tensorflow models)
``` 
cd software/tensorflow/models/research
export PYTHONPATH=$PYTHONPATH:$(pwd)/slim
echo $PYTHONPATH 
export PYTHONPATH=$PYTHONPATH:$(pwd):$(pwd)/object_detection 
```
5. pylsl
6. labstreaminglayer/liblsl (both work)
7.1. I got https://github.com/NunoDuarte/lsl_archived which has the version I need (because it was working)
8. Install, compile, and sudo make from the lsl_archived
9. yarp
10. pupil with zmq plugin
11. pupil LSL plugin (**pupil_lsl_relay.py**)
12. pupil with pupil remote and frame publisher activated
13. pupil-stream-lsl project (other repository)
14. check the requirements of the pupil-stream-lsl 
15. check that pupil-stream-lsl has the correct threads activated

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