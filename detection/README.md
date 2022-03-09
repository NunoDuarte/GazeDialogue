# Identifying from Gaze Fixations the Desired Objects and Faces

<img src="../doc/python.gif" width="800" height="450" />

### Current version of main.py uses tensorflow object detection network trained on icub faces
So you need to get the [icub-face-detection](https://github.com/NunoDuarte/icub-face-detection) repo to make it work

# Table of Contents

- [Dependencies](#dependencies)
- [Installation](#installation)
- [Possible Issues](#issues)

## Dependencies
- There are two conda virtual environments which were tested and was running (**pupilos** and **pupilos-10**). Pupilos was working for tensorflow 1.9 and CUDA-8.0 and pupilos-10 was working for tensorflow 2.7 and CUDA-11.2
1. add anaconda
2. source activate pupilos***
3. tensorflow models - object detection
4. import utils

## Installation 
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

## Issues
If you find this problem when using codeblocks:
```
pthread_create@2.25 
```
The solution is to add the thread library to both the compiler and linker. You can find an example in this [link](https://askubuntu.com/questions/568068/multithreading-in-codeblocks).

If you find that it does not recognize libYARP_init.so.3 even though it recognizes libYARP_init.so. A good quick workaround is to add the path to the yarp libs to the Search directories. You can find an example in this [link](http://forums.codeblocks.org/index.php?topic=18661.0). You have to go to project->Build options...->Search directories->Linker and add the directory there "../../../../middleware/yarp/build/lib"

If you cannot find shared library (e.g. liblsl64.so.1.2.0) it works if you right click project->Build options...->Search directories->Linker and add the directory there. The directory where liblsl64.so.1.2.0 is for example
