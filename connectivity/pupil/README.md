# How to make LSL pupil - YARP work (running on Ubuntu 16.04 LTS)
1. install CodeBlocks
2. install PyCharm or any alternative
3. install YARP  2.3.72
4. install Pupil 1.7.42
5. install Pupil Plugin - LSL Relay
6. install Pupil Plugin - Pupil Remote
7. open Pupil - python3 main.py 
8. Turn LSL Relay and Pupil Remove on 
9. get github repo gazePupil_iCub
10. open onlineDetection.py on PyCharm
11. install OpenCV on python3
12. install zmq on python3 
13. install pylsl on python3 
14. import utils
to import utils you need to install tensorflow with gpu then get the models of tensorflow for object recognition to recognize the import 
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
then the project should detect the utils (DO NOT install any other utils library)

17. in CodeBlocks you probably will not be able to detect the appropriate libraries (yarp, lsl, json)
18. implement the following in the "Build Options" of your project in CodeBlocks:
![1st part](https://github.com/NunoDuarte/armCoupling_iCub/blob/master/lsl/images/IMG_2989.jpg)
![2nd part](https://github.com/NunoDuarte/armCoupling_iCub/blob/master/lsl/images/IMG_2990.jpg)
![3rd part](https://github.com/NunoDuarte/armCoupling_iCub/blob/master/lsl/images/IMG_2991.jpg)

19. after you do this it should work (if not, there is something wrong, good luck) -> check # Issues for help
20. run main.cpp (check which functions are on: #define LSL_OT, LSL_PL1, LSL_PL1YRP, LSL_PL2 
21. The #define LSL_OT should be off
22. The #define LSL_PL1 and LSL_PL1YRP should be on
23. uncomment -> threads.push_back(std::thread(pl1_capture));
24. uncomment -> threads.push_back(std::thread(pl1_yarp));
25. open terminal
26. yarpserver --namespace
27. run CodeBlocks 
28. python3 onlineDetection.py
## there is something missing here  
### if you want to publish to yarp you need to change the code in onlineDetection - the GazeBehaviour file is the main part
30. check in YARP if the port pupil_gaze_tracker exists 

# Issues
## if you find this problem when using codeblocks 
- pthread_create@2.2.5 
The solution is to add the thread library to both the compiler and linker 
- source: https://askubuntu.com/questions/568068/multithreading-in-codeblocks
You need to do two things:
1. Build options -> Compiler Settings -> Other options; Add -pthread
2. Build options -> Linker Settings -> Other linker options; Add -pthread

## issue with not recognizing libYARP_init.so.3 even though it recognizes libYARP_init.so
- quick workaround is to add the path to the yarp libs to the Search directories -> Linker 
- source: http://forums.codeblocks.org/index.php?topic=18661.0 
- e.g. project->Build options...->Search directories->Linker and add the directory there
- ../../../../middleware/yarp/build/lib

## cannot find shared library (e.g. liblsl64.so.1.2.0) what to do?
It works if I right click project->Build options...->Search directories->Linker and add the directory there.
The directory where liblsl64.so.1.2.0 is for example

