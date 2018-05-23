// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/IControlLimits2.h>         // Control Limits for Drivers
#include <yarp/dev/ControlBoardInterfaces.h>  // joint control
#include <yarp/dev/CartesianControl.h>        // cartesian control
#include <yarp/dev/GazeControl.h>             // gaze control
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <opencv2/core/mat.hpp>               // add Mat variables from OpenCV

#include <string>
#include <iostream>
#include <math.h>
#include <complex>
#include <sstream>
#include <fstream>

//#include "init.h"
#include "compute.h"

#define NBSAMPLES 1

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

    std::vector< std::vector<float> > ControlThread::loadDataFile(std::string file, 
                                                                    bool convert = false)
    {
        // Load the dataset from a file
        float valTmp;
        char tmp[1024];
        unsigned int l=0, c=0;
        // initialize the vector size
        std::vector< std::vector<float> > result(
                1001,
                std::vector<float>(4)); 

        yInfo() << "loading file: " << file;
        std::ifstream f(file.c_str());

        if (f.is_open()){
            // Get number of rows
            while(!f.eof()){ 
                f.getline(tmp,1024);
                l++;
                if (l==1)
                {
                    // Get number of columns
                    std::istringstream strm;
                    strm.str(tmp); 
                    while (strm >> valTmp)
                    c++;
                }
            }
            l--;
            f.clear();
            f.seekg(0); // returns to beginning of the file

            // for head and arm data
            if (not convert) {
                for(unsigned int i=0;i<l;i++){ 
                    f.getline(tmp,1024);
                    std::istringstream strm;
                    strm.str(tmp); 
                    for(unsigned int j=0;j<c;j++){
                        strm >> result[i][j];
                        //printf("%f ",result[i][j]);
                    }
                    //printf("\n");
                }
            // for eyes data
            } else {
                for(unsigned int i=0;i<l;i++){ 
                    f.getline(tmp,1024);
                    std::istringstream strm;
                    strm.str(tmp); 
                    for(unsigned int j=0;j<c;j++){
                        strm >> result[i][j];
                        if (j == 1){
                            if (result[i][j] == 1){
                                result[i][1] = -0.55;
                                result[i][2] =  0.00;
                                result[i][3] =  0.45;                    
                            }else if (result[i][j] == 2){
                                result[i][1] = -0.35;  //move forward 10 cm
                                result[i][2] = -0.19;  //move left 20 cm
                                result[i][3] =  0.02;  //move down 10 cm
                            }else if (result[i][j] == 3){
                                result[i][1] = -0.25;
                                result[i][2] =  0.30;
                                result[i][3] =  0.45;                
                            }
                        }else{
                            continue;//printf("%f ", float(result[i][j]));
                        }
                    }
                    printf("\n");
                }
            }
        }else{
            std::cout  << std::endl << "Error opening file " << std::endl; 
        }
        f.close();
        return result;
    }    

    void ControlThread::run()
    {
      	
        count++;

        int look; // I need to change this to the correct type of variable
        cv::Mat state;
        /*int nRows = state.rows;
        int nCols = state.cols;
        cout << "MaxStates = "<< endl << " "  << state << endl << endl;

        // we only want the last column
        int i,j = nCols - 1;
        double *p;
        for( i = 0; i < nRows; ++i)
        {   
            // get the max probability and the state index
            p = state.ptr<double>(i);
            yInfo() << p[j];
            if (max < p[j]) {
                max = p[j];
                id = i;
            }
        }*/

        // begin
        if (count < 1){
            look = Eyes[0][1];

            fixate(look);
            yInfo()<<"fixating at ("<< look <<")";

        // duration of action
        }else if (count > 1 and count < 200){

            look = Eyes[count-1][1];

            fixate(look);
            yInfo()<<"fixating at ("<< look <<")";
            arm(look);


        // finish
        }else if (count > 2000){

            look = Eyes[999][1];

            fixate(look);
            yInfo()<<"fixating at ("<< look <<")";
        }

    }

int main(int argc, char *argv[]) 
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        printf("No yarp network, quitting\n");
        return 1;
    }

    RpcClient objectLocation;
    objectLocation.open("/objectBall");
    
    printf("Trying to connect to %s\n", "/icubSim/world");
    yarp.connect("/objectBall","/icubSim/world");
    Bottle cmd1, reply;

    // CREATE the Table not affected by gravity we will use
    cmd1.addString("world");
    cmd1.addString("mk"); 
    cmd1.addString("sbox");
    // table's size
    cmd1.addDouble(1.0);
    cmd1.addDouble(0.05);
    cmd1.addDouble(1.0);
    // table's position
    cmd1.addDouble(0);
    cmd1.addDouble(0.45);
    cmd1.addDouble(0.6);
    // table's colour
    cmd1.addDouble(1.0);
    cmd1.addDouble(1.0);
    cmd1.addDouble(0.7);

    printf("Sending message... %s\n", cmd1.toString().c_str());
    objectLocation.write(cmd1,reply);
    printf("Got response: %s\n", reply.toString().c_str());

    objectLocation.close();

    ControlThread myThread(5); //period is 10ms

    myThread.start();

    bool done=false;
    double startTime=Time::now();
    while(!done)
    {
        if ((Time::now()-startTime)>30)
            done=true;
    }
    
    myThread.stop();

    return 0;
}

