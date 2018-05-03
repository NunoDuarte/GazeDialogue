// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>  // joint control
#include <yarp/dev/CartesianControl.h>        // cartesian control
#include <yarp/dev/GazeControl.h>             // gaze control
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>

#include <string>
#include <iostream>
#include <math.h>
#include <complex>
#include <sstream>
#include <fstream>

#include "helpers.h"

#define NBSAMPLES 1

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

class ControlThread: public RateThread
{
    PolyDriver drvGaze;

    ICartesianControl *iarm;
    IGazeControl      *igaze;
    ObjectRetriever object;
    Port inPort;

    BufferedPort<Bottle> port;

    int startup_ctxt_gaze;

public:
    ControlThread(int period):RateThread(period){}

    bool threadInit()
    {
        //initialize here variables
        printf("ControlThread:starting\n");

        // Open cartesian solver for right and left arm
        string robot="icubSim";
        // open a client interface to connect to Gaze Cartesian controller
        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");

        if (!drvGaze.open(optGaze)){
            yError() << "Unable to open the Gaze controller";
            return false;
        }
        
        drvGaze.view(igaze);
        igaze->storeContext(&startup_ctxt_gaze);

        inPort.open("/read_pupil");

        port.open("/tracker/target:i");

        getchar();

        return true;
    }

    /***************************************************/
    void fixate(const Vector &x)
    {
        // simply look at x,
        // but when the movement is over
        // ensure that we'll still be looking at x

        igaze->lookAtFixationPoint(x);
        //igaze->waitMotionDone();
        //to track from now on
        igaze->setTrackingMode(true);
    }

    void threadRelease()
    {
        printf("ControlThread:stopping the robot\n");

        igaze->restoreContext(startup_ctxt_gaze);
        igaze->stopControl();
        drvGaze.close();
        port.close();
        
        printf("Done, goodbye from ControlThread\n");
    }

    void run()
    {
      	Vector pupil; string hand; Vector gaze;
	    yInfo() << "Im in !!!!!!";
        if (inPort.read(pupil))
        {
            if ( pupil(1) == 0){
                // make iCub look down
                Vector ang(3,0.0);
                ang[1]=-40.0;
                //ang[2]=-20;
                igaze->lookAtAbsAngles(ang);

                double timeout = 10.0; 
                bool done = false; 
                done = igaze->waitMotionDone(0.1,timeout); 
                if(!done){
                    yWarning("Something went wrong with the initial approach, using timeout");
                    igaze->stopControl();
                }
                // look for red ball
                Bottle *pTarget=port.read(false);
                if (pTarget!=NULL)
                {
                    if (pTarget->size()>2)
                    {
                        if (pTarget->get(2).asInt()!=0)
                        {
                            Vector px(2);
                            px[0]=pTarget->get(0).asDouble();
                            px[1]=pTarget->get(1).asDouble();

                            // track the moving target within the camera image
                            igaze->lookAtMonoPixel(0,px); // 0: left image, 1: for right
                            yInfo()<<"gazing at pixel: "<<px.toString(3,3);
                        }
                    }
                }
            }else{
                //yInfo()<<"retrieved 3D location = ("<<x.toString(3,3)<<")";
                //yInfo() << x(0);
                //yInfo() << x(1);
                //yInfo() << x(2);
                gaze = pupil;
                gaze(0) = -20; // -20 meters for x in robot frame (20 meters in z for pupil)
                gaze(1) =  0;
                gaze(2) =  0;

                yInfo() << gaze(0);
                yInfo() << gaze(1);
                yInfo() << gaze(2);
                fixate(gaze);
            }
        }
        else
            yInfo() << "didn't get object Location";
    }
};

int main(int argc, char *argv[]) 
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        printf("No yarp network, quitting\n");
        return 1;
    }

    ControlThread myThread(5); //period is 10ms

    myThread.start();

    bool done=false;
    double startTime=Time::now();
    while(!done)
    {
        if ((Time::now()-startTime)>50)
            done=true;
    }
    
    myThread.stop();

    return 0;
}

