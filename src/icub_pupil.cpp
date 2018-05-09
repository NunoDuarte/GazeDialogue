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
    PolyDriver drvGaze, drvHandR, drvHandL;

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

        // open a client interface to connect to the joint controller
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote","/icubSim/left_arm");
        optJoint.put("local","/position/left_arm");

        if (!drvHandL.open(optJoint))
        {
            yError()<<"Unable to connect to /icubSim/left_arm";
            return false;
        }

        // open a client interface to connect to the joint controller
        Property optJoint1;
        optJoint1.put("device","remote_controlboard");
        optJoint1.put("remote","/icubSim/right_arm");
        optJoint1.put("local","/position/right_arm");

        if (!drvHandR.open(optJoint1))
        {
            yError()<<"Unable to connect to /icubSim/right_arm";
            return false;
        }

        yInfo()<<"Begin moving arms to first initial position";
        // GET ARMS IN THE CORRECT POSITION
        Vector x(3);

        x[1] =  0.5; // to the right
        initArm(x);
        x[1] =  -0.5; // to the left
        initArm(x);

	    yInfo()<<"Finished moving the arms to initial position.";

        Network yarp;

        RpcClient objectLocation;
        objectLocation.open("/objectBall");
        
        printf("Trying to connect to %s\n", "/icubSim/world");
        yarp.connect("/objectBall","/icubSim/world");
        Bottle reply;

        Bottle cmd2;
        // Blue ball
        // CREATE the sphere affected by gravity we will use
        cmd2.addString("world");
        cmd2.addString("mk"); 
        cmd2.addString("sph");
        cmd2.addDouble(0.04); // radius 4 cm (made the ball smaller)
        // ball's position
        cmd2.addDouble(-0.10);
        cmd2.addDouble(0.59);
        cmd2.addDouble(0.3);
        // ball's colour
        cmd2.addDouble(0);
        cmd2.addDouble(0);
        cmd2.addDouble(1);

        printf("Sending message... %s\n", cmd2.toString().c_str());
        objectLocation.write(cmd2,reply);
        printf("Got response: %s\n", reply.toString().c_str());

        cmd2.clear();

        // Red Space
        // CREATE the sphere affected by gravity we will use
        cmd2.addString("world");
        cmd2.addString("mk"); 
        cmd2.addString("sbox");
        cmd2.addDouble(0.15); // radius 4 cm 
        cmd2.addDouble(0.01); // length 4 cm 
        cmd2.addDouble(0.15); // length 4 cm 
        // ball's position
        cmd2.addDouble(0.0);
        cmd2.addDouble(0.48);
        cmd2.addDouble(0.4);
        // ball's colour
        cmd2.addDouble(1);
        cmd2.addDouble(0);
        cmd2.addDouble(0);

        printf("Sending message... %s\n", cmd2.toString().c_str());
        objectLocation.write(cmd2,reply);
        printf("Got response: %s\n", reply.toString().c_str());

        cmd2.clear();

        // Green Space
        // CREATE the sphere affected by gravity we will use
        cmd2.addString("world");
        cmd2.addString("mk"); 
        cmd2.addString("sbox");
        cmd2.addDouble(0.15); // radius 4 cm 
        cmd2.addDouble(0.01); // length 4 cm 
        cmd2.addDouble(0.15); // length 4 cm 
        // ball's position
        cmd2.addDouble(0.0);
        cmd2.addDouble(0.48);
        cmd2.addDouble(0.8);
        // ball's colour
        cmd2.addDouble(0);
        cmd2.addDouble(1);
        cmd2.addDouble(0);

        printf("Sending message... %s\n", cmd2.toString().c_str());
        objectLocation.write(cmd2,reply);
        printf("Got response: %s\n", reply.toString().c_str());

        objectLocation.close();

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

    /***************************************************/
    void initArm(const Vector &x)
    {
        string hand=(x[1]>=0.0?"right":"left");

        // select the correct interface
        IControlLimits2   *ilim1;
        IPositionControl2 *ipos;
        IEncoders         *ienc1;
        IControlMode2     *imod1;

        if (hand=="right")
        {
            drvHandR.view(ilim1);
            drvHandR.view(ipos);
            drvHandR.view(imod1);
        }
        else
        {
            drvHandL.view(ilim1);
            drvHandL.view(ipos);
            drvHandL.view(imod1);
        }

        double target[] = {0, 90, 0, 20, 10, 0, 0};
        // we set up here the lists of joints we need to actuate
        // shoulders (3) + elbow + wrist (3)
        VectorOf<int> joints;
        for (int i=0; i<6; i++){
            joints.push_back(i);
        }
        // This option you will move each joint individually
        for (size_t i=0; i<joints.size(); i++)
        {
            int j=joints[i];
            // retrieve joint bounds
            double min_j,max_j,range;
            ilim1->getLimits(j,&min_j,&max_j);
            // set control mode
            imod1->setControlMode(j,VOCAB_CM_POSITION);
            // set up the speed in [deg/s]
            ipos->setRefSpeed(j,15.0);
            // set up max acceleration in [deg/s^2]
            ipos->setRefAcceleration(j,5.0);
            // yield the actual movement
            yInfo()<<"Yielding new target: "<<target[i]<<" [deg]";
            ipos->positionMove(j,target[i]);
        }
        // wait (with timeout) until the movement is completed
        bool done=false;
        double t0=Time::now();
        while (!done && (Time::now()-t0 < 1.0))
        {
            yInfo()<<"Waiting...";
            Time::delay(0.1);   // release the quantum to avoid starving resources
            ipos->checkMotionDone(&done);
        }

        if (done)
            yInfo()<<"Movement completed";
        else
            yWarning()<<"Timeout expired";

        // wait until all fingers have attained their set-points
    }

    void run()
    {
      	Vector pupil; string hand; Vector gaze;
	    yInfo() << "Im in !!!!!!";
        if (inPort.read(pupil))
        {
            if ( pupil(1) == 1){
                yInfo() << "1";
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
                            Vector px1(2);
                            std::string str1;
                            Vector px2(2);
                            std::string str2;
                            Vector px3(2);
                            std::string str3;
                            px1[0]=pTarget->get(0).asDouble();
                            px1[1]=pTarget->get(1).asDouble();
                            str1=pTarget->get(2).asString();
                            px2[0]=pTarget->get(3).asDouble();
                            px2[1]=pTarget->get(4).asDouble();
                            str2=pTarget->get(5).asString();
                            px3[0]=pTarget->get(6).asDouble();
                            px3[1]=pTarget->get(7).asDouble();
                            str3=pTarget->get(8).asString();

	                        yInfo() << "Im in again !!!!!!\n";
                            // track the moving target within the camera image
                            if ( pupil(1) > 1.1111 and pupil(1) < 1.2222){
                                igaze->lookAtMonoPixel(0,px1); // 0: left image, 1: for right
                                yInfo()<<"gazing at green object: "<<px1.toString(3,3);
                            } if ( pupil(1) > 1.2222 and pupil(1) < 1.3333){
                                igaze->lookAtMonoPixel(0,px2); // 0: left image, 1: for right
                                yInfo()<<"gazing at red object: "<<px2.toString(3,3);
                            } if ( pupil(1) > 1.3333){
                                igaze->lookAtMonoPixel(0,px3); // 0: left image, 1: for right
                                yInfo()<<"gazing at blue object: "<<px3.toString(3,3);
                            }
                        }
                    }
                }
            }else if ( pupil(1) == 2) {
                yInfo() << "2";
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
                            Vector px1(2);
                            std::string str1;
                            Vector px2(2);
                            std::string str2;
                            Vector px3(2);
                            std::string str3;
                            px1[0]=pTarget->get(0).asDouble();
                            px1[1]=pTarget->get(1).asDouble();
                            str1=pTarget->get(2).asString();
                            px2[0]=pTarget->get(3).asDouble();
                            px2[1]=pTarget->get(4).asDouble();
                            str2=pTarget->get(5).asString();
                            px3[0]=pTarget->get(6).asDouble();
                            px3[1]=pTarget->get(7).asDouble();
                            str3=pTarget->get(8).asString();

	                        yInfo() << "Im in again !!!!!!\n";
                            // track the moving target within the camera image
                            if ( pupil(1) > 1.1111 and pupil(1) < 1.2222){
                                igaze->lookAtMonoPixel(0,px1); // 0: left image, 1: for right
                                yInfo()<<"gazing at green object: "<<px1.toString(3,3);
                            } if ( pupil(1) > 1.2222 and pupil(1) < 1.3333){
                                igaze->lookAtMonoPixel(0,px2); // 0: left image, 1: for right
                                yInfo()<<"gazing at red object: "<<px2.toString(3,3);
                            } if ( pupil(1) > 1.3333){
                                igaze->lookAtMonoPixel(0,px3); // 0: left image, 1: for right
                                yInfo()<<"gazing at blue object: "<<px3.toString(3,3);
                            }
                        }
                    }
                }

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
            }else if ( pupil(1) == 3) {
                yInfo() << "3";
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
                            Vector px1(2);
                            std::string str1;
                            Vector px2(2);
                            std::string str2;
                            Vector px3(2);
                            std::string str3;
                            px1[0]=pTarget->get(0).asDouble();
                            px1[1]=pTarget->get(1).asDouble();
                            str1=pTarget->get(2).asString();
                            px2[0]=pTarget->get(3).asDouble();
                            px2[1]=pTarget->get(4).asDouble();
                            str2=pTarget->get(5).asString();
                            px3[0]=pTarget->get(6).asDouble();
                            px3[1]=pTarget->get(7).asDouble();
                            str3=pTarget->get(8).asString();

	                        yInfo() << "Im in again !!!!!!\n";
                            // track the moving target within the camera image
                            if ( pupil(1) > 1.1111 and pupil(1) < 1.2222){
                                igaze->lookAtMonoPixel(0,px1); // 0: left image, 1: for right
                                yInfo()<<"gazing at green object: "<<px1.toString(3,3);
                            } if ( pupil(1) > 1.2222 and pupil(1) < 1.3333){
                                igaze->lookAtMonoPixel(0,px2); // 0: left image, 1: for right
                                yInfo()<<"gazing at red object: "<<px2.toString(3,3);
                            } if ( pupil(1) > 1.3333){
                                igaze->lookAtMonoPixel(0,px3); // 0: left image, 1: for right
                                yInfo()<<"gazing at blue object: "<<px3.toString(3,3);
                            }
                        }
                    }
                }

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
            } else
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
        if ((Time::now()-startTime)>50)
            done=true;
    }
    
    myThread.stop();

    return 0;
}

