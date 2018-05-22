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
//#include "placing.h"
//#include "passing.h"
#include "compute.h"

#define NBSAMPLES 1

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;


    bool ControlThread::threadInit()
    {
        //initialize here variables
        printf("ControlThread:starting\n");

        // Open cartesian solver for right and left arm
        string robot="icubSim";

        if (!openCartesian(robot,"left_arm"))
        {
            drvArmL.close();
            return false;
        }
        // SELECT THE HAND
        //
        _hand="left";

	    // select the correct interface
        if (_hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

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
        x.resize(3);
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
        cmd2.addDouble(0.10);
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
        cmd2.addDouble(0.3);
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
        cmd2.addDouble(0.5);
        // ball's colour
        cmd2.addDouble(0);
        cmd2.addDouble(1);
        cmd2.addDouble(0);

        printf("Sending message... %s\n", cmd2.toString().c_str());
        objectLocation.write(cmd2,reply);
        printf("Got response: %s\n", reply.toString().c_str());

        objectLocation.close();

        getchar();

        // initiate variables for giving action
        count       = 0;
        Vmax        = 0.02; // 0.5 m/s
        epsilon     = 0.1; // 10 cm

        e.resize(3);
        ed.resize(3);
        unit_e.resize(3);
        v_mag = 0;
        acc_mag = 0.01;

        // pre-define initial position
        xi.resize(3);
		xi[0] = -0.15;  //move forward 25 cm
		xi[1] = -0.20;  //move left 5 cm
		xi[2] =   0.1;  //move down 10 cm

        // pre-define final handover position
        xf.resize(3);
        xf[0] = -0.4;
        xf[1] = -0.05;
        xf[2] =  0.15;

        // get arm in initial position
        x[1] =  -0.5; // to the left
        startingArm(x);
    
        // define the fingers joints
        abduction.push_back(7);
        thumb.push_back(8);
        for (int i=9; i<16; i++)
            fingers.push_back(i);

        getchar();

        // grasp the ball

        // the "closure" accounts for how much we should
        // close the fingers around the object:
        // if closure == 0.0, the finger joints have to reach their minimum
        // if closure == 1.0, the finger joints have to reach their maximum
        double fingers_closure=0.5; // default value
        bool ok=grasp_it(fingers_closure);
        // we assume the robot is not moving now
        if (ok)
        {
            reply.addString("ack");
            reply.addString("Yeah! I did it! Maybe...");
        }
        else
        {
            reply.addString("nack");
            reply.addString("I don't see any object!");
        }   

        // Select Action
        int num = 2; 

        string Result; 
        std::ostringstream Result_string;
        Result_string << num;
        Result = Result_string.str();
        bool convert;

        Eyes = loadDataFile("gazeBehavior_" + Result + ".txt", convert = true);  

        if (act.getAction(action))
        {
            yInfo()<<" retrieved Action = ("<<action <<")";
        }
        else 
            action = -1;     

        return true;
    }

    bool ControlThread::openCartesian(const string &robot, const string &arm)
    {
        PolyDriver &drvArm=(arm=="right_arm"?drvArmR:drvArmL);

        Property optArm;
        optArm.put("device","cartesiancontrollerclient");
        optArm.put("remote","/"+robot+"/cartesianController/"+arm);
        optArm.put("local","/cartesian_client/"+arm);

        // let's give the controller some time to warm up
        bool ok=false;
        double t0=Time::now();
        while (Time::now()-t0<10.0)
        {
            // this might fail if controller
            // is not connected to solver yet
            if (drvArm.open(optArm))
            {
                ok=true;
                break;
            }

            Time::delay(1.0);
        }
        if (!ok)
        {
            yError()<<"Unable to open the Cartesian Controller for "<<arm;
            return false;
        }
        return true;
    }

    float ControlThread::magnitude(Vector x)     //  <! Vector magnitude
    {
        return sqrt((x[0]*x[0])+(x[1]*x[1])+(x[2]*x[2]));
    }

    /***************************************************/
    void ControlThread::fixate(int maxState)
    {

        switch(maxState) {
            case 1 : {
                        cout << '1' << endl; 
                        Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-40.0;
                            igaze->lookAtAbsAngles(ang);
                        }

                        double timeout = 10.0; 
                        bool done = false; 
                        done = igaze->waitMotionDone(0.1,timeout); 
                        if(!done){
                            yWarning("Something went wrong, using timeout");
                            igaze->stopControl();
                        }
                        // look for red ball
                        Bottle *pTarget=port.read(false);
                        if (pTarget!=NULL)
                        {
                            if (pTarget->size()>2)
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

                                // track the moving target within the camera image
                                igaze->lookAtMonoPixel(0,px2); // 0: left image, 1: for right
                                yInfo()<<"gazing at Brick: "<<px2.toString(3,3);
                            }
                        }
                        break;
                     }
            case 2 : {
                        cout << '2' << endl; 

                        // look up if you haven't already
                        Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-40.0;
                            igaze->lookAtAbsAngles(ang);
                        }

                        Vector x, o;
                        iarm->getPose(x,o); //get current position of hand
                        yInfo()<<"fixating the Human's face";

                        Vector look = x;
                        look[0] = -0.55;
                        look[1] =  0.00;
                        look[2] =  0.45;         

                        igaze->lookAtFixationPoint(look);
                        //igaze->waitMotionDone();
                        //to track from now on
                        igaze->setTrackingMode(true);
                        break;
                     }
            case 3 : {
                        cout << '3' << endl; 

                        // look up if you haven't already
                        Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-40.0;
                            igaze->lookAtAbsAngles(ang);
                        }

                        Vector x, o;
                        iarm->getPose(x,o); //get current position of hand
                        yInfo()<<"fixating the Human's hand";

                        Vector look = x;
                        look[0] = -0.55;
                        look[1] =  0.00;
                        look[2] =  0.25;         

                        igaze->lookAtFixationPoint(look);
                        //igaze->waitMotionDone();
                        //to track from now on
                        igaze->setTrackingMode(true);
                        break;
                     }
            case 4 : {
                        cout << '4' << endl; 

                        // look up if you haven't already
                        Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-40.0;
                            igaze->lookAtAbsAngles(ang);
                        }

                        Vector x, o;
                        iarm->getPose(x,o); //get current position of hand
                        yInfo()<<"fixating at Own hand";

                        Vector look = x;
                        look[0] = -0.25;
                        look[1] = -0.05;
                        look[2] =  0.10;         

                        igaze->lookAtFixationPoint(look);
                        //igaze->waitMotionDone();
                        //to track from now on
                        igaze->setTrackingMode(true);
                        break;

                     }
            case 5 : {
                        cout << '5' << endl; 

                        // make iCub look down
                        Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-40.0;
                            igaze->lookAtAbsAngles(ang);
                        }

                        double timeout = 10.0; 
                        bool done = false; 
                        done = igaze->waitMotionDone(0.1,timeout); 
                        if(!done){
                            yWarning("Something went wrong, using timeout");
                            igaze->stopControl();
                        }
                        // look for red ball
                        Bottle *pTarget=port.read(false);
                        if (pTarget!=NULL)
                        {
                            if (pTarget->size()>2)
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

                                // track the moving target within the camera image
                                igaze->lookAtMonoPixel(0,px1); // 0: left image, 1: for right
                                yInfo()<<"gazing at Teammate's Tower: "<<px1.toString(3,3);
                            }
                        }
                        break;
                     }
            case 6 : {
                        cout << '6' << endl; 

                        // make iCub look down
                        Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-40.0;
                            igaze->lookAtAbsAngles(ang);
                        }

                        double timeout = 10.0; 
                        bool done = false; 
                        done = igaze->waitMotionDone(0.1,timeout); 
                        if(!done){
                            yWarning("Something went wrong, using timeout");
                            igaze->stopControl();
                        }
                        // look for red ball
                        Bottle *pTarget=port.read(false);
                        if (pTarget!=NULL)
                        {
                            if (pTarget->size()>2)
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

                                igaze->lookAtMonoPixel(0,px3); // 0: left image, 1: for right
                                yInfo()<<"gazing at My Tower: "<<px3.toString(3,3);
                            }
                        }
                        break;
                     }
        }
        yInfo() << "Which state was chosen?";
    }

    /***************************************************/
    bool ControlThread::place()
    {
        Vector x, o;
        iarm->getPose(x,o); //get current position of hand
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        // select the correct interface
        if (_hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

	    // half point
            Vector move = x;
	    if (_hand == "right"){
		    move[0] = -0.25;  //move forward 10 cm
		    move[1] =  0.05;  //move left 20 cm
		    move[2] =   0.1;  //move down 10 cm
	    }else{
		    move[0] = -0.25;  //move forward 10 cm
		    move[1] = -0.05;  //move left 20 cm
		    move[2] =   0.1;  //move down 10 cm
	    }
        
        iarm->goToPose(move, o);

	    // final end point
	    if (_hand == "right"){
		    move[0] = -0.30;  //move backwards 10 cm
		    move[1] =  0.05;  //move right 20 cm
		    move[2] =  0.02;  //move down 10 cm
	    }else{
		    move[0] = -0.35;  //move forward 10 cm
		    move[1] = -0.05;  //move left 20 cm
		    move[2] =  0.02;  //move down 10 cm
	    }

        // fixate the new end goal
        igaze->lookAtFixationPoint(move);
        yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

        o = computeHandOrientationPassing(_hand); //get default orientation
        iarm->goToPose(move, o);

        Time::delay(1.0);

	    igaze->lookAtFixationPoint(move);

        // we set up here the lists of joints we need to actuate
        VectorOf<int> fingers;

        for (int i=9; i<16; i++)
            fingers.push_back(i);

        // release
        moveFingers(_hand,fingers,0.0);
        yInfo()<<"released";

        return true;        
    }

    /***************************************************/
    void ControlThread::arm(int maxState)
    {

        switch(maxState) {
            case 1 : {
                        cout << "Brick" << endl; 

                        break;
                     }
            case 2 : {
                        cout << "Human's face" << endl; 
                        // get current location
                        iarm->getPose(p,o);
                        iarm->setPosePriority("orientation");
                        Vector od = o;
                        // get current velocities
                        iarm->getTaskVelocities(vcur, wcur);

                        od = computeHandOrientationPassing(_hand); //get default orientation
                        reachArmGiving(p, od, xf, vcur);

                        break;
                     }
            case 3 : {
                        cout << "Human's Hand" << endl; 

                        // get current location
                        iarm->getPose(p,o);
                        iarm->setPosePriority("orientation");
                        Vector od = o;
                        // get current velocities
                        iarm->getTaskVelocities(vcur, wcur);

                        od = computeHandOrientationPassing(_hand); //get default orientation
                        reachArmGiving(p, od, xf, vcur);

                        break;
                     }
            case 4 : {
                        cout << "Own Hand" << endl; 
                        break;

                     }
            case 5 : {
                        cout << "Teammates' Tower" << endl; 

                        // get current location
                        iarm->getPose(p,o);

                        Vector od = o;
                        // get current velocities
                        iarm->getTaskVelocities(vcur, wcur);

                        od = computeHandOrientationPassing(_hand); //get default orientation
                        reachArmGiving(p, od, xf, vcur);
                        break;
                     }
            case 6 : {
                        cout << "My Tower" << endl; 

                        // get current pose of hand 
                        iarm->getPose(x,o);
                        iarm->setPosePriority("orientation");

                        place();

                        break;
                     }
        }
        yInfo() << "Which state was chosen?";
    }

    void ControlThread::threadRelease()
    {
        printf("ControlThread:stopping the robot\n");

        iarm->stopControl();

        igaze->restoreContext(startup_ctxt_gaze);
        igaze->stopControl();
        drvGaze.close();
        port.close();
        rpcPort.close();
        
        printf("Done, goodbye from ControlThread\n");
    }

    /***************************************************/
    void ControlThread::startingArm(const Vector &x)
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

        double target[] = {-17, 20, 8, 41, 7, -21, -9, 48, 76, 31, 86, 58, 131, 54, 113, 164};
        // we set up here the lists of joints we need to actuate
        // shoulders (3) + elbow + wrist (3)
        VectorOf<int> joints;
        for (int i=0; i<16; i++){
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
    }

    /***************************************************/
    void ControlThread::initArm(const Vector &x)
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

    /***************************************************/
    void ControlThread::moveFingers(const string &hand, const VectorOf<int> &joints,
                    const double fingers_closure)
    {
        // select the correct interface

        // this should be in the beginning, where you initialize the necessary stuff
        IControlLimits2   *ilim;
        IPositionControl2 *ipos;
        IEncoders         *ienc;
        IControlMode2     *imod;

        if (hand=="right")
        {
            drvHandR.view(ilim);
            drvHandR.view(ipos);
            drvHandR.view(imod);
        }
        else
        {
            drvHandL.view(ilim);
            drvHandL.view(ipos);
            drvHandL.view(imod);
        }

        // enforce [0,1] interval
        double fingers_closure_sat=std::min(1.0,std::max(0.0,fingers_closure));

        // move each finger first:
        // if min_j and max_j are the minimum and maximum bounds of joint j,
        // then we should move to min_j+fingers_closure_sat*(max_j-min_j)

        // This option you will move each finger individually
        for (size_t i=0; i<joints.size(); i++)
        {
            int j=joints[i];
            yInfo()<<"j" << j;
            // retrieve joint bounds
            double min_j,max_j,range;
            ilim->getLimits(j,&min_j,&max_j);
            range=max_j-min_j;
            // select target
            double target;
            target=min_j+fingers_closure_sat*(range);
            // set control mode
            imod->setControlMode(j,VOCAB_CM_POSITION);
            // set up the speed in [deg/s]
            ipos->setRefSpeed(j,30.0);
            // set up max acceleration in [deg/s^2]
            ipos->setRefAcceleration(j,100.0);

            // yield the actual movement
            yInfo()<<"Yielding new target: "<<target<<" [deg]";
            ipos->positionMove(j,target);
        }
    }

    void ControlThread::reachArmGiving(Vector desired_p, Vector orientation, Vector x_pos, Vector velocity)
    {
        e[0] = x_pos[0] - desired_p[0];
        e[1] = x_pos[1] - desired_p[1];
        e[2] = x_pos[2] - desired_p[2]; 
        yInfo() << "e[0]:" << e[0] << "e[1]" << e[1] << "e[2]" << e[2];       

        mag_e = magnitude(e);
        unit_e[0] = e[0]/mag_e;  // this gives us the orientation of the 
        unit_e[1] = e[1]/mag_e;  // equilibrium point
        unit_e[2] = e[2]/mag_e;

        if( mag_e < epsilon ){
            v_mag = 0.9*v_mag*mag_e/epsilon;
            vcur[0] = v_mag * unit_e[0];
            vcur[1] = v_mag * unit_e[1];
            vcur[2] = v_mag * unit_e[2];
            wcur[0] = v_mag * unit_e[0];
            wcur[1] = v_mag * unit_e[1];
            wcur[2] = v_mag * unit_e[2];
            //iarm->setTaskVelocities(vcur, wcur);
           /* if(!send_or){
                send_or=1;
                Vector o = computeHandOrientation("left");
                approachTargetWithHand("left", x, o);
            } */
        }else if( mag_e > epsilon and magnitude(vcur) < Vmax){
            v_mag+=acc_mag;   

            vcur[0] = v_mag * unit_e[0];
            vcur[1] = v_mag * unit_e[1];
            vcur[2] = v_mag * unit_e[2];
            wcur[0] = v_mag * unit_e[0];
            wcur[1] = v_mag * unit_e[1];
            wcur[2] = v_mag * unit_e[2];
            //iarm->setTaskVelocities(vcur, wcur);
        }else{
            v_mag = Vmax;

            vcur[0] = v_mag * unit_e[0];
            vcur[1] = v_mag * unit_e[1];
            vcur[2] = v_mag * unit_e[2];
            wcur[0] = v_mag * unit_e[0];
            wcur[1] = v_mag * unit_e[1];
            wcur[2] = v_mag * unit_e[2];
            //iarm->setTaskVelocities(vcur, wcur);
        }
        iarm->goToPose(x_pos,orientation);

        if (count < 20 ){
            // let's put the hand in the pre-grasp configuration
            moveFingers(_hand, fingers, 0.0);
        } else {
            //closing the hand
            moveFingers(_hand, thumb,    1.0);
            moveFingers(_hand, fingers,  0.5);
        }

        yInfo() << "v[0]:" << vcur[0] << "v[1]" << vcur[1] << "v[2]" << vcur[2];    
        yInfo() << "w[0]:" << wcur[0] << "w[1]" << wcur[1] << "w[2]" << wcur[2];    

    }

    /***************************************************/
    void ControlThread::release(string hand)
    {
        IControlLimits2   *ilim;
        IPositionControl2 *ipos;
        IControlMode2     *imod;

        if (hand=="right")
        {
            drvHandR.view(ilim);
            drvHandR.view(ipos);
            drvHandR.view(imod);
        }
        else
        {
            drvHandL.view(ilim);
            drvHandL.view(ipos);
            drvHandL.view(imod);
        }

        double target[16] = {0, 90, 0, 20, 10, 0, 0, 50, 10, 0, 0, 0, 0, 0, 0, 0};   //icubSim
        //double target[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0};   // icub

        // we set up here the lists of joints we need to actuate (just hand)
        // shoulders (3) + elbow + wrist (3) + hand openning + thumb (3)
        VectorOf<int> joints;
        for (int i=0; i<16; i++){
            joints.push_back(i);
        }

        // This option you will move each joint individually
        for (size_t i=7; i<joints.size(); i++)
        {
            int j=joints[i];
            // retrieve joint bounds
            double min_j,max_j,range;
            ilim->getLimits(j,&min_j,&max_j);
            // set up the speed in [deg/s]
            ipos->setRefSpeed(j,15.0);
            // set up max acceleration in [deg/s^2]
            ipos->setRefAcceleration(j,5.0);
            // yield the actual movement
            yInfo()<<"Yielding new target: "<<target[i]<<" [deg]";
            imod->setControlMode(j,VOCAB_CM_POSITION);
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
        if (count < 1000){
            look = Eyes[0][1];

            fixate(look);
            yInfo()<<"fixating at ("<< look <<")";

        // duration of action
        }else if (count > 1015 and count < 2000){

            look = Eyes[count-1000][1];

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
        if ((Time::now()-startTime)>10)
            done=true;
    }
    
    myThread.stop();

    return 0;
}

