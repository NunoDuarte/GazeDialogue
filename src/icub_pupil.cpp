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
    PolyDriver drvArmR, drvArmL, drvGaze, drvHandR, drvHandL;

    ICartesianControl *iarm;
    IGazeControl      *igaze;
    ObjectRetriever object;
    ActionRetriever act;
    Port inPort;

    BufferedPort<Bottle> port;

    int startup_ctxt_gaze;
    string _hand;

    // position and orientation of the robot's arm
    Vector p, o;
    // current linear and angular velocities
    Vector vcur, wcur;
    // 3d location in the 3d world reference frame
    Vector x, xf, xi;

    // calculte distances for giving action
    float Vmax;
    float epsilon;
    int count;
    Vector e;
    Vector unit_e;
    double v_mag;
    double acc_mag;
    double mag_e; 

public:
    ControlThread(int period):RateThread(period){}

    bool threadInit()
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

        // initiate variables for giving action
        count       = 0;
        Vmax        = 0.02; // 0.5 m/s
        epsilon     = 0.01; // 10 cm

        e.resize(3);
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
        xf[2] =  0.02;

        return true;
    }

    bool openCartesian(const string &robot, const string &arm)
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

    float magnitude(Vector x)     //  <! Vector magnitude
    {
        return sqrt((x[0]*x[0])+(x[1]*x[1])+(x[2]*x[2]));
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

    /***************************************************/
    int predictAL(int state)
    {
        // input: state of the human
        // output: action to perform
        int action;

        if (act.getAction(action))
        {
            yInfo()<<" retrieved Action = ("<<action <<")";

            return action;
        }
        else 
            action = -1;

        
    }

    /***************************************************/
    void actionBehavior(int state)
    {
        // input: state of the human
        // output: behavior of robot
        int action;

        // call the predictor function
        action = predictAL(state);

        // make a decision based on the predictor's response
        if (action == 1){
            // get current location
            iarm->getPose(p,o);
            // get current velocities
            iarm->getTaskVelocities(vcur, wcur);

            reachArmGiving(p, o, xf, vcur);

        } else if (action == 0) {
            // just observe the action
            yInfo() << "I'm observing";

            iarm->getPose(p,o);
            // get current velocities
            iarm->getTaskVelocities(vcur, wcur);

            reachArmGiving(p, o, xi, vcur);

        } else 
            yInfo() << "Wrong action";
    }

    void reachArmGiving(Vector position, Vector orientation, Vector x_pos, Vector velocity)
    {
        e[0] = x_pos[0] - p[0];
        e[1] = x_pos[1] - p[1];
        e[2] = x_pos[2] - p[2];        

        /*if (count == 3500 ){
            release("left");
        }*/
        mag_e = magnitude(e);
        unit_e[0] = e[0]/mag_e;  // this gives us the orientation of the 
        unit_e[1] = e[1]/mag_e;  // equilibrium point
        unit_e[2] = e[2]/mag_e;

        if( mag_e < epsilon ){
            v_mag = 0.9*v_mag*mag_e/epsilon;
            vcur[0] = v_mag * unit_e[0];
            vcur[1] = v_mag * unit_e[1];
            vcur[2] = v_mag * unit_e[2];
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
            //iarm->setTaskVelocities(vcur, wcur);
        }else{
            v_mag = Vmax;

            vcur[0] = v_mag * unit_e[0];
            vcur[1] = v_mag * unit_e[1];
            vcur[2] = v_mag * unit_e[2];
            //iarm->setTaskVelocities(vcur, wcur);
        }
        iarm->goToPose(x_pos,o);
        yInfo() << "v:" << v_mag;

    }

    /***************************************************/
    void gazeBehavior(int state)
    {

        //igaze->lookAtFixationPoint(state);
        //igaze->waitMotionDone();
        //to track from now on
        igaze->setTrackingMode(true);
    }

    void run()
    {
      	Vector pupil; string hand; Vector gaze;
        int state; // state the human is in
        if (inPort.read(pupil))
        {
            if ( pupil(1) == 1){
                yInfo() << "Teammate's Tower";
                state = 1;
           
            }else if ( pupil(1) == 2) {
                yInfo() << "My Tower";
                state = 2;
                
            }else if ( pupil(1) == 3) {
                yInfo() << "Brick/Object";
                state = 3;
                
            }else if ( pupil(1) == 4) {
                yInfo() << "iCub's Face";
                state = 4;
                
            } else {
                yInfo() << "wrong state";
                state = -1;            
            }
        
            if (state != -1){
                // if you observe the human looking at one of the states then act
                actionBehavior( state);




            }
    
        } 
           
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

