// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//

#include <cmath>
#include <algorithm>

#include <yarp/sig/Vector.h>
#include <yarp/os/all.h>

#include <yarp/math/Math.h>

#include "configure.h"

/***************************************************/

using namespace yarp::math;

    /***************************************************/
    bool ControlThread::threadInit()
    {
        //initialize here variables
        printf("ControlThread:starting\n");

        seq = cv::Mat::zeros(cv::Size( 100000,1), CV_64FC1);
        seq_mat_wTime = cv::Mat::zeros(cv::Size( 2,10000), CV_64FC1);
        seq=seq-1;
        cnt=0;
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
        optJoint.put("remote","/"+ robot +"/left_arm");
        optJoint.put("local","/position/left_arm");

        if (!drvHandL.open(optJoint))
        {
            yError()<<"Unable to connect to /icubSim/left_arm";
            return false;
        }

        // open a client interface to connect to the joint controller
/*        Property optJoint1;
        optJoint1.put("device","remote_controlboard");
        optJoint1.put("remote","/"+ robot +"/right_arm");
        optJoint1.put("local","/position/right_arm");

        if (!drvHandR.open(optJoint1))
        {
            yError()<<"Unable to connect to /icubSim/right_arm";
            return false;
        }
*/
        yInfo()<<"Begin moving arms to first initial position";
        // GET ARMS IN THE CORRECT POSITION
        x.resize(3);
//        x[1] =  0.5; // to the right
//        initArm(x);
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
        //x[1] =  -0.5; // to the left
        //startingArm(x);
    
        // define the fingers joints
        abduction.push_back(7);
        thumb.push_back(8);
        for (int i=9; i<16; i++)
            fingers.push_back(i);

        // first, look down  
        look_down();

        // initialize the act_probability
        act_probability = cv::Mat(2,1, CV_64F);
        act_probability.at<double>(0,0) = 0.5;
        act_probability.at<double>(1,0) = 0.5;

        /*------------------------ LEADER GIVING MODEL ------------------------*/
        TRANSdataLGbhon = new double[36];        
        TRANSdataLGbhon[0] = 0.986088379705401;
        TRANSdataLGbhon[1] = 0.001636661211129;
        TRANSdataLGbhon[2] = 0.004500818330606;
        TRANSdataLGbhon[3] = 0.001636661211129;
        TRANSdataLGbhon[4] = 0.004091653027823;
        TRANSdataLGbhon[5] = 0.002045826513912;
        TRANSdataLGbhon[6] = 0.0; //0.003809523809524; // don't go back to brick
        TRANSdataLGbhon[7] = 0.950476190476191;
        TRANSdataLGbhon[8] = 0.043809523809524;
        TRANSdataLGbhon[9] = 0.001904761904762;
        TRANSdataLGbhon[10] = 0.0;
        TRANSdataLGbhon[11] = 0.0;
        TRANSdataLGbhon[12] = 0.0; //0.001762114537445; // don't go back to brick
        TRANSdataLGbhon[13] = 0.021145374449339;
        TRANSdataLGbhon[14] = 0.971806167400881;
        TRANSdataLGbhon[15] = 8.810572687224670e-04;
        TRANSdataLGbhon[16] = 0.004405286343612;
        TRANSdataLGbhon[17] = 0.0;
        TRANSdataLGbhon[18] = 0.0; // don't go back to brick
        TRANSdataLGbhon[19] = 0.0;
        TRANSdataLGbhon[20] = 0.057142857142857;
        TRANSdataLGbhon[21] = 0.933333333333333;
        TRANSdataLGbhon[22] = 0.009523809523810;
        TRANSdataLGbhon[23] = 0.0;
        TRANSdataLGbhon[24] = 0.0; //0.007246376811594; // don't go back to brick
        TRANSdataLGbhon[25] = 0.014492753623188;
        TRANSdataLGbhon[26] = 0.043478260869565;
        TRANSdataLGbhon[27] = 0.003623188405797;
        TRANSdataLGbhon[28] = 0.923913043478261;
        TRANSdataLGbhon[29] = 0.007246376811594;
        TRANSdataLGbhon[30] = 0.0; //0.056603773584906; // don't go back to brick
        TRANSdataLGbhon[31] = 0.003144654088050;
        TRANSdataLGbhon[32] = 0.003144654088050;
        TRANSdataLGbhon[33] = 0.012578616352201;
        TRANSdataLGbhon[34] = 0.0;
        TRANSdataLGbhon[35] = 0.924528301886793;
        TRANSLGbhon = cv::Mat(6,6,CV_64F,TRANSdataLGbhon).clone();  

        TRANSdataLGahon = new double[36];
        TRANSdataLGahon[0] = 0.0;
        TRANSdataLGahon[1] = 0.0;
        TRANSdataLGahon[2] = 0.0;
        TRANSdataLGahon[3] = 0.0;
        TRANSdataLGahon[4] = 0.0;
        TRANSdataLGahon[5] = 0.0;
        TRANSdataLGahon[6] = 0.0;
        TRANSdataLGahon[7] = 0.962328767123288;
        TRANSdataLGahon[8] = 0.020547945205479;
        TRANSdataLGahon[9] = 0.0;
        TRANSdataLGahon[10] = 0.015410958904110;
        TRANSdataLGahon[11] = 0.001712328767123;
        TRANSdataLGahon[12] = 0.0;
        TRANSdataLGahon[13] = 0.030927835051546;
        TRANSdataLGahon[14] = 0.942268041237113;
        TRANSdataLGahon[15] = 0.0;
        TRANSdataLGahon[16] = 0.024742268041237;
        TRANSdataLGahon[17] = 0.002061855670103;
        TRANSdataLGahon[18] = 0.0;
        TRANSdataLGahon[19] = 0.0;
        TRANSdataLGahon[20] = 0.0;
        TRANSdataLGahon[21] = 0.833333333333333;
        TRANSdataLGahon[22] = 0.166666666666667;
        TRANSdataLGahon[23] = 0.0;
        TRANSdataLGahon[24] = 0.0;
        TRANSdataLGahon[25] = 0.019607843137255;
        TRANSdataLGahon[26] = 0.003921568627451;
        TRANSdataLGahon[27] = 0.0;
        TRANSdataLGahon[28] = 0.971241830065360;
        TRANSdataLGahon[29] = 0.005228758169935;
        TRANSdataLGahon[30] = 0.0;
        TRANSdataLGahon[31] = 0.016949152542373;
        TRANSdataLGahon[32] = 0.016949152542373;
        TRANSdataLGahon[33] = 0.050847457627119;
        TRANSdataLGahon[34] = 0.0;
        TRANSdataLGahon[35] = 0.915254237288136;
        TRANSLGahon = cv::Mat(6,6,CV_64F,TRANSdataLGahon).clone();

        double INITdataLG[] = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        INITLG = cv::Mat(1,6,CV_64F,INITdataLG).clone();

        std::cout << "LG:";
        mcLG.printModel(TRANSLGbhon,TRANSLGahon,INITLG);

        /*------------------------ LEADER PLACING MODEL ------------------------*/

        getchar();

        // initialize grasping and releasing counter
        grasp = false;
        released = false;

        time(&timer1);           // get current time

        myfile.open ("log.txt");
        myfile << "[";
        myfile2.open ("log2.txt");

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

        double target[] = {-17, 20, 8, 41, 7, -21, -9, 48, 0, 0, 0, 0, 0, 0, 0, 0};
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

    void ControlThread::threadRelease()
    {
        printf("ControlThread:stopping the robot\n");

        myfile << "];";
        myfile.close();

        iarm->stopControl();

        igaze->restoreContext(startup_ctxt_gaze);
        igaze->stopControl();
        drvGaze.close();
        port.close();
        rpcPort.close();
        
        printf("Done, goodbye from ControlThread\n");
    }

