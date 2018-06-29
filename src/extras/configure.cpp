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

        seq = cv::Mat::zeros(cv::Size( 1000,1), CV_64FC1);
        seq=seq-1;

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
        TRANSdataLGbhon = new double[16];        
        TRANSdataLGbhon[0] = 0.969052224371373;
        TRANSdataLGbhon[1] = 0.021276595744681;
        TRANSdataLGbhon[2] = 0.007092198581560; 
        TRANSdataLGbhon[3] = 0.001934235976789;
        TRANSdataLGbhon[4] = 0.005853219270599;
        TRANSdataLGbhon[5] = 0.986492570914003;
        TRANSdataLGbhon[6] = 0.003151733453399;
        TRANSdataLGbhon[7] = 0.003151733453399; 
        TRANSdataLGbhon[8] = 0.044585987261146; 
        TRANSdataLGbhon[9] = 0.021231422505308;
        TRANSdataLGbhon[10] = 0.923566878980892;
        TRANSdataLGbhon[11] = 0.010615711252654;
        TRANSdataLGbhon[12] = 0.008968609865471;
        TRANSdataLGbhon[13] = 0.004484304932735;
        TRANSdataLGbhon[14] = 0.002242152466368;
        TRANSdataLGbhon[15] = 0.966367713004484;
        TRANSLGbhon = cv::Mat(4,4,CV_64F,TRANSdataLGbhon).clone();  

        TRANSdataLGahon = new double[16];
        TRANSdataLGahon[0] = 0.833333333333333;
        TRANSdataLGahon[1] = 0.0;       
        TRANSdataLGahon[2] = 0.0;       
        TRANSdataLGahon[3] = 0.100000000000000;       
        TRANSdataLGahon[4] = 0.015151515151515;       
        TRANSdataLGahon[5] = 0.840909090909091;       
        TRANSdataLGahon[6] = 0.0;       
        TRANSdataLGahon[7] = 0.143939393939394;       
        TRANSdataLGahon[8] = 0.030303030303030;       
        TRANSdataLGahon[9] = 0.0;       
        TRANSdataLGahon[10] = 0.909090909090909;       
        TRANSdataLGahon[11] = 0.060606060606061;       
        TRANSdataLGahon[12] = 7.936507936507937e-04;       
        TRANSdataLGahon[13] = 0.0;       
        TRANSdataLGahon[14] = 7.936507936507937e-04;       
        TRANSdataLGahon[15] = 0.995238095238095;      
        TRANSLGahon = cv::Mat(4,4,CV_64F,TRANSdataLGahon).clone();

        double INITdataLG[] = {1.0, 0.0, 0.0, 0.0};
        INITLG = cv::Mat(1,4,CV_64F,INITdataLG).clone();

        std::cout << "LG:";
        mcLG.printModel(TRANSLGbhon,TRANSLGahon,INITLG);

        /*------------------------ LEADER PLACING MODEL ------------------------*/

        getchar();

        // initialize grasping and releasing counter
        grasp = false;
        released = false;

        // Select Action
        // giving  - 0
        // placing - 1
        int num = 0; 

        string Result; 
        std::ostringstream Result_string;
        Result_string << num;
        Result = Result_string.str();
        bool convert;

        //Eyes = loadDataFile("gazeBehavior_" + Result + ".txt", convert = false);  

/*        if (act.getAction(action))
        {
            yInfo()<<" retrieved Action = ("<<action <<")";
        }
        else 
            action = -1;     
*/

        myfile.open ("log.txt");
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

        iarm->stopControl();

        igaze->restoreContext(startup_ctxt_gaze);
        igaze->stopControl();
        drvGaze.close();
        port.close();
        rpcPort.close();
        
        printf("Done, goodbye from ControlThread\n");
    }

