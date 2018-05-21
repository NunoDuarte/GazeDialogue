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

#include "helpers.h"
#include "extras/CvHMM.h"

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

    // we set up here the lists of joints we need to actuate
    VectorOf<int> abduction,thumb,fingers;

    // position and orientation of the robot's arm
    Vector p, o;
    // current linear and angular velocities
    Vector vcur, wcur;
    // 3d location in the 3d world reference frame
    Vector x, xf, xi;

    // Declare the probabilities of being giving or placing action
    cv::Mat act_probability;

    // calculte distances for giving action
    float Vmax;
    float epsilon;
    int count;
    Vector e, ed;
    Vector unit_e;
    double v_mag;
    double acc_mag;
    double mag_e; 

    // define matrixes for follower and leader
    CvHMM hmmFG;
    double *TRANSdataFG;
    cv::Mat TRANSFG;
    double *EMISdataFG;
    cv::Mat EMISFG;
    cv::Mat INITFG;

    CvHMM hmmFP;
    double *TRANSdataFP;
    cv::Mat TRANSFP;
    double *EMISdataFP;
    cv::Mat EMISFP;
    cv::Mat INITFP;

    // define the sequence of states
    cv::Mat seq_mat;

    // define the forward and backward probability matrix that are use to calculate the states
    cv::Mat forward;
    cv::Mat backward;

    // output from decode
    cv::Mat pstates;

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

        // initialize the act_probability
        yInfo() << "1";
        act_probability = cv::Mat(2,1, CV_64F);
        yInfo() << "1";
        act_probability.at<double>(0,0) = 0.5;
        act_probability.at<double>(1,0) = 0.5;

        /*------------------------ FOLOWER GIVING MODEL ------------------------*/
        TRANSdataFG = new double[16];        
        TRANSdataFG[0] = 0.968314321926489;
        TRANSdataFG[1] = 0.0209125475285171;
        TRANSdataFG[2] = 0.00697084917617237; 
        TRANSdataFG[3] = 0.00380228136882129;
        TRANSdataFG[4] = 0.00638297872340426;
        TRANSdataFG[5] = 0.979574468085106;
        TRANSdataFG[6] = 0.00297872340425532;
        TRANSdataFG[7] = 0.0110638297872340; 
        TRANSdataFG[8] = 0.0436507936507937; 
        TRANSdataFG[9] = 0.0198412698412698;
        TRANSdataFG[10] = 0.922619047619048;
        TRANSdataFG[11] = 0.0138888888888889;
        TRANSdataFG[12] = 0.00295159386068477;
        TRANSdataFG[13] = 0.00118063754427391;
        TRANSdataFG[14] = 0.00118063754427391;
        TRANSdataFG[15] = 0.994687131050767;

        TRANSFG = cv::Mat(4,4,CV_64F,TRANSdataFG).clone();  

        EMISdataFG = new double[24];
        EMISdataFG[0] = 0.699680511182109;
        EMISdataFG[1] = 0.0396166134185304;
        EMISdataFG[2] = 0.122683706070288;
        EMISdataFG[3] = 0.0217252396166134;
        EMISdataFG[4] = 0.0319488817891374;
        EMISdataFG[5] = 0.0843450479233227;
        EMISdataFG[6] = 0.302240512117055;
        EMISdataFG[7] = 0.186099679926840;
        EMISdataFG[8] = 0.362597165066301;  
        EMISdataFG[9] = 0.0214906264288980;
        EMISdataFG[10] = 0.0832190214906264;
        EMISdataFG[11] = 0.0443529949702789;
        EMISdataFG[12] = 0.619918699186992;
        EMISdataFG[13] = 0.0752032520325203;
        EMISdataFG[14] = 0.0894308943089431;
        EMISdataFG[15] = 0.0182926829268293;
        EMISdataFG[16] = 0.0406504065040650;
        EMISdataFG[17] = 0.156504065040650;
        EMISdataFG[18] = 0.181818181818182;
        EMISdataFG[19] = 0.183175033921303;
        EMISdataFG[20] = 0.244911804613297;
        EMISdataFG[21] = 0.0135685210312076;
        EMISdataFG[22] = 0.350746268656716;
        EMISdataFG[23] = 0.0257801899592944;

        EMISFG = cv::Mat(4,6,CV_64F,EMISdataFG).clone();

        double INITdataFG[] = {1.0, 0.0, 0.0, 0.0};
        INITFG = cv::Mat(1,6,CV_64F,INITdataFG).clone();

        std::cout << "FG:";
        hmmFG.printModel(TRANSFG,EMISFG,INITFG);

        /*------------------------ FOLOWER PLACING MODEL ------------------------*/
        TRANSdataFP = new double[16];
        TRANSdataFP[0] = 0.968184653774173;
        TRANSdataFP[1] = 0.0143480973175296;       
        TRANSdataFP[2] = 0.0162195882719900;       
        TRANSdataFP[3] = 0.00124766063630692;       
        TRANSdataFP[4] = 0.00898410504492053;       
        TRANSdataFP[5] = 0.979267449896337;       
        TRANSdataFP[6] = 0.0103662750518314;       
        TRANSdataFP[7] = 0.00138217000691085;       
        TRANSdataFP[8] = 0.0258843830888697;       
        TRANSdataFP[9] = 0.0120793787748059;       
        TRANSdataFP[10] = 0.958584987057809;       
        TRANSdataFP[11] = 0.00345125107851596;       
        TRANSdataFP[12] = 0.0179640718562874;       
        TRANSdataFP[13] = 0.00598802395209581;       
        TRANSdataFP[14] = 0.0419161676646707;       
        TRANSdataFP[15] = 0.934131736526946;       

        TRANSFP = cv::Mat(4,4,CV_64F,TRANSdataFP).clone();

        EMISdataFP = new double[24];        
        EMISdataFP[0] = 0.754639175257732;
        EMISdataFP[1] = 0.0130584192439863;
        EMISdataFP[2] = 0.0192439862542955;
        EMISdataFP[3] = 0.0219931271477663;
        EMISdataFP[4] = 0.0371134020618557;
        EMISdataFP[5] = 0.153951890034364;
        EMISdataFP[6] = 0.427316293929713;
        EMISdataFP[7] = 0.0479233226837061;
        EMISdataFP[8] = 0.00319488817891374;
        EMISdataFP[9] = 0.0359424920127796;
        EMISdataFP[10] = 0.159744408945687;
        EMISdataFP[11] = 0.325878594249201;
        EMISdataFP[12] = 0.557013118062563,
        EMISdataFP[13] = 0.0575176589303734;
        EMISdataFP[14] = 0.0171543895055499;
        EMISdataFP[15] = 0.0181634712411705;
        EMISdataFP[16] = 0.0171543895055499;
        EMISdataFP[17] = 0.332996972754793;
        EMISdataFP[18] = 0.725000000000000;
        EMISdataFP[19] = 0.00312500000000000;
        EMISdataFP[20] = 0.153125000000000;
        EMISdataFP[21] = 0.00625000000000000;
        EMISdataFP[22] = 0.0562500000000000;
        EMISdataFP[23] = 0.0562500000000000;

        EMISFP = cv::Mat(4,6,CV_64F,EMISdataFP).clone();

        double INITdataFP[] = {1.0, 0.0, 0.0, 0.0};
        INITFP = cv::Mat(1,6,CV_64F,INITdataFP).clone();

        std::cout << "FP:";
        hmmFP.printModel(TRANSFP,EMISFP,INITFP);

        // get arm in initial position
        x[1] =  -0.5; // to the left
        startingArm(x);
    
        // define the fingers joints
        abduction.push_back(7);
        thumb.push_back(8);
        for (int i=9; i<16; i++)
            fingers.push_back(i);

        getchar();

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

    /***************************************************/
    Vector computeHandOrientationPassing(const string &hand)
    {
        // we have to provide a 4x1 vector representing the
        // final orientation for the specified hand

        Matrix Rot(3,3);

        if (hand == "right"){
            Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 0.0; Rot(1,2)= -1.0;
            Rot(2,0)= 0.0; Rot(2,1)= -1.0; Rot(2,2)= 0.0;

        }
        if (hand == "left"){
            Rot(0,0)= -1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 0.0; Rot(1,2)= -1.0;
            Rot(2,0)= 0.0; Rot(2,1)=-1.0; Rot(2,2)= 0.0;
        }

        // add up a further slight rotation (30 deg) around -y:
        // this will prevent the thumb from hitting the table
        // create a rotation matrix
        

        return dcm2axis(Rot);
    }

    float magnitude(Vector x)     //  <! Vector magnitude
    {
        return sqrt((x[0]*x[0])+(x[1]*x[1])+(x[2]*x[2]));
    }

    /***************************************************/
    void fixate(int maxState)
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

    void threadRelease()
    {
        printf("ControlThread:stopping the robot\n");

        iarm->stopControl();

        igaze->restoreContext(startup_ctxt_gaze);
        igaze->stopControl();
        drvGaze.close();
        port.close();
        
        printf("Done, goodbye from ControlThread\n");
    }

    /***************************************************/
    void startingArm(const Vector &x)
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
    void moveFingers(const string &hand, const VectorOf<int> &joints,
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


    int predictAL(cv::Mat& act_prob, int cur_state, int cur_action){
        /***********************************************************************************
        act_prob is a 2x1 matrix with two probabilities, 
            <0,0> is probability for giving, and 
            <1,0> i probability for placing action. They sum to 1.
        This value (as argument) containes the current probabilities 
            for the two actions that are updated within the function.

        cur_state is 0-5 integer representing the loeaders focuse of attention. 
            0 - brick 
            1 - TM face 
            2 - TM hand 
            3 - Own hand 
            4 - TM tower 
            5 - Own tower

        cur_action //0-giving 1-placing -1-uncertain

        internal parameters of the fucntion

        APdataL - action prediction matrix that is obtained from the data, 
            it containtes probabilities for each of the action with respect 
            to the focus of attention of the LEADER
        rows are foc of leader, 
        columns are giving and placing probabilities
        treshold - is a parameter defining the required difference 
            between the two elements in act_prob to classify the action 
            as 0-giving 1-placing -1-uncertain
        alfa - a parameter for moving average filtering a = (1-alfa)*a+alfa*new_val;
        *************************************************************************************/
        double APdataL[] = {0.495420313454101, 0.504579686545899,
                          0.840179238237491, 0.159820761762509,
                          0.931922196796339, 0.0680778032036613,
                          0.520000000000000, 0.480000000000000,
                          0.748014440433213, 0.251985559566787,
                          0.186659772492244, 0.813340227507756};
        cv::Mat APL = cv::Mat(6,2,CV_64F,APdataL).clone();

        int new_action;//0-giving 1-placing -1-uncertain

        double rn = rand();

        double prob_G = 0.5;
        double prob_P = 0.5;
        double deltalgc = 0.0;
        double deltalpc = 0.0;
        double treshold = 0.15;
        double alfa = 0.01;//parameter of exponential moving average

        if(cur_state>=0 && cur_state<6)
        {
            prob_G = APL.at<double>(cur_state,0);
            prob_P = APL.at<double>(cur_state,1);
        }


        if(rn<prob_G)//it is GIVING
        {
            deltalgc = prob_G - act_prob.at<double>(0,0);
            deltalpc = -deltalgc;
        }
        else//it is placing
        {
            deltalpc = prob_P - act_prob.at<double>(1,0);
            deltalgc = -deltalpc;
        }

        deltalgc = act_prob.at<double>(0,0)+deltalgc;
        deltalpc = act_prob.at<double>(1,0)+deltalpc;

        //expoential moving average
        act_prob.at<double>(0,0)=(1-alfa)*act_prob.at<double>(0,0) + alfa*deltalgc;
        act_prob.at<double>(1,0)=(1-alfa)*act_prob.at<double>(1,0) + alfa*deltalpc;

        //act_prob.at<double>(0,0)=act_prob.at<double>(0,0)+0.1;
        //act_prob.at<double>(1,0)=act_prob.at<double>(1,0)-0.1;

        if((act_prob.at<double>(0,0)-act_prob.at<double>(1,0))>treshold)
            new_action = 0;//
        else if((act_prob.at<double>(0,0)-act_prob.at<double>(1,0))<-treshold)
            new_action = 1;
        else
            new_action = -1;

        return new_action;
    }

    /***************************************************/
    void actionBehavior(int state)
    {
        // input: state of the human
        //        logpseg -- irrelevant for now
        // output: behavior of robot
        int action;
        double logpseq;

        // call the predictor function
        action = predictAL(act_probability, state, action);
        //yInfo() << act_probability.at<double>(0,0);
        //yInfo() << act_probability.at<double>(1,0);

        // add state to sequence of states
        seq_mat.push_back(state);

        // make a decision based on the predictor's response
        if (action == 0){
            // get current location
            iarm->getPose(p,o);
            iarm->setPosePriority("orientation");
            Vector od = o;
            // get current velocities
            iarm->getTaskVelocities(vcur, wcur);

            od = computeHandOrientationPassing(_hand); //get default orientation
            reachArmGiving(p, od, xf, vcur);

            hmmFG.decodeMR2(seq_mat,TRANSFG,EMISFG,INITFG,logpseq,pstates,forward,backward);
            gazeBehavior(pstates);

        } else if (action == 1) {
            // just observe the action
            yInfo() << "I'm observing";

            iarm->getPose(p,o);
            // get current velocities
            iarm->getTaskVelocities(vcur, wcur);

            //closing back the hand
            moveFingers(_hand,thumb,1.0);
            moveFingers(_hand,fingers,0.5);
            // go back to start position
            x[1] =  -0.5; // to the left
            startingArm(x);

            hmmFP.decodeMR2(seq_mat,TRANSFP,EMISFP,INITFP,logpseq,pstates,forward,backward);
            gazeBehavior(pstates);

            count++; // count the number of times is giving
            // alternative is getting the percentage of giving and when it is higher than
            // 70% you can close the hand

        } else {
            yInfo() << "Wrong action";
            //cout << "M = "<< endl << " "  << seq_mat << endl << endl;
            hmmFP.decodeMR2(seq_mat,TRANSFP,EMISFP,INITFP,logpseq,pstates,forward,backward);
            //cout << "M = "<< endl << " "  << pstates << endl << endl;
            //getchar();
            gazeBehavior(pstates);
        }
    }

    void reachArmGiving(Vector desired_p, Vector orientation, Vector x_pos, Vector velocity)
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
    void gazeBehavior(cv::Mat &state)
    {
        // check all the state and take the highest probability one
        double max = 0;   
        int id = -1;

        int nRows = state.rows;
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
        }
        // sending the highest probable state to the robot
        fixate(id+1);
    }

    void run()
    {
      	Vector pupil; string hand; Vector gaze;
        int state; // state the human is in
        if (inPort.read(pupil))
        {
            if ( pupil(1) == 1){
                yInfo() << "Teammate's Tower";
                state = 5;
           
            }else if ( pupil(1) == 2) {
                yInfo() << "My Tower";
                state = 6;
                
            }else if ( pupil(1) == 3) {
                yInfo() << "Brick/Object";
                state = 1;
  
            }else if ( pupil(1) == 4) {
                yInfo() << "iCub's Face";
                state = 2;

            }else if ( pupil(1) == 5) {
                yInfo() << "iCub's Hand";
                state = 3;

            }else if ( pupil(1) == 4) {
                yInfo() << "Own's Hand";
                state = 4;
                
            } else {
                yInfo() << "wrong state";
                state = -1;            
            }
        
            if (state != -1){
                // if you observe the human looking at one of the states then act
                actionBehavior(state);
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
        if ((Time::now()-startTime)>10)
            done=true;
    }
    
    myThread.stop();

    return 0;
}

