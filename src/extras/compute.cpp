// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//

#include <cmath>
#include <algorithm>

#include <yarp/sig/Vector.h>
#include <yarp/os/all.h>

#include <yarp/math/Math.h>

#include "compute.h"

/***************************************************/

using namespace yarp::math;

/***************************************************/

    float ControlThread::magnitude(Vector x)     //  <! Vector magnitude
    {
        return sqrt((x[0]*x[0])+(x[1]*x[1])+(x[2]*x[2]));
    }


    /***************************************************/
    void ControlThread::fixatePoint(const Vector &x)
    {
        igaze->setSaccadesMode(true);  // this gives problem with waitMotionDone in simulation
        igaze->lookAtFixationPointSync(x);
        /*double timeout = 10.0; 
        bool done = false; 
        done = igaze->waitMotionDone(0.1,timeout); 
        if(!done){
            yWarning("Something went wrong with the initial approach, using timeout");
            igaze->stopControl();
        }*/
    }

    /***************************************************/
    Vector ControlThread::computeHandOrientation(const string &hand)
    {
        // we have to provide a 4x1 vector representing the
        // final orientation for the specified hand, with
        // the palm pointing downward

        Matrix Rot(3,3);
        // 30 degress of rotation
        Vector rotation(4);
        rotation[0] = 0.0;
        rotation[1] = -1.0;
        rotation[2] = 0.0;
        rotation[3] = 30.0*(M_PI/180.0);
        Matrix rotCor = axis2dcm(rotation);
        Matrix rotCorAdj = rotCor.submatrix(0,2,0,2);
        if (hand == "right"){
            Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 1.0; Rot(1,2)= 0.0;
            Rot(2,0)= 0.0; Rot(2,1)= 0.0; Rot(2,2)= -1.0;

        }
        if (hand == "left"){
            Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)=-1.0; Rot(1,2)= 0.0;
            Rot(2,0)= 0.0; Rot(2,1)= 0.0; Rot(2,2)= 1.0;
        }

        // add up a further slight rotation (30 deg) around -y:
        // this will prevent the thumb from hitting the table
        // create a rotation matrix
        

        return dcm2axis(Rot*rotCorAdj);
    }

    /***************************************************/
    Vector ControlThread::computeHandOrientationPassing(const string &hand)
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

    /***************************************************/
    void ControlThread::approachTargetWithHand(const string &hand,
                                const Vector &x,
                                const Vector &o, bool wait=true)
    {
        // select the correct interface
        if (hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        // enable all dofs but the roll of the torso
        Vector curDOF;
        iarm->getDOF(curDOF);
        
        Vector newDOF(10);
        newDOF[0] = 1;   // pitch
        newDOF[1] = 0;   // roll
        newDOF[2] = 1;   // yaw
        newDOF[3] = 1;
        newDOF[4] = 1;
        newDOF[5] = 1;
        newDOF[6] = 1;
        newDOF[7] = 1;
        newDOF[8] = 1;
        newDOF[9] = 1;
        iarm->setDOF(newDOF,curDOF);
	    if (_hand == "right"){
            iarm->setLimits(0.0,-50.0, 30.0);
            iarm->setLimits(2.0, -10.0, 10.0);
        }else {
            iarm->setLimits(0.0,-30.0, 50.0);
            iarm->setLimits(2.0, -10.0, 10.0);
        }

        // reach the first via-point
        // located 5 cm above the target x
        Vector approach=x;
        approach[2]+= 0.02; //5 cm
        //approach[1]+= 0.02; //2 cm to +z
        iarm->goToPoseSync(approach, o);

        double timeout = 10.0;
        bool done = false;
        if (wait){
            done = iarm->waitMotionDone(0.1,timeout); 
            if(!done){
                yWarning("Something went wrong with the initial approach, using timeout");
                iarm->stopControl();
            }
        }
        Vector xa, oa;
        iarm->getPose(xa,oa);

        yInfo() << "Position desired: "     << x.toString();
        yInfo() << "Position achieved: "    << xa.toString();
        yInfo() << "Orientation desired"    << o.toString();
        yInfo() << "Orientation achieved: " << oa.toString();
        // reach the final target x;
        approach[2]-= 0.02;
        iarm->goToPoseSync(approach, o);
        if (wait){
            timeout = 10.0; 
            done = false; 
            done = iarm->waitMotionDone(0.1,timeout); 
            if(!done){
                yWarning("Something went wrong with the initial approach, using timeout");
                iarm->stopControl();
            }
        }
    }

    /***************************************************/
    void ControlThread::changeDOFs()
    {
        // select the correct interface
        if (_hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        Vector curDOF;
        iarm->getDOF(curDOF);
        yInfo() << "DOF" << curDOF.toString().c_str();
        
        Vector newDOF(10);
        newDOF[0] = 0;
        newDOF[1] = 0;
        newDOF[2] = 1;
        newDOF[3] = 1;
        newDOF[4] = 1;
        newDOF[5] = 1;
        newDOF[6] = 1;
        newDOF[7] = 1;
        newDOF[8] = 1;
        newDOF[9] = 1;
        iarm->setDOF(newDOF,curDOF);
    }

    /***************************************************/
    void ControlThread::liftObject(const string &hand)
    {
        // select the correct interface
        if (hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        // just lift the hand of few centimeters
        // wrt the current position

        Vector x, o;
        iarm->getPose(x,o);

        Vector approach = x;
        approach[2]+= 0.1; //lift 10 cm

        iarm->goToPoseSync(approach, o);
        double timeout = 10.0; 
        bool done = false; 
        done = iarm->waitMotionDone(0.1,timeout); 
        if(!done){
            yWarning("Something went wrong with the initial approach, using timeout");
            iarm->stopControl();
        }
    }

    /***************************************************/
    void ControlThread::look_down()
    {
        // we ask the controller to keep the vergence
        // from now on fixed at 5.0 deg, which is the
        // configuration where we calibrated the stereo-vision;
        // without that, we cannot retrieve good 3D positions
        // with the real robot
        Vector ang(3,0.0);
        ang[1]=-70.0;
        //ang[2]=-20;
        igaze->lookAtAbsAngles(ang);

        double timeout = 10.0; 
        bool done = false; 
        done = igaze->waitMotionDone(0.1,timeout); 
        if(!done){
            yWarning("Something went wrong with the initial approach, using timeout");
            igaze->stopControl();
        }
    }

    /***************************************************/
    bool ControlThread::grasp_it(const double fingers_closure)
    {
        Vector x;
        if (object.getLocation(x))
        {
            yInfo()<<"retrieved 3D location = ("<<x.toString(3,3)<<")";

            // we select the hand accordingly
            //_hand=(x[1]>=0.0?"right":"left");
            _hand="left";
            yInfo()<<"selected hand = \""<<_hand<<'\"';
        }
        else
            return false;

        fixatePoint(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        Vector o=computeHandOrientationPassing(_hand);
        yInfo()<<"computed orientation = ("<<o.toString(3,3)<<")";

        // we set up here the lists of joints we need to actuate
        VectorOf<int> abduction,thumb,fingers;
        abduction.push_back(7);
        thumb.push_back(8);
        for (int i=9; i<16; i++)
            fingers.push_back(i);

        // let's put the hand in the pre-grasp configuration
        //moveFingers(_hand,abduction,0.8);
        moveFingers(_hand,fingers,0.0);
        yInfo()<<"prepared hand";

        fixatePoint(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        approachTargetWithHand(_hand,x,o);
        yInfo()<<"approached object";

        moveFingers(_hand,thumb,1.0); //move the thumb only after approachTargetWithHand
        moveFingers(_hand,fingers,fingers_closure);
        yInfo()<<"I have grasped the object";

        //moveFingers(hand,fingers,0.0);
        //yInfo()<<"released";

        return true;

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

                        Vector x, o;
                        iarm->getPose(x,o); //get current position of hand
                        yInfo()<<"fixating the Brick";

                        Vector look = x;
                        look[0] =  -0.20;
                        look[1] =  -0.10;
                        look[2] =  -0.02;         

                        igaze->lookAtFixationPoint(look);
                        //igaze->waitMotionDone();
                        //to track from now on
                        igaze->setTrackingMode(true);
                        break;
                     }
            case 2 : {
                        cout << '2' << endl; 

                        Vector x, o;
                        iarm->getPose(x,o); //get current position of hand
                        yInfo()<<"fixating the Human's face";

                        Vector look = x;
		                look[0] = -0.35;
		                look[1] =  0.0;
		                look[2] =  0.40;     

                        igaze->lookAtFixationPoint(look);
                        break;
                     }
            case 3 : {
                        cout << '3' << endl; 

                        // look up if you haven't already
                        /*Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-40.0;
                            igaze->lookAtAbsAngles(ang);
                        }*/

                        Vector x, o;
                        iarm->getPose(x,o); //get current position of hand
                        yInfo()<<"fixating the Human's hand";

                        Vector look = x;
		                look[0] = -0.35;
		                look[1] =  0.10;
		                look[2] =  0.20;    

                        igaze->lookAtFixationPoint(look);
                        igaze->setTrackingMode(true);
                        break;
                     }
            case 4 : {
                        cout << '4' << endl; 

                        // look up if you haven't already
                        /*Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-40.0;
                            igaze->lookAtAbsAngles(ang);
                        }*/

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
                        /*Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-70.0;
                            igaze->lookAtAbsAngles(ang);
                        }*/

                        Vector x, o;
                        iarm->getPose(x,o); //get current position of hand
                        yInfo()<<"fixating the Teammates Tower";

                        Vector look = x;
                        look[0] = -0.45;
                        look[1] =  0.00;
                        look[2] = -0.05;         

                        igaze->lookAtFixationPoint(look);
                        break;
                     }
            case 6 : {
                        cout << '6' << endl; 

                        // make iCub look down
                        Vector ang(3,0.0);
                        igaze->getAngles(ang);
                        if (ang[1] > -30){
                            ang[1]=-70.0;
                            igaze->lookAtAbsAngles(ang);
                        }

                        Vector x, o;
                        iarm->getPose(x,o); //get current position of hand
                        yInfo()<<"fixating My Tower";

                        Vector look = x;
                        look[0] = -0.20;
                        look[1] =  0.00;
                        look[2] = -0.05;         

                        igaze->lookAtFixationPoint(look);
                        //igaze->waitMotionDone();
                        //to track from now on
                        //igaze->setTrackingMode(true);
                        break;
                     }
        }
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
                        if (not grasp) {
                            // grasp the ball
                            Bottle reply;
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
                                yInfo() << "Yeah! I did it! Maybe...";
                                grasp = true;
                                getchar();
                            }
                            else
                            {
                                reply.addString("nack");
                                reply.addString("I don't see any object!");
                            }   

                        }
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
                        //reachArmGiving(p, od, xf);

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
                        //reachArmGiving(p, od, xf);

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
                        //reachArmGiving(p, od, xf);
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
    }

    /***************************************************/
    void ControlThread::moveFingers(const string &hand, const VectorOf<int> &joints,
                    const double fingers_closure)
    {
        // select the correct interface

        // this should be in the beginning, where you initialize the necessary stuff
        IControlLimits   *ilim;
        IPositionControl *ipos;
        IEncoders         *ienc;
        IControlMode     *imod;

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
            //yInfo()<<"Yielding new target: "<<target<<" [deg]";
            ipos->positionMove(j,target);
        }
        // wait (with timeout) until the movement is completed
        bool done=false;
        double t0=Time::now();
        while (!done && (Time::now()-t0 < 5.0))
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

    void ControlThread::reachArmGiving(Vector desired_p, Vector orientation, Vector x_pos, int counter)
    {
        e[0] = x_pos[0] - desired_p[0];
        e[1] = x_pos[1] - desired_p[1];
        e[2] = x_pos[2] - desired_p[2]; 
        //yInfo() << "e[0]:" << e[0] << "e[1]" << e[1] << "e[2]" << e[2];       

        
        iarm->setTrajTime(10-0.01*counter);
        iarm->goToPose(x_pos,orientation);
    }

    /***************************************************/
    void ControlThread::release(string hand)
    {
        IControlLimits   *ilim;
        IPositionControl *ipos;
        IControlMode     *imod;

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


