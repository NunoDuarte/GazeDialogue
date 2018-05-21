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


    /***************************************************/
    void CtrlModule::fixate(const Vector &x)
    {
        igaze->setSaccadesMode(true);  // this gives problem with waitMotionDone in simulation
        igaze->lookAtFixationPointSync(x);
        double timeout = 10.0; 
        bool done = false; 
        done = igaze->waitMotionDone(0.1,timeout); 
        if(!done){
            yWarning("Something went wrong with the initial approach, using timeout");
            igaze->stopControl();
        }
    }

    /***************************************************/
    Vector CtrlModule::computeHandOrientation(const string &hand)
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
    Vector CtrlModule::computeHandOrientationRotate(bool giveObject)
    {
        // we have to provide a 4x1 vector representing the
        // final orientation for the specified hand, with
        // the palm pointing downward

        Matrix Rot(3,3);
        
        if (_hand == "right"){
            if (giveObject) {
                Rot(0,0)= 0.0; Rot(0,1)= 0.0; Rot(0,2)=  1.0;
                Rot(1,0)=-1.0; Rot(1,1)= 0.0; Rot(1,2)=  0.0;
                Rot(2,0)= 0.0; Rot(2,1)= -1.0; Rot(2,2)= 0.0;
            }
            else{
                Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
                Rot(1,0)= 0.0; Rot(1,1)= 0.0; Rot(1,2)= -1.0;
                Rot(2,0)= 0.0; Rot(2,1)=-1.0; Rot(2,2)= 0.0;
            }

        }else{
            yError() << "Wrong hand!";
        }

        return dcm2axis(Rot);
    }

    /***************************************************/
    Vector CtrlModule::computeHandOrientationPassing(const string &hand)
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
            Rot(0,0)=-1.0; Rot(0,1)= 0.0; Rot(0,2)= 0.0;
            Rot(1,0)= 0.0; Rot(1,1)= 0.0; Rot(1,2)= -1.0;
            Rot(2,0)= 0.0; Rot(2,1)=-1.0; Rot(2,2)= 0.0;
        }

        // add up a further slight rotation (30 deg) around -y:
        // this will prevent the thumb from hitting the table
        // create a rotation matrix
        

        return dcm2axis(Rot);
    }

    /***************************************************/
    void CtrlModule::approachTargetWithHand(const string &hand,
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
    void CtrlModule::initArm(const Vector &x, string robot)
    {
        string hand=(x[1]>=0.0?"right":"left");

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

        //double target[16] = {0, 90, 0, 20, 10, 0, 0, 50, 10, 0, 0, 0, 0, 0, 0, 0};   //icubSim
        double target[16] = {0, 90, 0, 20, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   // icub

        // we set up here the lists of joints we need to actuate
        // shoulders (3) + elbow + wrist (3) + hand openning + thumb (3)
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

    /***************************************************/
    void CtrlModule::changeDOFs()
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
    void CtrlModule::liftObject(const string &hand)
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
    void CtrlModule::moveFingers(const string &hand,
                     const VectorOf<int> &joints,
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

    /***************************************************/
    void CtrlModule::look_down()
    {
        // we ask the controller to keep the vergence
        // from now on fixed at 5.0 deg, which is the
        // configuration where we calibrated the stereo-vision;
        // without that, we cannot retrieve good 3D positions
        // with the real robot
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
    }

    /***************************************************/
    bool CtrlModule::grasp_it(const double fingers_closure)
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

        fixate(x);
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

        fixate(x);
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

