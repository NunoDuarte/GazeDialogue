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
    bool CtrlModule::configure(ResourceFinder &rf)
    {
        string robot=rf.check("robot",Value("icubSim")).asString();

        /*if (!openCartesian(robot,"right_arm"))
            return false;
*/
        if (!openCartesian(robot,"left_arm"))
        {
            drvArmR.close();
            return false;
        }

        Property optGaze;
        optGaze.put("device","gazecontrollerclient");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");

        if (!drvGaze.open(optGaze)){
            yError() << "Unable to open the Gaze controller";
            return false;
        }

        // open a client interface to connect to the joint controller
        Property optJoint;
        optJoint.put("device","remote_controlboard");
        optJoint.put("remote","/"+ robot +"/left_arm");
        optJoint.put("local","/position/left_arm");

        if (!drvHandL.open(optJoint))
        {
            yError()<<"Unable to connect to /"+ robot +"left_arm";
            return false;
        }

        // open a client interface to connect to the joint controller
        /*Property optJoint1;
        optJoint1.put("device","remote_controlboard");
        optJoint1.put("remote","/"+ robot +"/right_arm");
        optJoint1.put("local","/position/right_arm");

        if (!drvHandR.open(optJoint1))
        {
            yError()<<"Unable to connect to /"+ robot +"right_arm";
            return false;
        } */    

        // save startup contexts
        /*drvArmR.view(iarm);
        drvArmR.view(ipos); //added this

        iarm->storeContext(&startup_ctxt_arm_right);
*/
        drvArmL.view(iarm);
        drvArmL.view(ipos); //added this

        iarm->storeContext(&startup_ctxt_arm_left);
        iarm->setPosePriority("position");

        drvGaze.view(igaze);
        igaze->storeContext(&startup_ctxt_gaze);

        rpcPort.open("/service");
        attach(rpcPort);

        yInfo()<<"Begin moving arms to first initial position";
        // GET ARMS IN THE CORRECT POSITION
        Vector x(3);

        //x[1] =  0.5; // to the right
        //initArm(x, robot);
        x[1] =  -0.5; // to the left
        initArm(x, robot);

        yInfo()<<"Finished moving the arms to initial position.";

        // Only when the arms are in place do I generate the ball
        Network yarp;
        CtrlModule mod;

        RpcClient objectLocation;
        objectLocation.open("/objectBall");
        
        printf("Trying to connect to %s\n", "/icubSim/world");
        yarp.connect("/objectBall","/icubSim/world");
        Bottle reply;

        Bottle cmd2;
        // CREATE the sphere affected by gravity we will use
        cmd2.addString("world");
        cmd2.addString("mk"); 
        cmd2.addString("sph");
        cmd2.addDouble(0.04); // radius 4 cm (made the ball smaller)
        // ball's position
        cmd2.addDouble(-0.05);
        cmd2.addDouble(0.69);
        cmd2.addDouble(0.2);
        // ball's colour
        cmd2.addDouble(0);
        cmd2.addDouble(0);
        cmd2.addDouble(1);

        printf("Sending message... %s\n", cmd2.toString().c_str());
        objectLocation.write(cmd2,reply);
        printf("Got response: %s\n", reply.toString().c_str());

        objectLocation.close();

        return true;
    }

    /***************************************************/
    bool CtrlModule::openCartesian(const string &robot, const string &arm)
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
                //yInfo()<<"Openned";
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
    bool CtrlModule::interruptModule()
    {
        return true;
    }

    /***************************************************/
    bool CtrlModule::close()
    {
        //drvArmR.view(iarm);
        //iarm->restoreContext(startup_ctxt_arm_right);

        drvArmL.view(iarm);
        iarm->restoreContext(startup_ctxt_arm_left);

        igaze->restoreContext(startup_ctxt_gaze);

        //drvArmR.close();
        drvArmL.close();
        drvGaze.close();
        //drvHandR.close();
        drvHandL.close();
        rpcPort.close();

        return true;
    }

    /***************************************************/
    bool CtrlModule::respond(const Bottle &command, Bottle &reply)
    {
        string cmd=command.get(0).asString();
        if (cmd=="help")
        {
            reply.addVocab(Vocab::encode("many"));
            reply.addString("Available commands:");
            reply.addString("- look_down");
            reply.addString("- pass");
            reply.addString("- place");
            reply.addString("- grasp_it");
            reply.addString("- quit");
        }
        else if (cmd=="look_down")
        {
            look_down();
            // we assume the robot is not moving now
            reply.addString("ack");
            reply.addString("Yep! I'm looking down now!");
        }
/*
        // ACTIONS
        else if (cmd=="pass")
        {
            bool ok=pass();
            // we assume the robot is not moving now
            if (ok)
            {
                // we assume the robot is not moving now
                reply.addString("ack");
                reply.addString("Yep! I'm passing to you!");
            }
            else
            {
                reply.addString("nack");
                reply.addString("There was a problem!");
            }
        }
        else if (cmd=="place")
        {
            bool ok=place();
            // we assume the robot is not moving now
            if (ok)
            {
                reply.addString("ack");
                reply.addString("Yeah! I did it! Maybe...");
            }
            else
            {
                reply.addString("nack");
                reply.addString("There was a problem!");
            }
        }
*/
        else if (cmd=="grasp_it")
        {
            // the "closure" accounts for how much we should
            // close the fingers around the object:
            // if closure == 0.0, the finger joints have to reach their minimum
            // if closure == 1.0, the finger joints have to reach their maximum
            double fingers_closure=0.5; // default value

            // we can pass a new value via rpc
            if (command.size()>1)
                fingers_closure=command.get(1).asDouble();

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
        }
        else
            // the father class already handles the "quit" command
            return RFModule::respond(command,reply);
        return true;
    }

    /***************************************************/
    double CtrlModule::getPeriod()
    {
        return 1.0;
    }

    /***************************************************/
    bool CtrlModule::updateModule()
    {
        return true;
    }
