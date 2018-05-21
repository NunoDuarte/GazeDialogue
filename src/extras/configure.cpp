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

    bool ControlThread::interruptModule()
    {
        return true;
    }

    /***************************************************/
    bool ControlThread::close()
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
    bool ControlThread::respond(const Bottle &command, Bottle &reply)
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
            return respond(command,reply);
        return true;
    }

    /***************************************************/
    double ControlThread::getPeriod()
    {
        return 1.0;
    }

    /***************************************************/
    bool ControlThread::updateModule()
    {
        return true;
    }
