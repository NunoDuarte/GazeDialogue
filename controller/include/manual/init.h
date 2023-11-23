// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>

#include "helpers.h"

/***************************************************/

using namespace std;
using namespace yarp::os;  // to define RFModule
using namespace yarp::dev; // to define PolyDriver
using namespace yarp::sig;


/***************************************************/

class CtrlModule : public RFModule
{
    protected:
            PolyDriver drvArmR, drvArmL, drvGaze;
            PolyDriver drvHandR, drvHandL;
            PolyDriver drvTorso;
            ICartesianControl *iarm;
            IPositionControl *ipos;
            ICartesianControl *itorso;
            IGazeControl      *igaze;

            int startup_ctxt_arm_right;
            int startup_ctxt_arm_left;
            int startup_ctxt_gaze;

            RpcServer rpcPort;
            ObjectRetriever object;

            string _hand;

            void fixate(const Vector &x);
            void look_down();
            bool grasp_it(const double fingers_closure);

            Vector computeHandOrientation(const string &hand);
            Vector computeHandOrientationPassing(const string &hand);
            Vector computeHandOrientationRotate(bool giveObject);

            void initArm(const Vector &x, string robot);
            void changeDOFs();

            void approachTargetWithHand(const string &hand,
                                const Vector &x,
                                const Vector &o, bool wait);


            void liftObject(const string &hand);
            void moveFingers(const string &hand,
                     const VectorOf<int> &joints,
                     const double fingers_closure);

            void home(const string &hand);

            bool openCartesian(const string &robot, const string &arm);

            bool placeLeft();
            bool placeCenter();
            bool placeRight();
            bool release();
            bool goBack(Vector approach);

            bool passLeft();
            bool passCenter();
            bool passRight();
            bool releasePass();
            bool goBackPass(Vector approach);

    public:
            bool configure(ResourceFinder &rf);
            bool interruptModule();
            bool close();
            bool respond(const Bottle &command, Bottle &reply);
            double getPeriod();
            bool updateModule();

    private:
   
};

#endif


