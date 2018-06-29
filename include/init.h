// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/Vector.h>
#include <opencv2/core/mat.hpp>               // add Mat variables from OpenCV
#include <iostream>
#include <fstream>                            // add libraries to define output files

#define NBSAMPLES 1

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;
#include "helpers.h"

/***************************************************/

using namespace std;
using namespace yarp::os;  // to define RFModule
using namespace yarp::dev; // to define PolyDriver
using namespace yarp::sig;


/***************************************************/

class ControlThread: public RateThread
{
    protected:

            PolyDriver drvArmR, drvArmL, drvGaze, drvHandR, drvHandL;

            ICartesianControl *iarm;
            IGazeControl      *igaze;

            ObjectRetriever object;

            ActionRetriever act;
            int action;    

            Port inPort;

            //BufferedPort<Bottle> port;
            ofstream myfile;
            ofstream myfile2;
            BufferedPort<Bottle> port;

            time_t timer1;       // define time variable 1
            time_t timer2;       // define time variable 2

            int startup_ctxt_gaze;
            string _hand;
    
            // Load the data from the eyes
            std::vector< std::vector<float> > Eyes;

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

            // define the sequence of states
            cv::Mat seq_mat;
            cv::Mat seq_mat_wTime;
            cv::Mat seq;
            int cnt;

            // output from decode
            cv::Mat pstates;

            // calculte distances for giving action
            float Vmax;
            float epsilon;
            int count;
            Vector e, ed;
            Vector unit_e;
            double v_mag;
            double acc_mag;
            double mag_e; 

            PolyDriver drvTorso;
            IPositionControl2 *ipos;
            ICartesianControl *itorso;

            int startup_ctxt_arm_right;
            int startup_ctxt_arm_left;

            RpcServer rpcPort;

            // grasp once
            bool grasp;
            bool released;

    public:

            ControlThread(int period):RateThread(period){}
            bool threadInit();
            void run();
            void threadRelease();

            bool openCartesian(const string &robot, const string &arm);

    private:

            void fixatePoint(const Vector &x);
            void look_down();
            bool grasp_it(const double fingers_closure);

            Vector computeHandOrientation(const string &hand);

            void changeDOFs();
            void approachTargetWithHand(const string &hand,
                                const Vector &x,
                                const Vector &o, bool wait);


            void liftObject(const string &hand);

            bool place();

            Vector computeHandOrientationPassing(const string &hand);
            float magnitude(Vector x);
            void fixate(int maxState);
            void arm(int maxState);
            void release(string hand);

            void startingArm(const Vector &x);
            void initArm(const Vector &x);
            void moveFingers(const string &hand, const VectorOf<int> &joints,
                    const double fingers_closure);

            //void reachArmGiving(Vector desired_p, Vector orientation, 
            //        Vector x_pos, Vector velocity);

            std::vector< std::vector<float> > loadDataFile(std::string file, 
                    bool convert);      

            int predictAL(cv::Mat& act_prob, int cur_state, int cur_action);
            void actionBehavior(int state);
            void reachArmGiving(Vector desired_p, Vector orientation, 
                    Vector x_pos, Vector velocity);
            void gazeBehavior(cv::Mat &state);
   

};

#endif


