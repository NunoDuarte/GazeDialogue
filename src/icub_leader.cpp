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
#include <time.h>

//#include "init.h"
#include "compute.h"

#define NBSAMPLES 1

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

    std::vector< std::vector<float> > ControlThread::loadDataFile(std::string file, 
                                                                    bool convert = false)
    {
        // Load the dataset from a file
        float valTmp;
        char tmp[1024];
        unsigned int l=0, c=0;
        // initialize the vector size
        std::vector< std::vector<float> > result(
                1001,
                std::vector<float>(4)); 

        yInfo() << "loading file: " << file;
        std::ifstream f(file.c_str());

        if (f.is_open()){
            // Get number of rows
            while(!f.eof()){ 
                f.getline(tmp,1024);
                l++;
                if (l==1)
                {
                    // Get number of columns
                    std::istringstream strm;
                    strm.str(tmp); 
                    while (strm >> valTmp)
                    c++;
                }
            }
            l--;
            f.clear();
            f.seekg(0); // returns to beginning of the file

            // for head and arm data
            if (not convert) {
                for(unsigned int i=0;i<l;i++){ 
                    f.getline(tmp,1024);
                    std::istringstream strm;
                    strm.str(tmp); 
                    for(unsigned int j=0;j<c;j++){
                        strm >> result[i][j];
                        //printf("%f ",result[i][j]);
                    }
                    //printf("\n");
                }
            // for eyes data
            } else {
                for(unsigned int i=0;i<l;i++){ 
                    f.getline(tmp,1024);
                    std::istringstream strm;
                    strm.str(tmp); 
                    for(unsigned int j=0;j<c;j++){
                        strm >> result[i][j];
                        if (j == 1){
                            if (result[i][j] == 1){
                                result[i][1] = -0.55;
                                result[i][2] =  0.00;
                                result[i][3] =  0.45;                    
                            }else if (result[i][j] == 2){
                                result[i][1] = -0.35;  //move forward 10 cm
                                result[i][2] = -0.19;  //move left 20 cm
                                result[i][3] =  0.02;  //move down 10 cm
                            }else if (result[i][j] == 3){
                                result[i][1] = -0.25;
                                result[i][2] =  0.30;
                                result[i][3] =  0.45;                
                            }
                        }else{
                            continue;//printf("%f ", float(result[i][j]));
                        }
                    }
                    printf("\n");
                }
            }
        }else{
            std::cout  << std::endl << "Error opening file " << std::endl; 
        }
        f.close();
        return result;
    }    

    int ControlThread::predictFollower(cv::Mat& act_prob, int cur_state, int cur_action){
        /***********************************************************************************
        act_prob is a 2x1 matrix with two probabilities, 
            <0,0> is probability for giving, and 
            <1,0> i probability for placing action. They sum to 1.
        This value (as argument) containes the current probabilities 
            for the two actions that are updated within the function.

        cur_state is 0-5 integer representing the loeaders focuse of attention. 
            0 - TM face 
            1 - TM hand 
            2 - TM tower 
            3 - Own tower

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
        double APdataL[] = {0.495, 0.505,
                          0.617, 0.383,
                          0.293, 0.706,
                          0.844, 0.156};
        cv::Mat APL = cv::Mat(4,2,CV_64F,APdataL).clone();

        int new_action;//0-giving 1-placing -1-uncertain

        double rn = rand();

        double prob_G = 0.5;
        double prob_P = 0.5;
        double deltalgc = 0.0;
        double deltalpc = 0.0;
        double treshold = 0.15;
        double alfa = 0.1;//parameter of exponential moving average

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
    void ControlThread::actionBehavior(int state)
    {
        // input: state of the human
        //        logpseg -- irrelevant for now
        // output: behavior of robot
        int action;
        double logpseq;

        // call the predictor function
        action = predictFollower(act_probability, state, action);
        //myfile << act_probability << "\n";
        yInfo() << "predicting" << act_probability.at<double>(0,0);
        yInfo() << "predicting" << act_probability.at<double>(1,0);

        time(&timer2);           // get current time
        float diffTime = timer2 - timer1;

        // add state to sequence of states
        int rows = seq_mat.rows;
        seq_mat.push_back(state);      
        //seq = seq_mat.t();
        seq.at<double>(0,cnt) = state;
        // store timestamp of the specific state
        seq_mat_wTime.at<double>(0,cnt) = state;
        seq_mat_wTime.at<double>(1,cnt) = diffTime;

        cout << "M = " << endl;
        for(int nu=0;nu<cnt;nu++) cout << " "  << seq.at<double>(0,nu);
        cout << endl;
        cnt++;
        
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



        } else {
            yInfo() << "I don't know yet";

        }
        //myfile2 << pstates << "\n";
    }

    void ControlThread::run()
    {
      	double pupil; string hand; Vector gaze;
        // state the human is in

        Bottle *b = inPort.read(false);
        if (b != NULL)
        {
            yInfo() << b;
            pupil = b->get(1).asDouble();
            yInfo() << "pupil" << pupil;
            yInfo() << "Human";
            if ( pupil == 2){
                yInfo() << "iCub's Tower";
                state = 4;
           
            }else if ( pupil == 1) {
                yInfo() << "Human Tower";
                state = 5;
                
            }else if ( pupil == 3) {
                yInfo() << "Brick/Object";
                state = 0;
  
            }else if ( pupil == 7) {
                yInfo() << "iCub's Face";
                state = 1;

            }else if ( pupil == 5) {
                yInfo() << "iCub's Hand";
                state = 2;

            }else if (pupil == 4) {
                yInfo() << "Human's Hand";
                state = 3;
                
            } else {
                yInfo() << "no state";
                state = -1;            
            }
            //state=5;
            if (state != -1){
                // if you observe the human looking at one of the states then act
                actionBehavior(state);
            }
        }else{
            yInfo() << "Exceeded the 0.1 ms of the thread frequency - output: no action";
        }
      	
        count++;

        // Mutual Alignment Model
        double logpseq;
        
        // Initialize the state
        if (count == 1){ state = 0;}

        // Add to Sequence
        seq.at<double>(0,count) = state;


        // Generate Next State
        state = mcLG.mutualAlign(seq,TRANSLGbhon,TRANSLGahon,INITLG,logpseq,pstates,count);

        if (count < 50){

            // start by looking at Brick
            int look = 1;       

            fixate(look);
            yInfo()<<"fixating at ("<< look <<")";

        // duration of action
        }else{

            fixate(state+1);

            yInfo() << "fixating at (" << state+1 << ")";
            arm(state+1);

            // -------------//----------------------
            // get current location
            iarm->getPose(p,o);

            e[0] = xf[0] - p[0];
            e[1] = xf[1] - p[1];
            e[2] = xf[2] - p[2];        

            if (magnitude(e) < 0.1 and (not released)){
                release("left");
                released = true;
            }
        }


        // save to output file - increment all for reading purposes
        myfile << state + 1 << ", ";
    }

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
        if ((Time::now()-startTime)>20)
            done=true;
    }
    
    myThread.stop();

    return 0;
}

