#include <yarp/sig/Vector.h>

#include "manual/placing.h"
#include "manual/passing.h"
#include "manual/compute.h"

/***************************************************/

using namespace yarp::sig; 
   
    /***************************************************/
    bool CtrlModule::placeLeft()
    {
        Vector x, o;
        iarm->getPose(x,o); //get current position of hand
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        // select the correct interface
        if (_hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        // get current pose of hand 
        iarm->getPose(x,o);

	// half point
        Vector move = x;
	if (_hand == "right"){
		move[0] = -0.22;  //move forward 10 cm
		move[1] = -0.05;  //move left 20 cm
		move[2] =   0.1;  //move down 10 cm
	}else{
		move[0] = -0.22;  //move forward 10 cm
		move[1] = -0.12;  //move left 20 cm
		move[2] =   0.1;  //move down 10 cm
	}
        iarm->goToPose(move, o);

        // fixate the new end goal
        fixate(move);
        yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

	// final end point
	if (_hand == "right"){
		move[0] = -0.30;  //move forward 10 cm
		move[1] = -0.05;  //move left 20 cm
		move[2] =  0.02;  //move down 10 cm
	}else{
		move[0] = -0.35;  //move forward 10 cm
		move[1] = -0.19;  //move left 20 cm
		move[2] =  0.02;  //move down 10 cm
	}

        // fixate the new end goal
        fixate(move);
        yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

        o = computeHandOrientationRotate(true); //get default orientation
        iarm->goToPositionSync(move);
        double timeout = 10.0; 
        bool done = false; 
        done = iarm->waitMotionDone(0.1,timeout); 
        if(!done){
            yWarning("Something went wrong with the initial approach, using timeout");
            iarm->stopControl();
        }
        Time::delay(1);

	release();

        // GO BACK
	if (_hand == "right"){
		move[0] = -0.25; //move backwards 10 cm
		move[1] =  0.05;    //move right 20 cm
		move[2] =  0.02; //move down 10 cm
	}else{
		move[0] = -0.25; //move backwards 10 cm
		move[1] = -0.05;    //move right 20 cm
		move[2] =  0.02; //move down 10 cm
	}
        goBack(move);

	fixate(move);
        
        return true;        
    }

    /***************************************************/
    bool CtrlModule::placeCenter()
    {
        Vector x, o;
        iarm->getPose(x,o); //get current position of hand
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        // select the correct interface
        if (_hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        // get current pose of hand 
        iarm->getPose(x,o);

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
        fixate(move);
        yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

        o = computeHandOrientationPassing(_hand); //get default orientation
        iarm->goToPose(move, o);

        Time::delay(1.0);

	fixate(move);
	release();

        // GO BACK
	if (_hand == "right"){
		move[0] = -0.25; //move backwards 10 cm
		move[1] =  0.05;    //move right 20 cm
		move[2] =  0.02; //move down 10 cm
	}else{
		move[0] = -0.25; //move backwards 10 cm
		move[1] = -0.05;    //move right 20 cm
		move[2] =  0.02; //move down 10 cm
	}
        goBack(move);

	fixate(move);

        return true;        
    }

    /***************************************************/
    bool CtrlModule::placeRight()
    {
        Vector x, o;
        iarm->getPose(x,o); //get current position of hand
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        // select the correct interface
        if (_hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        // get current pose of hand 
        iarm->getPose(x,o);
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
	
	// half point
        Vector move = x;
	if (_hand == "right"){
		move[0] = -0.22;  //move forward 10 cm
		move[1] =  0.10;  //move left 20 cm
		move[2] =   0.1;  //move down 10 cm
	}else{
		move[0] = -0.22;  //move forward 10 cm
		move[1] =  0.05;  //move left 20 cm
		move[2] =   0.1;  //move down 10 cm
	}
        iarm->goToPose(move, o);

        // fixate the new end goal
        fixate(move);
        yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

	// final end point
	if (_hand == "right"){
		move[0] = -0.30;  //move forward 10 cm
		move[1] =  0.15;  //move left 20 cm
		move[2] =  0.02;  //move down 10 cm

		// fixate the new end goal
		fixate(move);
		yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

		o = computeHandOrientationPassing(_hand); //get default orientation
		iarm->goToPoseSync(move, o);
		double timeout = 10.0; 
		bool done = false; 
		done = iarm->waitMotionDone(0.1,timeout); 
		if(!done){
		    yWarning("Something went wrong with the initial approach, using timeout");
		    iarm->stopControl();
		}
	}else{
		move[0] = -0.30;  //move forward 10 cm
		move[1] =  0.05;  //move left 20 cm
		move[2] =  0.02;  //move down 10 cm

		// fixate the new end goal
		fixate(move);
		yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

		iarm->goToPositionSync(move);
		double timeout = 10.0; 
		bool done = false; 
		done = iarm->waitMotionDone(0.1,timeout); 
		if(!done){
		    yWarning("Something went wrong with the initial approach, using timeout");
		    iarm->stopControl();
		}
	}

        Time::delay(0.5);

	release();

        // GO BACK
	if (_hand == "right"){
		move[0] = -0.25; //move backwards 10 cm
		move[1] =  0.05;    //move right 20 cm
		move[2] =  0.02; //move down 10 cm
	}else{
		move[0] = -0.25; //move backwards 10 cm
		move[1] = -0.05;    //move right 20 cm
		move[2] =  0.02; //move down 10 cm
	}
        goBack(move);

	fixate(move);

        return true;
    }

    /***************************************************/
    bool CtrlModule::release()
    {
    
        // we set up here the lists of joints we need to actuate
        VectorOf<int> fingers;

        for (int i=9; i<16; i++)
            fingers.push_back(i);

        moveFingers(_hand,fingers,0.0);
        yInfo()<<"released";

        Time::delay(0.5);

        moveFingers(_hand,fingers,0.5);
	yInfo()<<"grabbed the object again";

        return true;

    }

    /***************************************************/
    bool CtrlModule::goBack(Vector approach)
    {
        Vector x, o;
        o = computeHandOrientationPassing(_hand); //get default orientation

        // fixate the new end goal
        fixate(approach);
        yInfo()<<"fixating at ("<<approach.toString(3,3)<<")";

	x = approach;
	x[2] += 0.05;
	iarm->goToPose(x, o);

        Time::delay(0.2);

        // select the correct interface
        if (_hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        approachTargetWithHand(_hand,approach,o, true);

        yInfo()<< "Returned to initial position";
        return true;        
    }


