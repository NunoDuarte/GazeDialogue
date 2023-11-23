#include <yarp/sig/Vector.h>

#include "manual/passing.h"

/***************************************************/

using namespace yarp::sig; 
 
    /***************************************************/
    bool CtrlModule::passLeft()
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

        Vector move = x;
	if (_hand == "right"){
		move[0] = -0.35;  //move forward 22 cm
		move[1] = -0.12;  //move left 17 cm
		move[2] =  0.10;  //move up 18 cm
	}else{
		move[0] = -0.35;  //move forward 22 cm
		move[1] = -0.21;  //move left 17 cm
		move[2] =  0.17;  //move up 18 cm
	}

        // fixate the new end goal
	Vector look = move;
	if (_hand == "right"){
		look[0] = -0.55;
		look[1] = -0.35;
		look[2] =  0.45;
	}else{
		look[0] = -0.35;
		look[1] = -0.21;
		look[2] =  0.40;
	}


        fixate(look);
        yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

        // remove the torso DOF
        changeDOFs();

        o = computeHandOrientationPassing(_hand); //get default orientation
        iarm->goToPose(move, o);

        Time::delay(2.5);

	fixate(look);
	releasePass();

        // GO BACK
	if (_hand == "right"){
		move[0] = -0.25; //move to the initial position (x = -15 cm)
		move[1] =  0.05;    //move to the center of the table (y = 0 cm)
		move[2] =  0.02; //move close to the height of the table (z = -5 cm)
	}else{
		move[0] = -0.25; //move to the initial position (x = -15 cm)
		move[1] = -0.05;    //move to the center of the table (y = 0 cm)
		move[2] =  0.02; //move close to the height of the table (z = -5 cm)
	}

        goBackPass(move);

	fixate(move);
        
        return true;        
    }

    /***************************************************/
    bool CtrlModule::passCenter()
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

        Vector move = x;
	if (_hand == "right"){
		move[0] = -0.40;  //move forward 22 cm
		move[1] =  0.00;  //move left 17 cm
		move[2] =  0.15;  //move up 18 cm
	}else{
		move[0] = -0.35;  //move forward 22 cm
		move[1] = -0.04;  //move left 17 cm
		move[2] =  0.17;  //move up 18 cm
	}

        // fixate the new end goal
	Vector look = move;
	if (_hand == "right"){
		look[0] = -0.55;
		look[1] =  0.00;
		look[2] =  0.45;
	}else{
		look[0] = -0.35;
		look[1] = -0.04;
		look[2] =  0.40;
	}

        fixate(look);
        yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

        o = computeHandOrientationPassing(_hand); //get default orientation
        iarm->goToPose(move, o);

        Time::delay(1.0);

	fixate(look);
	releasePass();

        // GO BACK
	if (_hand == "right"){
		move[0] = -0.25; //move to the initial position (x = -15 cm)
		move[1] =  0.05;    //move to the center of the table (y = 0 cm)
		move[2] =  0.02; //move close to the height of the table (z = -5 cm)
	}else{
		move[0] = -0.25; //move to the initial position (x = -15 cm)
		move[1] = -0.05;    //move to the center of the table (y = 0 cm)
		move[2] =  0.02; //move close to the height of the table (z = -5 cm)
	}

        goBackPass(move);

	fixate(move);

        return true;        
    }

    /***************************************************/
    bool CtrlModule::passRight()
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

        Vector move = x;
	if (_hand == "right"){
		move[0] = -0.25;  //move forward 22 cm
		move[1] =  0.30;  //move left 17 cm
		move[2] =  0.10;  //move up 18 cm
	}else{
		move[0] = -0.35;  //move forward 22 cm
		move[1] =  0.11;  //move left 17 cm
		move[2] =  0.17;  //move up 18 cm
	}

        // fixate the new end goal
	Vector look = move;
	if (_hand == "right"){
		look[0] = -0.25;
		look[1] =  0.30;
		look[2] =  0.45;
	}else{
		look[0] = -0.35;
		look[1] =  0.11;
		look[2] =  0.40;
	}

        fixate(look);
        yInfo()<<"fixating at ("<<move.toString(3,3)<<")";

        // remove the torso DOF
        changeDOFs();

        o = computeHandOrientationPassing(_hand); //get default orientation
        iarm->goToPose(move, o);

        Time::delay(2.5);

	fixate(look);
	releasePass();

        // GO BACK
	if (_hand == "right"){
		move[0] = -0.25; //move to the initial position (x = -15 cm)
		move[1] =  0.05;    //move to the center of the table (y = 0 cm)
		move[2] =  0.02; //move close to the height of the table (z = -5 cm)
	}else{
		move[0] = -0.25; //move to the initial position (x = -15 cm)
		move[1] = -0.05;    //move to the center of the table (y = 0 cm)
		move[2] =  0.02; //move close to the height of the table (z = -5 cm)
	}

        goBackPass(move);

	fixate(move);

        return true;
    }

    /***************************************************/
    bool CtrlModule::releasePass()
    {
    
	// we set up here the lists of joints we need to actuate
	VectorOf<int> fingers;
	VectorOf<int> arm;
	Vector x, o;

	for (int i=9; i<16; i++)
	    fingers.push_back(i);

	for (int i=0; i<6; i++)
	    arm.push_back(i);

	// select the correct interface
	if (_hand=="right")
	    drvArmR.view(iarm);
	else
	    drvArmL.view(iarm);

	// get current pose of hand 
	iarm->getPose(x,o);

	o = computeHandOrientationPassing(_hand); //
	iarm->goToPoseSync(x, o);
	double timeout = 10.0; 
	bool done = false; 
	done = iarm->waitMotionDone(0.1,timeout); 
	if(!done){
	    yWarning("Something went wrong with the initial approach, using timeout");
	    iarm->stopControl();
	}

	moveFingers(_hand,fingers,0.0);
	yInfo()<<"released";

	Time::delay(5);

	moveFingers(_hand,fingers,0.5);
	yInfo()<<"grabbed the object again";

	// get current pose of hand 
	iarm->getPose(x,o);

	o = computeHandOrientationPassing(_hand); //
	iarm->goToPoseSync(x, o);
	done = iarm->waitMotionDone(0.1,timeout); 
	if(!done){
	    yWarning("Something went wrong with the initial approach, using timeout");
	    iarm->stopControl();
	}

	return true;
    }

    /***************************************************/
    bool CtrlModule::goBackPass(Vector approach)
    {
        Vector x, o;

        // get current pose of hand 
        iarm->getPose(x,o);

        // fixate the old goal first
        fixate(x);
        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";

        // select the correct interface
        if (_hand=="right")
            drvArmR.view(iarm);
        else
            drvArmL.view(iarm);

        // get current pose of hand 
        o = computeHandOrientationPassing(_hand); //get default orientation

        approachTargetWithHand(_hand,approach,o, false);

        // fixate the new end-goal
        fixate(approach);
        double timeout = 10.0; 
        bool done = false; 
        done = iarm->waitMotionDone(0.1,timeout); 
        if(!done){
            yWarning("Something went wrong with the initial approach, using timeout");
            iarm->stopControl();        
        }

        yInfo()<<"fixating at ("<<x.toString(3,3)<<")";
        yInfo()<< "Returned to initial position";
        return true;        
    }





