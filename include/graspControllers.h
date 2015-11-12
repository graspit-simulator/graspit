#ifndef GRASPIT_GRASPCONTROLLERS
#define GRASPIT_GRASPCONTROLLERS
#include <vector>
#include "matvec3D.h"

enum GraspState {NOTHING, APPROACHING, BACKING_OFF, TRENDING_LEFT, TRENDING_RIGHT, CENTER, CLOSING, ERR};
class Robot;
class Hand; 
class World;
class Body;
class transf;

class GraspControllerInterface{
public:
	virtual void turnOn() = 0;
	virtual void turnOff() = 0;
	virtual bool isOn() = 0;
	virtual void controlCommand() = 0; //Does some stuff and then calls setDOF on the relevant robots.
	virtual GraspState getState() = 0; //Gets the current state -- assumes a finite automata approach to grasping
	virtual bool setState(GraspState g) = 0; //Set the current state -- may do nothing if state is a derived property of the system
protected:							//IF IT DOES SET THE STATE, IT IS EXPECTED TO TEST THE LEGITAMACY OF THE STATE TRANSITION
	static GraspState gs;
	static World *myWorld;
	static bool controllerOn;
};


class TapTapController: public GraspControllerInterface{
public:
	TapTapController(World *w, bool doSetState = true);
	virtual void controlCommand();
	GraspState getState();
	virtual bool setState(GraspState g);
	virtual bool setTarget();
	Body *getTarget();
	bool setHand();
	Hand *getHand();
	bool setRobot(int n);
	Robot *getRobot();
	bool setEndPose();
	virtual void turnOn(){controllerOn = true;};
	virtual void turnOff(){controllerOn = false;};
	virtual bool isOn(){return controllerOn;};
protected:
	std::vector<GraspState> stateVector;
	Robot * puma;
	Hand * hand;
	Body * target;
	transf startPose, endPose, rightPose, leftPose, centerPose;
	void groupForces(double *gf);
	vec3 appVec;
	
};

class TapTapDOFController: public TapTapController{
public:
	TapTapDOFController(World * w);
	virtual void controlCommand();
	virtual bool setState(GraspState g);
	void setState();
	~TapTapDOFController();
protected:
	double * dofActivation;
	vec3 desiredVelocityVector;
	bool applyArmForces();
	double speed; //mm/s

};

#endif