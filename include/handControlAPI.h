//this class is designed for controlling the hand from outside GraspIt!
#ifndef _HAND_CONTROL_API_H_
#define _HAND_CONTROL_API_H_
#include <vector>
#include "matvec3D.h"
class Hand;

class HandControlAPI{
private:
	static Hand* h;

public:
	HandControlAPI(){}
	void importHand();
	void setJointValues(const std::vector<double>& j);
        void setPose(const transf t);
	void outputTactileLocations();
	std::vector< std::vector<double> > getTactileLocations();
	void autoGrasp(bool render = false);
	void forceOpen();
};
#endif
