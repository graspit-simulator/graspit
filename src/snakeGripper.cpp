#include "snakeGripper.h"
#include "world.h"


int 

SnakeGripper::loadFromXml(const TiXmlElement* root,QString rootPath)
{
	int result = Robot::loadFromXml(root, rootPath);
	if (result != SUCCESS) return result;
	myWorld->toggleCollisions(false, chainVec[0]->getLink(0), chainVec[1]->getLink(0));

	return result;
}

void 
SnakeGripper::cloneFrom(Hand *original)
{
	Hand::cloneFrom(original);
	myWorld->toggleCollisions(false, chainVec[0]->getLink(0), chainVec[1]->getLink(0));
}
