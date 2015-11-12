#include "strategyPlannerDlg.h"

#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "body.h"
#include "robot.h"
#include "triangle.h"

#include "environment.h"

void StrategyPlannerDlg::planButton_clicked()
{
	std::vector<Triangle> triangles;
	World * w = graspItGUI->getIVmgr()->getWorld();
	Hand * h = w->getCurrentHand();
	Body * b = w->getBody(0);
	b->getGeometryTriangles(&triangles);
	std::cout << "num of triangles are: " << triangles.size() << std::endl;

	Environment e;
	e.setSurfaceListFromTriangleSet(triangles);

	transf newTran = h->getTran() * transf(Quaternion::IDENTITY, vec3(0,0,-1000));
	
	bool result = h->moveTo(newTran, 50*Contact::THRESHOLD, M_PI/36.0);
	std::cout << "number of contacts are: " << w->getBody(0)->getNumContacts() << std::endl;
	std::list<Contact*> cl = w->getBody(0)->getContacts();

	std::cout << cl.front()->getLocation() << std::endl;
	e.findSurfaceHit(cl.front()->getLocation());

}