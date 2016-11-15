//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: 
//
//######################################################################

#include "arch.h"

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>

#include "body.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"

DynamicBody *rightBase;

Body* createSupport(double size, double thickness, World *world)
{
	Body *body = new Body(world,"Support");

	//8 vertices
	SbVec3f *points = new SbVec3f[8];
	//6 faces, 5 indices for each (last is -1)
//	int32_t *c = new int32_t[5*6];

	double z = thickness;

	points[3].setValue( size, size, z);
	points[2].setValue(-size, size, z);
	points[1].setValue(-size,-size, z);
	points[0].setValue( size,-size, z);

	points[7].setValue( size, size,-z);
	points[6].setValue(-size, size,-z);
	points[5].setValue(-size,-size,-z);
	points[4].setValue( size,-size,-z);

	int32_t c[5*6] = { 3, 2, 1, 0, -1,
					   4, 5, 6, 7, -1,
					   1, 5, 4, 0, -1,
					   2, 6, 5, 1, -1, 
					   3, 7, 6, 2, -1, 
					   4, 7, 3, 0, -1 };

	SoCoordinate3 *coords = new SoCoordinate3;
	coords->point.setValues( 0, 8, points );
	SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, 5*6, c);

	body->getIVGeomRoot()->addChild(coords);
	body->getIVGeomRoot()->addChild(ifs);
	body->setMaterial( world->getMaterialIdx("stone") );
	body->addIVMat();
	return body;
}


GraspableBody* create_block(double inner_radius, double outer_radius, double thickness, int n_blocks, World *world)
{
	double block_span = 3.14159 / n_blocks;
	double theta = block_span / 2;
	double radius = (inner_radius + outer_radius) / 2;

	double icx = inner_radius * cos(theta) - radius;
	double icz = inner_radius * sin(theta);

	double ocx = outer_radius * cos(theta) - radius;
	double ocz = outer_radius * sin(theta);

	double y = thickness / 2;

	GraspableBody *body = new GraspableBody(world,"Original Block");

	//8 vertices
	SbVec3f *points = new SbVec3f[8];
	//6 faces, 5 indices for each (last is -1)
//	int32_t *c = new int32_t[5*6];

	points[0].setValue( icx, y, -icz);
	points[1].setValue( icx, y,  icz);
	points[2].setValue( icx,-y,  icz);
	points[3].setValue( icx,-y, -icz);

	points[4].setValue( ocx, y, -ocz);
	points[5].setValue( ocx, y,  ocz);
	points[6].setValue( ocx,-y,  ocz);
	points[7].setValue( ocx,-y, -ocz);

	int32_t c[5*6] = { 3, 2, 1, 0, -1,
					   4, 5, 6, 7, -1,
					   1, 5, 4, 0, -1,
					   2, 6, 5, 1, -1, 
					   3, 7, 6, 2, -1, 
					   4, 7, 3, 0, -1 };

	SoCoordinate3 *coords = new SoCoordinate3;
	coords->point.setValues( 0, 8, points );
	SoIndexedFaceSet *ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, 5*6, c);

	body->getIVGeomRoot()->addChild(coords);
	body->getIVGeomRoot()->addChild(ifs);
	body->setMaterial( world->getMaterialIdx("stone") );
	body->addIVMat();

	double density = 2 * 1.0e-3; //from john ochsendorf: 2000 kg / m3 converted to g / mm3
	double volume = thickness * 0.5 * block_span * (outer_radius * outer_radius - inner_radius * inner_radius);
	double mass = density * volume;
	body->setMass(mass);
	body->setMaxRadius(body->computeDefaultMaxRadius());
	double I[9];
	position CoG;
	body->computeDefaultMassProp(CoG, I);
	body->setCoG(CoG);
	body->setInertiaMatrix(I);
	fprintf(stderr,"Volume %f and mass %f\n",volume, mass);

	return body;
}

void create_arch(World *world, double inner_radius, double outer_radius, double thickness, int n_blocks, bool add_supports)
{
	fprintf(stderr,"Building arch: inner radius %f, outer radius %f, thickness %f, %d blocks, %d add_supports\n",
					inner_radius, outer_radius, thickness, n_blocks, add_supports);

	GraspableBody* block = create_block(inner_radius, outer_radius, thickness, n_blocks, world);
	//add the reference block to collision detection
	block->addToIvc();
	//but disable its collisions
	world->toggleCollisions(false, block);

	transf blockTran,blockRot;
	double block_span = 3.14159 / n_blocks;
	double theta = block_span / 2;
	double radius = (inner_radius + outer_radius) / 2;
	for (int i=0; i<n_blocks; i++) {
		QString name("Block "),id;
		name.append(id.setNum(i));
		GraspableBody *addBlock = new GraspableBody(world,name.latin1());
		addBlock->cloneFrom(block); //this also adds it to collision detection!
		world->addBody(addBlock);

		Quaternion r( theta*(2*i+1) , vec3(0,-1,0) );
		//use radius + THRESHOLD so we don't have interpenetrating blocks due to numerical error
		vec3 t( radius+0.1 , 0, 0);

		blockRot.set(r, vec3(0,0,0) );
		blockTran.set(Quaternion::IDENTITY, t);

		addBlock->setTran( blockTran * blockRot);

		if (i==n_blocks-1)
			rightBase = addBlock;
	}
	if (add_supports) {
		Body* leftSupport = createSupport(0.9 * radius,50,world);
		leftSupport->setName( QString("Left Support") );
		Body* rightSupport = createSupport(0.9 * radius,50,world);
		rightSupport->setName( QString("Right Support") );

		leftSupport->addToIvc();
		rightSupport->addToIvc();

		vec3 t( radius+1, 0, -(50+Contact::THRESHOLD));
		blockTran.set( Quaternion::IDENTITY, t);
		leftSupport->setTran( blockTran );
		world->addBody(leftSupport);

		t.set( -radius-1, 0, -(50+Contact::THRESHOLD));
		blockTran.set( Quaternion::IDENTITY, t);
		rightSupport->setTran( blockTran );
		world->addBody(rightSupport);
	}
}

void archSnapshot()
{
	static int steps = 0;
/*
	if (steps%5 == 0) {
		QString fn( getenv("GRASPIT")+QString("/images/arch") );
		QString n;
		fn.append( n.setNum(steps) );
		fn.append( ".jpg" );
		graspItGUI->getIVmgr()->saveImage( fn );
	}
*/
	if (steps%10 == 0) {
		fprintf(stderr,"Set velocity\n");
		double newVel[6];
		for (int i=0; i<6; i++)
			newVel[i] = rightBase->getVelocity()[i];
		newVel[0] -= 100;
		rightBase->setVelocity(newVel);
	}
	steps++;
}
