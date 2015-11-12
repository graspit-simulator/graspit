//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s): Matei T. Ciocarlie
//
// $Id: 
//
//######################################################################

/*! \file 
	This function creates an arch of blocks. The original reason was a project
	that did not continue. However, it is a useful test of the dynamics engine
	so it was left in. It is also a good test of the body cloning mechanism.
	This can be called from the Misc. menu of the main window.
*/

class World;

void create_arch(World *world, double inner_radius, double outer_radius, 
				double thickness, int n_blocks, bool add_supports);
