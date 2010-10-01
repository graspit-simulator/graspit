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

/*! \file 
	This function creates an arch of blocks. The original reason was a project
	that did not continue. However, it is a useful test of the dynamics engine
	so it was left in. It is also a good test of the body cloning mechanism.
	This can be called from the Misc. menu of the main window.
*/

class World;

void create_arch(World *world, double inner_radius, double outer_radius, 
				double thickness, int n_blocks, bool add_supports);
