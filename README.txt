                                 GraspIt!
                              Version 0.9.0

Introduction
------------
GraspIt! is a robotic grasping simulator that can import a wide variety of
robot mechanisms and objects.  It includes a collision detection and contact
determination mechanism that allows a user to interactively control the
joints of an arm and hand to create grasps of a target object.  These grasps
can be analyzed and evaluated by the system.  It also includes a dynamics
engine that can simulate the motions of robots and objects under the
influence of controlled motor forces, contact forces, joint constraint forces,
and external forces.  Additionaly, there is a novel grasp planner for the
Barrett hand that can generate a number of candidate grasp for an object and
then evaluate those grasps to find the best ones.

Disclaimer
----------
This is code I have been using in my research.  Except for the included
packages and the grasp planner, I have written everything myself.  There are
still several unfinished pieces and plenty of ways to crash the system.  It's
by no means bullet proof.  Although I'm not working on it full time anymore,
I will continue to support it as best I can, and I intend to release future
versions.  However, rather than waiting any longer to release a version, I
wanted to release what I have in the hopes that it will be useful in your
research.

Distribution Contents
---------------------

BUGS.txt      -  A current list of known bugs.  Unfortuantely, this list is
                 far from complete.  If you find a bug, please e-mail me at
		 amiller@cs.columbia.edu, and I'll do my best to fix it.

doc/          -  HTML documentation.  Contains both user and developer
		 documentation.  Open doc/index.html for the contents.

images/       -  A place to put images saved from GraspIt!

include/      -  Include files for the main GraspIt! source code

INSTALL.txt   -  Points you to doc/html/user/install.html for installation
	         instructions.

LICENSE.txt   -  A copy of the license you accepted when you downloaded this.

Makefile      -  The main makefile for graspit (used for Linux, not needed for
	         Windows).

matlab/       -  MEX c-files that compile into matlab functions that can
		 communicate and interact with the GraspIt! system.

models/       -  The geometry and configuration files for all the robots and
	         objects.

PQP-VCOLLIDE  -  Modified versions of the PQP and VCOLLIDE systems from UNC.
                 These make up the collision detection and contact
	         determination system in GraspIt!.

qhull	      -  A popular package for computing n-dimensional convex hulls.
		 This is use both for the contact system and to create grasp
	         wrench spaces.

README        -  This file.

src           -  The source code for GraspIt!.  Contains the Qt project files
	         that must be edited for your system.

TODO.txt      -  An ever increasing list of things features that I would like
	         to add.

worlds/       -  A place to save GraspIt! worlds.  Also includes a few
	         examples.

Examples
--------
After building GraspIt!, try running it and choose Open... from the File menu.
In the worlds folder, there are a few examples you can try (I'll try to add more
in the future):

barrettGlassDyn.wld -  This is the setup for the experminent shown in the movie
on the main GraspIt! page.  Move in to look at the glass and start the
dynamics.  Then press CTRL-G to start an auto grasp.  You should see the grasp
squeezed out of the hand and fall to the floor.  Now open that world again
(DON'T save it!) and this time select the Barrett hand with the select tool
and change its material to rubber.  Now repeat the experiment, and it should
successfully grab the glass.

cvap.wld - This is a reproduction of the former living room lab at the Center
for Autonomous Systems at the Royal Institute of Technology in Stockholm.  It
demonstrates how multiple obstacles and robots can be combined into a large
world.  Try moving the robots on the platform around.

robonautTest.wld - This has a cube suspended over a robonaut hand.  Start the
dynamics and immediately press CTRL-G to start an autograsp.  If the grasp was
started early enough, the hand should ultimately grasp the cube and will close
tighter and tighter until the joint torque limits are reached.  When the cube
stops moving, the friction cones get a little bigger, as it moves from kinetic
to static friction.


Troubleshooting
---------------
Please e-mail amiller@cs.columbia.edu, and I will do my best to help you out.
Please also send comments and feedback!
