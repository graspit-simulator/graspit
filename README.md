[![Build Status](https://travis-ci.org/graspit-simulator/graspit.svg?branch=master)](https://travis-ci.org/graspit-simulator/graspit)

<h1>GraspIt!</h1>

<h2>Version 2.2.0</h2>

Introduction
------------
Please see the User Manual in doc/ for an introduction to GraspIt!, a list of
features, installation instructions, getting started examples, etc.

Disclaimer
----------
This is code that has been, and is currently being used, for research. There 
are still many unfinished pieces and plenty of ways to crash the system.  It's
by no means bullet proof. Please see the Introduction in the User Manual for 
more details.

Distribution Contents
---------------------

**CMakeLists.txt**: Used to compile GraspIt!, multiple flags that can be set with ccmake. 

**CMakeMacros**: contains .cmake files used to find GraspIt! dependencies. 

**cmdline**: command line parser used by GraspIt!. 

**ci**: contains scripts used by travis ci. 

**doc**: Documentation.  Contains both the User Manual and code Reference Manual. The User Manual contains installation instructions, pointers for getting started, examples, and trouble shooting and contact information.

**images**: A place to put images saved from GraspIt!

**include**: Header files for the main GraspIt! source code

**LICENSE.txt**: A copy of the license you accepted when you downloaded this.

**models**: The geometry and configuration files for all the robots and
		objects.

**plugins**:  Examples for creating plugins that can be loaded into GraspIt! 
                 at run time, and can use GraspIt! without being statically linked
		 into GraspIt's main executable.

**ply**:  Code for loading .ply files; see header files for authorship 
                information and detail.

**qhull**: A popular package for computing n-dimensional convex hulls.
		This is used both for the contact system and to create grasp
	      wrench spaces.

**README.md**:  This file.

**src**:  The source code for GraspIt!.

**src/DBase**: source code for the interface between GraspIt and the Columbia 
		Grasp Database

**tinyxml**: a library for processing XML documents. See the header files 
		therein for license and author information for this package.

**ui**:	The dialog windows and interfaces for GraspIt!.

**worlds**: A place to save GraspIt! worlds.  Also includes a few
		examples.

Examples of How to Integrate GraspIt! into your own Project
---------------------
https://github.com/graspit-simulator/graspit-ros

https://github.com/ros-interactive-manipulation/graspit_simulator

https://github.com/JenniferBuehler/graspit-pkgs

https://github.com/curg/graspit_interface

https://github.com/OSUrobotics/graspit_ros_plannings

Let me know if there are others that are missing.
