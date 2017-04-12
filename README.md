[![Build Status](https://travis-ci.org/graspit-simulator/graspit.svg?branch=master)](https://travis-ci.org/graspit-simulator/graspit)

<h1>GraspIt!</h1>

Introduction
------------
Please see the User Manual found at http://graspit-simulator.github.io/ for an introduction to GraspIt!, a list of
features, installation instructions, getting started examples, etc.

Distribution Contents
---------------------

**CMakeLists.txt**: Used to compile GraspIt!, multiple flags that can be set with ccmake. <br />
**CMakeMacros**: contains .cmake files used to find GraspIt! dependencies.  <br />
**cmdline**: command line parser used by GraspIt!.  <br />
**ci**: contains scripts used by travis ci.  <br />
**doc**: Documentation.  Contains both the User Manual and code Reference Manual. The User Manual contains installation instructions, pointers for getting started, examples, and trouble shooting and contact information. <br />
**images**: A place to put images saved from GraspIt! <br />
**include**: Header files for the main GraspIt! source code <br />
**LICENSE.txt**: A copy of the license you accepted when you downloaded this. <br />
**models**: The geometry and configuration files for all the robots and
		objects. <br />
**plugins**:  Examples for creating plugins that can be loaded into GraspIt! 
                 at run time, and can use GraspIt! without being statically linked
		 into GraspIt's main executable. <br />
**ply**:  Code for loading .ply files; see header files for authorship 
                information and detail. <br />
**qhull**: A popular package for computing n-dimensional convex hulls.
		This is used both for the contact system and to create grasp
	      wrench spaces. <br />
**README.md**:  This file. <br />
**src**:  The source code for GraspIt!. <br />
**src/DBase**: source code for the interface between GraspIt and the Columbia 
		Grasp Database <br />
**tinyxml**: a library for processing XML documents. See the header files 
		therein for license and author information for this package. <br />
**ui**:	The dialog windows and interfaces for GraspIt!. <br />
**worlds**: A place to save GraspIt! worlds.  Also includes a few
		examples. <br />

Examples of How to Integrate GraspIt! into your own Project
---------------------

https://github.com/graspit-simulator/graspit_interface <br />
https://github.com/JenniferBuehler/graspit-pkgs <br />
https://github.com/ros-interactive-manipulation/graspit_simulator <br />
https://github.com/OSUrobotics/graspit_ros_plannings <br />
