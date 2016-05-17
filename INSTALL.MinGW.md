## Install instructions for MinGW/MSYS

Prerequisite: This guide is for advanced users who already have at least some experience with using cmake and the bash shell (or similar).

### Step 1. Install MinGW, MSYS, Qt and CMake

1. Download **MinGW** toolchain (from MinGW-w64 project) already set up for Qt from [here](http://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/4.8.2/threads-posix/dwarf/i686-4.8.2-release-posix-dwarf-rt_v3-rev3.7z/download) and extract it, e.g. to C:/MinGW. See also [this wiki page on qt.io](https://wiki.qt.io/MinGW).

2. Download **Qt 4** and run the installer (default destination is C:/Qt). We recommend to use the file [from qt.io](https://download.qt.io/archive/qt/4.8/4.8.6/).
  During installation, you have to locate the MinGW files previously extracted (see also [this useful resource](https://github.com/iat-cener/tonatiuh/wiki/Installing%20Qt%20For%20Windows) for more information about the Qt install)

3. Add the **MinGW** *bin* directory to your PATH in Windows.

4. Download and install [cmake](https://cmake.org/download/) for Windows.

5. Install **MSYS2**
    - Download MSYS2 [from here](https://msys2.github.io/) and execute the installer.
    - Open a MSYS shell and add the MinGW *bin* directory to PATH, eg. ``echo "export PATH=${PATH}:<path-to-minGW-bin>" >> .bashrc``
    - *(optional)*: Install vim for MSYS shell with ``pacman -S vim``

**General notes for compiling with cmake**

CMake and MSYS do not get along that well. When you use cmake from within
the MSYS command line, it will complain about sh.exe being in your PATH.
So in general, the best is to use the cmake graphical interface for Windows
to generate the makefiles, and then use the MSYS shell to compile.


**If** you explicitly want to try and run cmake from within a MSYS shell, you should set it to generate MinGW makefiles:    
```
cd build
cmake -G "MinGW Makefiles" ..
```

*Hint:* Pass -DCMAKE_PREFIX_PATH with a custom install directory for any dependencies installed in custom paths.
      
*Note about the error with sh.exe*    
This error will go away after re-running cmake.
Be careful though, because this may lead to the wrong compilers being chosen, ie. using the MSYS
gcc/g++ instead of the MinGW ones. You could try to explicitly set them with CMAKE_CXX_COMPILER and CMAKE_C_COMPILER.
In this guide however, only instructions for using the Windows CMake GUI are given.

*Compiling*        

For convenience, you may want to create a softlink to *ming32-make.exe* and call it *make.exe*. Alternatively, just run the mingw *make* explicitly:

``mingw32-make``
    

### Step 2. Install basic dependencies

1. lapack/blas: Install from source (see also [this documentation](http://icl.cs.utk.edu/lapack-for-windows/lapack/)).
    You can use the [original lapack source](http://netlib.org/lapack/lapack.tgz).
2. qhull: Compile from source cloning [this repository](https://github.com/qhull/qhull), and install.

Alternatively, try pacman to install the libraries.     
``pacman -Ss lapack``       
will list the specific name of the package.
Then install with   
``pacman -S <name>``

For qhull you may try the same: find out package name with ``pacman -Ss qhull`` and install with ``-S``

###Step 3. Install Coin and SoQt

A good set of instructions can be found on [this github wiki](https://github.com/iat-cener/tonatiuh/wiki/Installing-SoQt-For-Windows).
Please refer to the instructions given there for installation.

Binaries for Coin/SoQt exist as well, provided on [ascend4.org](http://ascend4.org/Building_Coin3d_and_SoQt_on_MinGW).
However they are not the most recent version any more.

A few notes about the installation:

*Coin*    
-  Use the extra configure flag  ``--build=x86_64-w64-mingw32``, or if you are not on x86_64, get the output of ``gcc -dumpmachine``.
   Careful though not to use the MSYS build (it has to be the MinGW build).
- As of May 2016, the source code had to be edited as suggested in the instructions linked above, except the change in *freetype.cpp* which was not necessary
- You also may need to install diffutils: ``pacman -S diffutils``
- Add the *bin* directory of the resulting Coin install to your PATH environment variable.

*SoQt*    
- The environment variable QTDIR has to be set to the directory where all the Qt includes can be found.

###Step 4. Install bullet

This is very straight forward with cmake. Clone [the bullet repository](https://github.com/bulletphysics/bullet3) and build with cmake.

###Step 5. Compile graspit

For compiling with CMake, the path to the **MSYS bash.exe** needs to be passed to cmake.
This is required for cmake to be able to call *soqt-config* (which is not provided as Windows .exe file and needs to be run via bash).     
So you will need to pass the path to bash.exe (probably this is ``<your-msys-install>/usr/bin``) with the CMAKE_PREFIX_PATH.

Use the **CMake GUI** to generate the make files, as running cmake from the MSYS shell may be problematic (see also building with cmake explained in Step 1).

1. Start CMake GUI.
2. Select source and build directory, then click "Configure".
3. Select "MinGW Makefiles", and you may first try to stick to the default compilers.   
    *If* it does not compile, or the default compilers causes other issues, try to re-run cmake and this time select "Specify native compilers",
    choosing the fortran.exe/gcc.exe/g++.exe *from your MinGW install* (not MSYS).
4. The first configuration process may be unsuccessful because not all directories are included.
    Either way, you still need to add the following to your CMAKE_PREFIX_PATH:
    - the path to bash.exe
    - the path to the Qt files (QTDIR), and
    - any other dependency directories.      
    You can add the paths by clicking on "New Entry" and add a variable named CMAKE_PREFIX_PATH, in which you add all paths, separated by semicolons.
5. Click "Configure" again. It should be printing something like **"Result of soqt_config: ..." followed by a list of libraries**.
    *If it does not print the libraries, it did not find bash.exe, and the makefiles have not been generated properly!!*
6. After configuration was successful, click on "Generate".

After the cmake files were generated in the GUI,
go to the MSYS shell, change to the build directory, and type

``mingw32-make``

That's all, you should be ready to go, try the **graspit_simulator.exe** in your build directory!
