colladaconverter:

	Implements a stand-alone converter for converting Collada files
	into the OFF format, which GraspIt can read natively.

	Usage:

	Install FCollada which can be downloaded from 
	http://www.feelingsoftware.com/ (tested on version 3.05B). 
	If you choose to install using the binary file, 
	choose the destination folder to install to be 
	C:\Program Files\Feeling Software\FCollada\ (this is default).
	If you choose to download the source files, extract them to 
	C:\Program Files\Feeling Software\.

	Build FCOLLADA in "Debug Unicode" mode. We suggests that you use 
	Mricosoft Visual Studio 2005 or later (tested on MSVS 2005).

	Build colladaconverter in "Debug" mode.

	Run colladaconverter as follows:
	colladaconverter filename1 filename2 ...

	The result of a successful conversion will be an off file and an xml
	file corresponding to the off file.

	This converter uses a number of packages that are external to GraspIt:
	- FCollada (Feeling Software Inc, http://www.feelingsoftware.com/)
		Use its libraries to load collada and triangulate meshes.
	- ColladaLoader (Ricardo Ruiz Lopez, http://colladaloader.sourceforge.net/)
		Modify load-meshes part in COlladaLoader and use it to read in meshes
		and fetch vertices.

	Note that, for the moment, we have been unable to link and build this
	converter under Linux.