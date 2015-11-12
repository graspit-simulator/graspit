#CONFIG += dbg
#select PCA library.  
#possible values opencv
PCALIB = opencv

#select mesh library
#possible values openmesh
MESHLIB = openmesh


#------------------system-specific libraries---------------
CONFIG +=  $$MESHLIB $$PCALIB

dbg {
    CONFIG += debug
} else {
    CONFIG += release
}

win32 {
	include(graspit-lib-WINDOWS.pro)
} else {
	include(graspit-lib-LINUX.pro)
}

#------------------PCA utility core files---------------------
include(database_model_pca-core.pro)
 





