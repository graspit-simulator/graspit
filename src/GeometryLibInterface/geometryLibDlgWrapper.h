/*
	The purpose of this wrapper is to only include the dialog header file is the Primitive code actually exists.
	This is because the primitiveDlg actually links to that code
*/

#ifdef GEOMETRY_LIB
#include "GeometryLibInterface/geometryLibDlg.h"
#endif