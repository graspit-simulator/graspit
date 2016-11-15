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
// Author(s): Andrew T. Miller and Matei T. Ciocarlie
//
// $Id: body.cpp,v 1.81 2010/08/10 17:23:59 cmatei Exp $
//
//######################################################################

/*! \file
  \brief Implements the body class hierarchy.
*/

#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <QFile>
#include <QString>
#include <QTextStream>

#include "body.h"
#include "collisionInterface.h"
#include "bBox.h"
#include "triangle.h"
#include "world.h"
#include "robot.h"
#include "joint.h"
#include "dynJoint.h"
#include "ivmgr.h"
#include "contact.h"
#include "graspitGUI.h"
#include "tinyxml.h"

#ifdef PLY_READER
#include "mesh_loader.h"
#endif

extern "C" {
#include "maxdet.h"
}
#include "mytools.h"

#include <Inventor/SoDB.h>
#include <Inventor/SoInput.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLMaterial.h>
#include <Inventor/VRMLnodes/SoVRMLAppearance.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#include <Inventor/nodes/SoPickStyle.h>
#include <Inventor/nodes/SoSwitch.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/draggers/SoRotateDiscDragger.h>
#include <Inventor/nodekits/SoWrapperKit.h>
#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>

#include <Inventor/actions/SoWriteAction.h>

#ifdef USE_DMALLOC
#include "dmalloc.h"
#endif

//#define GRASPITDBG
#include "debug.h"

#define SQR(x) ((x)*(x))
#define CUBE(x) ((x)*(x)*(x))
#define AXES_SCALE 100.0

const float Body::CONE_HEIGHT = 20.0;
double DynamicBody::defaultMass = 300.0;

/*!
  Constructs an empty body with its material set to invalid, and the
  geometry pointer set to NULL.  Use \c load() to initialize the body from
  a model file.
*/
Body::Body(World *w,const char *name) : WorldElement(w,name)
{
	//defaults:
	mIsElastic = false;
	youngMod = -1;
	
	material = -1;
	numContacts=0;
	showFC = false;
	showVC = false;
	IVGeomRoot = NULL; IVTran = NULL; IVMat = NULL; IVContactIndicators=NULL;
        IVScaleTran = NULL;
        IVOffsetTran = NULL;
	IVBVRoot = NULL;
#ifdef GEOMETRY_LIB
	IVPrimitiveRoot = NULL;
#endif
	mUsesFlock = false;
	mBirdNumber = 0;
	mRenderGeometry = true;
	mGeometryFilename = "none";

	initializeIV();
}

/*!
  Copy constructor remains protected (should not be called by user)
*/
Body::Body(const Body &b) : WorldElement(b)
{
  mIsElastic = b.mIsElastic;
  material = b.material;
  youngMod = b.youngMod;
  showFC = b.showFC;
  mUsesFlock = b.mUsesFlock;
  mRenderGeometry = true;
  Tran = b.Tran;

  IVRoot = b.IVRoot;
  IVTran = b.IVTran;
  IVContactIndicators = b.IVContactIndicators;
#ifdef GEOMETRY_LIB
  IVPrimitiveRoot = b.IVPrimitiveRoot;
#endif
  IVBVRoot = b.IVBVRoot;
  IVGeomRoot = b.IVGeomRoot;
  IVMat = b.IVMat;
  mGeometryFilename = b.mGeometryFilename;

  createAxesGeometry();
}

/*! Breaks all contacts; it is up to the world to also remove it from 
	the collision detection system or the scene graph. In general, do 
	not delete bodies directly, but use World::removeElement(...)instead 
	which can also delete them.
 */
Body::~Body()
{
  breakContacts();
  DBGP("Deleting Body: " << myName.latin1());
}

/*! Clones this body from an original. This means that the two bodies have independent
	transform, materials, properties, etc, BUT they share the scene graph geometry and
	the collision detection geometry. They can still be used for collision detection 
	independently as they can have different transforms.
	
	WARNING: cloning mechanism is incomplete. If the original is deleted, the clone 
	becomes unpredicateble and can cause the system to crash
*/
void 
Body::cloneFrom(const Body* original)
{
	mIsElastic = original->mIsElastic;
	youngMod = original->youngMod;
	material = original->material;

	//add virtual contacts
	virtualContactList.clear();
	std::list<Contact*> vc = original->getVirtualContacts();
	std::list<Contact*>::iterator it;
	for (it = vc.begin(); it!=vc.end(); it++) {
		VirtualContact *newContact = new VirtualContact( (VirtualContact*)(*it) );
		newContact->setBody(this);
		addVirtualContact(newContact);
	}

	setRenderGeometry( original->getRenderGeometry() );

	int numGeomChildren = original->getIVGeomRoot()->getNumChildren();
	//create a CLONE of all geometry 
	for (int i=0; i<numGeomChildren; i++) {
		IVGeomRoot->addChild( original->getIVGeomRoot()->getChild(i) );
	}

	addIVMat(true);

	//add a CLONE to collision detection
	cloneToIvc(original);
	setTran(original->getTran());
}

/*! Parses the root node of an XML structure containing information
	for a body. It looks for all the relevant properties. Some of
	the properties are optional and if they are not found, they will
	be set to default values. Others will cause a failure if they
	are not present.
*/
int
Body::loadFromXml(const TiXmlElement *root, QString rootPath)
{
	//material
	const TiXmlElement* element = findXmlElement(root,"material");
	QString valueStr;
	if(element == NULL){
		DBGA("No material type found; using default.");
		material = myWorld->getMaterialIdx("wood");
	} else {
		valueStr = element->GetText();
		if (!valueStr.isEmpty()) {
			material = myWorld->getMaterialIdx(valueStr);
			if (material==-1) {
				QTWARNING("invalid material type in body file");
				return FAILURE;
			}
		} else{
			DBGA("No material type found; using default.");
			material = myWorld->getMaterialIdx("wood");
		}
	}

	//Young's modulus
	element = findXmlElement(root,"youngs");
	if(element) {
		valueStr = element->GetText();
		youngMod = valueStr.toDouble();
		if (youngMod <= 0) {
			QTWARNING("invalid Young's modulus in body file");
			return FAILURE;
		}
		mIsElastic = true;			
	}

	//Use Flock of Birds
	element = findXmlElement(root,"useFlockOfBirds");
	if(element){
		valueStr = element->GetText();
		mBirdNumber = valueStr.toDouble();
		mUsesFlock = true;
		DBGA("Object using Flock of Birds sensor " << mBirdNumber);
	}

	//load the geometry itself
	element = findXmlElement(root,"geometryFile");
	if (!element) {
		QTWARNING("Geometry file information missing");
		return FAILURE;
	} else {
		//the path in the file is relative to the body xml file
		mGeometryFilename = element->GetText();
		valueStr = rootPath + mGeometryFilename;
		mGeometryFileType = element->Attribute("type");
		//inventor is default
		if (mGeometryFileType.isNull()||mGeometryFileType.isEmpty()) mGeometryFileType="Inventor";
		int result;
		if (mGeometryFileType=="Inventor") {
			result = loadGeometryIV(valueStr);
		} else if (mGeometryFileType=="off") {
			result = loadGeometryOFF(valueStr);
		} else if (mGeometryFileType=="ply") {
			result = loadGeometryPLY(valueStr);
		} else {
			DBGA("Unknown geometry file type: " << mGeometryFileType.latin1());
			result = FAILURE;
		}
		if (result == FAILURE) {
			QTWARNING("Failed to open geometry file: " + valueStr);
			return FAILURE;
		}
	}
	
	//scaling of the geometry
        IVScaleTran = new SoTransform;
        IVScaleTran->scaleFactor.setValue(1.0, 1.0, 1.0);
	element = findXmlElement(root,"geometryScaling");
	if (element) {
	  valueStr = element->GetText();
	  double scale = valueStr.toDouble();
	  if (scale <= 0) {
	    DBGA("Scale geometry: negative scale found");
	    return FAILURE;
	  }
	  IVScaleTran->scaleFactor.setValue(scale, scale, scale);
	}
        IVGeomRoot->insertChild(IVScaleTran, 0);

	//any offset to the geometry, inserted inside the geometry itself
	//note that the offset gets added before the scale, so the offset is
	//always expressed in graspit's units
        IVOffsetTran = new SoTransform;
        transf::IDENTITY.toSoTransform(IVOffsetTran);
	element = findXmlElement(root,"geometryOffset");
	if (element) {
	  const TiXmlElement* transformElement = findXmlElement(element,"transform");
	  if(!transformElement){
	    DBGA("Geometry offset field missing transform information");
	    return FAILURE;
	  }
	  transf offsetTran;
	  if(!getTransform(transformElement, offsetTran)){
	    DBGA("Geometry offset field: failed to parse transform");
	    return FAILURE;
	  }
	  offsetTran.toSoTransform(IVOffsetTran);
	}
        IVGeomRoot->insertChild(IVOffsetTran, 0);

	return SUCCESS;
}

int
Body::saveToXml(QTextStream& xml){
	xml<<"\t\t\t<material>"<<myWorld->getMaterialName(material).latin1()<<"</material>"<<endl;
	if(youngMod>0)
		xml<<"\t\t\t<youngs>"<<youngMod<<"</youngs>"<<endl;
	if(mUsesFlock){
		xml<<"\t\t\t<useFlockOfBirds>"<<mBirdNumber<<"</useFlockOfBirds>"<<endl;
	}
	xml<<"\t\t\t<geometryFile type = \""<<mGeometryFileType.latin1()<<"\">"
		<<mGeometryFilename.latin1()<<"</geometryFile>"<<endl;
	return SUCCESS;
}


/*! Opens and loads a body information file.  \a filename is the complete path to
    the body file, which should be in XML format (with a .xml extension) and can 
    contain a number of Graspit-specific properties as XML tags, as well as a 
    pointer to a different file which contains the geometry itself.

    Alternatively, \a filename can also point directly to the geometry file.The
    type of the file will be detected by its extension (.iv, .wrl, .off or .ply)
    in which case GraspIt will use the default values for all the properties
    that normally come from the XML tags.
*/
int 
Body::load(const QString &filename)
{
	QString fileType = filename.section('.',-1,-1);
	QString xmlFilename;
	if (fileType == "xml"){
		//the file itself is XML
		xmlFilename = filename;
	} else {
		//file is geometry; use default XML file
		DBGA("Loading geometry file with boilerplate XML file");
		xmlFilename = QString(getenv("GRASPIT")) + QString("/models/objects/default.xml");
	}


	myFilename = relativePath(filename, getenv("GRASPIT"));
	if (myName.isEmpty() || myName == "unnamed") {
		setName(filename.section('/',-1).section('.',0,0));
	}

	//load the graspit specific information in XML format
	TiXmlDocument doc(xmlFilename);
	if(doc.LoadFile()==false){
		QTWARNING("Could not open " + xmlFilename);
		return FAILURE;
	}
	if (fileType != "xml") {
		//make geometry point at the right thing
		QString relFilename = relativePath(filename, QString(getenv("GRASPIT")) + 
						   QString("/models/objects/"));
		TiXmlElement * element = new TiXmlElement("geometryFile");
		if (fileType=="iv" || fileType=="wrl") {
			element->SetAttribute("type","Inventor");
		} else if (fileType=="off") {
			element->SetAttribute("type","off");
		} else if (fileType=="ply") {
			element->SetAttribute("type","ply");
		}
		TiXmlText * text = new TiXmlText( relFilename );
		element->LinkEndChild(text);
		doc.RootElement()->LinkEndChild(element);		
	}
	//the root path is the directory in which the xml file is placed
	QString root = xmlFilename.section('/',0,-2,QString::SectionIncludeTrailingSep);
	if (loadFromXml(doc.RootElement(), root) != SUCCESS) {
		return FAILURE;
	}
	//add material for controlling transparency
	addIVMat();
	return SUCCESS;
}

/*! Loads only the geometry part of this object, without any other 
    GraspIt specific information such as mass, material etc. The file
    must be in a format that is readable by Coin, which for now means
    either Inventor (.iv) or VRML.
*/
int
Body::loadGeometryIV(const QString &filename)
{
	SoInput myInput;
	if (!myInput.openFile(filename.latin1())) {
		QTWARNING("Could not open Inventor file " + filename);
		return FAILURE;
	}

	//we will read the geometry from the file
	SoGroup *fileGeomRoot;
	if (myInput.isFileVRML2()) {
		fileGeomRoot = SoDB::readAllVRML(&myInput);
	} else {
		fileGeomRoot = SoDB::readAll(&myInput);
	}
	myInput.closeFile();
	if (fileGeomRoot == NULL) {
		QTWARNING("A problem occurred while reading Inventor file" + filename);
		return FAILURE;
	}
	//and add it to scene graph
	IVGeomRoot->addChild(fileGeomRoot);
	return SUCCESS;
}


using std::string;
using std::getline;
using std::ifstream;
using std::istringstream;
using std::pow;
using std::sqrt;
using std::vector;


//! Helper for loadGeometryOff
/*! Strips off leading whitespace and comments */
bool GetOffLine(ifstream* file, istringstream* line)
{
  string buffer;
  if (!file->good()) return false;
  getline(*file, buffer);
  // remove comments and leading whitespace
  buffer = buffer.substr(buffer.find_first_not_of(" \t\n\f\r"), 
                         buffer.find_first_of("#"));
  if (!buffer.empty()) {
    line->clear();
    line->str(buffer);
    return true;
  }
  return GetOffLine(file, line);
}

//! Helper for loadGeometryOFF
int OFFReadFailure() {
  DBGA("OFF reader failure");
  return FAILURE;
}


/*! Loads the geometry of this object from an .off file. This was 
	primarily created for loading models from the Princeton Shape 
	Benchmark, allowing GraspIt to interact with the Columbia Grasp 
	Database.
*/
int
Body::loadGeometryOFF(const QString& filename) {
  ifstream file(filename.toStdString().c_str());
  istringstream line;

  // Skip the first line, which is always just "OFF"
  if (!GetOffLine(&file, &line)) return OFFReadFailure();

  // The header is the first line that isn't a comment (comments start with #)
  // The header contains num_vertices and num_faces
  if (!GetOffLine(&file, &line)) return OFFReadFailure();
  long num_vertices, num_faces;
  line >> num_vertices >> num_faces;
  if (line.fail()) return OFFReadFailure();
    
  SbVec3f* vertices = new SbVec3f[num_vertices];
  std::vector<int32_t> face_indices;
  // Read vertices
  for (long vertex = 0; vertex < num_vertices; ++vertex) {
    if (!GetOffLine(&file, &line)) return OFFReadFailure();
    float x, y, z;
    line >> x >> y >> z;
    if (line.fail()) return OFFReadFailure();
 	  vertices[vertex].setValue(x, y, z); 
  }
  // Read faces into a vector
  for (long face = 0; face < num_faces; ++face) {
    if (!GetOffLine(&file, &line)) return OFFReadFailure();
    int num_points, vertex_index;
    line >> num_points;
    // Read the points
    for (int point = 0; point < num_points; ++point) {
      line >> vertex_index;
      face_indices.push_back(vertex_index);
      // Triangulate the face as we go with a triangle fan and save the 
    }
    if (line.fail()) return OFFReadFailure();
    face_indices.push_back(SO_END_FACE_INDEX);
  }

	// Put everything into Coin geometry
	SoCoordinate3* coords = new SoCoordinate3;
	coords->point.setValues(0, num_vertices, vertices);
	SoIndexedFaceSet* ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, face_indices.size(), &(face_indices[0]));
	this->IVGeomRoot->addChild(coords);
	this->IVGeomRoot->addChild(ifs);

	DBGA("OFF reader success");
	return SUCCESS;
}

/*! Loads the geometry of this body from a .ply file.
    Uses ply loading code from ROS by Willow Garage, which in turn
    uses code from Greg Turk, Georgia Institute of Technology. PLY
    loading seems to be fairly complex, as the ply format is very 
    extensible. Right now, this will only load vertices and triangles
    and nothing else, not even other types of faces. Could be extended
    in the future.
 */
int
Body::loadGeometryPLY(const QString& filename)
{
#ifndef PLY_READER
	DBGA("PLY Reader not installed; can not read file");
	return FAILURE;
#else
	PLYModelLoader loader;
	std::vector<position> vertices;
	std::vector<int> triangles;
	if (loader.readFromFile(filename.toStdString(), vertices, triangles)) {
		DBGA("PLY loader error");
		return FAILURE;
	}
        return loadGeometryMemory(vertices, triangles);
#endif
}

int 
Body::loadGeometryMemory(const std::vector<position> &vertices, const std::vector<int> &triangles)
{
	int num_vertices = vertices.size();
	SbVec3f* sbVertices = new SbVec3f[num_vertices];
	for (size_t i=0; i<vertices.size(); i++) {
		sbVertices[i].setValue(vertices[i].x(), vertices[i].y(), vertices[i].z());
	}
	SoCoordinate3* coords = new SoCoordinate3;
	coords->point.setValues(0, num_vertices, sbVertices);
	std::vector<int32_t> face_indices;
	for (size_t i=0; i<triangles.size(); i++) {
		face_indices.push_back(triangles[i]);
		if (i%3==2) {
			face_indices.push_back(SO_END_FACE_INDEX);
		}
	}					      
	SoIndexedFaceSet* ifs = new SoIndexedFaceSet;
	ifs->coordIndex.setValues(0, face_indices.size(), &(face_indices[0]));
	this->IVGeomRoot->addChild(coords);
	this->IVGeomRoot->addChild(ifs);
	return SUCCESS;
}

/*! After the geometry has been set, this function adds a new Material 
	node after any Material node already present so we can change the 
	transparency of this object. Does not work on bodies loaded from 
	VRML files, as they have different types of material nodes. I have
	tried also searching for SoVRMLMaterial nodes, but the search fails
	even if those nodes exist; I suppose that is a bug in Coin.
*/
void 
Body::addIVMat(bool clone)
{
	IVMat = new SoMaterial;
	IVMat->diffuseColor.setIgnored(true);
	IVMat->ambientColor.setIgnored(true);
	IVMat->specularColor.setIgnored(true);
	IVMat->emissiveColor.setIgnored(true);
	IVMat->shininess.setIgnored(true);

	if (clone) {
		//clone's IVMat really does nothing except die with the clone
		IVGeomRoot->addChild(IVMat);		
	} else {
		SoSearchAction *sa = new SoSearchAction;
		sa->setInterest(SoSearchAction::ALL);
		sa->setType(SoMaterial::getClassTypeId());
		sa->apply(IVGeomRoot);
		
		if (sa->getPaths().getLength() == 0) {
			IVGeomRoot->insertChild(IVMat,0);
		} else {
			for (int i=0; i<sa->getPaths().getLength(); i++) {
				SoGroup *g = (SoGroup *)sa->getPaths()[i]->getNodeFromTail(1);
				if (((SoMaterial *)sa->getPaths()[i]->getTail())->transparency[0] == 0.0f) {
					g->insertChild(IVMat,sa->getPaths()[i]->getIndexFromTail(0)+1);	
				}
			}
		}
		delete sa;
	}
	/*
	FILE *fp = fopen("foo.txt","w");
	SoOutput *so  = new SoOutput;
	so->setFilePointer(fp);
	SoWriteAction *swa = new SoWriteAction(so);
	swa->apply(IVGeomRoot);
	delete swa;
	delete so;
	fclose(fp);
	*/
}

/*! Adds the body to the world's collision detection system
*/
void 
Body::addToIvc(bool ExpectEmpty)
{
	myWorld->getCollisionInterface()->addBody(this, ExpectEmpty);
	myWorld->getCollisionInterface()->setBodyTransform(this, Tran);
}

/*!	Clones the original's body geometry for the world
	collision detection system. The new body only gets 
	its own IVC transform.
*/
void 
Body::cloneToIvc(const Body *original)
{
	myWorld->getCollisionInterface()->cloneBody(this, original);
	myWorld->getCollisionInterface()->setBodyTransform(this, Tran);
}

/*! Collision geometry gets updated which could be slow.
*/
void Body::setGeometryScaling(double x, double y, double z)
{
  if (x<=0 || y<=0 || z<=0)
  {
    DBGA("Scale geometry: negative or zero scale found");
    return;
  }
  IVScaleTran->scaleFactor.setValue(x, y, z);
  myWorld->getCollisionInterface()->updateBodyGeometry(this);
}

/*! Collision geometry gets updated which could be slow.
*/
void Body::setGeometryOffset(transf tr)
{
  tr.toSoTransform(IVOffsetTran);
  myWorld->getCollisionInterface()->updateBodyGeometry(this);
}

void 
Body::setDefaultViewingParameters()
{
	showFC = false;
	showVC = false;
	setTransparency(0.0);	
}

/*! Initialized the empty scene graph structure that we will use
	in the future to render this body
*/
void 
Body::initializeIV()
{
	IVRoot = new SoSeparator;
	IVTran = new SoTransform;
	IVRoot->insertChild(IVTran,0);

	//axes get added at the beginning, so they are not affected
	//by what goes on in the other groups
	createAxesGeometry();

	IVContactIndicators = new SoSeparator;
	IVRoot->addChild(IVContactIndicators);

#ifdef GEOMETRY_LIB
	IVPrimitiveRoot = new SoSeparator;
	IVRoot->addChild(IVPrimitiveRoot);
#endif

	IVBVRoot = new SoSeparator;
	IVRoot->addChild(IVBVRoot);

	IVGeomRoot = new SoSeparator;
	IVRoot->addChild(IVGeomRoot);
}

/*! Shows a bounding box hierarchy for this body. Used for 
	debug purposes.
*/
void
Body::setBVGeometry(const std::vector<BoundingBox> &bvs)
{
	IVBVRoot->removeAllChildren();
	int mark = 0;
	for (int i=0; i<(int)bvs.size(); i++) {
		SoSeparator *bvSep = new SoSeparator;

		SoMaterial *bvMat = new SoMaterial;
		bvSep->addChild(bvMat);
		float r,g,b;
		//random colors
		r = ((float)rand()) / RAND_MAX;
		g = ((float)rand()) / RAND_MAX;
		b = ((float)rand()) / RAND_MAX;

		//mark collisions	
		if (bvs[i].mMark) {
			mark++;
			r = 0.8f; g=0.0f; b=0.0f;
		} else {
			r = g = b = 0.5f;
		}
		
		bvMat->diffuseColor = SbColor(r,g,b);
		bvMat->ambientColor = SbColor(r,g,b);
		bvMat->transparency = 0.5;

		SoTransform* bvTran = new SoTransform;
		bvs[i].getTran().toSoTransform(bvTran);
		bvSep->addChild(bvTran);

		
		//a single cube for the entire box
		SoCube *bvBox = new SoCube;
		bvBox->width = 2 * bvs[i].halfSize.x();
		bvBox->height = 2 * bvs[i].halfSize.y();
		bvBox->depth = 2 * bvs[i].halfSize.z();
		bvSep->addChild(bvBox);
		

		//2 cubes so we also see separarion plane
		/*
		SoCube *bvBox = new SoCube;
		bvBox->width =		bvs[i].halfSize.x();
		bvBox->height = 2 * bvs[i].halfSize.y();
		bvBox->depth = 2 * bvs[i].halfSize.z();
		SoTransform *halfCubeTran = new SoTransform;
		halfCubeTran->translation.setValue(bvs[i].halfSize.x()/2.0, 0.0, 0.0);
		bvSep->addChild(halfCubeTran);
		bvSep->addChild(bvBox);
		halfCubeTran = new SoTransform;
		halfCubeTran->translation.setValue(-bvs[i].halfSize.x(), 0.0, 0.0);
		bvSep->addChild(halfCubeTran);
		bvSep->addChild(bvBox);
		*/

		IVBVRoot->addChild(bvSep);
	}
	DBGA("Setting bv geom: " << bvs.size() << " boxes. Marked: " << mark);
}

/*!
  Returns the current transparency value for the body (between 0 and 1).
  \sa setTransparency()
*/
float
Body::getTransparency() const
{
  return IVMat->transparency[0];
}  
  

/*!
  Set the current transparency value for the body.
  \a t is a value between 0 and 1, where 0 is opaque, 1 is transparent.
  \sa getTransparency()
*/
void
Body::setTransparency(float t)
{
  IVMat->transparency = t;
}

  
/*!
  Set the current material of the body to mat
  \sa getMaterial()
*/
void
Body::setMaterial(int mat)
{
  std::list<Contact *>::iterator cp;

  material = mat;
  if (showFC || showVC) IVContactIndicators->removeAllChildren();

  for (cp=contactList.begin();cp!=contactList.end();cp++) {
    (*cp)->updateCof();
    (*cp)->getMate()->updateCof();
    (*cp)->getBody2()->redrawFrictionCones();
  }
  redrawFrictionCones();
}


/*!
  Sets whether friction cones should be shown for this body.
*/
void
Body::showFrictionCones(bool on, int vc)
{
  showFC = on;
  if (vc==1) showVC = true;
  else if (vc==2) showVC = false;
  redrawFrictionCones();
}


/*!
  Recomputes all the friction cones on the body
*/
void
Body::redrawFrictionCones()
{
  std::list<Contact *>::iterator cp;

  IVContactIndicators->removeAllChildren();
  if (showFC) {
    for (cp=contactList.begin();cp!=contactList.end();cp++)
		IVContactIndicators->addChild( (*cp)->getVisualIndicator() );
  }
  if (showVC) {
	for (cp = virtualContactList.begin(); cp!=virtualContactList.end(); cp++)
		IVContactIndicators->addChild( ( (VirtualContact*)(*cp) )->getVisualIndicator() );
  }
}

/*! Sets whether a change of this body's transform should automatically
	trigger a redraw. This seems to not always work...
*/
void
Body::setRenderGeometry(bool s)
{
	assert(IVTran);
	mRenderGeometry = s;
	if (!s) {
//		IVRoot->enableNotify(false);
//		IVTran->enableNotify(false);
//		IVMat->enableNotify(false);
//		IVGeomRoot->enableNotify(false);
//		IVContactIndicators->enableNotify(false);
		IVTran->translation.enableNotify(false);
		IVTran->rotation.enableNotify(false);
	} else {
//		IVRoot->enableNotify(true);
//		IVTran->enableNotify(true);
//		IVMat->enableNotify(true);
//		IVGeomRoot->enableNotify(true);
//		IVContactIndicators->enableNotify(true);
		IVTran->translation.enableNotify(true);
		IVTran->rotation.enableNotify(true);
	}
	
}

/*!
  Sets the current world pose of the body to \a tr.  Collisions are not
  checked.
*/
int
Body::setTran(transf const &tr)
{
	if (tr == Tran) return SUCCESS;
	breakContacts();

	if (!myWorld->wasModified() && tr != Tran) {
		myWorld->setModified();
	}
	Tran = tr;
	myWorld->getCollisionInterface()->setBodyTransform(this, Tran);
	if (IVTran) {
		Tran.toSoTransform(IVTran);
	}
	return SUCCESS;
}

 
/*!
  Given a motion relative to body coordinates, this determines whether
  the current contacts allow that motion.
*/
bool
Body::contactsPreventMotion(const transf& motion) const
{
  std::list<Contact *>::iterator cp;
  std::list<Contact *> contactList;

  contactList = getContacts();
  for (cp=contactList.begin();cp!=contactList.end();cp++) {
    if ((*cp)->preventsMotion(motion)) {
      return true;
    }
  }
  return false;
}

/*!
  Breaks all contacts on the body, deleting entire contact list, and removes
  all friction cones if necessary. Also clears the list of contacts from the
  previous time step.
*/
void
Body::breakContacts()
{
	std::list<Contact *>::iterator cp;

	if (!contactList.empty()) {
		setContactsChanged();
		for (cp=contactList.begin();cp!=contactList.end();cp++) {
			delete *cp; *cp = NULL;
		}
		contactList.clear();
	}
	numContacts = 0;

	for (cp = prevContactList.begin(); cp!=prevContactList.end(); cp++) {
		if ( (*cp)->getMate() != NULL ) {
			(*cp)->getMate()->setMate(NULL);
		}
		(*cp)->setMate(NULL);
		delete *cp; *cp = NULL;
	}
	prevContactList.clear();

	if (showFC)
		IVContactIndicators->removeAllChildren();
}

/*! Load in all virtual contacts from file fn*/
int
Body::loadContactData(QString fn)
{
	FILE *fp = fopen(fn.latin1(), "r");
	if (!fp) {
		fprintf(stderr,"Could not open filename %s\n",fn.latin1());
		return FAILURE;
	}
	int numContacts;
	VirtualContactOnObject *newContact;
	if(fscanf(fp,"%d",&numContacts) <= 0)
		return FAILURE;

	breakVirtualContacts();
	for (int i=0; i<numContacts; i++) {
		newContact = new VirtualContactOnObject();
		newContact->readFromFile(fp);
		newContact->setBody( this );
		((Contact*)newContact)->computeWrenches();
		addVirtualContact( newContact );//visualize the contact just read in
	}
	fclose(fp);
	return SUCCESS;
}
/*! Removes all virtual contacts. This only removes the virtual contacts.
*/
void
Body::breakVirtualContacts()
{
	std::list<Contact *>::iterator cp;
	for (cp = virtualContactList.begin(); cp!= virtualContactList.end(); cp++) {
		delete *cp;
	}
	//this deletes all contact indicators, should be fixed..
	if (showVC) {
		IVContactIndicators->removeAllChildren();
	}
	virtualContactList.clear();
}

/*!
  Moves all contacts to the prevContactList and clears the contactList for new contacts
  Old contacts from prevContactList are deleted.
*/
void
Body::resetContactList()
{
	std::list<Contact *>::iterator cp;
	for (cp = prevContactList.begin(); cp!=prevContactList.end(); cp++) {
		if ( (*cp)->getMate() != NULL ) {
			(*cp)->getMate()->setMate(NULL);
		}
		(*cp)->setMate(NULL);
		delete *cp; *cp = NULL;
	}
	prevContactList.clear();
	if (!contactList.empty()) {
		setContactsChanged();
		for (cp=contactList.begin();cp!=contactList.end();cp++) {
			prevContactList.push_back(*cp);
		}
		contactList.clear();
	}
	numContacts = 0;
	if (showFC)
		IVContactIndicators->removeAllChildren();
}

/*! 
  Adds contact \a c to the body's contact list.  Assumes the contact is not
  already in the list.
  \sa removeContact()
*/
void
Body::addContact(Contact *c)
{
	setContactsChanged();
	contactList.push_back(c);
	numContacts++;
	if (showFC) {
		assert(IVContactIndicators->getNumChildren() > numContacts-2);
		IVContactIndicators->insertChild( c->getVisualIndicator(), numContacts-1 );
	}
}

/*! The number of contacts on this body. If \a b is not null, it only counts
	contacts against \a b. If it is null, is returns all contacts on this body,
	regardless of who they are against.
*/
int 
Body::getNumContacts(Body *b) const
{
	if (!b) return numContacts;
	int c = 0;
	std::list<Contact*>::const_iterator it;
	for (it = contactList.begin(); it!=contactList.end(); it++) {
		if ( (*it)->getBody2() == b) c++;
	}
	return c;
}

/*! Returns a list of the contacts on this body. if \a b is not NULL, it returns
	only contacts against the body \a b. If \a b is NULL, it returns all the 
	contacts.
*/
std::list<Contact*>
Body::getContacts(Body *b) const
{
	if (!b) return contactList;
	std::list<Contact*> contacts;
	std::list<Contact*>::const_iterator it;
	for (it = contactList.begin(); it!=contactList.end(); it++) {
		if ( (*it)->getBody2() == b) contacts.push_back(*it);
	}
	return contacts;
}

/*! Adds a virtual contacts to this body */
void
Body::addVirtualContact(Contact *c)
{
	setContactsChanged();
	virtualContactList.push_back(c);
	if (showVC)
		IVContactIndicators->addChild( c->getVisualIndicator() );
}

/*!
  Checks if this contact inherits some other contact from the prevContactList.
  The check looks of contact points are close, normals agree and the second 
  body is the same. If inheritance is found, some properites of the contact 
  are passed from the previous contact to this one.
*/
Contact*
Body::checkContactInheritance(Contact *c)
{
	std::list<Contact *>::iterator cp;
	bool inheritance = false;
	for (cp = prevContactList.begin(); cp != prevContactList.end(); cp++) {
		if ( (*cp)->getBody1() != c->getBody1() )
			continue;
		if ( (*cp)->getBody2() != c->getBody2() )
			continue;
		vec3 d = (*cp)->getPosition() - c->getPosition();
		if (d.len() > Contact::INHERITANCE_THRESHOLD )
			continue;
		vec3 n1 = (*cp)->getNormal();
		vec3 n2 = c->getNormal();
		double theta = n1 % n2;
		if ( theta < Contact::INHERITANCE_ANGULAR_THRESHOLD )
			continue;
		inheritance = true;
		break;
	}
	if (!inheritance)
		return NULL;
	return (*cp);
}


/*!
  Removes contact c from the body's contact list.  Assumes the contact is in the list.
  \sa addContact() 
 */
void
Body::removeContact(Contact *c)
{
  int i;
  std::list<Contact *>::iterator cp;

  setContactsChanged();

  if (showFC) {
    for (cp=contactList.begin(),i=0;cp!=contactList.end();cp++,i++)
      if (*cp == c) {
	    contactList.erase(cp);
	    break;
      }
    IVContactIndicators->removeChild(i);
  }
  else contactList.remove(c);

  delete c;
  numContacts--;
}

/*!
	Removes a contact c from the prevContactList
*/
void Body::removePrevContact(Contact *c)
{
	prevContactList.remove(c);
	c->setMate(NULL);
	delete c;
}

/*!
  Output method for writing body data to a text world configuration file.
*/
QTextStream&
operator<<(QTextStream &os, const Body &b)
{
  os << b.myFilename << endl;
  os << b.myWorld->getMaterialName(b.material) << endl;
  return os;
}


/*! Helper callback for generating list of body triangles */
void addTriangleCallBack(void* info, SoCallbackAction * action,
						 const SoPrimitiveVertex * v1, 
						 const SoPrimitiveVertex * v2, 
						 const SoPrimitiveVertex * v3)
{
	std::vector<Triangle> *triangles = (std::vector<Triangle>*) info;

	SbVec3f p1, p2, p3;
	SbMatrix mm = action->getModelMatrix();

	// Transform vertices (remember vertices are in the object space coordinates for each triangle)
	mm.multVecMatrix( v1->getPoint(), p1 );
	mm.multVecMatrix( v2->getPoint(), p2 );
	mm.multVecMatrix( v3->getPoint(), p3 );

	// Don't add degenerate triangles!
	if ((p1 == p2) || (p2 == p3) || (p1 == p3)) return;

	position nv1(p1[0], p1[1], p1[2]);
	position nv2(p2[0], p2[1], p2[2]);
	position nv3(p3[0], p3[1], p3[2]);
	Triangle newTri(nv1,nv2,nv3);
	triangles->push_back( newTri );
}

/*! Helper callback for generating list of body vertices */
void addVertexCallBack(void* info, SoCallbackAction * action, const SoPrimitiveVertex * v1)
{
	std::vector<position> *vertices = (std::vector<position>*) info;
	SbVec3f p1;
	SbMatrix mm = action->getModelMatrix();
	// Transform vertex (remember vertices are in the object space coordinates for each triangle)
	mm.multVecMatrix( v1->getPoint(), p1 );
	position nv1(p1[0], p1[1], p1[2]);
	vertices->push_back( nv1 );
}

/*! Helper callback for generating list of body vertices */
void addVerticesFromTriangleCallBack(void* info, SoCallbackAction * action,
									 const SoPrimitiveVertex * v1, 
									 const SoPrimitiveVertex * v2, 
									 const SoPrimitiveVertex * v3)
{
	std::vector<position> *vertices = (std::vector<position>*) info;
	SbVec3f p1, p2, p3;
	SbMatrix mm = action->getModelMatrix();
	// Transform vertices (remember vertices are in the object space coordinates for each triangle)
	mm.multVecMatrix( v1->getPoint(), p1 );
	mm.multVecMatrix( v2->getPoint(), p2 );
	mm.multVecMatrix( v3->getPoint(), p3 );
	vertices->push_back(position(p1[0], p1[1], p1[2]));
	vertices->push_back(position(p2[0], p2[1], p2[2]));
	vertices->push_back(position(p3[0], p3[1], p3[2]));
}

/*! Returns all the triangle that make up the geometry of this body */
void
Body::getGeometryTriangles(std::vector<Triangle> *triangles) const
{
	SoCallbackAction ca;
	ca.addTriangleCallback(SoShape::getClassTypeId(), addTriangleCallBack, triangles);
	ca.apply(getIVGeomRoot());
}

/*!	Returns all the vertices that make up the geometry of this body. 
	This function will return duplicates, as vertices are reported once 
	for each triangle that they are part of
*/
void
Body::getGeometryVertices(std::vector<position> *vertices) const
{
	SoCallbackAction ca;
	//unfortunately, this does not work as triangle vertices are not considered points by Coin
	//ca.addPointCallback(SoShape::getClassTypeId(), addVertexCallBack, vertices);
	//we have to use a triangle callback which leads to duplication
	ca.addTriangleCallback(SoShape::getClassTypeId(), addVerticesFromTriangleCallBack, vertices);
	ca.apply(getIVGeomRoot());
}

/*! Creates the geometry of the axes which show this body's local
	coordinate system. The axes are usually shown centered at the c.o.m
*/
void Body::createAxesGeometry()
{  
  IVWorstCase = new SoSeparator;  
  IVAxes = new SoSwitch;  
  if (graspItGUI) {
    SoSeparator *axesSep = new SoSeparator;
    axesTranToCOG = new SoTranslation;
    axesTranToCOG->translation.setValue(0,0,0);
    axesSep->addChild(axesTranToCOG);
    axesSep->addChild(IVWorstCase);
    
    axesScale = new SoScale;
	axesScale->scaleFactor = SbVec3f(1,1,1);
    axesSep->addChild(axesScale);
    axesSep->addChild(graspItGUI->getIVmgr()->getPointers()->getChild(2));
    IVAxes->addChild(axesSep);
  }
  if (!IVRoot) IVRoot = new SoSeparator;

  IVRoot->addChild(IVAxes);
}

///////////////////////////////////////////////////////////////////////////////
//                              DynamicBody
///////////////////////////////////////////////////////////////////////////////

/*!
  If there is a dynamic joint connected to this body, it is deleted before the
  body is destroyed.
*/
DynamicBody::~DynamicBody()
{
  if (dynJoint) delete dynJoint;
}

void
DynamicBody::init()
{
  maxRadius = 0.0; mass = 0.0;
  showAx = showDynCF = false;
  fixed = false; dynJoint=NULL; dynamicsComputedFlag = false;
  useDynamics = true;
  bbox_min = vec3(-1e+6,-1e+6,-1e+6);
  bbox_max = vec3(1e+6,1e+6,1e+6);
  CoG.set(0.0,0.0,0.0);
  for(int i=0; i<9; i++) {
	  I[i] = 0.0;
  }
  resetDynamics();
}

/*! Sets acceleration and velocity to 0 and the 7-dimensional
	state vector to match the current transform (which includes
	the position of the center of gravity.
*/
void
DynamicBody::resetDynamics()
{
  resetExtWrenchAcc();
  for (int i=0; i<6; i++) {
    a[i] = 0.0;
    v[i] = 0.0;
  }
  Quaternion quat = Tran.rotation();
  vec3 cogOffset = quat * (CoG-position::ORIGIN);
  q[0] = Tran.translation().x()+cogOffset.x();
  q[1] = Tran.translation().y()+cogOffset.y();
  q[2] = Tran.translation().z()+cogOffset.z();
  q[3] = quat.w;
  q[4] = quat.x;
  q[5] = quat.y;
  q[6] = quat.z;
}

/*! Constructs an empty DynamicBody.  Use load() to initialize this class
	from a model file, or cloneFrom to initialize from a different 
	dynamic body.
*/
DynamicBody::DynamicBody(World *w, const char *name) : Body(w,name)
{
	init();
}

/*! 
  Creates a dynamic body from the basic body \a b.  Computes the mass
  properties automatically assuming a mass of \a m and uniform mass
  distribution.
*/
DynamicBody::DynamicBody(const Body &b, double m) : Body(b)
{
  init();
  position defCoG;
  double defI[9];
  computeDefaultMassProp(defCoG, defI);
  setMass(m);
  setCoG(defCoG);
  setInertiaMatrix(defI);
  setMaxRadius(computeDefaultMaxRadius());
  //we need the effects of get tran on dynamics
  setTran(b.getTran());
}

/*! Clones another dynamic body; all dynamic paramters are copied
	over.
*/
void
DynamicBody::cloneFrom(const DynamicBody *original)
{
	Body::cloneFrom(original);
	setMass(original->mass);
	setCoG(original->CoG);
	setInertiaMatrix(original->I);
	setMaxRadius(original->maxRadius);
}

int 
DynamicBody::loadFromXml(const TiXmlElement *root, QString rootPath)
{
	if (Body::loadFromXml(root, rootPath)==FAILURE) {
		return FAILURE;
	}
	double loadMass;
	bool overrideI, overrideCog;
	QString valueStr;

	const TiXmlElement* element = findXmlElement(root, "mass");
	if(element == NULL){
		loadMass = defaultMass;
		DBGA("Using default mass");
	} else{
		valueStr = element->GetText();
		loadMass = valueStr.toDouble();
		if (loadMass <= 0) {
			QTWARNING("invalid mass in dynamic body file: " + myFilename);
			return FAILURE;
		}
	}
	element = findXmlElement(root, "cog");
	position loadCog;
	if(element == NULL){
		overrideCog = false;
	} else{
		overrideCog = true;
		valueStr = element->GetText();
		valueStr = valueStr.simplifyWhiteSpace();
		QStringList l;
		l = QStringList::split(' ', valueStr.stripWhiteSpace());
		if(l.count()!=3){
			QTWARNING("Invalid Center of Gravity Input");
			return FAILURE;
		}		
		double x,y,z;
		x = l[0].toDouble();
		y = l[1].toDouble();
		z = l[2].toDouble();
		loadCog = position(x,y,z);
	}
	double loadI[9];
	element = findXmlElement(root, "inertia_matrix");
	if(element == NULL){//set default inertia matrix
		overrideI = false;
	} else{
		overrideI = true;
		valueStr = element->GetText();
		valueStr = valueStr.simplifyWhiteSpace();
		QStringList l;
		l = QStringList::split(' ', valueStr.stripWhiteSpace());
		if(l.count()!=9){ //error
			QTWARNING("Invalid Inertia Matrix Input");
			return FAILURE;
		}
		loadI[0]=l[0].toDouble();
		loadI[1]=l[1].toDouble();
		loadI[2]=l[2].toDouble();
		loadI[3]=l[3].toDouble();
		loadI[4]=l[4].toDouble();
		loadI[5]=l[5].toDouble();
		loadI[6]=l[6].toDouble();
		loadI[7]=l[7].toDouble();
		loadI[8]=l[8].toDouble();
	}
	if (!overrideI || !overrideCog) {
		position defaultCog;
		double defaultI[9];
		computeDefaultMassProp(defaultCog, defaultI);
		if (!overrideI) {
			memcpy(loadI, defaultI, 9*sizeof(double));
			DBGA("Using default inertia matrix");
		}
		if (!overrideCog) {
			loadCog = defaultCog;
			DBGA("Using default center of gravity");
		}
	}
	setMass(loadMass);
	setCoG(loadCog);
	setInertiaMatrix(loadI);
	setMaxRadius(computeDefaultMaxRadius());
	return SUCCESS;
}

/*! Computes both the center of gravity and the inertia matrix
	automatically, based on the geometry of the object and then
	sets them.
*/
void
DynamicBody::setDefaultDynamicParameters()
{
	position defaultCog;
	double defaultI[9];
	computeDefaultMassProp(defaultCog, defaultI);
	setCoG(defaultCog);
	setInertiaMatrix(defaultI);
}

int
DynamicBody::saveToXml(QTextStream &xml){
	if(Body::saveToXml(xml)!=SUCCESS)  return FAILURE;
	xml<<"\t\t\t<mass>"<<mass<<"</mass>"<<endl;
	xml<<"\t\t\t<cog>"<<CoG.x()<<" "<<CoG.y()<<" "<<CoG.z()<<"</cog>"<<endl;
	xml<<"\t\t\t<inertia_matrix>";
	for(int i=0;i<9;i++){
		xml<<I[i];
		if(i!=8)
			xml<<" ";
	}
	xml<<"</inertia_matrix>"<<endl;
	return SUCCESS;
}
/*! Also has to reset dynamics since the current state has to be changed
	to match the new CoG.
*/
void
DynamicBody::setCoG(const position &newCoG)
{
	CoG = newCoG;  
	resetDynamics();
	//use this to display axes at the CoG
	//axesTranToCOG->translation.setValue(CoG.x(), CoG.y(), CoG.z());

	//use this to display axes at body origin
	axesTranToCOG->translation.setValue(0,0,0);
}

/*! The max radius can be thought of as the largest distance from the center
	of gravity to the edge of the object.
*/
void
DynamicBody::setMaxRadius(double maxRad)
{
	maxRadius = maxRad;
	axesScale->scaleFactor = SbVec3f(maxRadius / AXES_SCALE, maxRadius / AXES_SCALE, maxRadius / AXES_SCALE);
}

void
DynamicBody::setInertiaMatrix(const double *newI)
{
	memcpy(I, newI, 9*sizeof(double));
}

double
DynamicBody::computeDefaultMaxRadius()
{
	std::vector<position> vertices;
	getGeometryVertices(&vertices);
	if (vertices.empty()) {
		DBGA("No vertices found when computing maxRadius!");
	}
	double maxRad = 0.0;
	for (int i=0; i<(int)vertices.size(); i++) {
		double tmpRadius = (CoG - vertices[i]).len();
		if (tmpRadius > maxRad) maxRad = tmpRadius;
	}
	return maxRad;
}

/*!
  Support routine for computing body mass properties.  This code is
  adapted from code written by Brian Mirtich.
*/
void
DynamicBody::compProjectionIntegrals(FACE &f, int A, int B)
{
  double a0, a1, da;
  double b0, b1, db;
  double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
  double a1_2, a1_3, b1_2, b1_3;
  double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
  double Cab, Kab, Caab, Kaab, Cabb, Kabb;
  int i;

  P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

  for (i = 0; i < 3; i++) {
    a0 = f.verts[i][A];
    b0 = f.verts[i][B];
    a1 = f.verts[(i+1) % 3][A];
    b1 = f.verts[(i+1) % 3][B];
    da = a1 - a0;
    db = b1 - b0;
    a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
    b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
    a1_2 = a1 * a1; a1_3 = a1_2 * a1; 
    b1_2 = b1 * b1; b1_3 = b1_2 * b1;

    C1 = a1 + a0;
    Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
    Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
    Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
    Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
    Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
    Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

    P1 += db*C1;
    Pa += db*Ca;
    Paa += db*Caa;
    Paaa += db*Caaa;
    Pb += da*Cb;
    Pbb += da*Cbb;
    Pbbb += da*Cbbb;
    Pab += db*(b1*Cab + b0*Kab);
    Paab += db*(b1*Caab + b0*Kaab);
    Pabb += da*(a1*Cabb + a0*Kabb);
  }

  P1 /= 2.0;
  Pa /= 6.0;
  Paa /= 12.0;
  Paaa /= 20.0;
  Pb /= -6.0;
  Pbb /= -12.0;
  Pbbb /= -20.0;
  Pab /= 24.0;
  Paab /= 60.0;
  Pabb /= -60.0;
}

/*!
  Support routine for computing body mass properties.  This code is
  adapted from code written by Brian Mirtich.
*/
void
DynamicBody::compFaceIntegrals(FACE &f,int A, int B, int C)
{
  double *n, w;
  double k1, k2, k3, k4;

  compProjectionIntegrals(f,A,B);

  w = f.w;
  n = f.norm;
  k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

  Fa = k1 * Pa;
  Fb = k1 * Pb;
  Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

  Faa = k1 * Paa;
  Fbb = k1 * Pbb;
  Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb
	 + w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

  Faaa = k1 * Paaa;
  Fbbb = k1 * Pbbb;
  Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab 
	   + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
	   + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
	   + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

  Faab = k1 * Paab;
  Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
  Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
	 + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
}

/*! Helper function that uses code by Brian Mirtich to compute the center of
	gravity and the inertia matrix for a list of triangles. Returns FAILURE
	if the computation fails, and some degenerate values are returned.
*/
int 
DynamicBody::computeDefaultInertiaMatrix(std::vector<Triangle> &triangles, double *defaultI)
{
  FACE f;
  double dx1,dy1,dz1,dx2,dy2,dz2;
  double nx, ny, nz,len;
  int i,A,B,C;
  double r[3], v1[3], v2[3], v3[3];
  double density;

  f.verts[0] = v1;
  f.verts[1] = v2;
  f.verts[2] = v3;

  // volume integrals
  double T0, T1[3], T2[3], TP[3];

  T0 = T1[0] = T1[1] = T1[2] 
     = T2[0] = T2[1] = T2[2] 
     = TP[0] = TP[1] = TP[2] = 0;


  for (i = 0; i < (int)triangles.size(); i++) {
	triangles[i].v1.get(v1);
	triangles[i].v2.get(v2);
	triangles[i].v3.get(v3);

	dx1 = f.verts[1][0] - f.verts[0][0];
    dy1 = f.verts[1][1] - f.verts[0][1];
    dz1 = f.verts[1][2] - f.verts[0][2];
    dx2 = f.verts[2][0] - f.verts[1][0];
    dy2 = f.verts[2][1] - f.verts[1][1];
    dz2 = f.verts[2][2] - f.verts[1][2];
    nx = dy1 * dz2 - dy2 * dz1;
    ny = dz1 * dx2 - dz2 * dx1;
    nz = dx1 * dy2 - dx2 * dy1;
    len = sqrt(nx * nx + ny * ny + nz * nz);

    f.norm[0] = nx / len;
    f.norm[1] = ny / len;
    f.norm[2] = nz / len;
    f.w = - f.norm[0] * f.verts[0][0]
           - f.norm[1] * f.verts[0][1]
           - f.norm[2] * f.verts[0][2];

    nx = fabs(f.norm[0]);
    ny = fabs(f.norm[1]);
    nz = fabs(f.norm[2]);
    if (nx > ny && nx > nz) C = 0;
    else C = (ny > nz) ? 1 : 2;
    A = (C + 1) % 3;
    B = (A + 1) % 3;

    compFaceIntegrals(f,A,B,C);

    T0 += f.norm[0] * ((A == 0) ? Fa : ((B == 0) ? Fb : Fc));

    T1[A] += f.norm[A] * Faa;
    T1[B] += f.norm[B] * Fbb;
    T1[C] += f.norm[C] * Fcc;
    T2[A] += f.norm[A] * Faaa;
    T2[B] += f.norm[B] * Fbbb;
    T2[C] += f.norm[C] * Fccc;
    TP[A] += f.norm[A] * Faab;
    TP[B] += f.norm[B] * Fbbc;
    TP[C] += f.norm[C] * Fcca;
  }

  T1[0] /= 2; T1[1] /= 2; T1[2] /= 2;
  T2[0] /= 3; T2[1] /= 3; T2[2] /= 3;
  TP[0] /= 2; TP[1] /= 2; TP[2] /= 2;

  r[0] = T1[0] / T0;
  r[1] = T1[1] / T0;
  r[2] = T1[2] / T0;

  DBGP("COM: " << r[0] << " " << r[1] << " " << r[2]);

  //default values in case of failure
  for (int i=0; i<9; i++) {
	  defaultI[i] = 0.0;
  }
  defaultI[0] = defaultI[3] = defaultI[6] = 1.0;

  //sanity checks
  if (r[0] != r[0]) return FAILURE;
  if (r[1] != r[1]) return FAILURE;
  if (r[2] != r[2]) return FAILURE;
  if (fabs(T0) < 1.0e-5) return FAILURE;
  if ( fabs(r[0]) > 5.0e2 || fabs(r[1]) > 5.0e2 || fabs(r[2]) > 5.0e2 ) {
	  return FAILURE;
  }

  //assume unity mass
  density = 1.0 / T0;

  /* compute inertia tensor */
  defaultI[0] = density * (T2[1] + T2[2]);
  defaultI[4] = density * (T2[2] + T2[0]);
  defaultI[8] = density * (T2[0] + T2[1]);
  defaultI[1] = defaultI[3] = - density * TP[0];
  defaultI[5] = defaultI[7] = - density * TP[1];
  defaultI[6] = defaultI[2] = - density * TP[2];

  /* translate inertia tensor to center of mass */
  defaultI[0] -= (r[1]*r[1] + r[2]*r[2]);
  defaultI[4] -= (r[2]*r[2] + r[0]*r[0]);
  defaultI[8] -= (r[0]*r[0] + r[1]*r[1]);
  defaultI[1] = defaultI[3] += r[0] * r[1]; 
  defaultI[5] = defaultI[7] += r[1] * r[2]; 
  defaultI[6] = defaultI[2] += r[2] * r[0]; 
  return SUCCESS;
}

/*! Helper function for computing cog and inertia matrix. Integrates
	a pointwise function over the surface of a triangle using 7-point
	Gaussian integration.
*/
template <class IntegrableFunction>
float Gaussian7Integrate(const Triangle& triangle, IntegrableFunction integrable_function) {
  // Setup
  static const float terms[8] = {
	0.333333333333333333333333333333333f,
	0.79742698535308732239802527616971f,
	0.10128650732345633880098736191512f,
	0.059715871789769820459117580973143f,
	0.47014206410511508977044120951345f,
	0.225f,
	0.12593918054482715259568394550018f,
	0.13239415278850618073764938783315f};
	  
  static const float barycentric_weights[7][3] = {
    {terms[0], terms[0], terms[0]}, 
    {terms[1], terms[2], terms[2]}, 
    {terms[2], terms[1], terms[2]}, 
    {terms[2], terms[2], terms[1]}, 
    {terms[3], terms[4], terms[4]}, 
    {terms[4], terms[3], terms[4]}, 
    {terms[4], terms[4], terms[3]}}; 
  static const float sample_weights[7] = {
    terms[5], terms[6], terms[6], terms[6], 
    terms[7], terms[7], terms[7]};
	// Calculate the integration sample points
  float integration_samples[7][3] = {{0}};
	const position* vertices[3] = {&(triangle.v1), &(triangle.v2), &(triangle.v3)};
	for(int sample_num = 0; sample_num < 7; ++sample_num) {
		float* sample = integration_samples[sample_num];
		const float* barycentric_weight = barycentric_weights[sample_num];
		for (int vertex_num = 0; vertex_num < 3; ++vertex_num) {
			const position& vertex = *(vertices[vertex_num]);
			for (int coord = 0; coord < 3; ++coord) {
				sample[coord] += vertex[coord] * barycentric_weight[vertex_num];
			}
		}
	}
  // Integration
  float result = 0;
  for (int sample = 0; sample < 7; ++sample){
    result += integrable_function(integration_samples[sample]) * sample_weights[sample];
   }
  return result * triangle.area();
} 

//! Integrable functor for computing triangle area
struct GetCoord {
  int coord;
  template <class T> float operator()(const T vertex) const { return vertex[coord]; }
};

//! Integrable functor for computing the covariance matrix
struct GetCovar {
  int coord_a, coord_b;
  float mean_a, mean_b;
  template <class T> float operator()(const T vertex) const { 
	  return (vertex[coord_a] - mean_a) * (vertex[coord_b] - mean_b); 
  }
};

/*! A different version for computing the cog of an object based
	on the surface triangles. Used as a fallback when the other
	version fails. Courtesy of Corey Goldfeder.
*/
int
computeDefaultCoG(std::vector<Triangle> &triangles, position &defaultCoG)
{
  // Figure out the center of mass consistent with how it was done in the CGDB
  double center_of_mass[3] = {0, 0, 0};
  // First we need the total area of the mesh
  float total_area = 0;
  for (std::vector<Triangle>::const_iterator triangle = triangles.begin();
       triangle != triangles.end(); ++triangle) {
    total_area += triangle->area();
  }

  if (total_area != 0) {
    // Compute the (weighted by area) means of x, y, and z
    float means[3] = {0};
    // set up local class representing function to integrate
    GetCoord get_coord;
    for(unsigned i = 0; i < 3; ++i) {
      get_coord.coord = i;
      for (std::vector<Triangle>::const_iterator triangle = triangles.begin();
          triangle != triangles.end(); ++triangle) {
         means[i] += Gaussian7Integrate(*triangle, get_coord);
      }
    }
    for (int i = 0; i < 3; ++i) center_of_mass[i] = means[i] / total_area;

	// Get the covariance
    float covars[3][3] = {{0}};
    GetCovar get_covar;
    for(unsigned a = 0; a < 3; ++a) {
      for(unsigned b = a; b < 3; ++b) {
        get_covar.coord_a = a;
        get_covar.coord_b = b;
		get_covar.mean_a = center_of_mass[a];
		get_covar.mean_b = center_of_mass[b];
        for (std::vector<Triangle>::const_iterator triangle = triangles.begin();
            triangle != triangles.end(); ++triangle) {
          covars[b][a] += Gaussian7Integrate(*triangle, get_covar);
        }
      }
	}
	defaultCoG.set(center_of_mass);
	return SUCCESS;
  } else {
	  defaultCoG.set(0,0,0);
 	  return FAILURE;
  }
}
/*!
  Given a list of the body vertices, and the number of triangles, this
  computes the center of gravity and inertia matrix by assuming a uniform
  mass distribution.
*/
void
DynamicBody::computeDefaultMassProp(position &defaultCoG, double *defaultI)
{
  std::vector<Triangle> triangles;
  getGeometryTriangles(&triangles);
  if (triangles.empty()) {
	 DBGA("No triangles found when computing mass properties!");
	 defaultCoG.set(0,0,0);
	 return;
  }
  if (computeDefaultInertiaMatrix(triangles, defaultI) == FAILURE) {
	DBGA("Failed to compute inertia matrix based on geometry; using identity");
  }
  for(int i=0; i<3; i++) {
	DBGP(defaultI[3*i] << " " << defaultI[3*i+1] << " " << defaultI[3*i+2]);
  }
  computeDefaultCoG(triangles, defaultCoG);
}

void 
DynamicBody::setDefaultViewingParameters()
{
	Body::setDefaultViewingParameters();
	showAx = false;
}

/*!
  Changes the state of the Inventor switch node controlling whether the
  coordinate axes on the body are visible
*/
void
DynamicBody::showAxes(bool on)
{
  DBGP("Show axes: " << on);
  if (on) IVAxes->whichChild = 0;
  else IVAxes->whichChild = -1;
  showAx = on;
}

/*!
  Sets whether dynamic contact forces should be drawn during dynamic
  simulation.
*/
void
DynamicBody::showDynContactForces(bool on)
{
  showDynCF = on;
}


/*!
  Resets the external wrench accumulator to 0.
*/
void
DynamicBody::resetExtWrenchAcc()
{
  extWrenchAcc[0] = extWrenchAcc[1] = extWrenchAcc[2] = 0.0;
  extWrenchAcc[3] = extWrenchAcc[4] = extWrenchAcc[5] = 0.0;
}

/*!
  Adds \a extW to the external wrench accumulator.
*/
void
DynamicBody::addExtWrench(double *extW){
  extWrenchAcc[0] += extW[0];
  extWrenchAcc[1] += extW[1];
  extWrenchAcc[2] += extW[2];
  extWrenchAcc[3] += extW[3];
  extWrenchAcc[4] += extW[4];
  extWrenchAcc[5] += extW[5];
}

/*! 
   Adds a force expressed in world coordinates, to the external force
   accumulator for this body.
*/
void
DynamicBody::addForce(vec3 force)
{
  extWrenchAcc[0] += force[0];
  extWrenchAcc[1] += force[1];
  extWrenchAcc[2] += force[2];

}

/*!
  Adds a torque expressed in world coordinates, to the external force
  accumulator for this body.
 */
void
DynamicBody::addTorque(vec3 torque)
{
  extWrenchAcc[3] += torque[0];
  extWrenchAcc[4] += torque[1];
  extWrenchAcc[5] += torque[2];
  DBGP("Adding torque "<< torque);
}

/*
  Adds a torque expressed in body coordinates, to the external force
  accumulator for this body.
*/
void
DynamicBody::addRelTorque(vec3 torque)
{
  vec3 worldTorque;
  DBGP("Adding rel torque "<< torque);
  worldTorque = Tran.rotation() * torque;
  extWrenchAcc[3] += worldTorque[0];
  extWrenchAcc[4] += worldTorque[1];
  extWrenchAcc[5] += worldTorque[2];
  DBGP("     world torque "<< worldTorque);
}


/*! 
   Adds a force expressed in world coordinates at a position also expressed
   world coordinates. This force and the computed torque are added to the
   external force accumulator for this body.
*/
void
DynamicBody::addForceAtPos(vec3 force,position pos)
{
  vec3 worldTorque;
  vec3 worldPos;
  worldPos = (pos - CoG * Tran);

  worldTorque = worldPos * force;
  extWrenchAcc[0] += force[0];
  extWrenchAcc[1] += force[1];
  extWrenchAcc[2] += force[2];

  extWrenchAcc[3] += worldTorque[0];
  extWrenchAcc[4] += worldTorque[1];
  extWrenchAcc[5] += worldTorque[2];
}


/*!
  Adds a force expressed in body coordinates at a position also expressed
  body coordinates. This force and the computed torque are added to the
  external force accumulator for this body.
*/
void
DynamicBody::addForceAtRelPos(vec3 force,position pos)
{
  vec3 worldForce;
  vec3 worldTorque;

  worldForce = Tran.rotation() * force;
  worldTorque = Tran.rotation() * ((pos - CoG) * force);
  extWrenchAcc[0] += worldForce[0];
  extWrenchAcc[1] += worldForce[1];
  extWrenchAcc[2] += worldForce[2];

  extWrenchAcc[3] += worldTorque[0];
  extWrenchAcc[4] += worldTorque[1];
  extWrenchAcc[5] += worldTorque[2];
  DBGP("Adding rel force "<< force << " at " << pos);
  DBGP("    world torque "<< worldTorque << " worldForce " << worldForce);
}

// Let the dynamics routine take care of breaking the contacts
bool 
DynamicBody::setPos(const double *new_q)
{
	double norm;
	// is the object within its permissable area?
	if (new_q[0] < bbox_min.x() || new_q[0] > bbox_max.x()) return false;
	if (new_q[1] < bbox_min.y() || new_q[1] > bbox_max.y()) return false;
	if (new_q[2] < bbox_min.z() || new_q[2] > bbox_max.z()) return false;
  
	memcpy(q,new_q,7*sizeof(double));

	// normalize the quaternion
	norm = sqrt(q[3]*q[3]+q[4]*q[4]+q[5]*q[5]+q[6]*q[6]);
	q[3] /= norm;
	q[4] /= norm;
	q[5] /= norm;
	q[6] /= norm;
	Quaternion rot(q[3],q[4],q[5],q[6]);
	vec3 cogOffset = rot * (CoG-position::ORIGIN);
	transf tr = transf(rot,vec3(q[0],q[1],q[2])-cogOffset);

	Tran = tr;
	Tran.toSoTransform(IVTran);
	myWorld->getCollisionInterface()->setBodyTransform(this, Tran);
	
	return true;
}

/*!
  Save the current dynamic state of the body.
  May soon be replaced by pushState().
*/
void
DynamicBody::markState()
{
  memcpy(markedQ,q,7*sizeof(double));
  memcpy(markedV,v,6*sizeof(double));
}

/*!
  Restore the marked dynamic state of the body.
  May soon be replaced by popState().
*/
void
DynamicBody::returnToMarkedState()
{
  memcpy(v,markedV,6*sizeof(double));
  setPos(markedQ);
}

/*! 
  Push the current dynamic state of the body onto a local stack.
*/
void
DynamicBody::pushState()
{
  double *tmp = new double[7];
  memcpy(tmp,q,7*sizeof(double));
  qStack.push_back(tmp);

  tmp = new double[6];
  memcpy(tmp,v,6*sizeof(double));
  vStack.push_back(tmp);  
}

/*!
  Pop the current dynamic state of the body from a local stack.
*/
bool
DynamicBody::popState()
{
	if (qStack.empty()) {
		DBGA("Pop state failed: stack empty");
		return false;
	}
	memcpy(v,vStack.back(),6*sizeof(double));
	setPos(qStack.back());

	// don't pop off the first saved state.
	if (++qStack.begin() != qStack.end()) {
		delete [] vStack.back();
		delete [] qStack.back();
		vStack.pop_back(); 
		qStack.pop_back();
		return true;
	} else {
		return false;
    }
}

void
DynamicBody::clearState()
{
	//we don't actually change the state of the object, just clear
	//the stack. For some reason we leave the last state in.
	if (qStack.empty()) return;
	while (++qStack.begin() != qStack.end()) {
		delete [] vStack.back();
		delete [] qStack.back();
		vStack.pop_back(); 
		qStack.pop_back();
	}
}
/*!
  Calls Body::setTran then updates the dynamic state of the body.
*/
int
DynamicBody::setTran(transf const& tr)
{
  if (tr == Tran) return SUCCESS;
  if (Body::setTran(tr) == FAILURE) return FAILURE;
  Quaternion quat = Tran.rotation();
  vec3 cogOffset = quat * (CoG-position::ORIGIN);
  q[0] = Tran.translation().x()+cogOffset.x();
  q[1] = Tran.translation().y()+cogOffset.y();
  q[2] = Tran.translation().z()+cogOffset.z();
  q[3] = quat.w;
  q[4] = quat.x;
  q[5] = quat.y;
  q[6] = quat.z;  
  if (fixed) {
	  if (!dynJoint->getPrevLink()) {
		  //if the previous link of the dynamic joint is NULL this means
		  //the body is fixed to the current position in the world
		  //we need to update that position
		  fix();
	  }
  }
  return SUCCESS;
}

/*!
  Fixes the body so that it does not move during dynamic simulation.  This
  is done by addind a fixed dynamic joint that will constrain all 6 body
  velocities to 0.
*/
void
DynamicBody::fix()
{
  fixed = true;
  setDynJoint(new FixedDynJoint(NULL,this,Tran));
}

/*!
  Removes the fixed dynamic joint so the body is free to move again during
  dynamic simulation.
*/
void
DynamicBody::unfix()
{
  fixed = false;
  setDynJoint(NULL);
}

/*!
  Sets the dynamic joint connected to this body to \a dj.
*/
void
DynamicBody::setDynJoint(DynJoint *dj)
{
  if (dynJoint) delete dynJoint;
  dynJoint = dj;
}

///////////////////////////////////////////////////////////////////////////////
//                            Link
///////////////////////////////////////////////////////////////////////////////

/*
  Initializes a robot link.  \a r is the pointer to the robot that owns
  this link, \a c is the index of the chain this link is in, and \a l is
  the link number of this link within that chain.
*/
Link::Link(Robot *r,int c, int l,World *w,const char *name) : DynamicBody(w,name)
{
  owner = r; chainNum = c; linkNum = l; showVC = false; showFC = false;
}


/*
  Stub destructor.
*/
Link::~Link()
{
}


/*!
  Sets BOTH the worldElement's AND the robot's \a contactsChanged flag.
*/
void
Link::setContactsChanged()
{
  WorldElement::setContactsChanged();
  owner->setContactsChanged();
}
/*!
	This acts like contactPreventMotion, except in the case of a link belonging to a robot.
	In this case, contact against another link of the same robot does not prevent motion,
	as we assume the entire robot is moving.
*/
bool
Link::externalContactsPreventMotion(const transf& motion)
{
	std::list<Contact *>::iterator cp;
	std::list<Contact *> contactList;

	contactList = getContacts();
	for (cp=contactList.begin();cp!=contactList.end();cp++) {
		if ( (*cp)->getBody2()->getOwner() == getOwner() )
			continue;
		if ((*cp)->preventsMotion(motion)) {
			return true;
		}
	}
  return false;
}

/*! By convention, in GraspIt! each link's origin is located at the next joint in the chain */
position
Link::getDistalJointLocation()
{
	return position(0,0,0);
}

/*! The translation component of the previous joint transform gives me the location of that joint
	in this joint's coordinate system */
position
Link::getProximalJointLocation()
{
	int jointNum = owner->getChain(chainNum)->getLastJoint(linkNum);
	Joint *j = owner->getChain(chainNum)->getJoint(jointNum);
	vec3 p = j->getTran().inverse().translation();
	return position( p.x(), p.y(), p.z() );
}

vec3
Link::getProximalJointAxis()
{
	int jointNum = owner->getChain(chainNum)->getLastJoint(linkNum);
	Joint *j = owner->getChain(chainNum)->getJoint(jointNum);
	vec3 r = vec3(0,0,1) * j->getTran().inverse();
	return r;
}

///////////////////////////////////////////////////////////////////////////////
//                            GraspableBody
///////////////////////////////////////////////////////////////////////////////

/*
  Stub constructor.
*/
GraspableBody::GraspableBody(World *w,const char *name) : DynamicBody(w,name)
{
#ifdef CGDB_ENABLED
	mGraspitDBModel = NULL;
#endif
}

/*
  Stub destructor.
*/
GraspableBody::~GraspableBody()
{

}

/*! Shows friction cones and axes, and sets transparency to 0.4 */
void 
GraspableBody::setDefaultViewingParameters()
{
	showFC = true;
	showVC = true;
	showAxes(true);
	setTransparency(0.4f);
}

/*! Probably not needed, just calls super*/
void
GraspableBody::cloneFrom(const GraspableBody *original) 
{
	DynamicBody::cloneFrom(original);
	setDefaultViewingParameters();
}

/*!
  Output method for writing body data to a text world configuration file
*/
QTextStream&
operator<<(QTextStream &os, const GraspableBody &gb)
{
  os << gb.myFilename << endl;
  return os;
}
