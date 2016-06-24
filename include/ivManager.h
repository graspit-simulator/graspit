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
// Author(s):  Jennifer Buehler  
//
//
//######################################################################

/*! \file 
  \brief Defines an abstract superclass for the IVmgr class, which handles 3D user interaction.
 */
#ifndef IVMANAGER_HXX
#define IVMANAGER_HXX

class IVManager {
public:
  IVManager(const char *name=0){}
  virtual ~IVManager(){}

  virtual void setCamera(double _px, double _py, double _pz, double _q1, double _q2, double _q3, double _q4, double _fd){
    px=_px;  
    py=_py;  
    pz=_pz;  
    q1=_q1;  
    q2=_q2;  
    q3=_q3;  
    q4=_q4;  
    fd=_fd;  
  }
  
  virtual void getCamera(float &_px, float &_py, float &_pz, float &_q1, float &_q2, float &_q3, float &_q4, float &_fd){
    _px=px;  
    _py=py;  
    _pz=pz;  
    _q1=q1;  
    _q2=q2;  
    _q3=q3;  
    _q4=q4;  
    _fd=fd;  

  }

  virtual void viewAll(){}

private:
  // the stored camera parameters in case the implementing class does not store them elsewhere
  double px, py, pz;
  double q1, q2, q3, q4;
  double fd;
  
};
#endif
