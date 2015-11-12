#ifndef SNAKEGRIPPER_H_
#define SNAKEGRIPPER_H_
#include "robot.h"
//! A special hand because collisions must be turned off between the first links of the two chains
/*! A special hand because collisions must be turned off between the
    first links of the two chains.  This is done by overriding the
    load method.
 */
class SnakeGripper : public Hand {
	Q_OBJECT

 public:

  /*! Empty constructor (placeholder) */
  SnakeGripper(World *w,const char *name) : Hand(w,name) {}
  
   /*! Performs the normal robot load routine from xml then turns off collisions 
       between the first links of the two chains
 */
  virtual int loadFromXml(const TiXmlElement* root,QString rootPath);

 /*! Performs the normal robot clone routine then turns off collisions between
     the first links of the two chains
 */
  virtual void cloneFrom(Hand *original);

};

#endif