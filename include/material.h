//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// This software is protected under a Research and Educational Use
// Only license, (found in the file LICENSE.txt), that you should have
// received with this distribution.
//
// Author(s):  Andrew T. Miller 
//
// $Id: material.h,v 1.3 2009/03/25 22:10:23 cmatei Exp $
//
//######################################################################

#ifndef MATERIAL_HXX

enum materialT {frictionless, glass, metal, wood,plastic, rubber, stone, invalid};

#define NUM_MATERIAL 8

extern double Cof[NUM_MATERIAL][NUM_MATERIAL];
extern double KineticCof[NUM_MATERIAL][NUM_MATERIAL];
extern char *matNameList[NUM_MATERIAL];

void initCof();
materialT readMaterial(const char *matStr);
void getMaterialStr(materialT mat,char *str);
const char *getMaterialStr(materialT mat);

#define MATERIAL_HXX
#endif

