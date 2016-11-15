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
// Author(s): Andrew T. Miller 
//
// $Id: material.cpp,v 1.3 2009/03/25 22:10:04 cmatei Exp $
//
//######################################################################

#include <stdio.h>
#include <string.h>

#include "material.h"

double Cof[NUM_MATERIAL][NUM_MATERIAL];
double KineticCof[NUM_MATERIAL][NUM_MATERIAL];
char matNameList[NUM_MATERIAL][30] = {"frictionless","glass","metal","wood",
				   "plastic","rubber","stone","invalid"};

materialT readMaterial(const char *matStr)
{
  if (!strcmp(matStr,"frictionless")) return(frictionless);
  if (!strcmp(matStr,"glass")) return(glass);
  if (!strcmp(matStr,"metal")) return(metal);
  if (!strcmp(matStr,"wood")) return(wood);
  if (!strcmp(matStr,"plastic")) return(plastic);
  if (!strcmp(matStr,"rubber")) return(rubber);
  if (!strcmp(matStr,"stone")) return(stone);
  return(invalid);
}

void getMaterialStr(materialT mat,char *str){
  switch (mat) {
  case frictionless:
    strcpy(str,"frictionless");
    break;
  case glass:
    strcpy(str,"glass");
    break;
  case metal:
    strcpy(str,"metal");
    break;
  case wood:
    strcpy(str,"wood");
    break;
  case plastic:
    strcpy(str,"plastic");
    break;
  case rubber:
    strcpy(str,"rubber");
    break;
  case stone:
    strcpy(str,"stone");
    break;
  case invalid:
    strcpy(str,"invalid");
    break;
  }
}

const char *
getMaterialStr(materialT mat){
  switch (mat) {
  case frictionless:
    return "frictionless";
  case glass:
    return "glass";
  case metal:
    return "metal";
  case wood:
    return "wood";
  case plastic:
    return "plastic";
  case rubber:
    return "rubber";
  case stone:
	  return "stone";
  case invalid:
    break;
  }
  return "invalid";
}

void initCof(){
  int i,j;
  for (i = 0; i < NUM_MATERIAL; i++)
    for (j = 0; j < NUM_MATERIAL; j++) {
      Cof[i][j] = 0.0;
      KineticCof[i][j] = 0.0;
    }

  /* from CRC handbook of chemistry and physics */
  /* assuming room temperature contact */

  /* Metal on Metal ?guess?: 0.2 */
  KineticCof[metal][metal] = 0.1;

  /* Wood on Wood -Dry-: 0.25-0.5 */
  KineticCof[wood][wood] = 0.3;

  /* Wood on Metal -Dry-: 0.2-0.6 */
  KineticCof[wood][metal] = KineticCof[metal][wood] = 0.2;

  /* Plastic on Plastic ?guess?: 0.3 */
  KineticCof[plastic][plastic] = 0.2;

  /* Plastic on Metal ?guess?: 0.2 */
  KineticCof[plastic][metal] = KineticCof[metal][plastic] = 0.1;

  /* Plastic on Wood ?guess?: 0.4 */
  KineticCof[plastic][wood] = KineticCof[wood][plastic] = 0.3;

  /* Rubber on Solids:  1-4 */
  KineticCof[rubber][rubber] = 1.9;  
  KineticCof[rubber][metal] = KineticCof[metal][rubber] = 0.9;
  KineticCof[rubber][wood] = KineticCof[wood][rubber] = 0.9;
  KineticCof[rubber][plastic] = KineticCof[plastic][rubber] = 0.9;

  /* for now we simply copy metal values for glass */
  KineticCof[glass][glass] = 0.1;
  KineticCof[glass][metal] = KineticCof[metal][glass] = 0.1;
  KineticCof[glass][wood] = KineticCof[wood][glass] = 0.2;
  KineticCof[glass][plastic] = KineticCof[plastic][glass] = 0.1;
  KineticCof[glass][rubber] = KineticCof[rubber][glass] = 0.9;

  /* guessed values for stone */
  KineticCof[stone][stone] = 0.6;
  KineticCof[stone][glass] = KineticCof[glass][stone] = 0.3;
  KineticCof[stone][metal] = KineticCof[metal][stone] = 0.3;
  KineticCof[stone][wood] = KineticCof[wood][stone] = 0.5;
  KineticCof[stone][plastic] = KineticCof[plastic][glass] = 0.5;
  KineticCof[glass][rubber] = KineticCof[rubber][glass] = 1.4;

  /* Metal on Metal ?guess?: 0.2 */
  Cof[metal][metal] = 0.2;

  /* Wood on Wood -Dry-: 0.25-0.5 */
  Cof[wood][wood] = 0.4;

  /* Wood on Metal -Dry-: 0.2-0.6 */
  Cof[wood][metal] = Cof[metal][wood] = 0.3;

  /* Plastic on Plastic ?guess?: 0.3 */
  Cof[plastic][plastic] = 0.3;

  /* Plastic on Metal ?guess?: 0.2 */
  Cof[plastic][metal] = Cof[metal][plastic] = 0.2;

  /* Plastic on Wood ?guess?: 0.4 */
  Cof[plastic][wood] = Cof[wood][plastic] = 0.4;

  /* Rubber on Solids:  1-4 */
  Cof[rubber][rubber] = 2.0;  
  Cof[rubber][metal] = Cof[metal][rubber] = 1.0;
  Cof[rubber][wood] = Cof[wood][rubber] = 1.0;
  Cof[rubber][plastic] = Cof[plastic][rubber] = 1.0;

  /* for now we simply copy metal values for glass */
  Cof[glass][glass] = 0.2;
  Cof[glass][metal] = Cof[metal][glass] = 0.2;
  Cof[glass][wood] = Cof[wood][glass] = 0.3;
  Cof[glass][plastic] = Cof[plastic][glass] = 0.2;
  Cof[glass][rubber] = Cof[rubber][glass] = 1.0;

  /* guessed values for stone */
  Cof[stone][stone] = 0.7;
  Cof[stone][glass] = Cof[glass][stone] = 0.4;
  Cof[stone][metal] = Cof[metal][stone] = 0.4;
  Cof[stone][wood] = Cof[wood][stone] = 0.6;
  Cof[stone][plastic] = Cof[plastic][stone] = 0.6;
  Cof[stone][rubber] = Cof[rubber][stone] = 1.5;

}  
  

