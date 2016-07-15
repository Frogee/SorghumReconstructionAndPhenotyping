#ifndef MESHGENERATION_H
#define MESHGENERATION_H

#include "inputParams.h"

int meshGeneration(int argc, char** argv, InputParameters inputParams);
int convertPCDToPLY(int argc, char** argv, InputParameters inputParams);
int multiCloudMeshGeneration(int argc, char** argv, InputParameters inputParams);
int convertPLYToPCD(int argc, char** argv, InputParameters inputParams);
#endif
