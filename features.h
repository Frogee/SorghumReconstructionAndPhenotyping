#ifndef FEATURES_H
#define FEATURES_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "inputParams.h"

/** \brief Computes point features for the input mesh.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0], and the remaining index (argv[1]) is the labeled or unlabeled mesh
  * for which features will be calculated.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int computePointFeatures(int argc, char** argv, InputParameters inputParams);
int supervoxelConstruction(int argc, char** argv, InputParameters inputParams);

#endif
