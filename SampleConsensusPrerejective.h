#ifndef SAMPLECONSENSUSPREREJECTIVE_H
#define SAMPLECONSENSUSPREREJECTIVE_H

#include "inputParams.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

int registerPointCloudsRANSACPrerejective(int argc, char **argv, InputParameters inputParams);

/** \brief Performs pre-rejective RANSAC on two point clouds and returns the transform that transforms the source to the target.
  *
  * \param[in] target pointer to the point cloud that will serve as the target.
  * \param[in] source pointer to the point cloud that will be registered to the target.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \param[in] visualizer pointer to the PCLVisualizer to be used. This isn't currently implemented.
  * \return Eigen::Matrix4f containing the transform to transform the source to the target.
  * \author Ryan McCormick
  */
Eigen::Matrix4f returnPairwiseRegistrationTransformUsingRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                                                    InputParameters inputParams,
                                                    pcl::visualization::PCLVisualizer *visualizer);


int registerKinectFusionPLYsRANSAC(int argc, char **argv, InputParameters inputParams);

#endif
