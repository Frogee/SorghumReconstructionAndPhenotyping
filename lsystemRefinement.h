
#ifndef LSYSTEM_REFINEMENT_H
#define LSYSTEM_REFINEMENT_H

#include <python2.7/Python.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lsystemParameters.h"

LSystemParameters refineLSystemParallel_ExpandedStem_InternodeLength(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams);

LSystemParameters refineLSystemParallel_ExpandedStem_LeafPhyllotaxyAngle(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams);

LSystemParameters refineLSystemParallel_ExpandedStem_LeafCurvaturesControlPointTwo(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams);

LSystemParameters refineLSystem_ExpandedStem_InternodeLength(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams);


LSystemParameters refineLSystem_ExpandedStem_LeafPhyllotaxyAngle(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams);

LSystemParameters refineLSystem_ExpandedStem_LeafCurvaturesControlPointTwo(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams);

#endif
