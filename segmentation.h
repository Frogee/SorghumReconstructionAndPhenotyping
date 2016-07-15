
#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <pcl/visualization/pcl_visualizer.h>

#include "inputParams.h"
#include "tupleTriplet.h"
#include "supervoxel_construction.h"
#include "plantSegmentationDataContainer.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>


int segmentationRegionGrowing(int argc, char** argv, InputParameters inputParams);

int segmentationSACFromNormals(int argc, char** argv, InputParameters inputParams);

/** \brief Function called by main to initiate segmentation of a plant mesh.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0]. Of the two remaining indices, argv[1] contains the .ply format mesh to be segmented.
  * and argv[2] contains the .ply format point cloud that contains the points from the same cloud as argv[1] with labels assigned
  * as colors. Currently, only the stem has labeled points via machine learning.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int segmentationFromPLY(int argc, char** argv, InputParameters inputParams);

/** \brief Segment a mesh mostly by using geodesic distances across the mesh. Called by segmentationFromPLY().
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0]. Of the two remaining indices, argv[1] contains the .ply format mesh to be segmented.
  * and argv[2] contains the .ply format point cloud that contains the points from the same cloud as argv[1] with labels assigned
  * as colors. Currently, only the stem has labeled points via machine learning.
  * \param[in] visualizer the pcl::visualization::PCLVisualizer that will be used to display the segmentation process
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int geodesicDistanceSegmentation(int argc, char **argv, pcl::visualization::PCLVisualizer *visualizer, InputParameters inputParams);

/** \brief Code to segment mesh points based on assignments made during machine learning.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0]. Of the two remaining indices, argv[1] contains the .ply format mesh to be segmented.
  * and argv[2] contains the .ply format point cloud that contains the points from the same cloud as argv[1] with labels assigned
  * as colors. Currently, only the stem has labeled points via machine learning.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \param[in] visualizer the pcl::visualization::PCLVisualizer that will be used to display the segmentation process.
  * \param[in] viewport the pcl::visualization::PCLVisualizer viewport that will be used to display the segmentation process.
  * \return PlantSegmentationDataContainer the PlantSegmentationDataContainer containing the segmented mesh.
  * \author Ryan McCormick
  */
PlantSegmentationDataContainer segmentFromLearnedPoints(int argc, char **argv,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints,
                                pcl::PointCloud<pcl::Normal>::Ptr cloudNormals,
                                InputParameters inputParams,
                                pcl::visualization::PCLVisualizer *visualizer,
                                int viewport);


/** \brief Segment the pot out of a series of registered point clouds.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0], and the remaining indices are transformed individual point clouds to be processed.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int segmentOutPot(int argc, char **argv, InputParameters inputParams);

int writeIndividualComponentsOfSegmentedMesh(pcl::PolygonMesh inputMesh, pcl::visualization::PCLVisualizer *visualizer);

pcl::PolygonMesh extractMeshFromPolygonMeshGivenPointCloud(pcl::PolygonMesh inputMesh, pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud);

int returnLabelOfSupervoxelWithMinimumStemPoint(PlantSegmentationDataContainer inputSegmentationData);
int returnLabelOfSupervoxelWithMaximumStemPoint(PlantSegmentationDataContainer inputSegmentationData);

PlantSegmentationDataContainer refineSegmentation(InputParameters inputParams, pcl::PolygonMesh *inputMesh,
                                                    PlantSegmentationDataContainer *inputSegmentationData, pcl::visualization::PCLVisualizer *visualizer);

#endif
