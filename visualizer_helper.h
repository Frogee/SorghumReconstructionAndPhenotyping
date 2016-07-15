
#ifndef VISUALIZER_HELPER_H
#define VISUALIZER_HELPER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <string>
#include "segmentation.h"
#include "supervoxel_construction.h"

void
addLineConnectionToViewer (pcl::PointXYZ pointBegin,
                                  pcl::PointXYZ pointEnd,
                                  std::string lineName,
                                  pcl::visualization::PCLVisualizer* viewer,
                                  int viewport);

void
addSupervoxelAdjacencyToViewer (SupervoxelDataContainer inputSupervoxelData,
                                  pcl::visualization::PCLVisualizer* viewer,
                                  int viewport);


/** \brief Updates a mesh in a PCLVisualizer given the mesh and segmentation data for the mesh.
  *
  * \param[in] currentMesh pointer to the PolygonMesh that will be used to update the mesh.
  * \param[in] inputSegmentationData PlantSegmentationDataContainer that holds the segmentation information.
  * \param[in] visualizer pointer to the PCLVisualizer to be used.
  * \param[in] viewport integer referring to the viewport to be used.
  * \author Ryan McCormick
  */
int updateSegmentedMeshInViewer(pcl::PolygonMesh *currentMesh,
                                PlantSegmentationDataContainer inputSegmentationData,
                                pcl::visualization::PCLVisualizer *visualizer,
                                std::string meshID,
                                int viewport);


int screenshotPLY(int argc, char** argv, InputParameters inputParams);



#endif

