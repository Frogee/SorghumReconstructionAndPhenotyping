
#ifndef SAMPLE_MESH_TO_POINT_CLOUD_H
#define SAMPLE_MESH_TO_POINT_CLOUD_H

#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>

pcl::PointXYZ sampleRandomPointOnTriangleSurface(pcl::PointXYZ inputPointOne, pcl::PointXYZ inputPointTwo, pcl::PointXYZ inputPointThree);
int sampleMeshToPointCloud(pcl::PolygonMesh* inputMeshToSample, pcl::PointCloud<pcl::PointXYZ> &outputCloud, InputParameters inputParams);
int sampleMeshToPointCloud_ForParallelCalling(pcl::PolygonMesh* inputMeshToSample, pcl::PointCloud<pcl::PointXYZ> &outputCloud, InputParameters inputParams);
#endif

