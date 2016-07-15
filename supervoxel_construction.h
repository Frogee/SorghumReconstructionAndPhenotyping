
#ifndef SUPERVOXEL_CONSTRUCTION_H
#define SUPERVOXEL_CONSTRUCTION_H

#include <map>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include "inputParams.h"
#include "tupleTriplet.h"
#include "supervoxel_construction.h"


/** \brief Contains data pertaining to the clustering of a point cloud into supervoxels.
  */
class SupervoxelDataContainer{
    public:
        SupervoxelDataContainer();
        ~SupervoxelDataContainer();

        void print_cloudSizes();
        void print_supervoxelClusters();
        void print_supervoxelAdjacency();
        void print_supervoxelAdjacency(uint32_t inputLabel);
        std::vector<uint32_t> returnVectorOfSupervoxelAdjacencyLabels(uint32_t inputLabel);
        void print_vv_colorVector();
        void constructInternalMaps();
        void trimAdjacencyMapOnGeodesicDistance(InputParameters inputParams, pcl::PolygonMesh *inputMesh);

        pcl::PointCloud<pcl::PointXYZL>::Ptr    _cloudWithLabeledPoints;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr _cloudWithPointsColorizedByLabel;
        pcl::PointCloud<pcl::PointXYZL>::Ptr    _cloudWithCentroidsByLabel;

        std::map <TupleTriplet, uint32_t> _map_pointsToLabel;
        std::map <uint32_t, TupleTriplet> _map_labelToCentroidOfSupervoxel;
        std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > _supervoxelClusters;
        std::multimap<uint32_t, uint32_t> _supervoxelAdjacency;
        std::vector< std::vector< uint8_t > > _vv_colorVector;

};

SupervoxelDataContainer constructSupervoxels(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsInput,
                        pcl::PointCloud<pcl::Normal>::Ptr inputNormals,
                        InputParameters inputParams);

pcl::PointXYZ returnClosestPointToCentroidGivenSupervoxel(pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr inputSupervoxel);

#endif
