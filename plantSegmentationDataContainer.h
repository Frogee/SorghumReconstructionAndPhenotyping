
#ifndef PLANT_SEGMENTATION_DATA_CONTAINER_H
#define PLANT_SEGMENTATION_DATA_CONTAINER_H

#include <map>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include "supervoxel_construction.h"
#include "inputParams.h"
#include "tupleTriplet.h"
#include "plantSegmentationDataContainer.h"


/** \brief Container to hold data associating X, Y, Z points with R, G, B colors.
  *
  * \author Ryan McCormick
  */
class PlantSegmentationDataContainer{
    public:
        PlantSegmentationDataContainer();
        PlantSegmentationDataContainer(pcl::PolygonMesh *inputMesh);
        ~PlantSegmentationDataContainer();

        bool allPointsAreLabeled();
        bool allSupervoxelsAreLabeled();

        int updateSupervoxelSegmentationMap();
        int updateLearnedPointsWithSupervoxels();
        std::multimap<uint32_t, uint32_t> returnSupervoxelAdjacencyForNonLeafSupervoxels();
        std::multimap<uint32_t, uint32_t> returnSupervoxelAdjacencyForUnlabeledSupervoxels();


        std::map <TupleTriplet, TupleTriplet> _map_segmentedPoints; /*!< \brief The X, Y, Z point and its corresponding R, G, B value. > */
        std::map <uint32_t, TupleTriplet> _map_segmentedSupervoxels; /*!< \brief The supervoxel label and its corresponding R, G, B value. > */
        SupervoxelDataContainer _supervoxelData; /*!< \brief Supervoxel data for the mesh. > */
        std::map <uint32_t, TupleTriplet> _map_leafTipLabels; /*!< \brief Supervoxel labels of the leaf tips. > */
        std::vector<uint32_t> _leafOrderBottomToTop; /*!< \brief Leaf labels ordered from bottom to top. > */
};




#endif
