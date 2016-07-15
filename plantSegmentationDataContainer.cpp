
#include <vector>
#include <map>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include "supervoxel_construction.h"
#include "plantSegmentationDataContainer.h"
#include "tupleTriplet.h"
#include "loggingHelper.h"


PlantSegmentationDataContainer::PlantSegmentationDataContainer() {
    //empty constructor
}

PlantSegmentationDataContainer::PlantSegmentationDataContainer(pcl::PolygonMesh *inputMesh) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointsInMesh (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromPCLPointCloud2(inputMesh->cloud, *pointsInMesh);

    for (uint i = 0; i < pointsInMesh->points.size(); i++) {
        pcl::PointXYZRGBA currentPoint = pointsInMesh->points[i];
        TupleTriplet tupleXYZ(currentPoint.x, currentPoint.y, currentPoint.z);
        TupleTriplet tupleRGB(currentPoint.r, currentPoint.g, currentPoint.b);
        _map_segmentedPoints.insert(std::pair<TupleTriplet, TupleTriplet> (tupleXYZ, tupleRGB));
    }
}

PlantSegmentationDataContainer::~PlantSegmentationDataContainer() {
    //empty destructor
}

bool PlantSegmentationDataContainer::allPointsAreLabeled() {
    ColorMap colorMap;
    bool allPointsAreLabeled = true;
    std::map <TupleTriplet, TupleTriplet>::iterator itr;
    for (itr = _map_segmentedPoints.begin(); itr != _map_segmentedPoints.end(); itr++) {
        if (itr->second == colorMap._unsegmented_color) {
            allPointsAreLabeled = false;
            break;
        }
    }
    return allPointsAreLabeled;
}

bool PlantSegmentationDataContainer::allSupervoxelsAreLabeled() {
    ColorMap colorMap;
    bool allPointsAreLabeled = true;
    std::map <uint32_t, TupleTriplet>::iterator itr;
    for (itr = _map_segmentedSupervoxels.begin(); itr != _map_segmentedSupervoxels.end(); itr++) {
        if (itr->second == colorMap._unsegmented_color) {
            allPointsAreLabeled = false;
            break;
        }
    }
    return allPointsAreLabeled;
}

int PlantSegmentationDataContainer::updateSupervoxelSegmentationMap() {
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator itr;
    for (itr = _supervoxelData._supervoxelClusters.begin(); itr != _supervoxelData._supervoxelClusters.end(); itr++) {

        pcl::PointXYZ closestPoint = returnClosestPointToCentroidGivenSupervoxel(itr->second);
        TupleTriplet closestPointTuple(closestPoint.x, closestPoint.y, closestPoint.z);
        TupleTriplet color = _map_segmentedPoints[closestPointTuple];
        //std::cout << "Updating supervoxel label " << itr->first << " with color ";
        //printTupleTriplet(color);
        if (_map_segmentedSupervoxels.find(itr->first) != _map_segmentedSupervoxels.end()) {
            _map_segmentedSupervoxels[itr->first] = color;
        }
        else {
            _map_segmentedSupervoxels.insert(std::pair<uint32_t, TupleTriplet> (itr->first, color));
        }

    }
    return 0;
}

int PlantSegmentationDataContainer::updateLearnedPointsWithSupervoxels() {
    ColorMap colorMap;
    LOG.DEBUG("Updating the segmentation of learned points based on the supervoxels.");
    // For each supervoxel, determine how many points are labeled as stem.
    // If the amount is greater than an assigned proportion of the points in the supervoxel, label the entire supervoxel as stem.
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator itr;
    for (itr = _supervoxelData._supervoxelClusters.begin(); itr != _supervoxelData._supervoxelClusters.end(); itr++) {
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr currentSupervoxel = itr->second;
        int totalPointsInSupervoxel = (itr->second)->voxels_->points.size(); // currentSupervoxel->voxels_->points.size()
        int numberOfStemPoints = 0;
        int numberOfInflorescencePoints = 0;
        for (uint32_t i = 0; i < currentSupervoxel->voxels_->points.size(); i++) {
            pcl::PointXYZRGBA currentPoint = currentSupervoxel->voxels_->points[i];
            TupleTriplet currentPointTuple(currentPoint.x, currentPoint.y, currentPoint.z);
            TupleTriplet currentColor = _map_segmentedPoints[currentPointTuple];
            if (currentColor == colorMap._stem_color) {
                numberOfStemPoints += 1;
            }
            if (currentColor == colorMap._inflorescence_color) {
                numberOfInflorescencePoints += 1;
            }
        }
        float f_numberOfStemPoints = numberOfStemPoints;
        float f_numberOfInflorescencePoints = numberOfInflorescencePoints;
        float f_totalPointsInSupervoxel = totalPointsInSupervoxel;

        float proportionStemPoints = f_numberOfStemPoints / f_totalPointsInSupervoxel;
        float proportionInflorescencePoints = f_numberOfInflorescencePoints / f_totalPointsInSupervoxel;
        float STEMPOINT_PROPORTIONTHRESHOLD = 0.33; //magic number that we should probably make an input parameter.
        float INFLORESCENCEPOINT_PROPORTIONTHRESHOLD = 0.33; //magic number that we should probably make an input parameter.
        // For mixed supervoxels, we give precedence to the stem.
        if (proportionStemPoints > STEMPOINT_PROPORTIONTHRESHOLD) {  //
            for (uint32_t i = 0; i < currentSupervoxel->voxels_->points.size(); i++) {
                pcl::PointXYZRGBA currentPoint = currentSupervoxel->voxels_->points[i];
                TupleTriplet currentPointTuple(currentPoint.x, currentPoint.y, currentPoint.z);
                _map_segmentedPoints[currentPointTuple] = colorMap._stem_color;
            }
        }
        else if(proportionInflorescencePoints > INFLORESCENCEPOINT_PROPORTIONTHRESHOLD) {
            for (uint32_t i = 0; i < currentSupervoxel->voxels_->points.size(); i++) {
                pcl::PointXYZRGBA currentPoint = currentSupervoxel->voxels_->points[i];
                TupleTriplet currentPointTuple(currentPoint.x, currentPoint.y, currentPoint.z);
                _map_segmentedPoints[currentPointTuple] = colorMap._inflorescence_color;
            }
        }
        else {
            for (uint32_t i = 0; i < currentSupervoxel->voxels_->points.size(); i++) {
                pcl::PointXYZRGBA currentPoint = currentSupervoxel->voxels_->points[i];
                TupleTriplet currentPointTuple(currentPoint.x, currentPoint.y, currentPoint.z);
                _map_segmentedPoints[currentPointTuple] = colorMap._unsegmented_color;
            }
        }
    }
}

std::multimap<uint32_t, uint32_t> PlantSegmentationDataContainer::returnSupervoxelAdjacencyForNonLeafSupervoxels() {
    this->updateSupervoxelSegmentationMap();
    ColorMap colorMap;
    // For each adjacency in the adjacency map
    std::cout << "Trimming supervoxel adjacency map." << std::endl;
    std::multimap<uint32_t,uint32_t>::iterator label_itr;
    std::multimap<uint32_t, uint32_t> trimmedAdjacencyMap;
    float averageDistanceBetweenSupervoxels = 0;
    float maxDistanceBetweenSupervoxels = 0;
    int totalIterations = 0;
    float totalDistanceBetweenSupervoxels = 0;
    int supervoxelCounter = 0;
    for (label_itr = _supervoxelData._supervoxelAdjacency.begin(); label_itr != _supervoxelData._supervoxelAdjacency.end(); ) { //We increment this iterator at the end of the loop
        supervoxelCounter = supervoxelCounter + 1;
        if (supervoxelCounter % 100 == 0) {
            std::cout << "\tOn supervoxel: " << supervoxelCounter << std::endl;
        }
        //First get the label
        uint32_t supervoxel_label = label_itr->first;

        //Get the color corresponding to the label
        TupleTriplet currentColor = _map_segmentedSupervoxels[supervoxel_label];

        //Now we need to iterate through the adjacent supervoxels
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
        for ( adjacent_itr = _supervoxelData._supervoxelAdjacency.equal_range(supervoxel_label).first;
                adjacent_itr != _supervoxelData._supervoxelAdjacency.equal_range (supervoxel_label).second; adjacent_itr++) {
            uint32_t adjacentLabel = adjacent_itr->second;
            TupleTriplet adjacentColor = _map_segmentedSupervoxels[adjacentLabel];

            // If the color of the current label is not contained in the leaf color map
            if (colorMap._leafColorMap_colorsToLabel.find(currentColor) == colorMap._leafColorMap_colorsToLabel.end()) {
                // and if the color of the adjacent label is not contained in the color map
                if (colorMap._leafColorMap_colorsToLabel.find(adjacentColor) == colorMap._leafColorMap_colorsToLabel.end()) {
                // add it to the trimmed map.
                    trimmedAdjacencyMap.insert(std::pair<uint32_t, uint32_t>(supervoxel_label, adjacentLabel));
                }
            }
            else {
                // One of them is segmented, so we aren't interested in traveling along that edge.
            }
        }
        //Move iterator forward to next label
        label_itr = _supervoxelData._supervoxelAdjacency.upper_bound(supervoxel_label);
    }
    std::cout << "Finished trimming adjacencies where one of the nodes is segmented." << std::endl;
    return trimmedAdjacencyMap;
}

std::multimap<uint32_t, uint32_t> PlantSegmentationDataContainer::returnSupervoxelAdjacencyForUnlabeledSupervoxels() {
    this->updateSupervoxelSegmentationMap();
    ColorMap colorMap;
    // For each adjacency in the adjacency map
    LOG.DEBUG("Trimming supervoxel adjacency map to leave only unlabeled supervoxels.");
    std::multimap<uint32_t,uint32_t>::iterator label_itr;
    std::multimap<uint32_t, uint32_t> trimmedAdjacencyMap;
    float averageDistanceBetweenSupervoxels = 0;
    float maxDistanceBetweenSupervoxels = 0;
    int totalIterations = 0;
    float totalDistanceBetweenSupervoxels = 0;
    int supervoxelCounter = 0;
    for (label_itr = _supervoxelData._supervoxelAdjacency.begin(); label_itr != _supervoxelData._supervoxelAdjacency.end(); ) { //We increment this iterator at the end of the loop
        supervoxelCounter = supervoxelCounter + 1;
        if (supervoxelCounter % 100 == 0) {
            std::cout << "\tOn supervoxel: " << supervoxelCounter << std::endl;
        }
        //First get the label
        uint32_t supervoxel_label = label_itr->first;

        //Get the color corresponding to the label
        TupleTriplet currentColor = _map_segmentedSupervoxels[supervoxel_label];

        //Now we need to iterate through the adjacent supervoxels
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
        for ( adjacent_itr = _supervoxelData._supervoxelAdjacency.equal_range(supervoxel_label).first;
                adjacent_itr != _supervoxelData._supervoxelAdjacency.equal_range (supervoxel_label).second; adjacent_itr++) {
            uint32_t adjacentLabel = adjacent_itr->second;
            TupleTriplet adjacentColor = _map_segmentedSupervoxels[adjacentLabel];

            // If the color of the current label is the unsegmented color
            if (currentColor == colorMap._unsegmented_color) {
                // and if the color of the adjacent label is the unsegmented color
                if (adjacentColor == colorMap._unsegmented_color) {
                // add it to the trimmed map.
                    trimmedAdjacencyMap.insert(std::pair<uint32_t, uint32_t>(supervoxel_label, adjacentLabel));
                }
            }
            else {
                // One of them is segmented, so we aren't interested in traveling along that edge.
            }
        }
        //Move iterator forward to next label
        label_itr = _supervoxelData._supervoxelAdjacency.upper_bound(supervoxel_label);
    }
    LOG.DEBUG("Finished trimming adjacencies where one of the nodes is segmented.");
    return trimmedAdjacencyMap;
}
