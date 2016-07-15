

#include <map>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
// Supervoxel clustering from http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php
#include <pcl/PolygonMesh.h>
#include <pcl/common/distances.h>

#include "supervoxel_construction.h"
#include "dijkstraPathfinding.h"
#include "loggingHelper.h"


SupervoxelDataContainer::SupervoxelDataContainer() {
    LOG.DEBUG("Constructing SupervoxelDataContainer");
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointerForCloudWithPointsColorizedByLabel (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr pointerForCloudWithLabeledPoints (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr pointerForCloudWithCentroidsByLabel (new pcl::PointCloud<pcl::PointXYZL>);
    _cloudWithPointsColorizedByLabel = pointerForCloudWithPointsColorizedByLabel;
    _cloudWithLabeledPoints = pointerForCloudWithLabeledPoints;
    _cloudWithCentroidsByLabel = pointerForCloudWithCentroidsByLabel;
}

SupervoxelDataContainer::~SupervoxelDataContainer() { //empty destructor
}

void SupervoxelDataContainer::print_cloudSizes() {
    std::ostringstream logStream;
    logStream << "Size of _cloudWithPointsColorizedByLabel: " << _cloudWithPointsColorizedByLabel->size() << std::endl <<
                    "Size of _cloudWithLabeledPoints: " << _cloudWithLabeledPoints->size() << std::endl <<
                    "Size of _cloudWithCentroidsByLabel: " << _cloudWithCentroidsByLabel->size();
    LOG.DEBUG(logStream.str()); logStream.str("");
}

void SupervoxelDataContainer::print_supervoxelClusters() {
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator clusterItr;
    for (clusterItr = _supervoxelClusters.begin(); clusterItr != _supervoxelClusters.end(); clusterItr++) {
        std::cout << "_supervoxelClusters: " << clusterItr->first << " " << clusterItr->second->centroid_ << std::endl;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointsInSupervoxel (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pointsInSupervoxel = clusterItr->second->voxels_;
        std::cout << "size of voxels_ cloud from supervoxel: " << pointsInSupervoxel->size() << std::endl;
    }
}

void SupervoxelDataContainer::print_supervoxelAdjacency() {
    std::multimap<uint32_t,uint32_t>::iterator label_itr;
    for (label_itr = _supervoxelAdjacency.begin(); label_itr != _supervoxelAdjacency.end(); ) { //We increment this iterator at the end of the loop

        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        std::cout << "Supervoxel label " << supervoxel_label << " is adjacent to ";

        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = _supervoxelClusters.at(supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
        for ( adjacent_itr = _supervoxelAdjacency.equal_range(supervoxel_label).first;
            adjacent_itr != _supervoxelAdjacency.equal_range (supervoxel_label).second; adjacent_itr++) {
                std::cout << adjacent_itr->second << " ";
                pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = _supervoxelClusters.at(adjacent_itr->second);

            }
        //Move iterator forward to next label
        label_itr = _supervoxelAdjacency.upper_bound (supervoxel_label);
        std::cout << std::endl;
    }
}

void SupervoxelDataContainer::print_supervoxelAdjacency(uint32_t inputLabel) {
    std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
    for ( adjacent_itr = _supervoxelAdjacency.equal_range(inputLabel).first;
        adjacent_itr != _supervoxelAdjacency.equal_range (inputLabel).second; adjacent_itr++) {
            std::cout << adjacent_itr->second << " ";
    }
    std::cout << std::endl;
}

std::vector<uint32_t> SupervoxelDataContainer::returnVectorOfSupervoxelAdjacencyLabels(uint32_t inputLabel) {
    std::vector<uint32_t> labelVectorToReturn;
    std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
    for ( adjacent_itr = _supervoxelAdjacency.equal_range(inputLabel).first;
        adjacent_itr != _supervoxelAdjacency.equal_range (inputLabel).second; adjacent_itr++) {
            labelVectorToReturn.push_back(adjacent_itr->second);
    }
    return labelVectorToReturn;
}

void SupervoxelDataContainer::print_vv_colorVector() {
    std::cout << "Color vector elements: ";
    for (int i = 0; i < _vv_colorVector.size(); i++) {
        std::cout << "(";
        for (int j = 0; j < _vv_colorVector[i].size(); j++) {
            std::cout << _vv_colorVector[i][j] << ", ";
        }
        std::cout << ") ";
    }
    std::cout << std::endl;
}

void SupervoxelDataContainer::constructInternalMaps() {
    for (int i = 0; i < _cloudWithLabeledPoints->points.size(); i++) {
        pcl::PointXYZL pclPointXYZL= _cloudWithLabeledPoints->points[i];
        TupleTriplet tuplePointXYZ(pclPointXYZL.x, pclPointXYZL.y, pclPointXYZL.z);
        _map_pointsToLabel.insert(std::pair<TupleTriplet, uint32_t> (tuplePointXYZ, pclPointXYZL.label));
    }

    for (int i = 0; i < _cloudWithCentroidsByLabel->points.size(); i++) {
        pcl::PointXYZL pclPointXYZL = _cloudWithCentroidsByLabel->points[i];
        TupleTriplet tuplePointXYZ(pclPointXYZL.x, pclPointXYZL.y, pclPointXYZL.z);
        _map_labelToCentroidOfSupervoxel.insert(std::pair<uint32_t, TupleTriplet> (pclPointXYZL.label, tuplePointXYZ));
    }
}

SupervoxelDataContainer constructSupervoxels(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsInput,
                        pcl::PointCloud<pcl::Normal>::Ptr inputNormals,
                        InputParameters inputParams) {
    std::ostringstream logStream;
    SupervoxelDataContainer containerToReturn;

    // The Supervoxel construction expects color. Just to prevent any funny business, we fill it in with a uniform value for now.
    for (uint32_t i = 0; i < cloudPointsInput->points.size(); i++) {
        cloudPointsInput->points[i].r = 0;
        cloudPointsInput->points[i].g = 255;
        cloudPointsInput->points[i].b = 0;
        cloudPointsInput->points[i].a = 255;
    }

    //Supervoxel construction, most of which was lifted from http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php
    float voxel_resolution = inputParams.supervoxelClusteringParameters.getVoxelResolution();
    float seed_resolution = inputParams.supervoxelClusteringParameters.getSeedResolution();
    float color_importance = inputParams.supervoxelClusteringParameters.getColorImportance();
    float spatial_importance = inputParams.supervoxelClusteringParameters.getSpatialImportance();
    float normal_importance = inputParams.supervoxelClusteringParameters.getNormalImportance();

    // Setting up the workhorse class behind the supervoxel construction.
    pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_resolution, seed_resolution, false);

    super.setInputCloud(cloudPointsInput);
    super.setNormalCloud(inputNormals);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);

    // Constructing a few data structures that will hold the output of the construction.
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointsInSupervoxel (new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator clusterItr;

    LOG.DEBUG("Extracting supervoxels.", inputParams.debuggingParameters.getDebuggingLevel(), 1);

    // Declare and populate the clouds that we can pull the data we need from.
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudWithLabeledPoints (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudWithLabeledPointsWithZeroLabelsRemoved (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudWithPointsColorizedByLabel (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudWithCentroidsByLabel (new pcl::PointCloud<pcl::PointXYZL>);

    // This extracts the supervoxels, but things don't get correctly written. We create a new map that will be correctly populated.
    super.extract(supervoxel_clusters);
    cloudWithCentroidsByLabel = super.getLabeledVoxelCloud ();
    cloudWithLabeledPoints = super.getLabeledCloud();

    /// Handling edge case where a label of 0 exists:
    // There seems to be a rare edge case where a label of 0 is assigned to points as a label; it's only come up on one sample (133_2 10/03)
    // I don't think this is supposed to be the case; no other mesh out of 100's of meshes has had that, and the entire rest of the
    // code base is written under the assumption that no 0 labels exist.
    uint32_t minLabelCentroids = 10000000; //arbitrarily large magic number
    uint32_t maxLabelCentroids = 0; //minimum value
    uint32_t minLabelPoints = 10000000; //arbitrarily large magic number
    uint32_t maxLabelPoints = 0; //minimum value
    for (uint32_t i = 0; i < cloudWithCentroidsByLabel->points.size(); i++) {
        pcl::PointXYZL labeledCentroid = cloudWithCentroidsByLabel->points[i];
        uint32_t centroidLabel = labeledCentroid.label;
        if (centroidLabel > maxLabelCentroids) {
            maxLabelCentroids = centroidLabel;
        }
        if (centroidLabel < minLabelCentroids) {
            minLabelCentroids = centroidLabel;
        }
    }

    for (uint32_t i = 0; i < cloudWithLabeledPoints->points.size(); i++) {
        pcl::PointXYZL labeledPoint = cloudWithLabeledPoints->points[i];
        uint32_t pointLabel = labeledPoint.label;

        if (pointLabel == 0) {
            LOG.DEBUG("WARNING\nWARNING: A point that has a label of 0 was found after supervoxel construction.\nWARNING");
            LOG.DEBUG("WARNING: Assigning it the label of the closest labeled point. This does not change adjacencies.");
            uint32_t localLabel = 1;
            float minimumDistance = 1000000.0; //Arbitrarily large magic number
            for (uint32_t j = 0; j < cloudWithLabeledPoints->points.size(); j++) {
                pcl::PointXYZL localPoint = cloudWithLabeledPoints->points[j];
                uint32_t localPointLabel = localPoint.label;
                float pairwiseDistance = pcl::euclideanDistance(labeledPoint, localPoint);
                if (pairwiseDistance < minimumDistance && localPointLabel != 0) {
                    localLabel = localPointLabel;
                    minimumDistance = pairwiseDistance;
                }

            }
            cloudWithLabeledPoints->points[i].label = localLabel;
        }
        labeledPoint = cloudWithLabeledPoints->points[i];
        pointLabel = labeledPoint.label;
        if (pointLabel > maxLabelPoints) {
            maxLabelPoints = pointLabel;
        }
        if (pointLabel < minLabelPoints) {
            minLabelPoints = pointLabel;
        }

    }

    logStream << cloudWithCentroidsByLabel->points.size() << " centroids/supervoxel labels were found from a total of " <<
                    cloudWithLabeledPoints->points.size() << " points in the original input.\n";
    logStream << "The minimum and maximum labels of the centroid cloud were " << minLabelCentroids << " and " << maxLabelCentroids << "." <<
                    "\nThe minimum and maximum labels of the point cloud were " << minLabelPoints << " and " << maxLabelPoints << ".";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    assert(minLabelCentroids > 0 && "A label of 0 exists after supervoxel construction; this should not be the case. Aborting.");
    assert(minLabelPoints > 0 && "A label of 0 exists after supervoxel construction; this should not be the case. Aborting.");
    /// Finished handling edge case with label of 0.

    LOG.DEBUG("Supervoxels extracted. Populating data structures.", inputParams.debuggingParameters.getDebuggingLevel(), 1);

    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxelClusters_reconstructed;

    containerToReturn._cloudWithCentroidsByLabel = cloudWithCentroidsByLabel;
    containerToReturn._cloudWithLabeledPoints = cloudWithLabeledPoints;

    // Based on the number of labels (i.e. number of supervoxels), populate a vector with that many random RGB triplets, and
    // populate the new map with that many labels keyed to that many Supervoxels.
    LOG.DEBUG("Populating supervoxel map.", inputParams.debuggingParameters.getDebuggingLevel(), 2);
    std::vector< std::vector< uint8_t > > vv_colorVector;
    for (uint32_t i = 0; i < super.getLabeledVoxelCloud()->size(); i++ ) {
        std::vector<uint8_t> RGB;
        RGB.push_back(rand()%255);
        RGB.push_back(rand()%255);
        RGB.push_back(rand()%255);
        vv_colorVector.push_back(RGB);
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr emptySupervoxel (new pcl::Supervoxel<pcl::PointXYZRGBA>);
        supervoxelClusters_reconstructed.insert(std::pair<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > (i + 1, emptySupervoxel ));
    }

    // Populate the empty Supervoxels in the map with their respective points. We also populated a regular
    // cloud for downstream visualization.
    // Check if we can do all this with a constructor for a pcl::PointXYZRGBA if this stays in the code.
    LOG.DEBUG("Populating supervoxel points.", inputParams.debuggingParameters.getDebuggingLevel(), 2);
    for (uint32_t i = 0; i < cloudWithLabeledPoints->points.size(); i++) {
        pcl::PointXYZL pointLabel = cloudWithLabeledPoints->points[i];
        pcl::PointXYZRGBA pointRGB;
        uint32_t labelOfSupervoxel = pointLabel.label;

        std::vector<uint8_t> colorVector;
        colorVector = vv_colorVector[labelOfSupervoxel - 1]; // labels aren't 0 indexed.

        pointRGB.x = pointLabel.x;
        pointRGB.y = pointLabel.y;
        pointRGB.z = pointLabel.z;
        pointRGB.r = colorVector[0];
        pointRGB.g = colorVector[1];
        pointRGB.b = colorVector[2];
        pointRGB.a = 255;
        cloudWithPointsColorizedByLabel->push_back(pointRGB);
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr currentSupervoxel = supervoxelClusters_reconstructed.at(labelOfSupervoxel);
        currentSupervoxel->voxels_->push_back(pointRGB);
    }

    containerToReturn._cloudWithPointsColorizedByLabel = cloudWithPointsColorizedByLabel;

    // Now that the Supervoxels are populated, calculate the centroid with their points.
    LOG.DEBUG("Caculating supervoxel centroids.", inputParams.debuggingParameters.getDebuggingLevel(), 2);
    for (clusterItr = supervoxelClusters_reconstructed.begin(); clusterItr != supervoxelClusters_reconstructed.end(); clusterItr++) {
        Eigen::Vector4f centroid;
        uint32_t labelOfSupervoxel = clusterItr->first;
        std::vector<uint8_t> colorVector = vv_colorVector[labelOfSupervoxel - 1]; // labels aren't 0 indexed.
        compute3DCentroid (*clusterItr->second->voxels_, centroid);
        pcl::PointXYZRGBA centroidPoint;
        centroidPoint.x = centroid[0];
        centroidPoint.y = centroid[1];
        centroidPoint.z = centroid[2];
        centroidPoint.r = colorVector[0];
        centroidPoint.g = colorVector[1];
        centroidPoint.b = colorVector[2];
        centroidPoint.a = 255;
        clusterItr->second->centroid_ = centroidPoint;
    }

    containerToReturn._supervoxelClusters = supervoxelClusters_reconstructed;

    LOG.DEBUG("Getting supervoxel adjacency.");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);
    containerToReturn._supervoxelAdjacency = supervoxel_adjacency;

    containerToReturn.constructInternalMaps();

    return containerToReturn;
}


void SupervoxelDataContainer::trimAdjacencyMapOnGeodesicDistance(InputParameters inputParams, pcl::PolygonMesh *inputMesh) {
    std::ostringstream logStream;
    // So we want to set up the input mesh as a series of nodes, distances, and connections for Dijkstra.
    // We should be able to take the std::vector< std::vector <int> > and make neighbors out of them.

    adjacency_list_t inputMeshAdjacencies = convertPolygonMeshToAdjacencyList(inputMesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(inputMesh->cloud, *cloudPoints);

    std::map<TupleTriplet, int> map_coordinatesToIndex;
    std::map<int, TupleTriplet> map_indexToCoordinates;
    for (int i = 0; i < cloudPoints->points.size(); i++) {
        TupleTriplet currentPoint = convertPclPointXYZtoTupleTriplet(cloudPoints->points[i]);
        map_coordinatesToIndex.insert(std::pair<TupleTriplet, int> (currentPoint, i));
        map_indexToCoordinates.insert(std::pair<int, TupleTriplet> (i, currentPoint));
    }

    // With the adjacencies of the mesh in hand, we need to iterate over the supervoxels, and
    // for each adjacent supervoxel, get the closest mesh point for its centroid, and find the
    // dijkstra distance for the two points.
    LOG.DEBUG("Trimming supervoxel adjacency map.");
    std::multimap<uint32_t,uint32_t>::iterator label_itr;
    std::multimap<uint32_t, uint32_t> trimmedAdjacencyMap;
    float averageDistanceBetweenSupervoxels = 0;
    float maxDistanceBetweenSupervoxels = 0;
    int totalIterations = 0;
    float totalDistanceBetweenSupervoxels = 0;
    int supervoxelCounter = 0;
    // This proportion has a huge impact on segmentation performance and should be refactored to be an input parameter.
    float MAGIC_PROPORTION_OF_SUPERVOXEL_SIZE = 1.15;  // This magic number is proportion larger than the supervoxel diagonal that an adjacency can be.
    //Set the maximum allowed distance to be the MAGIC_PROPORTION_OF_SUPERVOXEL_SIZE larger than the length of the diagonal of a voxel of the given resolution.
    float distanceThreshold = (inputParams.supervoxelClusteringParameters.getVoxelResolution() * sqrt(3) * MAGIC_PROPORTION_OF_SUPERVOXEL_SIZE); //1.10);
    // We also need to keep track of the labels found in the adjacency map since singletons can occur. We'll keep them in a map that we can search through later.
    std::map<uint32_t, bool> map_labelsWithAdjacencies;
    bool adjacencyFound = false;
    for (label_itr = _supervoxelAdjacency.begin(); label_itr != _supervoxelAdjacency.end(); ) { //We increment this iterator at the end of the loop
        supervoxelCounter = supervoxelCounter + 1;
        adjacencyFound = false;
        if (supervoxelCounter % 100 == 0) {
            logStream << "\tOn supervoxel: " << supervoxelCounter;
            LOG.DEBUG(logStream.str()); logStream.str("");

        }
        //First get the label
        uint32_t supervoxel_label = label_itr->first;

        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr currentSupervoxel = _supervoxelClusters.at(supervoxel_label);
        pcl::PointXYZ currentSupervoxelCentroid = returnClosestPointToCentroidGivenSupervoxel(currentSupervoxel);

        /// Hmm... we need the index of the point to be able to search for it in the mesh. I guess we need two maps, one mapping index to
        /// coordinate, and one mapping coordinate to index.

        // Get the index of the found point.
        TupleTriplet tuple_currentSupervoxelCentroid = convertPclPointXYZtoTupleTriplet(currentSupervoxelCentroid);
        int indexOfFirstCentroid = map_coordinatesToIndex[tuple_currentSupervoxelCentroid];

        std::vector<weight_t> min_distance;
        std::vector<vertex_t> previous;
        DijkstraComputePaths(indexOfFirstCentroid, inputMeshAdjacencies, min_distance, previous);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
        for ( adjacent_itr = _supervoxelAdjacency.equal_range(supervoxel_label).first;
                adjacent_itr != _supervoxelAdjacency.equal_range (supervoxel_label).second; adjacent_itr++) {
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr adjacentSupervoxel = _supervoxelClusters.at(adjacent_itr->second);
            pcl::PointXYZ adjacentSupervoxelCentroid = returnClosestPointToCentroidGivenSupervoxel(adjacentSupervoxel);

            TupleTriplet tuple_adjacentSupervoxelCentroid = convertPclPointXYZtoTupleTriplet(adjacentSupervoxelCentroid);
            int indexOfAdjacentCentroid = map_coordinatesToIndex[tuple_adjacentSupervoxelCentroid];

            float pathLength = min_distance[indexOfAdjacentCentroid];

            if (pathLength < distanceThreshold) {
                trimmedAdjacencyMap.insert(std::pair<uint32_t, uint32_t>(supervoxel_label, adjacent_itr->second));
                adjacencyFound = true;
            }

            if (pathLength > maxDistanceBetweenSupervoxels) {
                maxDistanceBetweenSupervoxels = pathLength;
            }
            totalIterations = totalIterations + 1;
            totalDistanceBetweenSupervoxels = totalDistanceBetweenSupervoxels + pathLength;

        }
        // If an adjacency was found, add it to the list of labels with adjacencies.
        if (adjacencyFound == true) {
            map_labelsWithAdjacencies.insert(std::pair<uint32_t, bool>(supervoxel_label, true));
        }

        //Move iterator forward to next label
        label_itr = _supervoxelAdjacency.upper_bound(supervoxel_label);
    }

    // In rare cases, a label can exist without any adjacencies. We'll iterate through the vector of labels with adjacencies and compare it to the label map to
    // identify these labels, and then make an adjacency with the nearest euclidean supervoxel.
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator labelExistsItr;
    for (labelExistsItr = _supervoxelClusters.begin(); labelExistsItr != _supervoxelClusters.end(); labelExistsItr++ ) {
        uint32_t labelToCheck = labelExistsItr->first;
        if (map_labelsWithAdjacencies.find(labelToCheck) == map_labelsWithAdjacencies.end()) {
            LOG.DEBUG("WARNING\nWARNING: A supervoxel without any adjacency was found. Consider relaxing supervoxel construction parameters. Connecting it to the nearest supervoxel.\nWARNING");

            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr currentSupervoxel = _supervoxelClusters.at(labelToCheck);
            pcl::PointXYZ currentSupervoxelCentroid = returnClosestPointToCentroidGivenSupervoxel(currentSupervoxel);

            std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator clusterItr;
            float minimumDistance = 1000000.0; //Arbitrarily large distance
            uint32_t minimumLabel = 1;
            for (clusterItr = _supervoxelClusters.begin(); clusterItr != _supervoxelClusters.end(); clusterItr++) {
                pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxelToTest = clusterItr->second;
                pcl::PointXYZ supervoxelToTestCentroid = returnClosestPointToCentroidGivenSupervoxel(supervoxelToTest);
                float distance = pcl::euclideanDistance(currentSupervoxelCentroid, supervoxelToTestCentroid);
                if (distance < minimumDistance && map_labelsWithAdjacencies.find(clusterItr->first) != map_labelsWithAdjacencies.end()) {
                    minimumDistance = distance;
                    minimumLabel = clusterItr->first;
                }
            }
            logStream << "WARNING: Label " << labelToCheck << " did not have any adjacencies. Connecting it to closest supervoxel at label " << minimumLabel;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            trimmedAdjacencyMap.insert(std::pair<uint32_t, uint32_t>(labelToCheck, minimumLabel));
            supervoxelCounter += 1;
        }
    }

    logStream << "Finished trimming adjacencies for a total of " << supervoxelCounter << " supervoxel labels.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    _supervoxelAdjacency = trimmedAdjacencyMap;
    averageDistanceBetweenSupervoxels = totalDistanceBetweenSupervoxels / totalIterations;
    logStream << "\tMax geodesic distance between adjacent supervoxels: " << maxDistanceBetweenSupervoxels << "\n";
    logStream << "\tAverage geodesic distance length between adjacent supervoxels: " << averageDistanceBetweenSupervoxels;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");


}

pcl::PointXYZ returnClosestPointToCentroidGivenSupervoxel(pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr inputSupervoxel) {
    Eigen::Vector4f centroid;
    compute3DCentroid(*(inputSupervoxel->voxels_), centroid);
    pcl::PointXYZ centroidPointXYZ(centroid[0], centroid[1], centroid[2]);
    float minimumDistance = 100000000;  //Arbitrarily large magic number.
    pcl::PointXYZRGBA firstPoint = inputSupervoxel->voxels_->points[0];
    pcl::PointXYZ minimumPoint(firstPoint.x, firstPoint.y, firstPoint.z);
    for (int i = 0; i < inputSupervoxel->voxels_->points.size(); i++) {
        pcl::PointXYZRGBA point = inputSupervoxel->voxels_->points[i];
        pcl::PointXYZ pointToCompare(point.x, point.y, point.z);
        float currentDistance = pcl::euclideanDistance(centroidPointXYZ, pointToCompare);
        if (currentDistance < minimumDistance) {
            minimumDistance = currentDistance;
            minimumPoint = pointToCompare;
        }
    }
    //std::cout << "Minimum distance point from centroid of supervoxel to point in supervoxel is: " << minimumPoint << std::endl;
    return minimumPoint;
}
