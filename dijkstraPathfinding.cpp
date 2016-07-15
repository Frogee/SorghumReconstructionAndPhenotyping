

#include <iostream>
#include <vector>
#include <string>
#include <list>
#include <limits> // for numeric_limits
#include <set>
#include <utility> // for pair
#include <algorithm>
#include <iterator>

#include <pcl/common/distances.h>

#include "dijkstraPathfinding.h"
#include "plantSegmentationDataContainer.h"
#include "loggingHelper.h"


/** Graph class
  */
Graph::Graph() {
    //empty constructor
}

/** Initializes a graph given input node adjacencies and input coordinates of those nodes (to find edge lengths) by calling buildGraph()
  */
Graph::Graph(std::multimap<uint32_t, uint32_t> inputNodeLabelAdjacency, std::map<uint32_t, TupleTriplet> inputNodeLabelToPointCoordinate) {
    _map_nodeLabelAdjacency = inputNodeLabelAdjacency;
    _map_nodeLabelToPointCoordinate = inputNodeLabelToPointCoordinate;
    this->buildGraph();
}

/** Initializes a graph using an input pcl::PolygonMesh
  */
Graph::Graph(pcl::PolygonMesh *inputMesh) {
    //If we receive this for the constructor, we just assume that node names are the same as indices.
    std::cout << "Converting a pcl::PolygonMesh to a Graph" << std::endl;

    std::multimap<uint32_t, uint32_t> nodeLabelAdjacency;
    std::map<uint32_t, TupleTriplet> nodeLabelToPointCoordinate;
    std::map<uint32_t, uint32_t> nodeLabelToListIndex;
    std::map<uint32_t, uint32_t> listIndexToNodeLabel;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(inputMesh->cloud, *cloudPoints);

    adjacency_list_t adjacencyList(cloudPoints->points.size());
    for (uint i = 0; i < inputMesh->polygons.size(); i++) {
        ///inputMesh->polygons[i] will return a 3-vector of three point indices.
        ///The point index will work as the node name, but we need the actual point
        ///To determine euclidean distance. It should correspond to the index of the point cloud.
        pcl::Vertices triangle = inputMesh->polygons[i];
        assert(triangle.vertices.size() == 3 && "There are not three vertices in the vertex.");
        uint32_t firstVertexIndex = triangle.vertices[0];
        uint32_t secondVertexIndex = triangle.vertices[1];
        uint32_t thirdVertexIndex = triangle.vertices[2];

        pcl::PointXYZ firstPointCoords = cloudPoints->points[firstVertexIndex];
        pcl::PointXYZ secondPointCoords = cloudPoints->points[secondVertexIndex];
        pcl::PointXYZ thirdPointCoords = cloudPoints->points[thirdVertexIndex];

        float distOneToTwo = pcl::euclideanDistance(firstPointCoords, secondPointCoords);
        float distOneToThree = pcl::euclideanDistance(firstPointCoords, thirdPointCoords);
        float distTwoToThree = pcl::euclideanDistance(secondPointCoords, thirdPointCoords);

        adjacencyList[firstVertexIndex].push_back(neighbor(secondVertexIndex, distOneToTwo));
        adjacencyList[firstVertexIndex].push_back(neighbor(thirdVertexIndex, distOneToThree));

        adjacencyList[secondVertexIndex].push_back(neighbor(firstVertexIndex, distOneToTwo));
        adjacencyList[secondVertexIndex].push_back(neighbor(thirdVertexIndex, distTwoToThree));

        adjacencyList[thirdVertexIndex].push_back(neighbor(firstVertexIndex, distOneToThree));
        adjacencyList[thirdVertexIndex].push_back(neighbor(secondVertexIndex, distTwoToThree));

        nodeLabelToPointCoordinate.insert(std::pair<uint32_t, TupleTriplet>(firstVertexIndex, convertPclPointXYZtoTupleTriplet(firstPointCoords)));
        nodeLabelToPointCoordinate.insert(std::pair<uint32_t, TupleTriplet>(secondVertexIndex, convertPclPointXYZtoTupleTriplet(secondPointCoords)));
        nodeLabelToPointCoordinate.insert(std::pair<uint32_t, TupleTriplet>(thirdVertexIndex, convertPclPointXYZtoTupleTriplet(thirdPointCoords)));

        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(firstVertexIndex, secondVertexIndex));
        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(firstVertexIndex, thirdVertexIndex));

        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(secondVertexIndex, firstVertexIndex));
        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(secondVertexIndex, thirdVertexIndex));

        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(thirdVertexIndex, firstVertexIndex));
        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(thirdVertexIndex, secondVertexIndex));

        nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t>(firstVertexIndex, firstVertexIndex));
        listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t>(firstVertexIndex, firstVertexIndex));

        nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t>(secondVertexIndex, secondVertexIndex));
        listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t>(secondVertexIndex, secondVertexIndex));

        nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t>(thirdVertexIndex, thirdVertexIndex));
        listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t>(thirdVertexIndex, thirdVertexIndex));
    }

    _map_nodeLabelToPointCoordinate = nodeLabelToPointCoordinate;
    _map_nodeLabelAdjacency = nodeLabelAdjacency;
    _map_nodeLabelToListIndex = nodeLabelToListIndex;
    _map_listIndexToNodeLabel = listIndexToNodeLabel;
    _adjacencyList = adjacencyList;

}

Graph::~Graph() {
    //empty destructor
}

/** Initializes a graph given input node adjacencies and input coordinates of those nodes (to find edge lengths); called from the Graph constructor.
  */
int Graph::buildGraph() {
    LOG.DEBUG("Building Graph for pathfinding.");
    assert(_map_nodeLabelAdjacency.size() > 0 && "The Graph has no adjacencies; this shouldn't be the case. Aborting.");
    assert(_map_nodeLabelToPointCoordinate.size() > 0 && "The Graph has no labels with point coordinates; this shouldn't be the case. Aborting.");

    // To use Dijkstra's algorithm, we'll need the nodes, their adjacency, and the edge weights between them.

    // We'll use the supervoxel labels as the nodes, their adjacency, and the distance between the points as the edge weights.
    std::multimap<uint32_t, uint32_t> adjacencyMap = _map_nodeLabelAdjacency;
    std::map<uint32_t, TupleTriplet> labelToCoordinates = _map_nodeLabelToPointCoordinate;
    std::multimap<uint32_t, uint32_t>::iterator label_itr;

    int numberOfUniqueValuesInMap = 0;
    for (label_itr = adjacencyMap.begin(); label_itr != adjacencyMap.end(); ) { //We increment this iterator at the end of the loop
        uint32_t supervoxel_label = label_itr->first;
        numberOfUniqueValuesInMap += 1;
        label_itr = adjacencyMap.upper_bound (supervoxel_label);
    }
    //std::cout << "The adjacency map to be traversed is of size: " << adjacencyMap.size() << " with " << numberOfUniqueValuesInMap << " unique values." << std::endl;

    adjacency_list_t adjacencyList(numberOfUniqueValuesInMap);
    int indexCounter = 0;
    for (label_itr = adjacencyMap.begin(); label_itr != adjacencyMap.end(); ) { //We increment this iterator at the end of the loop
        //First get the label
        uint32_t nodeLabel = label_itr->first;
        //std::cout << "Label " << nodeLabel << " has coordinate of ";
        TupleTriplet firstCoordinate = labelToCoordinates.at(nodeLabel);
        //printTupleTriplet(firstCoordinate);

        //Now we need to iterate through the adjacent labels to construct neighbors
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
        for ( adjacent_itr = adjacencyMap.equal_range(nodeLabel).first;
            adjacent_itr != adjacencyMap.equal_range(nodeLabel).second; adjacent_itr++) {
                uint32_t adjacentNodeLabel = adjacent_itr->second;
                TupleTriplet secondCoordinate = labelToCoordinates.at(adjacent_itr->second);
                pcl::PointXYZ pclFirstCoordinate = convertTupleTriplettoPclPointXYZ(firstCoordinate);
                pcl::PointXYZ pclSecondCoordinate = convertTupleTriplettoPclPointXYZ(secondCoordinate);
                float pairwiseDistance = pcl::euclideanDistance(pclFirstCoordinate, pclSecondCoordinate);
                weight_t distance = pairwiseDistance;
                //std::cout << "Node " << nodeLabel << " is adjacent to node " << adjacent_itr->second << "." << std::endl;
                // Now we need to get the indices of these labels in the adjacency list.
                //std::map<uint32_t, uint32_t> _map_nodeLabelToListIndex
                // if neither of the labels have been assigned indices.
                if (_map_nodeLabelToListIndex.find(nodeLabel) == _map_nodeLabelToListIndex.end() && _map_nodeLabelToListIndex.find(adjacentNodeLabel) == _map_nodeLabelToListIndex.end()) {
                    // They both need to be assigned indices.
                    uint32_t indexNodeLabel = indexCounter;
                    indexCounter += 1;
                    uint32_t indexAdjacentNodeLabel = indexCounter;
                    indexCounter += 1;

                    _map_nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t> (nodeLabel, indexNodeLabel));
                    _map_listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t> (indexNodeLabel, nodeLabel));

                    _map_nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t> (adjacentNodeLabel, indexAdjacentNodeLabel));
                    _map_listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t> (indexAdjacentNodeLabel, adjacentNodeLabel));

                    // Once they've been assigned indices, we can add the neighbors of the current node.
                    adjacencyList[indexNodeLabel].push_back(neighbor(indexAdjacentNodeLabel, distance));
                }
                // if only the current node has been assigned an index and the adjacent has not.
                else if (_map_nodeLabelToListIndex.find(nodeLabel) != _map_nodeLabelToListIndex.end() && _map_nodeLabelToListIndex.find(adjacentNodeLabel) == _map_nodeLabelToListIndex.end()) {
                    // The adjacent node needs to be assigned an index.
                    uint32_t indexAdjacentNodeLabel = indexCounter;
                    indexCounter += 1;

                    _map_nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t> (adjacentNodeLabel, indexAdjacentNodeLabel));
                    _map_listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t> (indexAdjacentNodeLabel, adjacentNodeLabel));

                    // Once they've been assigned indices, we can add the neighbors of the current node.
                    uint32_t indexCurrentNode = _map_nodeLabelToListIndex[nodeLabel];
                    adjacencyList[indexCurrentNode].push_back(neighbor(indexAdjacentNodeLabel, distance));
                }
                // if the current node has not been assigned an index, but the adjacent node has.
                else if (_map_nodeLabelToListIndex.find(nodeLabel) == _map_nodeLabelToListIndex.end() && _map_nodeLabelToListIndex.find(adjacentNodeLabel) != _map_nodeLabelToListIndex.end()) {
                    //The current node needs to be assigned an index:
                    uint32_t indexNodeLabel = indexCounter;
                    indexCounter += 1;

                    _map_nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t> (nodeLabel, indexNodeLabel));
                    _map_listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t> (indexNodeLabel, nodeLabel));

                    uint32_t indexAdjacentNode = _map_nodeLabelToListIndex[adjacentNodeLabel];
                    adjacencyList[indexNodeLabel].push_back(neighbor(indexAdjacentNode, distance));
                }
                // if they've both already been assigned an index
                else if (_map_nodeLabelToListIndex.find(nodeLabel) != _map_nodeLabelToListIndex.end() && _map_nodeLabelToListIndex.find(adjacentNodeLabel) != _map_nodeLabelToListIndex.end()) {
                    uint32_t indexCurrentNode = _map_nodeLabelToListIndex[nodeLabel];
                    uint32_t indexAdjacentNode = _map_nodeLabelToListIndex[adjacentNodeLabel];

                    adjacencyList[indexCurrentNode].push_back(neighbor(indexAdjacentNode, distance));
                }
                else {
                    bool unhandledGraphCondition = true;
                    assert(unhandledGraphCondition == false && "Reached an unhandled condition when constructing the graph. Aborting.");
                }
            }
        //Move iterator forward to next label
        label_itr = adjacencyMap.upper_bound (nodeLabel);
        //std::cout << std::endl;
    }
    _adjacencyList = adjacencyList;
    return 0;
}

/** Sets the adacency of nodes in the graph.
  */
int Graph::setNodeLabelAdjacency(std::multimap<uint32_t, uint32_t> inputNodeLabelAdjacency) {
    LOG.DEBUG("Setting node label adjacency");
    _map_nodeLabelAdjacency = inputNodeLabelAdjacency;
    this->buildGraph();
    return 0;
}

adjacency_list_t Graph::getAdjacencyList()  {
    return _adjacencyList;
}


// end Graph class

PathDataContainer::PathDataContainer() {
    //empty constructor
}
PathDataContainer::~PathDataContainer() {
    //empty constructor
}


/** DijkstraPathfinder class
  */
DijkstraPathfinder::DijkstraPathfinder() {
    //empty constructor
}

DijkstraPathfinder::DijkstraPathfinder(Graph inputGraphToTraverse) {
    _graphToTraverse = inputGraphToTraverse;
}

DijkstraPathfinder::~DijkstraPathfinder() {
    //empty destructor
}

int DijkstraPathfinder::setGraphToTraverse(Graph inputGraphToTraverse) {
    _graphToTraverse = inputGraphToTraverse;
}

PathDataContainer DijkstraPathfinder::findPathToFurthestUnsegmentedSupervoxel(int inputLabel, PlantSegmentationDataContainer *inputSegmentationData) {

    assert(inputSegmentationData->_map_segmentedSupervoxels.size() == _graphToTraverse._map_nodeLabelToListIndex.size() &&
                "The number of supervoxels is not equal to the number of nodes in the graph. Did the graph come from these supervoxels, or does the supervoxel segmentation need to be updated?");

    //std::cout << "length of the adjacency list: " << _graphToTraverse._adjacencyList.size() << std::endl;
    //std::cout << "length of the listIndexToNodeLabel map: " << _graphToTraverse._map_listIndexToNodeLabel.size() << std::endl;
    assert(_graphToTraverse._adjacencyList.size() == _graphToTraverse._map_listIndexToNodeLabel.size() &&
                "The number of indices in the adjacencyList is not equal to the number indices mapped to node labels. Something went wrong with building the Graph.");

    ColorMap colorMap;

    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    vertex_t indexOfLabel = _graphToTraverse._map_nodeLabelToListIndex[inputLabel];

    //std::cout << "Finding path to furthest unsegmented supervoxel for node with label " << inputLabel << " with index " << indexOfLabel << std::endl;

    computePaths(indexOfLabel, _graphToTraverse._adjacencyList, min_distance, previous);

    // We have the distances between the labels, now we need to be able to determine if the label is segmented.
    inputSegmentationData->updateSupervoxelSegmentationMap();

    weight_t maximumDistance = 0.0; //Hold the maximum of the minimum distances
    int indexWithMaxDistance = 0;
    for (int i = 0; i < min_distance.size(); i++) {
        int currentLabel = _graphToTraverse._map_listIndexToNodeLabel[i];
        TupleTriplet colorOfSupervoxel = inputSegmentationData->_map_segmentedSupervoxels[currentLabel];
        if (min_distance[i] > maximumDistance && colorOfSupervoxel == colorMap._unsegmented_color) {
            maximumDistance = min_distance[i];
            indexWithMaxDistance = i;
        }
    }
    uint32_t labelOfMaxDistanceNode = _graphToTraverse._map_listIndexToNodeLabel[indexWithMaxDistance];

    //std::cout << "Distance from label " << inputLabel << " (index " << indexOfLabel << ") to label " << labelOfMaxDistanceNode << " (index " << indexWithMaxDistance << ") is " << min_distance[indexWithMaxDistance] << std::endl;
    std::list<vertex_t> path = DijkstraGetShortestPathTo(indexWithMaxDistance, previous);
    //std::cout << "Path of the indices: ";
    //std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    //std::cout << std::endl;

    PathDataContainer containerToReturn;
    containerToReturn._sourceIndex = indexOfLabel;
    containerToReturn._sourceLabel = inputLabel;

    containerToReturn._targetIndex = indexWithMaxDistance;
    containerToReturn._targetLabel = labelOfMaxDistanceNode;

    containerToReturn._distance = min_distance[indexWithMaxDistance];

    containerToReturn._pathListOfIndices = path;
    std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
    std::vector<uint32_t> vectorOfLabels;
    for (int i = 0; i < vectorOfIndices.size(); i++) {
        vertex_t currentIndex = vectorOfIndices[i];
        vectorOfLabels.push_back(_graphToTraverse._map_listIndexToNodeLabel[currentIndex]);
    }
    containerToReturn._pathVectorOfLabels = vectorOfLabels;

    //std::cout << "Returning container." << std::endl;
    return containerToReturn;
}

PathDataContainer DijkstraPathfinder::findPathToClosestUnsegmentedSupervoxel(int inputLabel, PlantSegmentationDataContainer *inputSegmentationData) {


    assert(_graphToTraverse._adjacencyList.size() == _graphToTraverse._map_listIndexToNodeLabel.size() &&
                "The number of indices in the adjacencyList is not equal to the number indices mapped to node labels. Something went wrong with building the Graph.");

    ColorMap colorMap;

    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    vertex_t indexOfLabel = _graphToTraverse._map_nodeLabelToListIndex[inputLabel];

    //std::cout << "Finding path to closest unsegmented supervoxel path for node with label " << inputLabel << " with index " << indexOfLabel << std::endl;

    computePaths(indexOfLabel, _graphToTraverse._adjacencyList, min_distance, previous);

    // We have the distances between the labels, now we need to be able to determine if the label is segmented.
    inputSegmentationData->updateSupervoxelSegmentationMap();

    weight_t minimumDistance = 1000000.0; // Arbitrarily large magic number
    int indexWithMinimumDistance = 0;
    for (int i = 0; i < min_distance.size(); i++) {
        int currentLabel = _graphToTraverse._map_listIndexToNodeLabel[i];
        TupleTriplet colorOfSupervoxel = inputSegmentationData->_map_segmentedSupervoxels[currentLabel];
        if (min_distance[i] < minimumDistance && colorOfSupervoxel == colorMap._unsegmented_color) {
            minimumDistance = min_distance[i];
            indexWithMinimumDistance = i;
        }
    }
    uint32_t labelOfMinimumDistanceNode = _graphToTraverse._map_listIndexToNodeLabel[indexWithMinimumDistance];

    //std::cout << "For path to the nearest stem label, distance from label " << inputLabel << " (index " << indexOfLabel << ") to label " << labelOfMinimumDistanceNode << " (index " << indexWithMinimumDistance << ") is " << min_distance[indexWithMinimumDistance] << std::endl;
    std::list<vertex_t> path = DijkstraGetShortestPathTo(indexWithMinimumDistance, previous);
    //std::cout << "Path of the indices: ";
    //std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    //std::cout << std::endl;

    PathDataContainer containerToReturn;
    containerToReturn._sourceIndex = indexOfLabel;
    containerToReturn._sourceLabel = inputLabel;

    containerToReturn._targetIndex = indexWithMinimumDistance;
    containerToReturn._targetLabel = labelOfMinimumDistanceNode;

    containerToReturn._pathListOfIndices = path;
    std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
    std::vector<uint32_t> vectorOfLabels;
    for (int i = 0; i < vectorOfIndices.size(); i++) {
        vertex_t currentIndex = vectorOfIndices[i];
        vectorOfLabels.push_back(_graphToTraverse._map_listIndexToNodeLabel[currentIndex]);
    }
    containerToReturn._pathVectorOfLabels = vectorOfLabels;

    //std::cout << "Returning container." << std::endl;
    return containerToReturn;
}

PathDataContainer DijkstraPathfinder::findPathToClosestStemPoint(uint32_t inputLabel, PlantSegmentationDataContainer *inputSegmentationData) {

    assert(_graphToTraverse._adjacencyList.size() == _graphToTraverse._map_listIndexToNodeLabel.size() &&
                "The number of indices in the adjacencyList is not equal to the number indices mapped to node labels. Something went wrong with building the Graph.");

    ColorMap colorMap;

    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    vertex_t indexOfLabel = _graphToTraverse._map_nodeLabelToListIndex[inputLabel];

    //std::cout << "Finding path for closest stem point for node with label " << inputLabel << " with index " << indexOfLabel << std::endl;

    computePaths(indexOfLabel, _graphToTraverse._adjacencyList, min_distance, previous);

    // We have the distances between the labels, now we need to be able to determine if the label is segmented.
    inputSegmentationData->updateSupervoxelSegmentationMap();

    weight_t closestStemDistance = 1000000.0; // Arbitrarily large magic number
    int indexWithMinimumDistance = 0;
    for (int i = 0; i < min_distance.size(); i++) {
        int currentLabel = _graphToTraverse._map_listIndexToNodeLabel[i];
        TupleTriplet colorOfSupervoxel = inputSegmentationData->_map_segmentedSupervoxels[currentLabel];
        if (min_distance[i] < closestStemDistance && colorOfSupervoxel == colorMap._stem_color) {
            closestStemDistance = min_distance[i];
            indexWithMinimumDistance = i;
        }
    }
    uint32_t labelOfMinimumDistanceNode = _graphToTraverse._map_listIndexToNodeLabel[indexWithMinimumDistance];

    //std::cout << "For path to the nearest stem label, distance from label " << inputLabel << " (index " << indexOfLabel << ") to label " << labelOfMinimumDistanceNode << " (index " << indexWithMinimumDistance << ") is " << min_distance[indexWithMinimumDistance] << std::endl;
    std::list<vertex_t> path = DijkstraGetShortestPathTo(indexWithMinimumDistance, previous);
    //std::cout << "Path of the indices: ";
    //std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    //std::cout << std::endl;

    PathDataContainer containerToReturn;
    containerToReturn._sourceIndex = indexOfLabel;
    containerToReturn._sourceLabel = inputLabel;

    containerToReturn._targetIndex = indexWithMinimumDistance;
    containerToReturn._targetLabel = labelOfMinimumDistanceNode;

    containerToReturn._pathListOfIndices = path;
    std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
    std::vector<uint32_t> vectorOfLabels;
    for (int i = 0; i < vectorOfIndices.size(); i++) {
        vertex_t currentIndex = vectorOfIndices[i];
        vectorOfLabels.push_back(_graphToTraverse._map_listIndexToNodeLabel[currentIndex]);
    }
    containerToReturn._pathVectorOfLabels = vectorOfLabels;

    //std::cout << "Returning container." << std::endl;
    return containerToReturn;
}

PathDataContainer DijkstraPathfinder::findPathToLowestAndClosestStemPoint(uint32_t inputLabel, PlantSegmentationDataContainer *inputSegmentationData) {

    assert(_graphToTraverse._adjacencyList.size() == _graphToTraverse._map_listIndexToNodeLabel.size() &&
                "The number of indices in the adjacencyList is not equal to the number indices mapped to node labels. Something went wrong with building the Graph.");

    ColorMap colorMap;

    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    vertex_t indexOfLabel = _graphToTraverse._map_nodeLabelToListIndex[inputLabel];

    //std::cout << "Finding path for lowest and closest stem point for node with label " << inputLabel << " with index " << indexOfLabel << std::endl;

    computePaths(indexOfLabel, _graphToTraverse._adjacencyList, min_distance, previous);

    // We have the distances between the labels, now we need to be able to determine if the label is segmented.
    inputSegmentationData->updateSupervoxelSegmentationMap();

    // We want to check all of the nodes that belong to the stem, and get the nodes that the path could get to without going through a stem supervoxel (e.g. stem borders).
    std::vector<uint32_t> stemBorders;
    std::vector<uint32_t> allStemLabels;
    weight_t closestStemDistance = 1000000.0; // Arbitrarily large magic number
    int indexWithMinimumDistance = 0;
    for (int i = 0; i < min_distance.size(); i++) {
        int currentLabel = _graphToTraverse._map_listIndexToNodeLabel[i];

        TupleTriplet colorOfSupervoxel = inputSegmentationData->_map_segmentedSupervoxels[currentLabel];
        // For each stem supervoxel
        if (colorOfSupervoxel == colorMap._stem_color) {
            allStemLabels.push_back(currentLabel);
            // Does the shortest path involve going through another stem supervoxel?
            std::list<vertex_t> path = DijkstraGetShortestPathTo(i, previous);
            std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
            int numberOfStemSupervoxels = 0;
            for (int j = 0; j < vectorOfIndices.size(); j++) {
                int currentIndex = vectorOfIndices[j];
                int pathLabel = _graphToTraverse._map_listIndexToNodeLabel[currentIndex];
                TupleTriplet colorOfPathSupervoxel = inputSegmentationData->_map_segmentedSupervoxels[pathLabel];
                if (colorOfPathSupervoxel == colorMap._stem_color) {
                    numberOfStemSupervoxels += 1;
                }
            }
            // If there were more than two stem supervoxels in the path, (the end point and something in the middle)
            if (numberOfStemSupervoxels >= 2) {
                // We aren't interested in it.
            }
            else {
                // We want to save the label for later.
                stemBorders.push_back(currentLabel);
            }
        }
    }

    // We want the label with the smallest Z coordinate.
    float minimumZ = 100000.0; //Arbitrary magic number
    uint32_t labelOfMinimumZNode = 0;
    for (int i = 0; i < stemBorders.size(); i++) {
        TupleTriplet currentPoint = inputSegmentationData->_supervoxelData._map_labelToCentroidOfSupervoxel[stemBorders[i]];
        float currentZ = std::get<2>(currentPoint);
        if (currentZ < minimumZ) {
            minimumZ = currentZ;
            labelOfMinimumZNode = stemBorders[i];
        }
    }
    uint32_t indexOfMinimumZNode = _graphToTraverse._map_nodeLabelToListIndex[labelOfMinimumZNode];

    //std::cout << "For path to the nearest and lowest stem label, distance from label " << inputLabel << " (index " << indexOfLabel << ") to label " << labelOfMinimumZNode << " (index " << indexOfMinimumZNode << ") is " << min_distance[indexOfMinimumZNode] << std::endl;

    std::list<vertex_t> path = DijkstraGetShortestPathTo(indexOfMinimumZNode, previous);
    //std::cout << "Removing the last stem index, and returning the path container." << std::endl;
    path.pop_back();

    PathDataContainer containerToReturn;
    containerToReturn._sourceIndex = indexOfLabel;
    containerToReturn._sourceLabel = inputLabel;

    containerToReturn._targetIndex = indexOfMinimumZNode;
    containerToReturn._targetLabel = labelOfMinimumZNode;

    containerToReturn._distance = min_distance[indexOfMinimumZNode];

    containerToReturn._pathListOfIndices = path;
    std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
    std::vector<uint32_t> vectorOfLabels;
    for (int i = 0; i < vectorOfIndices.size(); i++) {
        vertex_t currentIndex = vectorOfIndices[i];
        vectorOfLabels.push_back(_graphToTraverse._map_listIndexToNodeLabel[currentIndex]);
    }
    containerToReturn._pathVectorOfLabels = vectorOfLabels;

    return containerToReturn;
}


PathDataContainer DijkstraPathfinder::findPathFromSourceLabelToUnsegmentedConnectedLabelThatIsClosestToUnconnectedLabel(uint32_t inputSourceLabel, uint32_t inputTargetLabel, PlantSegmentationDataContainer *inputSegmentationData) {

    // The assumption here is that our input point is has some unlabeled adjacencies, but that it is not connected to the target.
    // We want to find a new target that is close to the input target, and can be reached from the source.
    ColorMap colorMap;

    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    vertex_t indexOfSourceLabel = _graphToTraverse._map_nodeLabelToListIndex[inputSourceLabel];

    //std::cout << "Finding path From Source Label To Unsegmented Connected Label That Is Closest To Unconnected Label for node with label " << inputSourceLabel << " with index " << indexOfSourceLabel << std::endl;

    computePaths(indexOfSourceLabel, _graphToTraverse._adjacencyList, min_distance, previous);

    // The distance of every unconnected point will be infinity. We want to find the non-infinite point that minimizes the distance between itself and the target.
    float minDistance = 10000000.0; //Arbitrarily large magic number
    int minLabel = 0;
    int indexOfMinLabel = 0;
    PathDataContainer pathData;
    for (int i = 0; i < min_distance.size(); i++) {
        int currentLabel = _graphToTraverse._map_listIndexToNodeLabel[i];
        //if the points are connected to the source
        if (min_distance[i] != std::numeric_limits<double>::infinity()) {
            // we want to determine how far away it is from the stem.
            pathData = this->findPathToClosestStemPoint(currentLabel, inputSegmentationData);
            if (minDistance > pathData._distance) {
                minDistance = pathData._distance;
                minLabel = currentLabel;
                indexOfMinLabel = i;
            }
        }
    }

    //std::cout << "Distance from label " << inputSourceLabel << " (index " << indexOfSourceLabel << ") to label " << minLabel << " (index " << indexOfMinLabel << ") is " << min_distance[indexOfMinLabel] << std::endl;
    std::list<vertex_t> path = DijkstraGetShortestPathTo(indexOfMinLabel, previous);
    //std::cout << "Path of the indices: ";
    //std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    //std::cout << std::endl;

    PathDataContainer containerToReturn;
    containerToReturn._sourceIndex = indexOfSourceLabel;
    containerToReturn._sourceLabel = inputSourceLabel;

    containerToReturn._targetIndex = indexOfMinLabel;
    containerToReturn._targetLabel = minLabel;

    containerToReturn._distance = min_distance[indexOfMinLabel];

    containerToReturn._pathListOfIndices = path;
    std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
    std::vector<uint32_t> vectorOfLabels;
    for (int i = 0; i < vectorOfIndices.size(); i++) {
        vertex_t currentIndex = vectorOfIndices[i];
        vectorOfLabels.push_back(_graphToTraverse._map_listIndexToNodeLabel[currentIndex]);
    }
    containerToReturn._pathVectorOfLabels = vectorOfLabels;

    //std::cout << "Returning container." << std::endl;

    return containerToReturn;
}

PathDataContainer DijkstraPathfinder::findPathBetweenTwoLabels(uint32_t inputSourceLabel, uint32_t inputTargetLabel) {

    //std::cout << "length of the adjacency list: " << _graphToTraverse._adjacencyList.size() << std::endl;
    //std::cout << "length of the listIndexToNodeLabel map: " << _graphToTraverse._map_listIndexToNodeLabel.size() << std::endl;
    assert(_graphToTraverse._adjacencyList.size() == _graphToTraverse._map_listIndexToNodeLabel.size() &&
                "The number of indices in the adjacencyList is not equal to the number indices mapped to node labels. Something went wrong with building the Graph.");

    PathDataContainer containerToReturn;

    if (_graphToTraverse._map_nodeLabelToListIndex.find(inputSourceLabel) != _graphToTraverse._map_nodeLabelToListIndex.end() &&
            _graphToTraverse._map_nodeLabelToListIndex.find(inputTargetLabel) != _graphToTraverse._map_nodeLabelToListIndex.end()) {

        std::vector<weight_t> min_distance;
        std::vector<vertex_t> previous;
        vertex_t indexOfSourceLabel = _graphToTraverse._map_nodeLabelToListIndex[inputSourceLabel];

        //std::cout << "Finding path between two labels for input node with label " << inputSourceLabel << " and index " << indexOfSourceLabel << std::endl;

        computePaths(indexOfSourceLabel, _graphToTraverse._adjacencyList, min_distance, previous);

        uint32_t indexOfTargetLabel = _graphToTraverse._map_nodeLabelToListIndex[inputTargetLabel];

        //std::cout << "Distance from label " << inputSourceLabel << " (index " << indexOfSourceLabel << ") to label " << inputTargetLabel << " (index " << indexOfTargetLabel << ") is " << min_distance[indexOfTargetLabel] << std::endl;
        std::list<vertex_t> path = DijkstraGetShortestPathTo(indexOfTargetLabel, previous);
        //std::cout << "Path of the indices: ";
        //std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
        //std::cout << std::endl;


        containerToReturn._sourceIndex = indexOfSourceLabel;
        containerToReturn._sourceLabel = inputSourceLabel;

        containerToReturn._targetIndex = indexOfTargetLabel;
        containerToReturn._targetLabel = inputTargetLabel;

        containerToReturn._distance = min_distance[indexOfTargetLabel];

        containerToReturn._pathListOfIndices = path;
        std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
        std::vector<uint32_t> vectorOfLabels;
        for (int i = 0; i < vectorOfIndices.size(); i++) {
            vertex_t currentIndex = vectorOfIndices[i];
            vectorOfLabels.push_back(_graphToTraverse._map_listIndexToNodeLabel[currentIndex]);
        }
        containerToReturn._pathVectorOfLabels = vectorOfLabels;
    }
    else {
        std::cout << "WARNING! One of the two input lables does not exist in the graph. Returning a container with infinite distance." << std::endl;
        containerToReturn._sourceIndex = 0;
        containerToReturn._sourceLabel = inputSourceLabel;

        containerToReturn._targetIndex = 0;
        containerToReturn._targetLabel = inputTargetLabel;

        containerToReturn._distance = std::numeric_limits<double>::infinity();
    }


    return containerToReturn;
}

float DijkstraPathfinder::getZCoordinateOfFirstStemAdjacency(uint32_t inputSourceLabel, uint32_t inputStemTarget, PlantSegmentationDataContainer *inputSegmentationData) {

    assert(inputSegmentationData->_map_segmentedSupervoxels.size() == _graphToTraverse._map_nodeLabelToListIndex.size() &&
                "The number of supervoxels is not equal to the number of nodes in the graph. Did the graph come from these supervoxels, or does the supervoxel segmentation need to be updated?");
    assert(_graphToTraverse._adjacencyList.size() == _graphToTraverse._map_listIndexToNodeLabel.size() &&
                "The number of indices in the adjacencyList is not equal to the number indices mapped to node labels. Something went wrong with building the Graph.");

    ColorMap colorMap;

    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    vertex_t indexOfSourceLabel = _graphToTraverse._map_nodeLabelToListIndex[inputSourceLabel];

    //std::cout << "getting Z coordinate of first stem adjacency for node with label " << inputSourceLabel << " with index " << indexOfSourceLabel << std::endl;

    computePaths(indexOfSourceLabel, _graphToTraverse._adjacencyList, min_distance, previous);

    uint32_t indexOfTargetLabel = _graphToTraverse._map_nodeLabelToListIndex[inputStemTarget];

    //std::cout << "Distance from label " << inputSourceLabel << " (index " << indexOfSourceLabel << ") to label " << inputStemTarget << " (index " << indexOfTargetLabel << ") is " << min_distance[indexOfTargetLabel] << std::endl;
    std::list<vertex_t> path = DijkstraGetShortestPathTo(indexOfTargetLabel, previous);
    //std::cout << "Path of the indices: ";
    std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    std::cout << std::endl;

    PathDataContainer containerToReturn;
    containerToReturn._sourceIndex = indexOfSourceLabel;
    containerToReturn._sourceLabel = inputSourceLabel;

    containerToReturn._targetIndex = indexOfTargetLabel;
    containerToReturn._targetLabel = inputStemTarget;

    containerToReturn._distance = min_distance[indexOfTargetLabel];

    containerToReturn._pathListOfIndices = path;
    std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
    std::vector<uint32_t> vectorOfLabels;
    for (int i = 0; i < vectorOfIndices.size(); i++) {
        vertex_t currentIndex = vectorOfIndices[i];
        vectorOfLabels.push_back(_graphToTraverse._map_listIndexToNodeLabel[currentIndex]);
    }
    containerToReturn._pathVectorOfLabels = vectorOfLabels;

    //With the vector of labels, we need to find out the coordinate of the first label to be adjacent to a stem.
    // For each label in order of traversal
    bool foundLabelAdjacentToStem = false;
    float zCoordinate = 0.0;
    for (int i = 0; i < vectorOfLabels.size(); i++) {
        uint32_t currentLabel = vectorOfLabels[i];
        // For each of its adjacencies, we need to find the one with the mimum distance.
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
        for ( adjacent_itr = inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentLabel).first;
                adjacent_itr != inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentLabel).second; adjacent_itr++) {
            uint32_t adjacentLabel = adjacent_itr->second;
            TupleTriplet adjacentColor = inputSegmentationData->_map_segmentedSupervoxels[adjacentLabel];
            // If the adjacency is segmented as a stem, we're done.
            if (adjacentColor == colorMap._stem_color) {
                foundLabelAdjacentToStem = true;
                TupleTriplet coordinatePoint = inputSegmentationData->_supervoxelData._map_labelToCentroidOfSupervoxel[currentLabel];
                zCoordinate = std::get<2>(coordinatePoint);
                //std::cout << "Label " << currentLabel << " with Z coordinate of " << std::get<2>(coordinatePoint) << " is adjacent to " << adjacentLabel << " which is segmented as stem." << std::endl;
                break;
            }
        }
        if (foundLabelAdjacentToStem == true) {
            break;
        }
    }
    return zCoordinate;
}


/** The purpose of this is to find the leaf base.
*/
uint32_t DijkstraPathfinder::returnIndexOfInputSubsetMeshClosestToStemBottom(pcl::PolygonMesh *inputLeafMesh, pcl::PolygonMesh *inputSegmentedMesh) {
    std::ostringstream logStream;
    // construct a segementation data container so that we can use its color map downstream.
    PlantSegmentationDataContainer segmentationData(inputSegmentedMesh);

    // construct a map of the points in the input leaf mesh that we can search later.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOfLeafMesh (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsOfLeafMeshColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::fromPCLPointCloud2(inputLeafMesh->cloud, *cloudPointsOfLeafMesh);
    pcl::fromPCLPointCloud2(inputLeafMesh->cloud, *cloudPointsOfLeafMeshColored);

    std::map<TupleTriplet, uint32_t> subsetMeshPointsToIndices;
    for (uint32_t i = 0; i < cloudPointsOfLeafMeshColored->points.size(); i ++) {
        pcl::PointXYZRGBA currentPoint = cloudPointsOfLeafMeshColored->points[i];
        TupleTriplet currentPointTuple(currentPoint.x, currentPoint.y, currentPoint.z);
        subsetMeshPointsToIndices.insert(std::pair<TupleTriplet, uint32_t>(currentPointTuple, i));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOfSegmentedMesh (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsOfSegmentedMeshColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::fromPCLPointCloud2(inputSegmentedMesh->cloud, *cloudPointsOfSegmentedMesh);
    pcl::fromPCLPointCloud2(inputSegmentedMesh->cloud, *cloudPointsOfSegmentedMeshColored);

    assert(_graphToTraverse._adjacencyList.size() == cloudPointsOfSegmentedMesh->size() &&
                "The number of indices in the adjacencyList is not equal to the number of vertices in the provided mesh. \
                findPathFromStemBottomToClosestLeafPointOfGivenColor() assumes that the DijkstraPathfinder was constructed with \
                the same mesh provided as input. Check to make sure the same mesh is used to build the Graph as is being used as a function argument.");

    ColorMap colorMap;


    // First, we find the index on the minimum stem point.
    float minStemZCoord = 10000000.0; //Arbitrarily large magic number;
    uint32_t indexOfMinStemLabel = 0;
    for (uint32_t i = 0; i < cloudPointsOfSegmentedMeshColored->points.size(); i ++) {
        pcl::PointXYZRGBA currentPoint = cloudPointsOfSegmentedMeshColored->points[i];
        TupleTriplet currentColor(currentPoint.r, currentPoint.g, currentPoint.b);
        float zCoord = currentPoint.z;
        if (currentColor == colorMap._stem_color) {
            if (zCoord < minStemZCoord) {
                minStemZCoord = zCoord;
                indexOfMinStemLabel = i;
            }
        }
    }

    // Now that we have the point, we compute all of the paths.
    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;

    computePaths(indexOfMinStemLabel, _graphToTraverse._adjacencyList, min_distance, previous);

    // Then we need to find the point closest to the stem that is contained in the subset mesh.
    float minDistance = 10000000.0; //Arbitrarily large magic number
    int minLabel = 0;
    uint32_t indexOfMinLabel = 0;
    uint32_t indexOfMinLabelInSubsetMesh = 0;

    for (uint32_t i = 0; i < min_distance.size(); i++) {
        uint32_t currentIndex = i;
        pcl::PointXYZRGBA currentPoint = cloudPointsOfSegmentedMeshColored->points[i];
        TupleTriplet currentCoord(currentPoint.x, currentPoint.y, currentPoint.z);
        // If the current point's color is in the map of the subset mesh points, we consider it.
        if (subsetMeshPointsToIndices.find(currentCoord) != subsetMeshPointsToIndices.end()) {
            if (min_distance[i] < minDistance) {
                minDistance = min_distance[i];
                indexOfMinLabel = i;
                indexOfMinLabelInSubsetMesh = subsetMeshPointsToIndices[currentCoord];
            }
        }
    }

    logStream << "Minimum coordinate for leaf base found at " << cloudPointsOfSegmentedMeshColored->points[indexOfMinLabel];
    LOG.DEBUG(logStream.str()); logStream.str("");
    logStream << "In with index in subset mesh of " << indexOfMinLabelInSubsetMesh;
    LOG.DEBUG(logStream.str()); logStream.str("");
    logStream << " and coordinate in subset mesh of " << cloudPointsOfLeafMeshColored->points[indexOfMinLabelInSubsetMesh];
    LOG.DEBUG(logStream.str()); logStream.str("");
    return indexOfMinLabelInSubsetMesh;

}

PathDataContainer DijkstraPathfinder::findPathToMostDistantNodeFromInputSource(uint32_t inputLabel) {
    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;

    vertex_t indexOfInputLabel = _graphToTraverse._map_nodeLabelToListIndex[inputLabel];

    computePaths(indexOfInputLabel, _graphToTraverse._adjacencyList, min_distance, previous);

    weight_t maximumDistance = 0.0; //Hold the maximum of the minimum distances
    int indexWithMaxDistance = 0;
    for (uint32_t i = 0; i < min_distance.size(); i++) {
        assert (min_distance[i] != std::numeric_limits<double>::infinity() && "There is an unconnected node in this graph, which shouldn't be the case. Aborting.");
        if (min_distance[i] > maximumDistance) {
            maximumDistance = min_distance[i];
            indexWithMaxDistance = i;
        }
    }
    uint32_t labelMaxDistanceNode = _graphToTraverse._map_listIndexToNodeLabel[indexWithMaxDistance];

    std::list<vertex_t> path = DijkstraGetShortestPathTo(indexWithMaxDistance, previous);

    PathDataContainer containerToReturn;
    containerToReturn._sourceIndex = inputLabel;
    containerToReturn._sourceLabel = indexOfInputLabel;

    containerToReturn._targetIndex = indexWithMaxDistance;
    containerToReturn._targetLabel = labelMaxDistanceNode;

    containerToReturn._distance = min_distance[indexWithMaxDistance];

    containerToReturn._pathListOfIndices = path;
    std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
    std::vector<uint32_t> vectorOfLabels;
    for (int i = 0; i < vectorOfIndices.size(); i++) {
        vertex_t currentIndex = vectorOfIndices[i];
        vectorOfLabels.push_back(_graphToTraverse._map_listIndexToNodeLabel[currentIndex]);
    }
    containerToReturn._pathVectorOfLabels = vectorOfLabels;

    return containerToReturn;
}

PathDataContainer DijkstraPathfinder::findPathForIsolatedTip(uint32_t inputLabel, PlantSegmentationDataContainer *inputSegmentationData, InputParameters inputParams) {
    std::ostringstream logStream;
    // If we've gotten to this point, there is no path between the label and the stem, and the tip is isolated.
    LOG.DEBUG("\tAttempting to find a path for the isolated leaf tip.");
    ColorMap colorMap;
    PathDataContainer containerToReturn;
    // If a leaf tip is isolated and can't be pathed to, it is either an artifactual leaf tip or merged.

    // If it's a real tip just partially occluded, there should be a good chunk of unsegmented space it can travel though?

    // Maybe something like:
    // While haven't traveled over 4 supervoxels,
    // travel towards lowest and closest stem point until a segmented node is hit.
    // Then travel to either the closest unsegmented adjacent node that reduces distance to the target
    // or travel over a segmented node that brings you closer to the target

    //PathDataContainer pathDataTipToStem = this->findPathToLowestAndClosestStemPoint(inputLabel, inputSegmentationData);
    PathDataContainer pathDataTipToStem = this->findPathToClosestStemPoint(inputLabel, inputSegmentationData);
    int labelOfTargetStemPoint = pathDataTipToStem._targetLabel;
    PathDataContainer currentPath = pathDataTipToStem;
    int numberSegmentedNodesTraversedOver = 0;
    bool foundValidPathToStem = false;
    std::vector<uint32_t> pathTraversed;
    int MAGIC_NUMBER_OF_SUPERVOXELS_TO_TRAVERSE_OVER = 2; // Magic number of supervoxels that can be "jumped" over. This should probably be an input parameter.

    while(numberSegmentedNodesTraversedOver <= MAGIC_NUMBER_OF_SUPERVOXELS_TO_TRAVERSE_OVER) {
        logStream << "The currentPath for the isolated tip is: ";
        for (uint32_t i = 0; i < currentPath._pathVectorOfLabels.size(); i++) {
            logStream << currentPath._pathVectorOfLabels[i] << " ";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        logStream << "The final path traversed state is: ";
        for (uint32_t i = 0; i < pathTraversed.size(); i++) {
            logStream << pathTraversed[i] << " ";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        logStream << "\t and the number of segmented nodes traversed over is: " << numberSegmentedNodesTraversedOver << std::endl;
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        for (int i = 0; i < currentPath._pathVectorOfLabels.size(); i++) {
            int currentPathLabel = currentPath._pathVectorOfLabels[i];
            TupleTriplet currentColor = inputSegmentationData->_map_segmentedSupervoxels[currentPathLabel];
            PathDataContainer pathToBeat = this->findPathBetweenTwoLabels(currentPathLabel, labelOfTargetStemPoint);
            if (currentColor != colorMap._unsegmented_color && foundValidPathToStem == false) {
                logStream << "A segmented node exists in the path: Node " << currentPathLabel << ". Deciding where to go next.";
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");

                //I think we need to check all of the adjacencies to decide what to do.
                bool unsegmentedAdjacencyExists = false;
                std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
                for ( adjacent_itr = inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).first;
                    adjacent_itr != inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).second; adjacent_itr++) {
                    uint32_t adjacentLabel = adjacent_itr->second;
                    if (inputSegmentationData->_map_segmentedSupervoxels[adjacentLabel] == colorMap._unsegmented_color) {
                        unsegmentedAdjacencyExists = true;
                    }
                }

                /// We've determined if there is an unsegmented adjacency. Now we handle each individual case.

                // Case 1, If there is an unsegmented adjacency node
                // check all unsegmented nodes and get their distances to the target.
                // find the one that brings us closest to the target
                // set the current path to be the path from this node and the target
                // break out of the for loop; no need to increment the segmented nodes.
                if (unsegmentedAdjacencyExists == true) {
                    LOG.DEBUG("Unsegmented adjacency(ies) exists. Testing for its (their) utility.");
                    float minimumDistance = 10000000.0; //Arbitrarily large magic number
                    int minimumLabel = 0;
                    PathDataContainer minimumPath;
                    for ( adjacent_itr = inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).first;
                        adjacent_itr != inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).second; adjacent_itr++) {
                        uint32_t adjacentLabel = adjacent_itr->second;
                        if (inputSegmentationData->_map_segmentedSupervoxels[adjacentLabel] == colorMap._unsegmented_color) {
                            PathDataContainer pathAdjacentToTarget = this->findPathBetweenTwoLabels(adjacentLabel, labelOfTargetStemPoint);
                            if (pathAdjacentToTarget._distance < minimumDistance) {
                                minimumDistance = pathAdjacentToTarget._distance;
                                minimumLabel = adjacentLabel;
                                minimumPath = pathAdjacentToTarget;
                            }
                        }
                    }
                    // If this brings us closer to the target, we should use it.
                    if (minimumPath._distance < pathToBeat._distance) {
                        uint32_t previousLabel = 0;

                        // I think this need to reference the last path traversed element if possible to find the clear path.
                        if (pathTraversed.size() > 0) {
                            previousLabel = pathTraversed.back();
                        }
                        else {
                            previousLabel = currentPathLabel;
                        }

                        logStream << "Node " << minimumLabel << " can bring us closer to the target. Attempting to find a clear path between " <<
                                    "the path traversed node prior to " << currentPathLabel << ", which is node " << previousLabel << ".";
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");

                        // Can we get there without moving over segmented space? Let's try to back up one and get a path there.
                        // Bugfix 12/16/2015: For some reason this was returning supervoxel adjacency for non leaf supervoxels, like so:
                        //std::multimap<uint32_t, uint32_t> nonLeafAdjacencies = inputSegmentationData->returnSupervoxelAdjacencyForNonLeafSupervoxels();
                        // I think we want unlabeled supervoxels, not just non-leaf supervoxels. Otherwise we can travel through the stem, leading to strange behavior.
                        std::multimap<uint32_t, uint32_t> unlabeledAdjacencies = inputSegmentationData->returnSupervoxelAdjacencyForUnlabeledSupervoxels();
                        Graph localGraph(unlabeledAdjacencies, inputSegmentationData->_supervoxelData._map_labelToCentroidOfSupervoxel);
                        DijkstraPathfinder localPathfinder(localGraph);

                        PathDataContainer localPathData = localPathfinder.findPathBetweenTwoLabels(previousLabel, minimumLabel);
                        if (localPathData._distance != std::numeric_limits<double>::infinity()) {
                            logStream << "Found a clear path to the useful unsegmented label via ";
                            for (uint32_t pathIndex = 0; pathIndex < localPathData._pathVectorOfLabels.size(); pathIndex++) {
                                logStream << localPathData._pathVectorOfLabels[pathIndex] << " ";
                            }
                            logStream << " with distance " << localPathData._distance;
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");
                            currentPath = minimumPath;
                            for (int pathIndex = 0; pathIndex < localPathData._pathVectorOfLabels.size(); pathIndex++) {
                                    pathTraversed.push_back(localPathData._pathVectorOfLabels[pathIndex]);
                            }
                        }
                        else {
                            LOG.DEBUG("Unable to find a clear path to the useful unsegmented label. Moving over a segmented label.");
                            currentPath = minimumPath;
                            pathTraversed.push_back(currentPathLabel);
                            numberSegmentedNodesTraversedOver += 1;
                            pathTraversed.push_back(minimumLabel);
                        }
                        // We want to break here if the point is useful.
                        break;
                    }
                    else {
                        LOG.DEBUG("Unsegmented adjacency is present, but it's further away from the target than the current spot.");
                    }
                    // If not, we need to find another route.
                }

                // Case 2, there are no unsegmented nodes around OR the path via an unsegmented node doesn't beat the path to beat.
                // check the adjacencies of all adjacent nodes.
                // if there is an unsegmented node, store all of the possible nodes that will bring us closer to the target.
                // set the current path to be the path from this node and the target
                // break out of the for loop, and increment the segmented nodes by 1.

                // getting to this point means the code didn't exit via case 1.
                // For each primary adjacency
                { //Scoping 2nd case
                LOG.DEBUG("Unable to find a suitable unsegmented adjacency. Testing the secondary adjacencies.");
                float minimumDistance = 10000000.0; //Arbitrarily large magic number
                uint32_t minimumPrimaryAdjacencyLabel = 0;
                uint32_t minimumSecondaryAdjacencyLabel = 0;
                PathDataContainer minimumPath;
                minimumPath._distance = minimumDistance;
                for ( adjacent_itr = inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).first;
                        adjacent_itr != inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).second; adjacent_itr++) {
                    uint32_t adjacentLabel = adjacent_itr->second;
                    std::multimap<uint32_t,uint32_t>::iterator secondaryAdjacentItr;
                    // For each secondary adjacency
                    for ( secondaryAdjacentItr = inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(adjacentLabel).first;
                            secondaryAdjacentItr != inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(adjacentLabel).second; secondaryAdjacentItr++) {
                        uint32_t secondaryAdjacentLabel = secondaryAdjacentItr->second;
                        TupleTriplet secondaryAdjacentColor = inputSegmentationData->_map_segmentedSupervoxels[secondaryAdjacentLabel];
                        // Pick the secondary adjacency that gets us closer to the target.
                        if (inputSegmentationData->_map_segmentedSupervoxels[secondaryAdjacentLabel] == colorMap._unsegmented_color) {
                            PathDataContainer pathForSecondaryAdjacencyToTarget = this->findPathBetweenTwoLabels(secondaryAdjacentLabel, labelOfTargetStemPoint);
                            if (pathForSecondaryAdjacencyToTarget._distance < minimumDistance) {
                                minimumDistance = pathForSecondaryAdjacencyToTarget._distance;
                                minimumPrimaryAdjacencyLabel = adjacentLabel;
                                minimumSecondaryAdjacencyLabel = secondaryAdjacentLabel;
                                minimumPath = pathForSecondaryAdjacencyToTarget;
                            }
                        }
                    }
                }

                // If this brings us closer to the target, we should use it.
                if (minimumPath._distance < pathToBeat._distance) {
                    currentPath = minimumPath;
                    pathTraversed.push_back(currentPathLabel);
                    numberSegmentedNodesTraversedOver += 1;
                    // The primary adjacency had to have been segmented to get here.
                    pathTraversed.push_back(minimumPrimaryAdjacencyLabel);
                    numberSegmentedNodesTraversedOver += 1;
                    pathTraversed.push_back(minimumSecondaryAdjacencyLabel);
                    LOG.DEBUG("Found a useful secondary adjacent unsegmented label. Moving to it.");
                    break;
                }
                else {
                    LOG.DEBUG("Unable to find a useful secondary adjacent unsegmented label.");
                }
                } //end scoping of Second Case

                // Case 3, there are no unsegmented nodes around, nor any unsegmented secondary adjacencies.
                // If we get here, we just have to continue the loop by traveling over the segmented patch.
                pathTraversed.push_back(currentPathLabel);
                numberSegmentedNodesTraversedOver += 1;
                // I think we run into a problem here if all of the rest of the nodes are segmented; it never breaks out of this to test the while statement.
                // The logic of this code block should be re-examined, since we have so many break statements.
                if (numberSegmentedNodesTraversedOver > MAGIC_NUMBER_OF_SUPERVOXELS_TO_TRAVERSE_OVER) {
                    logStream << "The path has traveled over " << numberSegmentedNodesTraversedOver << ", which is greater than the number permitted, " <<
                                    MAGIC_NUMBER_OF_SUPERVOXELS_TO_TRAVERSE_OVER << ". Breaking.";
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    break;
                }
            }
            else {
                pathTraversed.push_back(currentPathLabel);
            }
        }
    }

    PathDataContainer pathData;
    pathData._pathVectorOfLabels = pathTraversed;

    // If the final point in the path is a stem, pop it.
    if (inputSegmentationData->_map_segmentedSupervoxels[pathData._pathVectorOfLabels.back()] == colorMap._stem_color) {
        pathData._pathVectorOfLabels.pop_back();
    }

    //std::cout << "The final path for the isolated tip is: ";
    //for (int i = 0; i < pathData._pathVectorOfLabels.size(); i++) {
    //    std::cout << pathData._pathVectorOfLabels[i] << " ";
    //}
    //std::cout << std::endl;

    containerToReturn = pathData;
    return containerToReturn;
}

/** I'm not sure if this is guaranteed to find the geodesic diameter, but I suspect it's a decent approximation.
  * We pick an arbitrary node on the graph, and then travel to the most distant node from it.
  * We set that node as the source, then find the most distant node from it.
  */
PathDataContainer DijkstraPathfinder::findGeodesicDiameter() {
    std::ostringstream logStream;
    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;

    // Pick the first node arbitrarily. Use it to find an initial distant point.
    vertex_t indexOfInitialLabel = _graphToTraverse._map_nodeLabelToListIndex[0];

    computePaths(indexOfInitialLabel, _graphToTraverse._adjacencyList, min_distance, previous);

    weight_t maximumDistance = 0.0; //Hold the maximum of the minimum distances
    int indexWithMaxDistance = 0;
    for (uint32_t i = 0; i < min_distance.size(); i++) {
        assert (min_distance[i] != std::numeric_limits<double>::infinity() && "There is an unconnected node in this graph, which shouldn't be the case. Aborting.");
        if (min_distance[i] > maximumDistance) {
            maximumDistance = min_distance[i];
            indexWithMaxDistance = i;
        }
    }
    uint32_t labelNextSource = _graphToTraverse._map_listIndexToNodeLabel[indexWithMaxDistance];
    // Now we have an initial extreme point. Find the most distant node from it to approximate the geodesic diameter.
    uint32_t indexNextSource = indexWithMaxDistance;

    min_distance.clear();
    previous.clear();

    computePaths(indexNextSource, _graphToTraverse._adjacencyList, min_distance, previous);

    maximumDistance = 0.0; //Hold the maximum of the minimum distances
    indexWithMaxDistance = 0;
    for (uint32_t i = 0; i < min_distance.size(); i++) {
        if (min_distance[i] > maximumDistance) {
            maximumDistance = min_distance[i];
            indexWithMaxDistance = i;
        }
    }
    uint32_t labelMaxDistanceNode = _graphToTraverse._map_listIndexToNodeLabel[indexWithMaxDistance];

    std::list<vertex_t> path = DijkstraGetShortestPathTo(indexWithMaxDistance, previous);

    PathDataContainer containerToReturn;
    containerToReturn._sourceIndex = indexNextSource;
    containerToReturn._sourceLabel = labelNextSource;

    containerToReturn._targetIndex = indexWithMaxDistance;
    containerToReturn._targetLabel = labelMaxDistanceNode;

    containerToReturn._distance = min_distance[indexWithMaxDistance];

    containerToReturn._pathListOfIndices = path;
    std::vector<vertex_t> vectorOfIndices{std::begin(path), std::end(path)};
    std::vector<uint32_t> vectorOfLabels;
    for (int i = 0; i < vectorOfIndices.size(); i++) {
        vertex_t currentIndex = vectorOfIndices[i];
        vectorOfLabels.push_back(_graphToTraverse._map_listIndexToNodeLabel[currentIndex]);
    }
    containerToReturn._pathVectorOfLabels = vectorOfLabels;

    return containerToReturn;

}

void DijkstraPathfinder::computePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list,
                          std::vector<weight_t> &min_distance,
                          std::vector<vertex_t> &previous)
{
    int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    std::set<std::pair<weight_t, vertex_t> > vertex_queue;
    vertex_queue.insert(std::make_pair(min_distance[source], source));

    while (!vertex_queue.empty())
    {
        weight_t dist = vertex_queue.begin()->first;
        vertex_t u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        // Visit each edge exiting u
	const std::vector<neighbor> &neighbors = adjacency_list[u];
        for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
             neighbor_iter != neighbors.end();
             neighbor_iter++)
        {
            vertex_t v = neighbor_iter->target;
            weight_t weight = neighbor_iter->weight;
            weight_t distance_through_u = dist + weight;
	    if (distance_through_u < min_distance[v]) {
	        vertex_queue.erase(std::make_pair(min_distance[v], v));

	        min_distance[v] = distance_through_u;
	        previous[v] = u;
	        vertex_queue.insert(std::make_pair(min_distance[v], v));

	    }

        }
    }
}

std::vector<uint32_t> DijkstraPathfinder::returnLabelsOfLargestConnectedSet() {

    assert(_graphToTraverse._adjacencyList.size() == _graphToTraverse._map_listIndexToNodeLabel.size() &&
                "The number of indices in the adjacencyList is not equal to the number indices mapped to node labels. Something went wrong with building the Graph.");

    std::map<uint32_t, uint32_t> map_visitedIndices;
    std::map<uint32_t, uint32_t> map_largestConnectedSetIndices;

    uint32_t indexCurrentNode = 0;

    // While we haven't accounted for all nodes in the mesh.
    while(map_visitedIndices.size() != _graphToTraverse._adjacencyList.size()) {

        std::vector<weight_t> min_distance;
        std::vector<vertex_t> previous;

        std::map<uint32_t, uint32_t> map_currentConnectedSetIndices;

        computePaths(indexCurrentNode, _graphToTraverse._adjacencyList, min_distance, previous);
        bool unconnectedNodePresent = false;
        uint32_t indexOfUnconnectedNode = 0;
        for (uint32_t i = 0; i < min_distance.size(); i++) {
            float currentDistance = min_distance[i];
            // if we've already visited the node, no need to address it again.
            if (map_visitedIndices.find(i) != map_visitedIndices.end()) {
                continue;
            }

            // if distance is infinity, the node is not connected.
            if (currentDistance == std::numeric_limits<double>::infinity()) {
                indexOfUnconnectedNode = i;
                unconnectedNodePresent = true;
            }
            // else the node can be reached from the current node. I guess this tries to add the current node index repeatedly, which is unnecessary.
            else{
                map_currentConnectedSetIndices.insert(std::pair<uint32_t, uint32_t>(indexCurrentNode, 1));
                map_currentConnectedSetIndices.insert(std::pair<uint32_t, uint32_t>(i, 1));
                map_visitedIndices.insert(std::pair<uint32_t, uint32_t>(indexCurrentNode, 1));
                map_visitedIndices.insert(std::pair<uint32_t, uint32_t>(i, 1));
            }
        }
        indexCurrentNode = indexOfUnconnectedNode;
        if (map_currentConnectedSetIndices.size() > map_largestConnectedSetIndices.size()) {
            map_largestConnectedSetIndices = map_currentConnectedSetIndices;
        }
        //std::cout << "Finished an iteration of visiting indices. indexCurrentNode is : " << indexCurrentNode <<
        //       ", size of the visitedIndices map is " << map_visitedIndices.size() << ", size of the" <<
        //        " adjacencyList for the graph to traverse is " << _graphToTraverse._adjacencyList.size() << ", and the " <<
        //        "size of min_distance is " << min_distance.size() << std::endl;
    }


    std::vector<uint32_t> nodesToReturn;
    std::map<uint32_t, uint32_t>::iterator itr;
    for (itr = map_largestConnectedSetIndices.begin(); itr != map_largestConnectedSetIndices.end(); itr++) {
        nodesToReturn.push_back(_graphToTraverse._map_listIndexToNodeLabel[itr->first]);
    }
    return nodesToReturn;
}

Graph DijkstraPathfinder::getGraphToTraverse() {
    return _graphToTraverse;
}

//end DijkstraPathfinder class

pcl::PolygonMesh returnLargestConnectedMesh(pcl::PolygonMesh *inputMesh) {

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromPCLPointCloud2(inputMesh->cloud, *cloudPoints);

    std::map<uint32_t, TupleTriplet> meshPointCoordinates;
    TupleTriplet meshColor;
    for (uint32_t i = 0; i < cloudPoints->points.size(); i++) {
        pcl::PointXYZRGBA currentPointRGBA = cloudPoints->points[i];
        pcl::PointXYZ currentPoint(currentPointRGBA.x, currentPointRGBA.y, currentPointRGBA.z);
        TupleTriplet currentColor(currentPointRGBA.r, currentPointRGBA.g, currentPointRGBA.b);
        meshColor = currentColor;
        TupleTriplet currentPointTuple = convertPclPointXYZtoTupleTriplet(currentPoint);
        meshPointCoordinates.insert(std::pair<uint32_t, TupleTriplet>(i, currentPointTuple));
    }

    Graph meshGraph(inputMesh);
    DijkstraPathfinder connectednessPathfinder(meshGraph);
    std::vector<uint32_t> connectedLabels = connectednessPathfinder.returnLabelsOfLargestConnectedSet();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOfPointsToKeep (new pcl::PointCloud<pcl::PointXYZ>);
    for (uint32_t i = 0; i < connectedLabels.size(); i++) {
        TupleTriplet coordinateToKeep = meshPointCoordinates[connectedLabels[i]];
        pcl::PointXYZ pointCoordinate = convertTupleTriplettoPclPointXYZ(coordinateToKeep);
        cloudOfPointsToKeep->points.push_back(pointCoordinate);
    }

    pcl::PolygonMesh meshToKeep = extractMeshFromPolygonMeshGivenPointCloud(*inputMesh, cloudOfPointsToKeep);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentCloudPointsColored (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromPCLPointCloud2(meshToKeep.cloud, *currentCloudPointsColored);

    for (uint32_t i = 0; i < currentCloudPointsColored->points.size(); i++) {
        currentCloudPointsColored->points[i].r = std::get<0>(meshColor);
        currentCloudPointsColored->points[i].g = std::get<1>(meshColor);
        currentCloudPointsColored->points[i].b = std::get<2>(meshColor);
        currentCloudPointsColored->points[i].a = 255.0;
    }

    pcl::PCLPointCloud2 newMeshCloud;
    pcl::toPCLPointCloud2(*currentCloudPointsColored, newMeshCloud);
    meshToKeep.cloud = newMeshCloud;

    return meshToKeep;

}

/** Despite the DijkstraPathfinder class, this function is sticking around until I can refactor
  * the supervoxel adjacency trimming code that depends on it.
  *
 */
adjacency_list_t convertPolygonMeshToAdjacencyList(pcl::PolygonMesh *inputMesh) {

    std::cout << "Converting a pcl::PolygonMesh to an adjacency list" << std::endl;

    std::multimap<uint32_t, uint32_t> nodeLabelAdjacency;
    std::map<uint32_t, TupleTriplet> nodeLabelToPointCoordinate;
    std::map<uint32_t, uint32_t> nodeLabelToListIndex;
    std::map<uint32_t, uint32_t> listIndexToNodeLabel;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(inputMesh->cloud, *cloudPoints);

    adjacency_list_t adjacencyList(cloudPoints->points.size());
    for (uint i = 0; i < inputMesh->polygons.size(); i++) {
        ///inputMesh->polygons[i] will return a 3-vector of three point indices.
        ///The point index will work as the node name, but we need the actual point
        ///To determine euclidean distance. It should correspond to the index of the point cloud.
        pcl::Vertices triangle = inputMesh->polygons[i];
        assert(triangle.vertices.size() == 3 && "There are not three vertices in the vertex.");
        uint32_t firstVertexIndex = triangle.vertices[0];
        uint32_t secondVertexIndex = triangle.vertices[1];
        uint32_t thirdVertexIndex = triangle.vertices[2];

        pcl::PointXYZ firstPointCoords = cloudPoints->points[firstVertexIndex];
        pcl::PointXYZ secondPointCoords = cloudPoints->points[secondVertexIndex];
        pcl::PointXYZ thirdPointCoords = cloudPoints->points[thirdVertexIndex];

        float distOneToTwo = pcl::euclideanDistance(firstPointCoords, secondPointCoords);
        float distOneToThree = pcl::euclideanDistance(firstPointCoords, thirdPointCoords);
        float distTwoToThree = pcl::euclideanDistance(secondPointCoords, thirdPointCoords);

        adjacencyList[firstVertexIndex].push_back(neighbor(secondVertexIndex, distOneToTwo));
        adjacencyList[firstVertexIndex].push_back(neighbor(thirdVertexIndex, distOneToThree));

        adjacencyList[secondVertexIndex].push_back(neighbor(firstVertexIndex, distOneToTwo));
        adjacencyList[secondVertexIndex].push_back(neighbor(thirdVertexIndex, distTwoToThree));

        adjacencyList[thirdVertexIndex].push_back(neighbor(firstVertexIndex, distOneToThree));
        adjacencyList[thirdVertexIndex].push_back(neighbor(secondVertexIndex, distTwoToThree));

        nodeLabelToPointCoordinate.insert(std::pair<uint32_t, TupleTriplet>(firstVertexIndex, convertPclPointXYZtoTupleTriplet(firstPointCoords)));
        nodeLabelToPointCoordinate.insert(std::pair<uint32_t, TupleTriplet>(secondVertexIndex, convertPclPointXYZtoTupleTriplet(secondPointCoords)));
        nodeLabelToPointCoordinate.insert(std::pair<uint32_t, TupleTriplet>(thirdVertexIndex, convertPclPointXYZtoTupleTriplet(thirdPointCoords)));

        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(firstVertexIndex, secondVertexIndex));
        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(firstVertexIndex, thirdVertexIndex));

        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(secondVertexIndex, firstVertexIndex));
        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(secondVertexIndex, thirdVertexIndex));

        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(thirdVertexIndex, firstVertexIndex));
        nodeLabelAdjacency.insert(std::pair<uint32_t, uint32_t>(thirdVertexIndex, secondVertexIndex));

        nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t>(firstVertexIndex, firstVertexIndex));
        listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t>(firstVertexIndex, firstVertexIndex));

        nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t>(secondVertexIndex, secondVertexIndex));
        listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t>(secondVertexIndex, secondVertexIndex));

        nodeLabelToListIndex.insert(std::pair<uint32_t, uint32_t>(thirdVertexIndex, thirdVertexIndex));
        listIndexToNodeLabel.insert(std::pair<uint32_t, uint32_t>(thirdVertexIndex, thirdVertexIndex));
    }

    return adjacencyList;

}

// The following is copied from http://rosettacode.org/wiki/Dijkstra's_algorithm#C.2B.2B :
//(Modified from LiteratePrograms, which is MIT/X11 licensed.)
//Solution follows Dijkstra's algorithm as described elsewhere. Data like min-distance, previous node, neighbors, are kept in separate
//  data structures instead of part of the vertex. We number the vertexes starting from 0, and represent the graph using an adjacency
//  list (vector whose i'th element is the vector of neighbors that vertex i has edges to) for simplicity.
//For the priority queue of vertexes, we use a self-balancing binary search tree (std::set), which should bound time complexity
//  by O(E log V). Although C++ has heaps, without knowing the index of an element it would take linear time to find it to re-order it for a changed weight.
//  It is not easy to keep the index of vertexes in the heap because the heap operations are opaque without callbacks. On the other hand, using a self-balancing
//  binary search tree is efficient because it has the same log(n) complexity for insertion and removal of the head element as a binary heap.
//  In addition, a self-balancing binary search tree also allows us to find and remove any other element in log(n) time,
//  allowing us to perform the decrease-key step in logarithmic time by removing and re-inserting.
//  We do not need to keep track of whether a vertex is "done" ("visited") as in the Wikipedia description,
//  since re-reaching such a vertex will always fail the relaxation condition (when re-reaching a "done" vertex, the new
//  distance will never be less than it was originally), so it will be skipped anyway.

/** This function is copied straight from http://rosettacode.org/wiki/Dijkstra's_algorithm#C.2B.2B
  * It does the great majority of the heavy lifting for pathfinding.
  *
 */

void DijkstraComputePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list,
                          std::vector<weight_t> &min_distance,
                          std::vector<vertex_t> &previous)
{
    int n = adjacency_list.size();
    min_distance.clear();
    min_distance.resize(n, max_weight);
    min_distance[source] = 0;
    previous.clear();
    previous.resize(n, -1);
    std::set<std::pair<weight_t, vertex_t> > vertex_queue;
    vertex_queue.insert(std::make_pair(min_distance[source], source));

    while (!vertex_queue.empty())
    {
        weight_t dist = vertex_queue.begin()->first;
        vertex_t u = vertex_queue.begin()->second;
        vertex_queue.erase(vertex_queue.begin());

        // Visit each edge exiting u
	const std::vector<neighbor> &neighbors = adjacency_list[u];
        for (std::vector<neighbor>::const_iterator neighbor_iter = neighbors.begin();
             neighbor_iter != neighbors.end();
             neighbor_iter++)
        {
            vertex_t v = neighbor_iter->target;
            weight_t weight = neighbor_iter->weight;
            weight_t distance_through_u = dist + weight;
	    if (distance_through_u < min_distance[v]) {
	        vertex_queue.erase(std::make_pair(min_distance[v], v));

	        min_distance[v] = distance_through_u;
	        previous[v] = u;
	        vertex_queue.insert(std::make_pair(min_distance[v], v));

	    }

        }
    }
}

/** This function is copied straight from http://rosettacode.org/wiki/Dijkstra's_algorithm#C.2B.2B
  *
  *
 */
std::list<vertex_t> DijkstraGetShortestPathTo(
    vertex_t vertex, const std::vector<vertex_t> &previous)
{
    std::list<vertex_t> path;
    for ( ; vertex != -1; vertex = previous[vertex])
        path.push_front(vertex);
    return path;
}

/** This function is copied straight from http://rosettacode.org/wiki/Dijkstra's_algorithm#C.2B.2B
  *
  *
 */
int DijkstraExample()
{
    // remember to insert edges both ways for an undirected graph
    adjacency_list_t adjacency_list(6);
    // 0 = a
    adjacency_list[0].push_back(neighbor(1, 7));
    adjacency_list[0].push_back(neighbor(2, 9));
    adjacency_list[0].push_back(neighbor(5, 14));
    // 1 = b
    adjacency_list[1].push_back(neighbor(0, 7));
    adjacency_list[1].push_back(neighbor(2, 10));
    adjacency_list[1].push_back(neighbor(3, 15));
    // 2 = c
    adjacency_list[2].push_back(neighbor(0, 9));
    adjacency_list[2].push_back(neighbor(1, 10));
    adjacency_list[2].push_back(neighbor(3, 11));
    adjacency_list[2].push_back(neighbor(5, 2));
    // 3 = d
    adjacency_list[3].push_back(neighbor(1, 15));
    adjacency_list[3].push_back(neighbor(2, 11));
    adjacency_list[3].push_back(neighbor(4, 6));
    // 4 = e
    adjacency_list[4].push_back(neighbor(3, 6));
    adjacency_list[4].push_back(neighbor(5, 9));
    // 5 = f
    adjacency_list[5].push_back(neighbor(0, 14));
    adjacency_list[5].push_back(neighbor(2, 2));
    adjacency_list[5].push_back(neighbor(4, 9));

    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    DijkstraComputePaths(0, adjacency_list, min_distance, previous);
    std::cout << "Distance from 0 to 4: " << min_distance[4] << std::endl;
    std::list<vertex_t> path = DijkstraGetShortestPathTo(4, previous);
    std::cout << "Path : ";
    std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    std::cout << std::endl;

    return 0;
}

/** This function is mostly a copy from http://rosettacode.org/wiki/Dijkstra's_algorithm#C.2B.2B
  * It is slightly modified to test the behavior of unconnected nodes.
  *
 */
int DijkstraUnconnectedTest()
{
    std::cout << "6 and 7 are unconnected. Testing them." << std::endl;
    // remember to insert edges both ways for an undirected graph
    adjacency_list_t adjacency_list(8);
    // 0 = a
    adjacency_list[0].push_back(neighbor(1, 7));
    adjacency_list[0].push_back(neighbor(2, 9));
    adjacency_list[0].push_back(neighbor(5, 14));
    // 1 = b
    adjacency_list[1].push_back(neighbor(0, 7));
    adjacency_list[1].push_back(neighbor(2, 10));
    adjacency_list[1].push_back(neighbor(3, 15));
    // 2 = c
    adjacency_list[2].push_back(neighbor(0, 9));
    adjacency_list[2].push_back(neighbor(1, 10));
    adjacency_list[2].push_back(neighbor(3, 11));
    adjacency_list[2].push_back(neighbor(5, 2));
    // 3 = d
    adjacency_list[3].push_back(neighbor(1, 15));
    adjacency_list[3].push_back(neighbor(2, 11));
    adjacency_list[3].push_back(neighbor(4, 6));
    // 4 = e
    adjacency_list[4].push_back(neighbor(3, 6));
    adjacency_list[4].push_back(neighbor(5, 9));
    // 5 = f
    adjacency_list[5].push_back(neighbor(0, 14));
    adjacency_list[5].push_back(neighbor(2, 2));
    adjacency_list[5].push_back(neighbor(4, 9));
    // 6
    adjacency_list[6].push_back(neighbor(7, 2));
    adjacency_list[7].push_back(neighbor(6, 2));

    std::vector<weight_t> min_distance;
    std::vector<vertex_t> previous;
    DijkstraComputePaths(0, adjacency_list, min_distance, previous);
    std::cout << "Distance from 0 to 6: " << min_distance[6] << std::endl;
    double inf = std::numeric_limits<double>::infinity();
    if (min_distance[6] == inf) {
        std::cout << "The distance is infinity, indicating it's not connected." << std::endl;
    }
    DijkstraComputePaths(2, adjacency_list, min_distance, previous);
    std::cout << "Distance from 2 to 7: " << min_distance[7] << std::endl;
    DijkstraComputePaths(6, adjacency_list, min_distance, previous);
    std::cout << "Distance from 6 to 7: " << min_distance[7] << std::endl;
    std::cout << "Preparing path from 0 to 7, which are not connected:" << std::endl;
    DijkstraComputePaths(0, adjacency_list, min_distance, previous);
    std::list<vertex_t> path = DijkstraGetShortestPathTo(7, previous);
    std::cout << "Path : ";
    std::copy(path.begin(), path.end(), std::ostream_iterator<vertex_t>(std::cout, " "));
    std::cout << std::endl;

    return 0;
}


// Breadth-First Search (BFS) implementation found from http://www.geeksforgeeks.org/find-if-there-is-a-path-between-two-vertices-in-a-given-graph/
// The code originates from http://www.geeksforgeeks.org/find-if-there-is-a-path-between-two-vertices-in-a-given-graph/
// And the original location at which I found the code referenced is at these StackExchange questions:
// http://stackoverflow.com/questions/20058802/various-ways-and-their-runtime-to-check-if-a-path-exists-between-two-nodes-or-ve
// http://stackoverflow.com/questions/354330/how-to-determine-if-two-nodes-are-connected
/*
// Program to check if there is exist a path between two vertices of a graph.
#include<iostream>
#include <list>

using namespace std;

// This class represents a directed graph using adjacency list representation
class Graph
{
    int V;    // No. of vertices
    list<int> *adj;    // Pointer to an array containing adjacency lists
public:
    Graph(int V);  // Constructor
    void addEdge(int v, int w); // function to add an edge to graph
    bool isReachable(int s, int d);  // returns true if there is a path from s to d
};

Graph::Graph(int V)
{
    this->V = V;
    adj = new list<int>[V];
}

void Graph::addEdge(int v, int w)
{
    adj[v].push_back(w); // Add w to v’s list.
}

// A BFS based function to check whether d is reachable from s.
bool Graph::isReachable(int s, int d)
{
    // Base case
    if (s == d)
      return true;

    // Mark all the vertices as not visited
    bool *visited = new bool[V];
    for (int i = 0; i < V; i++)
        visited[i] = false;

    // Create a queue for BFS
    list<int> queue;

    // Mark the current node as visited and enqueue it
    visited[s] = true;
    queue.push_back(s);

    // it will be used to get all adjacent vertices of a vertex
    list<int>::iterator i;

    while (!queue.empty())
    {
        // Dequeue a vertex from queue and print it
        s = queue.front();
        queue.pop_front();

        // Get all adjacent vertices of the dequeued vertex s
        // If a adjacent has not been visited, then mark it visited
        // and enqueue it
        for (i = adj[s].begin(); i != adj[s].end(); ++i)
        {
            // If this adjacent node is the destination node, then return true
            if (*i == d)
                return true;

            // Else, continue to do BFS
            if (!visited[*i])
            {
                visited[*i] = true;
                queue.push_back(*i);
            }
        }
    }

    return false;
}

// Driver program to test methods of graph class
int main()
{
    // Create a graph given in the above diagram
    Graph g(4);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 2);
    g.addEdge(2, 0);
    g.addEdge(2, 3);
    g.addEdge(3, 3);

    int u = 1, v = 3;
    if(g.isReachable(u, v))
        cout<< "\n There is a path from " << u << " to " << v;
    else
        cout<< "\n There is no path from " << u << " to " << v;

    u = 3, v = 1;
    if(g.isReachable(u, v))
        cout<< "\n There is a path from " << u << " to " << v;
    else
        cout<< "\n There is no path from " << u << " to " << v;

    return 0;
}
*/
