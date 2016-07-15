
#ifndef DIJKSTRA_PATHFINDING_H
#define DIJKSTRA_PATHFINDING_H

#include <map>
#include <vector>
#include <limits>

#include "tupleTriplet.h"
#include "segmentation.h"
#include "plantSegmentationDataContainer.h"
#include "supervoxel_construction.h"


/// These are some typedefs for the Dijkstra code. This should be refactored eventually
/// The implementation of Dijkstra's Algorithm we're using is lifted directly from this site (MIT/X11 licensed):
/// http://rosettacode.org/wiki/Dijkstra's_algorithm#C.2B.2B
/// A few classes have been implemented to interface with it, but the core algorithm implementation
/// is exactly from that site.

typedef int vertex_t;
typedef double weight_t;

const weight_t max_weight = std::numeric_limits<double>::infinity();

/** \brief Container to hold information for pathfinding. Associated with the Dijkstra implementation from http://rosettacode.org/wiki/Dijkstra's_algorithm#C.2B.2B
  */
struct neighbor {
    vertex_t target;
    weight_t weight;
    neighbor(vertex_t arg_target, weight_t arg_weight)
        : target(arg_target), weight(arg_weight) { }
};

typedef std::vector<std::vector<neighbor> > adjacency_list_t;

/** \brief Container to hold data corresponding to a graph that will be used for pathfinding via Dijkstra's algorithm.
  *
  * The primary purpose is to keep track of which node label belongs to which index for the Dijkstra implementation we're using.
  * The edges of the graph will be calculated using the point coordinates provided as part of _map_nodeLabelToPointCoordinate
  * \author Ryan McCormick
  */
class Graph {
    public:
        Graph();
        Graph(std::multimap<uint32_t, uint32_t> inputNodeLabelAdjacency, std::map<uint32_t, TupleTriplet> inputNodeLabelToPointCoordinate);
        Graph(pcl::PolygonMesh *inputMesh);
        ~Graph();

        int buildGraph(); /*!< \brief Uses the input node adjacency and point coordinates to construct the internal data structure used by the Dijkstra code. > */
        int setNodeLabelAdjacency(std::multimap<uint32_t, uint32_t> inputNodeLabelAdjacency);
        int setNodeLabelToPointCoordinates(std::map<uint32_t, TupleTriplet> inputNodeLabelToPointCoordinate);
        adjacency_list_t getAdjacencyList(); /*!< \brief Returns the adjacency list used by the internal Dijkstra code. > */

        adjacency_list_t _adjacencyList;
        std::map<uint32_t, uint32_t> _map_nodeLabelToListIndex; /*!< \brief Maps an integer node ID to its index in the list used by the Dijkstra code. > */
        std::map<uint32_t, uint32_t> _map_listIndexToNodeLabel; /*!< \brief Maps the index in the list used by the Dijkstra code to an integer node ID. > */
        std::multimap<uint32_t, uint32_t> _map_nodeLabelAdjacency; /*!< \brief The adjacency of node IDs that will be used to construct the internal data structures. > */
        std::map<uint32_t, TupleTriplet> _map_nodeLabelToPointCoordinate; /*!< \brief Maps the node ID to a x, y, z coordinate that will be used to calculate edge weights  > */
};

/** \brief Container to hold data corresponding to the path found via Dijkstra's algorithm.
  *
  * \author Ryan McCormick
  */
class PathDataContainer {
    public:
        PathDataContainer();
        ~PathDataContainer();

        vertex_t _sourceIndex; /*!< \brief The index in the adjacency list of the source point > */
        uint32_t _sourceLabel; /*!< \brief The node ID of the source point. > */
        vertex_t _targetIndex; /*!< \brief The index in the adjacency list of the target point  > */
        uint32_t _targetLabel; /*!< \brief The node ID of the target point  > */
        std::list<vertex_t> _pathListOfIndices;  /*!< \brief List of indices to get from the source point to the target point. > */
        std::vector<uint32_t> _pathVectorOfLabels;  /*!< \brief Vector of node IDs to get from the source point to the target point. > */
        weight_t _distance;  /*!< \brief The length of the path. > */

};

/** \brief Class to hold content for pathfinding across a graph.
  *
  * This class does most of the heavy lifting for pathfinding with respect to segmentation.
  * \author Ryan McCormick
  */
class DijkstraPathfinder {
    public:
        DijkstraPathfinder();
        DijkstraPathfinder(Graph inputGraphToTraverse);
        ~DijkstraPathfinder();

        int setGraphToTraverse(Graph inputGraphToTraverse);
        PathDataContainer findPathToFurthestUnsegmentedSupervoxel(int inputLabel, PlantSegmentationDataContainer *inputSegmentationData);
        PathDataContainer findPathToClosestUnsegmentedSupervoxel(int inputLabel, PlantSegmentationDataContainer *inputSegmentationData);
        PathDataContainer findPathFromSourceLabelToUnsegmentedConnectedLabelThatIsClosestToUnconnectedLabel(uint32_t inputSourceLabel, uint32_t inputTargetLabel, PlantSegmentationDataContainer *inputSegmentationData);
        PathDataContainer findPathToClosestStemPoint(uint32_t inputLabel, PlantSegmentationDataContainer *inputSegmentationData);
        PathDataContainer findPathToLowestAndClosestStemPoint(uint32_t inputLabel, PlantSegmentationDataContainer *inputSegmentationData);
        PathDataContainer findPathForIsolatedTip(uint32_t inputLabel, PlantSegmentationDataContainer *inputSegmentationData, InputParameters inputParams);
        PathDataContainer findPathToMostDistantNodeFromInputSource(uint32_t inputLabel);
        uint32_t returnIndexOfInputSubsetMeshClosestToStemBottom(pcl::PolygonMesh *inputLeafMesh, pcl::PolygonMesh *inputSegmentedMesh);
        PathDataContainer findPathBetweenTwoLabels(uint32_t inputSourceLabel, uint32_t inputTargetLabel);
        PathDataContainer findGeodesicDiameter();
        float getZCoordinateOfFirstStemAdjacency(uint32_t inputSourceLabel, uint32_t inputStemTarget, PlantSegmentationDataContainer *inputSegmentationData);
        Graph getGraphToTraverse();


        std::vector<uint32_t> returnLabelsOfLargestConnectedSet();

    private:
        void computePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list,
                          std::vector<weight_t> &min_distance,
                          std::vector<vertex_t> &previous);

        Graph _graphToTraverse;



};

/** Remaing loose code for Dijkstra pathfinding. Needs to be refactored into the DijkstraPathfinder class.
  */
int DijkstraExample();

/** Remaing loose code for Dijkstra pathfinding. Needs to be refactored into the DijkstraPathfinder class.
  */
int DijkstraUnconnectedTest();

/** Remaing loose code for Dijkstra pathfinding. Needs to be refactored into the DijkstraPathfinder class.
  */
void DijkstraComputePaths(vertex_t source,
                          const adjacency_list_t &adjacency_list,
                          std::vector<weight_t> &min_distance,
                          std::vector<vertex_t> &previous);

/** Remaing loose code for Dijkstra pathfinding. Needs to be refactored into the DijkstraPathfinder class.
  */
std::list<vertex_t> DijkstraGetShortestPathTo(vertex_t vertex, const std::vector<vertex_t> &previous);


/** \brief Takes a set of vertices and edges, and returns the largest connected set.
  * \param[in] currentMesh pointer to a pcl::PolygonMesh for which the largest connected subset will be found.
  * \return pcl::PolygonMesh a pcl::PolygonMesh that is the largest connected subset of the input mesh.
  * \author Ryan McCormick
  */
pcl::PolygonMesh returnLargestConnectedMesh(pcl::PolygonMesh *currentMesh);

/** \brief Writes content to the logging file and stdout if the debugging level is set sufficiently high by the user.
  * \param[in] inputMesh pointer to a pcl::PolygonMesh to convert to an adjacency list.
  * \return adjacency_list_t the adjacency list corresponding to the input mesh.
  * \author Ryan McCormick
  */
adjacency_list_t convertPolygonMeshToAdjacencyList(pcl::PolygonMesh *inputMesh);

#endif
