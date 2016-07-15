#include <unordered_map>
#include <map>
#include <tuple>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include "boundingBox.h"
#include "tupleTriplet.h"
#include "visualizer_helper.h"
#include "supervoxel_construction.h"
#include "plantSegmentationDataContainer.h"
#include "segmentation.h"
#include "dijkstraPathfinding.h"
#include "inputParams.h"
#include "loggingHelper.h"

typedef pcl::PointNormal PointNT;

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerXYZ;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerNT;

bool pointsXYZareEqual(pcl::PointXYZ point1, pcl::PointXYZ point2) {
    bool allCoordsAreEqual = true;
    if (point1.x != point2.x) {
        allCoordsAreEqual = false;
    }
    if (point1.y != point2.y) {
        allCoordsAreEqual = false;
    }
    if (point1.z != point2.z) {
        allCoordsAreEqual = false;
    }
    return allCoordsAreEqual;
}



/** Take an input mesh and a point cloud that had been segmented from that mesh.
  * The function should get all of the triangles from the original mesh that would be in the segmented mesh, and output
  * the segmented mesh.
  */
pcl::PolygonMesh extractMeshFromPolygonMeshGivenPointCloud(pcl::PolygonMesh inputMesh, pcl::PointCloud<pcl::PointXYZ>::Ptr segmentedCloud) {
    std::map <TupleTriplet, int> map_segmentedPointsOriginalIndex;
    std::map <TupleTriplet, int> map_segmentedPointsNewIndex;
    std::map <TupleTriplet, int> map_pointsAlreadyAddedToNewCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpSegmentedCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudPoints);

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = segmentedCloud->begin(); it != segmentedCloud->end(); it++) {  /// This needs to be the segmented subset of the original cloud
        TupleTriplet tuple_convertedPoint = convertPclPointXYZtoTupleTriplet(*it);
        map_segmentedPointsOriginalIndex.insert(std::pair<TupleTriplet, int> (tuple_convertedPoint, it - segmentedCloud->begin()));
    }

    // This code block generates a cloud containing the points that make up complete triangles in the segmented cloud and
    // stores the indices those points had in the original mesh.
    std::vector<std::vector<int> > polygonsInSubsetWithOriginalIndices;
    for (uint32_t i = 0; i < inputMesh.polygons.size(); i++) {   /// This needs to be the original polygon mesh
        std::vector<int> v_trianglePointIndices;
        bool allIndicesPresent = true; // default to true, will set to false if any of the three are not present.
        for (uint32_t j = 0; j < inputMesh.polygons[i].vertices.size(); j++) {
            //the vertices value is stored as a uint32_t as written in the PCL documentation
            int singleIndex;
            singleIndex = inputMesh.polygons[i].vertices[j];
            v_trianglePointIndices.push_back(singleIndex);
            //If the point corresponding to this index is not in the segmented cloud, we don't want the triangle.
            pcl::PointXYZ pointCoordinates = cloudPoints->points[ singleIndex ];
            TupleTriplet tuple_pointCoordinates = convertPclPointXYZtoTupleTriplet(pointCoordinates);
            if (map_segmentedPointsOriginalIndex.find(tuple_pointCoordinates) == map_segmentedPointsOriginalIndex.end() ) { //If the index isn't present in the segmented map, toss the triangle.
                allIndicesPresent = false;
            }
        }
        // When all three points of a triangle are in the segmented cloud, record those indices and add points to the new cloud
        if (allIndicesPresent == true) {
            polygonsInSubsetWithOriginalIndices.push_back(v_trianglePointIndices);
            // Get the coordinates of each point from the original cloud.
            for (uint32_t j = 0; j < v_trianglePointIndices.size(); j++) {
                pcl::PointXYZ pointCoordinates = cloudPoints->points[ v_trianglePointIndices[j] ];
                TupleTriplet tuple_pointCoordinates = convertPclPointXYZtoTupleTriplet(pointCoordinates);
                // Check if the point has already been added, to prevent points getting added multiple times for each time they are in a face.
                if (map_pointsAlreadyAddedToNewCloud.find(tuple_pointCoordinates) == map_pointsAlreadyAddedToNewCloud.end()){
                    tmpSegmentedCloud->push_back(pointCoordinates);
                    map_pointsAlreadyAddedToNewCloud.insert(std::pair<TupleTriplet, int> (tuple_pointCoordinates, 1));
                }
                else {
                    // Don't need to do anything if it has already been added.
                }
            }
        }
    }

    map_segmentedPointsOriginalIndex.clear();

    //Get the new indices for the new cloud.
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = tmpSegmentedCloud->begin(); it != tmpSegmentedCloud->end(); it++) {
        TupleTriplet tuple_convertedPoint = convertPclPointXYZtoTupleTriplet(*it);
        map_segmentedPointsNewIndex.insert(std::pair<TuplePointXYZ, int> (tuple_convertedPoint, it - tmpSegmentedCloud->begin()));
    }

    // Now connect the triangles based on how they were in the original cloud.
    std::vector<pcl::Vertices> polygonsInSubset;
    for (uint32_t i = 0; i < polygonsInSubsetWithOriginalIndices.size(); i++) {
        pcl::Vertices triangleFaceIndicesForNewMesh;  //This is the set of three points were trying to find in the new mesh.
        for (uint32_t j = 0; j < polygonsInSubsetWithOriginalIndices[i].size(); j++) {
                pcl::PointXYZ pointCoordinates = cloudPoints->points[ polygonsInSubsetWithOriginalIndices[i][j] ];
                TupleTriplet tuple_pointCoordinates = convertPclPointXYZtoTupleTriplet(pointCoordinates);
                //Find the index of those point in the segmented cloud.
                if (map_segmentedPointsNewIndex.find(tuple_pointCoordinates) == map_segmentedPointsNewIndex.end() ) {
                    std::cerr << "Error: These coordinates should be in the segmented map." << std::endl;
                }
                //Add the triangle based on the index in the segmented cloud.
                else {
                    triangleFaceIndicesForNewMesh.vertices.push_back(map_segmentedPointsNewIndex[tuple_pointCoordinates]);
                }
        }
        polygonsInSubset.push_back(triangleFaceIndicesForNewMesh);
    }
    map_segmentedPointsNewIndex.clear();

    pcl::PolygonMesh segmentedMesh;
    pcl::PCLPointCloud2 meshCloud2;
    pcl::toPCLPointCloud2(*tmpSegmentedCloud, meshCloud2);
    segmentedMesh.cloud = meshCloud2;
    segmentedMesh.polygons = polygonsInSubset;

    return segmentedMesh;

}



PlantSegmentationDataContainer segmentStemWithRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints,
                                                        pcl::PointCloud<pcl::Normal>::Ptr cloudNormals,
                                                        InputParameters inputParams) {
    ColorMap colorMap;
    PlantSegmentationDataContainer containerToReturn;

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr RANSACinliers (new pcl::PointIndices);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (inputParams.sacSegmentationFromNormalsParameters.getMaxIterations());
    seg.setDistanceThreshold (inputParams.sacSegmentationFromNormalsParameters.getDistanceThreshold());
    seg.setNormalDistanceWeight (inputParams.sacSegmentationFromNormalsParameters.getNormalDistanceWeight());
    seg.setRadiusLimits (inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMin(),
                        inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMax());
    seg.setInputCloud (cloudPoints);
    seg.setInputNormals (cloudNormals);

    PCL_INFO ("Performing segmentation..\n");
    seg.segment (*RANSACinliers, *coefficients);

        Eigen::VectorXf vCoefficients(7);
    std::cerr << "Model coefficients:" << std::endl;
    for (uint32_t i = 0; i < coefficients->values.size(); i++) {
            std::cerr << coefficients->values[i] << std::endl;
            vCoefficients[i] = coefficients->values[i];
    }

    //This needs to be optimized, a hardcoded value (e.g. 20) likely will not work for all plants.
    pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> expandedCylinderModel(cloudPoints);
    expandedCylinderModel.setInputCloud(cloudPoints);
    expandedCylinderModel.setInputNormals(cloudNormals);
    std::vector<int> expandedModelInliers;
    expandedCylinderModel.selectWithinDistance(vCoefficients, inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue(), expandedModelInliers);
    std::cout << "Found " << expandedModelInliers.size() << " in expandedModelInliers." << std::endl;
    pcl::PointIndices::Ptr ptr_expandedModelInliers (new pcl::PointIndices);
    ptr_expandedModelInliers->indices = expandedModelInliers;

    pcl::PointCloud<pcl::PointXYZ>::Ptr hollowStemCloud (new pcl::PointCloud<pcl::PointXYZ>);
    //We want to keep the small hollow tube of the stem for measurement
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudPoints);
    extract.setIndices(RANSACinliers);
    extract.setNegative(false);
    extract.filter(*hollowStemCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_remainder (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWholeStem (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_remainderNormals (new pcl::PointCloud<pcl::Normal>);

    //But we want to use the expanded tube for removing the stem
    extract.setInputCloud(cloudPoints);
    extract.setIndices(ptr_expandedModelInliers);
    extract.setNegative(true);
    extract.filter(*cloud_remainder);
    extract.setNegative(false);
    extract.filter(*cloudWholeStem);

    for (uint32_t i = 0; i < cloudPoints->points.size(); i++) {
        pcl::PointXYZ point = cloudPoints->points[i];
        TupleTriplet tuple_convertedPoint = convertPclPointXYZtoTupleTriplet(point);
        TupleTriplet tuple_RGB = colorMap._unsegmented_color;
        containerToReturn._map_segmentedPoints.insert(std::pair<TupleTriplet, TupleTriplet> (tuple_convertedPoint, tuple_RGB));
    }

    for (uint32_t i = 0; i < cloudWholeStem->points.size(); i++) {
        pcl::PointXYZ point = cloudWholeStem->points[i];
        TupleTriplet tuple_convertedPoint = convertPclPointXYZtoTupleTriplet(point);
        TupleTriplet tuple_RGB = colorMap._stem_color;
        containerToReturn._map_segmentedPoints[tuple_convertedPoint] = tuple_RGB;
    }

    return containerToReturn;
}








/** Second version of segmenting stem with learned points so that we can get back to the working original
  */
PlantSegmentationDataContainer segmentFromLearnedPoints(int argc, char **argv,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints,
                                pcl::PointCloud<pcl::Normal>::Ptr cloudNormals,
                                InputParameters inputParams,
                                pcl::visualization::PCLVisualizer *visualizer,
                                int viewport) {
    std::ostringstream logStream;
    ColorMap colorMap;
    LOG.DEBUG("Segmenting stem with learned points.");
    PlantSegmentationDataContainer containerToReturn;

    int segmentStemWithLearnedPointsExpectingAdditionalPlyFile = argc;
    assert(segmentStemWithLearnedPointsExpectingAdditionalPlyFile > 2 && "A second PLY file should be provided that contains the learned stem points.");

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputLearnedCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertedLearnedCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertedLearnedPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertedLearnedStem (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr convertedLearnedStemNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertedLearnedInflorescence (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr convertedLearnedInflorescenceNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::io::loadPLYFile(argv[2], *inputLearnedCloud);

    /// We need to "reconstruct" the learned cloud using the point values from the input mesh, otherwise
    /// we have problems searching it in the tuple map. This assumes the point indices are the same.
    for (uint i = 0; i < cloudPoints->points.size(); i++) {
        pcl::PointXYZ currentPointOriginalCloud = cloudPoints->points[i];
        pcl::Normal currentNormal = cloudNormals->points[i];
        pcl::PointXYZRGBA currentPointLearnedCloud = inputLearnedCloud->points[i];
        pcl::PointXYZRGBA mergedColorPoint;
        mergedColorPoint.x = currentPointOriginalCloud.x;
        mergedColorPoint.y = currentPointOriginalCloud.y;
        mergedColorPoint.z = currentPointOriginalCloud.z;
        mergedColorPoint.r = currentPointLearnedCloud.r;
        mergedColorPoint.g = currentPointLearnedCloud.g;
        mergedColorPoint.b = currentPointLearnedCloud.b;
        mergedColorPoint.a = currentPointLearnedCloud.a;
        convertedLearnedCloud->points.push_back(mergedColorPoint);
        TupleTriplet currentColor(mergedColorPoint.r, mergedColorPoint.g, mergedColorPoint.b);
        if (currentColor == colorMap._stem_color) {
            convertedLearnedStem->points.push_back(currentPointOriginalCloud);
            convertedLearnedStemNormals->points.push_back(currentNormal);
            convertedLearnedPoints->points.push_back(currentPointOriginalCloud);
        }
        if (currentColor == colorMap._inflorescence_color) {
            convertedLearnedInflorescence->points.push_back(currentPointOriginalCloud);
            convertedLearnedInflorescenceNormals->points.push_back(currentNormal);
            convertedLearnedPoints->points.push_back(currentPointOriginalCloud);
        }
    }

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visualizer->addPointCloud(convertedLearnedPoints, ColorHandlerXYZ(convertedLearnedPoints, 255.0, 255.0, 255.0), "learnedPoints", viewport);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying learned points; currently this handles cyan stem and gold inflorescence. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    /// Instead of fitting a cylinder, could we region grow the learned points?
    /////////////////////////////////////////////////////
    /////////////////////////////////////////////////////
    /// Region growing for the stem and inflorescence.
    //Clean up the learned points with region growing.

    pcl::search::Search<pcl::PointXYZ>::Ptr regionGrowingTree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(inputParams.regionGrowingParameters.getMinClusterSize());
    reg.setMaxClusterSize(inputParams.regionGrowingParameters.getMaxClusterSize());
    reg.setSearchMethod(regionGrowingTree);
    reg.setNumberOfNeighbours(inputParams.regionGrowingParameters.getNumberOfNeighbours());
    reg.setInputCloud(convertedLearnedStem);
    reg.setInputNormals(convertedLearnedStemNormals);
    reg.setSmoothnessThreshold((inputParams.regionGrowingParameters.getSmoothnessThreshold() * M_PI) / 180.0); //M_PI/18 = 10 degrees
    reg.setCurvatureThreshold(inputParams.regionGrowingParameters.getCurvatureThreshold());

    std::vector <pcl::PointIndices> clusters;
    LOG.DEBUG("Performing region growing to identify stem.");
    reg.extract (clusters);

    logStream << "The number of clusters found by region growing: " << clusters.size ();
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    uint32_t maxSize = 0;
    pcl::PointIndices indicesOfClusterToKeep;
    for (uint32_t i = 0; i < clusters.size(); i++) {
        if (clusters[i].indices.size() > 10) {   // 10 is magic number to keep it from printing out negligibly small clusters.
            logStream << "Cluster " << i << " has " << clusters[i].indices.size() << " points.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
        }
        if (clusters[i].indices.size() > maxSize) {
            maxSize = clusters[i].indices.size();
            indicesOfClusterToKeep = clusters[i];
            logStream << "Changing cluster to keep to index " << i;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
        }
    }

    pcl::PointIndices::Ptr largestClusterInliers(new pcl::PointIndices(indicesOfClusterToKeep));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudStemSegmentation (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(convertedLearnedStem);
    extract.setIndices(largestClusterInliers);
    extract.setNegative(false);
    extract.filter(*cloudStemSegmentation);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr regionGrowingColoredCloud = reg.getColoredCloud();
        visualizer->updatePointCloud(regionGrowingColoredCloud, "learnedPoints");
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the region growing results for the stem. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visualizer->updatePointCloud(cloudStemSegmentation, "learnedPoints");
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the points to keep as stem Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    // Finished with stem. Now do the same thing for the inflorescence if there are at least a handful of inflorescence points.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInflorescenceSegmentation (new pcl::PointCloud<pcl::PointXYZ>);
    if (convertedLearnedInflorescence->points.size() > 5) {
        reg.setInputCloud(convertedLearnedInflorescence);
        reg.setInputNormals(convertedLearnedInflorescenceNormals);
        std::vector <pcl::PointIndices> clustersInflorescence;
        LOG.DEBUG("Performing region growing to identify inflorescence.");
        reg.extract(clustersInflorescence);

        logStream << "The number of clusters found by region growing: " << clustersInflorescence.size ();
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
        uint32_t maxSizeClusterInflorescence = 0;  // Defined above.
        pcl::PointIndices indicesOfClusterToKeepForInflorescence;
        for (uint32_t i = 0; i < clustersInflorescence.size(); i++) {
            if (clustersInflorescence[i].indices.size() > 10) {   // 10 is magic number to keep it from printing out negligibly small clusters.
                logStream << "Cluster " << i << " has " << clustersInflorescence[i].indices.size() << " points.";
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            }
            if (clustersInflorescence[i].indices.size() > maxSizeClusterInflorescence) {
                maxSizeClusterInflorescence = clustersInflorescence[i].indices.size();
                indicesOfClusterToKeepForInflorescence = clustersInflorescence[i];
                logStream << "Changing cluster to keep to index " << i;
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            }
        }

        pcl::PointIndices::Ptr largestClusterInliersInflorescence (new pcl::PointIndices(indicesOfClusterToKeepForInflorescence));
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(convertedLearnedInflorescence);
        extract.setIndices(largestClusterInliersInflorescence);
        extract.setNegative(false);
        extract.filter(*cloudInflorescenceSegmentation);
    }

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr regionGrowingColoredCloud = reg.getColoredCloud();
        visualizer->updatePointCloud(regionGrowingColoredCloud, "learnedPoints");
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the region growing results for the inflorescence. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visualizer->updatePointCloud(cloudInflorescenceSegmentation, "learnedPoints");
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the points to keep as inflorescence Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    /// Finished Region growing for the stem and inflorescence.
    /////////////////////////////////////////////////////
    /////////////////////////////////////////////////////

    // First, set every point in the cloud to unsegmented
    for (uint32_t i = 0; i < cloudPoints->points.size(); i++) {
        pcl::PointXYZ point = cloudPoints->points[i];
        TupleTriplet tuple_convertedPoint = convertPclPointXYZtoTupleTriplet(point);
        TupleTriplet tuple_RGB = colorMap._unsegmented_color;
        containerToReturn._map_segmentedPoints.insert(std::pair<TupleTriplet, TupleTriplet> (tuple_convertedPoint, tuple_RGB));
    }

    // Then update the points that were learned as stem
    for (uint32_t i = 0; i < cloudStemSegmentation->points.size(); i++) {
        pcl::PointXYZ point = cloudStemSegmentation->points[i];
        TupleTriplet tuple_convertedPoint = convertPclPointXYZtoTupleTriplet(point);
        TupleTriplet tuple_RGB = colorMap._stem_color;
        containerToReturn._map_segmentedPoints[tuple_convertedPoint] = tuple_RGB;
    }

    // Then update the points that were learned as inflorescence
    for (uint32_t i = 0; i < cloudInflorescenceSegmentation->points.size(); i++) {
        pcl::PointXYZ point = cloudInflorescenceSegmentation->points[i];
        TupleTriplet tuple_convertedPoint = convertPclPointXYZtoTupleTriplet(point);
        TupleTriplet tuple_RGB = colorMap._inflorescence_color;
        containerToReturn._map_segmentedPoints[tuple_convertedPoint] = tuple_RGB;
    }

    return containerToReturn;
}


/** This is a third prototype of a segmentation method so that it can be broken and we can get back to the old one if necessary.
  * This time around, we're using an implementation of Dijkstra's Algorithm to find paths over the mesh instead of the
  * geodesic path implementation of Surazhsky et al. "Fast Exact and Approximate Geodesics on Meshes".
  */
int geodesicDistanceSegmentation(int argc, char **argv, pcl::visualization::PCLVisualizer *visualizer, InputParameters inputParams) {
    std::ostringstream logStream;
    int stemViewport = 1;
    int segmentationViewport = 2;
    int supervoxelViewport = 3;
    ColorMap colorMap;
    int leafNumber = 0;

    logStream << "Loading PLY to polygon mesh from file " << argv[1];
    LOG.DEBUG(logStream.str()); logStream.str("");
    pcl::PolygonMesh inputMesh;
    pcl::io::loadPLYFile(argv[1], inputMesh);

    std::string segmentedMeshString = "segmentedMesh";
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visualizer->addPolygonMesh(inputMesh, segmentedMeshString, segmentationViewport);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the mesh to be segmented. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    // Extract clouds from the input mesh for use downstream.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsColored (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsSupervoxeled (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);

    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudPoints);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudPointsColored);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudWithNormals);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudNormals);

    /// Use the stem points identified from machine learning to label the stem.
    PlantSegmentationDataContainer segmentationData = segmentFromLearnedPoints(argc, argv, cloudPoints, cloudNormals, inputParams, visualizer, stemViewport);
    // store a copy of it to use later.
    PlantSegmentationDataContainer stemSegmentationData = segmentationData;

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        updateSegmentedMeshInViewer(&inputMesh, segmentationData, visualizer, segmentedMeshString, segmentationViewport);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the mesh with the learned points segmented. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    /// Partition the cloud into supervoxels that can be used during segmentation
    // Consider refactoring this to a SupervoxelConstructor class to encapsulate this free floating function.
    SupervoxelDataContainer supervoxelData = constructSupervoxels(cloudPointsColored, cloudNormals, inputParams);

    //Trim the supervoxel adjacency map on the basis of geodesic distance.
    logStream << "Prior to trimming, there are " << supervoxelData._supervoxelAdjacency.size() << " supervoxel adjacencies " <<
                    "for " << supervoxelData._supervoxelClusters.size() << " supervoxel labels.";
    LOG.DEBUG(logStream.str()); logStream.str("");
    supervoxelData.trimAdjacencyMapOnGeodesicDistance(inputParams, &inputMesh);
    logStream << "After trimming, there are " << supervoxelData._supervoxelAdjacency.size() << " supervoxel adjacencies " <<
                "for " << supervoxelData._supervoxelClusters.size() << " supervoxel labels.";
    LOG.DEBUG(logStream.str()); logStream.str("");

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr reconstitutedSupervoxelPoints (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PolygonMesh reconstitutedSupervoxelMesh;
        for (uint32_t i = 0; i < cloudPoints->points.size(); i++ ) {
            pcl::PointXYZRGBA newSupervoxelPoint;
            newSupervoxelPoint = supervoxelData._cloudWithPointsColorizedByLabel->points[i];
            reconstitutedSupervoxelPoints->points.push_back(newSupervoxelPoint);
        }
        pcl::PCLPointCloud2 meshSupervoxelCloud;
        pcl::toPCLPointCloud2(*reconstitutedSupervoxelPoints, meshSupervoxelCloud);
        reconstitutedSupervoxelMesh.cloud = meshSupervoxelCloud;
        reconstitutedSupervoxelMesh.polygons = inputMesh.polygons;

        addSupervoxelAdjacencyToViewer(supervoxelData, visualizer, supervoxelViewport);
        visualizer->addPolygonMesh(reconstitutedSupervoxelMesh, "testMesh", supervoxelViewport);

        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the supervoxel adjacency. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }
    // Add the supervoxelData to the segmentation data container.
    segmentationData._supervoxelData = supervoxelData;
    // Convert all supervoxels that have stem points in them to be entirely stem.
    segmentationData.updateLearnedPointsWithSupervoxels();
    segmentationData.updateSupervoxelSegmentationMap();

    /// Traverse the supervoxel adjacency graph using Dijkstra's Algorithm as opposed to the geodesic implementation.
    Graph supervoxelGraph(segmentationData._supervoxelData._supervoxelAdjacency, segmentationData._supervoxelData._map_labelToCentroidOfSupervoxel);
    DijkstraPathfinder dijkstraPathfinder(supervoxelGraph);

    logStream << "The constructed graph contains " << (dijkstraPathfinder.getGraphToTraverse())._map_nodeLabelToListIndex.size() << " nodes. " <<
                "The segmented supervoxel map has " << segmentationData._map_segmentedSupervoxels.size() << " supervoxels.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    /// First, find the index of the supervoxel that contains the stem point with the minimum Z.
    // Consider refactoring this as member of the segmentation data container class.
    int supervoxelMinStemLabel = returnLabelOfSupervoxelWithMinimumStemPoint(segmentationData);
    // Saving this for plotting later.
    PathDataContainer initialPath = dijkstraPathfinder.findPathToFurthestUnsegmentedSupervoxel(supervoxelMinStemLabel, &segmentationData);

    /// This is the first, rough segmentation. It finds unlabeled points that are most distant to the stem bottom and grows them to identify leaves.
    /// This initial rough segmentation gets refined later.
    // While not all of the supervoxels are labeled, process them.
    uint32_t iterationCounter = 0;
    while (segmentationData.allSupervoxelsAreLabeled() == false) {
        // Find the most distant unsegmented supervoxel.
        PathDataContainer currentPathData = dijkstraPathfinder.findPathToFurthestUnsegmentedSupervoxel(supervoxelMinStemLabel, &segmentationData);

        if (iterationCounter % 100 == 0) {
            logStream << "On iteration " << iterationCounter;
            logStream << " There are " << leafNumber << " putative leaves.";
            LOG.DEBUG(logStream.str()); logStream.str("");
            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                updateSegmentedMeshInViewer(&inputMesh, segmentationData, visualizer, segmentedMeshString, segmentationViewport);

                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the updated segmented mesh. Press q to continue.");
                    visualizer->spin();
                }
                else {
                    visualizer->spinOnce();
                }
            }
        }

        // Get the supervoxel corresponding to the label.
        int supervoxelLabel = currentPathData._targetLabel;
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr currentSupervoxel = segmentationData._supervoxelData._supervoxelClusters[supervoxelLabel];

        // Ensuring that the supervoxel is truly unsegmented since the pathfinder defaults to returning index 0
        if (segmentationData._map_segmentedSupervoxels[supervoxelLabel] != colorMap._unsegmented_color) {
            LOG.DEBUG("WARNING: ");
            LOG.DEBUG("WARNING: The segmentation data returned a labeled supervoxel. Breaking out of segmentation.");
            LOG.DEBUG("WARNING: ");
            break;
        }

        // Get the labels of the adjacent supervoxels.
        std::map<TupleTriplet, uint32_t> segmentationLabels;
        std::vector<uint32_t> adjacentLabels = supervoxelData.returnVectorOfSupervoxelAdjacencyLabels(supervoxelLabel);
        for (uint32_t i = 0; i < adjacentLabels.size(); i++) {
            // Get the supervoxel
            uint32_t adjacentLabel = adjacentLabels[i];
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr adjacentSupervoxel = supervoxelData._supervoxelClusters[adjacentLabel];
            // Get the centroid of the supervoxel.
            pcl::PointXYZ singlePoint = returnClosestPointToCentroidGivenSupervoxel(adjacentSupervoxel);
            TupleTriplet tupleSupervoxelPoint(singlePoint.x, singlePoint.y, singlePoint.z);
            // Find the segmentation color of that point to decide if it's labeled.
            TupleTriplet colorLabel = segmentationData._map_segmentedPoints[tupleSupervoxelPoint];
            segmentationLabels.insert(std::pair<TupleTriplet, uint32_t> (colorLabel, adjacentLabels[i]));
        }

        // Check what the segmentation of the adjacent supervoxels to decide what the current supervoxel should be segmented as.
        std::map<TupleTriplet, uint32_t>::iterator labelItr;
        uint32_t numSegmentedBorders = 0;
        bool supervoxelAdjacentToBorder = false;
        bool supervoxelAdjacentToStem = false;
        for (labelItr = segmentationLabels.begin(); labelItr != segmentationLabels.end(); labelItr++) {
            // We'll handle borders separately, so we won't consider them segmented here.
            if (labelItr->first != colorMap._unsegmented_color && labelItr->first != colorMap._border_color) {
                numSegmentedBorders = numSegmentedBorders + 1;
            }
            // We should handle borders separately to avoid growing them.
            if (labelItr->first == colorMap._border_color) {
                supervoxelAdjacentToBorder = true;
            }
            if (labelItr->first == colorMap._stem_color) {
                supervoxelAdjacentToStem = true;
            }

        }

        TupleTriplet colorToAssignAllPointsInCurrentSupervoxel = colorMap._debug_color;
        //Adjacency to the stem automatically means that the supervoxel needs to be a border.
        if (supervoxelAdjacentToStem == true) {
            colorToAssignAllPointsInCurrentSupervoxel = colorMap._border_color;
        }
        // If the tip is actually unconnected to the mesh, just flag it; it won't get added as a leaf tip.
        else if (currentPathData._distance == std::numeric_limits<double>::infinity()) {
            colorToAssignAllPointsInCurrentSupervoxel = colorMap._debug_color;
        }
        // If there are more than two different adjacent segmented supervoxels, set the supervoxel as a border supervoxel.
        else if (numSegmentedBorders >= 2) {
            colorToAssignAllPointsInCurrentSupervoxel = colorMap._border_color;
        }
        // If there is one adjacent segmented supervoxel, we should grow it.
        else if (numSegmentedBorders == 1) {
            colorToAssignAllPointsInCurrentSupervoxel = (segmentationLabels.begin()->first);
        }
        // If the only thing adjacent is a border (which should be rare), we pick the color label that the current
        // supervoxel has the smallest geodesic distance to.
        else if (supervoxelAdjacentToBorder == true) {
            // This is a bit of a relic of the initial segmentation prototype. We'll just set it to debug color for now since we don't want to make it a new leaf.
            colorToAssignAllPointsInCurrentSupervoxel = colorMap._debug_color;
        }
        else {
            // Create a new color.
            leafNumber = leafNumber + 1;
            srand(time(NULL));
            TupleTriplet newLabel(rand()%256, rand()%256, rand()%256);
            if (colorMap._leafColorMap.find(leafNumber) != colorMap._leafColorMap.end()) {
                newLabel = colorMap._leafColorMap[leafNumber];
                segmentationData._map_leafTipLabels.insert(std::pair<uint32_t, TupleTriplet>(supervoxelLabel, newLabel));
            }
            else {
                // On the off chance that the random number is equal to something already present, try again.
                // This will need to be changed to include all colors in the current cloud.
                while (newLabel == colorMap._unsegmented_color || newLabel == colorMap._border_color || newLabel == colorMap._stem_color) {
                    srand(time(NULL));
                    TupleTriplet nextColorAttempt(rand()%256, rand()%256, rand()%256);
                    newLabel = nextColorAttempt;
                }
            }
            colorToAssignAllPointsInCurrentSupervoxel = newLabel;
        }

        //std::cout << "Color to be assigned to the supervoxel:" << std::endl;
        //printTupleTriplet(colorToAssignAllPointsInCurrentSupervoxel);

        //assign the color
        for (uint32_t i = 0; i < currentSupervoxel->voxels_->points.size(); i++) {
            pcl::PointXYZRGBA currentPoint = currentSupervoxel->voxels_->points[i];
            TupleTriplet tupleCurrentPoint(currentPoint.x, currentPoint.y, currentPoint.z);
            segmentationData._map_segmentedPoints[tupleCurrentPoint] = colorToAssignAllPointsInCurrentSupervoxel;
        }

        iterationCounter = iterationCounter + 1;
        segmentationData.updateSupervoxelSegmentationMap();
    }

    // If debugging level is high enough, show the first path as an example.
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        updateSegmentedMeshInViewer(&inputMesh, segmentationData, visualizer, segmentedMeshString, segmentationViewport);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPaths (new pcl::PointCloud<pcl::PointXYZ>);
        int firstSupervoxelLabel = initialPath._sourceLabel;
        int secondSupervoxelLabel = initialPath._targetLabel;
        pcl::PointXYZ firstCentroid = convertTupleTriplettoPclPointXYZ(segmentationData._supervoxelData._map_labelToCentroidOfSupervoxel[firstSupervoxelLabel]);
        pcl::PointXYZ secondCentroid = convertTupleTriplettoPclPointXYZ(segmentationData._supervoxelData._map_labelToCentroidOfSupervoxel[secondSupervoxelLabel]);
        cloudPaths->points.push_back(firstCentroid);
        cloudPaths->points.push_back(secondCentroid);
        visualizer->addPointCloud(cloudPaths, ColorHandlerXYZ(cloudPaths, 255.0, 0.0, 0.0), "BeginAndEnd", segmentationViewport);
        visualizer->addPointCloud(cloudPaths, ColorHandlerXYZ(cloudPaths, 255.0, 0.0, 0.0), "BeginAndEnd2", supervoxelViewport);

        for (uint32_t i = 0; i < initialPath._pathVectorOfLabels.size() - 1; i++) {
            std::stringstream ss;
            ss << i;
            //Using the basic addLine() function crashes with a confusing assertion.
            pcl::PointXYZ firstPathCentroid = convertTupleTriplettoPclPointXYZ(segmentationData._supervoxelData._map_labelToCentroidOfSupervoxel[initialPath._pathVectorOfLabels[i]]);
            pcl::PointXYZ secondPathCentroid = convertTupleTriplettoPclPointXYZ(segmentationData._supervoxelData._map_labelToCentroidOfSupervoxel[initialPath._pathVectorOfLabels[i + 1]]);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                addLineConnectionToViewer(firstPathCentroid, secondPathCentroid, ss.str(), visualizer, segmentationViewport);
            }
        }
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the path and endpoints of the furthestStemPoint geodesic distance. Press shift and + to increase point size. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    /// The rough segmentation is complete.

    { // Begin scope to merge close leaf tips.
    /// Refine the segmentation based on the leaf tips from the rough segmentation above.
    // First, we merge leaf tips that are within a small distance of each other, as these are likely just registration errors.
    // This will be an all vs all comparison.
    std::vector< std::pair<uint32_t, uint32_t> > pairsToMerge;
    std::map <uint32_t, TupleTriplet>::iterator tipItrFirst;
    for (tipItrFirst = segmentationData._map_leafTipLabels.begin(); tipItrFirst != segmentationData._map_leafTipLabels.end(); tipItrFirst++) {
        uint32_t firstLabel = tipItrFirst->first;
        TupleTriplet firstCoordinate = segmentationData._supervoxelData._map_labelToCentroidOfSupervoxel[firstLabel];
        std::map <uint32_t, TupleTriplet>::iterator tipItrSecond;
        for (tipItrSecond = segmentationData._map_leafTipLabels.begin(); tipItrSecond != segmentationData._map_leafTipLabels.end(); tipItrSecond++) {
            uint32_t secondLabel = tipItrSecond->first;
            TupleTriplet secondCoordinate = segmentationData._supervoxelData._map_labelToCentroidOfSupervoxel[secondLabel];
            if (firstLabel == secondLabel) {
                //no need to compare it with itself.
                continue;
            }
            pcl::PointXYZ firstPointCoordinate = convertTupleTriplettoPclPointXYZ(firstCoordinate);
            pcl::PointXYZ secondPointCoordinate = convertTupleTriplettoPclPointXYZ(secondCoordinate);
            float euclideanDistance = pcl::euclideanDistance(firstPointCoordinate, secondPointCoordinate);
            float MAGIC_EUCLIDEAN_DISTANCE = 50.0;  // This should probably be refactored to an input parameter
            if (euclideanDistance < MAGIC_EUCLIDEAN_DISTANCE) { //magic number
                logStream << "Leaf tip labels " << firstLabel << " and " << secondLabel << " are less than " << MAGIC_EUCLIDEAN_DISTANCE << " units from each other in euclidean distance. Testing geodesic distance.";
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

                PathDataContainer geodesicPath = dijkstraPathfinder.findPathBetweenTwoLabels(firstLabel, secondLabel);
                logStream << "Leaf tip labels " << firstLabel << " and " << secondLabel << " have geodesic distance of " << geodesicPath._distance << ".";
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
                float MAGIC_GEODESIC_DISTANCE = MAGIC_EUCLIDEAN_DISTANCE * 3.0; // This should probably be refactored to an input parameter
                if (geodesicPath._distance < MAGIC_GEODESIC_DISTANCE) {
                    //make sure the reverse pairing isn't already in there
                    bool alreadyPresent = false;
                    for (uint32_t i = 0; i < pairsToMerge.size(); i++) {
                        logStream << "Leaf tip labels " << firstLabel << " and " << secondLabel << " are less than " << MAGIC_GEODESIC_DISTANCE <<  " units from each other in geodesic distance. Merging.";
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        std::pair<uint32_t, uint32_t> currentPair = pairsToMerge[i];
                        if (currentPair.first == secondLabel) {
                            if (currentPair.second == firstLabel) {
                                alreadyPresent = true;
                            }
                        }
                    }
                    if (alreadyPresent == false) {
                        pairsToMerge.push_back(std::pair<uint32_t, uint32_t>(firstLabel, secondLabel));
                    }

                }
            }
        }
    }

    for (uint32_t i = 0; i < pairsToMerge.size(); i++) {
        std::pair<uint32_t, uint32_t> currentPair = pairsToMerge[i];
        logStream << "Looking to merge: " << currentPair.first << " and " << currentPair.second;
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        // Figure out which one has more points and choose that as the primary tip.
        TupleTriplet colorFirst = segmentationData._map_segmentedSupervoxels[currentPair.first];
        uint32_t numFirst = 0;
        TupleTriplet colorSecond = segmentationData._map_segmentedSupervoxels[currentPair.second];
        uint32_t numSecond = 0;
        std::map <uint32_t, TupleTriplet>::iterator voxelItr;
        for (voxelItr = segmentationData._map_segmentedSupervoxels.begin(); voxelItr != segmentationData._map_segmentedSupervoxels.end(); voxelItr++) {
            TupleTriplet currentColor = voxelItr->second;
            if (currentColor == colorFirst) {
                numFirst += 1;
            }
            else if (currentColor == colorSecond) {
                numSecond += 1;
            }
            else {
                //empty else
            }
        }
        // Remove the tip corresponding to the smaller color, and swap the color of all supervoxels for counting purposes in the next step.
        logStream << "Node " << currentPair.first << " and node " << currentPair.second << " have " << numFirst << " and " << numSecond << " points, respectively.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        if (numFirst >= numSecond) {
            for (voxelItr = segmentationData._map_segmentedSupervoxels.begin(); voxelItr != segmentationData._map_segmentedSupervoxels.end(); voxelItr++) {
                TupleTriplet currentVoxelColor = voxelItr->second;
                if (currentVoxelColor == colorSecond) {
                    voxelItr->second = colorFirst;
                }
            }
            logStream << "Removing leaf tip node " << currentPair.second << ".";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            segmentationData._map_leafTipLabels.erase(currentPair.second);
        }
        else {
            for (voxelItr = segmentationData._map_segmentedSupervoxels.begin(); voxelItr != segmentationData._map_segmentedSupervoxels.end(); voxelItr++) {
                TupleTriplet currentVoxelColor = voxelItr->second;
                if (currentVoxelColor == colorFirst) {
                    voxelItr->second = colorSecond;
                }
            }
            logStream << "Removing leaf tip node " << currentPair.first << ".";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            segmentationData._map_leafTipLabels.erase(currentPair.first);
        }

    }
    } // End scope to merge close leaf tips.

    { // Begin scope to remove leaf tips with fewer than the specified number of supervoxels
    uint32_t MAGIC_NUMBER_OF_LABELED_SUPERVOXELS_TO_RETAIN_LEAFTIP = 2;
    logStream << "Removing leaf tips for leaves with fewer than " << MAGIC_NUMBER_OF_LABELED_SUPERVOXELS_TO_RETAIN_LEAFTIP << " supervoxels.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");

    std::map <uint32_t, TupleTriplet>::iterator tipItr;
    std::vector<uint32_t> leafTipIDsToRemove;
    for (tipItr = segmentationData._map_leafTipLabels.begin(); tipItr != segmentationData._map_leafTipLabels.end(); tipItr++) {
        uint32_t currentTipLabel = tipItr->first;
        TupleTriplet currentTipColor = tipItr->second;
        uint32_t numberLabeled = 0;
        std::map <uint32_t, TupleTriplet>::iterator voxelItr;
        for (voxelItr = segmentationData._map_segmentedSupervoxels.begin(); voxelItr != segmentationData._map_segmentedSupervoxels.end(); voxelItr++) {
            TupleTriplet currentVoxelColor = voxelItr->second;
            if (currentTipColor == currentVoxelColor) {
                numberLabeled += 1;
            }
        }
        if (numberLabeled < MAGIC_NUMBER_OF_LABELED_SUPERVOXELS_TO_RETAIN_LEAFTIP) {
            leafTipIDsToRemove.push_back(currentTipLabel);
        }
    }
    for (uint32_t i = 0; i < leafTipIDsToRemove.size(); i++) {
        logStream << "Removing leaf tip node " << leafTipIDsToRemove[i] << ".";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        segmentationData._map_leafTipLabels.erase(leafTipIDsToRemove[i]);
    }
    } // End scope to remove leaf tips with fewer than the specified number of supervoxels.

    /// Refine the segmentation.
    PlantSegmentationDataContainer refinedSegmentation = refineSegmentation(inputParams, &inputMesh, &segmentationData, visualizer);
    segmentationData = refinedSegmentation;
    segmentationData.updateSupervoxelSegmentationMap();

    /// And finally, we want to recolor the leaves based on their ordering.
    std::map<TupleTriplet, TupleTriplet> map_colorConversions;
    for (uint32_t i = 0; i < segmentationData._leafOrderBottomToTop.size(); i++) {
        uint32_t currentLabel = segmentationData._leafOrderBottomToTop[i];
        TupleTriplet currentColor = segmentationData._map_segmentedSupervoxels[currentLabel];
        TupleTriplet targetColor = colorMap._leafColorMap[i + 1];
        map_colorConversions[currentColor] = targetColor;
        logStream << "Leaf label " << currentLabel << " with color ";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        printTupleTriplet(currentColor, inputParams.debuggingParameters.getDebuggingLevel(), 2);
        logStream << "Is slated to be leaf " << i + 1 << " with color ";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        printTupleTriplet(targetColor, inputParams.debuggingParameters.getDebuggingLevel(), 2);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsInOriginalMesh (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointsInMeshToWrite (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PolygonMesh completeMeshToWrite = inputMesh;

    pcl::fromPCLPointCloud2(completeMeshToWrite.cloud, *pointsInOriginalMesh);

    // Re-label the colors.
    for (uint32_t i = 0; i < pointsInOriginalMesh->points.size(); i++) {
        pcl::PointXYZ currentPoint = pointsInOriginalMesh->points[i];
        TupleTriplet tupleXYZ = convertPclPointXYZtoTupleTriplet(currentPoint);
        TupleTriplet tupleRGB = segmentationData._map_segmentedPoints[tupleXYZ];
        if (map_colorConversions.find(tupleRGB) != map_colorConversions.end()) {
            segmentationData._map_segmentedPoints[tupleXYZ] = map_colorConversions[tupleRGB];
        }
    }
    segmentationData.updateSupervoxelSegmentationMap();

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        updateSegmentedMeshInViewer(&inputMesh, segmentationData, visualizer, segmentedMeshString, segmentationViewport);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the updated segmented mesh with leaves ordered by color. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    /// Use the stem to align the plant to the z axis.
    for (uint32_t i = 0; i < pointsInOriginalMesh->points.size(); i++) {
        pcl::PointXYZ currentPoint = pointsInOriginalMesh->points[i];

        TupleTriplet tupleXYZ = convertPclPointXYZtoTupleTriplet(currentPoint);
        TupleTriplet tupleRGB = segmentationData._map_segmentedPoints[tupleXYZ];

        pcl::PointXYZRGBA newPoint;
        newPoint.x = currentPoint.x;
        newPoint.y = currentPoint.y;
        newPoint.z = currentPoint.z;
        newPoint.r = std::get<0>(tupleRGB);
        newPoint.g = std::get<1>(tupleRGB);
        newPoint.b = std::get<2>(tupleRGB);
        newPoint.a = 255.0;
        pointsInMeshToWrite->points.push_back(newPoint);
    }

    pcl::PCLPointCloud2 segmentedMeshCloudPCL2;
    pcl::toPCLPointCloud2(*pointsInMeshToWrite, segmentedMeshCloudPCL2);
    completeMeshToWrite.cloud = segmentedMeshCloudPCL2;

    // Save the pre transformation as a comparison
    pcl::io::savePLYFile("segmentation_PLYs/segmentedMeshPreTransformation.ply", completeMeshToWrite);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentedCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentedCloudTransformed (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromPCLPointCloud2(completeMeshToWrite.cloud, *segmentedCloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentedStem (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmentedStemProjected (new pcl::PointCloud<pcl::PointXYZRGBA>);

    float minimumZ = 1000000.0; // Arbitrarily large value
    for (uint32_t i = 0; i < segmentedCloud->points.size(); i++) {
        pcl::PointXYZRGBA currentPoint = segmentedCloud->points[i];
        TupleTriplet currentRGB(currentPoint.r, currentPoint.g, currentPoint.b);
        if (currentRGB == colorMap._stem_color) {
            segmentedStem->points.push_back(currentPoint);
            if (currentPoint.z < minimumZ) {
                minimumZ = currentPoint.z;
            }
        }
    }

    Eigen::Vector4f centroidStem;
    compute3DCentroid(*segmentedStem, centroidStem);
    Eigen::Vector3f translationPoint(centroidStem[0], centroidStem[1], minimumZ);

    pcl::PCA<pcl::PointXYZRGBA> pca;
    pca.setInputCloud(segmentedStem);
    pca.project(*segmentedStem, *segmentedStemProjected);
    logStream << "EigenVectors:\n" << pca.getEigenVectors() << "\n";
    logStream << "EigenValues:\n" << pca.getEigenValues();
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    Eigen::Vector3f firstComponent = (pca.getEigenVectors()).block<3,1>(0,0);
    // If the z component is negative, reverse the orientation of the principal component.
    if (firstComponent(2) < 0) {
        firstComponent(0) = firstComponent(0) * -1.0;
        firstComponent(1) = firstComponent(1) * -1.0;
        firstComponent(2) = firstComponent(2) * -1.0;
    }
    logStream << "First component:\n" << firstComponent;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    Eigen::Vector3f vectorOrigin(0, 0, 1);

    Eigen::Vector3f vector_A = firstComponent;
    Eigen::Vector3f vector_B = vectorOrigin;
    Eigen::Vector3f modelVectorAxisPoint = translationPoint;

    { //scoping
    // This is an implementation of Rodrigues' Rotation Formula
    // https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
    // Following http://math.stackexchange.com/questions/293116/rotating-one-3-vector-to-another?rq=1
    // Problem: Given two 3-vectors, A and B, find the rotation of A so that its orientation matches B.
    // There are some edge cases where this implementation will fail, notably if the norm of the cross product = 0.

    // Step 1: Find axis (X)
    Eigen::Vector3f crossProduct = vector_A.cross(vector_B);
    float crossProductNorm = crossProduct.norm();
    Eigen::Vector3f vector_X = (crossProduct / crossProductNorm);

    // Step 2: Find angle (theta)
    float dotProduct = vector_A.dot(vector_B);
    float norm_A = vector_A.norm();
    float norm_B = vector_B.norm();
    float dotProductOfNorms = norm_A * norm_B;
    float dotProductDividedByDotProductOfNorms = (dotProduct / dotProductOfNorms);
    float thetaAngleRad = acos(dotProductDividedByDotProductOfNorms);

    // Step 3: Construct A, the skew-symmetric matrix corresponding to X
    Eigen::Matrix3f matrix_A = Eigen::Matrix3f::Identity();

    matrix_A(0,0) = 0.0;
    matrix_A(0,1) = -1.0 * (vector_X(2));
    matrix_A(0,2) = vector_X(1);
    matrix_A(1,0) = vector_X(2);
    matrix_A(1,1) = 0.0;
    matrix_A(1,2) = -1.0 * (vector_X(0));
    matrix_A(2,0) = -1.0 * (vector_X(1));
    matrix_A(2,1) = vector_X(0);
    matrix_A(2,2) = 0.0;

    // Step 4: Plug and chug.
    Eigen::Matrix3f IdentityMat = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f firstTerm = sin(thetaAngleRad) * matrix_A;
    Eigen::Matrix3f secondTerm = (1.0 - cos(thetaAngleRad)) * matrix_A * matrix_A;

    Eigen::Matrix3f matrix_R = IdentityMat + firstTerm + secondTerm;

    // This is the rotation matrix. Finished with the Rodrigues' Rotation Formula implementation.
    logStream << "matrix_R:\n" << matrix_R;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    // We copy the rotation matrix into the matrix that will be used for the transformation.
    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
    Transform(0,0) = matrix_R(0,0);
    Transform(0,1) = matrix_R(0,1);
    Transform(0,2) = matrix_R(0,2);
    Transform(1,0) = matrix_R(1,0);
    Transform(1,1) = matrix_R(1,1);
    Transform(1,2) = matrix_R(1,2);
    Transform(2,0) = matrix_R(2,0);
    Transform(2,1) = matrix_R(2,1);
    Transform(2,2) = matrix_R(2,2);

    // Now that we have the rotation matrix, we can use it to also find the translation to move the cloud to the origin.
    // First, rotate a point of interest to the new location.
    Eigen::Vector3f modelVectorAxisPointTransformed =  matrix_R * modelVectorAxisPoint;

    // Add the translation to the matrix.
    Transform(0,3) = modelVectorAxisPointTransformed(0) * (-1.0);
    Transform(1,3) = modelVectorAxisPointTransformed(1) * (-1.0);
    Transform(2,3) = modelVectorAxisPointTransformed(2) * (-1.0);

    // Perform the transformation.
    pcl::transformPointCloud(*segmentedCloud, *segmentedCloudTransformed, Transform);
    } //end scoping

    pcl::PCLPointCloud2 segmentedMeshTransformedCloudPCL2;
    pcl::toPCLPointCloud2(*segmentedCloudTransformed, segmentedMeshTransformedCloudPCL2);
    completeMeshToWrite.cloud = segmentedMeshTransformedCloudPCL2;

    /// Write out the final transformed, segmented mesh.
    pcl::io::savePLYFile("segmentation_PLYs/segmentedMesh.ply", completeMeshToWrite);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visualizer->addPointCloud(segmentedCloudTransformed, "alignedCloud", segmentationViewport);
        visualizer->addCoordinateSystem(300.0, "coordinates", segmentationViewport);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the points of the mesh transformed to the Z axis. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }
	return 0;
}

/** Given the initial rough segmentation, use the leaf tips that it identified to refine the segmentation.
 *
 *
 */
PlantSegmentationDataContainer refineSegmentation(InputParameters inputParams, pcl::PolygonMesh *inputMesh,
                                                    PlantSegmentationDataContainer *inputSegmentationData, pcl::visualization::PCLVisualizer *visualizer) {
    std::ostringstream logStream;
    //int stemViewport = 1;
    int segmentationViewport = 2;
    //int supervoxelViewport = 3;
    std::string segmentedMeshString = "segmentedMesh";

    ColorMap colorMap;

    LOG.DEBUG("Refining segmentation.");
    inputSegmentationData->updateSupervoxelSegmentationMap();

    // Create a new segmentation data container. We need it to have the same supervoxels and stem points as the first.
    PlantSegmentationDataContainer refinedSegmentation(inputMesh);
    std::map<TupleTriplet, TupleTriplet>::iterator segmentedItr;
    for (segmentedItr = refinedSegmentation._map_segmentedPoints.begin(); segmentedItr != refinedSegmentation._map_segmentedPoints.end();
            segmentedItr++) {
            TupleTriplet currentPoint = segmentedItr->first;
            if (inputSegmentationData->_map_segmentedPoints[currentPoint] == colorMap._stem_color) {
                refinedSegmentation._map_segmentedPoints[currentPoint] = colorMap._stem_color;
            }
            if (inputSegmentationData->_map_segmentedPoints[currentPoint] == colorMap._inflorescence_color) {
                refinedSegmentation._map_segmentedPoints[currentPoint] = colorMap._inflorescence_color;
            }
    }
    refinedSegmentation._supervoxelData = inputSegmentationData->_supervoxelData;
    refinedSegmentation.updateSupervoxelSegmentationMap();

    /// Here we want to take a phytomer approach to segmentation and grow from the ground up, in a somewhat opposite fashion to the first approach.
    /// This requires identifying the right order to process the leaves.
    /// Let's loop through the leaf tips and rank them based on their minimum geodesic distance to the stem bottom.
    /// Let's also identify the Z coordinate at which their geodesic path is first adjacent to a stem supervoxel.
    ///
    /// The Z coordinate at which their geodesic path is first adjacent to a stem supervoxel should be the primary feature for ordering.
    /// Leaves below the top of the stem can be ordered by taking the minimum Z coordinate. If the Z coordinates are close, then take the leaf
    /// with the minimum geodesic path length.
    /// Leaves above the top of the stem (i.e., the whorl), should be ordered by maximum geodesic path length, since the most fully
    /// emerged leaves will be the longest.
    ///
    /// First, find the index of the supervoxels that contains the stem points with minimum and maximum Z coordinates.
    int supervoxelMinStemLabel = returnLabelOfSupervoxelWithMinimumStemPoint(refinedSegmentation);
    int supervoxelMaxStemLabel = returnLabelOfSupervoxelWithMaximumStemPoint(refinedSegmentation);

    /// Next, determine how many leaf tips were identified in the initial round, and find the distances for each tip.
    ///
    /// This step seems to be one of the most important parts. Doing it based on geodesic distance to the bottom of the stem
    /// runs into a problem with the whorl.
    /// Maybe we can do it based on the Z coordinate at which the path intercepts a stem point.
    /// Maybe the z coordinate at which the path becomes adjacent to a stem point?
    /// This should be relatively large Zs for the whorl
    ///
    /// We combine two approaches here.
    /// 1) Find the Z coordinate of the first leaf supervoxel that is adjacent to a stem supervoxel, and
    /// 2) Find the geodesic distance between the leaf tip and the stem base.
    Graph initialLeafOrderingGraph(refinedSegmentation._supervoxelData._supervoxelAdjacency, refinedSegmentation._supervoxelData._map_labelToCentroidOfSupervoxel);
    DijkstraPathfinder leafOrderingPathfinder(initialLeafOrderingGraph);

    uint32_t numberLeafTips = inputSegmentationData->_map_leafTipLabels.size();
    refinedSegmentation._map_leafTipLabels = inputSegmentationData->_map_leafTipLabels;
    std::map<uint32_t, float> leafTipDistances;
    std::map<uint32_t, float> zCoordinateOfStemAdjacency;
    std::map<uint32_t, TupleTriplet>::iterator tipIterator;
    for (tipIterator = inputSegmentationData->_map_leafTipLabels.begin(); tipIterator != inputSegmentationData->_map_leafTipLabels.end(); tipIterator++) {
        int tipLabel = tipIterator->first;
        float zVal = leafOrderingPathfinder.getZCoordinateOfFirstStemAdjacency(tipLabel, supervoxelMinStemLabel, &refinedSegmentation);
        PathDataContainer pathData = leafOrderingPathfinder.findPathBetweenTwoLabels(tipLabel, supervoxelMinStemLabel);
        logStream << "Tip label " << tipLabel << " with color ";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        printTupleTriplet(inputSegmentationData->_map_segmentedSupervoxels[tipLabel], inputParams.debuggingParameters.getDebuggingLevel(), 2);
        logStream << "has z adjacency value of " << zVal << " and distance to bottom of " << pathData._distance;
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        leafTipDistances.insert(std::pair<uint32_t, float> (tipLabel, pathData._distance));
        zCoordinateOfStemAdjacency.insert(std::pair<uint32_t, float> (tipLabel, zVal));
    }

    /// Once we have the Z coordinate of the first stem adjacency and the geodesic distance between stem base and leaf tip, we can use that information to order leaves.
    /// First, we use the Z coordinate of the first stem adjacency as the primary feature.
    std::vector<uint32_t> v_traversalOrder;
    std::map<uint32_t, uint32_t> map_indexAlreadyAdded;
    std::map<uint32_t, float>::iterator tipDistanceIterator;
    std::map<uint32_t, float>::iterator zCoordinateOfAdjacencyIterator;
    float zCoordinateOfMaxSupervoxel = std::get<2>(inputSegmentationData->_supervoxelData._map_labelToCentroidOfSupervoxel[supervoxelMaxStemLabel]);
    while (v_traversalOrder.size() < numberLeafTips) {
        float minimumZ = 1000000.0; // Arbitrarily large magic number.
        int labelHolder = 0;
        for (zCoordinateOfAdjacencyIterator = zCoordinateOfStemAdjacency.begin();
                zCoordinateOfAdjacencyIterator != zCoordinateOfStemAdjacency.end(); zCoordinateOfAdjacencyIterator++) {

            float zCoord = zCoordinateOfAdjacencyIterator->second;
            float label = zCoordinateOfAdjacencyIterator->first;

            // If the z coordinate is less than 90% of the current minimum, it's the smaller of the two.
            if ( (zCoord < minimumZ * 0.90) && (map_indexAlreadyAdded.find(label) == map_indexAlreadyAdded.end()) ) {
                minimumZ = zCoord;
                labelHolder = label;
                //std::cout << "New minimum z coordinate. Changing minimumDistance to " << minimumZ << " and labelHolder to " << labelHolder << std::endl;
            }

            // If the z coordinate is within 10%, choose the one with the shortest distance to the bottom of the stem.
            else if ( (zCoord >= minimumZ * 0.90) && (zCoord <= minimumZ * 1.10) && (map_indexAlreadyAdded.find(label) == map_indexAlreadyAdded.end())) {
                int previousMinLabel = labelHolder;
                float currentDistance = leafTipDistances[label];
                float previousDistance = leafTipDistances[previousMinLabel];
                //std::cout << "Z coords are similar for labels " << label << " (zCoord of " << zCoord <<  ") and label " <<
                    //previousMinLabel << " (zCoord of " << zCoordinateOfStemAdjacency[previousMinLabel] << ")." << std::endl;
                //std::cout << "Comparing their distances to leaf tips." << std::endl;
                //std::cout << "Distance for labels: " << label << ": " << currentDistance << " and label " <<
                    //previousMinLabel << ": " << previousDistance << std::endl;

                if (previousDistance < currentDistance) {
                    labelHolder = previousMinLabel;
                    minimumZ = zCoordinateOfStemAdjacency[labelHolder];
                    //std::cout << "Setting label holder for: " << labelHolder << std::endl;

                }
                else {
                    labelHolder = label;
                    minimumZ = zCoordinateOfStemAdjacency[labelHolder];
                    //std::cout << "Setting label holder for: " << labelHolder << std::endl;
                }
            }
        }
        v_traversalOrder.push_back(labelHolder);
        map_indexAlreadyAdded.insert(std::pair<int, int>(labelHolder, 1));
    }

    /// That finishes ordering based on Z coordinate of stem adjacency and minimum leaf tip distance.
    /// This doesn't work well for leaves above the whorl.
    /// For example, if a leaf is part of the whorl, it gives precedence to a young whorl leaf.
    /// So, for each supervoxel in the traversal order that has a stemAdjacency greater than the max of the stem, we re-order it based on maximum leaf distance.
    /// That is, older leaves will be longer than younger leaves above the stem for a typical plant.

    LOG.DEBUG("Reordering the leaves that are above the stem.", inputParams.debuggingParameters.getDebuggingLevel(), 1);

    std::vector<uint32_t> whorlLeavesToOrder;
    std::vector<uint32_t> originalIndices;
    map_indexAlreadyAdded.clear();
    for (uint32_t i = 0; i < v_traversalOrder.size(); i++) {
        int currentLabel = v_traversalOrder[i];
        logStream << "Comparing label " << currentLabel << " with z coordinate of stem adjacency of " << zCoordinateOfStemAdjacency[currentLabel] << " with zCoordinate of max stem supervoxel " <<
                    zCoordinateOfMaxSupervoxel << ".";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        if (zCoordinateOfStemAdjacency[currentLabel] > zCoordinateOfMaxSupervoxel) {
            whorlLeavesToOrder.push_back(currentLabel);
            originalIndices.push_back(i);
        }
    }

    logStream << "The original label order was:\n";
    for (uint32_t i = 0; i < v_traversalOrder.size(); i ++) {
        logStream << v_traversalOrder[i] << " ";
    }
    logStream << "\n";

    logStream << "The whorl leaves that need ordering are:\n";
    for (uint32_t i = 0; i < whorlLeavesToOrder.size(); i ++) {
        logStream << " Whorl leaf label " << whorlLeavesToOrder[i] << " with original index of " << originalIndices[i] << "\n";
    }
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    std::vector<int> whorlTraversalOrder;
    while (whorlTraversalOrder.size() < whorlLeavesToOrder.size()) {
        float maximumDistance = -1000000.0; // Arbitrarily small magic number.
        int labelHolder = 0;
        for (uint32_t i = 0; i < whorlLeavesToOrder.size(); i++) {
            int label = whorlLeavesToOrder[i];
            float currentDistance = leafTipDistances[label];
            if (currentDistance > maximumDistance && (map_indexAlreadyAdded.find(label) == map_indexAlreadyAdded.end())) {
                maximumDistance = currentDistance;
                labelHolder = label;
            }
        }
        whorlTraversalOrder.push_back(labelHolder);
        map_indexAlreadyAdded.insert(std::pair<int, int>(labelHolder, 1));
    }

    // Update the whorl leaf indices:
    for (uint32_t i = 0; i < originalIndices.size(); i++) {
        int originalIndex = originalIndices[i];
        int newLabel = whorlTraversalOrder[i];
        v_traversalOrder[originalIndex] = newLabel;
    }

    logStream << "The new order is:\n";
    for (uint32_t i = 0; i < v_traversalOrder.size(); i ++) {
        logStream << v_traversalOrder[i] << " ";
    }
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    // We also want to save the traversal order for later.
    refinedSegmentation._leafOrderBottomToTop = v_traversalOrder;

    /// Done finding the ordering that we want to work with the leaves.

    /// Moving on with refinement. Trace the path from each leaf tip to the stem.
    //For each tip, color each supervoxel in its path the color of the leaf tip.
    for (uint32_t i = 0; i < v_traversalOrder.size(); i++) {
        int currentSupervoxelLabel = v_traversalOrder[i];
        TupleTriplet currentColor = inputSegmentationData->_map_segmentedSupervoxels[currentSupervoxelLabel];
        // Update the adjacency list based on what has been segmented.
        std::multimap<uint32_t, uint32_t> nonLeafAdjacencies = refinedSegmentation.returnSupervoxelAdjacencyForNonLeafSupervoxels();
        Graph updatedGraph(nonLeafAdjacencies, refinedSegmentation._supervoxelData._map_labelToCentroidOfSupervoxel);
        DijkstraPathfinder dijkstraPathfinder(updatedGraph);

        PathDataContainer pathData = dijkstraPathfinder.findPathToLowestAndClosestStemPoint(currentSupervoxelLabel, &refinedSegmentation);

        // The Dijkstra code returns std::numeric_limits<double>::infinity() as distance if there is no connection.
        if (pathData._distance == std::numeric_limits<double>::infinity()) {
            LOG.DEBUG("No path found. Handling the isolated leaf tip.");
            Graph currentSegmentationGraph(refinedSegmentation._supervoxelData._supervoxelAdjacency, refinedSegmentation._supervoxelData._map_labelToCentroidOfSupervoxel);
            DijkstraPathfinder isolatedTipPathfinder(currentSegmentationGraph);
            pathData = isolatedTipPathfinder.findPathForIsolatedTip(currentSupervoxelLabel, &refinedSegmentation, inputParams);
            logStream << "Finished finding a path for the isolated tip. The path contains the labels: ";
            for (uint32_t vecIndex = 0; vecIndex < pathData._pathVectorOfLabels.size(); vecIndex ++) {
                logStream << pathData._pathVectorOfLabels[vecIndex] << " ";
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        }

        // Label each supervoxel in the path with the current color.
        for (uint32_t j = 0; j < pathData._pathVectorOfLabels.size(); j++) {
            uint32_t currentPathLabel = pathData._pathVectorOfLabels[j];
            //assign the color so long as the supervoxel isn't a stem.
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr currentSupervoxel = inputSegmentationData->_supervoxelData._supervoxelClusters[currentPathLabel];
            for (uint32_t k = 0; k < currentSupervoxel->voxels_->points.size(); k++) {
                pcl::PointXYZRGBA currentPoint = currentSupervoxel->voxels_->points[k];
                TupleTriplet tupleCurrentPoint(currentPoint.x, currentPoint.y, currentPoint.z);
                refinedSegmentation._map_segmentedPoints[tupleCurrentPoint] = currentColor;
            }

            /// We should probably do something like: Label adjacent supervoxels except when one of the adjacent supervoxels is labeled.
            /// in which case don't.
            // Check all of the adjacencies to see if any are labeled.
            bool segmentedAdjacencyExists = false;
            std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
            for ( adjacent_itr = inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).first;
                    adjacent_itr != inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).second; adjacent_itr++) {
                uint32_t adjacentLabel = adjacent_itr->second;
                pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr adjacentSupervoxel = inputSegmentationData->_supervoxelData._supervoxelClusters[adjacentLabel];
                //if (refinedSegmentation._map_segmentedSupervoxels[adjacentLabel] == colorMap._stem_color) {
                // Let's try it out such that if anything adjacent is labeled, don't label all of the adjacencies.
                if (refinedSegmentation._map_segmentedSupervoxels[adjacentLabel] != colorMap._unsegmented_color) {
                    segmentedAdjacencyExists = true;
                }
            }
            // if none of the adjacencies are labeled, label them.
            if (segmentedAdjacencyExists == false) {
                for ( adjacent_itr = inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).first;
                        adjacent_itr != inputSegmentationData->_supervoxelData._supervoxelAdjacency.equal_range(currentPathLabel).second; adjacent_itr++) {
                    uint32_t adjacentLabel = adjacent_itr->second;
                    if (refinedSegmentation._map_segmentedSupervoxels[adjacentLabel] == colorMap._unsegmented_color) {
                        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr adjacentSupervoxel = inputSegmentationData->_supervoxelData._supervoxelClusters[adjacentLabel];
                        for (uint32_t k = 0; k < adjacentSupervoxel->voxels_->points.size(); k++) {
                            pcl::PointXYZRGBA currentPoint = adjacentSupervoxel->voxels_->points[k];
                            TupleTriplet tupleCurrentPoint(currentPoint.x, currentPoint.y, currentPoint.z);
                            refinedSegmentation._map_segmentedPoints[tupleCurrentPoint] = currentColor;
                        }
                    }
                }
            }

        }

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            updateSegmentedMeshInViewer(inputMesh, refinedSegmentation, visualizer, segmentedMeshString, segmentationViewport);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the mesh with the segmented stem during refinement. Press q to continue.");
                visualizer->spin();
            }
            else {
                visualizer->spinOnce();
            }
        }

    }

    /// Now we fill in the remaining unlabeled supervoxels using a majority rules fill-in.
    /// Each iteration, this fills in supervoxels with the most labeled adjacencies based on which color is most abundant in the adjacencies.
    LOG.DEBUG("Filling in unlabeled supervoxels. This may take a moment.");
    while (refinedSegmentation.allSupervoxelsAreLabeled() == false) {
        //Find the supervoxels with the most segmented adjacenies.
        std::vector<uint32_t> voxelLabelsToProcess;
        std::map <uint32_t, TupleTriplet>::iterator supervoxelItr;
        int maximumAdjacency = 0;
        for (supervoxelItr = refinedSegmentation._map_segmentedSupervoxels.begin();
                    supervoxelItr != refinedSegmentation._map_segmentedSupervoxels.end(); supervoxelItr++) {
            int currentAdjacency = 0;
            uint32_t currentLabel = supervoxelItr->first;
            TupleTriplet currentColor = supervoxelItr->second;

            // If the voxel is uncolored.
            if (currentColor == colorMap._unsegmented_color) {

                // For starters, just make sure the point can be reached.
                PathDataContainer testPath = leafOrderingPathfinder.findPathBetweenTwoLabels(supervoxelMinStemLabel, currentLabel);
                if (testPath._distance == std::numeric_limits<double>::infinity()) {
                    //if it can't be reached, label it with the debug color and move on.
                    pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr currentSupervoxel = refinedSegmentation._supervoxelData._supervoxelClusters[currentLabel];
                    for (uint32_t k = 0; k < currentSupervoxel->voxels_->points.size(); k++) {
                        pcl::PointXYZRGBA currentPoint = currentSupervoxel->voxels_->points[k];
                        TupleTriplet tupleCurrentPoint(currentPoint.x, currentPoint.y, currentPoint.z);
                        refinedSegmentation._map_segmentedPoints[tupleCurrentPoint] = colorMap._debug_color;
                    }
                    continue;
                }

                //Count how many of its neighbors are colored.
                std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
                for ( adjacent_itr = refinedSegmentation._supervoxelData._supervoxelAdjacency.equal_range(currentLabel).first;
                            adjacent_itr != refinedSegmentation._supervoxelData._supervoxelAdjacency.equal_range(currentLabel).second; adjacent_itr++) {
                    uint32_t adjacentLabel = adjacent_itr->second;
                    // increment the counter if the adjacency is segmented.
                    // Let's try not considering stem points.
                    if (refinedSegmentation._map_segmentedSupervoxels[adjacentLabel] != colorMap._unsegmented_color) {
                        currentAdjacency += 1;
                    }
                }
            }
            if (currentAdjacency > maximumAdjacency) {
                maximumAdjacency = currentAdjacency;
                voxelLabelsToProcess.clear();
                voxelLabelsToProcess.push_back(currentLabel);
            }
            else if (currentAdjacency == maximumAdjacency) {
                voxelLabelsToProcess.push_back(currentLabel);
            }
            else {
                // Don't need to do anything with the label.
            }
        }

        // Now we have a list of voxel labels to process. For each label

        for (uint32_t i = 0; i < voxelLabelsToProcess.size(); i++) {
            uint32_t currentLabel = voxelLabelsToProcess[i];
            //Figure out how many of each color are around.
            std::map<TupleTriplet, int> colorCounter;
            std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
            for ( adjacent_itr = refinedSegmentation._supervoxelData._supervoxelAdjacency.equal_range(currentLabel).first;
                            adjacent_itr != refinedSegmentation._supervoxelData._supervoxelAdjacency.equal_range(currentLabel).second; adjacent_itr++) {
                uint32_t adjacentLabel = adjacent_itr->second;
                TupleTriplet adjacentColor = refinedSegmentation._map_segmentedSupervoxels[adjacentLabel];
                if (adjacentColor != colorMap._unsegmented_color) {
                    //std::cout << "Adjacent color is: ";
                    //printTupleTriplet(adjacentColor);
                    if (colorCounter.find(adjacentColor) == colorCounter.end()) {
                        colorCounter.insert(std::pair<TupleTriplet, int>(adjacentColor, 1));
                    }
                    else {
                        colorCounter[adjacentColor] += 1;
                    }
                }
            }
            // Find the max color.
            int maxValue = -10000;
            TupleTriplet maxColor = colorMap._debug_color;
            std::map<TupleTriplet, int>::iterator colorItr;
            for (colorItr = colorCounter.begin(); colorItr != colorCounter.end(); colorItr++) {
                // We don't want to grow stems if we can avoid it, so check the next color if it's a stem.
                if (colorItr->second > maxValue && colorItr->first != colorMap._stem_color) {
                    maxValue = colorItr->second;
                    maxColor = colorItr->first;
                }
            }
            // But if the only color is a stem, we have no choice.
            if (colorCounter.size() == 1 && colorCounter.find(colorMap._stem_color) != colorCounter.end()) {
                maxColor = colorMap._stem_color;
            }

            //Color all of the points that color.
            //std::cout << "Filling in label " << currentLabel << " with color ";
            //printTupleTriplet(maxColor);

            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr currentSupervoxel = refinedSegmentation._supervoxelData._supervoxelClusters[currentLabel];
            for (uint32_t k = 0; k < currentSupervoxel->voxels_->points.size(); k++) {
                pcl::PointXYZRGBA currentPoint = currentSupervoxel->voxels_->points[k];
                TupleTriplet tupleCurrentPoint(currentPoint.x, currentPoint.y, currentPoint.z);
                refinedSegmentation._map_segmentedPoints[tupleCurrentPoint] = maxColor;
            }

        }

        refinedSegmentation.updateSupervoxelSegmentationMap();

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            updateSegmentedMeshInViewer(inputMesh, refinedSegmentation, visualizer, segmentedMeshString, segmentationViewport);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the mesh with the segmented stem during refinement. Press q to continue.");
                visualizer->spin();
            }
            else {
                visualizer->spinOnce();
            }
        }
    }

    /// Finally, we need to make sure that all of the leaf tips in the original ordering are well represented in the final
    /// refined mesh. If they aren't, we should remove the tip and recolor points as necessary. We should also recolor debug color points
    /// based on the nearest euclidean color.
    { // begin scope to clean out tips and debugging color points
    LOG.DEBUG("Cleaning up minor leaf tips and debugging colors.");
    std::map<uint32_t, TupleTriplet>::iterator tipItr;
    std::map<TupleTriplet, TupleTriplet>::iterator pointItr;
    std::map <uint32_t, TupleTriplet> updatedLeafTipMap;
    std::map <TupleTriplet, uint32_t> colorsToReplace;
    std::vector<uint32_t> updatedLeafTipTraversalOrder;
    for (tipItr = refinedSegmentation._map_leafTipLabels.begin(); tipItr != refinedSegmentation._map_leafTipLabels.end(); tipItr++) {
        int tipLabel = tipItr->first;
        TupleTriplet tipColor = tipItr->second;
        int numPointsWithColor = 0;

        std::map<TupleTriplet, TupleTriplet>::iterator pointItr;
        for (pointItr = refinedSegmentation._map_segmentedPoints.begin(); pointItr != refinedSegmentation._map_segmentedPoints.end(); pointItr++) {
            TupleTriplet pointColor = pointItr->second;
            if (pointColor == tipColor) {
                numPointsWithColor += 1;
            }
        }

        if (numPointsWithColor > 10) { // Arbitrary magic number of points.
            updatedLeafTipMap.insert(std::pair<uint32_t, TupleTriplet>(tipLabel, tipColor));
            logStream << "Tip label " << tipLabel << " with color ";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            printTupleTriplet(tipColor, inputParams.debuggingParameters.getDebuggingLevel(), 2);
            logStream << "\t has " << numPointsWithColor << " points in the mesh.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        }
        else {
            logStream << "Tip label " << tipLabel << " with color ";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");
            printTupleTriplet(tipColor, inputParams.debuggingParameters.getDebuggingLevel(), 0);
            LOG.DEBUG("\tis going to be removed because of a lack of points associated with it; its points, if any, will be changed to the closest color in euclidean distance.");
            colorsToReplace.insert(std::pair<TupleTriplet, uint32_t>(tipColor, 1));
        }
    }
    // Now need to check the leaf orders and get rid of tips to be removed.
    for (uint32_t i = 0; i < refinedSegmentation._leafOrderBottomToTop.size(); i++) {
        uint32_t currentTipLabel = refinedSegmentation._leafOrderBottomToTop[i];
        if (updatedLeafTipMap.find(currentTipLabel) != updatedLeafTipMap.end()) {
            updatedLeafTipTraversalOrder.push_back(currentTipLabel);
        }
    }

    refinedSegmentation._leafOrderBottomToTop = updatedLeafTipTraversalOrder;
    refinedSegmentation._map_leafTipLabels = updatedLeafTipMap;
    refinedSegmentation.updateSupervoxelSegmentationMap();

    LOG.DEBUG("Updated leaf labeling: ");
    for (uint32_t i = 0; i < refinedSegmentation._leafOrderBottomToTop.size(); i++) {
        uint32_t currentLabel = refinedSegmentation._leafOrderBottomToTop[i];
        TupleTriplet currentColor = refinedSegmentation._map_segmentedSupervoxels[currentLabel];
        logStream << "Leaf label " << currentLabel << " with color ";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        printTupleTriplet(currentColor, inputParams.debuggingParameters.getDebuggingLevel(), 2);
    }

    // We've updated the leaf tip map and traversal order. We also need to update the colors in the map.
    // We also want to replace the debugging color.
    LOG.DEBUG("Searching for colors that need to be replaced, if any.");
    colorsToReplace.insert(std::pair<TupleTriplet, uint32_t>(colorMap._debug_color, 1));
    // For each point, we check if it's color needs to be replaced, and then find a replacement color based on euclidean distance.
    for (pointItr = refinedSegmentation._map_segmentedPoints.begin(); pointItr != refinedSegmentation._map_segmentedPoints.end(); pointItr++) {
        TupleTriplet currentPoint = pointItr->first;
        TupleTriplet pointColor = pointItr->second;
        // If this evaluates to true, then we need to substitute the color of this point with a new color.
        if (colorsToReplace.find(pointColor) != colorsToReplace.end()) {
            logStream << "Attempting to find replacement color for point with color: ";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            printTupleTriplet(currentPoint, inputParams.debuggingParameters.getDebuggingLevel(), 2);
            printTupleTriplet(pointColor, inputParams.debuggingParameters.getDebuggingLevel(), 2);

            // We should find the point with the minimum euclidean distance to the point. This involes iterating through the points again.
            float minDistance = 100000.0; //Arbitrarily large magic number.
            TupleTriplet minimumColor(colorMap._debug_color);
            std::map<TupleTriplet, TupleTriplet>::iterator secondPointItr;
            for (secondPointItr = refinedSegmentation._map_segmentedPoints.begin(); secondPointItr != refinedSegmentation._map_segmentedPoints.end(); secondPointItr++) {
                TupleTriplet secondPoint = secondPointItr->first;
                TupleTriplet secondColor = secondPointItr->second;
                pcl::PointXYZ firstPCLPoint = convertTupleTriplettoPclPointXYZ(currentPoint);
                pcl::PointXYZ secondPCLPoint = convertTupleTriplettoPclPointXYZ(secondPoint);
                float distance = pcl::euclideanDistance(firstPCLPoint, secondPCLPoint);
                // if the distance is smaller than the minimum distance...
                if (distance < minDistance) {
                    // and if the point is not among the colors to replace, then we've found a new potential point color.
                    if (colorsToReplace.find(secondColor) == colorsToReplace.end()) {
                        minDistance = distance;
                        minimumColor = secondColor;
                    }
                }
            }
            // We've checked all the points, now we can assign it.
            refinedSegmentation._map_segmentedPoints[currentPoint] = minimumColor;
        }
    }
    } // end scope to clean out tips and debugging color points.


    LOG.DEBUG("Finished refining segmentation.");
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            updateSegmentedMeshInViewer(inputMesh, refinedSegmentation, visualizer, segmentedMeshString, segmentationViewport);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the refined mesh. Press q to continue.");
                visualizer->spin();
            }
            else {
                visualizer->spinOnce();
            }
    }

    return refinedSegmentation;
}

int writeIndividualComponentsOfSegmentedMesh(pcl::PolygonMesh inputMesh, pcl::visualization::PCLVisualizer *visualizer) {

    ColorMap colorMap;
    PlantSegmentationDataContainer segmentationData(&inputMesh);

    std::map<TupleTriplet, int> colorMapOfCurrentMesh;
    std::map<TupleTriplet, TupleTriplet>::iterator segmentedPointsItr;

    for (segmentedPointsItr = segmentationData._map_segmentedPoints.begin();
        segmentedPointsItr != segmentationData._map_segmentedPoints.end();
        segmentedPointsItr++) {
            TupleTriplet currentColor = segmentedPointsItr->second;
            if (colorMapOfCurrentMesh.find(currentColor) == colorMapOfCurrentMesh.end()) {
                colorMapOfCurrentMesh.insert(std::pair<TupleTriplet, int> (segmentedPointsItr->second, 1));
            }
    }
    std::cout << "Found " << colorMapOfCurrentMesh.size() << " colors in the mesh." << std::endl;

    //First, for each color, pull out the mesh corresponding to that color.
    std::vector<pcl::PolygonMesh> v_meshes;
    std::map<TupleTriplet, int>::iterator colorMapItr;
    for (colorMapItr = colorMapOfCurrentMesh.begin(); colorMapItr != colorMapOfCurrentMesh.end(); colorMapItr++) {
        TupleTriplet currentColor = colorMapItr->first;
        std::cout << "Processing the following color from the input mesh: ";
        printTupleTriplet(currentColor);

        //First we collect all of the points in the input mesh that correspond to the current color.
        pcl::PointCloud<pcl::PointXYZ>::Ptr currentCloud (new pcl::PointCloud<pcl::PointXYZ>);
        int pointCounter = 0;
        for (segmentedPointsItr = segmentationData._map_segmentedPoints.begin();
            segmentedPointsItr != segmentationData._map_segmentedPoints.end();
            segmentedPointsItr++) {
                if (TupleTripletsAreEqual(segmentedPointsItr->second, currentColor)) {
                    TupleTriplet currentTuplePoint = segmentedPointsItr->first;
                    pcl::PointXYZ currentPoint(std::get<0>(currentTuplePoint), std::get<1>(currentTuplePoint), std::get<2>(currentTuplePoint));
                    currentCloud->points.push_back(currentPoint);
                    pointCounter = pointCounter + 1;
                }
        }
        std::cout << "Found " << pointCounter << " points that match the color." << std::endl;
        //Skip the color if there are too few points corresponding to it; it likely is some sort of segmentation error.
        //if (pointCounter < 30) {
        //    std::cout << "There are fewer than 30 points of this color. This color will be skipped due to insufficient size." << std::endl;
        //    continue;
        //}
        pcl::PolygonMesh currentMesh = extractMeshFromPolygonMeshGivenPointCloud(inputMesh, currentCloud);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentCloudPointsColored (new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::fromPCLPointCloud2(currentMesh.cloud, *currentCloudPointsColored);

        for (uint32_t i = 0; i < currentCloudPointsColored->points.size(); i++) {
            currentCloudPointsColored->points[i].r = std::get<0>(currentColor);
            currentCloudPointsColored->points[i].g = std::get<1>(currentColor);
            currentCloudPointsColored->points[i].b = std::get<2>(currentColor);
            currentCloudPointsColored->points[i].a = 255.0;
        }

        pcl::PCLPointCloud2 newMeshCloud;
        pcl::toPCLPointCloud2(*currentCloudPointsColored, newMeshCloud);
        currentMesh.cloud = newMeshCloud;
        std::cout << "Keeping a mesh that contains " << currentCloudPointsColored->points.size() << " vertices." << std::endl;
        v_meshes.push_back(currentMesh);
    }

    // Then we output the mesh based on its color.
    for (uint32_t i = 0; i < v_meshes.size(); i++) {
        pcl::PolygonMesh currentMesh = v_meshes[i];
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

        pcl::fromPCLPointCloud2(currentMesh.cloud, *currentCloud);
        pcl::PointXYZRGBA singlePoint = currentCloud->points[0];

        TupleTriplet currentRGB(singlePoint.r, singlePoint.g, singlePoint.b);

        // If the mesh corresponds to a stem
        if (TupleTripletsAreEqual(currentRGB, colorMap._stem_color)) {
            std::string fileOut = "segmentation_PLYs/stem.ply";
            std::cout << "Writing stem out to " << fileOut << std::endl;
            pcl::io::savePLYFile(fileOut, currentMesh);
            pcl::PolygonMesh connectedMesh = returnLargestConnectedMesh(&currentMesh);
            std::string fileOutConnected = "segmentation_PLYs/stem_largestConnected.ply";
            std::cout << "Writing largest connected stem piece out to " << fileOutConnected << std::endl;
            pcl::io::savePLYFile(fileOutConnected, connectedMesh);
            //visu->addPolygonMesh(currentMesh, fileOut, mesh_vp_2);
        }
        // If the mesh corresponds to the border colors
        else if (TupleTripletsAreEqual(currentRGB, colorMap._border_color)) {
            std::string fileOut = "segmentation_PLYs/borders.ply";
            std::cout << "Writing borders out to " << fileOut << std::endl;
            pcl::io::savePLYFile(fileOut, currentMesh);
            pcl::PolygonMesh connectedMesh = returnLargestConnectedMesh(&currentMesh);
            std::string fileOutConnected = "segmentation_PLYs/borders_largestConnected.ply";
            std::cout << "Writing largest connected border piece out to " << fileOutConnected << std::endl;\
            pcl::io::savePLYFile(fileOutConnected, connectedMesh);
            //visu->addPolygonMesh(currentMesh, fileOut, mesh_vp_2);

        }
        // If the mesh corresponds to a leaf
        else if (colorMap._leafColorMap_colorsToLabel.find(currentRGB) != colorMap._leafColorMap_colorsToLabel.end()) {
            uint32_t leafNumber = colorMap._leafColorMap_colorsToLabel[currentRGB];
            std::stringstream ss;
            ss << leafNumber;
            std::string fileOut = "segmentation_PLYs/leaf_" + ss.str() + ".ply";
            std::cout << "Writing leaf out to " << fileOut << std::endl;
            pcl::io::savePLYFile(fileOut, currentMesh);
            pcl::PolygonMesh connectedMesh = returnLargestConnectedMesh(&currentMesh);
            std::string fileOutConnected = "segmentation_PLYs/leaf_" + ss.str() + "_largestConnected.ply";
            std::cout << "Writing largest connected leaf piece out to " << fileOutConnected << std::endl;
            pcl::io::savePLYFile(fileOutConnected, connectedMesh);
            //visu->addPolygonMesh(currentMesh, fileOut, mesh_vp_2);
        }
        else {
            printTupleTriplet(currentRGB);
            bool colorInTheMeshIsInColorMap = false;
            assert(colorInTheMeshIsInColorMap == true && "The color in the mesh (printed above) is not expected in the color map. This likely indicates something is wrong. Aborting.");
        }
    }
    return 0;

}

/** Placeholder until segmentation is worked out.
  *
  */
int segmentationFromPLY(int argc, char** argv, InputParameters inputParams) {
    LOG.DEBUG("Initializing segmentation. This currently expects an input mesh and an input point cloud with approximate stem points labeled.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tFor region growing of the learned stem points:");
    inputParams.regionGrowingParameters.printParameters();
    LOG.DEBUG("\tTo construct supervoxels:");
    inputParams.supervoxelClusteringParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();
    LOG.DEBUG("\tCamera parameters if debugging is enabled:");
    inputParams.cameraParameters.printParameters();

    pcl::visualization::PCLVisualizer *visu;
    int mesh_vp_1, mesh_vp_2, mesh_vp_3;
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer (argc, argv, "PlyViewer");
        visu->initCameraParameters();

        //For a three viewport viewer
        visu->createViewPort (0.00, 0.0, 0.33, 1.0, mesh_vp_1);
        visu->createViewPort (0.33, 0.0, 0.66, 1.0, mesh_vp_2);
        visu->createViewPort (0.66, 0.0, 1.00, 1.0, mesh_vp_3);
        visu->setCameraPosition(inputParams.cameraParameters.getxCoordLocation(),
                            inputParams.cameraParameters.getyCoordLocation(),
                            inputParams.cameraParameters.getzCoordLocation(),
                            inputParams.cameraParameters.getxViewComponent(),
                            inputParams.cameraParameters.getyViewComponent(),
                            inputParams.cameraParameters.getzViewComponent(),
                            inputParams.cameraParameters.getxUpComponent(),
                            inputParams.cameraParameters.getyUpComponent(),
                            inputParams.cameraParameters.getzUpComponent());
    }

    geodesicDistanceSegmentation(argc, argv, visu, inputParams);

    return(0);
}


/** This function is currently a monolithic monster. It needs to be refactored.
  *
  * The premise behind it is:
  *
  * First, crudely cut out points corresponding to the plant with a pass-through filter.
  * Then, use a combined clound to first find a circle corresponding to the pot;
  * we find a circle instead of a cylinder because the cylinder tends to fit remaining portion of the plant rather than the pot.
  * That circle is used to orient the pot to the Z axis at the origin.
  * Then a cylinder and a plane are used to identify the pot and the dirt, respectively.
  * These are removed, and then region growing is used to get rid of any extra pot points.
  */
int segmentOutPot(int argc, char **argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Initializing segmentation of pot from individual images.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo filter points on the x, y, and z dimensions (e.g. to filter out plant part of the image that might harm pot identification):");
    inputParams.passThroughFilterParameters.printParameters();
    LOG.DEBUG("\tTo downsample the point clouds and speed up RANSAC:");
    inputParams.voxelGridFilterParameters.printParameters();
    LOG.DEBUG("\tTo find the circle corresponding to the top of the pot via RANSAC:");
    inputParams.circleRANSACParameters.printParameters();
    LOG.DEBUG("\tTo find the plane corresponding to the dirt surface via RANSAC:");
    inputParams.planeRANSACParameters.printParameters();
    LOG.DEBUG("\tTo remove statistical outliers from the combined cloud and assist RANSAC:");
    inputParams.statisticalOutlierRemovalParameters.printParameters();
    LOG.DEBUG("\tTo calculate normals to for use with region growing and RANSAC :");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo clean up the segmentation via region growing:");
    inputParams.regionGrowingParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();

    pcl::visualization::PCLVisualizer *visu;
    int mesh_vp_1, mesh_vp_2;
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer (argc, argv, "PlyViewer");

        visu->initCameraParameters();

        //For a two viewport viewer
        visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
        visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
    }

    //Combine each individual input cloud into one combined cloud.
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalObject (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalObjectNonVoxelized (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr originalObjectNonVoxelizedTransformed (new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 1; i < argc; i++) {
        // Point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr individualCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 cloudBlob;
        logStream << "Loading point cloud from file:\t" << argv[i];
        LOG.DEBUG(logStream.str()); logStream.str("");
        pcl::io::loadPCDFile(argv[i], cloudBlob);
        pcl::fromPCLPointCloud2(cloudBlob, *individualCloud);
        assert(individualCloud->size() > 5); //5 is an arbitrary value to make sure the cloud has some points.
        // The plus operator can be used to concatenate clouds.
        *originalObject += *individualCloud;
        *originalObjectNonVoxelized += *individualCloud;
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr objectVoxel (new pcl::PointCloud<pcl::PointXYZ>);
    LOG.DEBUG("Downsampling with voxel grid.", inputParams.debuggingParameters.getDebuggingLevel(), 2);
    pcl::VoxelGrid<pcl::PointXYZ> gridObject;

    const float leaf = inputParams.voxelGridFilterParameters.getLeafSize();
    gridObject.setLeafSize (leaf, leaf, leaf);

    gridObject.setInputCloud(originalObject);
    gridObject.filter (*objectVoxel);
    originalObject = objectVoxel;

    //////////////////////////////////////////
    //////////////////////////////////////////

    /// Crudely cut off the top part of the plant so that a large circle doesn't mistakenly get fit to the plant
    /// A better solution would be to constrain the circle radius in a similar manner to how the cylinder radius can be constrained.
    /// However, this doesn't seem to be implemented in PCL right now. So doing this as a temporary fix.
    LOG.DEBUG("Filtering using pass through.", inputParams.debuggingParameters.getDebuggingLevel(), 2);
    //PassThrough filter application
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThrough_Intermediate (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThrough_nonNormalized (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    passThroughFilter.setInputCloud(originalObject);
    passThroughFilter.setFilterFieldName("z");
    passThroughFilter.setFilterLimits(inputParams.passThroughFilterParameters.getZmin(), inputParams.passThroughFilterParameters.getZmax());
    passThroughFilter.filter(*ptr_cloud_filteredPassThrough_Intermediate);
    passThroughFilter.setInputCloud(ptr_cloud_filteredPassThrough_Intermediate);
    passThroughFilter.setFilterFieldName("y");
    passThroughFilter.setFilterLimits(inputParams.passThroughFilterParameters.getYmin(), inputParams.passThroughFilterParameters.getYmax());
    passThroughFilter.filter(*ptr_cloud_filteredPassThrough_Intermediate);
    passThroughFilter.setInputCloud(ptr_cloud_filteredPassThrough_Intermediate);
    passThroughFilter.setFilterFieldName("x");
    passThroughFilter.setFilterLimits(inputParams.passThroughFilterParameters.getXmin(), inputParams.passThroughFilterParameters.getXmax());
    passThroughFilter.filter(*ptr_cloud_filteredPassThrough);

    { //Scoping for statistical outlier removal
    /// In order to help with identification of the circle, we remove statistical outliers from the pass through filtered cloud.
    LOG.DEBUG("Removing statistical outliers prior to circle segmentation.");
    //Statistical removal of outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredStatistical (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(ptr_cloud_filteredPassThrough);
    sor.setMeanK(inputParams.statisticalOutlierRemovalParameters.getMeanK());
    sor.setStddevMulThresh(inputParams.statisticalOutlierRemovalParameters.getStdDevMulThresh());
    sor.filter(*ptr_cloud_filteredStatistical);

    ptr_cloud_filteredPassThrough = ptr_cloud_filteredStatistical;

    } //end scoping for statistical outlier removal

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addPointCloud(ptr_cloud_filteredPassThrough, ColorHandlerXYZ(ptr_cloud_filteredPassThrough, 0.0, 255.0, 0.0), "Presegmentation", mesh_vp_1);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the cloud from which a circle will be used to identify the pot rim. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    //////////////////////////////////////////
    //////////////////////////////////////////
    /// Finding the circle corresponding to the pot and translating it to the origin aligned with the z axis.
    pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>::Ptr circleModel(new pcl::SampleConsensusModelCircle3D<pcl::PointXYZ>(ptr_cloud_filteredPassThrough));

    //Now we should be able to do RANSAC with the points.
    pcl::ModelCoefficients::Ptr circleCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr RANSACcircleInliers (new pcl::PointIndices);

    Eigen::VectorXf vCircleCoefficients(7);
    std::vector<int> circleInliers;

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransacCircle(circleModel);
    ransacCircle.setMaxIterations(inputParams.circleRANSACParameters.getMaxIterations());
    ransacCircle.setDistanceThreshold(inputParams.circleRANSACParameters.getDistanceThreshold());
    LOG.DEBUG("Performing RANSAC to identify a circle model corresponding to the pot.");
    ransacCircle.computeModel();
    ransacCircle.getModelCoefficients(vCircleCoefficients);

    logStream << "Circle model coefficients:" << std::endl;
    for (uint32_t i = 0; i < vCircleCoefficients.size(); i++) {
            logStream << vCircleCoefficients[i] << "\n";
    }
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

    pcl::ExtractIndices<pcl::PointXYZ> extract;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPot (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SampleConsensusModelCircle3D<pcl::PointXYZ> expandedCircleModel(originalObject);
    expandedCircleModel.setInputCloud(originalObject);
    std::vector<int> expandedCircleInliers;
    expandedCircleModel.selectWithinDistance(vCircleCoefficients, inputParams.circleRANSACParameters.getSelectWithinDistanceValue(), expandedCircleInliers);
    logStream << "Found " << expandedCircleInliers.size() << " in expandedCircleInliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    pcl::PointIndices::Ptr ptr_expandedCircleInliers (new pcl::PointIndices);
    ptr_expandedCircleInliers->indices = expandedCircleInliers;

    extract.setInputCloud(originalObject);
    extract.setIndices(ptr_expandedCircleInliers);
    extract.setNegative(false);
    extract.filter(*cloudPot);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addPointCloud(cloudPot, ColorHandlerXYZ(cloudPot, 0.0, 255.0, 0.0), "Inliers", mesh_vp_2);
        visu->addPointCloud(cloudPot, ColorHandlerXYZ(cloudPot, 255.0, 255.0, 0.0), "PotCircle", mesh_vp_1);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the identified circle. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPotTransformed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ modelAxisPoint(vCircleCoefficients[0], vCircleCoefficients[1], vCircleCoefficients[2]);
    Eigen::Vector3f modelVectorAxisPoint(vCircleCoefficients[0], vCircleCoefficients[1], vCircleCoefficients[2]);
    Eigen::Vector3f vectorOrigin(0, 0, 1);
    Eigen::Vector3f vectorModelNormal(vCircleCoefficients[4], vCircleCoefficients[5], vCircleCoefficients[6]);
    pcl::PointXYZ modelVectorNormal(vCircleCoefficients[4], vCircleCoefficients[5], vCircleCoefficients[6]);


    Eigen::Vector3f vector_A = vectorModelNormal;
    Eigen::Vector3f vector_B = vectorOrigin;

    // This is an implementation of Rodrigues' Rotation Formula
    // https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
    // Following http://math.stackexchange.com/questions/293116/rotating-one-3-vector-to-another?rq=1
    // Problem: Given two 3-vectors, A and B, find the rotation of A so that its orientation matches B.
    // There are some edge cases where this implementation will fail, notably if the norm of the cross product = 0.

    // Step 1: Find axis (X)
    Eigen::Vector3f crossProduct = vector_A.cross(vector_B);
    float crossProductNorm = crossProduct.norm();
    Eigen::Vector3f vector_X = (crossProduct / crossProductNorm);

    // Step 2: Find angle (theta)
    float dotProduct = vector_A.dot(vector_B);
    float norm_A = vector_A.norm();
    float norm_B = vector_B.norm();
    float dotProductOfNorms = norm_A * norm_B;
    float dotProductDividedByDotProductOfNorms = (dotProduct / dotProductOfNorms);
    float thetaAngleRad = acos(dotProductDividedByDotProductOfNorms);

    // Step 3: Construct A, the skew-symmetric matrix corresponding to X
    Eigen::Matrix3f matrix_A = Eigen::Matrix3f::Identity();

    matrix_A(0,0) = 0.0;
    matrix_A(0,1) = -1.0 * (vector_X(2));
    matrix_A(0,2) = vector_X(1);
    matrix_A(1,0) = vector_X(2);
    matrix_A(1,1) = 0.0;
    matrix_A(1,2) = -1.0 * (vector_X(0));
    matrix_A(2,0) = -1.0 * (vector_X(1));
    matrix_A(2,1) = vector_X(0);
    matrix_A(2,2) = 0.0;

    // Step 4: Plug and chug.
    Eigen::Matrix3f IdentityMat = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f firstTerm = sin(thetaAngleRad) * matrix_A;
    Eigen::Matrix3f secondTerm = (1.0 - cos(thetaAngleRad)) * matrix_A * matrix_A;

    Eigen::Matrix3f matrix_R = IdentityMat + firstTerm + secondTerm;

    // This is the rotation matrix. Finished with the Rodrigues' Rotation Formula implementation.
    logStream << "matrix_R" << std::endl << matrix_R;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");


    // We copy the rotation matrix into the matrix that will be used for the transformation.
    Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
    Transform(0,0) = matrix_R(0,0);
    Transform(0,1) = matrix_R(0,1);
    Transform(0,2) = matrix_R(0,2);
    Transform(1,0) = matrix_R(1,0);
    Transform(1,1) = matrix_R(1,1);
    Transform(1,2) = matrix_R(1,2);
    Transform(2,0) = matrix_R(2,0);
    Transform(2,1) = matrix_R(2,1);
    Transform(2,2) = matrix_R(2,2);

    // Now that we have the rotation matrix, we can use it to also find the translation to move the cloud to the origin.
    // First, rotate a point of interest to the new location.
    Eigen::Vector3f modelVectorAxisPointTransformed =  matrix_R * modelVectorAxisPoint;

    // Add the translation to the matrix.
    Transform(0,3) = modelVectorAxisPointTransformed(0) * (-1.0);
    Transform(1,3) = modelVectorAxisPointTransformed(1) * (-1.0);
    Transform(2,3) = modelVectorAxisPointTransformed(2) * (-1.0);

    // Perform the transformation. This Transform matrix is very long lived and used waaay at the bottom of the function.
    pcl::transformPointCloud(*cloudPot, *cloudPotTransformed, Transform);

    pcl::PointCloud<pcl::PointXYZ>::Ptr originalObjectTransformed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*originalObject, *originalObjectTransformed, Transform);

    // Check the orientation. Right now we use the centroid, but maybe we should use the max distance between the origin and the top and the bottom.
    bool axisNeedsToBeSwapped = false;
    Eigen::Vector4f centroid;
    compute3DCentroid(*originalObjectTransformed, centroid);
    float centerZ = centroid[2];
    if (centerZ < 0.0) {
        logStream << "Swapping the Z axis of the transformed cloud because current center Z point is " << centerZ;
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
        axisNeedsToBeSwapped = true;
        for (uint32_t i = 0; i < originalObjectTransformed->points.size(); i++) {
            originalObjectTransformed->points[i].z = originalObjectTransformed->points[i].z * (-1.0);
        }
    }
    if (axisNeedsToBeSwapped == true) {
        for (uint32_t i = 0; i < cloudPotTransformed->points.size(); i++) {
            cloudPotTransformed->points[i].z = cloudPotTransformed->points[i].z * (-1.0);
        }
    }

    // At this point we have a circle corresponding to the rim of the pot, and the clouds are aligned to the Z axis.
    // Now I think we want to use the circle to cut off the bottom portion so that we can fit a plane to the dirt.

    float maxZvalue = 0;
    for (uint32_t i = 0; i < cloudPotTransformed->points.size(); i++ ) {
        if (cloudPotTransformed->points[i].z > maxZvalue) {
            maxZvalue = cloudPotTransformed->points[i].z;
        }
    }
    logStream << "Max z value from the pot is: " << maxZvalue;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsBelowZThreshold (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr normalsBelowZThreshold (new pcl::PointCloud<pcl::PointXYZ>);
    for (uint32_t i = 0; i < originalObjectTransformed->points.size(); i++ ) {
        pcl::PointXYZ point = originalObjectTransformed->points[i];
        if (point.z < maxZvalue) {
            pointsBelowZThreshold->points.push_back(point);
        }
    }

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addPointCloud(pointsBelowZThreshold, ColorHandlerXYZ(pointsBelowZThreshold, 255.0, 255.0, 0.0), "PointsBelowThresh", mesh_vp_2);
        visu->addCoordinateSystem(300.0, "coordinate", mesh_vp_1);
        visu->addCoordinateSystem(300.0, "coordinate2", mesh_vp_2);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying all points below the top of the pot transformed to the origin; you may need to reset the camera with \"r\" to center the view. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    /// Finished finding the circle corresponding to the pot and translating it to the origin aligned with the z axis.
    //////////////////////////////////////////
    //////////////////////////////////////////

    //////////////////////////////////////////
    //////////////////////////////////////////
    /// Getting the rest of the pot out using RANSAC
    // I guess we can RANSAC a cylinder out, then a plane for the dirt
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr planeModel(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(pointsBelowZThreshold));

    //Now we should be able to do RANSAC with the points.
    pcl::ModelCoefficients::Ptr planeCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr RANSACplaneInliers (new pcl::PointIndices);

    Eigen::VectorXf vPlaneCoefficients(4);
    std::vector<int> planeInliers;

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransacPlane(planeModel);
    ransacPlane.setMaxIterations (inputParams.sacSegmentationFromNormalsParameters.getMaxIterations());
    ransacPlane.setDistanceThreshold (inputParams.sacSegmentationFromNormalsParameters.getDistanceThreshold());
    LOG.DEBUG("Performing plane segmentation.");
    ransacPlane.computeModel();
    ransacPlane.getModelCoefficients(vPlaneCoefficients);
    ransacPlane.getInliers(planeInliers);

    RANSACplaneInliers->indices = planeInliers;

    logStream << "Plane model coefficients:" << std::endl;
    for (uint32_t i = 0; i < vPlaneCoefficients.size(); i++) {
            logStream << vPlaneCoefficients[i] << "\n";
    }
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_planeRemoved(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(pointsBelowZThreshold);
    extract.setIndices(RANSACplaneInliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    //extract.setNegative(false);
    //extract.filter(*cloud_planeRemoved);

    /////////////////////// Trying to find cylinder.
    LOG.DEBUG("Estimating normals using K earch.");
    pcl::PointCloud<pcl::Normal>::Ptr normals_pointsBelowZThreshold (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_cloudPlaneRemoved (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_cloudPotTransformed (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
    nest.setSearchMethod (treeNormal);
    nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());

    nest.setInputCloud (pointsBelowZThreshold);
    nest.compute (*normals_pointsBelowZThreshold);
    //nest.setInputCloud (cloud_planeRemoved);
    //nest.compute (*normals_cloudPlaneRemoved);
    //nest.setInputCloud (cloudPotTransformed);
    //nest.compute (*normals_cloudPotTransformed);

    /* // This may not actually be necessary since we can make the cylinder model based on the circle radius.
    // I guess we can RANSAC a cylinder out, then a plane for the dirt
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (inputParams.sacSegmentationFromNormalsParameters.getMaxIterations());
    seg.setDistanceThreshold (inputParams.sacSegmentationFromNormalsParameters.getDistanceThreshold());
    seg.setNormalDistanceWeight (inputParams.sacSegmentationFromNormalsParameters.getNormalDistanceWeight());
    seg.setRadiusLimits (inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMin(),
                        inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMax());
    //seg.setInputCloud(pointsBelowZThreshold);
    //seg.setInputNormals(normals_pointsBelowZThreshold);
    //seg.setInputCloud(cloud_planeRemoved);
    //seg.setInputNormals(normals_cloudPlaneRemoved);
    seg.setInputCloud(cloudPotTransformed);
    seg.setInputNormals(normals_cloudPotTransformed);

    pcl::ModelCoefficients::Ptr cylinderCoefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr RANSACcylinderInliers (new pcl::PointIndices);
    PCL_INFO ("Performing cylinder segmentation..\n");
    seg.segment (*RANSACcylinderInliers, *cylinderCoefficients);

    Eigen::VectorXf vCylinderCoefficients(7);
    std::cerr << "Cylinder model coefficients:" << std::endl;
    for (size_t i = 0; i < cylinderCoefficients->values.size(); i++) {
            std::cerr << cylinderCoefficients->values[i] << std::endl;
            vCylinderCoefficients[i] = cylinderCoefficients->values[i];
    }
    */

    /// Instead of estimating the cylinder model, we manually set it to be at the origin and based on the circle radius.
    Eigen::VectorXf vCylinderCoefficients(7);
    vCylinderCoefficients[0] = 0.0; vCylinderCoefficients[1] = 0.0; vCylinderCoefficients[2] = 0.0;
    vCylinderCoefficients[3] = 0.0; vCylinderCoefficients[4] = 0.0; vCylinderCoefficients[5] = 1.0;
    vCylinderCoefficients[6] = vCircleCoefficients[3];

    logStream << "Manual cylinder model coefficients based on the segmented circle:" << std::endl;
    for (uint32_t i = 0; i < vCylinderCoefficients.size(); i++) {
            logStream << vCylinderCoefficients[i] << "\n";
    }
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> expandedCylinderModel(pointsBelowZThreshold);
    expandedCylinderModel.setInputCloud(pointsBelowZThreshold);
    expandedCylinderModel.setInputNormals(normals_pointsBelowZThreshold);
    //expandedCylinderModel.setInputCloud(cloud_planeRemoved);
    //expandedCylinderModel.setInputNormals(normals_cloudPlaneRemoved);
    std::vector<int> expandedModelInliers;
    expandedCylinderModel.selectWithinDistance(vCylinderCoefficients, inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue(), expandedModelInliers);
    logStream << "Found " << expandedModelInliers.size() << " in expandedModelInliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    pcl::PointIndices::Ptr ptr_expandedModelInliers (new pcl::PointIndices);
    ptr_expandedModelInliers->indices = expandedModelInliers;

    extract.setInputCloud(pointsBelowZThreshold);
    extract.setIndices(ptr_expandedModelInliers);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);

    for (uint32_t i = 0; i < cloud_cylinder->points.size(); i++) {
        pcl::PointXYZ currentPoint = cloud_cylinder->points[i];
        if (currentPoint.z > maxZvalue) {
            maxZvalue = currentPoint.z;
        }
    }
    logStream << "Using cylinder, max z value updated to: " << maxZvalue;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    /// Finished getting the rest of the pot out using RANSAC
    //////////////////////////////////////////
    //////////////////////////////////////////

    // So now I think we have the models necessary to get rid of the pot points.
    // I think any point in the original cloud the exists in the cylinder model and beneath the threshold point on the Z axis can be removed
    // And any point in the original cloud that exists in the plane model within the cylinder radius can be removed.
    // Then we can probably use a relaxed region growing to get any additional noise out.
    // Lets construct maps of inliers for the plane model and inliers for the cylinder model.
    // For each point in the whole cloud, if it is in the plane model within the radius, get rid of it.
    // if it is in the cylinder model beneath the max Z, get rid of it.

    //We need to use the non-voxelized cloud for this:
    pcl::transformPointCloud(*originalObjectNonVoxelized, *originalObjectNonVoxelizedTransformed, Transform);
    //Swap the orientation if necessary.
    if (axisNeedsToBeSwapped == true) {
        for (uint32_t i = 0; i < originalObjectNonVoxelizedTransformed->points.size(); i++) {
            originalObjectNonVoxelizedTransformed->points[i].z = originalObjectNonVoxelizedTransformed->points[i].z * (-1.0);
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr final_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_plane(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::Normal>::Ptr normals_originalObjectNonVoxelizedTransformed (new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalFinalCylinder (new pcl::search::KdTree<pcl::PointXYZ> ());
    LOG.DEBUG("Estimating scene normals using K search.");
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest_finalCylinder;
    nest_finalCylinder.setSearchMethod (treeNormalFinalCylinder);
    nest_finalCylinder.setKSearch(inputParams.normalEstimationParameters.getKSearch());
    nest_finalCylinder.setInputCloud(originalObjectNonVoxelizedTransformed);
    nest_finalCylinder.compute(*normals_originalObjectNonVoxelizedTransformed);

    pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> finalCylinderModel(originalObjectNonVoxelizedTransformed);
    finalCylinderModel.setInputCloud(originalObjectNonVoxelizedTransformed);
    finalCylinderModel.setInputNormals(normals_originalObjectNonVoxelizedTransformed);
    std::vector<int> finalCylinderInliers;
    //finalCylinderModel.selectWithinDistance(vCylinderCoefficients, inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue(), finalCylinderInliers);
    /// Need to add another input parameter value for the select within distance here.
    finalCylinderModel.selectWithinDistance(vCylinderCoefficients, 40.0, finalCylinderInliers);
    logStream << "Found " << finalCylinderInliers.size() << " in finalCylinderInliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    pcl::PointIndices::Ptr ptr_finalCylinderInliers (new pcl::PointIndices);
    ptr_finalCylinderInliers->indices = finalCylinderInliers;

    extract.setInputCloud(originalObjectNonVoxelizedTransformed);
    extract.setIndices(ptr_finalCylinderInliers);
    extract.setNegative(false);
    extract.filter(*final_cylinder);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addPointCloud(final_cylinder, ColorHandlerXYZ(final_cylinder, 255.0, 0.0, 0.0), "final_cylinder", mesh_vp_2);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            logStream << "Displaying the cylinder identified as the pot. The cylinder will likely extend into the plant, but only points" <<
                            " below the top of the segmented circle will be removed. Press q to continue.";
            LOG.DEBUG(logStream.str()); logStream.str("");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    std::map<TupleTriplet, int> map_cylinderPoints;
    for (uint32_t i = 0; i < final_cylinder->points.size(); i++ ) {
        pcl::PointXYZ currentPoint = final_cylinder->points[i];
        TupleTriplet tuplePoint = convertPclPointXYZtoTupleTriplet(currentPoint);
        map_cylinderPoints.insert(std::pair<TupleTriplet, int>( tuplePoint, 1));
    }


    pcl::SampleConsensusModelPlane<pcl::PointXYZ> finalPlaneModel(originalObjectNonVoxelizedTransformed);
    finalPlaneModel.setInputCloud(originalObjectNonVoxelizedTransformed);
    std::vector<int> finalPlaneInliers;
    finalPlaneModel.selectWithinDistance(vPlaneCoefficients, inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue(), finalPlaneInliers);
    logStream << "Found " << finalPlaneInliers.size() << " in finalPlaneInliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2);
    pcl::PointIndices::Ptr ptr_finalPlaneInliers (new pcl::PointIndices);
    ptr_finalPlaneInliers->indices = finalPlaneInliers;

    extract.setInputCloud(originalObjectNonVoxelizedTransformed);
    extract.setIndices(ptr_finalPlaneInliers);
    extract.setNegative(false);
    extract.filter(*final_plane);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addPointCloud(final_plane, ColorHandlerXYZ(final_plane, 255.0, 0.0, 255.0), "final_plane", mesh_vp_2);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the plane identified as the pot dirt surface. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    std::map<TupleTriplet, int> map_planePoints;
    for (uint32_t i = 0; i < final_plane->points.size(); i++ ) {
        pcl::PointXYZ currentPoint = final_plane->points[i];
        TupleTriplet tuplePoint = convertPclPointXYZtoTupleTriplet(currentPoint);
        map_planePoints.insert(std::pair<TupleTriplet, int>( tuplePoint, 1));
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithCylinderAndPlaneRemoved (new pcl::PointCloud<pcl::PointXYZ>);
    // Remove points that don't meet specifications.
    for (uint32_t i = 0; i < originalObjectNonVoxelizedTransformed->points.size(); i++ ) {
        pcl::PointXYZ currentPoint = originalObjectNonVoxelizedTransformed->points[i];
        TupleTriplet tuplePoint = convertPclPointXYZtoTupleTriplet(currentPoint);

        if (map_cylinderPoints.find(tuplePoint) != map_cylinderPoints.end()) {
            // Change the max Z value to the max z value of the RANSACed cylinder with some wiggle room.
            if (currentPoint.z > maxZvalue + inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue()) {
                cloudWithCylinderAndPlaneRemoved->points.push_back(currentPoint);
            }

        }
        else if (map_planePoints.find(tuplePoint) != map_planePoints.end()) {
            // If point is within the radius of the cylinder (use X and Y dimensions).
            // Assuming the center of the pot is in the ball park of the origin, (0,0).
            float xd = currentPoint.x;
            float yd = currentPoint.y;
            float distance = sqrt(xd * xd + yd * yd);
            // If the distance is outside of the cylinder radius of the pot with some wiggle room, keep it.
            if (distance > vCylinderCoefficients[6] + inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue() ) {
                cloudWithCylinderAndPlaneRemoved->points.push_back(currentPoint);
            }
        }
        else {
            cloudWithCylinderAndPlaneRemoved->points.push_back(currentPoint);
        }
    }

    /////////////////////////////////////////////////////
    /////////////////////////////////////////////////////
    /// Region growing.
    //And finally, let's clean it up with region growing.

    pcl::PointCloud<pcl::Normal>::Ptr normals_cloudWithCylinderAndPlaneRemoved (new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormalCylinderAndPlaneRemoved (new pcl::search::KdTree<pcl::PointXYZ> ());
    LOG.DEBUG("Estimating scene normals using K search.");
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest_CylinderAndPlaneRemoved;
    nest_finalCylinder.setSearchMethod (treeNormalCylinderAndPlaneRemoved);
    nest_finalCylinder.setKSearch(inputParams.normalEstimationParameters.getKSearch());
    nest_finalCylinder.setInputCloud(cloudWithCylinderAndPlaneRemoved);
    nest_finalCylinder.compute(*normals_cloudWithCylinderAndPlaneRemoved);

    pcl::search::Search<pcl::PointXYZ>::Ptr regionGrowingTree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(inputParams.regionGrowingParameters.getMinClusterSize());
    reg.setMaxClusterSize(inputParams.regionGrowingParameters.getMaxClusterSize());
    reg.setSearchMethod(regionGrowingTree);
    reg.setNumberOfNeighbours(inputParams.regionGrowingParameters.getNumberOfNeighbours());
    reg.setInputCloud(cloudWithCylinderAndPlaneRemoved);
    reg.setInputNormals (normals_cloudWithCylinderAndPlaneRemoved);
    reg.setSmoothnessThreshold((inputParams.regionGrowingParameters.getSmoothnessThreshold() * M_PI) / 180.0); //M_PI/18 = 10 degrees
    reg.setCurvatureThreshold (inputParams.regionGrowingParameters.getCurvatureThreshold());

    std::vector <pcl::PointIndices> clusters;
    LOG.DEBUG("Performing segmentation to clean up remaining pot pieces.");
    reg.extract (clusters);

    logStream << "The number of clusters found by region growing: " << clusters.size ();
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    uint maxSize = 0;
    pcl::PointIndices indicesOfClusterToKeep;
    for (uint32_t i = 0; i < clusters.size(); i++) {
        if (clusters[i].indices.size() > 10) {   // 10 is magic number to keep it from printing out negligibly small clusters.
            logStream << "Cluster " << i << " has " << clusters[i].indices.size() << " points.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        }
        if (clusters[i].indices.size() > maxSize) {
            maxSize = clusters[i].indices.size();
            indicesOfClusterToKeep = clusters[i];
            logStream << "Changing cluster to keep to index " << i;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        }
    }

    pcl::PointIndices::Ptr largestClusterInliers(new pcl::PointIndices(indicesOfClusterToKeep));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFinalSegmentation (new pcl::PointCloud<pcl::PointXYZ>);

    extract.setInputCloud(cloudWithCylinderAndPlaneRemoved);
    extract.setIndices(largestClusterInliers);
    extract.setNegative(false);
    extract.filter(*cloudFinalSegmentation);

    /// Finished Region growing.
    /////////////////////////////////////////////////////
    /////////////////////////////////////////////////////

    //So now we have a segmented plant. Lets put those points into a map that can be searched when cleaning each individual cloud.

    std::map<TupleTriplet, int> map_finalSegmentationPoints;
    for (uint32_t i = 0; i < cloudFinalSegmentation->points.size(); i++) {
        pcl::PointXYZ currentPoint = cloudFinalSegmentation->points[i];
        TupleTriplet tuplePoint = convertPclPointXYZtoTupleTriplet(currentPoint);
        map_finalSegmentationPoints.insert(std::pair<TupleTriplet, int>( tuplePoint, 1));
    }

    // And finally, we can iterate through each individual cloud, keep points that are in the final segmentation, and output for final registration.
    for (int i = 1; i < argc; i++) {
        // Point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr individualCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr individualCloudTransformed (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 cloudBlob;
        logStream << "Loading point cloud from file:\n" << argv[i] << "\n\tfor its final cleaning.";
        LOG.DEBUG(logStream.str()); logStream.str("");
        pcl::io::loadPCDFile(argv[i], cloudBlob);
        pcl::fromPCLPointCloud2(cloudBlob, *individualCloud);
        assert(individualCloud->size() > 5 && "Cloud has too few points. Aborting."); //5 is an arbitrary value to make sure the cloud has some points.

        //This transform matrix comes from waaay up in the function.
        pcl::transformPointCloud(*individualCloud, *individualCloudTransformed, Transform);
        if (axisNeedsToBeSwapped == true) {
            for (uint32_t i = 0; i < individualCloudTransformed->points.size(); i++) {
                individualCloudTransformed->points[i].z = individualCloudTransformed->points[i].z * (-1.0);
            }
        }
        for (uint32_t j = 0; j < individualCloudTransformed->points.size(); j++) {
            pcl::PointXYZ currentPoint = individualCloudTransformed->points[j];
            TupleTriplet tuplePoint = convertPclPointXYZtoTupleTriplet(currentPoint);
            if (map_finalSegmentationPoints.find(tuplePoint) != map_finalSegmentationPoints.end()) {
                outputCloud->push_back(currentPoint);
            }
        }

        std::stringstream ss;
        ss << i << "_ICP_potRemoved.pcd";
        std::string filePath = "sample_globalICP/" + ss.str();
        logStream << "Writing " << outputCloud->size() << " points to file " << filePath;
        LOG.DEBUG(logStream.str()); logStream.str("");
        pcl::io::savePCDFileBinary (filePath, *outputCloud);
    }

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr regionGrowingColoredCloud = reg.getColoredCloud();
        visu->addPointCloud(regionGrowingColoredCloud, "Transformed", mesh_vp_1);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the region growing results. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }


    return(0);
}


/** This should probably get refactored into the supervoxel code?
 *
*/
int returnLabelOfSupervoxelWithMinimumStemPoint(PlantSegmentationDataContainer inputSegmentationData) {
    std::ostringstream logStream;
    //Find minimum stem coordinate.
    LOG.DEBUG("Seeking minimum stem coordinate.");
    ColorMap colorMap;
    TupleTriplet minimumPoint(10000.0, 10000.0, 10000.0);
    std::map <TupleTriplet, TupleTriplet>::iterator mapItr;
    for (mapItr = inputSegmentationData._map_segmentedPoints.begin(); mapItr != inputSegmentationData._map_segmentedPoints.end(); mapItr++) {
        if (mapItr->second == colorMap._stem_color) {
            if (std::get<2>(mapItr->first) < std::get<2>(minimumPoint)){
                minimumPoint = mapItr->first;
            }
        }
    }
    LOG.DEBUG("Found minimum point: ");
    printTupleTriplet(minimumPoint);

    LOG.DEBUG("Finding corresponding supervoxel.");
    int label = inputSegmentationData._supervoxelData._map_pointsToLabel[minimumPoint];
    logStream << "Corresponding supervoxel label is " << label;
    LOG.DEBUG(logStream.str()); logStream.str("");

    return label;
}

/** This should probably get refactored into the supervoxel code?
 *
*/
int returnLabelOfSupervoxelWithMaximumStemPoint(PlantSegmentationDataContainer inputSegmentationData) {
    //Find minimum stem coordinate.
    std::cout << "Seeking maximum stem coordinate." << std::endl;
    ColorMap colorMap;
    TupleTriplet maximumPoint(-10000.0, -10000.0, -10000.0);
    std::map <TupleTriplet, TupleTriplet>::iterator mapItr;
    for (mapItr = inputSegmentationData._map_segmentedPoints.begin(); mapItr != inputSegmentationData._map_segmentedPoints.end(); mapItr++) {
        if (mapItr->second == colorMap._stem_color) {
            if (std::get<2>(mapItr->first) > std::get<2>(maximumPoint)){
                maximumPoint = mapItr->first;
            }
        }
    }
    std::cout << "Found maximum point: ";
    printTupleTriplet(maximumPoint);
    std::cout << std::endl;

    std::cout << "Finding corresponding supervoxel. ";
    int label = inputSegmentationData._supervoxelData._map_pointsToLabel[maximumPoint];
    std::cout << "Corresponding supervoxel label is " << label << std::endl;

    return label;
}
