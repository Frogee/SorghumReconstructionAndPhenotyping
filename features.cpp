
#include <iostream>
#include <fstream>
#include <assert.h>
#include <stdlib.h>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/common/distances.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>


#include <pcl/visualization/pcl_visualizer.h>


#include "pointCloudFromDepthImage.h"
#include "inputParams.h"
#include "features.h"
#include "loggingHelper.h"
#include "tupleTriplet.h"


typedef pcl::PointNormal PointNT;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerXYZ;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerNT;

/** Compute point features for use with machine learning. If the point is colored cyan (0, 255, 255),
 *  the point will be labeled as STEM, otherwise will be unlabeled. Features for all points are written
 *  to an output file (an .arf) that can be input to MultiBoost.
 */
int computePointFeatures(int argc, char** argv, InputParameters inputParams) {
    std::ostringstream logStream;
    ColorMap colorMap;
    LOG.DEBUG("Entering the computation of point features. Note that only cyan (0, 255, 255) and gold (255, 215, 0) points are used (labeled as STEM and INFLORESCENCE, respectively).");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo calculate point features:");
    inputParams.featureEstimationParameters.printParameters();
    LOG.DEBUG("\tCurrently, the K search input parameter is used as a second Radius search for feature estimation.");
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();

    pcl::visualization::PCLVisualizer *visu;
    int mesh_vp_1, mesh_vp_2;
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer;
        visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
        visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PCLPointCloud2 cloud_blob;

    logStream << "Loading PLY to polygon mesh from file " << argv[1];
    LOG.DEBUG(logStream.str()); logStream.str("");

    pcl::PolygonMesh::Ptr ptr_triangles (new pcl::PolygonMesh);
    pcl::io::loadPLYFile(argv[1], *ptr_triangles);

    pcl::fromPCLPointCloud2(ptr_triangles->cloud, *cloud);
    pcl::fromPCLPointCloud2(ptr_triangles->cloud, *cloudNormals);

    logStream << "Loaded PLY file has " << cloud->size() << " points.";
    LOG.DEBUG(logStream.str()); logStream.str("");

    assert(cloud->size() > 5 && "Input mesh has too few points. Aborting.");
    assert(cloudNormals->size() > 5 && "Ensure that the input mesh has normals. Aborting.");



    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addPolygonMesh(*ptr_triangles, "FeatureMesh", mesh_vp_1);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            std::cout << "Displaying the mesh for which point features will be calculated. Press q to continue." << std::endl;
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    /// Estimate features. We use two "scales" of features to try and improve identification.
    logStream << "\tEstimating features with radius search of " << inputParams.featureEstimationParameters.getRadiusSearch();
    LOG.DEBUG(logStream.str()); logStream.str("");
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudPFHfeaturesMethodOne (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> festMethodOne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeMethodOne(new pcl::search::KdTree<pcl::PointXYZRGB>);
    festMethodOne.setNumberOfThreads(4);
    festMethodOne.setSearchMethod (treeMethodOne);
    festMethodOne.setRadiusSearch(inputParams.featureEstimationParameters.getRadiusSearch());
    festMethodOne.setInputCloud(cloud);
    festMethodOne.setInputNormals(cloudNormals);
    festMethodOne.compute(*cloudPFHfeaturesMethodOne);

    logStream << "\tEstimating features with radius search of " << inputParams.featureEstimationParameters.getKSearch();
    LOG.DEBUG(logStream.str()); logStream.str("");
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloudPFHfeaturesMethodTwo (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> festMethodTwo;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treeMethodTwo(new pcl::search::KdTree<pcl::PointXYZRGB>);
    festMethodTwo.setNumberOfThreads(4);
    festMethodTwo.setSearchMethod (treeMethodTwo);
    festMethodTwo.setRadiusSearch(inputParams.featureEstimationParameters.getKSearch());
    festMethodTwo.setInputCloud(cloud);
    festMethodTwo.setInputNormals(cloudNormals);
    festMethodTwo.compute(*cloudPFHfeaturesMethodTwo);


    ofstream featuresOutputFile;
    featuresOutputFile.open ("featuresOutputFile.arf");
    featuresOutputFile << "%.arff format features.\n\n";
    featuresOutputFile << "@RELATION pointLabels\n\n";
    featuresOutputFile << "@ATTRIBUTE X NUMERIC\n";
    featuresOutputFile << "@ATTRIBUTE Y NUMERIC\n";
    featuresOutputFile << "@ATTRIBUTE Z NUMERIC\n";
    featuresOutputFile << "@ATTRIBUTE distanceFromMinY NUMERIC\n";
    featuresOutputFile << "@ATTRIBUTE distanceFromMaxY NUMERIC\n";
    featuresOutputFile << "@ATTRIBUTE distanceFromCentroid NUMERIC\n";
    featuresOutputFile << "@ATTRIBUTE normal_X NUMERIC\n";
    featuresOutputFile << "@ATTRIBUTE normal_Y NUMERIC\n";
    featuresOutputFile << "@ATTRIBUTE normal_Z NUMERIC\n";


    for (int i = 0; i < 33; i++) {
            featuresOutputFile << "@ATTRIBUTE FeatureBin_RadiusSearch-" << inputParams.featureEstimationParameters.getRadiusSearch() << "_Bin-" << i + 1 << " NUMERIC\n";
    }


    for (int i = 0; i < 33; i++) {
            featuresOutputFile << "@ATTRIBUTE FeatureBin_RadiusSearch-" << inputParams.featureEstimationParameters.getKSearch() << "_Bin-" << i + 1 << " NUMERIC\n";
    }

    //featuresOutputFile << "@ATTRIBUTE class {STEM,LEAF,TIP}\n\n";
    featuresOutputFile << "@ATTRIBUTE class {STEM,UNLABELED,INFLORESCENCE}\n\n";
    featuresOutputFile << "@DATA\n";

    Eigen::Vector4f centroid;   // x, y, and z are 0, 1, and 2.
    Eigen::Vector4f maxCoords;
    Eigen::Vector4f minCoords;

    compute3DCentroid (*cloud, centroid);
    getMinMax3D(*cloud, minCoords, maxCoords);
    float maxY = maxCoords[1];
    float minY = minCoords[1];
    pcl::PointXYZ centroidPointXYZ(centroid[0], centroid[1], centroid[2]);


    /// Without a doubt, the features calculated here can be (and should be) improved.
    /// Things to add: geodesic distance from bottom of plant?
    /// geodesic distance to center of plant?
    LOG.DEBUG("Outputting features to file.");
    for(uint i = 0; i < cloud->points.size(); i++) {
        pcl::PointXYZRGB currentPoint = cloud->points[i];
        pcl::Normal currentNormal = cloudNormals->points[i];
        pcl::PointXYZ currentPointXYZ(currentPoint.x, currentPoint.y, currentPoint.z);
        float distanceFromMinY = 0;
        float distanceFromMaxY = 0;
        float distanceFromCentroid = 0;
        distanceFromMaxY = maxY - currentPointXYZ.y;
        distanceFromMinY = currentPointXYZ.y - minY;
        distanceFromCentroid = pcl::euclideanDistance(currentPointXYZ, centroidPointXYZ);
        //std::cout << "Max coords: " << maxCoords << " Min coords: " << minCoords << " centroid " << centroidPointXYZ << std::endl;
        //std::cout << "Cur Y " << currentPointXYZ.y << " distanceFromMinY " << distanceFromMinY << " distance from Max Y " << distanceFromMaxY << std::endl;
        //std::cout << "Current point " << currentPoint << " distanceFromCentroid " << distanceFromCentroid << std::endl;

        featuresOutputFile << currentPoint.x << "," << currentPoint.y << "," << currentPoint.z << ",";
        featuresOutputFile << distanceFromMinY << "," << distanceFromMaxY << "," << distanceFromCentroid << ",";
        featuresOutputFile << currentNormal.normal_x << "," << currentNormal.normal_y << "," << currentNormal.normal_z << ",";

        for (int j = 0; j < 33; j++) {
            featuresOutputFile << cloudPFHfeaturesMethodOne->points[i].histogram[j] << ",";
        }
        for (int j = 0; j < 33; j++) {
            featuresOutputFile << cloudPFHfeaturesMethodTwo->points[i].histogram[j] << ",";
        }

        TupleTriplet currentColor(currentPoint.r, currentPoint.g, currentPoint.b);
        if (currentColor == colorMap._stem_color) {
            featuresOutputFile << "STEM\n";
        }
        else if (currentColor == colorMap._inflorescence_color) {
            featuresOutputFile << "INFLORESCENCE\n";
        }
        else {
            featuresOutputFile << "UNLABELED\n";
        }
    }
    featuresOutputFile.close();

    return(0);
}

// Supervoxel clustering from http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>


//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                       PointCloudT &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer* viewer,
                                       int viewport);


int
supervoxelConstruction (int argc, char** argv, InputParameters inputParams)
{
    PCL_INFO ("Supervoxel construction\n");

    // Initialize the viewer to visualize clouds.
    pcl::visualization::PCLVisualizer *visu;
    visu = new pcl::visualization::PCLVisualizer (argc, argv, "supervoxels");
    int mesh_vp_1, mesh_vp_2;
    visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
    visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::PolygonMesh::Ptr ptr_triangles (new pcl::PolygonMesh);
    pcl::io::loadPLYFile(argv[1], *ptr_triangles);
    pcl::fromPCLPointCloud2(ptr_triangles->cloud, *cloud);

/*
    // Read in the XYZ point cloud from file.
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile (argv[1], cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud); */

    // The Supervoxel construction expects color. Just to prevent any funny business, we fill it in with a uniform value for now.
    for (uint i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].r = 0;
        cloud->points[i].g = 255;
        cloud->points[i].b = 0;
        cloud->points[i].a = 255;
    }


    // Assuming the cloud didn't haven normals, we calculate them
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);

    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
    pcl::console::print_highlight ("Estimating scene normals using K search...\n");
    pcl::NormalEstimationOMP<pcl::PointXYZRGBA, pcl::Normal> nest;
    nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());
    nest.setInputCloud(cloud);
    nest.compute(*cloudNormals);

    //Supervoxel construction, most of which was lifted from http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php
    float voxel_resolution = inputParams.supervoxelClusteringParameters.getVoxelResolution();
    float seed_resolution = inputParams.supervoxelClusteringParameters.getSeedResolution();
    float color_importance = inputParams.supervoxelClusteringParameters.getColorImportance();
    float spatial_importance = inputParams.supervoxelClusteringParameters.getSpatialImportance();
    float normal_importance = inputParams.supervoxelClusteringParameters.getNormalImportance();

    // Setting up the workhorse class behind the supervoxel construction.
    pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (voxel_resolution, seed_resolution, false);

    super.setInputCloud(cloud);
    super.setNormalCloud(cloudNormals);
    super.setColorImportance (color_importance);
    super.setSpatialImportance (spatial_importance);
    super.setNormalImportance (normal_importance);

    // Constructing a few data structures that will hold the output of the construction.
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointsInSupervoxel (new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr >::iterator clusterItr;

    pcl::console::print_highlight ("Extracting supervoxels!\n");

    // And here's where things get a little funky.
    // As far as I can tell, extract() runs and all the internals work, but what gets written to the output is a bunch of
    // empty Supervoxels. I think the internals work because all of the super.get...Cloud() functions work fine.
    // Changing the parameters can give non-empty Supervoxels. The following parameters gave non-empty Supervoxels.
    /*
  		<Parameter name="VoxelResolution" description="TODO: Add description" type="float" value="0.08"> 20.0 </Parameter>
		<Parameter name="SeedResolution" description="TODO: Add description" type="float" value="0.10"> 5.0 </Parameter>
		<Parameter name="ColorImportance" description="TODO: Add description" type="float" value="0.20"> </Parameter>
		<Parameter name="SpatialImportance" description="TODO: Add description" type="float" value="0.40"> </Parameter>
		<Parameter name="NormalImportance" description="TODO: Add description" type="float" value="1.0"> </Parameter>
    */
    // Whereas the following did not:
    /*
        <Parameter name="VoxelResolution" description="TODO: Add description" type="float" value="0.09"> 20.0 </Parameter>
		<Parameter name="SeedResolution" description="TODO: Add description" type="float" value="0.10"> 5.0 </Parameter>
		<Parameter name="ColorImportance" description="TODO: Add description" type="float" value="0.20"> </Parameter>
		<Parameter name="SpatialImportance" description="TODO: Add description" type="float" value="0.40"> </Parameter>
		<Parameter name="NormalImportance" description="TODO: Add description" type="float" value="1.0"> </Parameter>
    */
    // Instead of tracking that down, we just reconstruct the std::map that is supposed to be written out using
    // the output of super.get...Cloud().

    // This extracts the supervoxels, but things don't get correctly written. We create a new map that will be correctly populated.
    super.extract(supervoxel_clusters);
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxelClusters_reconstructed;

    // Supervoxels can be refined, though I haven't tested it.
    //std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > refined_supervoxel_clusters;
    //super.refineSupervoxels(3, refined_supervoxel_clusters);

    // Declare and populate the clouds that we can pull the data we need from.
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudWithLabeledPoints (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudWithPointsColorizedByLabel (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudWithCentroidsByLabel (new pcl::PointCloud<pcl::PointXYZL>);
    cloudWithCentroidsByLabel = super.getLabeledVoxelCloud ();
    cloudWithLabeledPoints = super.getLabeledCloud();

    // Based on the number of labels (i.e. number of supervoxels), populate a vector with that many random RGB triplets, and
    // populate the new map with that many labels keyed to that many Supervoxels.
    std::vector< std::vector< uint8_t > > vv_colorVector;
    for (int i = 0; i < super.getLabeledVoxelCloud()->size(); i++ ) {
        std::vector<uint8_t> RGB;
        RGB.push_back(rand()%255);
        RGB.push_back(rand()%255);
        RGB.push_back(rand()%255);
        vv_colorVector.push_back(RGB);
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr emptySupervoxel (new pcl::Supervoxel<pcl::PointXYZRGBA>);
        supervoxelClusters_reconstructed.insert(std::pair<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > (i + 1, emptySupervoxel ));
    }


    // Demonstrates that we have the correct number of labels and empty supervoxels.
    //for (clusterItr = supervoxelClusters_reconstructed.begin(); clusterItr != supervoxelClusters_reconstructed.end(); clusterItr++) {
    //    std::cout << "supervoxelClusters_reconstructed map: " << clusterItr->first << " " << clusterItr->second->centroid_ << std::endl;
    //    pointsInSupervoxel = clusterItr->second->voxels_;
    //    std::cout << "size of voxels_ cloud from supervoxel: " << pointsInSupervoxel->size() << std::endl;
    //}

    // Populate the empty Supervoxels in the map with their respective points. We also populated a regular
    // cloud for downstream visualization.
    // Check if we can do all this with a constructor for a pcl::PointXYZRGBA if this stays in the code.
    for (int i = 0; i < cloudWithLabeledPoints->points.size(); i++) {
        pcl::PointXYZL pointLabel = cloudWithLabeledPoints->points[i];
        pcl::PointXYZRGBA pointRGB;
        uint32_t labelOfSupervoxel = pointLabel.label;
        std::vector<uint8_t> colorVector = vv_colorVector[labelOfSupervoxel - 1]; // labels aren't 0 indexed.
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

    // Now that the Supervoxels are populated, calculate the centroid with their points.
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

        // Show that the map now contains what we intended.
        //std::cout << "supervoxelClusters_reconstructed map after reconstruction: " << clusterItr->first << " " << clusterItr->second->centroid_ << std::endl;
        //pointsInSupervoxel = clusterItr->second->voxels_;
        //std::cout << "size of voxels_ cloud from supervoxel after reconstruction: " << pointsInSupervoxel->size() << std::endl;
    }

    // With the above print statement, shows that the centroid calculated by Supervoxel reconstruction correspond to the centroids we calculated
    // via compute3DCentroid.
    //for (int i = 0; i < cloudWithCentroidsByLabel->points.size(); i++) {
    //   std::cout << "Cloud with centroids by label: " << cloudWithCentroidsByLabel->points[i] << std::endl;
    //}

    visu->addPointCloud (cloudWithPointsColorizedByLabel, "coloredByLabel", mesh_vp_1);

    cloudWithPointsColorizedByLabel->width = cloudWithPointsColorizedByLabel->size();
    cloudWithPointsColorizedByLabel->height = 1;

    std::string filePathUnprocessed = "testSupervoxels.ply";
    std::cout << "\tWriting the original " << cloudWithPointsColorizedByLabel->size() << " points to file " << filePathUnprocessed << std::endl;
    pcl::io::savePLYFileASCII(filePathUnprocessed, *cloudWithPointsColorizedByLabel);

    visu->spin();

    // This can serve as a double check that points got labeled correctly.
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr coloredCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //coloredCloud = super.getColoredCloud();
    //visu->addPointCloud (coloredCloud, "coloredPoints", mesh_vp_1);


    // Now that the data structure is properly populated, we can move on to getting adjacency.
    pcl::console::print_highlight ("Getting supervoxel adjacency\n");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency(supervoxel_adjacency);

    // To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    std::multimap<uint32_t,uint32_t>::iterator label_itr;
    int lineCounter = 0;
    for (label_itr = supervoxel_adjacency.begin(); label_itr != supervoxel_adjacency.end(); ) { //We increment this iterator at the end of the loop

        //First get the label
        uint32_t supervoxel_label = label_itr->first;

        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxelClusters_reconstructed.at(supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
        for ( adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
              adjacent_itr != supervoxel_adjacency.equal_range (supervoxel_label).second; adjacent_itr++) {
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxelClusters_reconstructed.at(adjacent_itr->second);
            adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);

            std::stringstream ssCounter;
            ssCounter << lineCounter;
            //if (lineCounter < 1000) {
            //    std::cout << "Adding line from: " << supervoxel->centroid_ << " to " << neighbor_supervoxel->centroid_ << std::endl;
            //    visu->addLine(supervoxel->centroid_, neighbor_supervoxel->centroid_, ssCounter.str(), mesh_vp_2);
            //}
            lineCounter++;
        }

        //Now we make a name for this polygon
        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        std::stringstream ss2;
        ss2 << "supervoxelcopy_" << supervoxel_label;
        //std::cout << "Supervoxel label: " << supervoxel_label << std::endl;*/

        //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
        //addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), visu, mesh_vp_1);
        addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss2.str (), visu, mesh_vp_2);



        //addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), visu, mesh_vp_1);
        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }

    visu->spin();

    return (0);

}

void
addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer* viewer,
                                  int viewport)
{
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
      vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
      vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

      //Iterate through all adjacent points, and add a center point to adjacent point pair
      PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
      for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
      {
        points->InsertNextPoint (supervoxel_center.data);
        points->InsertNextPoint (adjacent_itr->data);
      }
      // Create a polydata to store everything in
      vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
      // Add the points to the dataset
      polyData->SetPoints (points);
      polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
      for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
        polyLine->GetPointIds ()->SetId (i,i);
      cells->InsertNextCell (polyLine);
      // Add the lines to the dataset
      polyData->SetLines (cells);
      viewer->addModelFromPolyData (polyData, supervoxel_name, viewport);
}
