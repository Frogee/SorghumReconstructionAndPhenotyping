#include <iostream>
#include <assert.h>
#include <stdlib.h>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include "pointCloudFromDepthImage.h"
#include "inputParams.h"
#include "loggingHelper.h"

// The KINECT_SCALE_FACTOR is a value that we empirically determined to represent
// the distance between two pixels in mm at 500 units depth.
#define KINECT_SCALE_FACTOR 1.4089 //1.33
#define KINECT_MINIMUM_DISTANCE 500.0

// Some typedefs to make coloring point clouds in the pcl visualizer more convenient.
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerXYZ;

/** Currently, this only handles depth images. In the future we should be able to stitch RGB and depth images together
  *
  * Relevant sets of input parameters to convert depth images to point cloud:
  * PassThroughFilterParameters to remove non-interesting parts of the image that could harm downstream processing.
  * StatisticalOutlierRemovalParameters to do a coarse cleanup of the point cloud.
  * DebuggingParameters to effect the verbosity of the output.
  */
int convertDepthImagesToPointCloud(int argc, char** argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Initializing conversion of input depth images to point cloud.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo filter points on the x, y, and z dimensions (e.g. to filter out part of the image):");
    inputParams.passThroughFilterParameters.printParameters();
    LOG.DEBUG("\tTo remove statistical outliers from each individual image:");
    inputParams.statisticalOutlierRemovalParameters.printParameters();
    LOG.DEBUG("\tDebugging parameters:");
    inputParams.debuggingParameters.printParameters();

    VectorMats v_depthImages;
    loadDataFromImage(argc, argv, v_depthImages, inputParams);
    logStream << "Loaded " << v_depthImages.size() << " images.";
    LOG.DEBUG(logStream.str()); logStream.str("");
    writePointClouds(&v_depthImages, inputParams);
    return(0);
}

/** Load .png images from a list of files names contained in argv, and load them to a vector of OpenCV Mat
  *
  */
void loadDataFromImage(int argc, char **argv, VectorMats &v_mats, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Loading data from .png image files.");
    for (int i = 1; i < argc; i++) {
        std::string fileName = std::string(argv[i]);
        logStream << "Loading from file: " << fileName;
        LOG.DEBUG(logStream.str()); logStream.str("");
        cv::Mat depthImage;
        depthImage = cv::imread(fileName, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR );
        assert(depthImage.data && "Unable to load data from the input depth image."); //abort if data not loaded
        logStream << "Size and type of loaded matrix: " << depthImage.size() << "\t" << depthImage.type();
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
        v_mats.push_back(depthImage);
    }
}

/** Convert a vector of OpenCV Mat images to point clouds, and write them to file
  *
  */
void writePointClouds (VectorMats *v_mats, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Converting images to point clouds.");
    for (uint32_t i = 0; i < v_mats->size(); i++) {
        logStream << "Converting image " << i;
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
        cv::Mat currentImage = (*v_mats)[i];
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        cv::Size depthImageSize = currentImage.size();
        int rows = depthImageSize.height;
        int cols = depthImageSize.width;
        int colCounter = 0;
        int rowCounter = 0;
        for (int i = 0; i < (rows * cols); ++i) {
            pcl::PointXYZ basic_point;
            basic_point.x = colCounter;
            basic_point.y = rows - rowCounter;
            basic_point.z = currentImage.at<ushort>(rowCounter, colCounter);
            ptr_cloud->points.push_back(basic_point);
            if (colCounter == cols - 1) {
                colCounter = 0;
                rowCounter++;
            }
            else {
                colCounter++;
            }
        }
        logStream << "Converted image to " << ptr_cloud->size() << " points.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*ptr_cloud,*ptr_cloud, indices);

        ptr_cloud->width = ptr_cloud->size();
        ptr_cloud->height = 1;

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            std::stringstream ss2;
            ss2 << i+1 << "_unprocessed.pcd";
            std::string filePathUnprocessed = "sample_clouds_unprocessed/" + ss2.str();
            logStream << "\tWriting the original " << ptr_cloud->size() << " points to file " << filePathUnprocessed;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            pcl::io::savePCDFileASCII(filePathUnprocessed, *ptr_cloud);

            std::stringstream ss3;
            ss3 << i+1 << "_unprocessed.ply";
            std::string filePathUnprocessedPly = "sample_clouds_unprocessed/" + ss3.str();
            logStream << "\tWriting the original " << ptr_cloud->size() << " points to file " << filePathUnprocessedPly << std::endl;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            pcl::io::savePLYFile(filePathUnprocessedPly, *ptr_cloud, false);
        }

        //Scale the Y and X axes to the Z axis based on depth from camera.
        // Perhaps we can eventually use something like this to make RGB-D clouds?
        // https://threeconstants.wordpress.com/2014/11/09/kinect-v2-depth-camera-calibration/
        // http://wiki.ros.org/kinect_calibration/technical
        for(pcl::PointCloud<pcl::PointXYZ>::iterator it = ptr_cloud->begin(); it != ptr_cloud->end(); it++){
            float imageCenterY = (float)(rows) / 2.0;
            float imageCenterX = (float)(cols) / 2.0;
            float distance = (float)(it->z);
            float y = (float)(it->y);
            float x = (float)(it->x);
            float baselineDistance = KINECT_MINIMUM_DISTANCE;
            float baselineScaling = KINECT_SCALE_FACTOR; //~1.33mm per pixel
            float scaledY, scaledX;
            scaledY = ((y - imageCenterY) * (distance / baselineDistance) * baselineScaling);
            scaledX = ((x - imageCenterX) * (distance / baselineDistance) * baselineScaling);
            it->x = round(scaledX);
            it->y = round(scaledY);
            //std::cout << it->x << ", " << it->y << ", " << it->z << std::endl;
        }

        LOG.DEBUG("Filtering using pass through.", inputParams.debuggingParameters.getDebuggingLevel(), 1);
        //PassThrough filter application
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThrough_Intermediate (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThrough_nonNormalized (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
        passThroughFilter.setInputCloud(ptr_cloud);
        passThroughFilter.setFilterFieldName("z");
        passThroughFilter.setFilterLimits(inputParams.passThroughFilterParameters.getZmin(), inputParams.passThroughFilterParameters.getZmax()); //500 and 4500 are the min and max "good values" from the kinect)
        passThroughFilter.filter(*ptr_cloud_filteredPassThrough_Intermediate);
        passThroughFilter.setInputCloud(ptr_cloud_filteredPassThrough_Intermediate);
        passThroughFilter.setFilterFieldName("y");
        passThroughFilter.setFilterLimits(inputParams.passThroughFilterParameters.getYmin(), inputParams.passThroughFilterParameters.getYmax()); //424; I think these may be "upside down"
        passThroughFilter.filter(*ptr_cloud_filteredPassThrough_Intermediate);
        passThroughFilter.setInputCloud(ptr_cloud_filteredPassThrough_Intermediate);
        passThroughFilter.setFilterFieldName("x");
        passThroughFilter.setFilterLimits(inputParams.passThroughFilterParameters.getXmin(), inputParams.passThroughFilterParameters.getXmax()); //512
        passThroughFilter.filter(*ptr_cloud_filteredPassThrough);
        passThroughFilter.filter(*ptr_cloud_filteredPassThrough_nonNormalized);

        ptr_cloud_filteredPassThrough_nonNormalized->width = ptr_cloud_filteredPassThrough_nonNormalized->size();
        ptr_cloud_filteredPassThrough_nonNormalized->height = 1;

        LOG.DEBUG("\tRemoving statistical outliers.", inputParams.debuggingParameters.getDebuggingLevel(), 1);
        //Statistical removal of outliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredStatistical (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (ptr_cloud_filteredPassThrough);
        sor.setMeanK (inputParams.statisticalOutlierRemovalParameters.getMeanK());
        sor.setStddevMulThresh (inputParams.statisticalOutlierRemovalParameters.getStdDevMulThresh());
        sor.filter (*ptr_cloud_filteredStatistical);

        pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZ>);
        finalCloud = ptr_cloud_filteredStatistical;

        finalCloud->width = finalCloud->size();
        finalCloud->height = 1;

        std::stringstream ss;
        ss << i+1 << ".pcd";
        std::string filePath = "sample_clouds/" + ss.str();
        logStream << "\tWriting " << finalCloud->size() << " points to file " << filePath;
        LOG.DEBUG(logStream.str()); logStream.str("");
        //pcl::io::savePCDFileASCII(filePath, *finalCloud);  //To save as ASCII
        pcl::io::savePCDFile(filePath, *finalCloud, true);  //To save as binary

        std::stringstream ssPLY;
        ssPLY << i+1 << ".ply";
        std::string filePathPLY = "sample_cloud_PLYs/" + ssPLY.str();
        logStream << "\tWriting " << finalCloud->size() << " points to file " << filePathPLY;
        LOG.DEBUG(logStream.str()); logStream.str("");
        pcl::io::savePLYFile(filePathPLY, *finalCloud, false);

    }
}



/** This function was originally used to filter clouds when they were initially read in,
  * but the filtering step has since moved to other places. This should be refactored to a more
  * general filtratrion module.
  */
int removeStatisticalOutliers(int argc, char** argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Initializing post-registration refinement.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo downsample the point cloud so that it has a more uniform density:");
    inputParams.voxelGridFilterParameters.printParameters();
    LOG.DEBUG("\tTo remove statistical outliers from the combined cloud:");
    inputParams.statisticalOutlierRemovalParameters.printParameters();
    LOG.DEBUG("\tTo clean up the registered mesh via region growing (e.g. to remove any additional pot points):");
    inputParams.regionGrowingParameters.printParameters();
    LOG.DEBUG("\tTo smooth the cloud via Moving Least Squares (MLS):");
    inputParams.movingLeastSquaresParameters.printParameters();
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

    logStream << "Loading point cloud from file:\t" << argv[1];
    LOG.DEBUG(logStream.str()); logStream.str("");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloud_blob;
    pcl::io::loadPCDFile (argv[1], cloud_blob);
    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
    assert(cloud->size() > 5); //5 is an arbitrary value to make sure the cloud has some points.

    LOG.DEBUG("Downsampling with voxel grid.");
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    float leaf = inputParams.voxelGridFilterParameters.getLeafSize();
    grid.setLeafSize(leaf, leaf, leaf);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    grid.setInputCloud(cloud);
    grid.filter (*voxelFiltered);

    LOG.DEBUG("Removing statistical outliers.");
    //Statistical removal of outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredStatistical (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (voxelFiltered);
    sor.setMeanK (inputParams.statisticalOutlierRemovalParameters.getMeanK());
    sor.setStddevMulThresh (inputParams.statisticalOutlierRemovalParameters.getStdDevMulThresh());
    sor.filter (*ptr_cloud_filteredStatistical);

    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (inputParams.movingLeastSquaresParameters.getUpsampleFlag() == 1) {
        LOG.DEBUG("Starting moving least squares with upsampling");
        pcl::search::KdTree<pcl::PointXYZ>::Ptr treeMLS (new pcl::search::KdTree<pcl::PointXYZ> ());
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud (ptr_cloud_filteredStatistical);
        mls.setComputeNormals(true);
        mls.setSearchRadius (inputParams.movingLeastSquaresParameters.getSearchRadius());
        mls.setPolynomialFit (true);
        mls.setSearchMethod(treeMLS);
        mls.setPolynomialOrder (inputParams.movingLeastSquaresParameters.getPolynomialOrder());

        //mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::VOXEL_GRID_DILATION);
        //mls.setDilationVoxelSize(5.0);
        //mls.setDilationIterations(1);

        //mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::RANDOM_UNIFORM_DENSITY);
        //mls.setPointDensity(50);

        mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
        mls.setUpsamplingRadius (inputParams.movingLeastSquaresParameters.getUpsamplingRadius());
        mls.setUpsamplingStepSize (inputParams.movingLeastSquaresParameters.getUpsamplingStepSize());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
        mls.process (*cloud_smoothed);
        finalCloud = cloud_smoothed;
    }
    else {
        LOG.DEBUG("Starting moving least squares, not upsampling");
        pcl::search::KdTree<pcl::PointXYZ>::Ptr treeMLS (new pcl::search::KdTree<pcl::PointXYZ> ());
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud (ptr_cloud_filteredStatistical);
        mls.setComputeNormals(true);
        mls.setSearchRadius (inputParams.movingLeastSquaresParameters.getSearchRadius());
        mls.setPolynomialFit (true);
        mls.setSearchMethod(treeMLS);
        mls.setPolynomialOrder (inputParams.movingLeastSquaresParameters.getPolynomialOrder());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
        mls.process (*cloud_smoothed);
        finalCloud = cloud_smoothed;
        finalCloud = ptr_cloud_filteredStatistical;
    }


   /////////////////////////////////////////////////////
    /////////////////////////////////////////////////////
    /// Region growing. This should help clean up portions that are disconnected from the plant.
    //And finally, let's clean it up with region growing.

    pcl::PointCloud<pcl::Normal>::Ptr normals_finalCloud (new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeFinalCloud (new pcl::search::KdTree<pcl::PointXYZ> ());
    LOG.DEBUG("Estimating scene normals using K search.");
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest_finalCloud;
    nest_finalCloud.setSearchMethod (treeFinalCloud);
    nest_finalCloud.setKSearch(inputParams.normalEstimationParameters.getKSearch());
    nest_finalCloud.setInputCloud(finalCloud);
    nest_finalCloud.compute(*normals_finalCloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr regionGrowingTree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(inputParams.regionGrowingParameters.getMinClusterSize());
    reg.setMaxClusterSize(inputParams.regionGrowingParameters.getMaxClusterSize());
    reg.setSearchMethod(regionGrowingTree);
    reg.setNumberOfNeighbours(inputParams.regionGrowingParameters.getNumberOfNeighbours());
    reg.setInputCloud(finalCloud);
    reg.setInputNormals(normals_finalCloud);
    reg.setSmoothnessThreshold((inputParams.regionGrowingParameters.getSmoothnessThreshold() * M_PI) / 180.0); //M_PI/18 = 10 degrees
    reg.setCurvatureThreshold (inputParams.regionGrowingParameters.getCurvatureThreshold());

    std::vector <pcl::PointIndices> clusters;
    LOG.DEBUG("Performing region growing segmentation to identify the largest cluster.");
    reg.extract(clusters);

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

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(finalCloud);
    extract.setIndices(largestClusterInliers);
    extract.setNegative(false);
    extract.filter(*cloudFinalSegmentation);

    /// Finished Region growing.
    /////////////////////////////////////////////////////
    /////////////////////////////////////////////////////

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        pcl::PointCloud <pcl::PointXYZRGB>::Ptr regionGrowingColoredCloud = reg.getColoredCloud();
        visu->addPointCloud (cloud, ColorHandlerXYZ(cloud, 0.0, 255.0, 0.0), "original_cloud", mesh_vp_1);
        visu->addPointCloud(regionGrowingColoredCloud, "RegionGrowing", mesh_vp_2);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the region growing results. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    cloudFinalSegmentation->width = cloudFinalSegmentation->size();
    cloudFinalSegmentation->height = 1;

    logStream << "Writing " << cloudFinalSegmentation->size() << " points to files " << "sample_registeredClouds/combinedCloudFiltered.pcd and sample_registeredClouds/combinedCloudFiltered.ply";
    LOG.DEBUG(logStream.str()); logStream.str("");
    pcl::io::savePCDFile("sample_registeredClouds/combinedCloudFiltered.pcd", *cloudFinalSegmentation, true);
    pcl::io::savePLYFile("sample_registeredClouds/combinedCloudFiltered.ply", *cloudFinalSegmentation, false);

  return(0);
}
