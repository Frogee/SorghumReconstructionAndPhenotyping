#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "inputParams.h"
#include "loggingHelper.h"

bool global_visualizerInitialized = false; //Removing clouds from the visualizer causes a seg fault for unknown reasons.
                                                // Thus, we update clouds after the initial cloud has been added, and we
                                                //   use this variable to track whether or not the first cloud has been added.

/// Align a rigid object to a scene with clutter and occlusions. Most of this code
///   has its origins from here: http://pointclouds.org/documentation/tutorials/alignment_prerejective.php#alignment-prerejective
int registerPointCloudsRANSACPrerejective(int argc, char **argv, InputParameters inputParams) {

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();
    pcl::visualization::PCLVisualizer visu("Prerejective RANSAC");

    assert(argc >= 3);     //At least two point clouds should have been passed along with the option choice.

    //For each pair of clouds...
    for (int i = 2; i < argc; i++) {
        // Point clouds

        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObject (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObjectTransformed (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointNormal>::Ptr objectAligned (new pcl::PointCloud<pcl::PointNormal>);

        pcl::PointCloud<pcl::PointXYZ>::Ptr originalScene (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointCloud<pcl::Normal>::Ptr objectNormals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr sceneNormals (new pcl::PointCloud<pcl::Normal>);

        pcl::PointCloud<pcl::PointNormal>::Ptr objectWithNormals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr sceneWithNormals (new pcl::PointCloud<pcl::PointNormal>);


        pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);

        pcl::PCLPointCloud2 cloudBlob_originalObject, cloudBlob_originalScene;

        std::cout << "Loading point clouds from files:\n" <<
            "\t" << argv[i-1] << " and " << argv[i] << std::endl;
        pcl::io::loadPCDFile(argv[i-1], cloudBlob_originalScene);
        pcl::fromPCLPointCloud2(cloudBlob_originalScene, *originalScene); //scene gets modified, original scene does not.
        pcl::fromPCLPointCloud2(cloudBlob_originalScene, *scene);
        assert(scene->size() > 5);  //5 is an arbitrary value to make sure the cloud has some points.

        pcl::io::loadPCDFile(argv[i], cloudBlob_originalObject);
        pcl::fromPCLPointCloud2(cloudBlob_originalObject, *originalObject);  //object gets modified, original object does not.
        pcl::fromPCLPointCloud2(cloudBlob_originalObject, *object);
        assert(object->size() > 5); //5 is an arbitrary value to make sure the cloud has some points.

        if (i == 2) {
            PCL_INFO ("Writing initial cloud (%d points) to file.\n", originalScene->size());
            std::stringstream ss2;
            ss2 << i-1 << "_RANSAC.pcd";
            std::string filePath = "sample_RANSACclouds/" + ss2.str();
            pcl::io::savePCDFileBinary (filePath, *originalScene);
        }


        pcl::PointCloud<pcl::PointXYZ>::Ptr objectVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sceneVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::console::print_highlight ("Downsampling...\n");
        pcl::VoxelGrid<pcl::PointXYZ> gridObject;
        pcl::VoxelGrid<pcl::PointXYZ> gridScene;
        const float leaf = inputParams.voxelGridFilterParameters.getLeafSize();
        gridObject.setLeafSize (leaf, leaf, leaf);
        gridScene.setLeafSize(leaf, leaf, leaf);

        gridObject.setInputCloud (object);
        gridObject.filter (*objectVoxel);
        gridScene.setInputCloud (scene);
        gridScene.filter (*sceneVoxel);
        scene = sceneVoxel;
        object = objectVoxel;

        /// TODO: Here we also want to add a pass through filter to grossly cut off the pot for scene and object.
        /// For small samples, the pot can ruin registration. We should have a filtration header and .cpp for pass through and stat outlier removal.
        printf("\tFiltering using pass through.\n");
        //PassThrough filter application
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThroughScene (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThroughObject (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> passThroughFilterScene;
        pcl::PassThrough<pcl::PointXYZ> passThroughFilterObject;

        passThroughFilterScene.setInputCloud(sceneVoxel);
        passThroughFilterScene.setFilterFieldName("z");
        passThroughFilterScene.setFilterLimits(inputParams.passThroughFilterParameters.getZmin(), inputParams.passThroughFilterParameters.getZmax()); //500 and 4500 are the min and max "good values" from the kinect)
        passThroughFilterScene.filter(*ptr_cloud_filteredPassThroughScene);
        passThroughFilterScene.setInputCloud(ptr_cloud_filteredPassThroughScene);
        passThroughFilterScene.setFilterFieldName("y");
        passThroughFilterScene.setFilterLimits(inputParams.passThroughFilterParameters.getYmin(), inputParams.passThroughFilterParameters.getYmax()); //424; I think these may be "upside down"
        passThroughFilterScene.filter(*ptr_cloud_filteredPassThroughScene);
        passThroughFilterScene.setInputCloud(ptr_cloud_filteredPassThroughScene);
        passThroughFilterScene.setFilterFieldName("x");
        passThroughFilterScene.setFilterLimits(inputParams.passThroughFilterParameters.getXmin(), inputParams.passThroughFilterParameters.getXmax()); //512
        passThroughFilterScene.filter(*ptr_cloud_filteredPassThroughScene);

        passThroughFilterObject.setInputCloud(objectVoxel);
        passThroughFilterObject.setFilterFieldName("z");
        passThroughFilterObject.setFilterLimits(inputParams.passThroughFilterParameters.getZmin(), inputParams.passThroughFilterParameters.getZmax()); //500 and 4500 are the min and max "good values" from the kinect)
        passThroughFilterObject.filter(*ptr_cloud_filteredPassThroughObject);
        passThroughFilterObject.setInputCloud(ptr_cloud_filteredPassThroughObject);
        passThroughFilterObject.setFilterFieldName("y");
        passThroughFilterObject.setFilterLimits(inputParams.passThroughFilterParameters.getYmin(), inputParams.passThroughFilterParameters.getYmax()); //424; I think these may be "upside down"
        passThroughFilterObject.filter(*ptr_cloud_filteredPassThroughObject);
        passThroughFilterObject.setInputCloud(ptr_cloud_filteredPassThroughObject);
        passThroughFilterObject.setFilterFieldName("x");
        passThroughFilterObject.setFilterLimits(inputParams.passThroughFilterParameters.getXmin(), inputParams.passThroughFilterParameters.getXmax()); //512
        passThroughFilterObject.filter(*ptr_cloud_filteredPassThroughObject);

        scene = ptr_cloud_filteredPassThroughScene;
        object = ptr_cloud_filteredPassThroughObject;

        /// End pass through filter

        std::cout << "\tSize of filtered scene cloud: " << scene->size() << std::endl;
        std::cout << "\tSize of filtered object cloud: " << object->size() << std::endl;

        // Estimate normals for scene
        std::cout << "\tEstimating scene normals..." << std::endl;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZ> ());
        pcl::console::print_highlight ("Estimating scene normals using K search...\n");
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;

        nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());

        Eigen::Vector4f centroid;
        //compute3DCentroid (*scene, centroid);
        //nest.setViewPoint (centroid[0], centroid[1], centroid[2]);
        nest.setInputCloud (scene);
        nest.compute (*sceneNormals);
        concatenateFields (*scene, *sceneNormals, *sceneWithNormals);

        //compute3DCentroid (*object, centroid);
        //nest.setViewPoint (centroid[0], centroid[1], centroid[2]);
        nest.setInputCloud(object);
        nest.compute(*objectNormals);
        concatenateFields (*object, *objectNormals, *objectWithNormals);

        /*std::cout << "\tDownsampling..." << std::endl;
        float decimateProportion = inputParams.randomSampleParameters.getSampleProportion();
        pcl::RandomSample<pcl::PointNormal> random_sampler;
        int num_output_points = (int) (decimateProportion*objectWithNormals->points.size());
        random_sampler.setSample(num_output_points);
        random_sampler.setInputCloud(objectWithNormals);
        random_sampler.filter(*objectWithNormals);
        num_output_points = (int) (decimateProportion*sceneWithNormals->points.size());
        random_sampler.setSample(num_output_points);
        random_sampler.setInputCloud(sceneWithNormals);
        random_sampler.filter(*sceneWithNormals);*/

        std::cout << "\tSize of filtered scene cloud: " << sceneWithNormals->size() << std::endl;
        std::cout << "\tSize of filtered object cloud: " << objectWithNormals->size() << std::endl;

        if (global_visualizerInitialized == false) {
            visu.addPointCloud (scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene, 0.0, 255.0, 0.0), "scene");
            visu.addPointCloud (object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(object, 0.0, 0.0, 255.0), "object_aligned");
            visu.addPointCloudNormals<pcl::PointNormal>(objectWithNormals, 1, 20, "normals", 0);
            global_visualizerInitialized = true;
            visu.spinOnce();
        }
        else {
            visu.updatePointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene, 0.0, 255.0, 0.0), "scene");
            visu.updatePointCloud(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(object, 0.0, 0.0, 255.0), "object_aligned");
            visu.spinOnce();
        }

        // Estimate features
        std::cout << "\tEstimating features..." << std::endl;
        pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
        fest.setRadiusSearch(inputParams.featureEstimationParameters.getRadiusSearch());
        fest.setInputCloud (objectWithNormals);
        fest.setInputNormals (objectWithNormals);
        fest.compute (*object_features);
        fest.setInputCloud (sceneWithNormals);
        fest.setInputNormals (sceneWithNormals);
        fest.compute (*scene_features);

        // Perform alignment
        std::cout << "\tStarting alignment..." << std::endl;
        pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
        align.setInputSource(objectWithNormals);
        align.setSourceFeatures(object_features);
        align.setInputTarget(sceneWithNormals);
        align.setTargetFeatures(scene_features);
        align.setMaximumIterations(inputParams.sampleConsensusPrerejectiveParameters.getMaximumIterations()); // Number of RANSAC iterations
        align.setNumberOfSamples(inputParams.sampleConsensusPrerejectiveParameters.getNumberOfSamples()); // Number of points to sample for generating/prerejecting a pose
        align.setCorrespondenceRandomness(inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness()); // Number of nearest features to use
        align.setSimilarityThreshold(inputParams.sampleConsensusPrerejectiveParameters.getSimilarityThreshold()); // Polygonal edge length similarity threshold
        align.setMaxCorrespondenceDistance(inputParams.sampleConsensusPrerejectiveParameters.getMaxCorrespondenceDistance()); // Inlier threshold
        align.setInlierFraction(inputParams.sampleConsensusPrerejectiveParameters.getInlierFraction()); // Required inlier fraction for accepting a pose hypothesis
        {
            REALIGNIFFAILED: pcl::ScopeTime t("Alignment"); //A goto statement if the alignment failed since it's typically a case of not enough iterations. The raptors are coming.
            align.align (*objectAligned);   //If an alignment is not possible, it'll sit in an infinite loop.
            std::cout << "\t";
        }

        if (align.hasConverged ()) {
            std::cout << "\tFitness Score:\t" << align.getFitnessScore() << std::endl;

            PairTransform = align.getFinalTransformation ();
            GlobalTransform = GlobalTransform * PairTransform;

            pcl::transformPointCloud(*originalObject, *originalObjectTransformed, GlobalTransform);

            // Show alignment
            visu.updatePointCloud(originalScene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalScene, 0.0, 255.0, 0.0), "scene");
            visu.updatePointCloud(originalObjectTransformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObjectTransformed, 255.0, 0.0, 0.0), "object_aligned");
            visu.spinOnce();

            std::stringstream ss2;
            ss2 << i << "_RANSAC.pcd";
            std::string filePath = "sample_RANSACclouds/" + ss2.str();
            std::cout << "\tWriting " << originalObjectTransformed->size() << " points to file " << filePath << std::endl;
            pcl::io::savePCDFileBinary (filePath, *originalObjectTransformed);
        }
        else if (align.getInlierFraction() > 0.20) {
            PCL_INFO("Alignment failed. Relaxing parameters and increasing iterations\n");
            align.setInlierFraction(align.getInlierFraction() * 0.95);  //reduce the amount of inliers by 10%
            align.setMaximumIterations(align.getMaximumIterations() * 1.05); // increase the number of iterations by 10%
            align.setSimilarityThreshold(align.getSimilarityThreshold() * 0.95); // Reduce the polygonal edge similarity by 10%
            align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance() * 1.05); // increase the max correspondence distance by 10%
            std::cout << "New parameters:\n" <<
                "\tInlier fraction:\t" << align.getInlierFraction() << "\n" <<
                "\tMaximum iterations:\t" << align.getMaximumIterations() << "\n" <<
                "\tSimilarity threshold:\t" << align.getSimilarityThreshold() << "\n" <<
                "\tMax Correspondence Distance:\t" << align.getMaxCorrespondenceDistance() << std::endl;
            goto REALIGNIFFAILED;
        }
        else {
            pcl::console::print_error ("Alignment failed completely! Aborting\n");
            return (1);
      }
  }
  return (0);
}

/** This function is meant to take a pair of clouds and return a transform to register them using prerejective RANSAC.
  * The target is kept fixed and the transform that is returned will transform the source to the target.
  * much of the code for this function
  * has origins from here: http://pointclouds.org/documentation/tutorials/alignment_prerejective.php#alignment-prerejective
  */
Eigen::Matrix4f returnPairwiseRegistrationTransformUsingRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                                                    InputParameters inputParams,
                                                    pcl::visualization::PCLVisualizer *visualizer) {

    std::cout << "Entering the pairwise transform using RANSAC." << std::endl;
    std::cout << "The following input parameters will be used during this process:" << std::endl;
    std::cout << "\tRANSAC parameters:" << std::endl;
    inputParams.sampleConsensusPrerejectiveParameters.printParameters();
    std::cout << "Note that RANSAC is performed using the entire point clouds without filtering or voxel-based downsampling." << std::endl;
    std::cout << "\tTo calculate point normals:" << std::endl;
    inputParams.normalEstimationParameters.printParameters();
    std::cout << "\tTo calculate point features:" << std::endl;
    inputParams.featureEstimationParameters.printParameters();
    std::cout << "\tDebugging level:" << std::endl;
    inputParams.debuggingParameters.printParameters();

    //Just renaming variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr objectAligned (new pcl::PointCloud<pcl::PointNormal>);

    scene = target;
    object = source;

    Eigen::Matrix4f pairTransformToReturn = Eigen::Matrix4f::Identity();

    std::cout << "\tSize of filtered scene cloud: " << scene->size() << std::endl;
    std::cout << "\tSize of filtered object cloud: " << object->size() << std::endl;

    // Estimate normals for scene
    pcl::PointCloud<pcl::Normal>::Ptr objectNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr sceneNormals (new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::PointNormal>::Ptr objectWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr sceneWithNormals (new pcl::PointCloud<pcl::PointNormal>);

    std::cout << "\tEstimating scene normals..." << std::endl;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::console::print_highlight ("Estimating scene normals using K search...\n");
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;

    nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());

    Eigen::Vector4f centroid;
    //compute3DCentroid (*scene, centroid);
    //nest.setViewPoint (centroid[0], centroid[1], centroid[2]);
    nest.setInputCloud (scene);
    nest.compute (*sceneNormals);
    concatenateFields (*scene, *sceneNormals, *sceneWithNormals);

    //compute3DCentroid (*object, centroid);
    //nest.setViewPoint (centroid[0], centroid[1], centroid[2]);
    nest.setInputCloud(object);
    nest.compute(*objectNormals);
    concatenateFields (*object, *objectNormals, *objectWithNormals);

/*
    visu->updatePointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene, 0.0, 255.0, 0.0), "scene");
    visu->updatePointCloud(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(object, 0.0, 0.0, 255.0), "object_aligned");
    visu->spinOnce();
*/

    // Estimate features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);

    std::cout << "\tEstimating features..." << std::endl;
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
    fest.setRadiusSearch(inputParams.featureEstimationParameters.getRadiusSearch());
    fest.setInputCloud (objectWithNormals);
    fest.setInputNormals (objectWithNormals);
    fest.compute (*object_features);
    fest.setInputCloud (sceneWithNormals);
    fest.setInputNormals (sceneWithNormals);
    fest.compute (*scene_features);

    // Perform alignment
    std::cout << "\tStarting alignment..." << std::endl;
    pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
    align.setInputSource(objectWithNormals);
    align.setSourceFeatures(object_features);
    align.setInputTarget(sceneWithNormals);
    align.setTargetFeatures(scene_features);
    align.setMaximumIterations(inputParams.sampleConsensusPrerejectiveParameters.getMaximumIterations()); // Number of RANSAC iterations
    align.setNumberOfSamples(inputParams.sampleConsensusPrerejectiveParameters.getNumberOfSamples()); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness(inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness()); // Number of nearest features to use
    align.setSimilarityThreshold(inputParams.sampleConsensusPrerejectiveParameters.getSimilarityThreshold()); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(inputParams.sampleConsensusPrerejectiveParameters.getMaxCorrespondenceDistance()); // Inlier threshold
    align.setInlierFraction(inputParams.sampleConsensusPrerejectiveParameters.getInlierFraction()); // Required inlier fraction for accepting a pose hypothesis
    {
        REALIGNIFFAILED: pcl::ScopeTime t("Alignment"); //A goto statement if the alignment failed since it's typically a case of not enough iterations. The raptors are coming.
        align.align (*objectAligned);   //If an alignment is not possible, it'll sit in an infinite loop.
        std::cout << "\t";
    }

    if (align.hasConverged ()) {
        std::cout << "\tFitness Score:\t" << align.getFitnessScore() << std::endl;

        pairTransformToReturn = align.getFinalTransformation ();

        return pairTransformToReturn;

    }
    else if (align.getInlierFraction() > 0.20) {
        PCL_INFO("Alignment failed. Relaxing parameters and increasing iterations\n");
        align.setInlierFraction(align.getInlierFraction() * 0.95);  //reduce the amount of inliers by 10%
        align.setMaximumIterations(align.getMaximumIterations() * 1.05); // increase the number of iterations by 10%
        align.setSimilarityThreshold(align.getSimilarityThreshold() * 0.95); // Reduce the polygonal edge similarity by 10%
        align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance() * 1.05); // increase the max correspondence distance by 10%
        std::cout << "New parameters:\n" <<
            "\tInlier fraction:\t" << align.getInlierFraction() << "\n" <<
            "\tMaximum iterations:\t" << align.getMaximumIterations() << "\n" <<
            "\tSimilarity threshold:\t" << align.getSimilarityThreshold() << "\n" <<
            "\tMax Correspondence Distance:\t" << align.getMaxCorrespondenceDistance() << std::endl;
        goto REALIGNIFFAILED;
    }
    else {
        pcl::console::print_error ("Alignment failed completely! Aborting\n");
        bool alignmentFailed = true;
        assert(alignmentFailed == false && "RANSAC alignment failed.");
        return Eigen::Matrix4f::Identity();
    }

}

int registerKinectFusionPLYsRANSAC(int argc, char **argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Initializing registration of Kinect Fusion clouds");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo filter points on the x, y, and z dimensions (e.g. to filter out part of the image that might harm ICP registration):");
    inputParams.passThroughFilterParameters.printParameters();
    LOG.DEBUG("\tTo downsample the point clouds and speed up registration:");
    inputParams.voxelGridFilterParameters.printParameters();
    LOG.DEBUG("\tTo define ICP parameters:");
    inputParams.iterativeClosestPointParameters.printParameters();
    LOG.DEBUG("\tTo remove statistical outliers from the individual cloud prior to registration:");
    inputParams.statisticalOutlierRemovalParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();

    LOG.DEBUG("Beginning registration.");

    logStream << "Loading PLY file " << argv[1] << ".";
    LOG.DEBUG(logStream.str()); logStream.str("");

    pcl::PolygonMesh targetMesh;
    pcl::PolygonMesh sourceMesh;
    pcl::io::loadPLYFile(argv[1], targetMesh);
    pcl::io::loadPLYFile(argv[2], sourceMesh);

    pcl::visualization::PCLVisualizer *visu;
    int originalViewport = 1;
    int measurementViewport = 2;
    int featureViewport = 3;
    std::string originalViewportString = "orignalMesh";
    std::string measurementViewportString = "measurementMesh";
    std::string featureViewportString = "featureMesh";
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer (argc, argv, "PlyViewer");
        //For a two viewport viewer
        //visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
        //visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
        //For a three viewport viewer
        visu->createViewPort (0.00, 0.0, 0.33, 1.0, originalViewport);
        visu->createViewPort (0.33, 0.0, 0.66, 1.0, measurementViewport);
        visu->createViewPort (0.66, 0.0, 1.00, 1.0, featureViewport);
        visu->setBackgroundColor(0.5, 0.5, 0.5);
        visu->setSize(1700, 1000);
        visu->addCoordinateSystem(1.0);
        /*
        visu->setCameraPosition(inputParams.cameraParameters.getxCoordLocation(),
                                inputParams.cameraParameters.getyCoordLocation(),
                                inputParams.cameraParameters.getzCoordLocation(),
                                inputParams.cameraParameters.getxViewComponent(),
                                inputParams.cameraParameters.getyViewComponent(),
                                inputParams.cameraParameters.getzViewComponent(),
                                inputParams.cameraParameters.getxUpComponent(),
                                inputParams.cameraParameters.getyUpComponent(),
                                inputParams.cameraParameters.getzUpComponent());*/

        // Still working around a bug that prevents removal of a cloud.
        // To do so, we're adding a mock cloud here with a fixed string that we can update later.
        // Updating doesn't crash, though removal does.
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ tmpPoint(1, 1, 1);
        tempCloud->points.push_back(tmpPoint);
        //visu->addPointCloud(tempCloud, featureViewportString, featureViewport);
    }

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addPolygonMesh(targetMesh, originalViewportString, originalViewport);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the mesh to be measured. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    std::cout << "Entering the pairwise transform using RANSAC." << std::endl;
    std::cout << "The following input parameters will be used during this process:" << std::endl;
    std::cout << "\tRANSAC parameters:" << std::endl;
    inputParams.sampleConsensusPrerejectiveParameters.printParameters();
    std::cout << "Note that RANSAC is performed using the entire point clouds without filtering or voxel-based downsampling." << std::endl;
    std::cout << "\tTo calculate point normals:" << std::endl;
    inputParams.normalEstimationParameters.printParameters();
    std::cout << "\tTo calculate point features:" << std::endl;
    inputParams.featureEstimationParameters.printParameters();
    std::cout << "\tDebugging level:" << std::endl;
    inputParams.debuggingParameters.printParameters();

    //Just renaming variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal>::Ptr objectAligned (new pcl::PointCloud<pcl::PointNormal>);

    pcl::PointCloud<pcl::PointNormal>::Ptr sourceAligned (new pcl::PointCloud<pcl::PointNormal>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr targetPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetPointsColored (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr targetPointsSupervoxeled (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointNormal>::Ptr targetWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr targetNormals (new pcl::PointCloud<pcl::Normal>);

    pcl::fromPCLPointCloud2(targetMesh.cloud, *targetPoints);
    pcl::fromPCLPointCloud2(targetMesh.cloud, *targetPointsColored);
    pcl::fromPCLPointCloud2(targetMesh.cloud, *targetWithNormals);
    pcl::fromPCLPointCloud2(targetMesh.cloud, *targetNormals);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourcePoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcePointsColored (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcePointsSupervoxeled (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr sourceNormals (new pcl::PointCloud<pcl::Normal>);

    pcl::fromPCLPointCloud2(sourceMesh.cloud, *sourcePoints);
    pcl::fromPCLPointCloud2(sourceMesh.cloud, *sourcePointsColored);
    pcl::fromPCLPointCloud2(sourceMesh.cloud, *sourceWithNormals);
    pcl::fromPCLPointCloud2(sourceMesh.cloud, *sourceNormals);


    scene = targetPoints;
    object = sourcePoints;

    Eigen::Matrix4f pairTransformToReturn = Eigen::Matrix4f::Identity();

    std::cout << "\tSize of filtered scene cloud: " << scene->size() << std::endl;
    std::cout << "\tSize of filtered object cloud: " << object->size() << std::endl;

    // Estimate features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeatures (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr sourceFeatures (new pcl::PointCloud<pcl::FPFHSignature33>);

    std::cout << "\tEstimating features..." << std::endl;
    pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
    fest.setNumberOfThreads(4);
    fest.setRadiusSearch(inputParams.featureEstimationParameters.getRadiusSearch());
    fest.setInputCloud (targetWithNormals);
    fest.setInputNormals (targetWithNormals);
    fest.compute (*targetFeatures);
    fest.setInputCloud (sourceWithNormals);
    fest.setInputNormals (sourceWithNormals);
    fest.compute (*sourceFeatures);

    // Perform alignment
    std::cout << "\tStarting alignment..." << std::endl;
    pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
    align.setInputTarget(targetWithNormals);
    align.setTargetFeatures(targetFeatures);
    align.setInputSource(sourceWithNormals);
    align.setSourceFeatures(sourceFeatures);
    align.setMaximumIterations(inputParams.sampleConsensusPrerejectiveParameters.getMaximumIterations()); // Number of RANSAC iterations
    align.setNumberOfSamples(inputParams.sampleConsensusPrerejectiveParameters.getNumberOfSamples()); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness(inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness()); // Number of nearest features to use
    align.setSimilarityThreshold(inputParams.sampleConsensusPrerejectiveParameters.getSimilarityThreshold()); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(inputParams.sampleConsensusPrerejectiveParameters.getMaxCorrespondenceDistance()); // Inlier threshold
    align.setInlierFraction(inputParams.sampleConsensusPrerejectiveParameters.getInlierFraction()); // Required inlier fraction for accepting a pose hypothesis
    {
        REALIGNIFFAILED: pcl::ScopeTime t("Alignment"); //A goto statement if the alignment failed since it's typically a case of not enough iterations. The raptors are coming.
        align.align (*sourceAligned);   //If an alignment is not possible, it'll sit in an infinite loop.
        std::cout << "\t";
    }

    if (align.hasConverged ()) {
        std::cout << "\tFitness Score:\t" << align.getFitnessScore() << std::endl;

        pairTransformToReturn = align.getFinalTransformation ();


    }
    else if (align.getInlierFraction() > 0.20) {
        PCL_INFO("Alignment failed. Relaxing parameters and increasing iterations\n");
        align.setInlierFraction(align.getInlierFraction() * 0.95);  //reduce the amount of inliers by 10%
        align.setMaximumIterations(align.getMaximumIterations() * 1.05); // increase the number of iterations by 10%
        align.setSimilarityThreshold(align.getSimilarityThreshold() * 0.95); // Reduce the polygonal edge similarity by 10%
        align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance() * 1.05); // increase the max correspondence distance by 10%
        std::cout << "New parameters:\n" <<
            "\tInlier fraction:\t" << align.getInlierFraction() << "\n" <<
            "\tMaximum iterations:\t" << align.getMaximumIterations() << "\n" <<
            "\tSimilarity threshold:\t" << align.getSimilarityThreshold() << "\n" <<
            "\tMax Correspondence Distance:\t" << align.getMaxCorrespondenceDistance() << std::endl;
        goto REALIGNIFFAILED;
    }
    else {
        pcl::console::print_error ("Alignment failed completely! Aborting\n");
        bool alignmentFailed = true;
        assert(alignmentFailed == false && "RANSAC alignment failed.");
        pairTransformToReturn = Eigen::Matrix4f::Identity();

    }

    //Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    PairTransform = pairTransformToReturn;

    //GlobalTransform = GlobalTransform * PairTransform;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr sourcePointsColoredTransformed (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::transformPointCloud(*sourcePointsColored, *sourcePointsColoredTransformed, GlobalTransform);
    pcl::transformPointCloud(*sourcePointsColored, *sourcePointsColoredTransformed, PairTransform);

    // Show alignment

    //visu->updatePointCloud(originalScene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalScene, 0.0, 255.0, 0.0), "scene");
    //visu->updatePointCloud(originalObjectTransformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObjectTransformed, 0.0, 0.0, 255.0), "object_aligned");
    //visu->spinOnce();

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addPointCloud(targetPointsColored, "targetOriginal", measurementViewport);
        visu->addPointCloud(sourcePointsColored, "sourceOriginal", measurementViewport);
        visu->addPointCloud(targetPointsColored, featureViewportString, featureViewport);
        visu->addPointCloud(sourcePointsColoredTransformed, "transformed", featureViewport);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the mesh to be measured. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    return (0);
}
