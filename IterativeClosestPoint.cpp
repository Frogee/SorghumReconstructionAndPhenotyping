#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include "IterativeClosestPoint.h"
#include "inputParams.h"
#include "SampleConsensusPrerejective.h"
#include "loggingHelper.h"

/** This global variable is used because, at least when this code was originally being written,
  * trying to remove point clouds from the PCL visualizer would result in a segmentation fault.
  * However, updating a point cloud in the viewer worked fine. To get around it, we just monitored
  * if a cloud was already added to the visualizer with this variable, and then updated it if it is was.
 */
bool global_visualizerInitializedICP = false;

/** Incrementally registers a group of pointclouds. Each pair is registered, and a global transform is updated to put all clouds
  * into the same reference frame as the first. Much of this code originates from the PCL tutorial
  * at http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php
  *
  * This is useful for a "coarse" registration to get each cloud in roughly the same frame. Jumping straight to a global ICP
  * seems to often provide poor results, whereas using a coarse pairwise registration first has worked well.
  *
  * This will also try a RANSAC approach to registration if the ICP fitness score is too large.
  *
  * Relevant sets of input parameters:
  * PassThroughFilterParameters to crudely remove parts of the image that would ruin ICP registration (e.g., the pot).
  * VoxelGridFilterParameters to downsample the clouds and speed up registration.
  * IterativeClosestPointParameters to define ICP parameters
  * SampleConsensusPRerejectiveParameters in case ICP fails to be underneath a fitness threshold specified in the configuration.
  * DebuggingParameters to affect the verbosity of output and to choose whether or not to display ICP updates in the visualizer.
  */
int registerPointCloudsICP_pairwise(int argc, char **argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Initializing pairwise registration of input .pcd format point clouds.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo filter points on the x, y, and z dimensions (e.g. to filter out part of the image that might harm ICP registration):");
    inputParams.passThroughFilterParameters.printParameters();
    LOG.DEBUG("\tTo downsample the point clouds and speed up registration:");
    inputParams.voxelGridFilterParameters.printParameters();
    LOG.DEBUG("\tTo define ICP parameters:");
    inputParams.iterativeClosestPointParameters.printParameters();
    LOG.DEBUG("\tTo define pre-rejective RANSAC parameters in case ICP fails:");
    inputParams.sampleConsensusPrerejectiveParameters.printParameters();
    LOG.DEBUG("\tTo calculate point normals in case RANSAC is used:");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo calculate point features in case RANSAC is used:");
    inputParams.featureEstimationParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();

    pcl::visualization::PCLVisualizer *visu;
    std::string objectString = "object";
    std::string sceneString = "source";
    int viewport = 0;
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer;
        //For a two viewport viewer
        //visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
        //visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
        //For a three viewport viewer
        //visu->createViewPort (0.00, 0.0, 0.33, 1.0, originalViewport);
        //visu->createViewPort (0.33, 0.0, 0.66, 1.0, measurementViewport);
        //visu->createViewPort (0.66, 0.0, 1.00, 1.0, featureViewport);
        /*visu->setBackgroundColor(0.5, 0.5, 0.5);
        visu->setSize(1700, 1000);
        visu->addCoordinateSystem(50.0);
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
        visu->addPointCloud(tempCloud, objectString, viewport);
        visu->addPointCloud(tempCloud, sceneString, viewport);
    }

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    int numberOfCloudsPlusOne = argc;
    assert(numberOfCloudsPlusOne >= 3 && "At least two point clouds should be passed at the command line.");

    // Register cloud i and cloud i - 1 in a pairwise fashion.
    for (int i = 2; i < argc; i++) {
        // Point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObject (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObjectTransformed (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectAligned (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointCloud<pcl::PointXYZ>::Ptr originalScene (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PCLPointCloud2 cloudBlob_originalObject, cloudBlob_originalScene;

        logStream << "Loading point clouds from files:\n" <<
            "\t" << argv[i-1] << " and " << argv[i];
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

        pcl::io::loadPCDFile(argv[i-1], cloudBlob_originalScene);
        pcl::fromPCLPointCloud2(cloudBlob_originalScene, *originalScene); //scene gets modified, original scene does not.
        pcl::fromPCLPointCloud2(cloudBlob_originalScene, *scene);
        assert(scene->size() > 5 && "Input cloud has too few points. Is it empty?");  //5 is an arbitrary value to make sure the cloud has some points.

        pcl::io::loadPCDFile(argv[i], cloudBlob_originalObject);
        pcl::fromPCLPointCloud2(cloudBlob_originalObject, *originalObject);  //object gets modified, original object does not.
        pcl::fromPCLPointCloud2(cloudBlob_originalObject, *object);
        assert(object->size() > 5 && "Input cloud has too few points. Is it empty?"); //5 is an arbitrary value to make sure the cloud has some points.

        // Everything is transformed to the frame of the first cloud, and the first cloud doesn't need to be transformed.
        if (i == 2) {
            std::stringstream ss2;
            ss2 << i-1 << "_ICP.pcd";
            std::string filePath = "sample_pairwiseICP/" + ss2.str();
            logStream << "Writing initial cloud (" << originalScene->size() << " points) to file " << filePath;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            pcl::io::savePCDFileBinary (filePath, *originalScene);
        }

        /// Here we downsample the plant to speed up registration.
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sceneVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        LOG.DEBUG("Downsampling with voxel grid.", inputParams.debuggingParameters.getDebuggingLevel(), 2);
        pcl::VoxelGrid<pcl::PointXYZ> gridObject;
        pcl::VoxelGrid<pcl::PointXYZ> gridScene;
        const float leaf = inputParams.voxelGridFilterParameters.getLeafSize();
        gridObject.setLeafSize (leaf, leaf, leaf);
        gridScene.setLeafSize(leaf, leaf, leaf);

        gridObject.setInputCloud (object);
        gridObject.filter (*objectVoxel);
        gridScene.setInputCloud (scene);
        gridScene.filter (*sceneVoxel);

        /// Here we crudely cut off the pot to prevent it from causing problems during registration of the plant.
        { //Begin scoping for pass through filter.
        LOG.DEBUG("Filtering using pass through", inputParams.debuggingParameters.getDebuggingLevel(), 2);
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
        } // End scoping for pass through filter

        logStream << "Size of filtered scene cloud: " << scene->size() << std::endl <<
                        "Size of filtered object cloud: " << object->size();
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        /// If debugging level is set to high, visualize each individual transformation.
        /// There is tremendous duplication of code here between the debugging level if statment cases(and all throughout the ICP code). Consider refactoring.
        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(object, 0.0, 255.0, 255.0), objectString);
            visu->spinOnce();

            // Perform alignment
            LOG.DEBUG("Starting alignment.");

            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
            align.setTransformationEpsilon(1e-6); // Magic number for the epsilon
            align.setInputSource(object);
            align.setInputTarget(scene);
            align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

            align.setMaximumIterations (2); //So that we can watch the iterations, we set this two 2, and do for loop over align() calls.
            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
            int stopConditionCounter = 0;
            float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
            float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
            float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
            float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();
            for (int i = 0; i < inputParams.iterativeClosestPointParameters.getMaximumIterations(); ++i) {
                align.align(*objectAligned);
                object = objectAligned;
                align.setInputSource (object);

                //accumulate transformation between each Iteration
                Ti = align.getFinalTransformation () * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
                    if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                        logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        stopConditionCounter = 0;
                    }
                    else {
                        if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= 1.0) {
                            logStream << "\tCurrent distance minus step size is smaller than 1.0. Setting distance to correspondence distance of " << 1.0;
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(1.0);
                            stopConditionCounter = 0;
                        }
                        else {
                            logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            stopConditionCounter = 0;
                        }
                    }
                }

                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
                    stopConditionCounter = stopConditionCounter + 1;
                }

                prev = align.getLastIncrementalTransformation ();

                // visualize current state
                visu->updatePointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene, 0.0, 255.0, 0.0), sceneString);
                visu->updatePointCloud(objectAligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(objectAligned, 0.0, 255.0, 255.0), objectString);
                visu->spinOnce();

                if (stopConditionCounter > 5) {
                    LOG.DEBUG("ICP stop condition met.");
                    break;
                }

            }

            logStream << "Fitness Score:\t" << align.getFitnessScore();
            LOG.DEBUG(logStream.str()); logStream.str("");

            /// Sometimes ICP isn't sufficient. Let's try adding a check based on fitness score, and send the pair to Prerejective RANSAC
            /// to get a better transformation.
            if (align.getFitnessScore() > inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure()) {
                logStream << "\tUnable to identify sufficiently good fit under threshold of " << inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure() <<
                                " using ICP. Attempting to use RANSAC.";
                LOG.DEBUG(logStream.str()); logStream.str("");
                PairTransform = returnPairwiseRegistrationTransformUsingRANSAC(originalScene, originalObject, inputParams, visu);
            }
            else {
                PairTransform = Ti;
            }

            GlobalTransform = GlobalTransform * PairTransform;

            pcl::transformPointCloud(*originalObject, *originalObjectTransformed, GlobalTransform);

            // Show alignment

            visu->updatePointCloud(originalScene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalScene, 0.0, 255.0, 0.0), "scene");
            visu->updatePointCloud(originalObjectTransformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObjectTransformed, 0.0, 0.0, 255.0), "object_aligned");
            visu->spinOnce();

            if (align.hasConverged ()) {
                std::stringstream ss2;
                ss2 << i << "_ICP.pcd";
                std::string filePath = "sample_pairwiseICP/" + ss2.str();
                logStream << "Writing " << originalObjectTransformed->size() << " points to file " << filePath;
                LOG.DEBUG(logStream.str()); logStream.str("");
                pcl::io::savePCDFileBinary (filePath, *originalObjectTransformed);
            }
            else {
                LOG.DEBUG("Alignment failed!");
                bool alignmentFailed = true;
                assert(alignmentFailed == false && "ICP alignment failed. Aborting.");
            }
        }
        /// Else if the debugging level is not high, try to do it more quickly. We should probably still reduce max correspondence distance as we go.
        else {
            LOG.DEBUG("Starting alignment.");
            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
            align.setTransformationEpsilon (1e-6);
            align.setInputSource(object);
            align.setInputTarget(scene);
            align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());
            align.setMaximumIterations(inputParams.iterativeClosestPointParameters.getMaximumIterations());

            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
            int stopConditionCounter = 0;
            float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
            float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
            float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
            float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();

            while ((align.getMaxCorrespondenceDistance() > minimumCorrespondenceDistance)) {
                align.align(*objectAligned);
                object = objectAligned;
                align.setInputSource(object);

                //accumulate transformation between each Iteration
                Ti = align.getFinalTransformation () * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
                    if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                        logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        stopConditionCounter = 0;
                    }
                    else {
                        if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= 1.0) {
                            logStream << "\tCurrent distance minus step size is smaller than 1.0. Setting distance to correspondence distance of " << 1.0;
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(1.0);
                            stopConditionCounter = 0;
                        }
                        else {
                            logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(1.0);
                            align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            stopConditionCounter = 0;
                        }
                    }
                }

                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
                    stopConditionCounter = stopConditionCounter + 1;
                }

                prev = align.getLastIncrementalTransformation ();

                if (stopConditionCounter > 5) {
                    LOG.DEBUG("ICP stop condition met.");
                    break;
                }

            }

            logStream << "Fitness Score:\t" << align.getFitnessScore();
            LOG.DEBUG(logStream.str()); logStream.str("");

            /// Sometimes ICP isn't sufficient. Let's try adding a check based on fitness score, and send the pair to Prerejective RANSAC
            /// to get a better transformation.
            if (align.getFitnessScore() > inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure()) {
                logStream << "\tUnable to identify sufficiently good fit under threshold of " << inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure() <<
                                " using ICP. Attempting to use RANSAC.";
                LOG.DEBUG(logStream.str()); logStream.str("");
                PairTransform = returnPairwiseRegistrationTransformUsingRANSAC(originalScene, originalObject, inputParams, visu);
            }
            else {
                PairTransform = Ti;
            }

            GlobalTransform = GlobalTransform * PairTransform;

            pcl::transformPointCloud(*originalObject, *originalObjectTransformed, GlobalTransform);

            if (align.hasConverged ()) {
                std::stringstream ss2;
                ss2 << i << "_ICP.pcd";
                std::string filePath = "sample_pairwiseICP/" + ss2.str();
                logStream << "Writing " << originalObjectTransformed->size() << " points to file " << filePath;
                LOG.DEBUG(logStream.str()); logStream.str("");
                pcl::io::savePCDFileBinary (filePath, *originalObjectTransformed);
            }
            else {
                LOG.DEBUG("Alignment failed!");
                bool alignmentFailed = true;
                assert(alignmentFailed == false && "ICP alignment failed. Aborting.");
            }

        }
    }

  return(0);
}


/** This one against global ICP is helpful to refine the alignment after an initial round of the pairwise. Much of this code originates from the PCL tutorial
  * at http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php
  * Much of the code is duplicated from the pairwise registration.
  *
  * We take the time to use this one after the initial pairwise since the initial pairwise can be off by a few centimeters which has potential to mess
  * things up when trying to segment the pot out.
  */
int registerPointCloudsICP_oneAgainstGlobal(int argc, char **argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Initializing one against global registration of input .pcd format point clouds.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo filter points on the x, y, and z dimensions (e.g. to filter out part of the image that might harm ICP registration):");
    inputParams.passThroughFilterParameters.printParameters();
    LOG.DEBUG("\tTo downsample the point clouds and speed up registration:");
    inputParams.voxelGridFilterParameters.printParameters();
    LOG.DEBUG("\tTo define ICP parameters:");
    inputParams.iterativeClosestPointParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();

    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    pcl::visualization::PCLVisualizer *visu;
    std::string objectString = "object";
    std::string sceneString = "source";
    int viewport = 0;
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer;
        //For a two viewport viewer
        //visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
        //visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
        //For a three viewport viewer
        //visu->createViewPort (0.00, 0.0, 0.33, 1.0, originalViewport);
        //visu->createViewPort (0.33, 0.0, 0.66, 1.0, measurementViewport);
        //visu->createViewPort (0.66, 0.0, 1.00, 1.0, featureViewport);
        /*visu->setBackgroundColor(0.5, 0.5, 0.5);
        visu->setSize(1700, 1000);
        visu->addCoordinateSystem(50.0);
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
        visu->addPointCloud(tempCloud, objectString, viewport);
        visu->addPointCloud(tempCloud, sceneString, viewport);
    }

    int numberOfCloudsPlusOne = argc;
    assert(numberOfCloudsPlusOne >= 3 && "At least two point clouds should be passed at the command line.");

    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudVoxeled (new pcl::PointCloud<pcl::PointXYZ>);
    logStream << "Loading the first cloud: " << argv[1];
    LOG.DEBUG(logStream.str()); logStream.str("");

    pcl::PCLPointCloud2 cloudBlob_firstCloud;
    pcl::io::loadPCDFile(argv[1], cloudBlob_firstCloud);
    pcl::fromPCLPointCloud2(cloudBlob_firstCloud, *globalCloud); //scene gets modified, original scene does not.
    assert(globalCloud->size() > 5 && "Input cloud has too few points. Is it empty?");  //5 is an arbitrary value to make sure the cloud has some points.

    std::string filePath = "sample_registeredClouds/1_ICP.pcd";
    logStream << "Writing initial cloud (" << globalCloud->size() << " points) to file " << filePath;
    LOG.DEBUG(logStream.str()); logStream.str("");

    pcl::io::savePCDFileBinary (filePath, *globalCloud);

    for (int i = 2; i < argc; i++) {
        // Point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObject (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObjectTransformed (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectAligned (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PCLPointCloud2 cloudBlob_originalObject;

        logStream << "Loading point cloud from file:\n" << argv[i];
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

        pcl::io::loadPCDFile(argv[i], cloudBlob_originalObject);
        pcl::fromPCLPointCloud2(cloudBlob_originalObject, *originalObject);  //object gets modified, original object does not.
        pcl::fromPCLPointCloud2(cloudBlob_originalObject, *object);
        assert(object->size() > 5); //5 is an arbitrary value to make sure the cloud has some points.

        pcl::PointCloud<pcl::PointXYZ>::Ptr objectVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr globalVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        LOG.DEBUG("Downsampling with voxel grid.", inputParams.debuggingParameters.getDebuggingLevel(), 2);
        pcl::VoxelGrid<pcl::PointXYZ> gridObject;
        pcl::VoxelGrid<pcl::PointXYZ> gridGlobal;
        const float leaf = inputParams.voxelGridFilterParameters.getLeafSize();
        gridObject.setLeafSize (leaf, leaf, leaf);
        gridGlobal.setLeafSize (leaf, leaf, leaf);

        gridObject.setInputCloud (object);
        gridObject.filter (*objectVoxel);
        object = objectVoxel;
        gridGlobal.setInputCloud (globalCloud);
        gridGlobal.filter (*globalCloudVoxeled);


        /// Here we crudely cut off the pot to prevent it from causing problems during registration of the plant.
        { //Begin scoping for pass through filter.
        LOG.DEBUG("Filtering using pass through", inputParams.debuggingParameters.getDebuggingLevel(), 2);
        //PassThrough filter application
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThroughGlobal (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThroughObject (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> passThroughFilterGlobal;
        pcl::PassThrough<pcl::PointXYZ> passThroughFilterObject;

        passThroughFilterGlobal.setInputCloud(globalCloudVoxeled);
        passThroughFilterGlobal.setFilterFieldName("z");
        passThroughFilterGlobal.setFilterLimits(inputParams.passThroughFilterParameters.getZmin(), inputParams.passThroughFilterParameters.getZmax()); //500 and 4500 are the min and max "good values" from the kinect)
        passThroughFilterGlobal.filter(*ptr_cloud_filteredPassThroughGlobal);
        passThroughFilterGlobal.setInputCloud(ptr_cloud_filteredPassThroughGlobal);
        passThroughFilterGlobal.setFilterFieldName("y");
        passThroughFilterGlobal.setFilterLimits(inputParams.passThroughFilterParameters.getYmin(), inputParams.passThroughFilterParameters.getYmax()); //424; I think these may be "upside down"
        passThroughFilterGlobal.filter(*ptr_cloud_filteredPassThroughGlobal);
        passThroughFilterGlobal.setInputCloud(ptr_cloud_filteredPassThroughGlobal);
        passThroughFilterGlobal.setFilterFieldName("x");
        passThroughFilterGlobal.setFilterLimits(inputParams.passThroughFilterParameters.getXmin(), inputParams.passThroughFilterParameters.getXmax()); //512
        passThroughFilterGlobal.filter(*ptr_cloud_filteredPassThroughGlobal);

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

        globalCloudVoxeled = ptr_cloud_filteredPassThroughGlobal;
        object = ptr_cloud_filteredPassThroughObject;

        } // End scoping for pass through filter

        logStream << "Size of global cloud: " << globalCloudVoxeled->size() << std::endl <<
                        "Size of filtered object cloud: " << object->size();
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        /// If debugging level is set to high, visualize each individual transformation.
        /// There is tremendous duplication of code here between the debugging level if statment cases(and all throughout the ICP code). Consider refactoring.
        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(globalCloudVoxeled, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(globalCloudVoxeled, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(object, 0.0, 0.0, 255.0), objectString);
            visu->spinOnce();

            // Perform alignment
            LOG.DEBUG("Starting alignment.");
            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
            align.setTransformationEpsilon (1e-6);
            align.setInputSource(object);
            align.setInputTarget(globalCloudVoxeled);
            align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

            align.setMaximumIterations (2); //So that we can watch the iterations, we set this two 2, and do for loop over align() calls.
            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
            int stopConditionCounter = 0;
            float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
            float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
            float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
            float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();
            for (int i = 0; i < inputParams.iterativeClosestPointParameters.getMaximumIterations(); ++i) {
                align.align(*objectAligned);
                object = objectAligned;
                align.setInputSource (object);

                //accumulate transformation between each Iteration
                Ti = align.getFinalTransformation () * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
                    if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                        logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        stopConditionCounter = 0;
                    }
                    else {
                        if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= 1.0) {
                            logStream << "\tCurrent distance minus step size is smaller than 1.0. Setting distance to correspondence distance of " << 1.0;
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(1.0);
                            stopConditionCounter = 0;
                        }
                        else {
                            logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            stopConditionCounter = 0;
                        }
                    }

                }

                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
                    stopConditionCounter = stopConditionCounter + 1;
                }

                prev = align.getLastIncrementalTransformation ();

                // visualize current state
                visu->updatePointCloud(globalCloudVoxeled, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(globalCloudVoxeled, 0.0, 255.0, 0.0), sceneString);
                visu->updatePointCloud(objectAligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(objectAligned, 255.0, 0.0, 0.0), objectString);
                visu->spinOnce();

                if (stopConditionCounter > 5) {
                    LOG.DEBUG("ICP stop condition met.");
                    break;
                }

            }

            logStream << "Fitness Score:\t" << align.getFitnessScore();
            LOG.DEBUG(logStream.str()); logStream.str("");

            PairTransform = Ti;
            pcl::transformPointCloud(*originalObject, *originalObjectTransformed, PairTransform);

            logStream << "\tAdding cloud " << i << " to the global cloud.";
            LOG.DEBUG(logStream.str()); logStream.str("");
            *globalCloud = *globalCloud + *originalObjectTransformed;

            // Show alignment

            visu->updatePointCloud(globalCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(globalCloud, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(originalObjectTransformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObjectTransformed, 0.0, 0.0, 255.0), objectString);
            visu->spinOnce();

            if (align.hasConverged ()) {
                std::stringstream ss2;
                ss2 << i << "_ICP.pcd";
                std::string filePath = "sample_registeredClouds/" + ss2.str();
                logStream << "Writing " << originalObjectTransformed->size() << " points to file " << filePath;
                LOG.DEBUG(logStream.str()); logStream.str("");
                pcl::io::savePCDFileBinary (filePath, *originalObjectTransformed);
            }
            else {
                LOG.DEBUG("Alignment failed!");
                bool alignmentFailed = true;
                assert(alignmentFailed == false && "ICP alignment failed. Aborting.");
            }
        }
        /// Else if the debugging level is not high, try to do it more quickly. We should probably still reduce max correspondence distance as we go.
        else {
            // Perform alignment
            LOG.DEBUG("Starting alignment.");
            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
            align.setTransformationEpsilon (1e-6);
            align.setInputSource(object);
            align.setInputTarget(globalCloudVoxeled);
            align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

            align.setMaximumIterations(inputParams.iterativeClosestPointParameters.getMaximumIterations());
            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
            int stopConditionCounter = 0;
            float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
            float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
            float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
            float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();
            while ((align.getMaxCorrespondenceDistance() > minimumCorrespondenceDistance)) {
                align.align(*objectAligned);
                object = objectAligned;
                align.setInputSource (object);

                //accumulate transformation between each Iteration
                Ti = align.getFinalTransformation () * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
                    if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                        logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        stopConditionCounter = 0;
                    }
                    else {
                        if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= 1.0) {
                            logStream << "\tCurrent distance minus step size is smaller than 1.0. Setting distance to correspondence distance of " << 1.0;
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(1.0);
                            stopConditionCounter = 0;
                        }
                        else {
                            logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            stopConditionCounter = 0;
                        }
                    }

                }

                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
                    stopConditionCounter = stopConditionCounter + 1;
                }

                prev = align.getLastIncrementalTransformation ();

                if (stopConditionCounter > 5) {
                    LOG.DEBUG("ICP stop condition met.");
                    break;
                }

            }

            logStream << "Fitness Score:\t" << align.getFitnessScore();
            LOG.DEBUG(logStream.str()); logStream.str("");

            PairTransform = Ti;
            pcl::transformPointCloud(*originalObject, *originalObjectTransformed, PairTransform);

            logStream << "\tAdding cloud " << i << " to the global cloud.";
            LOG.DEBUG(logStream.str()); logStream.str("");
            *globalCloud = *globalCloud + *originalObjectTransformed;

            if (align.hasConverged ()) {
                std::stringstream ss2;
                ss2 << i << "_ICP.pcd";
                std::string filePath = "sample_registeredClouds/" + ss2.str();
                logStream << "Writing " << originalObjectTransformed->size() << " points to file " << filePath;
                LOG.DEBUG(logStream.str()); logStream.str("");
                pcl::io::savePCDFileBinary (filePath, *originalObjectTransformed);
            }
            else {
                LOG.DEBUG("Alignment failed!");
                bool alignmentFailed = true;
                assert(alignmentFailed == false && "ICP alignment failed. Aborting.");
            }
        }
    }

    //Combine the results into a cloud ready for processing.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCombined (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 1; i < argc; ++i) {
        std::stringstream ss;
        ss << i << "_ICP.pcd";
        std::string fileString = "sample_registeredClouds/" + ss.str();
        LOG.DEBUG("Combining clouds.");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromFile (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(fileString, *cloudFromFile);
        *cloudCombined = *cloudCombined + *cloudFromFile;
    }

    logStream << "Writing " << cloudCombined->size() << " points to file " << "sample_registeredClouds/combinedCloud.pcd";
    LOG.DEBUG(logStream.str()); logStream.str("");
    pcl::io::savePCDFile("sample_registeredClouds/combinedCloud.pcd", *cloudCombined, true);

    return (0);
}

/** Nearly identical to the oneAgainstGlobal, except adds a step to remove statistical outliers.
  * The idea here is that, prior to removing the pot from the images, the high density of points relative to the plant skews
  * statistical outlier removal. Additionally, removing outliers from the combined point cloud doesn't prevent the noise
  * from harming registration. So, this special step here is used to remove outliers and then register AFTER the pot has been removed.
  */
int registerPointCloudsICP_refinement(int argc, char **argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Initializing refinement registration, a one against global registration of input .pcd format point clouds with statistical outlier removal.");
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

    LOG.DEBUG("Beginning one against global ICP refinement");

    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    pcl::visualization::PCLVisualizer *visu;
    std::string objectString = "object";
    std::string sceneString = "source";
    int viewport = 0;
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer;
        //For a two viewport viewer
        //visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
        //visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
        //For a three viewport viewer
        //visu->createViewPort (0.00, 0.0, 0.33, 1.0, originalViewport);
        //visu->createViewPort (0.33, 0.0, 0.66, 1.0, measurementViewport);
        //visu->createViewPort (0.66, 0.0, 1.00, 1.0, featureViewport);
        /*visu->setBackgroundColor(0.5, 0.5, 0.5);
        visu->setSize(1700, 1000);
        visu->addCoordinateSystem(50.0);
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
        visu->addPointCloud(tempCloud, objectString, viewport);
        visu->addPointCloud(tempCloud, sceneString, viewport);
    }

    int numberOfCloudsPlusOne = argc;
    assert(numberOfCloudsPlusOne >= 3 && "At least two point clouds should be passed at the command line.");

    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalCloudVoxeled (new pcl::PointCloud<pcl::PointXYZ>);
    logStream << "Loading the first cloud: " << argv[1];
    LOG.DEBUG(logStream.str()); logStream.str("");
    pcl::PCLPointCloud2 cloudBlob_firstCloud;
    pcl::io::loadPCDFile(argv[1], cloudBlob_firstCloud);
    pcl::fromPCLPointCloud2(cloudBlob_firstCloud, *globalCloud);
    assert(globalCloud->size() > 5 && "Input cloud has too few points. Is it empty?");  //5 is an arbitrary value to make sure the cloud has some points.

    { //Scoping so we can use this filter again below.
    LOG.DEBUG("\tRemoving statistical outliers.");
    //Statistical removal of outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredStatistical (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(globalCloud);
    sor.setMeanK(inputParams.statisticalOutlierRemovalParameters.getMeanK());
    sor.setStddevMulThresh(inputParams.statisticalOutlierRemovalParameters.getStdDevMulThresh());
    sor.filter(*ptr_cloud_filteredStatistical);

    globalCloud = ptr_cloud_filteredStatistical;
    } //end scoping

    std::string filePath = "sample_registeredClouds/1_ICP.pcd";
    logStream << "Writing initial cloud (" << globalCloud->size() << " points) to file " << filePath;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    pcl::io::savePCDFileBinary (filePath, *globalCloud);

    for (int i = 2; i < argc; i++) {
        // Point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObject (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObjectTransformed (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectAligned (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PCLPointCloud2 cloudBlob_originalObject;

        logStream << "Loading point cloud from file:\n" << argv[i];
        LOG.DEBUG(logStream.str()); logStream.str("");

        pcl::io::loadPCDFile(argv[i], cloudBlob_originalObject);
        pcl::fromPCLPointCloud2(cloudBlob_originalObject, *originalObject);  //object gets modified, original object does not.
        pcl::fromPCLPointCloud2(cloudBlob_originalObject, *object);
        assert(object->size() > 5 && "Input cloud has too few points. Is it empty?"); //5 is an arbitrary value to make sure the cloud has some points.

        { //Scoping for statistical outlier removal
        LOG.DEBUG("Removing statistical outliers.");
        //Statistical removal of outliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredStatistical (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredStatisticalOriginal (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(object);
        sor.setMeanK(inputParams.statisticalOutlierRemovalParameters.getMeanK());
        sor.setStddevMulThresh(inputParams.statisticalOutlierRemovalParameters.getStdDevMulThresh());
        sor.filter(*ptr_cloud_filteredStatistical);

        object = ptr_cloud_filteredStatistical;

        sor.setInputCloud(originalObject);
        sor.setMeanK(inputParams.statisticalOutlierRemovalParameters.getMeanK());
        sor.setStddevMulThresh(inputParams.statisticalOutlierRemovalParameters.getStdDevMulThresh());
        sor.filter(*ptr_cloud_filteredStatisticalOriginal);

        originalObject = ptr_cloud_filteredStatisticalOriginal;
        } //end scoping for statistical outlier removal

        { //Scoping for voxel grid filtering of cloud.
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr globalVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        LOG.DEBUG("Downsampling with voxel grid.", inputParams.debuggingParameters.getDebuggingLevel(), 2);
        pcl::VoxelGrid<pcl::PointXYZ> gridObject;
        pcl::VoxelGrid<pcl::PointXYZ> gridGlobal;
        const float leafSize = inputParams.voxelGridFilterParameters.getLeafSize();
        gridObject.setLeafSize (leafSize, leafSize, leafSize);
        gridGlobal.setLeafSize (leafSize, leafSize, leafSize);

        gridObject.setInputCloud (object);
        gridObject.filter (*objectVoxel);
        object = objectVoxel;
        gridGlobal.setInputCloud (globalCloud);
        gridGlobal.filter (*globalCloudVoxeled);
        } //End scoping for voxel grid filtering of cloud.

        logStream << "\tSize of filtered object cloud: " << object->size();
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        /// If debugging level is set to high, visualize each individual transformation.
        /// There is tremendous duplication of code here between the debugging level if statment cases(and all throughout the ICP code). Consider refactoring.
        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(globalCloudVoxeled, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(globalCloudVoxeled, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(object, 0.0, 0.0, 255.0), objectString);
            visu->spinOnce();

            // Perform alignment
            LOG.DEBUG("Starting alignment.");
            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
            align.setTransformationEpsilon (1e-6);
            align.setInputSource(object);
            align.setInputTarget(globalCloudVoxeled);
            align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

            align.setMaximumIterations(2); //So that we can watch the iterations, we set this two 2, and do for loop over align() calls.
            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
            int stopConditionCounter = 0;
            float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
            float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
            float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
            float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();
            for (int i = 0; i < inputParams.iterativeClosestPointParameters.getMaximumIterations(); ++i) {
                align.align(*objectAligned);
                object = objectAligned;
                align.setInputSource (object);

                Ti = align.getFinalTransformation () * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
                    if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                        logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        stopConditionCounter = 0;
                    }
                    else {
                        if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= 1.0) {
                            logStream << "\tCurrent distance minus step size is smaller than 1.0. Setting distance to correspondence distance of " << 1.0;
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(1.0);
                            stopConditionCounter = 0;
                        }
                        else {
                            logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            stopConditionCounter = 0;
                        }
                    }

                }

                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
                    stopConditionCounter = stopConditionCounter + 1;
                }

                prev = align.getLastIncrementalTransformation ();

                // visualize current state
                visu->updatePointCloud(globalCloudVoxeled, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(globalCloudVoxeled, 0.0, 255.0, 0.0), sceneString);
                visu->updatePointCloud(objectAligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(objectAligned, 255.0, 0.0, 0.0), objectString);
                visu->spinOnce();

                if (stopConditionCounter > 5) {
                    LOG.DEBUG("ICP stop condition met.");
                    break;
                }

            }

            logStream << "Fitness Score:\t" << align.getFitnessScore();
            LOG.DEBUG(logStream.str()); logStream.str("");

            PairTransform = Ti;
            pcl::transformPointCloud(*originalObject, *originalObjectTransformed, PairTransform);

            logStream << "\tAdding cloud " << i << " to the global cloud.";
            LOG.DEBUG(logStream.str()); logStream.str("");
            *globalCloud = *globalCloud + *originalObjectTransformed;

            // Show alignment

            visu->updatePointCloud(globalCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(globalCloud, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(originalObjectTransformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObjectTransformed, 0.0, 0.0, 255.0), objectString);
            visu->spinOnce();

            if (align.hasConverged ()) {
                std::stringstream ss2;
                ss2 << i << "_ICP.pcd";
                std::string filePath = "sample_registeredClouds/" + ss2.str();
                logStream << "Writing " << originalObjectTransformed->size() << " points to file " << filePath;
                LOG.DEBUG(logStream.str()); logStream.str("");
                pcl::io::savePCDFileBinary (filePath, *originalObjectTransformed);
            }
            else {
                LOG.DEBUG("Alignment failed!");
                bool alignmentFailed = true;
                assert(alignmentFailed == false && "ICP alignment failed. Aborting.");
            }
        }
        /// Else if the debugging level is not high, try to do it more quickly. We should probably still reduce max correspondence distance as we go.
        else {
            // Perform alignment
            LOG.DEBUG("Starting alignment.");
            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
            align.setTransformationEpsilon (1e-6);
            align.setInputSource(object);
            align.setInputTarget(globalCloudVoxeled);
            align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

            align.setMaximumIterations(inputParams.iterativeClosestPointParameters.getMaximumIterations());
            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
            int stopConditionCounter = 0;
            float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
            float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
            float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
            float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();
            while ((align.getMaxCorrespondenceDistance() > minimumCorrespondenceDistance)) {
                align.align(*objectAligned);
                object = objectAligned;

                // Estimate
                align.setInputSource (object);

                //accumulate transformation between each Iteration
                Ti = align.getFinalTransformation () * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
                    if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                        logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        stopConditionCounter = 0;
                    }
                    else {
                        if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= 1.0) {
                            logStream << "\tCurrent distance minus step size is smaller than 1.0. Setting distance to correspondence distance of " << 1.0;
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(1.0);
                            stopConditionCounter = 0;
                        }
                        else {
                            logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            stopConditionCounter = 0;
                        }
                    }

                }

                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
                    stopConditionCounter = stopConditionCounter + 1;
                }

                prev = align.getLastIncrementalTransformation ();

                if (stopConditionCounter > 5) {
                    LOG.DEBUG("ICP stop condition met.");
                    break;
                }

            }

            logStream << "Fitness Score:\t" << align.getFitnessScore();
            LOG.DEBUG(logStream.str()); logStream.str("");

            PairTransform = Ti;
            pcl::transformPointCloud(*originalObject, *originalObjectTransformed, PairTransform);

            logStream << "\tAdding cloud " << i << " to the global cloud.";
            LOG.DEBUG(logStream.str()); logStream.str("");
            *globalCloud = *globalCloud + *originalObjectTransformed;

            if (align.hasConverged ()) {
                std::stringstream ss2;
                ss2 << i << "_ICP.pcd";
                std::string filePath = "sample_registeredClouds/" + ss2.str();
                logStream << "Writing " << originalObjectTransformed->size() << " points to file " << filePath;
                LOG.DEBUG(logStream.str()); logStream.str("");
                pcl::io::savePCDFileBinary (filePath, *originalObjectTransformed);
            }
            else {
                LOG.DEBUG("Alignment failed!");
                bool alignmentFailed = true;
                assert(alignmentFailed == false && "ICP alignment failed. Aborting.");
            }
        }
    }

    //Combine the results into a cloud ready for processing.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCombined (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 1; i < argc; ++i) {
        std::stringstream ss;
        ss << i << "_ICP.pcd";
        std::string fileString = "sample_registeredClouds/" + ss.str();
        LOG.DEBUG("Combining clouds");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFromFile (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(fileString, *cloudFromFile);
        *cloudCombined = *cloudCombined + *cloudFromFile;
    }

    logStream << "Writing " << cloudCombined->size() << " points to file " << "sample_registeredClouds/combinedCloud.pcd";
    LOG.DEBUG(logStream.str()); logStream.str("");
    pcl::io::savePCDFile("sample_registeredClouds/combinedCloud.pcd", *cloudCombined, true);

    return (0);
}


int registerKinectFusionPLYs(int argc, char **argv, InputParameters inputParams) {
    std::ostringstream logStream;

    LOG.DEBUG("Initializing pairwise registration of input Kinect Fusion .ply format point clouds.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo filter points on the x, y, and z dimensions (e.g. to filter out part of the image that might harm ICP registration):");
    inputParams.passThroughFilterParameters.printParameters();
    LOG.DEBUG("\tTo downsample the point clouds and speed up registration:");
    inputParams.voxelGridFilterParameters.printParameters();
    LOG.DEBUG("\tTo define ICP parameters:");
    inputParams.iterativeClosestPointParameters.printParameters();
    LOG.DEBUG("\tTo define pre-rejective RANSAC parameters in case ICP fails:");
    inputParams.sampleConsensusPrerejectiveParameters.printParameters();
    LOG.DEBUG("\tTo calculate point normals in case RANSAC is used:");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo calculate point features in case RANSAC is used:");
    inputParams.featureEstimationParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();


    pcl::visualization::PCLVisualizer *visu;
    std::string objectString = "object";
    std::string sceneString = "source";
    int viewport = 0;
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer;
        //For a two viewport viewer
        //visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
        //visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
        //For a three viewport viewer
        //visu->createViewPort (0.00, 0.0, 0.33, 1.0, originalViewport);
        //visu->createViewPort (0.33, 0.0, 0.66, 1.0, measurementViewport);
        //visu->createViewPort (0.66, 0.0, 1.00, 1.0, featureViewport);
        /*visu->setBackgroundColor(0.5, 0.5, 0.5);
        visu->setSize(1700, 1000);
        visu->addCoordinateSystem(50.0);
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
        visu->addPointCloud(tempCloud, objectString, viewport);
        visu->addPointCloud(tempCloud, sceneString, viewport);
    }


    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    int numberOfCloudsPlusOne = argc;
    assert(numberOfCloudsPlusOne >= 3 && "At least two point clouds should be passed at the command line.");

    // Register cloud i and cloud i - 1 in a pairwise fashion.
    for (int i = 2; i < argc; i++) {
        // Point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObject (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr originalObjectTransformed (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr object (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectAligned (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointCloud<pcl::PointXYZ>::Ptr originalScene (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PCLPointCloud2 cloudBlob_originalObject, cloudBlob_originalScene;
        pcl::PolygonMesh originalObjectMesh, originalSceneMesh;

        logStream << "Loading point clouds from files:\n" <<
            "\t" << argv[i-1] << " and " << argv[i];
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

        pcl::io::loadPLYFile(argv[i-1], originalSceneMesh);
        pcl::fromPCLPointCloud2(originalSceneMesh.cloud, *originalScene); //scene gets modified, original scene does not.
        pcl::fromPCLPointCloud2(originalSceneMesh.cloud, *scene);
        assert(scene->size() > 5 && "Input cloud has too few points. Is it empty?");  //5 is an arbitrary value to make sure the cloud has some points.

        pcl::io::loadPLYFile(argv[i], originalObjectMesh);
        pcl::fromPCLPointCloud2(originalObjectMesh.cloud, *originalObject);  //object gets modified, original object does not.
        pcl::fromPCLPointCloud2(originalObjectMesh.cloud, *object);
        assert(object->size() > 5 && "Input cloud has too few points. Is it empty?"); //5 is an arbitrary value to make sure the cloud has some points.

        // Everything is transformed to the frame of the first cloud, and the first cloud doesn't need to be transformed.
        if (i == 2) {
            std::stringstream ss2;
            ss2 << i-1 << "_ICP.pcd";
            std::string filePath = "sample_pairwiseICP/" + ss2.str();
            logStream << "Writing initial cloud (" << originalScene->size() << " points) to file " << filePath;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            pcl::io::savePCDFileBinary (filePath, *originalScene);
        }

        /// Here we downsample the plant to speed up registration.
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sceneVoxel (new pcl::PointCloud<pcl::PointXYZ>);
        LOG.DEBUG("Downsampling with voxel grid.", inputParams.debuggingParameters.getDebuggingLevel(), 2);
        pcl::VoxelGrid<pcl::PointXYZ> gridObject;
        pcl::VoxelGrid<pcl::PointXYZ> gridScene;
        const float leaf = inputParams.voxelGridFilterParameters.getLeafSize();
        gridObject.setLeafSize (leaf, leaf, leaf);
        gridScene.setLeafSize(leaf, leaf, leaf);

        gridObject.setInputCloud (object);
        gridObject.filter (*objectVoxel);
        gridScene.setInputCloud (scene);
        gridScene.filter (*sceneVoxel);

        /// Here we crudely cut off the pot to prevent it from causing problems during registration of the plant.
        { //Begin scoping for pass through filter.
        LOG.DEBUG("Filtering using pass through", inputParams.debuggingParameters.getDebuggingLevel(), 2);
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
        } // End scoping for pass through filter

        logStream << "Size of filtered scene cloud: " << scene->size() << std::endl <<
                        "Size of filtered object cloud: " << object->size();
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        /// If debugging level is set to high, visualize each individual transformation.
        /// There is tremendous duplication of code here between the debugging level if statment cases(and all throughout the ICP code). Consider refactoring.
        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(object, 0.0, 255.0, 255.0), objectString);
            visu->spinOnce();

            // Perform alignment
            LOG.DEBUG("Starting alignment.");

            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
            align.setTransformationEpsilon(1e-6); // Magic number for the epsilon
            align.setInputSource(object);
            align.setInputTarget(scene);
            align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

            align.setMaximumIterations (2); //So that we can watch the iterations, we set this two 2, and do for loop over align() calls.
            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
            int stopConditionCounter = 0;
            float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
            float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
            float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
            float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();
            for (int i = 0; i < inputParams.iterativeClosestPointParameters.getMaximumIterations(); ++i) {
                align.align(*objectAligned);
                object = objectAligned;
                align.setInputSource (object);

                //accumulate transformation between each Iteration
                Ti = align.getFinalTransformation () * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
                    if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                        logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        stopConditionCounter = 0;
                    }
                    else {
                        if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= 0.001) {
                            logStream << "\tCurrent distance minus step size is smaller than 0.001. Setting distance to correspondence distance of " << 0.001;
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(0.001);
                            stopConditionCounter = 0;
                        }
                        else {
                            logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            stopConditionCounter = 0;
                        }
                    }
                }

                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
                    stopConditionCounter = stopConditionCounter + 1;
                }

                prev = align.getLastIncrementalTransformation ();

                // visualize current state
                visu->updatePointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(scene, 0.0, 255.0, 0.0), sceneString);
                visu->updatePointCloud(objectAligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(objectAligned, 0.0, 255.0, 255.0), objectString);
                visu->spinOnce();

                if (stopConditionCounter > 5) {
                    LOG.DEBUG("ICP stop condition met.");
                    break;
                }

            }

            logStream << "Fitness Score:\t" << align.getFitnessScore();
            LOG.DEBUG(logStream.str()); logStream.str("");

            /// Sometimes ICP isn't sufficient. Let's try adding a check based on fitness score, and send the pair to Prerejective RANSAC
            /// to get a better transformation.
            if (align.getFitnessScore() > inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure()) {
                logStream << "\tUnable to identify sufficiently good fit under threshold of " << inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure() <<
                                " using ICP. Attempting to use RANSAC.";
                LOG.DEBUG(logStream.str()); logStream.str("");
                PairTransform = returnPairwiseRegistrationTransformUsingRANSAC(originalScene, originalObject, inputParams, visu);
            }
            else {
                PairTransform = Ti;
            }

            GlobalTransform = GlobalTransform * PairTransform;

            pcl::transformPointCloud(*originalObject, *originalObjectTransformed, GlobalTransform);

            // Show alignment

            visu->updatePointCloud(originalScene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalScene, 0.0, 255.0, 0.0), "scene");
            visu->updatePointCloud(originalObjectTransformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObjectTransformed, 0.0, 0.0, 255.0), "object_aligned");
            visu->spinOnce();

            if (align.hasConverged ()) {
                std::stringstream ss2;
                ss2 << i << "_ICP.pcd";
                std::string filePath = "sample_pairwiseICP/" + ss2.str();
                logStream << "Writing " << originalObjectTransformed->size() << " points to file " << filePath;
                LOG.DEBUG(logStream.str()); logStream.str("");
                pcl::io::savePCDFileBinary (filePath, *originalObjectTransformed);
            }
            else {
                LOG.DEBUG("Alignment failed!");
                bool alignmentFailed = true;
                assert(alignmentFailed == false && "ICP alignment failed. Aborting.");
            }
        }
        /// Else if the debugging level is not high, try to do it more quickly. We should probably still reduce max correspondence distance as we go.
        else {
            LOG.DEBUG("Starting alignment.");
            pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
            align.setTransformationEpsilon (1e-6);
            align.setInputSource(object);
            align.setInputTarget(scene);
            align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());
            align.setMaximumIterations(inputParams.iterativeClosestPointParameters.getMaximumIterations());

            Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
            int stopConditionCounter = 0;
            float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
            float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
            float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
            float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();

            while ((align.getMaxCorrespondenceDistance() > minimumCorrespondenceDistance)) {
                align.align(*objectAligned);
                object = objectAligned;
                align.setInputSource(object);

                //accumulate transformation between each Iteration
                Ti = align.getFinalTransformation () * Ti;

                //if the difference between this transformation and the previous one
                //is smaller than the threshold, refine the process by reducing
                //the maximal correspondence distance
                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
                    if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                        logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                        stopConditionCounter = 0;
                    }
                    else {
                        if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= 1.0) {
                            logStream << "\tCurrent distance minus step size is smaller than 1.0. Setting distance to correspondence distance of " << 1.0;
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(1.0);
                            stopConditionCounter = 0;
                        }
                        else {
                            logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            align.setMaxCorrespondenceDistance(1.0);
                            align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                            stopConditionCounter = 0;
                        }
                    }
                }

                if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
                   (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
                    stopConditionCounter = stopConditionCounter + 1;
                }

                prev = align.getLastIncrementalTransformation ();

                if (stopConditionCounter > 5) {
                    LOG.DEBUG("ICP stop condition met.");
                    break;
                }

            }

            logStream << "Fitness Score:\t" << align.getFitnessScore();
            LOG.DEBUG(logStream.str()); logStream.str("");

            /// Sometimes ICP isn't sufficient. Let's try adding a check based on fitness score, and send the pair to Prerejective RANSAC
            /// to get a better transformation.
            if (align.getFitnessScore() > inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure()) {
                logStream << "\tUnable to identify sufficiently good fit under threshold of " << inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure() <<
                                " using ICP. Attempting to use RANSAC.";
                LOG.DEBUG(logStream.str()); logStream.str("");
                PairTransform = returnPairwiseRegistrationTransformUsingRANSAC(originalScene, originalObject, inputParams, visu);
            }
            else {
                PairTransform = Ti;
            }

            GlobalTransform = GlobalTransform * PairTransform;

            pcl::transformPointCloud(*originalObject, *originalObjectTransformed, GlobalTransform);

            if (align.hasConverged ()) {
                std::stringstream ss2;
                ss2 << i << "_ICP.pcd";
                std::string filePath = "sample_pairwiseICP/" + ss2.str();
                logStream << "Writing " << originalObjectTransformed->size() << " points to file " << filePath;
                LOG.DEBUG(logStream.str()); logStream.str("");
                pcl::io::savePCDFileBinary (filePath, *originalObjectTransformed);
            }
            else {
                LOG.DEBUG("Alignment failed!");
                bool alignmentFailed = true;
                assert(alignmentFailed == false && "ICP alignment failed. Aborting.");
            }

        }
    }



    return (0);
}


pcl::CorrespondencesPtr registerPointCloudsICPAndReturnCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                                                                    pcl::visualization::PCLVisualizer* visu,
                                                                    InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Initializing pairwise registration of input point clouds.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo define ICP parameters:");
    inputParams.iterativeClosestPointParameters.printParameters();
    LOG.DEBUG("\tTo define pre-rejective RANSAC parameters in case ICP fails:");
    inputParams.sampleConsensusPrerejectiveParameters.printParameters();
    LOG.DEBUG("\tTo calculate point normals in case RANSAC is used:");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo calculate point features in case RANSAC is used:");
    inputParams.featureEstimationParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    std::string objectString = "object";
    std::string sceneString = "source";
    std::string LSystemCloudString = "LSystemCloud";
    std::string LSystemString = "LSystemMesh";


    logStream << "Size of scene cloud: " << inputSourceCloud->size() << std::endl <<
                "Size of target cloud: " << inputTargetCloud->size();
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    // Perform alignment
    LOG.DEBUG("Starting alignment.");

    //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
    IterativeClosestPointNonLinear_Exposed<pcl::PointXYZ, pcl::PointXYZ> align;
    align.setTransformationEpsilon(1e-6); // Magic number for the epsilon
    align.setInputSource(inputSourceCloud);
    align.setInputTarget(inputTargetCloud);
    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

    align.setMaximumIterations (2); //So that we can watch the iterations, we set this two 2, and do for loop over align() calls.
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
    int stopConditionCounter = 0;
    float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
    //float minimumCorrespondenceDistance = inputParams.voxelGridFilterParameters.getLeafSize() * sqrt(3);
    float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
    float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
    float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceAligned (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inputParams.iterativeClosestPointParameters.getMaximumIterations(); ++i) {
        align.align(*sourceAligned);
        source = sourceAligned;
        align.setInputSource(source);

        //accumulate transformation between each Iteration
        Ti = align.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
            if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                stopConditionCounter = 0;
            }
            else {
                if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance()) {
                    logStream << "\tCurrent distance minus step size is smaller than " << inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance() << "Setting distance to correspondence distance of " << inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance());
                    stopConditionCounter = 0;
                }
                else {
                    logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                    stopConditionCounter = 0;
                }
            }
        }

        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
            stopConditionCounter = stopConditionCounter + 1;
        }

        prev = align.getLastIncrementalTransformation ();

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(inputTargetCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(inputTargetCloud, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(sourceAligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(sourceAligned, 255.0, 0.0, 255.0), LSystemCloudString);
            visu->spinOnce();
        }

        if (stopConditionCounter > 5) {
            LOG.DEBUG("ICP stop condition met.");
            break;
        }

    }

    logStream << "Fitness Score:\t" << align.getFitnessScore();
    LOG.DEBUG(logStream.str()); logStream.str("");

            /// Sometimes ICP isn't sufficient. Let's try adding a check based on fitness score, and send the pair to Prerejective RANSAC
            /// to get a better transformation.
            //if (align.getFitnessScore() > inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure()) {
             //   logStream << "\tUnable to identify sufficiently good fit under threshold of " << inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure() <<
             //                   " using ICP. Attempting to use RANSAC.";
             //   LOG.DEBUG(logStream.str()); logStream.str("");
             //   PairTransform = returnPairwiseRegistrationTransformUsingRANSAC(originalScene, originalObject, inputParams, visu);
            //}
            //else {
    PairTransform = Ti;
            //}

    GlobalTransform = GlobalTransform * PairTransform;

    pcl::transformPointCloud(*inputSourceCloud, *sourceAligned, GlobalTransform);



    //http://docs.pointclouds.org/1.7.0/classpcl_1_1registration_1_1_correspondence_rejector.html
    std::vector< pcl::registration::CorrespondenceRejector::Ptr > correspondenceRejectors = align.getCorrespondenceRejectors();



    //align.setMaxCorrespondenceDistance(inputParams.voxelGridFilterParameters.getLeafSize() * sqrt(3));
    //align.setInputSource(source);
    //align.setInputTarget(inputTargetCloud);
    //align.align(*sourceAligned);

    logStream << "size of correspondences?: " << align.getCorrespondencesPtr()->size();
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    pcl::CorrespondencesPtr correspondences = align.getCorrespondencesPtr();

    /*for (uint32_t i = 0; i < correspondences->size(); i++) {
        pcl::Correspondence currentCorrespondence = (*correspondences)[i];
        logStream << "From outside the class. Index of the source point: " << currentCorrespondence.index_query << std::endl;
        logStream << "Index of the matching target point: " << currentCorrespondence.index_match << std::endl;
        logStream << "Distance between the corresponding points: " << currentCorrespondence.distance << std::endl;
        logStream << "Weight of the confidence in the correspondence: " << currentCorrespondence.weight;
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        //visu->addPlane(*planeCoefficients, "planeString", viewport);
        srand(time(NULL));
        std::stringstream lineStringNameStream;
        lineStringNameStream << "lineString_" << rand()%10000 + i;
        visu->addLine(sourceAligned->points[currentCorrespondence.index_query], inputTargetCloud->points[currentCorrespondence.index_match], lineStringNameStream.str());
    }*/

    // Show alignment
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(inputTargetCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(inputTargetCloud, 0.0, 255.0, 0.0), sceneString);
        visu->updatePointCloud(sourceAligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(sourceAligned, 255.0, 0.0, 255.0), LSystemCloudString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the registration between the L-system and the point cloud. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }



    return(correspondences);
}


int registerLSystemICP(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputSourceTransformedToTarget,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams) {

    std::ostringstream logStream;
    LOG.DEBUG("Initializing pairwise registration of input point clouds.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo define ICP parameters:");
    inputParams.iterativeClosestPointParameters.printParameters();
    LOG.DEBUG("\tTo define pre-rejective RANSAC parameters in case ICP fails:");
    inputParams.sampleConsensusPrerejectiveParameters.printParameters();
    LOG.DEBUG("\tTo calculate point normals in case RANSAC is used:");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo calculate point features in case RANSAC is used:");
    inputParams.featureEstimationParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    std::string objectString = "object";
    std::string sceneString = "source";
    std::string LSystemCloudString = "LSystemCloud";
    std::string LSystemString = "LSystemMesh";


    logStream << "Size of scene cloud: " << inputSourceCloud->size() << std::endl <<
                "Size of target cloud: " << inputTargetCloud->size();
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    // Perform alignment
    LOG.DEBUG("Starting alignment.");

    //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
    IterativeClosestPointNonLinear_Exposed<pcl::PointXYZ, pcl::PointXYZ> align;
    align.setTransformationEpsilon(1e-6); // Magic number for the epsilon
    align.setInputSource(inputSourceCloud);
    align.setInputTarget(inputTargetCloud);
    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

    align.setMaximumIterations (2); //So that we can watch the iterations, we set this two 2, and do for loop over align() calls.
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
    int stopConditionCounter = 0;
    float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
    //float minimumCorrespondenceDistance = inputParams.voxelGridFilterParameters.getLeafSize() * sqrt(3);
    float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
    float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
    float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceAligned (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inputParams.iterativeClosestPointParameters.getMaximumIterations(); ++i) {
        align.align(*sourceAligned);
        source = sourceAligned;
        align.setInputSource(source);

        //accumulate transformation between each Iteration
        Ti = align.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
            if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                stopConditionCounter = 0;
            }
            else {
                if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance()) {
                    logStream << "\tCurrent distance minus step size is smaller than " << inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance() << "Setting distance to correspondence distance of " << inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance());
                    stopConditionCounter = 0;
                }
                else {
                    logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                    stopConditionCounter = 0;
                }
            }
        }

        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
            stopConditionCounter = stopConditionCounter + 1;
        }

        prev = align.getLastIncrementalTransformation ();

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(inputTargetCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(inputTargetCloud, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(sourceAligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(sourceAligned, 255.0, 0.0, 255.0), LSystemCloudString);
            visu->spinOnce();
        }

        if (stopConditionCounter > 5) {
            LOG.DEBUG("ICP stop condition met.");
            break;
        }

    }

    logStream << "Fitness Score:\t" << align.getFitnessScore();
    LOG.DEBUG(logStream.str()); logStream.str("");

            /// Sometimes ICP isn't sufficient. Let's try adding a check based on fitness score, and send the pair to Prerejective RANSAC
            /// to get a better transformation.
            //if (align.getFitnessScore() > inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure()) {
             //   logStream << "\tUnable to identify sufficiently good fit under threshold of " << inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure() <<
             //                   " using ICP. Attempting to use RANSAC.";
             //   LOG.DEBUG(logStream.str()); logStream.str("");
             //   PairTransform = returnPairwiseRegistrationTransformUsingRANSAC(originalScene, originalObject, inputParams, visu);
            //}
            //else {
    PairTransform = Ti;
            //}

    GlobalTransform = GlobalTransform * PairTransform;

    pcl::transformPointCloud(*inputSourceCloud, *sourceAligned, GlobalTransform);

    outputSourceTransformedToTarget->points = sourceAligned->points;


    // Show alignment
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(inputTargetCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(inputTargetCloud, 0.0, 255.0, 0.0), sceneString);
        visu->updatePointCloud(outputSourceTransformedToTarget, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(outputSourceTransformedToTarget, 255.0, 0.0, 255.0), LSystemCloudString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the registration between the L-system and the point cloud. Press q to continue.");
            visu->spinOnce();
        }
        else {
            visu->spinOnce();
        }
    }

    return(0);
}

Eigen::Matrix4f registerLSystemICPAndReturnTranslationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputSourceTransformedToTarget,
                                                            pcl::visualization::PCLVisualizer* visu,
                                                            InputParameters inputParams) {

    std::ostringstream logStream;
    LOG.DEBUG("Initializing pairwise registration of input point clouds to find a translation matrix.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo define ICP parameters:");
    inputParams.iterativeClosestPointParameters.printParameters();
    LOG.DEBUG("\tTo define pre-rejective RANSAC parameters in case ICP fails:");
    inputParams.sampleConsensusPrerejectiveParameters.printParameters();
    LOG.DEBUG("\tTo calculate point normals in case RANSAC is used:");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo calculate point features in case RANSAC is used:");
    inputParams.featureEstimationParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    std::string objectString = "object";
    std::string sceneString = "source";
    std::string LSystemCloudString = "LSystemCloud";
    std::string LSystemString = "LSystemMesh";


    logStream << "Size of scene cloud: " << inputSourceCloud->size() << std::endl <<
                "Size of target cloud: " << inputTargetCloud->size();
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    // Perform alignment
    LOG.DEBUG("Starting alignment.");

    //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
    IterativeClosestPointNonLinear_Exposed<pcl::PointXYZ, pcl::PointXYZ> align;
    align.setTransformationEpsilon(1e-6); // Magic number for the epsilon
    align.setInputSource(inputSourceCloud);
    align.setInputTarget(inputTargetCloud);
    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

    align.setMaximumIterations (2); //So that we can watch the iterations, we set this two 2, and do for loop over align() calls.
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
    int stopConditionCounter = 0;
    float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
    //float minimumCorrespondenceDistance = inputParams.voxelGridFilterParameters.getLeafSize() * sqrt(3);
    float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
    float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
    float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceAligned (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inputParams.iterativeClosestPointParameters.getMaximumIterations(); ++i) {
        align.align(*sourceAligned);
        source = sourceAligned;
        align.setInputSource(source);

        //accumulate transformation between each Iteration
        Ti = align.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
            if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                stopConditionCounter = 0;
            }
            else {
                if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance()) {
                    logStream << "\tCurrent distance minus step size is smaller than " << inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance() << "Setting distance to correspondence distance of " << inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance());
                    stopConditionCounter = 0;
                }
                else {
                    logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                    stopConditionCounter = 0;
                }
            }
        }

        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
            stopConditionCounter = stopConditionCounter + 1;
        }

        prev = align.getLastIncrementalTransformation ();

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(inputTargetCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(inputTargetCloud, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(sourceAligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(sourceAligned, 255.0, 0.0, 255.0), LSystemCloudString);
            visu->spinOnce();
        }

        if (stopConditionCounter > 5) {
            LOG.DEBUG("ICP stop condition met.");
            break;
        }

    }

    logStream << "Fitness Score:\t" << align.getFitnessScore();
    LOG.DEBUG(logStream.str()); logStream.str("");

            /// Sometimes ICP isn't sufficient. Let's try adding a check based on fitness score, and send the pair to Prerejective RANSAC
            /// to get a better transformation.
            //if (align.getFitnessScore() > inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure()) {
             //   logStream << "\tUnable to identify sufficiently good fit under threshold of " << inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure() <<
             //                   " using ICP. Attempting to use RANSAC.";
             //   LOG.DEBUG(logStream.str()); logStream.str("");
             //   PairTransform = returnPairwiseRegistrationTransformUsingRANSAC(originalScene, originalObject, inputParams, visu);
            //}
            //else {
    PairTransform = Ti;
            //}

    GlobalTransform = GlobalTransform * PairTransform;

    pcl::transformPointCloud(*inputSourceCloud, *sourceAligned, GlobalTransform);

    outputSourceTransformedToTarget->points = sourceAligned->points;


    // Show alignment
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(inputTargetCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(inputTargetCloud, 0.0, 255.0, 0.0), sceneString);
        visu->updatePointCloud(outputSourceTransformedToTarget, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(outputSourceTransformedToTarget, 255.0, 0.0, 255.0), LSystemCloudString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the registration between the L-system and the point cloud. Press q to continue.");
            visu->spinOnce();
        }
        else {
            visu->spinOnce();
        }
    }

    return(GlobalTransform);
}

float registerLSystemICPAndReturnFitness(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputSourceTransformedToTarget,
                                                            pcl::visualization::PCLVisualizer* visu,
                                                            InputParameters inputParams) {

    std::ostringstream logStream;
    LOG.DEBUG("Initializing pairwise registration of input point clouds to find a translation matrix.");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo define ICP parameters:");
    inputParams.iterativeClosestPointParameters.printParameters();
    LOG.DEBUG("\tTo define pre-rejective RANSAC parameters in case ICP fails:");
    inputParams.sampleConsensusPrerejectiveParameters.printParameters();
    LOG.DEBUG("\tTo calculate point normals in case RANSAC is used:");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo calculate point features in case RANSAC is used:");
    inputParams.featureEstimationParameters.printParameters();
    LOG.DEBUG("\tDebugging level:");
    inputParams.debuggingParameters.printParameters();

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    std::string objectString = "object";
    std::string sceneString = "source";
    std::string LSystemCloudString = "LSystemCloud";
    std::string LSystemString = "LSystemMesh";


    logStream << "Size of scene cloud: " << inputSourceCloud->size() << std::endl <<
                "Size of target cloud: " << inputTargetCloud->size();
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    // Perform alignment
    LOG.DEBUG("Starting alignment.");

    //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
    IterativeClosestPointNonLinear_Exposed<pcl::PointXYZ, pcl::PointXYZ> align;
    align.setTransformationEpsilon(1e-6); // Magic number for the epsilon
    align.setInputSource(inputSourceCloud);
    align.setInputTarget(inputTargetCloud);
    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

    align.setMaximumIterations (2); //So that we can watch the iterations, we set this two 2, and do for loop over align() calls.
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
    int stopConditionCounter = 0;
    float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
    //float minimumCorrespondenceDistance = inputParams.voxelGridFilterParameters.getLeafSize() * sqrt(3);
    float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
    float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
    float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceAligned (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inputParams.iterativeClosestPointParameters.getMaximumIterations(); ++i) {
        align.align(*sourceAligned);
        source = sourceAligned;
        align.setInputSource(source);

        //accumulate transformation between each Iteration
        Ti = align.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
            if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                stopConditionCounter = 0;
            }
            else {
                if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance()) {
                    logStream << "\tCurrent distance minus step size is smaller than " << inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance() << "Setting distance to correspondence distance of " << inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance());
                    stopConditionCounter = 0;
                }
                else {
                    logStream << "\tReducing max correspondence distance to " << (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                    stopConditionCounter = 0;
                }
            }
        }

        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
            stopConditionCounter = stopConditionCounter + 1;
        }

        prev = align.getLastIncrementalTransformation ();

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(inputTargetCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(inputTargetCloud, 0.0, 255.0, 0.0), sceneString);
            visu->updatePointCloud(sourceAligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(sourceAligned, 255.0, 0.0, 255.0), LSystemCloudString);
            visu->spinOnce();
        }

        if (stopConditionCounter > 5) {
            LOG.DEBUG("ICP stop condition met.");
            break;
        }

    }

    logStream << "Fitness Score:\t" << align.getFitnessScore();
    LOG.DEBUG(logStream.str()); logStream.str("");

            /// Sometimes ICP isn't sufficient. Let's try adding a check based on fitness score, and send the pair to Prerejective RANSAC
            /// to get a better transformation.
            //if (align.getFitnessScore() > inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure()) {
             //   logStream << "\tUnable to identify sufficiently good fit under threshold of " << inputParams.iterativeClosestPointParameters.getFitnessThresholdDefiningFailure() <<
             //                   " using ICP. Attempting to use RANSAC.";
             //   LOG.DEBUG(logStream.str()); logStream.str("");
             //   PairTransform = returnPairwiseRegistrationTransformUsingRANSAC(originalScene, originalObject, inputParams, visu);
            //}
            //else {
    PairTransform = Ti;
            //}

    GlobalTransform = GlobalTransform * PairTransform;

    pcl::transformPointCloud(*inputSourceCloud, *sourceAligned, GlobalTransform);

    outputSourceTransformedToTarget->points = sourceAligned->points;


    // Show alignment
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(inputTargetCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(inputTargetCloud, 0.0, 255.0, 0.0), sceneString);
        visu->updatePointCloud(outputSourceTransformedToTarget, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(outputSourceTransformedToTarget, 255.0, 0.0, 255.0), LSystemCloudString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the registration between the L-system and the point cloud. Press q to continue.");
            visu->spinOnce();
        }
        else {
            visu->spinOnce();
        }
    }

    return(align.getFitnessScore());
}

float registerLSystemICPAndReturnFitness_ForParallelCalling(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputSourceTransformedToTarget,
                                                            InputParameters inputParams) {
    std::ostringstream logStream;

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f PairTransform = Eigen::Matrix4f::Identity();

    //pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> align;
    IterativeClosestPointNonLinear_Exposed<pcl::PointXYZ, pcl::PointXYZ> align;
    align.setTransformationEpsilon(1e-6); // Magic number for the epsilon
    align.setInputSource(inputSourceCloud);
    align.setInputTarget(inputTargetCloud);
    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMaxCorrespondenceDistance());

    align.setMaximumIterations (2); //So that we can watch the iterations, we set this two 2, and do for loop over align() calls.
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev;
    int stopConditionCounter = 0;
    float minimumCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance();
    //float minimumCorrespondenceDistance = inputParams.voxelGridFilterParameters.getLeafSize() * sqrt(3);
    float largeStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction();
    float largeThresholdCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps();
    float smallStepCorrespondenceDistance = inputParams.iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction();

    pcl::PointCloud<pcl::PointXYZ>::Ptr sourceAligned (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < inputParams.iterativeClosestPointParameters.getMaximumIterations(); ++i) {
        align.align(*sourceAligned);
        source = sourceAligned;
        align.setInputSource(source);

        //accumulate transformation between each Iteration
        Ti = align.getFinalTransformation () * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () > minimumCorrespondenceDistance))  {
            if (align.getMaxCorrespondenceDistance () > largeThresholdCorrespondenceDistance) {
                align.setMaxCorrespondenceDistance(align.getMaxCorrespondenceDistance () - largeStepCorrespondenceDistance);
                stopConditionCounter = 0;
            }
            else {
                if ( (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance) <= inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance()) {
                    align.setMaxCorrespondenceDistance(inputParams.iterativeClosestPointParameters.getMinCorrespondenceDistance());
                    stopConditionCounter = 0;
                }
                else {
                    align.setMaxCorrespondenceDistance (align.getMaxCorrespondenceDistance () - smallStepCorrespondenceDistance);
                    stopConditionCounter = 0;
                }
            }
        }

        if ( (fabs ((align.getLastIncrementalTransformation () - prev).sum ()) < align.getTransformationEpsilon ()) &&
            (align.getMaxCorrespondenceDistance () <= minimumCorrespondenceDistance))  {
            stopConditionCounter = stopConditionCounter + 1;
        }

        prev = align.getLastIncrementalTransformation ();

        if (stopConditionCounter > 5) {
            break;
        }

    }

    PairTransform = Ti;
    GlobalTransform = GlobalTransform * PairTransform;

    pcl::transformPointCloud(*inputSourceCloud, *sourceAligned, GlobalTransform);

    outputSourceTransformedToTarget->points = sourceAligned->points;

    return(align.getFitnessScore());
}




