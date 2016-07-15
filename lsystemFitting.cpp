
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string>
#include <math.h>
#include <algorithm>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <python2.7/Python.h>

#include "loggingHelper.h"
#include "boundingBox.h"
#include "lsystemFitting.h"
#include "lsystemParameters.h"
#include "sampleMeshToPointCloud.h"
#include "IterativeClosestPoint.h"
#include "utilityFunctions.h"
#include "lsystemRefinement.h"


int identifyCylinderCloudsBasedOnStemModel(pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputInnerCylinderPoints,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputOuterCylinderPointsExcludingInnerCylinder,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputOuterCylinderPoints,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputOutsideOuterCylinderPoints,
                                            Eigen::VectorXf cylinderCoefficients,
                                            pcl::visualization::PCLVisualizer *visu,
                                            InputParameters inputParams) {
    std::ostringstream logStream;
    std::string objectString = "object";
    std::string sceneString = "source";
    int viewport = 0;

    outputInnerCylinderPoints->clear(); outputOuterCylinderPointsExcludingInnerCylinder->clear(); outputOuterCylinderPoints->clear(); outputOutsideOuterCylinderPoints->clear();
    /// I think the easiest approach will be to transform the cylinder to the z axis, and then find if a point is within the X,Y circle.

    Eigen::Vector3f vectorUpXaxis(1, 0, 0);
    Eigen::Vector3f vectorUpYaxis(0, 1, 0);
    Eigen::Vector3f vectorUpZaxis(0, 0, 1);
    Eigen::Vector3f cylinderNormal(cylinderCoefficients[3], cylinderCoefficients[4], cylinderCoefficients[5]);
    /// IMPORTANT MAGIC NUMBERS HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
    /// The inner radius number determines a scaling factor of how much the inner cylinder will be expanded.
    /// The outer radius number determines a scaling factor of how much the outer radius will be expanded relative to the inner radius.
    float innerRadius = cylinderCoefficients[6] + (inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue() * 1.1);
    float outerRadius = innerRadius * 4.0;

    Eigen::Matrix4f rotationMatrix = returnRotationMatrixToTranslateFirstNormalToSecondNormal(cylinderNormal, vectorUpZaxis);
    // Transform the cylinder axis to determine how to translate it to origin.
    // http://pointclouds.org/documentation/tutorials/matrix_transform.php
    Eigen::Vector4f cylinderOrigin4f(cylinderCoefficients[0], cylinderCoefficients[1], cylinderCoefficients[2], 1);
    Eigen::Vector4f cylinderOriginTransformed = rotationMatrix * cylinderOrigin4f;
    // Add the translation
    rotationMatrix(0,3) = cylinderOriginTransformed[0] * -(1.0);
    rotationMatrix(1,3) = cylinderOriginTransformed[1] * -(1.0);
    rotationMatrix(2,3) = cylinderOriginTransformed[2] * -(1.0);

    // Reset the cylinder origin vector so that we can test the transformation with the translation.
    cylinderOrigin4f[0] = cylinderCoefficients[0];
    cylinderOrigin4f[1] = cylinderCoefficients[1];
    cylinderOrigin4f[2] = cylinderCoefficients[2];
    cylinderOrigin4f[3] = 1;
    cylinderOriginTransformed = rotationMatrix * cylinderOrigin4f;

    Eigen::Vector4f cylinderNormal4f(cylinderCoefficients[3], cylinderCoefficients[4], cylinderCoefficients[5], 0);
    Eigen::Vector4f cylinderNormalTransformed = rotationMatrix * cylinderNormal4f;



    pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloudTransformed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*originalCloud, *originalCloudTransformed, rotationMatrix);

    //if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
    //    visu->updatePointCloud(originalCloudTransformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalCloudTransformed, 0.0, 255.0, 0.0), objectString);
    //    visu->updatePointCloud(originalCloudTransformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalCloudTransformed, 0.0, 255.0, 0.0), sceneString);
    //   pcl::PointXYZ cylinderStartPoint(cylinderOriginTransformed[0] - cylinderNormalTransformed[0] * 100,
    //                                    cylinderOriginTransformed[1]  - cylinderNormalTransformed[1] * 100,
    //                                   cylinderOriginTransformed[2]  - cylinderNormalTransformed[2] * 100);
    //   pcl::PointXYZ cylinderEndPoint(cylinderOriginTransformed[0] + cylinderNormalTransformed[0] * 100,
    //                                    cylinderOriginTransformed[1]  + cylinderNormalTransformed[1] * 100,
    //                                    cylinderOriginTransformed[2]  + cylinderNormalTransformed[2] * 100);
    //    srand(time(NULL));
    //    std::stringstream lineStringNameStream;
    //    lineStringNameStream << "lineStringSpotCheckCylinder_" << rand()%10000;
    //    visu->addLine(cylinderStartPoint, cylinderEndPoint, lineStringNameStream.str(), viewport);
    //    if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
    //        LOG.DEBUG("Displaying the transformed cloud used to find the radius about the Z axis. Press q to continue.");
    //        visu->spin();
    //    }
    //    else {
    //       visu->spinOnce();
    //    }
    //}

    for (uint32_t i = 0; i < originalCloudTransformed->size(); i++) {
        pcl::PointXYZ currentPointPreTransformation = originalCloud->points[i];
        pcl::PointXYZ currentPoint = originalCloudTransformed->points[i];
        // If point is within the radius of the cylinder (use X and Y dimensions).
        // Assuming the center in the origin, (0,0).
        float xd = currentPoint.x;
        float yd = currentPoint.y;
        float distance = sqrt(xd * xd + yd * yd);
        // If the distance is outside of the cylinder radius of the pot with some wiggle room, keep it.
        if (distance > innerRadius && distance <= outerRadius) {
            outputOuterCylinderPointsExcludingInnerCylinder->points.push_back(currentPointPreTransformation);
        }
        if (distance <= innerRadius) {
            outputInnerCylinderPoints->points.push_back(currentPointPreTransformation);
        }
        if (distance <= outerRadius) {
            outputOuterCylinderPoints->points.push_back(currentPointPreTransformation);
        }
        if (distance > outerRadius) {
            outputOutsideOuterCylinderPoints->points.push_back(currentPointPreTransformation);
        }

    }

    logStream << "The inner cylinder cloud (i.e. stem) has: " << outputInnerCylinderPoints->size() << " points.";
    logStream << "The outer cylinder with the inner cylinder subtracted has: " << outputOuterCylinderPointsExcludingInnerCylinder->size() << " points.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

    LOG.DEBUG("Finished identifying clouds based on stem model. Returning.");
    return(0);

}

int identifyPointsOfEmergingLeaves(pcl::PointCloud<pcl::PointXYZ>::Ptr inputStemCloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr inputPutativeLeafPoints,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr outputLeafPoints,
                                    InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Identifying which points should be considered as emerging leaves.");

    // I think we can take a region growing approach. Start with the stem cloud. Add any putative leaf points that are adjacent to it. Repeat until no more are adjacent.
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearch;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWithLeafPointsGrownFromStem (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPutativeLeafPointsRemainingToTest_CurrentIteration (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPutativeLeafPointsRemainingToTest_NextIteration (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPutativeLeafPointsToRetain (new pcl::PointCloud<pcl::PointXYZ>);

    int numberPutativeLeafPointsWithNeighbors = 999999; // this variable represents, on a current iteration, the number of putative leaf points that have a neighbor grown from the stem.
    cloudWithLeafPointsGrownFromStem->points = inputStemCloud->points;
    cloudPutativeLeafPointsRemainingToTest_CurrentIteration->points = inputPutativeLeafPoints->points;

    int iterationCounter = 0;
    while (numberPutativeLeafPointsWithNeighbors > 0) {
        logStream << "On iteration " << iterationCounter << ". There are " <<  cloudWithLeafPointsGrownFromStem->size() << " points in the cloud growing from the stem, and " <<
                        cloudPutativeLeafPointsRemainingToTest_CurrentIteration->size() << " putative leaf points remaining to be tested.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

        numberPutativeLeafPointsWithNeighbors = 0;
        cloudPutativeLeafPointsRemainingToTest_NextIteration->clear();
        kdtreeSearch.setInputCloud(cloudWithLeafPointsGrownFromStem);

        for (uint32_t i = 0; i < cloudPutativeLeafPointsRemainingToTest_CurrentIteration->size(); i ++) {
            pcl::PointXYZ currentPoint = cloudPutativeLeafPointsRemainingToTest_CurrentIteration->points[i];
            std::vector<int> pointIndicesRadiusSearch;
            std::vector<float> distanceRadiusSearch;
            /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
            /// This number determines the radius away from the stem points that putative leaf points will be considered neighbors.
            int numberNeighbors = kdtreeSearch.radiusSearch(currentPoint, 10.0, pointIndicesRadiusSearch, distanceRadiusSearch);
            if (numberNeighbors >= 1) {
                cloudWithLeafPointsGrownFromStem->points.push_back(currentPoint);
                numberPutativeLeafPointsWithNeighbors = numberPutativeLeafPointsWithNeighbors + 1;
                if (iterationCounter != 0) { //We don't want to include points that are immediately adjacent to the stem, since these can be points just missed by the cylinder
                    cloudPutativeLeafPointsToRetain->points.push_back(currentPoint);
                }
            }
            else {
                cloudPutativeLeafPointsRemainingToTest_NextIteration->points.push_back(currentPoint);
            }
        }

        cloudPutativeLeafPointsRemainingToTest_CurrentIteration->points = cloudPutativeLeafPointsRemainingToTest_NextIteration->points;

        iterationCounter = iterationCounter + 1;

    }

    outputLeafPoints->points = cloudPutativeLeafPointsToRetain->points;

    return(0);
}


Eigen::Vector4f findPlaneToBisectStem(Eigen::Matrix3f inputLeafPCAEigenVectors,
                                        Eigen::Vector3f inputLeafPCAEigenValues,
                                        pcl::PointXYZ inputOriginPoint,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr nonCylinderPoints,
                                        Eigen::VectorXf inputCylinderCoefficients,
                                        InputParameters inputParams) {

    std::ostringstream logStream;
    Eigen::Vector4f planeModelToReturn;
    LOG.DEBUG("Identifying a plane that bisects the stem to partition the leaves.");

    /// I think the trick will be to find the plane that contains both the plane normal vector and the cylinder vector.
    /// Then we can drop the vector of the plane normal to be 90 degrees relative to the cylinder vector using that plane.
    // http://mathhelpforum.com/pre-calculus/168196-equation-3rd-vector-coplanar-2-other-vectors-right-angles.html
    // http://math.stackexchange.com/questions/562123/equation-of-plane-containing-two-vectors

    // Vectors for testing.
    //Eigen::Vector3f vectorUpXaxis(1, 0, 0);
    //Eigen::Vector3f vectorUpYaxis(0, 1, 0);
    //Eigen::Vector3f vectorUpZaxis(0, 0, 1);

    //Eigen::Vector3f planeNormal(0.0, 0.7071, 0.7071);
    //Eigen::Vector3f planeNormal(0.7071, -0.7071, 0);
    //Eigen::Vector3f planeNormal(-0.7071, 0.7071, 0);
    //Eigen::Vector3f planeNormal(-0.7071, -0.7071, 0);
    //Eigen::Vector3f planeNormal(0, 0.819152, 0.57358);

    // The vector up Z axis corresponds to the stem cylinder vector.
    // the plane normal correspond to the leaf principal component.
    Eigen::Vector3f vectorUpZaxis(inputCylinderCoefficients[3], inputCylinderCoefficients[4], inputCylinderCoefficients[5]);
    Eigen::Vector3f planeNormal(inputLeafPCAEigenVectors(0,0), inputLeafPCAEigenVectors(1,0), inputLeafPCAEigenVectors(2,0));

    //logStream << "The vector approximating the stem is:\n" << vectorUpZaxis;
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "The vector corresponding to the leaf principal component is:\n" << planeNormal;
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "acos of dot product of x axis and plane normal is: " << acos(vectorUpXaxis.dot(planeNormal)) * (180.0 / M_PI);
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "acos of dot product of y axis and plane normal is: " << acos(vectorUpYaxis.dot(planeNormal)) * (180.0 / M_PI);
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "acos of dot product of z axis and plane normal is: " << acos(vectorUpZaxis.dot(planeNormal)) * (180.0 / M_PI);
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "acos of dot product of plane normal and x axis: " << acos(planeNormal.dot(vectorUpXaxis)) * (180.0 / M_PI);
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "acos of dot product of plane normal and y axis: " << acos(planeNormal.dot(vectorUpYaxis)) * (180.0 / M_PI);
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "acos of dot product of plane normal and z axis: " << acos(planeNormal.dot(vectorUpZaxis)) * (180.0 / M_PI);
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    // if the linear combination of two independent vectors = another coplanar vector, then we should be able to do something like (see http://mathhelpforum.com/pre-calculus/168196-equation-3rd-vector-coplanar-2-other-vectors-right-angles.html )
    // plane normal = A * unknownVector + B * z axis.
    // plane normal = cos(angleBetween plane normal and z) * unknownVector + sin(angle between plane normal and z) * z.

    float acosOfDotProduct = acos(planeNormal.dot(vectorUpZaxis));
    float A = sin(acosOfDotProduct);
    float B = cos(acosOfDotProduct);

    //planeNormal = (sin(acosOfDotProduct) * unknown) + (cos(acosOfDotProduct) * vectorUpZaxis), so...
    Eigen::Vector3f targetVector = ( planeNormal - (B * vectorUpZaxis) ) / A;

    //logStream << "A calculated to be: " << A << " and B calculated to be: " << B;
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "B times vectorUpZaxis is\n" << B * vectorUpZaxis;
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "leaf principal component - B times vectorUpZAxis is\n" << planeNormal - (B * vectorUpZaxis);
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "The resulting vector was calculated to be:\n" << targetVector;
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "If they are coplanar, then u dot (v x w) should be 0. It is actually " << targetVector.dot(vectorUpZaxis.cross(planeNormal));
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    //logStream << "If the target is perpendicular to the stem axis, then acos(targetVector.dot(vectorUpZAxis)) should be 90. It is actually " << acos(targetVector.dot(vectorUpZaxis)) * (180.0 / M_PI);
    //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    float intercept = -(targetVector[0] * inputOriginPoint.x) - (targetVector[1] * inputOriginPoint.y) - (targetVector[2] * inputOriginPoint.z);

    planeModelToReturn[0] = targetVector[0];
    planeModelToReturn[1] = targetVector[1];
    planeModelToReturn[2] = targetVector[2];
    planeModelToReturn[3] = intercept;


    /// In some cases, this can be misleading if the second principal component is leaf width as opposed to leaf length.
    /// To try to resolve cases where leaf width mistakenly gets used.
    /// If the leaf width dimension gets mistakenly used, a large proportion of the points on either side of the plane will be adjacent to each other.
    ///     If this is the case, I think we can swap the x and y of the principal component for the plane and try again.

    // First, make the clouds to check
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOnNormalSide (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOnOppositeNormalSide (new pcl::PointCloud<pcl::PointXYZ>);
    for (uint32_t i = 0; i < nonCylinderPoints->size(); i ++) {
        pcl::PointXYZ currentPoint = nonCylinderPoints->points[i];
        float planeSideResult =  (planeModelToReturn(0) * currentPoint.x) + (planeModelToReturn(1) * currentPoint.y) + (planeModelToReturn(2) * currentPoint.z) + planeModelToReturn(3); // Index 3 holds the intercept.
        if (planeSideResult > 0) {
            cloudPointsOnNormalSide->push_back(currentPoint);
        }
        else if (planeSideResult < 0) {
            cloudPointsOnOppositeNormalSide->push_back(currentPoint);
        }
    }

    // Then check adjacencies, but only if some were found on each of the two of the sides:
    if (cloudPointsOnNormalSide->size() > 0 && cloudPointsOnOppositeNormalSide->size() > 0) {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearch;
        kdtreeSearch.setInputCloud(cloudPointsOnNormalSide);

        int numberOfPointsFromCloudPointsOnOppositeNormalSideWithAdjacencyInCloudPointsOnNormalSide = 0;
        int numberOfPointsFromCloudPointsOnOppositeNormalSideWithoutAdjacencyInCloudPointsOnNormalSide = 0;
        for (uint32_t i = 0; i < cloudPointsOnOppositeNormalSide->size(); i ++) {
            pcl::PointXYZ currentPoint = cloudPointsOnOppositeNormalSide->points[i];
            std::vector<int> pointIndicesRadiusSearch;
            std::vector<float> distanceRadiusSearch;
            /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
            /// This number determines the radius away from the stem points that putative leaf points will be considered neighbors.
            int numberNeighbors = kdtreeSearch.radiusSearch(currentPoint, 15.0, pointIndicesRadiusSearch, distanceRadiusSearch);
            if (numberNeighbors >= 1) {
                numberOfPointsFromCloudPointsOnOppositeNormalSideWithAdjacencyInCloudPointsOnNormalSide += 1;
            }
            else {
                numberOfPointsFromCloudPointsOnOppositeNormalSideWithoutAdjacencyInCloudPointsOnNormalSide += 1;
            }
        }
        // Ideally, there should be no adjacencies; if there are, it likely indicates the leaf width dimension was used as the second principal component.

        float proportionAdjacent = (float)numberOfPointsFromCloudPointsOnOppositeNormalSideWithAdjacencyInCloudPointsOnNormalSide / (float)cloudPointsOnOppositeNormalSide->size();
        // If the proportion of points with an adjacency is too high, we should try swapping the x and y. I'm not sure this works in all cases and needs to be tested.
        if (proportionAdjacent > 0.05) {
            LOG.DEBUG("Found points on both sides of the leaf bisection that are adjacent to each other. Changing the plane.");
            planeModelToReturn[0] = targetVector[1];
            planeModelToReturn[1] = targetVector[0];
            planeModelToReturn[2] = targetVector[2];
            planeModelToReturn[3] = -(targetVector[1] * inputOriginPoint.x) - (targetVector[0] * inputOriginPoint.y) - (targetVector[2] * inputOriginPoint.z);
        }
    }

    return planeModelToReturn;

}


int fitLSystemToPointCloud(int argc, char** argv, InputParameters inputParams){
    std::ostringstream logStream;
    LOG.DEBUG("Initializing the fitting of an L-system to point clouds");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo estimate normals with K search:");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo perform segmentation to identify a cylinder corresponding to the stem:");
    inputParams.sacSegmentationFromNormalsParameters.printParameters();

    LOG.DEBUG("\tDebugging parameters:");
    inputParams.debuggingParameters.printParameters();


    /// Load the necessary elements for embedded python.
    LOG.DEBUG("Initializing Python interpreter for downstream L-system construction.");
    Py_Initialize();
    Py_SetProgramName("BuildLSystemFromC++");
    PyObject *sys = PyImport_ImportModule("sys");
    PyObject *path = PyObject_GetAttrString(sys, "path");
    PyList_Append(path, PyString_FromString("."));

    PyObject *pName = PyString_FromString("externalLPYcalling_v2");
    PyObject *pythonModuleForLSystemConstruction = PyImport_Import(pName);
    Py_DECREF(pName);
    assert(pythonModuleForLSystemConstruction != NULL && "The Python module for creating L-systems could not be successfully imported. Aborting.");

    PyObject *pythonFunctionForLSystemConstruction = PyObject_GetAttrString(pythonModuleForLSystemConstruction, "constructLSystem");
    assert(pythonFunctionForLSystemConstruction && PyCallable_Check(pythonFunctionForLSystemConstruction) && "The Python function for creating L-systems was not found. Aborting.");
    /// Finished loading elements for embedded python.

    int numberOfCloudsPlusOne = argc;
    assert(numberOfCloudsPlusOne >= 2 && "A cloud should be passed at the command line.");

    /// Set up the visualizer for viewing.
    pcl::visualization::PCLVisualizer *visu;
    std::string objectString = "object";
    std::string sceneString = "source";
    std::string LSystemCloudString = "LSystemCloud";
    std::string LSystemString = "LSystemMesh";
    // Mock up a temporary mesh:
    pcl::PolygonMesh tempMesh; // For temporary addition of an empty mesh that can later be updated.
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempViewerCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ tmpPoint1(0, 0, 0); pcl::PointXYZ tmpPoint2(0, 0.5, 0.5); pcl::PointXYZ tmpPoint3(0, 0, 0.5);
    tempViewerCloud->points.push_back(tmpPoint1); tempViewerCloud->points.push_back(tmpPoint2); tempViewerCloud->points.push_back(tmpPoint3);
    std::vector<pcl::Vertices> tmpMeshFaces;
    pcl::Vertices tmpTriangleFaceIndices;
    tmpTriangleFaceIndices.vertices.push_back(0); tmpTriangleFaceIndices.vertices.push_back(1); tmpTriangleFaceIndices.vertices.push_back(2);
    tmpMeshFaces.push_back(tmpTriangleFaceIndices);
    pcl::PCLPointCloud2 pc2_tempMeshCloud;
    pcl::toPCLPointCloud2(*tempViewerCloud, pc2_tempMeshCloud);
    tempMesh.cloud = pc2_tempMeshCloud;
    tempMesh.polygons = tmpMeshFaces;
    // Finished mocking up a temporary mesh.
    int viewport = 0;
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer;
        visu->addCoordinateSystem(100.0);
        //For a two viewport viewer
        //visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
        //visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
        //For a three viewport viewer
        //visu->createViewPort (0.00, 0.0, 0.33, 1.0, originalViewport);
        //visu->createViewPort (0.33, 0.0, 0.66, 1.0, measurementViewport);
        //visu->createViewPort (0.66, 0.0, 1.00, 1.0, featureViewport);
        /*visu->setBackgroundColor(0.5, 0.5, 0.5);
        visu->setSize(1700, 1000);

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
        visu->addPointCloud(tempCloud, LSystemCloudString, viewport);
        visu->addPolygonMesh(tempMesh, LSystemString, viewport); //Add the temporary mesh for later removal.
    }
    /// Finished setting up the visualizer for viewing.

    pcl::PointCloud<pcl::PointXYZ>::Ptr originalObject (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloudBlob_originalObject;

    logStream << "Loading point cloud from file:\n" << argv[1];
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

    pcl::io::loadPCDFile(argv[1], cloudBlob_originalObject);
    pcl::fromPCLPointCloud2(cloudBlob_originalObject, *originalObject);
    assert(originalObject->size() > 5 && "Input cloud has too few points. Is it empty?"); //5 is an arbitrary value to make sure the cloud has some points.

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(originalObject, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObject, 0.0, 255.0, 0.0), objectString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the loaded point cloud. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    /// As a first attempt, we'll move from the bottom, up and fit single phytomer L-systems to the cloud.
    /// I think the best way to do this will be to fit a cylinder the the very bottom of the plant (i.e., no leaves)
    ///     and keep moving up incrementally until a large number of points don't fit with the cylinder (i.e., a leaf is found).

    /// Assumptions:
    ///     Plant stem is reasonably well aligned with the Z axis (oriented via pot segmentation)

    /// First, get a short portion of the bottom stem (pass through filter) and find a cylinder that fits it (RANSAC).

    /// To get a short portion of the bottom stem, we've move 5% up from the bottom to the top along the z axis.

    float MAGIC_NUMBER_PROPORTION_UP_AABB_TO_LAYER = 0.05;
    BoundingBox boundingBox;
    BoundingBoxMaker<pcl::PointXYZ> boundingBoxMaker;
    AxisAlignedBoundingBox axisAlignedBoundingBox;
    axisAlignedBoundingBox = boundingBoxMaker.returnAxisAlignedBoundingBoxOfCloud(originalObject);
    float distanceToTravelUp = (axisAlignedBoundingBox.maxZ - axisAlignedBoundingBox.minZ) * MAGIC_NUMBER_PROPORTION_UP_AABB_TO_LAYER;

    LOG.DEBUG("Filtering using pass through.", inputParams.debuggingParameters.getDebuggingLevel(), 1);
    //PassThrough filter application
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    passThroughFilter.setInputCloud(originalObject);
    passThroughFilter.setFilterFieldName("z");
    passThroughFilter.setFilterLimits(axisAlignedBoundingBox.minZ - 10, axisAlignedBoundingBox.minZ + distanceToTravelUp);
    passThroughFilter.filter(*cloudFilteredPassThrough);

    cloudFilteredPassThrough->width = cloudFilteredPassThrough->size();
    cloudFilteredPassThrough->height = 1;

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(cloudFilteredPassThrough, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudFilteredPassThrough, 0.0, 255.0, 0.0), objectString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the loaded point cloud. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    /// Now, find a cylinder that fits those points.
    /// RANSAC of stem to find radius. Let's consider modifying this to find multiple stem layers up the stem instead of one large cylinder.
    // These need to be persistent, so they get elevated scope.
    pcl::ModelCoefficients::Ptr cylinderCoefficients (new pcl::ModelCoefficients);
    Eigen::VectorXf vCoefficients(7);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::vector<int> expandedModelInliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // I don't think we should assume that the points have normals at this point, so we'll recalculate them.
    LOG.DEBUG("\tEstimating stem normals with K search with the following parameters.");
    inputParams.normalEstimationParameters.printParameters();

    pcl::PointCloud<pcl::Normal>::Ptr cloudFilteredPassThroughNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
    nest.setSearchMethod(treeNormal);
    nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());
    nest.setInputCloud(cloudFilteredPassThrough);
    nest.compute(*cloudFilteredPassThroughNormals);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (inputParams.sacSegmentationFromNormalsParameters.getMaxIterations());
    seg.setDistanceThreshold (inputParams.sacSegmentationFromNormalsParameters.getDistanceThreshold());
    seg.setNormalDistanceWeight (inputParams.sacSegmentationFromNormalsParameters.getNormalDistanceWeight());
    seg.setRadiusLimits (inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMin(),
                        inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMax());
    seg.setInputCloud(cloudFilteredPassThrough);
    seg.setInputNormals(cloudFilteredPassThroughNormals);

    LOG.DEBUG("\tPerforming segmentation to identify a cylinder corresponding to the stem with the following parameters.");
    inputParams.sacSegmentationFromNormalsParameters.printParameters();
    seg.segment (*inliers, *cylinderCoefficients);

    logStream << "PointCloud after segmentation has " << inliers->indices.size () << " inliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    assert(inliers->indices.size() > 0 && "RANSAC unable to identify a cylinder for the stem. Verify stem and parameters.");

    logStream << "Model coefficients:" << std::endl;
    for (size_t i = 0; i < cylinderCoefficients->values.size(); i++) {
            logStream << cylinderCoefficients->values[i] << std::endl;
            vCoefficients[i] = cylinderCoefficients->values[i];
    }
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    if (vCoefficients[5] < 0) {
        LOG.DEBUG("Z component of cylinder is negative. Flipping cylinder normal for consistency.");
        vCoefficients[3] = -vCoefficients[3];
        vCoefficients[4] = -vCoefficients[4];
        vCoefficients[5] = -vCoefficients[5];
        cylinderCoefficients->values[3] = -cylinderCoefficients->values[3];
        cylinderCoefficients->values[4] = -cylinderCoefficients->values[4];
        cylinderCoefficients->values[5] = -cylinderCoefficients->values[5];
    }

    pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> expandedCylinderModel(cloudFilteredPassThrough);
    expandedCylinderModel.setInputCloud(cloudFilteredPassThrough);
    expandedCylinderModel.setInputNormals(cloudFilteredPassThroughNormals);
    expandedCylinderModel.selectWithinDistance(vCoefficients, inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue(), expandedModelInliers);
    logStream << "Found " << expandedModelInliers.size() << " in expandedModelInliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    pcl::PointIndices::Ptr ptr_expandedModelInliers (new pcl::PointIndices);
    ptr_expandedModelInliers->indices = expandedModelInliers;

    extract.setInputCloud(cloudFilteredPassThrough);
    extract.setIndices(ptr_expandedModelInliers);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(cloud_cylinder, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObject, 0.0, 0.0, 255.0), sceneString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the stem cylinder identified with RANSAC. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    logStream << "stem radius and diameter calculated to be: " << vCoefficients[6] << " and " << vCoefficients[6] * 2;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

    //if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        //pcl::PointXYZ cylinderStartPoint(cylinderCoefficients->values[0] - cylinderCoefficients->values[3] * 100,
        //                                        cylinderCoefficients->values[1] - cylinderCoefficients->values[4] * 100,
        //                                        cylinderCoefficients->values[2] - cylinderCoefficients->values[5] * 100);

        //pcl::PointXYZ cylinderEndPoint(cylinderCoefficients->values[0] + cylinderCoefficients->values[3] * 100,
        //                                        cylinderCoefficients->values[1] + cylinderCoefficients->values[4] * 100,
        //                                        cylinderCoefficients->values[2] + cylinderCoefficients->values[5] * 100);
        //visu->addLine(cylinderStartPoint, cylinderEndPoint, "lineToSpotCheckCylinder1", viewport);
        //if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
        //    LOG.DEBUG("Displaying the next iteration up through point cloud. Press q to continue.");
        //    visu->spin();
        //}
        //else {
        //    visu->spinOnce();
        //}
    //}



    /// So we have a starting cylindrical model (i.e., cylinder radius for the lsystem). Can we iteratively move up the stem until the points outside of it
    /// become significant? Let's work with a donut model, where the "hole" is the stem, and points within a second radius can be considered leaves.

    logStream << "Starting to iteratively move up the plant. There are " << expandedModelInliers.size() <<
                    " inliers in the cylinder model currently, and " << cloudFilteredPassThrough->size() <<
                    " points in the layer.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    /// Here's the important logic loop for constructing the L-System.
    int numberIterations = 1;
    LSystemParameters lsystemParams;
    lsystemParams.setNumberDerivations(0);
    bool phytomerAddedToLSystemInPreviousIteration = false;
    int numberIterationsSinceCylinderRefit = 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsUnacccountedFor (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_AllPointsBeneathCurrentZLevel (new pcl::PointCloud<pcl::PointXYZ>);
    cloudOriginalObjectPointsUnacccountedFor->points = originalObject->points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudStemOnly (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSurroundingStem (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudExpandedStemCylinder (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRemainingPoints (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudStemOnly_AllPointsBeneathCurrentZLevel (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSurroundingStem_AllPointsBeneathCurrentZLevel (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudExpandedStemCylinder_AllPointsBeneathCurrentZLevel (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRemainingPoints_AllPointsBeneathCurrentZLevel (new pcl::PointCloud<pcl::PointXYZ>);

    // While the top of the plant is greater than the current layer.
    float zCoordinateOfCurrentLayer = axisAlignedBoundingBox.minZ + (distanceToTravelUp * numberIterations);
    while (axisAlignedBoundingBox.maxZ >= zCoordinateOfCurrentLayer) {
        cloudStemOnly->clear(); cloudSurroundingStem->clear(); cloudExpandedStemCylinder->clear(); cloudRemainingPoints->clear();
        numberIterations = numberIterations + 1;
        numberIterationsSinceCylinderRefit = numberIterationsSinceCylinderRefit + 1;
        zCoordinateOfCurrentLayer = axisAlignedBoundingBox.minZ + (distanceToTravelUp * numberIterations);
        logStream << "On iteration " << numberIterations << " at z-coordinate layer of " << zCoordinateOfCurrentLayer;
        logStream << ". " << numberIterationsSinceCylinderRefit << " iterations have occurred since the cylinder has been fit again.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        // First, get points below the current level.
        passThroughFilter.setInputCloud(cloudOriginalObjectPointsUnacccountedFor);
        passThroughFilter.setFilterFieldName("z");
        passThroughFilter.setFilterLimits(axisAlignedBoundingBox.minZ, zCoordinateOfCurrentLayer);
        passThroughFilter.filter(*cloudFilteredPassThrough);

        // Save this one for fitting the L-System later on.
        passThroughFilter.setInputCloud(originalObject);
        passThroughFilter.setFilterFieldName("z");
        passThroughFilter.setFilterLimits(axisAlignedBoundingBox.minZ, zCoordinateOfCurrentLayer);
        passThroughFilter.filter(*cloud_AllPointsBeneathCurrentZLevel);

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(cloudFilteredPassThrough, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudFilteredPassThrough, 0.0, 255.0, 0.0), objectString);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the next iteration up through point cloud. Press q to continue.");
                visu->spin();
            }
            else {
                visu->spinOnce();
            }
        }
        // Next, remove points that are too far away from the shoot cylinder. This lets us locate stem-leaf junctions for L-system construction more easily.

        // First, get the largest cylinder as a cloud.
        // Then, take the negative and positve of the smaller cylinder.
        identifyCylinderCloudsBasedOnStemModel(cloudFilteredPassThrough, cloudStemOnly, cloudSurroundingStem, cloudExpandedStemCylinder, cloudRemainingPoints, vCoefficients, visu, inputParams);
        identifyCylinderCloudsBasedOnStemModel(cloud_AllPointsBeneathCurrentZLevel, cloudStemOnly_AllPointsBeneathCurrentZLevel,
                                                cloudSurroundingStem_AllPointsBeneathCurrentZLevel, cloudExpandedStemCylinder_AllPointsBeneathCurrentZLevel,
                                                cloudRemainingPoints_AllPointsBeneathCurrentZLevel, vCoefficients, visu, inputParams);

        // If a phytomer was added in the previous iteration, we need to update things in the cloud and restimate the cylinder model.
        if (phytomerAddedToLSystemInPreviousIteration == true || numberIterationsSinceCylinderRefit >= 3) {
            numberIterationsSinceCylinderRefit = 0;
            LOG.DEBUG("The previous iteration added a phytomer to the L-System or too many iterations have passed since cylinder refit.");
            // If we added a phytomer, we probably need to update the orientation of the stem cylinder.
            LOG.DEBUG("Updating the stem cylinder using the previous phytomer.");
            // We've moved up a little, so take the current layer down to the bottom of 2 phytomers below..
            // First, get points below the current level.
            float bottomOfLayerSlice = 0.0;
            float distanceFromCurrentLayerToTwoDerivationsDown = 0.0;
            if (lsystemParams.getNumberDerivations() <= 2) {
                bottomOfLayerSlice = axisAlignedBoundingBox.minZ; // If there haven't been any L-System phytomers set, use the whole stem.
            }
            else {
                distanceFromCurrentLayerToTwoDerivationsDown = (zCoordinateOfCurrentLayer - (lsystemParams.getInternodeLengths()[lsystemParams.getNumberDerivations() - 3] + lsystemParams.getInternodeLengths()[lsystemParams.getNumberDerivations() - 2]) );
                if (distanceFromCurrentLayerToTwoDerivationsDown > 0 ) {
                    bottomOfLayerSlice = distanceFromCurrentLayerToTwoDerivationsDown;
                }
                else {
                    bottomOfLayerSlice = axisAlignedBoundingBox.minZ;
                }
            }

            logStream << "The z slice being examined for the stem is currently from " << bottomOfLayerSlice << " to " << zCoordinateOfCurrentLayer;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudStemWithinLayer (new pcl::PointCloud<pcl::PointXYZ>);
            passThroughFilter.setInputCloud(cloudStemOnly_AllPointsBeneathCurrentZLevel); // Use all points since some of the stem may have been accounted for already giving RANSAC less to work with.
            passThroughFilter.setFilterFieldName("z");
            passThroughFilter.setFilterLimits(bottomOfLayerSlice, zCoordinateOfCurrentLayer);
            passThroughFilter.filter(*cloudStemWithinLayer);

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                visu->updatePointCloud(cloudStemWithinLayer, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudStemWithinLayer, 255.0, 255.0, 0.0), objectString);
                visu->updatePointCloud(cloudStemWithinLayer, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudStemWithinLayer, 255.0, 255.0, 0.0), sceneString);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Using the yellow points to identify the new cylinder. Press q to continue.");
                    visu->spin();
                }
                else {
                    visu->spinOnce();
                }
            }

            pcl::PointCloud<pcl::Normal>::Ptr cloudStemWithinLayerNormals (new pcl::PointCloud<pcl::Normal>);
            // This normal estimation object was declared above.
            nest.setInputCloud(cloudStemWithinLayer);
            nest.compute(*cloudStemWithinLayerNormals);
            // This RANSAC class object was declared above.
            seg.setInputCloud(cloudStemWithinLayer);
            seg.setInputNormals(cloudStemWithinLayerNormals);
            seg.segment (*inliers, *cylinderCoefficients);

            logStream << "Stem layer after segmentation has " << inliers->indices.size () << " inliers.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            assert(inliers->indices.size() > 0 && "RANSAC unable to identify a cylinder for the stem. Verify stem and parameters.");

            logStream << "Updated cylinder model coefficients:" << std::endl;
            for (size_t i = 0; i < cylinderCoefficients->values.size(); i++) {
                    logStream << cylinderCoefficients->values[i] << std::endl;
                    vCoefficients[i] = cylinderCoefficients->values[i];
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            if (vCoefficients[5] < 0) {
                LOG.DEBUG("Z component of cylinder is negative. Flipping cylinder normal for consistency.");
                vCoefficients[3] = -vCoefficients[3];
                vCoefficients[4] = -vCoefficients[4];
                vCoefficients[5] = -vCoefficients[5];
                cylinderCoefficients->values[3] = -cylinderCoefficients->values[3];
                cylinderCoefficients->values[4] = -cylinderCoefficients->values[4];
                cylinderCoefficients->values[5] = -cylinderCoefficients->values[5];
            }

            ptr_expandedModelInliers->indices = expandedModelInliers;

            extract.setInputCloud(cloudStemWithinLayer);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_cylinder);

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                //pcl::PointXYZ cylinderStartPoint(cylinderCoefficients->values[0] - cylinderCoefficients->values[3] * 100,
                 //                               cylinderCoefficients->values[1] - cylinderCoefficients->values[4] * 100,
                //                                cylinderCoefficients->values[2] - cylinderCoefficients->values[5] * 100);

                //pcl::PointXYZ cylinderEndPoint(cylinderCoefficients->values[0] + cylinderCoefficients->values[3] * 100,
                //                                cylinderCoefficients->values[1] + cylinderCoefficients->values[4] * 100,
                //                                cylinderCoefficients->values[2] + cylinderCoefficients->values[5] * 100);

                visu->updatePointCloud(cloud_cylinder, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObject, 0.0, 0.0, 255.0), sceneString);
                visu->updatePointCloud(cloud_cylinder, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObject, 0.0, 0.0, 255.0), objectString);
                //visu->addLine(cylinderStartPoint, cylinderEndPoint, "lineToSpotCheckCylinder2", viewport);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the stem cylinder identified with RANSAC. Press q to continue.");
                    visu->spin();
                }
                else {
                    visu->spinOnce();
                }
            }


        }
        phytomerAddedToLSystemInPreviousIteration = false;

        // Next, remove points that are too far away from the shoot cylinder. This lets us locate stem-leaf junctions for L-system construction more easily.

        // First, get the largest cylinder as a cloud.
        // Then, take the negative and positve of the smaller cylinder.
        identifyCylinderCloudsBasedOnStemModel(cloudFilteredPassThrough, cloudStemOnly, cloudSurroundingStem, cloudExpandedStemCylinder, cloudRemainingPoints, vCoefficients, visu, inputParams);
        identifyCylinderCloudsBasedOnStemModel(cloud_AllPointsBeneathCurrentZLevel, cloudStemOnly_AllPointsBeneathCurrentZLevel,
                                                cloudSurroundingStem_AllPointsBeneathCurrentZLevel, cloudExpandedStemCylinder_AllPointsBeneathCurrentZLevel,
                                                cloudRemainingPoints_AllPointsBeneathCurrentZLevel, vCoefficients, visu, inputParams);

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(cloudRemainingPoints, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudRemainingPoints, 255.0, 0.0, 0.0), LSystemCloudString);
            visu->updatePointCloud(cloudStemOnly, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudStemOnly, 0.0, 0.0, 255.0), sceneString);
            visu->updatePointCloud(cloudSurroundingStem, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudSurroundingStem, 0.0, 255.0, 0.0), objectString);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the inner stem cylinder as blue, the outer cylinder used to identify leaves as green, and ignored points as red. Press q to continue.");
                visu->spin();
            }
            else {
                visu->spinOnce();
            }
        }
        logStream << "Iteration complete. There are currently " << cloudStemOnly->size() <<
                    " inliers in the stem cylinder model, " << cloudSurroundingStem->size() << " points being examined around the stem, and " <<
                    cloudFilteredPassThrough->size() << " total points in the layer. Checking for what might be a leaf in this layer.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        /// If we've found a set of points that don't belong to the stem, it's likely a leaf emerging.
        /// We can probably save the internode height and radius here.
        /// Then we can figure out the orientation that the leaf is emerging.
        if (cloudSurroundingStem->size() > (cloudSurroundingStem->size() + cloudStemOnly->size()) * .1) {
            LOG.DEBUG("Found what might be a leaf. Examining.");
            /// One common error is that leaf points not accounted for are present in the next layer up (due to angle and curvature). I think
            ///   we can correct for those by only considering points that are adjacent to stem points for adding a phytomer, that is, only consider points
            ///   that might be from leaf emerging from the stem, rather than points that might be from a leaf from a phytomer below that are far away from the stem.
            // So, we need to look at all of the points surrounding the stem, keep the ones that "grow" from the stem, and use them to make a leaf determination.
            // This function takes the stem cloud and the points surrounding it, and returns points that should be considered as leaves. It attempts to clean up points
            // that shouldn't be considered.
            pcl::PointCloud<pcl::PointXYZ>::Ptr pointsToConsiderForLeaves (new pcl::PointCloud<pcl::PointXYZ>);
            identifyPointsOfEmergingLeaves(cloudStemOnly, cloudSurroundingStem, pointsToConsiderForLeaves, inputParams);

            logStream << "Upon further examination, " << pointsToConsiderForLeaves->size() <<
                " points were found that appear to be leaves of the original " << cloudSurroundingStem->size() << " points being examined around the stem.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            // Now that it's been cleaned and updated, check again.
            if (pointsToConsiderForLeaves->size() < (cloudStemOnly->size()) * 0.05) {
                logStream << "Not considering this layer to have a leaf because only " << pointsToConsiderForLeaves->size() << " points were found to be considered for leaves relative to the " <<
                        cloudStemOnly->size() << " points present in the stem cloud. Continuing.";
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                continue;
            }
            else {
                LOG.DEBUG("This layer appears to have a leaf. Proceeding with leaf identification and phytomer addition.");
            }
            cloudSurroundingStem = pointsToConsiderForLeaves;


            /// I think one way to do this, since we expect ~ 180 degree phyllotaxy, is to bisect the stem to isolate a leaf (if more than one
            ///     leaf are appearing), then use orientation of the principal component of the leaf points to orient the L system.
            /// I think we can use the axis of the cylinder as one dimension for the plane, then find the principal component of non cylinder points
            ///     and use an orthogonal normal that is most orthogonal to the cylinder axis.
            std::vector<float> layerMinMaxAvg;
            float layerMin = axisAlignedBoundingBox.minZ + (distanceToTravelUp * (numberIterations - 1));
            float layerMax = axisAlignedBoundingBox.minZ + (distanceToTravelUp * numberIterations);
            float layerAvg = (layerMin + layerMax) / 2.0;
            layerMinMaxAvg.push_back(layerMin);
            layerMinMaxAvg.push_back(layerMax);
            layerMinMaxAvg.push_back(layerAvg);
            logStream << "The min, max, and average of the current z layer are: " << layerMin << ", " << layerMax << ", " << layerAvg;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            LOG.DEBUG("Finding the principal components of the non-cylinder points.");
            pcl::PointCloud<pcl::PointXYZ>::Ptr nonCylinderPoints(new pcl::PointCloud<pcl::PointXYZ>);
            nonCylinderPoints = cloudSurroundingStem;
            logStream << "Found " << nonCylinderPoints->size() << " points that are non-stem cylinderPoints.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PCA<pcl::PointXYZ> pca;
            pca.setInputCloud(nonCylinderPoints);
            pca.project(*nonCylinderPoints, *cloudPCAprojection);

            Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
            Eigen::Vector3f eigenValues = pca.getEigenValues();
            Eigen::Vector4f mean = pca.getMean();
            logStream << "EigenVectors:\n" << eigenVectors << std::endl;
            logStream << "EigenValues:\n" << eigenValues << std::endl;
            logStream << "Mean:\n" << mean;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            /// Try to draw a line that starts halway in the layer z interval and centered at the cylinder center, pointing
            ///     in the direction of the first principal component.
            /// http://math.stackexchange.com/questions/404440/what-is-the-formula-for-a-3d-line
            // (x, y, z) = (x_0, y_0, z_0) + t(a, b, c)
            // x = x_0 + ta
            // y = y_0 + tb
            // z = z_0 + tc
            // x_0 = cylinderCoefficients->values[0]
            // y_0 = cylinderCoefficients->values[1]
            // z_0 = cylinderCoefficients->values[2]
            // a = cylinderCoefficients->values[3]
            // b = cylinderCoefficients->values[4]
            // c = cylinderCoefficients->values[5]

            // We know where in the z dimension we want the line, so first find t.
            // z = z_0 + tc
            float t = (layerAvg - cylinderCoefficients->values[2]) / cylinderCoefficients->values[5];
            float x = cylinderCoefficients->values[0] + t * cylinderCoefficients->values[3];
            float y = cylinderCoefficients->values[1] + t * cylinderCoefficients->values[4];

            pcl::PointXYZ leafPointOrigin(x, y, layerAvg);

            float xEnd = x + (sqrt(eigenValues(0)) * eigenVectors(0,0));
            float yEnd = y + (sqrt(eigenValues(0)) * eigenVectors(1,0));
            float zEnd = layerAvg + (sqrt(eigenValues(0)) * eigenVectors(2,0));
            // If zEnd is below the layerAvg, let's flip the eigenvector for visualization and consistency with downstream processing:
            if (zEnd < layerAvg) {
                eigenVectors(0,0) = eigenVectors(0,0) * -1.0;
                eigenVectors(1,0) = eigenVectors(1,0) * -1.0;
                eigenVectors(2,0) = eigenVectors(2,0) * -1.0;
                xEnd = x + (sqrt(eigenValues(0)) * eigenVectors(0,0));
                yEnd = y + (sqrt(eigenValues(0)) * eigenVectors(1,0));
                zEnd = layerAvg + (sqrt(eigenValues(0)) * eigenVectors(2,0));
            }

            pcl::PointXYZ leafPointEnd(xEnd, yEnd, zEnd);

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                //visu->addPlane(*planeCoefficients, "planeString", viewport);
                srand(time(NULL));
                std::stringstream lineStringNameStream;
                lineStringNameStream << "lineString_" << rand()%10000;
                visu->addLine(leafPointOrigin, leafPointEnd, lineStringNameStream.str(), viewport);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the line approximating the first principal component of the leaf points. Press q to continue.");
                    visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            /// Find a plane that bisects the stem to try and partition the leaf points to one half of the plant. This is to correctly
            /// generate two phytomers if the layer contains two leaves. Plane model parameters contains the x, y, and z of the plane equation in indices 0, 1, and 2. Index 3 contains the intercept.
            // http://stackoverflow.com/questions/15688232/check-which-side-of-a-plane-points-are-on
            Eigen::Vector4f planeModelParameters = findPlaneToBisectStem(eigenVectors, eigenValues, leafPointOrigin, nonCylinderPoints, vCoefficients, inputParams);
            logStream << "The identified plane model was :\n" << planeModelParameters;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            int sideNormal = 0;
            int sideOppositeNormal = 0;
            for (uint32_t i = 0; i < nonCylinderPoints->size(); i ++) {
                pcl::PointXYZ currentPoint = nonCylinderPoints->points[i];
                // index 3 of the planeModelParameters holds the intercept.
                float planeSideResult = (planeModelParameters(0) * currentPoint.x) + (planeModelParameters(1) * currentPoint.y) + (planeModelParameters(2) * currentPoint.z) + planeModelParameters(3); // Index 3 holds the intercept.
                if (planeSideResult > 0) {
                    sideNormal = sideNormal + 1;
                }
                else if (planeSideResult < 0) {
                    sideOppositeNormal = sideOppositeNormal + 1;
                }
            }

            float proportionOnNormalSide = (float)sideNormal / ((float)sideNormal + (float)sideOppositeNormal);
            float proportionOnOppositeNormalSide = (float)sideOppositeNormal / ((float)sideNormal + (float)sideOppositeNormal);
            logStream << "Points on the side of the normal: " << sideNormal << " (" << proportionOnNormalSide << ")" <<  std::endl;
            logStream << "Points on the opposite side of the normal: " << sideOppositeNormal << " (" << proportionOnOppositeNormalSide << ")";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");


            float xPlaneEnd = x + (sqrt(eigenValues(0)) * planeModelParameters(0));
            float yPlaneEnd = y + (sqrt(eigenValues(0))  * planeModelParameters(1));
            float zPlaneEnd = layerAvg + (sqrt(eigenValues(0)) * planeModelParameters(2));
            pcl::PointXYZ leafPointPlaneEnd(xPlaneEnd, yPlaneEnd, zPlaneEnd);

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                srand(time(NULL));
                std::stringstream lineStringNameStream;
                lineStringNameStream << "lineString_" << (rand()%10000 + 1);
                visu->addLine(leafPointOrigin, leafPointPlaneEnd, 1.0, 0.0, 0.0, lineStringNameStream.str(), viewport);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the normal of the plane that bisects the stem and will be used to separate leaf points. Press q to continue.");
                    visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            bool flag_twoDerivationsAddedThisIteration = false;
            /// I think we pretty much have to do everything above again, but once for each set of points if two significant sets of points are found.
            if (proportionOnNormalSide > 0.15 && proportionOnNormalSide < 0.85) {
                flag_twoDerivationsAddedThisIteration = true;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOnNormalSide (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOnOppositeNormalSide (new pcl::PointCloud<pcl::PointXYZ>);
                for (uint32_t i = 0; i < nonCylinderPoints->size(); i ++) {
                    pcl::PointXYZ currentPoint = nonCylinderPoints->points[i];
                    float planeSideResult = (planeModelParameters(0) * currentPoint.x) + (planeModelParameters(1) * currentPoint.y) + (planeModelParameters(2) * currentPoint.z) + planeModelParameters(3); // Index 3 holds the intercept.
                    if (planeSideResult > 0) {
                        cloudPointsOnNormalSide->push_back(currentPoint);
                    }
                    else if (planeSideResult < 0) {
                        cloudPointsOnOppositeNormalSide->push_back(currentPoint);
                    }
                }

                LOG.DEBUG("Finding the principal components for each of the leaf sets.");

                logStream << "There are " << cloudPointsOnNormalSide->size() << " points that are on the side of the plane normal." << std::endl;
                logStream << "There are " << cloudPointsOnOppositeNormalSide->size() << " points that are on the opposite side of the plane normal.";
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojectionOnNormalSide (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojectionOnOppositeOfNormalSide (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PCA<pcl::PointXYZ> pcaForBisection;

                pcaForBisection.setInputCloud(cloudPointsOnNormalSide);
                pcaForBisection.project(*cloudPointsOnNormalSide, *cloudPCAprojectionOnNormalSide);

                Eigen::Matrix3f eigenVectorsForPlaneNormalSide = pcaForBisection.getEigenVectors();
                Eigen::Vector3f eigenValuesForPlaneNormalSide = pcaForBisection.getEigenValues();
                Eigen::Vector4f meanForPlaneNormalSide = pcaForBisection.getMean();
                logStream << "EigenVectors for plane normal side:\n" << eigenVectorsForPlaneNormalSide << std::endl;
                logStream << "EigenValues for plane normal side:\n" << eigenValuesForPlaneNormalSide << std::endl;
                logStream << "Mean for plane normal side:\n" << meanForPlaneNormalSide;
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                pcl::PCA<pcl::PointXYZ> pcaForBisectionOpposite;
                pcaForBisectionOpposite.setInputCloud(cloudPointsOnOppositeNormalSide);
                pcaForBisectionOpposite.project(*cloudPointsOnOppositeNormalSide, *cloudPCAprojectionOnOppositeOfNormalSide);

                Eigen::Matrix3f eigenVectorsForOppositeNormalSide = pcaForBisectionOpposite.getEigenVectors();
                Eigen::Vector3f eigenValuesForOppositeNormalSide = pcaForBisectionOpposite.getEigenValues();
                Eigen::Vector4f meanForOppositeNormalSide = pcaForBisectionOpposite.getMean();
                logStream << "EigenVectors for opposite normal side:\n" << eigenVectorsForOppositeNormalSide << std::endl;
                logStream << "EigenValues for opposite normal side:\n" << eigenValuesForOppositeNormalSide << std::endl;
                logStream << "Mean for opposite normal side:\n" << meanForOppositeNormalSide;
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                /// Try to draw a line that starts halway in the layer z interval and centered at the cylinder center, pointing
                ///     in the direction of the first principal component.
                /// http://math.stackexchange.com/questions/404440/what-is-the-formula-for-a-3d-line
                // (x, y, z) = (x_0, y_0, z_0) + t(a, b, c)
                // x = x_0 + ta
                // y = y_0 + tb
                // z = z_0 + tc
                // x_0 = cylinderCoefficients->values[0]
                // y_0 = cylinderCoefficients->values[1]
                // z_0 = cylinderCoefficients->values[2]
                // a = cylinderCoefficients->values[3]
                // b = cylinderCoefficients->values[4]
                // c = cylinderCoefficients->values[5]

                // We know where in the z dimension we want the line, so first find t.
                // z = z_0 + tc
                float t_bisection = (layerAvg - cylinderCoefficients->values[2]) / cylinderCoefficients->values[5];
                float x_bisection = cylinderCoefficients->values[0] + t_bisection * cylinderCoefficients->values[3];
                float y_bisection = cylinderCoefficients->values[1] + t_bisection * cylinderCoefficients->values[4];

                pcl::PointXYZ leafPointOrigin_bisection(x_bisection, y_bisection, layerAvg);

                float xEndPlaneNormal = x_bisection + (sqrt(eigenValuesForPlaneNormalSide(0)) * eigenVectorsForPlaneNormalSide(0,0));
                float yEndPlaneNormal = y_bisection + (sqrt(eigenValuesForPlaneNormalSide(0)) * eigenVectorsForPlaneNormalSide(1,0));
                float zEndPlaneNormal = layerAvg + (sqrt(eigenValuesForPlaneNormalSide(0)) * eigenVectorsForPlaneNormalSide(2,0));
                // If zEnd is below the layerAvg, let's flip the eigenvector for visualization and consistency with downstream processing:
                if (zEndPlaneNormal < layerAvg) {
                    eigenVectorsForPlaneNormalSide(0,0) = eigenVectorsForPlaneNormalSide(0,0) * -1.0;
                    eigenVectorsForPlaneNormalSide(1,0) = eigenVectorsForPlaneNormalSide(1,0) * -1.0;
                    eigenVectorsForPlaneNormalSide(2,0) = eigenVectorsForPlaneNormalSide(2,0) * -1.0;
                    xEndPlaneNormal = x + (sqrt(eigenValuesForPlaneNormalSide(0)) * eigenVectorsForPlaneNormalSide(0,0));
                    yEndPlaneNormal = y + (sqrt(eigenValuesForPlaneNormalSide(0)) * eigenVectorsForPlaneNormalSide(1,0));
                    zEndPlaneNormal = layerAvg + (sqrt(eigenValuesForPlaneNormalSide(0)) * eigenVectorsForPlaneNormalSide(2,0));
                }
                pcl::PointXYZ leafPointEndPlaneNormalSide(xEndPlaneNormal, yEndPlaneNormal, zEndPlaneNormal);

                float xEndOppositePlaneNormal = x_bisection + (sqrt(eigenValuesForOppositeNormalSide(0)) * eigenVectorsForOppositeNormalSide(0,0));
                float yEndOppositePlaneNormal = y_bisection + (sqrt(eigenValuesForOppositeNormalSide(0)) * eigenVectorsForOppositeNormalSide(1,0));
                float zEndOppositePlaneNormal = layerAvg + (sqrt(eigenValuesForOppositeNormalSide(0)) * eigenVectorsForOppositeNormalSide(2,0));
                // If zEnd is below the layerAvg, let's flip the eigenvector for visualization and consistency with downstream processing:
                if (zEndOppositePlaneNormal < layerAvg) {
                    eigenVectorsForOppositeNormalSide(0,0) = eigenVectorsForOppositeNormalSide(0,0) * -1.0;
                    eigenVectorsForOppositeNormalSide(1,0) = eigenVectorsForOppositeNormalSide(1,0) * -1.0;
                    eigenVectorsForOppositeNormalSide(2,0) = eigenVectorsForOppositeNormalSide(2,0) * -1.0;
                    xEndOppositePlaneNormal = x_bisection + (sqrt(eigenValuesForOppositeNormalSide(0)) * eigenVectorsForOppositeNormalSide(0,0));
                    yEndOppositePlaneNormal = y_bisection + (sqrt(eigenValuesForOppositeNormalSide(0)) * eigenVectorsForOppositeNormalSide(1,0));
                    zEndOppositePlaneNormal = layerAvg + (sqrt(eigenValuesForOppositeNormalSide(0)) * eigenVectorsForOppositeNormalSide(2,0));
                }

                pcl::PointXYZ leafPointEndOppositePlaneNormal(xEndOppositePlaneNormal, yEndOppositePlaneNormal, zEndOppositePlaneNormal);

                if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                    //visu->addPlane(*planeCoefficients, "planeString", viewport);
                    srand(time(NULL));
                    std::stringstream lineStringNameStream;
                    lineStringNameStream << "lineStringBisection_" << rand()%10000;
                    visu->addLine(leafPointOrigin_bisection, leafPointEndPlaneNormalSide, 0.0, 0.5, 0.5, lineStringNameStream.str(), viewport);
                    visu->updatePointCloud(cloudPointsOnNormalSide, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudPointsOnNormalSide, 0.0, 255.0, 255.0), objectString);
                    lineStringNameStream << "lineStringBisection_" << rand()%10000 + 3;
                    visu->addLine(leafPointOrigin_bisection, leafPointEndOppositePlaneNormal, 0.5, 0.0, 0.5, lineStringNameStream.str(), viewport);
                    visu->updatePointCloud(cloudPointsOnOppositeNormalSide, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudPointsOnOppositeNormalSide, 255.0, 0.0, 255.0), sceneString);
                    if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                        LOG.DEBUG("Displaying the line approximating the leaf points. Cyan is plane normal, purple is opposite. Press q to continue.");
                        visu->spin();
                    }
                }
                else {
                    visu->spinOnce();
                }

                /// Maybe what we really want is to use the x and the y of the leaf projection, and fix the z to find a plane.
                // http://mathworld.wolfram.com/Plane.html

                float xPlaneEndPlaneNormalSide = x + (sqrt(eigenValuesForPlaneNormalSide(0)) * eigenVectorsForPlaneNormalSide(0,0));
                float yPlaneEndPlaneNormalSide = y + (sqrt(eigenValuesForPlaneNormalSide(0))  * eigenVectorsForPlaneNormalSide(1,0));
                float zPlaneEndPlaneNormalSide = layerAvg + (sqrt(eigenValuesForPlaneNormalSide(0)) * eigenVectorsForPlaneNormalSide(2,0) * 0.0);
                pcl::PointXYZ leafPointPlaneEndPlaneNormalSide(xPlaneEndPlaneNormalSide, yPlaneEndPlaneNormalSide, zPlaneEndPlaneNormalSide);

                float xPlaneEndOppositePlaneNormal = x + (sqrt(eigenValuesForOppositeNormalSide(0)) * eigenVectorsForOppositeNormalSide(0,0));
                float yPlaneEndOppositePlaneNormal = y + (sqrt(eigenValuesForOppositeNormalSide(0))  * eigenVectorsForOppositeNormalSide(1,0));
                float zPlaneEndOppositePlaneNormal = layerAvg + (sqrt(eigenValuesForOppositeNormalSide(0)) * eigenVectorsForOppositeNormalSide(2,0) * 0.0);
                pcl::PointXYZ leafPointPlaneEndOppositePlaneNormal(xPlaneEndOppositePlaneNormal , yPlaneEndOppositePlaneNormal , zPlaneEndOppositePlaneNormal );


                if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                    srand(time(NULL));
                    std::stringstream lineStringNameStream;
                    lineStringNameStream << "lineStringBisection_" << (rand()%10000 + 4);
                    visu->addLine(leafPointOrigin, leafPointPlaneEndPlaneNormalSide, 1.0, 1.0, 0.0, lineStringNameStream.str(), viewport);
                    lineStringNameStream << "lineStringBisection_" << (rand()%10000 + 5);
                    visu->addLine(leafPointOrigin, leafPointPlaneEndOppositePlaneNormal, 1.0, 0.25, 1.0, lineStringNameStream.str(), viewport);
                    if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                        LOG.DEBUG("Displaying an orthogonal line. Press q to continue.");
                        visu->spin();
                    }
                }
                else {
                    visu->spinOnce();
                }
                /// Now I think we have enough data to build a rough L-system. We have estimations of:
                ///     how tall the internode is.
                ///     the radius of the internode.
                ///     the phyllotaxy of the leaf with respect to the internode.
                ///     the angle of emergence of the leaf with respect to the internode.
                /// Now let's build an L-system from that.
                // Add these parameters to the l system.
                // First, add the one with the smallest mean z coordinate.
                // lsystemParams.setNumberDerivations(lsystemParams.getNumberDerivations() + 1);
                if (meanForPlaneNormalSide[2] > meanForOppositeNormalSide[2]) {
                    LOG.DEBUG("The mean of the side with the plane normal was larger. Treating the side opposite the normal as the first phytomer.");
                    //process opposite normal side as phytomer 1.
                    /// Internode lengths. Each phytomer will be different.
                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> internodeLengths;
                        internodeLengths.push_back(meanForOppositeNormalSide[2] - axisAlignedBoundingBox.minZ );
                        internodeLengths.push_back(meanForPlaneNormalSide[2] - meanForOppositeNormalSide[2]);
                        lsystemParams.setInternodeLengths(internodeLengths);
                    }
                    else {
                        // Use height up to the previous derivation. Note that we haven't incremented the derivation yet.
                        float approximationOfTotalHeight = layerAvg - axisAlignedBoundingBox.minZ;
                        float sumOfInternodesToCurrent = 0.0;
                        for (uint32_t derivationIterator = 1; derivationIterator <= lsystemParams.getNumberDerivations(); derivationIterator++) {
                            sumOfInternodesToCurrent =  sumOfInternodesToCurrent + lsystemParams.getInternodeLengths()[derivationIterator - 1]; // vector is 0 indexed.
                        }
                        logStream << "Adding internode of length " << approximationOfTotalHeight - sumOfInternodesToCurrent << ". The total height approximation is " <<
                            approximationOfTotalHeight << " and the sum of the internodes to current is " << sumOfInternodesToCurrent << ".";
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                        lsystemParams.appendInternodeLength(meanForOppositeNormalSide[2] - sumOfInternodesToCurrent);
                        lsystemParams.appendInternodeLength(meanForPlaneNormalSide[2] - meanForOppositeNormalSide[2]);
                    }

                    /// Internode radii. Both phytomers will be equivalent.
                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> internodeRadii;
                        internodeRadii.push_back(vCoefficients[6]);
                        internodeRadii.push_back(vCoefficients[6]);
                        lsystemParams.setInternodeRadii(internodeRadii);
                    }
                    else {
                        lsystemParams.appendInternodeRadius(vCoefficients[6]);
                        lsystemParams.appendInternodeRadius(vCoefficients[6]);
                    }

                    /// Stem turn and pitch. The second phytomer will be 0, 0
                    Eigen::Vector3f cylinderNormalForTurnAndPitch(vCoefficients[3], vCoefficients[4], vCoefficients[5]);
                    std::pair<float, float> turnAndPitch = returnTurnAndPitchAnglesToMoveZAxisToNormal(cylinderNormalForTurnAndPitch, inputParams);

                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> internodePitchAngles;
                        std::vector<float> internodeTurnAngles;
                        internodeTurnAngles.push_back(turnAndPitch.first);
                        internodeTurnAngles.push_back(0.0);
                        internodePitchAngles.push_back(turnAndPitch.second);
                        internodePitchAngles.push_back(0.0);
                        lsystemParams.setInternodeTurnAngles(internodeTurnAngles);
                        lsystemParams.setInternodePitchAngles(internodePitchAngles);
                    }
                    else {
                        // To reset, need to check the derivation below (considering that the current derivation hasn't been added yet), and add its negative.
                        float previousLSystemTurnAngle = 0.0;
                        float previousLSystemPitchAngle = 0.0;
                        // if the derivation below is 0, we need to check the one below it.
                        bool flag_reachedPhytomerWithNonZeroPitchAndTurnAngles = false;
                        int derivationCounter = 1; // Start this counter at 1 since derivations are 1 based but the vectors are zero based.
                        while (flag_reachedPhytomerWithNonZeroPitchAndTurnAngles == false && (lsystemParams.getNumberDerivations() - derivationCounter) >= 0) {
                            logStream << "Checking index of " << (lsystemParams.getNumberDerivations() - derivationCounter) << " for turn and pitch.";
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            previousLSystemTurnAngle = lsystemParams.getInternodeTurnAngles()[lsystemParams.getNumberDerivations() - derivationCounter];
                            previousLSystemPitchAngle = lsystemParams.getInternodePitchAngles()[lsystemParams.getNumberDerivations() - derivationCounter];
                            // Since we're dealing with floats, just make sure the values aren't too close to zero.
                            if (previousLSystemTurnAngle < -0.001 || previousLSystemTurnAngle > 0.001) {
                                if (previousLSystemPitchAngle < -0.001 || previousLSystemPitchAngle > 0.001) {
                                    flag_reachedPhytomerWithNonZeroPitchAndTurnAngles = true;
                                    LOG.DEBUG("Setting flag for reached nonzero pitch and turn to true.");
                                }
                            }
                            if (flag_reachedPhytomerWithNonZeroPitchAndTurnAngles == false) {
                                previousLSystemTurnAngle = 0.0;
                                previousLSystemPitchAngle = 0.0;
                            }
                            derivationCounter = derivationCounter + 1;
                        }

                        logStream << "While setting the internode pitch and turn angles, resetting based on previous turn and pitch of: " << previousLSystemTurnAngle << " and " << previousLSystemPitchAngle;
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                        lsystemParams.appendInternodeTurnAngle(turnAndPitch.first + (previousLSystemTurnAngle * -1.0));
                        lsystemParams.appendInternodeTurnAngle(0.0);
                        lsystemParams.appendInternodePitchAngle(turnAndPitch.second + (previousLSystemPitchAngle * -1.0));
                        lsystemParams.appendInternodePitchAngle(0.0);
                    }

                    /// Leaf widths. Set to the same as a default for now.
                    // For now, just set it to a default width.
                    float leafWidth = 30.0;
                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> leafWidths;
                        leafWidths.push_back(leafWidth);
                        leafWidths.push_back(leafWidth);
                        lsystemParams.setLeafWidths(leafWidths);
                    }
                    else {
                        lsystemParams.appendLeafWidth(leafWidth);
                        lsystemParams.appendLeafWidth(leafWidth);
                    }

                    /// Leaf phyllotaxy angle.
                    Eigen::Vector3f leafPrincipalComponentPlaneNormalSide(eigenVectorsForPlaneNormalSide(0,0), eigenVectorsForPlaneNormalSide(1,0), eigenVectorsForPlaneNormalSide(2,0));
                    Eigen::Vector3f leafPrincipalComponentOppositePlaneNormal(eigenVectorsForOppositeNormalSide(0,0), eigenVectorsForOppositeNormalSide(1,0), eigenVectorsForOppositeNormalSide(2,0));
                    float leafPhyllotaxyAnglePlaneNormalSide = returnLeafPhyllotaxyAngleToMoveXAxisToNormal(cylinderNormalForTurnAndPitch, leafPrincipalComponentPlaneNormalSide, inputParams);
                    float leafPhyllotaxyAngleOppositePlaneNormal = returnLeafPhyllotaxyAngleToMoveXAxisToNormal(cylinderNormalForTurnAndPitch, leafPrincipalComponentOppositePlaneNormal, inputParams);

                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> leafPhyllotaxyAngles;
                        leafPhyllotaxyAngles.push_back(leafPhyllotaxyAngleOppositePlaneNormal);
                        leafPhyllotaxyAngles.push_back(leafPhyllotaxyAnglePlaneNormalSide);
                        lsystemParams.setLeafPhyllotaxyAngles(leafPhyllotaxyAngles);
                    }
                    else {
                        lsystemParams.appendLeafPhyllotaxyAngle(leafPhyllotaxyAngleOppositePlaneNormal);
                        lsystemParams.appendLeafPhyllotaxyAngle(leafPhyllotaxyAnglePlaneNormalSide);
                    }

                    /// Leaf curvature.
                    // Can we also use the leaf principal component to define the first part of the leaf curvature?
                    // As a first approximation, let's say that the second curvature point of the leaf is 100 units greater than the unit vector.
                    // Let's also say the leaf point origin is equal to the mean of the leaf PCA.
                    pcl::PointXYZ leafPointMeanOppositePlaneNormal(meanForOppositeNormalSide[0], meanForOppositeNormalSide[1], meanForOppositeNormalSide[2]);
                    pcl::PointXYZ leafPointMeanPlaneNormalSide(meanForPlaneNormalSide[0], meanForPlaneNormalSide[1], meanForPlaneNormalSide[2]);
                    std::pair<float,float> leafCurveOppositePlaneNormal_P1(0.0, 0.0);
                    std::pair<float,float> leafCurveOppositePlaneNormal_P2 = returnSecondLeafCurvatureControlPointEstimate(leafPointMeanOppositePlaneNormal, leafPointPlaneEndOppositePlaneNormal, inputParams);
                    std::pair<float,float> leafCurveOppositePlaneNormal_P3(120.0, 130.0);
                    std::pair<float,float> leafCurveOppositePlaneNormal_P4(200.0, 80.0);
                    std::vector< std::pair<float,float> > leafCurveOppositePlaneNormal = {leafCurveOppositePlaneNormal_P1, leafCurveOppositePlaneNormal_P2, leafCurveOppositePlaneNormal_P3, leafCurveOppositePlaneNormal_P4};

                    std::pair<float,float> leafCurvePlaneNormalSide_P1(0.0, 0.0);
                    std::pair<float,float> leafCurvePlaneNormalSide_P2 = returnSecondLeafCurvatureControlPointEstimate(leafPointMeanPlaneNormalSide, leafPointPlaneEndPlaneNormalSide, inputParams);
                    std::pair<float,float> leafCurvePlaneNormalSide_P3(120.0, 130.0);
                    std::pair<float,float> leafCurvePlaneNormalSide_P4(200.0, 80.0);
                    std::vector< std::pair<float,float> > leafCurvePlaneNormalSide = {leafCurvePlaneNormalSide_P1, leafCurvePlaneNormalSide_P2, leafCurvePlaneNormalSide_P3, leafCurvePlaneNormalSide_P4};

                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector< std::vector < std::pair<float, float> > > leafCurvatures;
                        leafCurvatures.push_back(leafCurveOppositePlaneNormal);
                        leafCurvatures.push_back(leafCurvePlaneNormalSide);
                        lsystemParams.setLeafCurvatures(leafCurvatures);
                    }
                    else {
                        lsystemParams.appendLeafCurvature(leafCurveOppositePlaneNormal);
                        lsystemParams.appendLeafCurvature(leafCurvePlaneNormalSide);

                    }


                    lsystemParams.setNumberDerivations(lsystemParams.getNumberDerivations() + 2);
                }
                else {
                    LOG.DEBUG("The mean of the side with the plane normal was smaller. Treating the side with the normal as the first phytomer.");
                    /// Process normal side first.
                    //process opposite normal side as phytomer 1.
                    /// Internode lengths.
                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> internodeLengths;
                        internodeLengths.push_back(meanForPlaneNormalSide[2] - axisAlignedBoundingBox.minZ );
                        internodeLengths.push_back(meanForOppositeNormalSide[2] - meanForPlaneNormalSide[2]);
                        lsystemParams.setInternodeLengths(internodeLengths);
                    }
                    else {
                        // Use height up to the previous derivation. Note that we haven't incremented the derivation yet.
                        float approximationOfTotalHeight = layerAvg - axisAlignedBoundingBox.minZ;
                        float sumOfInternodesToCurrent = 0.0;
                        for (uint32_t derivationIterator = 1; derivationIterator <= lsystemParams.getNumberDerivations(); derivationIterator++) {
                            sumOfInternodesToCurrent =  sumOfInternodesToCurrent + lsystemParams.getInternodeLengths()[derivationIterator - 1]; // vector is 0 indexed.
                        }
                        logStream << "Adding internode of length " << approximationOfTotalHeight - sumOfInternodesToCurrent << ". The total height approximation is " <<
                            approximationOfTotalHeight << " and the sum of the internodes to current is " << sumOfInternodesToCurrent << ".";
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                        lsystemParams.appendInternodeLength(meanForPlaneNormalSide[2] - sumOfInternodesToCurrent);
                        lsystemParams.appendInternodeLength(meanForOppositeNormalSide[2] - meanForPlaneNormalSide[2]);
                    }

                    /// Internode radii. Both phytomers will be equivalent.
                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> internodeRadii;
                        internodeRadii.push_back(vCoefficients[6]);
                        internodeRadii.push_back(vCoefficients[6]);
                        lsystemParams.setInternodeRadii(internodeRadii);
                    }
                    else {
                        lsystemParams.appendInternodeRadius(vCoefficients[6]);
                        lsystemParams.appendInternodeRadius(vCoefficients[6]);
                    }

                    /// Stem turn and pitch. The second phytomer will be 0, 0
                    Eigen::Vector3f cylinderNormalForTurnAndPitch(vCoefficients[3], vCoefficients[4], vCoefficients[5]);
                    std::pair<float, float> turnAndPitch = returnTurnAndPitchAnglesToMoveZAxisToNormal(cylinderNormalForTurnAndPitch, inputParams);

                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> internodePitchAngles;
                        std::vector<float> internodeTurnAngles;
                        internodeTurnAngles.push_back(turnAndPitch.first);
                        internodeTurnAngles.push_back(0.0);
                        internodePitchAngles.push_back(turnAndPitch.second);
                        internodePitchAngles.push_back(0.0);
                        lsystemParams.setInternodeTurnAngles(internodeTurnAngles);
                        lsystemParams.setInternodePitchAngles(internodePitchAngles);
                    }
                    else {
                        // To reset, need to check the derivation below (considering that the current derivation hasn't been added yet), and add its negative.
                        float previousLSystemTurnAngle = 0.0;
                        float previousLSystemPitchAngle = 0.0;
                        // if the derivation below is 0, we need to check the one below it.
                        bool flag_reachedPhytomerWithNonZeroPitchAndTurnAngles = false;
                        int derivationCounter = 1; // Start this counter at 1 since derivations are 1 based but the vectors are zero based.
                        while (flag_reachedPhytomerWithNonZeroPitchAndTurnAngles == false && (lsystemParams.getNumberDerivations() - derivationCounter) >= 0) {
                            logStream << "Checking index of " << (lsystemParams.getNumberDerivations() - derivationCounter) << " for turn and pitch.";
                            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                            previousLSystemTurnAngle = lsystemParams.getInternodeTurnAngles()[lsystemParams.getNumberDerivations() - derivationCounter];
                            previousLSystemPitchAngle = lsystemParams.getInternodePitchAngles()[lsystemParams.getNumberDerivations() - derivationCounter];
                            // Since we're dealing with floats, just make sure the values aren't too close to zero.
                            if (previousLSystemTurnAngle < -0.001 || previousLSystemTurnAngle > 0.001) {
                                if (previousLSystemPitchAngle < -0.001 || previousLSystemPitchAngle > 0.001) {
                                    flag_reachedPhytomerWithNonZeroPitchAndTurnAngles = true;
                                    LOG.DEBUG("Setting flag for reached nonzero pitch and turn to true.");
                                }
                            }
                            if (flag_reachedPhytomerWithNonZeroPitchAndTurnAngles == false) {
                                previousLSystemTurnAngle = 0.0;
                                previousLSystemPitchAngle = 0.0;
                            }
                            derivationCounter = derivationCounter + 1;
                        }

                        logStream << "While setting the internode pitch and turn angles, resetting based on previous turn and pitch of: " << previousLSystemTurnAngle << " and " << previousLSystemPitchAngle;
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                        lsystemParams.appendInternodeTurnAngle(turnAndPitch.first + (previousLSystemTurnAngle * -1.0));
                        lsystemParams.appendInternodeTurnAngle(0.0);
                        lsystemParams.appendInternodePitchAngle(turnAndPitch.second + (previousLSystemPitchAngle * -1.0));
                        lsystemParams.appendInternodePitchAngle(0.0);
                    }

                    /// Leaf widths. Set to the same as a default for now.
                    // For now, just set it to a default width.
                    float leafWidth = 30.0;
                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> leafWidths;
                        leafWidths.push_back(leafWidth);
                        leafWidths.push_back(leafWidth);
                        lsystemParams.setLeafWidths(leafWidths);
                    }
                    else {
                        lsystemParams.appendLeafWidth(leafWidth);
                        lsystemParams.appendLeafWidth(leafWidth);
                    }

                    /// Leaf phyllotaxy angle.
                    Eigen::Vector3f leafPrincipalComponentPlaneNormalSide(eigenVectorsForPlaneNormalSide(0,0), eigenVectorsForPlaneNormalSide(1,0), eigenVectorsForPlaneNormalSide(2,0));
                    Eigen::Vector3f leafPrincipalComponentOppositePlaneNormal(eigenVectorsForOppositeNormalSide(0,0), eigenVectorsForOppositeNormalSide(1,0), eigenVectorsForOppositeNormalSide(2,0));
                    float leafPhyllotaxyAnglePlaneNormalSide = returnLeafPhyllotaxyAngleToMoveXAxisToNormal(cylinderNormalForTurnAndPitch, leafPrincipalComponentPlaneNormalSide, inputParams);
                    float leafPhyllotaxyAngleOppositePlaneNormal = returnLeafPhyllotaxyAngleToMoveXAxisToNormal(cylinderNormalForTurnAndPitch, leafPrincipalComponentOppositePlaneNormal, inputParams);

                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector<float> leafPhyllotaxyAngles;
                        leafPhyllotaxyAngles.push_back(leafPhyllotaxyAnglePlaneNormalSide);
                        leafPhyllotaxyAngles.push_back(leafPhyllotaxyAngleOppositePlaneNormal);
                        lsystemParams.setLeafPhyllotaxyAngles(leafPhyllotaxyAngles);
                    }
                    else {
                        lsystemParams.appendLeafPhyllotaxyAngle(leafPhyllotaxyAnglePlaneNormalSide);
                        lsystemParams.appendLeafPhyllotaxyAngle(leafPhyllotaxyAngleOppositePlaneNormal);
                    }

                    /// Leaf curvature.
                    // Can we also use the leaf principal component to define the first part of the leaf curvature?
                    // As a first approximation, let's say that the second curvature point of the leaf is 100 units greater than the unit vector.
                    // Let's also say the leaf point origin is equal to the mean of the leaf PCA.
                    pcl::PointXYZ leafPointMeanOppositePlaneNormal(meanForOppositeNormalSide[0], meanForOppositeNormalSide[1], meanForOppositeNormalSide[2]);
                    pcl::PointXYZ leafPointMeanPlaneNormalSide(meanForPlaneNormalSide[0], meanForPlaneNormalSide[1], meanForPlaneNormalSide[2]);
                    std::pair<float,float> leafCurveOppositePlaneNormal_P1(0.0, 0.0);
                    std::pair<float,float> leafCurveOppositePlaneNormal_P2 = returnSecondLeafCurvatureControlPointEstimate(leafPointMeanOppositePlaneNormal, leafPointPlaneEndOppositePlaneNormal, inputParams);
                    std::pair<float,float> leafCurveOppositePlaneNormal_P3(120.0, 130.0);
                    std::pair<float,float> leafCurveOppositePlaneNormal_P4(200.0, 80.0);
                    std::vector< std::pair<float,float> > leafCurveOppositePlaneNormal = {leafCurveOppositePlaneNormal_P1, leafCurveOppositePlaneNormal_P2, leafCurveOppositePlaneNormal_P3, leafCurveOppositePlaneNormal_P4};

                    std::pair<float,float> leafCurvePlaneNormalSide_P1(0.0, 0.0);
                    std::pair<float,float> leafCurvePlaneNormalSide_P2 = returnSecondLeafCurvatureControlPointEstimate(leafPointMeanPlaneNormalSide, leafPointPlaneEndPlaneNormalSide, inputParams);
                    std::pair<float,float> leafCurvePlaneNormalSide_P3(120.0, 130.0);
                    std::pair<float,float> leafCurvePlaneNormalSide_P4(200.0, 80.0);
                    std::vector< std::pair<float,float> > leafCurvePlaneNormalSide = {leafCurvePlaneNormalSide_P1, leafCurvePlaneNormalSide_P2, leafCurvePlaneNormalSide_P3, leafCurvePlaneNormalSide_P4};

                    if (lsystemParams.getNumberDerivations() == 0) {
                        std::vector< std::vector < std::pair<float, float> > > leafCurvatures;
                        leafCurvatures.push_back(leafCurvePlaneNormalSide);
                        leafCurvatures.push_back(leafCurveOppositePlaneNormal);
                        lsystemParams.setLeafCurvatures(leafCurvatures);
                    }
                    else {
                        lsystemParams.appendLeafCurvature(leafCurvePlaneNormalSide);
                        lsystemParams.appendLeafCurvature(leafCurveOppositePlaneNormal);

                    }


                    lsystemParams.setNumberDerivations(lsystemParams.getNumberDerivations() + 2);
                }

            } // End if there is a significant bisection.
            else { // else there is no significant bisection.

                // First, internode height is going to be layerAvg - axisAlignedBoundingBox.minZ , though this only works for the first internode.

                /// Internode lengths.
                if (lsystemParams.getNumberDerivations() == 0) {
                    std::vector<float> internodeLengths;
                    internodeLengths.push_back(layerAvg - axisAlignedBoundingBox.minZ );
                    lsystemParams.setInternodeLengths(internodeLengths);
                }
                else {
                    // Use height up to the previous derivation. Note that we haven't incremented the derivation yet.
                    float approximationOfTotalHeight = layerAvg - axisAlignedBoundingBox.minZ;
                    float sumOfInternodesToCurrent = 0.0;
                    for (uint32_t derivationIterator = 1; derivationIterator <= lsystemParams.getNumberDerivations(); derivationIterator++) {
                        sumOfInternodesToCurrent =  sumOfInternodesToCurrent + lsystemParams.getInternodeLengths()[derivationIterator - 1]; // vector is 0 indexed.
                    }
                    logStream << "Adding internode of length " << approximationOfTotalHeight - sumOfInternodesToCurrent << ". The total height approximation is " <<
                            approximationOfTotalHeight << " and the sum of the internodes to current is " << sumOfInternodesToCurrent << ".";
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    lsystemParams.appendInternodeLength(approximationOfTotalHeight - sumOfInternodesToCurrent);
                }

                /// Internode radii.
                if (lsystemParams.getNumberDerivations() == 0) {
                    std::vector<float> internodeRadii;
                    internodeRadii.push_back(vCoefficients[6]);
                    lsystemParams.setInternodeRadii(internodeRadii);
                }
                else {
                    lsystemParams.appendInternodeRadius(vCoefficients[6]);
                }

                /// Stem turn and pitch.
                // Turn and pitch is retained across derivations because of the turtle.
                // As such, need to "reset" every derivation.
                Eigen::Vector3f cylinderNormalForTurnAndPitch(vCoefficients[3], vCoefficients[4], vCoefficients[5]);
                std::pair<float, float> turnAndPitch = returnTurnAndPitchAnglesToMoveZAxisToNormal(cylinderNormalForTurnAndPitch, inputParams);

                if (lsystemParams.getNumberDerivations() == 0) {
                    std::vector<float> internodePitchAngles;
                    std::vector<float> internodeTurnAngles;
                    internodeTurnAngles.push_back(turnAndPitch.first);
                    internodePitchAngles.push_back(turnAndPitch.second);
                    lsystemParams.setInternodeTurnAngles(internodeTurnAngles);
                    lsystemParams.setInternodePitchAngles(internodePitchAngles);
                }
                else {
                    // To reset, need to check the derivation below (considering that the current derivation hasn't been added yet), and add its negative.
                    float previousLSystemTurnAngle = 0.0;
                    float previousLSystemPitchAngle = 0.0;
                    // if the derivation below is 0, we need to check the one below it.
                    bool flag_reachedPhytomerWithNonZeroPitchAndTurnAngles = false;
                    int derivationCounter = 1; // Start this counter at 1 since derivations are 1 based but the vectors are zero based.
                    while (flag_reachedPhytomerWithNonZeroPitchAndTurnAngles == false && (lsystemParams.getNumberDerivations() - derivationCounter) >= 0) {
                        logStream << "Checking index of " << (lsystemParams.getNumberDerivations() - derivationCounter) << " for turn and pitch.";
                        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                        previousLSystemTurnAngle = lsystemParams.getInternodeTurnAngles()[lsystemParams.getNumberDerivations() - derivationCounter];
                        previousLSystemPitchAngle = lsystemParams.getInternodePitchAngles()[lsystemParams.getNumberDerivations() - derivationCounter];
                        // Since we're dealing with floats, just make sure the values aren't too close to zero.
                        if (previousLSystemTurnAngle < -0.001 || previousLSystemTurnAngle > 0.001) {
                            if (previousLSystemPitchAngle < -0.001 || previousLSystemPitchAngle > 0.001) {
                                flag_reachedPhytomerWithNonZeroPitchAndTurnAngles = true;
                                LOG.DEBUG("Setting flag for reached nonzero pitch and turn to true.");
                            }
                        }
                        if (flag_reachedPhytomerWithNonZeroPitchAndTurnAngles == false) {
                            previousLSystemTurnAngle = 0.0;
                            previousLSystemPitchAngle = 0.0;
                        }
                        derivationCounter = derivationCounter + 1;
                    }
                    logStream << "While setting the internode pitch and turn angles, resetting based on previous turn and pitch of: " << previousLSystemTurnAngle << " and " << previousLSystemPitchAngle;
                    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                    lsystemParams.appendInternodeTurnAngle(turnAndPitch.first + (previousLSystemTurnAngle * -1.0));
                    lsystemParams.appendInternodePitchAngle(turnAndPitch.second + (previousLSystemPitchAngle * -1.0));
                }

                /// Leaf widths.
                // For now, just set it to a default width.
                float leafWidth = 30.0;
                if (lsystemParams.getNumberDerivations() == 0) {
                    std::vector<float> leafWidths;
                    leafWidths.push_back(leafWidth);
                    lsystemParams.setLeafWidths(leafWidths);
                }
                else {
                    lsystemParams.appendLeafWidth(leafWidth);
                }
                /// Leaf phyllotaxy angle.
                Eigen::Vector3f leafPrincipalComponent(eigenVectors(0,0), eigenVectors(1,0), eigenVectors(2,0) );
                float leafPhyllotaxyAngle = returnLeafPhyllotaxyAngleToMoveXAxisToNormal(cylinderNormalForTurnAndPitch, leafPrincipalComponent, inputParams);

                if (lsystemParams.getNumberDerivations() == 0) {
                    std::vector<float> leafPhyllotaxyAngles;
                    leafPhyllotaxyAngles.push_back(leafPhyllotaxyAngle);
                    lsystemParams.setLeafPhyllotaxyAngles(leafPhyllotaxyAngles);
                }
                else {
                    lsystemParams.appendLeafPhyllotaxyAngle(leafPhyllotaxyAngle);
                }

                /// Leaf curvature.
                // Can we also use the leaf principal component to define the first part of the leaf curvature?
                // As a first approximation, let's say that the second curvature point of the leaf is 100 units greater than the unit vector.
                std::pair<float,float> leafCurveFirst_P1(0.0, 0.0);
                std::pair<float,float> leafCurveFirst_P2 = returnSecondLeafCurvatureControlPointEstimate(leafPointOrigin, leafPointPlaneEnd, inputParams);
                std::pair<float,float> leafCurveFirst_P3(120.0, 130.0);
                std::pair<float,float> leafCurveFirst_P4(200.0, 80.0);
                std::vector< std::pair<float,float> > leafCurveFirst = {leafCurveFirst_P1, leafCurveFirst_P2, leafCurveFirst_P3, leafCurveFirst_P4};

                if (lsystemParams.getNumberDerivations() == 0) {
                    std::vector< std::vector < std::pair<float, float> > > leafCurvatures;
                    leafCurvatures.push_back(leafCurveFirst);
                    lsystemParams.setLeafCurvatures(leafCurvatures);
                }
                else {
                    lsystemParams.appendLeafCurvature(leafCurveFirst);
                }

                lsystemParams.setNumberDerivations(lsystemParams.getNumberDerivations() + 1);


            }// End else there is no significant bisection.

            std::vector<int> phytomersToRefine = {1, 2};
            if (flag_twoDerivationsAddedThisIteration == true) {
                // If only two derivations, just do 1 and 2 since they are the only ones that are present.
                if ( lsystemParams.getNumberDerivations() <= 2) {
                    phytomersToRefine.clear();
                    phytomersToRefine.push_back(lsystemParams.getNumberDerivations() - 1);
                    phytomersToRefine.push_back(lsystemParams.getNumberDerivations());
                }
                // Else need to do all three. Since the most bottom derivation will have already been refined once, do it last.
                else {
                    phytomersToRefine.clear();
                    phytomersToRefine.push_back(lsystemParams.getNumberDerivations() - 1);
                    phytomersToRefine.push_back(lsystemParams.getNumberDerivations());
                    phytomersToRefine.push_back(lsystemParams.getNumberDerivations() - 2);
                }
            }
            else { // else if only one derivation added, only check the first derivation if only one present, else check the current and the one below it.
                if ( lsystemParams.getNumberDerivations() <= 1) {
                    phytomersToRefine.clear();
                    phytomersToRefine.push_back(lsystemParams.getNumberDerivations());
                }
                else { //refine the bottom derivation last since it will have already been refined once.
                    phytomersToRefine.clear();
                    phytomersToRefine.push_back(lsystemParams.getNumberDerivations());
                    phytomersToRefine.push_back(lsystemParams.getNumberDerivations() - 1);

                }
            }

            /// 04/26/16 Resume from here; work on making sure the cylinder gets identified correctly. Derivation 4 appears to be having problems.
            bool temporaryDebuggingFlag = true;
            //bool temporaryDebuggingFlag = false;
            int skipRefiningDerivationsAtOrBelowThisValue = 3;
            /// To fast forward when testing things, can set up a fixed L-System here.
            if (temporaryDebuggingFlag == true && lsystemParams.getNumberDerivations() <= skipRefiningDerivationsAtOrBelowThisValue) {
                LOG.DEBUG("\t\tNot refining LSystem parameters this iteration.");
                if (lsystemParams.getNumberDerivations() <= 2) {
                    lsystemParams = returnPredefinedLSystem_debugSystemOne_twoPhytomers();
                }
                else if (lsystemParams.getNumberDerivations() <= 3) {
                    lsystemParams = returnPredefinedLSystem_debugSystemOne_threePhytomers();
                }
            }
            else {
                LSystemParameters refinedParameters =  refineLSystemParallel_ExpandedStem_InternodeLength(pythonFunctionForLSystemConstruction, lsystemParams, cloudExpandedStemCylinder_AllPointsBeneathCurrentZLevel, phytomersToRefine, visu, inputParams);
                refinedParameters = refineLSystemParallel_ExpandedStem_LeafPhyllotaxyAngle(pythonFunctionForLSystemConstruction, refinedParameters, cloudExpandedStemCylinder_AllPointsBeneathCurrentZLevel, phytomersToRefine, visu, inputParams);
                refinedParameters = refineLSystemParallel_ExpandedStem_LeafCurvaturesControlPointTwo(pythonFunctionForLSystemConstruction, refinedParameters, cloudExpandedStemCylinder_AllPointsBeneathCurrentZLevel, phytomersToRefine, visu, inputParams);
                lsystemParams = refinedParameters;
            }

            pcl::PolygonMesh LSystemMesh = buildLSystem(pythonFunctionForLSystemConstruction, lsystemParams, visu, inputParams);


            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                //visu->updatePolygonMesh(LSystemMesh, LSystemString);
                visu->removePolygonMesh(LSystemString);
                visu->addPolygonMesh(LSystemMesh, LSystemString, viewport);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the polygon mesh generated from the L-system after refinement. Press q to continue.");
                visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }


            pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystem (new pcl::PointCloud<pcl::PointXYZ>);
            /// Now we need to get a point cloud sampled from the mesh of the L-system to create what a point cloud generated from that L-system might look like.
            sampleMeshToPointCloud(&LSystemMesh, *sampledLSystem, inputParams);

            logStream << "Received a cloud with: " << sampledLSystem->size() << " points after sampling.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");


            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                visu->removePolygonMesh(LSystemString);
                visu->updatePointCloud(sampledLSystem, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(sampledLSystem, 255.0, 0.0, 255.0), LSystemCloudString);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Removed the polygon mesh, and displaying the cloud sampled from the L-System. Press q to continue.");
                visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

                /// Now I guess we can fit this L-system and make minor refinements to things like internode height, leaf phyllotaxy, and the first
                ///     two control points (really, only the second control point since the first is always (0, 0). I don't think we'd want to refine
                ///     either of the other two control points since we might not have the full leaf yet.

                /// This will allow us to assign points as accounted for or not by the L-System.
                /// Iterative closest point seems as good as anything else for the time being, though really we should be able to just transform
                /// the Lsystem to the RANSAC cylinder of the stem and get pretty close?

            pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
            //registerLSystemICP(sampledLSystem, cloudExpandedStemCylinder, registeredLSystemPointCloud, visu, inputParams);
            registerLSystemICP(sampledLSystem, cloudExpandedStemCylinder_AllPointsBeneathCurrentZLevel, registeredLSystemPointCloud, visu, inputParams);

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                visu->updatePointCloud(registeredLSystemPointCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(registeredLSystemPointCloud, 255.0, 0.0, 255.0), LSystemCloudString);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the cloud sampled from the L-System registered to the target. Press q to continue.");
                visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            // I think we also need to be a bit more "greedy" than just correspondences. A nearest neighbor approach may work.
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsAcccounted (new pcl::PointCloud<pcl::PointXYZ>);
            cloudOriginalObjectPointsUnacccountedFor->clear();
            kdtree.setInputCloud(registeredLSystemPointCloud);

            for (uint32_t i = 0; i < originalObject->points.size(); i++) {
                pcl::PointXYZ currentPoint = originalObject->points[i];
                std::vector<int> pointIndicesRadiusSearch;
                std::vector<float> distanceRadiusSearch;
                /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
                /// This number determines the radius away from the L-System points that points in the original cloud will be considered accounted for in the L-system.
                int numberNeighbors = kdtree.radiusSearch(currentPoint, 30.0, pointIndicesRadiusSearch, distanceRadiusSearch);
                if (numberNeighbors >= 1) {
                    cloudOriginalObjectPointsAcccounted->points.push_back(currentPoint);
                }
                else {
                    cloudOriginalObjectPointsUnacccountedFor->points.push_back(currentPoint);
                }
            }

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                visu->updatePointCloud(cloudOriginalObjectPointsAcccounted, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudOriginalObjectPointsAcccounted, 0.0, 0.0, 255.0), LSystemCloudString);
                visu->updatePointCloud(registeredLSystemPointCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(registeredLSystemPointCloud, 255.0, 0.0, 255.0), objectString);
                visu->updatePointCloud(cloudOriginalObjectPointsUnacccountedFor, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudOriginalObjectPointsUnacccountedFor, 0.0, 255.0, 0.0), sceneString);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the points in the L-system in purple, the points accounted for by the L-system in blue, and unaccounted points in green. Press q to continue.");
                    visu->spin();
                }
                else {
                    visu->spinOnce();
                }
                visu->updatePointCloud(tempViewerCloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(tempViewerCloud, 0.0, 0.0, 255.0), LSystemCloudString);
            }

            phytomerAddedToLSystemInPreviousIteration = true;

/*
            pcl::CorrespondencesPtr correspondencesBetweenLSystemAndTarget = registerPointCloudsICPAndReturnCorrespondences(sampledLSystem, cloudExpandedStemCylinder, visu, inputParams);
            pcl::PointIndices::Ptr indicesOfPointsInTargetWithCorrespondence (new pcl::PointIndices);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRemainingAfterLSystemFit (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsInTargetWithCorrespondence (new pcl::PointCloud<pcl::PointXYZ>);
            std::map<uint32_t, uint32_t> mapOfCorrespondences;
            for (uint32_t i = 0; i < correspondencesBetweenLSystemAndTarget->size(); i++) {
                pcl::Correspondence currentCorrespondence = (*correspondencesBetweenLSystemAndTarget)[i];
                //logStream << "From outside the class. Index of the source point: " << currentCorrespondence.index_query << std::endl;
                //logStream << "Index of the matching target point: " << currentCorrespondence.index_match << std::endl;
                //logStream << "Distance between the corresponding points: " << currentCorrespondence.distance << std::endl;
                //logStream << "Weight of the confidence in the correspondence: " << currentCorrespondence.weight;
                //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                //srand(time(NULL));
                //std::stringstream lineStringNameStream;
                //lineStringNameStream << "lineString_" << rand()%10000 + i;
                //visu->addLine(sampledLSystem->points[currentCorrespondence.index_query], cloudFilteredPassThrough->points[currentCorrespondence.index_match], lineStringNameStream.str());
                if (mapOfCorrespondences.find(currentCorrespondence.index_match) == mapOfCorrespondences.end() ) {
                    indicesOfPointsInTargetWithCorrespondence->indices.push_back(currentCorrespondence.index_match);
                    mapOfCorrespondences.insert(std::pair<uint32_t, uint32_t>(currentCorrespondence.index_match, 1));
                    cloudPointsInTargetWithCorrespondence->points.push_back(cloudExpandedStemCylinder->points[currentCorrsepondence.index_match]);
                }
                else {
                    continue;
                }
            }


            //extract.setInputCloud(cloudFilteredPassThrough);
            //extract.setIndices(indicesOfPointsInTargetWithCorrespondence);
            //extract.setNegative(true);
            extract.filter(*cloudRemainingAfterLSystemFit);
            logStream << "Found " << cloudRemainingAfterLSystemFit->size() << " points remaining (i.e. do not correspond to points in the LSystem.)";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");


            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                visu->updatePointCloud(cloudRemainingAfterLSystemFit, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudRemainingAfterLSystemFit, 0.0, 255.0, 0.0), sceneString);
                visu->updatePointCloud(cloudRemainingAfterLSystemFit, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudRemainingAfterLSystemFit, 0.0, 255.0, 0.0), objectString);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the cloud that remains after removing L-system points. Press q to continue.");
                visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            /// At this point, I think we'll have a decent L-system for the first phytomer.
            ///     The next step is to move up one and add a phytomer, taking into consideration the current L-system and the points that remain after fitting it.
*/

        }
    }

    Py_Finalize();
    return(0);
}



pcl::PolygonMesh buildLSystem(PyObject *inputLpyFunction, LSystemParameters inputLSystemParams, pcl::visualization::PCLVisualizer *visu, InputParameters inputParams) {
    std::ostringstream logStream;
    std::string objectString = "object";
    int viewport = 0;
    LOG.DEBUG("Building L system given parameters.");

    std::vector<float> internodeLengths = inputLSystemParams.getInternodeLengths();
    std::vector<float> internodeRadii = inputLSystemParams.getInternodeRadii();
    std::vector<float> internodePitchAngles = inputLSystemParams.getInternodePitchAngles();
    std::vector<float> internodeTurnAngles = inputLSystemParams.getInternodeTurnAngles();

    std::vector<float> leafPhyllotaxyAngles = inputLSystemParams.getLeafPhyllotaxyAngles();
    std::vector<float> leafWidths = inputLSystemParams.getLeafWidths();
    std::vector< std::vector< std::pair <float, float> > > leafCurvatures = inputLSystemParams.getLeafCurvatures();

    // Check that everything is still okay with the embedded Python interpreter.
    assert(Py_IsInitialized() > 0 && "Python interpreter has not been initialized. Aborting." );
    assert(inputLpyFunction && PyCallable_Check(inputLpyFunction) && "The Python function for creating L-systems was not found. Aborting.");

    // Convert the passed data into Python objects.
    PyObject *pInternodeLengths = PyList_New(0);
    for (uint32_t i = 0; i < internodeLengths.size(); i++) {
        PyList_Append(pInternodeLengths, Py_BuildValue("f", internodeLengths[i]) );
    }

    PyObject *pInternodeRadii = PyList_New(0);
    for (uint32_t i = 0; i < internodeRadii.size(); i++) {
        PyList_Append(pInternodeRadii, Py_BuildValue("f", internodeRadii[i]) );
    }

    PyObject *pInternodeTurnAngles = PyList_New(0);
    for (uint32_t i = 0; i < internodeTurnAngles.size(); i++) {
        PyList_Append(pInternodeTurnAngles, Py_BuildValue("f", internodeTurnAngles[i]) );
    }

    PyObject *pInternodePitchAngles = PyList_New(0);
    for (uint32_t i = 0; i < internodePitchAngles.size(); i++) {
        PyList_Append(pInternodePitchAngles, Py_BuildValue("f", internodePitchAngles[i]) );
    }

    PyObject *pLeafPhyllotaxyAngles = PyList_New(0);
    for (uint32_t i = 0; i < leafPhyllotaxyAngles.size(); i++) {
        PyList_Append(pLeafPhyllotaxyAngles, Py_BuildValue("f", leafPhyllotaxyAngles[i]) );
    }

    PyObject *pLeafWidths = PyList_New(0);
    for (uint32_t i = 0; i < leafWidths.size(); i++) {
        PyList_Append(pLeafWidths, Py_BuildValue("f", leafWidths[i]) );
    }

    PyObject *pLeafCurvatures = PyList_New(0);
    for (uint32_t i = 0; i < leafCurvatures.size(); i++) {
        PyList_Append(pLeafCurvatures, Py_BuildValue("[(ffi), (ffi), (ffi), (ffi)]", leafCurvatures[i][0].first, leafCurvatures[i][0].second, 1,
                                                                                       leafCurvatures[i][1].first, leafCurvatures[i][1].second, 1,
                                                                                       leafCurvatures[i][2].first, leafCurvatures[i][2].second, 1,
                                                                                       leafCurvatures[i][3].first, leafCurvatures[i][3].second, 1));
    }
    //PyObject *pLeafWidths = Py_BuildValue("i, f, f", 0.564, 23, 33);
    //PyObject *pLeafCurve = Py_BuildValue("[(ffi), (ffi), (ffi), (ffi)]", -0.33, 0.95, 1, -0.272718, 1.30435, 1, -0.092, 1.39, 1, 0.099, 1.33, 1);
    std::vector<size_t> vectorOfSizes = {internodeLengths.size(), internodeRadii.size(), internodePitchAngles.size(), internodeTurnAngles.size(),
                                            leafPhyllotaxyAngles.size(), leafWidths.size(), leafCurvatures.size()};

    size_t maximumNumberDerivations = *std::min_element(vectorOfSizes.begin(), vectorOfSizes.end());

    if (inputLSystemParams.getNumberDerivations() != maximumNumberDerivations) {
        LOG.DEBUG("WARNING:");
        LOG.DEBUG("WARNING: The number of derivations requested by the L-System is different than the amount of data available. The assigned number of derivations will");
        LOG.DEBUG("WARNING:\t be based on the minimum length of all data vectors.");
        LOG.DEBUG("WARNING:");
    }

    logStream << "Proceeding with " << maximumNumberDerivations << " derivations of the L-System.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

    PyObject *pNumberDerivations = PyInt_FromSize_t(maximumNumberDerivations);

    // Set up the input to the python function.
    PyObject *pArgTuple = PyTuple_New(8);
    PyTuple_SetItem(pArgTuple, 0, pNumberDerivations);
    PyTuple_SetItem(pArgTuple, 1, pInternodeLengths);
    PyTuple_SetItem(pArgTuple, 2, pInternodeRadii);
    PyTuple_SetItem(pArgTuple, 3, pInternodePitchAngles);
    PyTuple_SetItem(pArgTuple, 4, pInternodeTurnAngles);
    PyTuple_SetItem(pArgTuple, 5, pLeafPhyllotaxyAngles);
    PyTuple_SetItem(pArgTuple, 6, pLeafWidths);
    PyTuple_SetItem(pArgTuple, 7, pLeafCurvatures);

    // Call the python function to construct the L-system and return the mesh.
    PyObject *pReturnedValue = PyObject_CallObject(inputLpyFunction, pArgTuple);

    Py_DECREF(pInternodeLengths);
    Py_DECREF(pInternodeRadii);
    Py_DECREF(pInternodePitchAngles);
    Py_DECREF(pInternodeTurnAngles);
    Py_DECREF(pLeafWidths);
    Py_DECREF(pLeafCurvatures);
    Py_DECREF(pArgTuple);

    int numberOfPoints = 0;
    int numberOfFaces = 0;
    Py_ssize_t pSizeOfReturnedList = PyList_Size(pReturnedValue);
    Py_ssize_t minIndexOfPoints = 2;
    Py_ssize_t maxIndexOfPoints = 0;
    Py_ssize_t minIndexOfFaces = 0;
    Py_ssize_t maxIndexOfFaces = pSizeOfReturnedList;

    // Process the returned mesh and convert it to a point cloud.
    if (pSizeOfReturnedList > 2) {
        numberOfPoints = PyInt_AsLong(PyList_GetItem(pReturnedValue, 0));
        LOG.DEBUG("Number of points in returned mesh:");
        logStream << numberOfPoints;
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

        numberOfFaces = PyInt_AsLong(PyList_GetItem(pReturnedValue, 1));
        LOG.DEBUG("Number of faces in returned mesh:");
        logStream << numberOfFaces;
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

            maxIndexOfPoints = minIndexOfPoints + numberOfPoints - 1;
            minIndexOfFaces = minIndexOfPoints + numberOfPoints;
            maxIndexOfFaces = minIndexOfFaces + numberOfFaces - 1;
    }

    // Construct a point cloud from the points.
    pcl::PointCloud<pcl::PointXYZ>::Ptr initialLSystemCloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (Py_ssize_t i = minIndexOfPoints; i <= maxIndexOfPoints; i++) {
        pcl::PointXYZ currentPoint(PyFloat_AsDouble(PyList_GetItem(PyList_GetItem(pReturnedValue, i), 0)),
                                    PyFloat_AsDouble(PyList_GetItem(PyList_GetItem(pReturnedValue, i), 1)),
                                    PyFloat_AsDouble(PyList_GetItem(PyList_GetItem(pReturnedValue, i), 2)));
        initialLSystemCloud->points.push_back(currentPoint);
        //logStream << "Point: " << currentPoint;
        //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    }

    // Construct the faces from the connected edges.
    std::vector<pcl::Vertices> initialLSystemFaces;
    for (Py_ssize_t i = minIndexOfFaces; i <= maxIndexOfFaces; i++) {
        pcl::Vertices triangleFaceIndices;
        triangleFaceIndices.vertices.push_back(PyInt_AsSsize_t(PyList_GetItem(PyList_GetItem(pReturnedValue, i), 0)));
        triangleFaceIndices.vertices.push_back(PyInt_AsSsize_t(PyList_GetItem(PyList_GetItem(pReturnedValue, i), 1)));
        triangleFaceIndices.vertices.push_back(PyInt_AsSsize_t(PyList_GetItem(PyList_GetItem(pReturnedValue, i), 2)));
        initialLSystemFaces.push_back(triangleFaceIndices);

        //logStream << "Face: " << PyInt_AsSsize_t(PyList_GetItem(PyList_GetItem(pReturnedValue, i), 0)) << " " <<
           //              PyInt_AsSsize_t(PyList_GetItem(PyList_GetItem(pReturnedValue, i), 1)) << " " <<
           //              PyInt_AsSsize_t(PyList_GetItem(PyList_GetItem(pReturnedValue, i), 2));
          //               LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

    }


    Py_DECREF(pReturnedValue);

    pcl::PolygonMesh initialLSystemMesh;
    pcl::PCLPointCloud2 pc2_meshCloud;
    pcl::toPCLPointCloud2(*initialLSystemCloud, pc2_meshCloud);
    initialLSystemMesh.cloud = pc2_meshCloud;
    initialLSystemMesh.polygons = initialLSystemFaces;

    return(initialLSystemMesh);
}

/// This function is currently a mess and needs to be cleaned up. The mess is mostly a consequence of my lack of understanding regarding
///     vector rotations. The clutter will be kept until enough test cases have been run to be convincing that this generally works.
std::pair<float, float> returnTurnAndPitchAnglesToMoveZAxisToNormal(Eigen::Vector3f vInputNormal, InputParameters inputParams) {
    std::ostringstream logStream;
    std::pair<float, float> anglePairToReturn_TurnAndPitch(99.99, 99.99);

    //vCoefficients [3], [4], and [5] should be the normal of the cylinder. We need to find how much to rotate the axis of the
    // l system to approximate the cylinder. We should only need to rotate around the x and y, and I think we can treat as 2 dimensional vectors.
    Eigen::Vector3f vectorUpXaxis(1, 0, 0);
    Eigen::Vector3f vectorUpYaxis(0, 1, 0);
    Eigen::Vector3f vectorUpZaxis(0, 0, 1);
    //Eigen::Vector3f vectorOfCylinder(vCoefficients[3], vCoefficients[4],  0);
    //Eigen::Vector3f vectorOfCylinder(0.7071, 0.7071, 0);
    Eigen::Vector3f vectorOfCylinder(0, vInputNormal[1], vInputNormal[2]); //(0, -vCoefficients[4],  -vCoefficients[5]);

    // Make the cylinder vector a unit vector.
    logStream << "vectorOfCylinder prior to normalization:\n" << vectorOfCylinder;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    vectorOfCylinder.normalize();
    logStream << "vectorOfCylinder after normalization:\n" << vectorOfCylinder;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    /// These unit vectors can be used to test calculations.
    //Eigen::Vector3f vectorOfCylinder(0.7071, 0.7071, 0);
    //Eigen::Vector3f vectorOfCylinder(0.7071, -0.7071, 0);
    //Eigen::Vector3f vectorOfCylinder(-0.7071, 0.7071, 0);
    //Eigen::Vector3f vectorOfCylinder(-0.7071, -0.7071, 0);
    //Eigen::Vector3f vectorOfCylinder(0.13381, 0.0376869, 0);

    // http://www.euclideanspace.com/maths/algebra/vectors/angleBetween/
    // angle = acos(v1•v2)
            logStream << "acos of dot product of x axis and cylinder is: " << acos(vectorUpXaxis.dot(vectorOfCylinder)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of y axis and cylinder is: " << acos(vectorUpYaxis.dot(vectorOfCylinder)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of z axis and cylinder is: " << acos(vectorUpZaxis.dot(vectorOfCylinder)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of cylinder and x axis: " << acos(vectorOfCylinder.dot(vectorUpXaxis)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of cylinder and y axis: " << acos(vectorOfCylinder.dot(vectorUpYaxis)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of cylinder and z axis: " << acos(vectorOfCylinder.dot(vectorUpZaxis)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            //logStream << "atan2 of x axis and cylinder is: " << (atan2(vectorUpXaxis[1], vectorUpXaxis[0]) - atan2(vectorOfCylinder[1], vectorOfCylinder[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of y axis and cylinder is: " << (atan2(vectorUpYaxis[1], vectorUpYaxis[0]) - atan2(vectorOfCylinder[1], vectorOfCylinder[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of z axis and cylinder is: " << (atan2(vectorUpZaxis[1], vectorUpZaxis[0]) - atan2(vectorOfCylinder[1], vectorOfCylinder[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of cylinder and x axis: " << (atan2(vectorOfCylinder[1], vectorOfCylinder[0]) - atan2(vectorUpXaxis[1], vectorUpXaxis[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of cylinder and y axis: " << (atan2(vectorOfCylinder[1], vectorOfCylinder[0]) - atan2(vectorUpYaxis[1], vectorUpYaxis[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of cylinder and z axis: " << (atan2(vectorOfCylinder[1], vectorOfCylinder[0]) - atan2(vectorUpZaxis[1], vectorUpZaxis[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            logStream << "atan2 of z axis and cylinder is: " << (atan2(vectorUpZaxis[2], vectorUpZaxis[1]) - atan2(vectorOfCylinder[2], vectorOfCylinder[1])) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            /// We need to find one rotation first, then the second since it's dependent on the first.
            /// Rotation matrix about the x axis: https://en.wikipedia.org/wiki/Rotation_matrix
            Eigen::Matrix3f firstRotationMatrix = Eigen::Matrix3f::Identity();
            firstRotationMatrix(0,0) = 1;
            firstRotationMatrix(0,1) = 0;
            firstRotationMatrix(0,2) = 0;
            firstRotationMatrix(1,0) = 0;
            firstRotationMatrix(1,1) = cos((atan2(vectorUpZaxis[2], vectorUpZaxis[1]) - atan2(vectorOfCylinder[2], vectorOfCylinder[1]))); //cos(acos(vectorUpZaxis.dot(vectorOfCylinder)));
            firstRotationMatrix(1,2) = sin((atan2(vectorUpZaxis[2], vectorUpZaxis[1]) - atan2(vectorOfCylinder[2], vectorOfCylinder[1]))); //sin(acos(vectorUpZaxis.dot(vectorOfCylinder)));
            firstRotationMatrix(2,0) = 0;
            firstRotationMatrix(2,1) = -sin((atan2(vectorUpZaxis[2], vectorUpZaxis[1]) - atan2(vectorOfCylinder[2], vectorOfCylinder[1]))); //-sin(acos(vectorUpZaxis.dot(vectorOfCylinder)));
            firstRotationMatrix(2,2) = cos((atan2(vectorUpZaxis[2], vectorUpZaxis[1]) - atan2(vectorOfCylinder[2], vectorOfCylinder[1]))); //cos(acos(vectorUpZaxis.dot(vectorOfCylinder)));

            //Eigen::Vector3f initialRotationProduct = firstRotationMatrix * vectorOfCylinder;

            //logStream << "firstRotationMatrix\n" << firstRotationMatrix;
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            //logStream << "initialRotationProduct\n" << initialRotationProduct;
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            Eigen::Vector3f initialRotationProduct(vInputNormal[0], 0, vInputNormal[2]); //-vCoefficients[3], 0, -vCoefficients[5]);
            logStream << "initialRotationProduct prior to normalization:\n" << initialRotationProduct;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            vectorOfCylinder.normalize();
            logStream << "initialRotationProduct after normalization:\n" << initialRotationProduct;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            logStream << "acos of dot product of x axis and firstRotationProduct is: " << acos(vectorUpXaxis.dot(initialRotationProduct)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of y axis and firstRotationProduct is: " << acos(vectorUpYaxis.dot(initialRotationProduct)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of z axis and firstRotationProduct is: " << acos(vectorUpZaxis.dot(initialRotationProduct)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of firstRotationProduct and x axis: " << acos(initialRotationProduct.dot(vectorUpXaxis)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of firstRotationProduct and y axis: " << acos(initialRotationProduct.dot(vectorUpYaxis)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            logStream << "acos of dot product of firstRotationProduct and z axis: " << acos(initialRotationProduct.dot(vectorUpZaxis)) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            //logStream << "atan2 of x axis and firstRotationProduct is: " << (atan2(vectorUpXaxis[1], vectorUpXaxis[0]) - atan2(initialRotationProduct[1], initialRotationProduct[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of y axis and firstRotationProduct is: " << (atan2(vectorUpYaxis[1], vectorUpYaxis[0]) - atan2(initialRotationProduct[1], initialRotationProduct[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of z axis and firstRotationProduct is: " << (atan2(vectorUpZaxis[1], vectorUpZaxis[0]) - atan2(initialRotationProduct[1], initialRotationProduct[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of firstRotationProduct and x axis: " << (atan2(initialRotationProduct[1], initialRotationProduct[0]) - atan2(vectorUpXaxis[1], vectorUpXaxis[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of firstRotationProduct and y axis: " << (atan2(initialRotationProduct[1], initialRotationProduct[0]) - atan2(vectorUpYaxis[1], vectorUpYaxis[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            //logStream << "atan2 of firstRotationProduct and z axis: " << (atan2(initialRotationProduct[1], initialRotationProduct[0]) - atan2(vectorUpZaxis[1], vectorUpZaxis[0])) * (180.0 / M_PI);
            //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            logStream << "atan2 of firstRotationProduct and z axis: " << (atan2(vectorUpZaxis[2], vectorUpZaxis[0]) - atan2(initialRotationProduct[2], initialRotationProduct[0])) * (180.0 / M_PI);
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");



            //internodePitchAngles.push_back( (atan2(vectorOfCylinder[1], vectorOfCylinder[0]) - atan2(vectorUpXaxis[1], vectorUpXaxis[0])) * (180.0 / M_PI));
            //internodeTurnAngles.push_back( (atan2(vectorOfCylinder[1], vectorOfCylinder[0]) - atan2(vectorUpYaxis[1], vectorUpYaxis[0])) * (180.0 / M_PI));
            //internodePitchAngles.push_back( acos(vectorUpZaxis.dot(vectorOfCylinder)) * (180.0 / M_PI));
            //internodeTurnAngles.push_back(  acos(vectorUpZaxis.dot(initialRotationProduct)) * (180.0 / M_PI));

            //internodeTurnAngles.push_back( acos(vectorUpZaxis.dot(vectorOfCylinder)) * (180.0 / M_PI));
            //internodeTurnAngles.push_back( (atan2(vectorUpZaxis[2], vectorUpZaxis[1]) - atan2(vectorOfCylinder[2], vectorOfCylinder[1])) * (180.0 / M_PI) );
            //internodePitchAngles.push_back( (atan2(vectorUpZaxis[2], vectorUpZaxis[0]) - atan2(initialRotationProduct[2], initialRotationProduct[0])) * (180.0 / M_PI));

            anglePairToReturn_TurnAndPitch.first = (atan2(vectorUpZaxis[2], vectorUpZaxis[1]) - atan2(vectorOfCylinder[2], vectorOfCylinder[1])) * (180.0 / M_PI);
            anglePairToReturn_TurnAndPitch.second = (atan2(vectorUpZaxis[2], vectorUpZaxis[0]) - atan2(initialRotationProduct[2], initialRotationProduct[0])) * (180.0 / M_PI);
    return anglePairToReturn_TurnAndPitch;
}

/// This function is also a mess; for now we just find phyllotaxy based on the normal of the plane for the leaf.
/// For the leaf phyllotaxy angle, I suppose we can align the cylinder to the z axis, use the rotation matrix from that alignment, apply it to the
/// leaf plane normal, then find the angle between the leaf plane normal and the x axis, and use that as the leaf phyllotaxy angle. This
/// doesn't seem to work in application since we don't have any guarantee where the plane axis will end up.
/// It can be fixed once a better solution is identified.
/// and by better solution, one that keeps the L system as close of a transformation as possible to the original cloud (ideally just a translation).
float returnLeafPhyllotaxyAngleToMoveXAxisToNormal(Eigen::Vector3f vInputCylinderAxisNormal,  Eigen::Vector3f vInputPlaneNormal, InputParameters inputParams) {
    std::ostringstream logStream;
    float phyllotaxyAngleRelativeToXAxis = 99.99;
    LOG.DEBUG("Finding phyllotaxy angle.");

    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPotTransformed (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointXYZ modelAxisPoint(vCircleCoefficients[0], vCircleCoefficients[1], vCircleCoefficients[2]);
    //Eigen::Vector3f modelVectorAxisPoint(vCircleCoefficients[0], vCircleCoefficients[1], vCircleCoefficients[2]);
    //Eigen::Vector3f vectorOrigin(0, 0, 1);
    //Eigen::Vector3f vectorModelNormal(vCircleCoefficients[4], vCircleCoefficients[5], vCircleCoefficients[6]);
    //pcl::PointXYZ modelVectorNormal(vCircleCoefficients[4], vCircleCoefficients[5], vCircleCoefficients[6]);

    Eigen::Vector3f vectorUpXaxis(1, 0, 0);
    Eigen::Vector3f vectorUpYaxis(0, 1, 0);
    Eigen::Vector3f vectorUpZaxis(0, 0, 1);

    Eigen::Vector3f vector_A = vInputCylinderAxisNormal;
    Eigen::Vector3f vector_B = vectorUpXaxis;

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

    logStream << "Prior to rotation, the cylinder normal is :\n" << vInputCylinderAxisNormal << std::endl;
    logStream << "Prior to rotation, the plane normal is:\n" << vInputPlaneNormal << std::endl;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    // Now we multiply the rotation matrix by the plane normal to align it with the z axis.
    Eigen::Vector3f rotatedCylinderNormal = matrix_R * vInputCylinderAxisNormal;
    Eigen::Vector3f rotatedPlaneNormal = matrix_R * vInputPlaneNormal;

    //rotatedPlaneNormal(0) = 0.70;
    //rotatedPlaneNormal(1) = 0.70;
    //rotatedPlaneNormal(2) = 0.0;
    /// This line pretty much overrides everything above with respect to finding the rotated plane. It can be removed once a better solution is identified.
    /// and by better solution, one that keeps the L system as close of a transformation as possible to the original cloud (ideally just a translation).
    rotatedPlaneNormal = vInputPlaneNormal;

    logStream << "The rotated cylinder normal is now:\n" << rotatedCylinderNormal << std::endl;
    logStream << "the rotated plane normal is now:\n" << rotatedPlaneNormal << std::endl;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    phyllotaxyAngleRelativeToXAxis = (atan2(rotatedPlaneNormal[1], rotatedPlaneNormal[0]) - atan2(vectorUpXaxis[1], vectorUpXaxis[0]) ) * (180.0 / M_PI);

    logStream << "atan2 of rotatedPlaneNormal and x axis: " << phyllotaxyAngleRelativeToXAxis;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    return phyllotaxyAngleRelativeToXAxis;
}

std::pair<float, float> returnSecondLeafCurvatureControlPointEstimate(pcl::PointXYZ inputLeafPointOrigin, pcl::PointXYZ inputLeafPointEnd, InputParameters inputParams) {
    std::ostringstream logStream;
    std::pair<float, float> secondControlPointEstimate(99.99, 99.99);
    LOG.DEBUG("Estimating second leaf curvature control point.");

    // Arbitrary magic scaling factor of 100 and fixing the z axis. Change this to something more appropriate once it's worked out.
    //float xEnd = (100.0 * vInputLeafNormal(0,0));
    //float yEnd = (100.0 * vInputLeafNormal(1,0));
    //float zEnd = (100.0 * vInputLeafNormal(2,0));

    pcl::PointXYZ rightTrianglePoint(inputLeafPointEnd.x, inputLeafPointEnd.y, inputLeafPointOrigin.z);

    secondControlPointEstimate.first = abs(inputLeafPointEnd.x - inputLeafPointOrigin.x);
    secondControlPointEstimate.second = abs(inputLeafPointEnd.y - inputLeafPointOrigin.y);
    logStream << "Second control point estimate: (" << secondControlPointEstimate.first << ", " << secondControlPointEstimate.second << ")";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    return secondControlPointEstimate;
}


int fitLSystemToPointCloudv2(int argc, char** argv, InputParameters inputParams){
    std::ostringstream logStream;
    LOG.DEBUG("Initializing the fitting of an L-system to point clouds");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo estimate normals with K search:");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo perform segmentation to identify a cylinder corresponding to the stem:");
    inputParams.sacSegmentationFromNormalsParameters.printParameters();

    LOG.DEBUG("\tDebugging parameters:");
    inputParams.debuggingParameters.printParameters();


    /// Load the necessary elements for embedded python.
    LOG.DEBUG("Initializing Python interpreter for downstream L-system construction.");
    Py_Initialize();
    Py_SetProgramName("BuildLSystemFromC++");
    PyObject *sys = PyImport_ImportModule("sys");
    PyObject *path = PyObject_GetAttrString(sys, "path");
    PyList_Append(path, PyString_FromString("."));

    PyObject *pName = PyString_FromString("externalLPYcalling_v2");
    PyObject *pythonModuleForLSystemConstruction = PyImport_Import(pName);
    Py_DECREF(pName);
    assert(pythonModuleForLSystemConstruction != NULL && "The Python module for creating L-systems could not be successfully imported. Aborting.");

    PyObject *pythonFunctionForLSystemConstruction = PyObject_GetAttrString(pythonModuleForLSystemConstruction, "constructLSystem");
    assert(pythonFunctionForLSystemConstruction && PyCallable_Check(pythonFunctionForLSystemConstruction) && "The Python function for creating L-systems was not found. Aborting.");
    /// Finished loading elements for embedded python.

    int numberOfCloudsPlusOne = argc;
    assert(numberOfCloudsPlusOne >= 2 && "A cloud should be passed at the command line.");

    pcl::visualization::PCLVisualizer *visu;
    std::string objectString = "object";
    std::string sceneString = "source";
    std::string LSystemCloudString = "LSystemCloud";
    std::string LSystemString = "LSystemMesh";
    // Mock up a temporary mesh:
    pcl::PolygonMesh tempMesh; // For temporary addition of an empty mesh that can later be updated.
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempViewerCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ tmpPoint1(0, 0, 0); pcl::PointXYZ tmpPoint2(0, 0.5, 0.5); pcl::PointXYZ tmpPoint3(0, 0, 0.5);
    tempViewerCloud->points.push_back(tmpPoint1); tempViewerCloud->points.push_back(tmpPoint2); tempViewerCloud->points.push_back(tmpPoint3);
    std::vector<pcl::Vertices> tmpMeshFaces;
    pcl::Vertices tmpTriangleFaceIndices;
    tmpTriangleFaceIndices.vertices.push_back(0); tmpTriangleFaceIndices.vertices.push_back(1); tmpTriangleFaceIndices.vertices.push_back(2);
    tmpMeshFaces.push_back(tmpTriangleFaceIndices);
    pcl::PCLPointCloud2 pc2_tempMeshCloud;
    pcl::toPCLPointCloud2(*tempViewerCloud, pc2_tempMeshCloud);
    tempMesh.cloud = pc2_tempMeshCloud;
    tempMesh.polygons = tmpMeshFaces;
    // Finished mocking up a temporary mesh.
    int viewport = 0;
    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu = new pcl::visualization::PCLVisualizer;
        visu->addCoordinateSystem(100.0);
        //For a two viewport viewer
        //visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
        //visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
        //For a three viewport viewer
        //visu->createViewPort (0.00, 0.0, 0.33, 1.0, originalViewport);
        //visu->createViewPort (0.33, 0.0, 0.66, 1.0, measurementViewport);
        //visu->createViewPort (0.66, 0.0, 1.00, 1.0, featureViewport);
        /*visu->setBackgroundColor(0.5, 0.5, 0.5);
        visu->setSize(1700, 1000);

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
        visu->addPointCloud(tempCloud, LSystemCloudString, viewport);
        visu->addPolygonMesh(tempMesh, LSystemString, viewport); //Add the temporary mesh for later removal.
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr originalObject (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloudBlob_originalObject;

    logStream << "Loading point cloud from file:\n" << argv[1];
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

    pcl::io::loadPCDFile(argv[1], cloudBlob_originalObject);
    pcl::fromPCLPointCloud2(cloudBlob_originalObject, *originalObject);
    assert(originalObject->size() > 5 && "Input cloud has too few points. Is it empty?"); //5 is an arbitrary value to make sure the cloud has some points.

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(originalObject, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObject, 0.0, 255.0, 0.0), objectString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the loaded point cloud. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    /// As a first attempt, we'll move from the bottom, up and fit single phytomer L-systems to the cloud.
    /// I think the best way to do this will be to fit a cylinder the the very bottom of the plant (i.e., no leaves)
    ///     and keep moving up incrementally until a large number of points don't fit with the cylinder (i.e., a leaf is found).

    /// Assumptions:
    ///     Plant stem is reasonably well aligned with the Z axis (oriented via pot segmentation)

    /// First, get a short portion of the bottom stem (pass through filter) and find a cylinder that fits it (RANSAC).

    /// To get a short portion of the bottom stem, we've move 5% up from the bottom to the top along the z axis.

    float MAGIC_NUMBER_PROPORTION_UP_AABB_TO_LAYER = 0.05;
    BoundingBox boundingBox;
    BoundingBoxMaker<pcl::PointXYZ> boundingBoxMaker;
    AxisAlignedBoundingBox axisAlignedBoundingBox;
    axisAlignedBoundingBox = boundingBoxMaker.returnAxisAlignedBoundingBoxOfCloud(originalObject);
    float distanceToTravelUp = (axisAlignedBoundingBox.maxZ - axisAlignedBoundingBox.minZ) * MAGIC_NUMBER_PROPORTION_UP_AABB_TO_LAYER;

    LOG.DEBUG("Filtering using pass through.", inputParams.debuggingParameters.getDebuggingLevel(), 1);
    //PassThrough filter application
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    passThroughFilter.setInputCloud(originalObject);
    passThroughFilter.setFilterFieldName("z");
    passThroughFilter.setFilterLimits(axisAlignedBoundingBox.minZ - 10, axisAlignedBoundingBox.minZ + distanceToTravelUp);
    passThroughFilter.filter(*cloudFilteredPassThrough);

    cloudFilteredPassThrough->width = cloudFilteredPassThrough->size();
    cloudFilteredPassThrough->height = 1;

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(cloudFilteredPassThrough, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudFilteredPassThrough, 0.0, 255.0, 0.0), objectString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the loaded point cloud. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    /// Now, find a cylinder that fits those points.
    /// RANSAC of stem to find radius. Let's consider modifying this to find multiple stem layers up the stem instead of one large cylinder.
    // These need to be persistent, so they get elevated scope.
    pcl::ModelCoefficients::Ptr cylinderCoefficients (new pcl::ModelCoefficients);
    Eigen::VectorXf vCoefficients(7);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::vector<int> expandedModelInliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    { // Begin scope for RANSAC
    // I don't think we should assume that the points have normals at this point, so we'll recalculate them.
    LOG.DEBUG("\tEstimating stem normals with K search with the following parameters.");
    inputParams.normalEstimationParameters.printParameters();

    pcl::PointCloud<pcl::Normal>::Ptr cloudFilteredPassThroughNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
    nest.setSearchMethod(treeNormal);
    nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());
    nest.setInputCloud(cloudFilteredPassThrough);
    nest.compute(*cloudFilteredPassThroughNormals);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (inputParams.sacSegmentationFromNormalsParameters.getMaxIterations());
    seg.setDistanceThreshold (inputParams.sacSegmentationFromNormalsParameters.getDistanceThreshold());
    seg.setNormalDistanceWeight (inputParams.sacSegmentationFromNormalsParameters.getNormalDistanceWeight());
    seg.setRadiusLimits (inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMin(),
                        inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMax());
    seg.setInputCloud(cloudFilteredPassThrough);
    seg.setInputNormals(cloudFilteredPassThroughNormals);

    LOG.DEBUG("\tPerforming segmentation to identify a cylinder corresponding to the stem with the following parameters.");
    inputParams.sacSegmentationFromNormalsParameters.printParameters();
    seg.segment (*inliers, *cylinderCoefficients);

    logStream << "PointCloud after segmentation has " << inliers->indices.size () << " inliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    assert(inliers->indices.size() > 0 && "RANSAC unable to identify a cylinder for the stem. Verify stem and parameters.");

    logStream << "Model coefficients:" << std::endl;
    for (size_t i = 0; i < cylinderCoefficients->values.size(); i++) {
            logStream << cylinderCoefficients->values[i] << std::endl;
            vCoefficients[i] = cylinderCoefficients->values[i];
    }
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    if (vCoefficients[5] < 0) {
        LOG.DEBUG("Z component of cylinder is negative. Flipping cylinder normal for consistency.");
        vCoefficients[3] = -vCoefficients[3];
        vCoefficients[4] = -vCoefficients[4];
        vCoefficients[5] = -vCoefficients[5];
        cylinderCoefficients->values[3] = -cylinderCoefficients->values[3];
        cylinderCoefficients->values[4] = -cylinderCoefficients->values[4];
        cylinderCoefficients->values[5] = -cylinderCoefficients->values[5];
    }

    pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> expandedCylinderModel(cloudFilteredPassThrough);
    expandedCylinderModel.setInputCloud(cloudFilteredPassThrough);
    expandedCylinderModel.setInputNormals(cloudFilteredPassThroughNormals);
    expandedCylinderModel.selectWithinDistance(vCoefficients, inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue(), expandedModelInliers);
    logStream << "Found " << expandedModelInliers.size() << " in expandedModelInliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    pcl::PointIndices::Ptr ptr_expandedModelInliers (new pcl::PointIndices);
    ptr_expandedModelInliers->indices = expandedModelInliers;

    extract.setInputCloud(cloudFilteredPassThrough);
    extract.setIndices(ptr_expandedModelInliers);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(cloud_cylinder, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObject, 0.0, 0.0, 255.0), sceneString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the stem cylinder identified with RANSAC. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    logStream << "stem radius and diameter calculated to be: " << vCoefficients[6] << " and " << vCoefficients[6] * 2;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    } // End scope for RANSAC

    /// So we have a vague cylindrical model (i.e., cylinder radius for the lsystem). Can we iteratively move up the stem until the points outside of it
    /// become significant?

    logStream << "Starting to iteratively move up the plant. There are " << expandedModelInliers.size() <<
                    " inliers in the cylinder model currently, and " << cloudFilteredPassThrough->size() <<
                    " points in the layer.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    int numberIterations = 1;
    //while (expandedModelInliers.size() > cloudFilteredPassThrough->size() * .9) {
    while (true) {
        numberIterations = numberIterations + 1;
        passThroughFilter.setInputCloud(originalObject);
        passThroughFilter.setFilterFieldName("z");
        passThroughFilter.setFilterLimits(axisAlignedBoundingBox.minZ - 10, axisAlignedBoundingBox.minZ + (distanceToTravelUp * numberIterations));
        passThroughFilter.filter(*cloudFilteredPassThrough);

        cloudFilteredPassThrough->width = cloudFilteredPassThrough->size();
        cloudFilteredPassThrough->height = 1;

        pcl::PointCloud<pcl::Normal>::Ptr cloudFilteredPassThroughNormals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZ> ());
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
        nest.setSearchMethod(treeNormal);
        nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());
        nest.setInputCloud(cloudFilteredPassThrough);
        nest.compute(*cloudFilteredPassThroughNormals);

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(cloudFilteredPassThrough, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudFilteredPassThrough, 0.0, 255.0, 0.0), objectString);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the next iteration up through point cloud. Press q to continue.");
                visu->spin();
            }
            else {
                visu->spinOnce();
            }
        }

        { // Scoping
        pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> expandedCylinderModel(cloudFilteredPassThrough);
        expandedCylinderModel.setInputCloud(cloudFilteredPassThrough);
        expandedCylinderModel.setInputNormals(cloudFilteredPassThroughNormals);
        expandedCylinderModel.selectWithinDistance(vCoefficients, inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue(), expandedModelInliers);
        logStream << "Found " << expandedModelInliers.size() << " in expandedModelInliers.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        pcl::PointIndices::Ptr ptr_expandedModelInliers (new pcl::PointIndices);
        ptr_expandedModelInliers->indices = expandedModelInliers;

        extract.setInputCloud(cloudFilteredPassThrough);
        extract.setIndices(ptr_expandedModelInliers);
        extract.setNegative(false);
        extract.filter(*cloud_cylinder);

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(cloud_cylinder, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_cylinder, 0.0, 0.0, 255.0), sceneString);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the stem cylinder identified with RANSAC. Press q to continue.");
                visu->spin();
            }
            else {
                visu->spinOnce();
            }
        }
        logStream << "Iteration complete. There are " << expandedModelInliers.size() <<
                    " inliers in the cylinder model currently, and " << cloudFilteredPassThrough->size() <<
                    " points in the layer.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        /// If we've found a set of points that don't belong to the stem, it's likely a leaf emerging.
        /// We can probably save the internode height and radius here.
        /// Then we can figure out the orientation that the leaf is emerging.
        if (expandedModelInliers.size() < cloudFilteredPassThrough->size() * .9) {
            LOG.DEBUG("Found what appears to be a leaf. Examining.");
            /// I think one way to do this, since we expect ~ 180 degree phyllotaxy, is to bisect the stem to isolate a leaf (if more than one
            ///     leaf are appearing), then use orientation of the principal component of the leaf points to orient the L system.
            /// I think we can use the axis of the cylinder as one dimension for the plane, then find the principal component of non cylinder points
            ///     and use an orthogonal normal that is most orthogonal to the cylinder axis.
            std::vector<float> layerMinMaxAvg;
            float layerMin = axisAlignedBoundingBox.minZ + (distanceToTravelUp * (numberIterations - 1));
            float layerMax = axisAlignedBoundingBox.minZ + (distanceToTravelUp * numberIterations);
            float layerAvg = (layerMin + layerMax) / 2.0;
            layerMinMaxAvg.push_back(layerMin);
            layerMinMaxAvg.push_back(layerMax);
            layerMinMaxAvg.push_back(layerAvg);
            logStream << "The min, max, and average of the current z layer are: " << layerMin << ", " << layerMax << ", " << layerAvg;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");



            LOG.DEBUG("Finding the principal components of the non-cylinder points.");
            pcl::PointCloud<pcl::PointXYZ>::Ptr nonCylinderPoints(new pcl::PointCloud<pcl::PointXYZ>);

            extract.setInputCloud(cloudFilteredPassThrough);
            extract.setIndices(ptr_expandedModelInliers);
            extract.setNegative(true);
            extract.filter(*nonCylinderPoints);
            logStream << "Found " << nonCylinderPoints->size() << " points that are nonCylinderPoints.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PCA<pcl::PointXYZ> pca;
            pca.setInputCloud(nonCylinderPoints);
            pca.project(*nonCylinderPoints, *cloudPCAprojection);

            Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
            Eigen::Vector3f eigenValues = pca.getEigenValues();
            Eigen::Vector4f mean = pca.getMean();
            logStream << "EigenVectors:\n" << eigenVectors << std::endl;
            logStream << "EigenValues:\n" << eigenValues << std::endl;
            logStream << "Mean:\n" << mean;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            /// Try to draw a line that starts halway in the layer z interval and centered at the cylinder center, pointing
            ///     in the direction of the first principal component.
            /// http://math.stackexchange.com/questions/404440/what-is-the-formula-for-a-3d-line
            // (x, y, z) = (x_0, y_0, z_0) + t(a, b, c)
            // x = x_0 + ta
            // y = y_0 + tb
            // z = z_0 + tc
            // x_0 = cylinderCoefficients->values[0]
            // y_0 = cylinderCoefficients->values[1]
            // z_0 = cylinderCoefficients->values[2]
            // a = cylinderCoefficients->values[3]
            // b = cylinderCoefficients->values[4]
            // c = cylinderCoefficients->values[5]

            // We know where in the z dimension we want the line, so first find t.
            // z = z_0 + tc
            float t = (layerAvg - cylinderCoefficients->values[2]) / cylinderCoefficients->values[5];
            float x = cylinderCoefficients->values[0] + t * cylinderCoefficients->values[3];
            float y = cylinderCoefficients->values[1] + t * cylinderCoefficients->values[4];

            pcl::PointXYZ leafPointOrigin(x, y, layerAvg);

            float xEnd = x + (sqrt(eigenValues(0)) * eigenVectors(0,0));
            float yEnd = y + (sqrt(eigenValues(0)) * eigenVectors(1,0));
            float zEnd = layerAvg + (sqrt(eigenValues(0)) * eigenVectors(2,0));
            // If zEnd is below the layerAvg, let's flip the eigenvector for visualization and consistency with downstream processing:
            if (zEnd < layerAvg) {
                eigenVectors(0,0) = eigenVectors(0,0) * -1.0;
                eigenVectors(1,0) = eigenVectors(1,0) * -1.0;
                eigenVectors(2,0) = eigenVectors(2,0) * -1.0;
                xEnd = x + (sqrt(eigenValues(0)) * eigenVectors(0,0));
                yEnd = y + (sqrt(eigenValues(0)) * eigenVectors(1,0));
                zEnd = layerAvg + (sqrt(eigenValues(0)) * eigenVectors(2,0));
            }

            pcl::PointXYZ leafPointEnd(xEnd, yEnd, zEnd);

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                //visu->addPlane(*planeCoefficients, "planeString", viewport);
                srand(time(NULL));
                std::stringstream lineStringNameStream;
                lineStringNameStream << "lineString_" << rand()%10000;
                visu->addLine(leafPointOrigin, leafPointEnd, lineStringNameStream.str(), viewport);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the line approximating the leaf points. Press q to continue.");
                    visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            /// Maybe what we really want is to use the x and the y of the leaf projection, and fix the z to find a plane.
            // http://mathworld.wolfram.com/Plane.html

            float xPlaneEnd = x + (sqrt(eigenValues(0)) * eigenVectors(0,0));
            float yPlaneEnd = y + (sqrt(eigenValues(0))  * eigenVectors(1,0));
            float zPlaneEnd = layerAvg + (sqrt(eigenValues(0)) * eigenVectors(2,0) * 0.0);
            pcl::PointXYZ leafPointPlaneEnd(xPlaneEnd, yPlaneEnd, zPlaneEnd);

            // based on http://mathworld.wolfram.com/Plane.html
            // To find the equation of the plane, I think we can use:
            //  the normal (eigenVectors(0,0), eigenVectors(1,0), 0.0)
            //  the normal origin (leafPointOrigin)
            //  and a point on the plane (the cylinder axis point: x_0 = cylinderCoefficients->values[0], y_0 = cylinderCoefficients->values[1]
            //                          z_0 = cylinderCoefficients->values[2]
            // ax + by + cz + d = 0
            // d = -ax_0 - by_0 - cz_0

            float intercept = -(eigenVectors(0,0) * leafPointOrigin.x) - (eigenVectors(1,0) * leafPointOrigin.y) - (0.0 * leafPointOrigin.z);

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                srand(time(NULL));
                std::stringstream lineStringNameStream;
                lineStringNameStream << "lineString_" << (rand()%10000 + 1);
                visu->addLine(leafPointOrigin, leafPointPlaneEnd, 1.0, 0.0, 0.0, lineStringNameStream.str(), viewport);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying an orthogonal line. Press q to continue.");
                    visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }


            /// With the rotated line, we use it as the basis of a plane to bisect the points. All points that are one one side belong to a new
            ///     leaf point cloud.
            // http://stackoverflow.com/questions/15688232/check-which-side-of-a-plane-points-are-on

            int sideNormal = 0;
            int sideOppositeNormal = 0;
            for (uint32_t i = 0; i < nonCylinderPoints->size(); i ++) {
                pcl::PointXYZ currentPoint = nonCylinderPoints->points[i];
                float planeSideResult = (eigenVectors(0,0) * currentPoint.x) + (eigenVectors(1,0) * currentPoint.y) + (currentPoint.z * 0.0) + intercept;
                if (planeSideResult > 0) {
                    sideNormal = sideNormal + 1;
                }
                else if (planeSideResult < 0) {
                    sideOppositeNormal = sideOppositeNormal + 1;
                }
            }
            logStream << "Points on the side of the normal: " << sideNormal << std::endl;
            logStream << "Points on the opposite side of the normal: " << sideOppositeNormal;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            /// Now I think we have enough data to build a rough L-system. We have estimations of:
            ///     how tall the internode is.
            ///     the radius of the internode.
            ///     the phyllotaxy of the leaf with respect to the internode.
            ///     the angle of emergence of the leaf with respect to the internode.
            /// Now let's build an L-system from that.

            LSystemParameters lsystemParams;
            // First, internode height is going to be layerAvg - axisAlignedBoundingBox.minZ , though this only works for the first internode.
            std::vector<float> internodeLengths;
            std::vector<float> internodeRadii;
            std::vector<float> internodePitchAngles;
            std::vector<float> internodeTurnAngles;

            std::vector<float> leafPhyllotaxyAngles;
            std::vector< std::vector < std::pair<float, float> > > leafCurvatures;

            internodeLengths.push_back(layerAvg - axisAlignedBoundingBox.minZ );
            internodeRadii.push_back(vCoefficients[6]);

            Eigen::Vector3f cylinderNormalForTurnAndPitch(vCoefficients[3], vCoefficients[4], vCoefficients[5]);
            std::pair<float, float> turnAndPitch = returnTurnAndPitchAnglesToMoveZAxisToNormal(cylinderNormalForTurnAndPitch, inputParams);
            internodeTurnAngles.push_back(turnAndPitch.first);
            internodePitchAngles.push_back(turnAndPitch.second);


            Eigen::Vector3f leafPrincipalComponent(eigenVectors(0,0), eigenVectors(1,0), eigenVectors(2,0) );
            float leafPhyllotaxyAngle = returnLeafPhyllotaxyAngleToMoveXAxisToNormal(cylinderNormalForTurnAndPitch, leafPrincipalComponent, inputParams);
            leafPhyllotaxyAngles.push_back(leafPhyllotaxyAngle);

            // Can we also use the leaf principal component to define the first part of the leaf curvature?
            // As a first approximation, let's say that the second curvature point of the leaf is 100 units greater than the unit vector.
            std::pair<float,float> leafCurveFirst_P1(0.0, 0.0);
            std::pair<float,float> leafCurveFirst_P2 = returnSecondLeafCurvatureControlPointEstimate(leafPointOrigin, leafPointPlaneEnd, inputParams);
            std::pair<float,float> leafCurveFirst_P3(120.0, 130.0);
            std::pair<float,float> leafCurveFirst_P4(200.0, 80.0);
            std::vector< std::pair<float,float> > leafCurveFirst = {leafCurveFirst_P1, leafCurveFirst_P2, leafCurveFirst_P3, leafCurveFirst_P4};
            leafCurvatures.push_back(leafCurveFirst);

            lsystemParams.setInternodeLengths(internodeLengths);
            lsystemParams.setInternodeRadii(internodeRadii);
            lsystemParams.setInternodeTurnAngles(internodeTurnAngles);
            lsystemParams.setInternodePitchAngles(internodePitchAngles);
            lsystemParams.setLeafPhyllotaxyAngles(leafPhyllotaxyAngles);
            lsystemParams.setLeafCurvatures(leafCurvatures);

            pcl::PolygonMesh LSystemMesh = buildLSystem(pythonFunctionForLSystemConstruction, lsystemParams, visu, inputParams);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                //visu->updatePolygonMesh(LSystemMesh, LSystemString);
                visu->removePolygonMesh(LSystemString);
                visu->addPolygonMesh(LSystemMesh, LSystemString, viewport);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the polygon mesh generated from the L-system. Press q to continue.");
                visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystem (new pcl::PointCloud<pcl::PointXYZ>);

            /// Now we need to get a point cloud sampled from the mesh of the L-system to create what a point cloud generated from that L-system might look like.
            sampleMeshToPointCloud(&LSystemMesh, *sampledLSystem, inputParams);

            logStream << "Received a cloud with: " << sampledLSystem->size() << " points after sampling.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");


            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                visu->removePolygonMesh(LSystemString);
                visu->updatePointCloud(sampledLSystem, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(sampledLSystem, 255.0, 0.0, 255.0), LSystemCloudString);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Removed the polygon mesh, and displaying the cloud sampled from the L-System. Press q to continue.");
                visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            /// Now I guess we can fit this L-system and make minor refinements to things like internode height, leaf phyllotaxy, and the first
            ///     two control points (really, only the second control point since the first is always (0, 0). I don't think we'd want to refine
            ///     either of the other two control points since we might not have the full leaf yet.

            /// This will allow us to assign points as accounted for or not by the L-System.
            /// Iterative closest point seems as good as anything else for the time being, though really we should be able to just transform
            /// the Lsystem to the RANSAC cylinder of the stem and get pretty close?


            pcl::CorrespondencesPtr correspondencesBetweenLSystemAndTarget = registerPointCloudsICPAndReturnCorrespondences(sampledLSystem, cloudFilteredPassThrough, visu, inputParams);
            pcl::PointIndices::Ptr indicesOfPointsInTargetWithCorrespondence (new pcl::PointIndices);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRemainingAfterLSystemFit (new pcl::PointCloud<pcl::PointXYZ>);
            std::map<uint32_t, uint32_t> mapOfCorrespondences;
            for (uint32_t i = 0; i < correspondencesBetweenLSystemAndTarget->size(); i++) {
                pcl::Correspondence currentCorrespondence = (*correspondencesBetweenLSystemAndTarget)[i];
                //logStream << "From outside the class. Index of the source point: " << currentCorrespondence.index_query << std::endl;
                //logStream << "Index of the matching target point: " << currentCorrespondence.index_match << std::endl;
                //logStream << "Distance between the corresponding points: " << currentCorrespondence.distance << std::endl;
                //logStream << "Weight of the confidence in the correspondence: " << currentCorrespondence.weight;
                //LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
                //srand(time(NULL));
                //std::stringstream lineStringNameStream;
                //lineStringNameStream << "lineString_" << rand()%10000 + i;
                //visu->addLine(sampledLSystem->points[currentCorrespondence.index_query], cloudFilteredPassThrough->points[currentCorrespondence.index_match], lineStringNameStream.str());
                if (mapOfCorrespondences.find(currentCorrespondence.index_match) == mapOfCorrespondences.end() ) {
                    indicesOfPointsInTargetWithCorrespondence->indices.push_back(currentCorrespondence.index_match);
                    mapOfCorrespondences.insert(std::pair<uint32_t, uint32_t>(currentCorrespondence.index_match, 1));
                }
                else {
                    continue;
                }
            }

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                visu->updatePointCloud(sampledLSystem, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(sampledLSystem, 255.0, 0.0, 255.0), LSystemCloudString);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the cloud sampled from the L-System. Press q to continue.");
                visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            extract.setInputCloud(cloudFilteredPassThrough);
            extract.setIndices(indicesOfPointsInTargetWithCorrespondence);
            extract.setNegative(true);
            extract.filter(*cloudRemainingAfterLSystemFit);
            logStream << "Found " << cloudRemainingAfterLSystemFit->size() << " points remaining (i.e. do not correspond to points in the LSystem.)";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                visu->updatePointCloud(cloudRemainingAfterLSystemFit, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudRemainingAfterLSystemFit, 0.0, 255.0, 0.0), sceneString);
                visu->updatePointCloud(cloudRemainingAfterLSystemFit, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudRemainingAfterLSystemFit, 0.0, 255.0, 0.0), objectString);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the cloud that remains after removing L-system points. Press q to continue.");
                visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            /// At this point, I think we'll have a decent L-system for the first phytomer.
            ///     The next step is to move up one and add a phytomer, taking into consideration the current L-system and the points that remain after fitting it.


        }
        } // End scope.
    }

    Py_Finalize();
    return(0);
}

int fitLSystemToPointCloudv1(int argc, char** argv, InputParameters inputParams){
    std::ostringstream logStream;
    LOG.DEBUG("Initializing the fitting of an L-system to point clouds");
    LOG.DEBUG("The following input parameters will be used during this process:");
    LOG.DEBUG("\tTo estimate normals with K search:");
    inputParams.normalEstimationParameters.printParameters();
    LOG.DEBUG("\tTo perform segmentation to identify a cylinder corresponding to the stem:");
    inputParams.sacSegmentationFromNormalsParameters.printParameters();

    LOG.DEBUG("\tDebugging parameters:");
    inputParams.debuggingParameters.printParameters();

    //int numberOfCloudsPlusOne = argc;
    //assert(numberOfCloudsPlusOne >= 2 && "At least two point clouds should be passed at the command line.");

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


    pcl::PointCloud<pcl::PointXYZ>::Ptr originalObject (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 cloudBlob_originalObject;

    logStream << "Loading point cloud from file:\n" << argv[1];
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

    pcl::io::loadPCDFile(argv[1], cloudBlob_originalObject);
    pcl::fromPCLPointCloud2(cloudBlob_originalObject, *originalObject);
    assert(originalObject->size() > 5 && "Input cloud has too few points. Is it empty?"); //5 is an arbitrary value to make sure the cloud has some points.

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud (originalObject, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObject, 0.0, 255.0, 0.0), objectString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the loaded point cloud. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    /// As a first attempt, we'll move from the bottom, up and fit single phytomer L-systems to the cloud.
    /// I think the best way to do this will be to fit a cylinder the the very bottom of the plant (i.e., no leaves)
    ///     and keep moving up incrementally until a large number of points don't fit with the cylinder (i.e., a leaf is found).

    /// Assumptions:
    ///     Plant stem is reasonably well aligned with the Z axis (oriented via pot segmentation)

    /// First, get a short portion of the bottom stem (pass through filter) and find a cylinder that fits it (RANSAC).

    /// To get a short portion of the bottom stem, we've move 5% up from the bottom to the top along the z axis.

    float MAGIC_NUMBER_PROPORTION_UP_AABB_TO_LAYER = 0.05;
    BoundingBox boundingBox;
    BoundingBoxMaker<pcl::PointXYZ> boundingBoxMaker;
    AxisAlignedBoundingBox axisAlignedBoundingBox;
    axisAlignedBoundingBox = boundingBoxMaker.returnAxisAlignedBoundingBoxOfCloud(originalObject);
    float distanceToTravelUp = (axisAlignedBoundingBox.maxZ - axisAlignedBoundingBox.minZ) * MAGIC_NUMBER_PROPORTION_UP_AABB_TO_LAYER;

    LOG.DEBUG("Filtering using pass through.", inputParams.debuggingParameters.getDebuggingLevel(), 1);
    //PassThrough filter application
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilteredPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    passThroughFilter.setInputCloud(originalObject);
    passThroughFilter.setFilterFieldName("z");
    passThroughFilter.setFilterLimits(axisAlignedBoundingBox.minZ - 10, axisAlignedBoundingBox.minZ + distanceToTravelUp);
    passThroughFilter.filter(*cloudFilteredPassThrough);

    cloudFilteredPassThrough->width = cloudFilteredPassThrough->size();
    cloudFilteredPassThrough->height = 1;

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(cloudFilteredPassThrough, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudFilteredPassThrough, 0.0, 255.0, 0.0), objectString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the loaded point cloud. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    /// Now, find a cylinder that fits those points.
    /// RANSAC of stem to find radius. Let's consider modifying this to find multiple stem layers up the stem instead of one large cylinder.
    // These need to be persistent, so they get elevated scope.
    pcl::ModelCoefficients::Ptr cylinderCoefficients (new pcl::ModelCoefficients);
    Eigen::VectorXf vCoefficients(7);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    std::vector<int> expandedModelInliers;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    { // Begin scope for RANSAC
    // I don't think we should assume that the points have normals at this point, so we'll recalculate them.
    LOG.DEBUG("\tEstimating stem normals with K search with the following parameters.");
    inputParams.normalEstimationParameters.printParameters();

    pcl::PointCloud<pcl::Normal>::Ptr cloudFilteredPassThroughNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
    nest.setSearchMethod(treeNormal);
    nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());
    nest.setInputCloud(cloudFilteredPassThrough);
    nest.compute(*cloudFilteredPassThroughNormals);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (inputParams.sacSegmentationFromNormalsParameters.getMaxIterations());
    seg.setDistanceThreshold (inputParams.sacSegmentationFromNormalsParameters.getDistanceThreshold());
    seg.setNormalDistanceWeight (inputParams.sacSegmentationFromNormalsParameters.getNormalDistanceWeight());
    seg.setRadiusLimits (inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMin(),
                        inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMax());
    seg.setInputCloud(cloudFilteredPassThrough);
    seg.setInputNormals(cloudFilteredPassThroughNormals);

    LOG.DEBUG("\tPerforming segmentation to identify a cylinder corresponding to the stem with the following parameters.");
    inputParams.sacSegmentationFromNormalsParameters.printParameters();
    seg.segment (*inliers, *cylinderCoefficients);

    logStream << "PointCloud after segmentation has " << inliers->indices.size () << " inliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    assert(inliers->indices.size() > 0 && "RANSAC unable to identify a cylinder for the stem. Verify stem and parameters.");

    logStream << "Model coefficients:" << std::endl;
    for (size_t i = 0; i < cylinderCoefficients->values.size(); i++) {
            logStream << cylinderCoefficients->values[i] << std::endl;
            vCoefficients[i] = cylinderCoefficients->values[i];
    }
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

    pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> expandedCylinderModel(cloudFilteredPassThrough);
    expandedCylinderModel.setInputCloud(cloudFilteredPassThrough);
    expandedCylinderModel.setInputNormals(cloudFilteredPassThroughNormals);
    expandedCylinderModel.selectWithinDistance(vCoefficients, inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue(), expandedModelInliers);
    logStream << "Found " << expandedModelInliers.size() << " in expandedModelInliers.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    pcl::PointIndices::Ptr ptr_expandedModelInliers (new pcl::PointIndices);
    ptr_expandedModelInliers->indices = expandedModelInliers;

    extract.setInputCloud(cloudFilteredPassThrough);
    extract.setIndices(ptr_expandedModelInliers);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->updatePointCloud(cloud_cylinder, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObject, 0.0, 0.0, 255.0), sceneString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the stem cylinder identified with RANSAC. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    logStream << "stem radius and diameter calculated to be: " << vCoefficients[6] << " and " << vCoefficients[6] * 2;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    } // End scope for RANSAC

    /// So we have a vague cylindrical model (i.e., cylinder radius for the lsystem). Can we iteratively move up the stem until the points outside of it
    /// become significant?

    logStream << "Starting to iteratively move up the plant. There are " << expandedModelInliers.size() <<
                    " inliers in the cylinder model currently, and " << cloudFilteredPassThrough->size() <<
                    " points in the layer.";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    int numberIterations = 1;
    //while (expandedModelInliers.size() > cloudFilteredPassThrough->size() * .9) {
    while (true) {
        numberIterations = numberIterations + 1;
        passThroughFilter.setInputCloud(originalObject);
        passThroughFilter.setFilterFieldName("z");
        passThroughFilter.setFilterLimits(axisAlignedBoundingBox.minZ - 10, axisAlignedBoundingBox.minZ + (distanceToTravelUp * numberIterations));
        passThroughFilter.filter(*cloudFilteredPassThrough);

        cloudFilteredPassThrough->width = cloudFilteredPassThrough->size();
        cloudFilteredPassThrough->height = 1;

        pcl::PointCloud<pcl::Normal>::Ptr cloudFilteredPassThroughNormals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZ> ());
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
        nest.setSearchMethod(treeNormal);
        nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());
        nest.setInputCloud(cloudFilteredPassThrough);
        nest.compute(*cloudFilteredPassThroughNormals);

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(cloudFilteredPassThrough, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloudFilteredPassThrough, 0.0, 255.0, 0.0), objectString);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the next iteration up through point cloud. Press q to continue.");
                visu->spin();
            }
            else {
                visu->spinOnce();
            }
        }

        { // Scoping
        pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal> expandedCylinderModel(cloudFilteredPassThrough);
        expandedCylinderModel.setInputCloud(cloudFilteredPassThrough);
        expandedCylinderModel.setInputNormals(cloudFilteredPassThroughNormals);
        expandedCylinderModel.selectWithinDistance(vCoefficients, inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue(), expandedModelInliers);
        logStream << "Found " << expandedModelInliers.size() << " in expandedModelInliers.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        pcl::PointIndices::Ptr ptr_expandedModelInliers (new pcl::PointIndices);
        ptr_expandedModelInliers->indices = expandedModelInliers;

        extract.setInputCloud(cloudFilteredPassThrough);
        extract.setIndices(ptr_expandedModelInliers);
        extract.setNegative(false);
        extract.filter(*cloud_cylinder);

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->updatePointCloud(cloud_cylinder, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(originalObject, 0.0, 0.0, 255.0), sceneString);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the stem cylinder identified with RANSAC. Press q to continue.");
                visu->spin();
            }
            else {
                visu->spinOnce();
            }
        }
        logStream << "Iteration complete. There are " << expandedModelInliers.size() <<
                    " inliers in the cylinder model currently, and " << cloudFilteredPassThrough->size() <<
                    " points in the layer.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        /// If we've found a set of points that don't belong to the stem, it's likely a leaf emerging.
        /// We can probably save the internode height and radius here.
        /// Then we can figure out the orientation that the leaf is emerging.
        if (expandedModelInliers.size() < cloudFilteredPassThrough->size() * .9) {
            LOG.DEBUG("Found what appears to be a leaf. Examining.");
            /// I think one way to do this, since we expect ~ 180 degree phyllotaxy, is to bisect the stem to isolate a leaf (if more than one
            ///     leaf are appearing), then use orientation of the principal component of the leaf points to orient the L system.
            /// I think we can use the axis of the cylinder as one dimension for the plane, then find the principal component of non cylinder points
            ///     and use an orthogonal normal that is most orthogonal to the cylinder axis.
            std::vector<float> layerMinMaxAvg;
            float layerMin = axisAlignedBoundingBox.minZ + (distanceToTravelUp * (numberIterations - 1));
            float layerMax = axisAlignedBoundingBox.minZ + (distanceToTravelUp * numberIterations);
            float layerAvg = (layerMin + layerMax) / 2.0;
            layerMinMaxAvg.push_back(layerMin);
            layerMinMaxAvg.push_back(layerMax);
            layerMinMaxAvg.push_back(layerAvg);
            logStream << "The min, max, and average of the current z layer are: " << layerMin << ", " << layerMax << ", " << layerAvg;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");



            LOG.DEBUG("Finding the principal components of the non-cylinder points.");
            pcl::PointCloud<pcl::PointXYZ>::Ptr nonCylinderPoints(new pcl::PointCloud<pcl::PointXYZ>);

            extract.setInputCloud(cloudFilteredPassThrough);
            extract.setIndices(ptr_expandedModelInliers);
            extract.setNegative(true);
            extract.filter(*nonCylinderPoints);
            logStream << "Found " << nonCylinderPoints->size() << " points that are nonCylinderPoints.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PCA<pcl::PointXYZ> pca;
            pca.setInputCloud(nonCylinderPoints);
            pca.project(*nonCylinderPoints, *cloudPCAprojection);

            Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
            Eigen::Vector3f eigenValues = pca.getEigenValues();
            Eigen::Vector4f mean = pca.getMean();
            logStream << "EigenVectors:\n" << eigenVectors << std::endl;
            logStream << "EigenValues:\n" << eigenValues << std::endl;
            logStream << "Mean:\n" << mean;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            /// Try to draw a line that starts halway in the layer z interval and centered at the cylinder center, pointing
            ///     in the direction of the first principal component.
            /// http://math.stackexchange.com/questions/404440/what-is-the-formula-for-a-3d-line
            // (x, y, z) = (x_0, y_0, z_0) + t(a, b, c)
            // x = x_0 + ta
            // y = y_0 + tb
            // z = z_0 + tc
            // x_0 = cylinderCoefficients->values[0]
            // y_0 = cylinderCoefficients->values[1]
            // z_0 = cylinderCoefficients->values[2]
            // a = cylinderCoefficients->values[3]
            // b = cylinderCoefficients->values[4]
            // c = cylinderCoefficients->values[5]

            // We know where in the z dimension we want the line, so first find t.
            // z = z_0 + tc
            float t = (layerAvg - cylinderCoefficients->values[2]) / cylinderCoefficients->values[5];
            float x = cylinderCoefficients->values[0] + t * cylinderCoefficients->values[3];
            float y = cylinderCoefficients->values[1] + t * cylinderCoefficients->values[4];

            pcl::PointXYZ leafPointOrigin(x, y, layerAvg);

            float xEnd = x + (100.0 * eigenVectors(0,0));
            float yEnd = y + (100.0 * eigenVectors(1,0));
            float zEnd = layerAvg + (100.0 * eigenVectors(2,0));
            // If zEnd is below the layerAvg, we can flip it for visualization:
            if (zEnd < layerAvg) {
                xEnd = x + (-100.0 * eigenVectors(0,0));
                yEnd = y + (-100.0 * eigenVectors(1,0));
                zEnd = layerAvg + (-100.0 * eigenVectors(2,0));
            }

            pcl::PointXYZ leafPointEnd(xEnd, yEnd, zEnd);

            pcl::ModelCoefficients::Ptr planeCoefficients (new pcl::ModelCoefficients);
            planeCoefficients->values.push_back(eigenVectors(0,0));
            planeCoefficients->values.push_back(eigenVectors(1,0));
            planeCoefficients->values.push_back(eigenVectors(2,0));
            planeCoefficients->values.push_back(0);


            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                //visu->addPlane(*planeCoefficients, "planeString", viewport);
                srand(time(NULL));
                std::stringstream lineStringNameStream;
                lineStringNameStream << "lineString_" << rand()%10000;
                visu->addLine(leafPointOrigin, leafPointEnd, lineStringNameStream.str(), viewport);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying the line approximating the leaf points. Press q to continue.");
                    visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }

            /// So we should have a rough orientation of the leaf. Now we want to bisect the stem. I think we can do this by ignoring
            ///     the z dimension, and finding a line that is orthogonal to the leaf orientation.
            // http://answers.unity3d.com/questions/564166/how-to-find-perpendicular-line-in-2d.html
            float xEndOrthogonal2nd = x + (100.0 * eigenVectors(0,1));
            float yEndOrthogonal2nd = y + (100.0 * eigenVectors(1,1));
            float zEndOrthogonal2nd = layerAvg + (100.0 * eigenVectors(2,1));
            // If zEnd is below the layerAvg, we can flip it for visualization:
            if (zEndOrthogonal2nd < layerAvg) {
                xEndOrthogonal2nd = x + (-100.0 * eigenVectors(0,1));
                yEndOrthogonal2nd = y + (-100.0 * eigenVectors(1,1));
                zEndOrthogonal2nd = layerAvg + (-100.0 * eigenVectors(2,1));
            }
            pcl::PointXYZ leafPointEndOrthogonal2nd(xEndOrthogonal2nd, yEndOrthogonal2nd, zEndOrthogonal2nd);

            float xEndOrthogonal3rd = x + (100.0 * eigenVectors(0,2));
            float yEndOrthogonal3rd = y + (100.0 * eigenVectors(1,2));
            float zEndOrthogonal3rd = layerAvg + (100.0 * eigenVectors(2,2));
            // If zEnd is below the layerAvg, we can flip it for visualization:
            if (zEndOrthogonal3rd < layerAvg) {
                xEndOrthogonal3rd = x + (-100.0 * eigenVectors(0,2));
                yEndOrthogonal3rd = y + (-100.0 * eigenVectors(1,2));
                zEndOrthogonal3rd = layerAvg + (-100.0 * eigenVectors(2,2));
            }
            pcl::PointXYZ leafPointEndOrthogonal3rd(xEndOrthogonal3rd, yEndOrthogonal3rd, zEndOrthogonal3rd);

            /// Maybe instead of an orthogonal rotation, one way to go is to just rotate it by 90 around one of the axes.
            // http://inside.mines.edu/fs_home/gmurray/ArbitraryAxisRotation/
            // https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
            // v_rot = v cos(theta) + (k * v) sin(theta) + k(k dot v)(1-cos(theta))
            // v is the vector to rotate. k is the unit vector.

            float angleOfRotationDeg = 90.0;
            Eigen::Vector3f firstPrincipalComponent(eigenVectors(0,0), eigenVectors(1,0), eigenVectors(2,0));
            Eigen::Vector3f axisToRotateAround(0, 1, 0);

            Eigen::Vector3f firstTerm = firstPrincipalComponent * cos(angleOfRotationDeg * M_PI / 180.0);
            Eigen::Vector3f secondTerm = (axisToRotateAround.cross(firstPrincipalComponent)) * sin(angleOfRotationDeg * M_PI / 180.0);
            Eigen::Vector3f thirdTerm = axisToRotateAround * ( axisToRotateAround.dot(firstPrincipalComponent) ) * (1.0 - cos(angleOfRotationDeg * M_PI / 180.0));
            Eigen::Vector3f rotatedVector = firstTerm + secondTerm + thirdTerm;

            logStream << "rotatedVector:\n" << rotatedVector << std::endl;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            float xEndRot = x + (100.0 * rotatedVector(0));
            float yEndRot = y + (100.0 * rotatedVector(1));
            float zEndRot = layerAvg + (100.0 * rotatedVector(2));
            pcl::PointXYZ leafPointEndRotation(xEndRot, yEndRot, zEndRot);

            /// Maybe what we really want is to use the x and the y of the leaf projection, and fix the z to find a plane.
            // http://mathworld.wolfram.com/Plane.html

            float xPlaneEnd = x + (100.0 * eigenVectors(0,0));
            float yPlaneEnd = y + (100.0 * eigenVectors(1,0));
            float zPlaneEnd = layerAvg + (100.0 * eigenVectors(2,0) * 0.0);
            pcl::PointXYZ leafPointPlaneEnd(xPlaneEnd, yPlaneEnd, zPlaneEnd);

            // based on http://mathworld.wolfram.com/Plane.html
            // To find the equation of the plane, I think we can use:
            //  the normal (eigenVectors(0,0), eigenVectors(1,0), 0.0)
            //  the normal origin (leafPointOrigin)
            //  and a point on the plane (the cylinder axis point: x_0 = cylinderCoefficients->values[0], y_0 = cylinderCoefficients->values[1]
            //                          z_0 = cylinderCoefficients->values[2]
            // ax + by + cz + d = 0
            // d = -ax_0 - by_0 - cz_0

            float intercept = -(eigenVectors(0,0) * leafPointOrigin.x) - (eigenVectors(1,0) * leafPointOrigin.y) - (0.0 * leafPointOrigin.z);

            if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
                srand(time(NULL));
                std::stringstream lineStringNameStream;
                lineStringNameStream << "lineString_" << (rand()%10000 + 1);
                visu->addLine(leafPointOrigin, leafPointEndOrthogonal2nd, 0.5, 0.5, 0.0, lineStringNameStream.str(), viewport);
                lineStringNameStream << "lineString_" << (rand()%10000 + 2);
                visu->addLine(leafPointOrigin, leafPointEndOrthogonal3rd, 0.0, 0.5, 0.5, lineStringNameStream.str(), viewport);
                lineStringNameStream << "lineString_" << (rand()%10000 + 3);
                visu->addLine(leafPointOrigin, leafPointEndRotation, 0.0, 0.0, 1.0, lineStringNameStream.str(), viewport);
                lineStringNameStream << "lineString_" << (rand()%10000 + 4);
                visu->addLine(leafPointOrigin, leafPointPlaneEnd, 1.0, 0.0, 0.0, lineStringNameStream.str(), viewport);
                if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                    LOG.DEBUG("Displaying an orthogonal line. Press q to continue.");
                    visu->spin();
                }
            }
            else {
                visu->spinOnce();
            }


            /// With the rotated line, we use it as the basis of a plane to bisect the points. All points that are one one side belong to a new
            ///     leaf point cloud.
            // http://stackoverflow.com/questions/15688232/check-which-side-of-a-plane-points-are-on

            int sideNormal = 0;
            int sideOppositeNormal = 0;
            for (uint32_t i = 0; i < nonCylinderPoints->size(); i ++) {
                pcl::PointXYZ currentPoint = nonCylinderPoints->points[i];
                float planeSideResult = (eigenVectors(0,0) * currentPoint.x) + (eigenVectors(1,0) * currentPoint.y) + (currentPoint.z * 0.0) + intercept;
                if (planeSideResult > 0) {
                    sideNormal = sideNormal + 1;
                }
                else if (planeSideResult < 0) {
                    sideOppositeNormal = sideOppositeNormal + 1;
                }
            }
            logStream << "Points on the side of the normal: " << sideNormal << std::endl;
            logStream << "Points on the opposite side of the normal: " << sideOppositeNormal;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        }
        } // End scope.
    }

    return(0);
}

