#include <stdlib.h>
#include <utility>
#include <string>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <map>
#include <assert.h>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_circle3d.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include "boundingBox.h"
#include "segmentation.h"
#include "inputParams.h"
#include "measurementDataContainer.h"
#include "visualizer_helper.h"
#include "segmentation.h"
#include "tupleTriplet.h"
#include "meshMeasurements.h"
#include "loggingHelper.h"
#include "dijkstraPathfinding.h"


float calculateAreaPolygon(const pcl::PointCloud<pcl::PointXYZ> &polygon) {
    float area=0.0;
    int num_points = polygon.size();
    //std::cout << "Polygon size: " << polygon.size() << std::endl;
    int j = 0;
    Eigen::Vector3f va,vb,res;
    res(0) = res(1) = res(2) = 0.0f;
    for (int i = 0; i < num_points; ++i) {
        j = (i + 1) % num_points;
        //std::cout << "i and j: " << i << " " << j << std::endl;
        //std::cout << "polygon[i]: " << polygon[i] << " polygon[j]: " << polygon[j] << std::endl;
        va = polygon[i].getVector3fMap();
        vb = polygon[j].getVector3fMap();
        //std::cout << "va: " << va << "\tvb: " << vb << std::endl;
        res += va.cross(vb);
        //std::cout << "res: " << res << std::endl;
    }
    area=res.norm();
    //std::cout << area << std::endl;
    return area*0.5;
}

float calculateDistanceBetweenPoints(const pcl::PointCloud<pcl::PointXYZ> &edge) {
    float distance=0.0;
    Eigen::Vector3f va, vb;
    va = edge[0].getVector3fMap();
    vb = edge[1].getVector3fMap();
    float xTerm, yTerm, zTerm;
    xTerm = pow( (va(0) - vb(0)), 2 );
    yTerm = pow( (va(1) - vb(1)), 2 );
    zTerm = pow( (va(2) - vb(2)), 2 );

    distance = sqrt(xTerm + yTerm + zTerm);
    return distance;
}

std::pair<float,float> calculateLengthAndWidthFromAreaAndPerimeter(float area, float perimeter) {
    //http://www.had2know.com/academics/rectangle-sides-area-perimeter-quadratic-equation.html
    std::cout << "Using the following area and perimeter:\t" << area << " " << perimeter << std::endl;
    std::pair<float,float> result;
    float length=0.0;
    float width=0.0;
    length = 0.25 * (perimeter + sqrt( (perimeter * perimeter) - (16.0 * area)  ) );
    width = 0.25 * (perimeter - sqrt( (perimeter * perimeter) - (16.0 * area)  ) );
    result = std::make_pair(length, width);
    return result;
}

float returnSurfaceAreaOfMesh(pcl::PolygonMesh *inputMesh){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(inputMesh->cloud, *cloudPoints);

    float cumulativeAreaSum = 0.0;
    for (int i = 0; i < inputMesh->polygons.size(); i++) {
        pcl::PointIndices::Ptr polygonIndices (new pcl::PointIndices);
        pcl::PointCloud<pcl::PointXYZ>::Ptr singlePolygonCloud (new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < 3; j++) {  //Magic number 3 here to indicate number of indices.
            polygonIndices->indices.push_back(inputMesh->polygons[i].vertices[j]);  //indices is a std::vector int
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloudPoints);
        extract.setIndices(polygonIndices);
        extract.filter(*singlePolygonCloud);

        cumulativeAreaSum = cumulativeAreaSum + calculateAreaPolygon(*singlePolygonCloud);
    }
    return cumulativeAreaSum;
}

bool zSortingFunction (pcl::PointXYZ p1, pcl::PointXYZ p2) { return (p1.z < p2.z); }

/** The intent of this should be to take a segmented mesh and write all of
  * the measurements to a file.
  *
 */
int makePlantMeasurements(int argc, char** argv, InputParameters inputParams) {
    std::ostringstream logStream;
    MeasurementDataContainer measurementData;

    LOG.DEBUG("Taking measurements of the entire plant.");

    LOG.DEBUG("Loading PLY to polygon mesh.");
    pcl::PolygonMesh inputMesh;
    pcl::io::loadPLYFile(argv[1], inputMesh);

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
        visu->addCoordinateSystem(50.0);
        visu->setCameraPosition(inputParams.cameraParameters.getxCoordLocation(),
                                inputParams.cameraParameters.getyCoordLocation(),
                                inputParams.cameraParameters.getzCoordLocation(),
                                inputParams.cameraParameters.getxViewComponent(),
                                inputParams.cameraParameters.getyViewComponent(),
                                inputParams.cameraParameters.getzViewComponent(),
                                inputParams.cameraParameters.getxUpComponent(),
                                inputParams.cameraParameters.getyUpComponent(),
                                inputParams.cameraParameters.getzUpComponent());

        // Still working around a bug that prevents removal of a cloud.
        // To do so, we're adding a mock cloud here with a fixed string that we can update later.
        // Updating doesn't crash, though removal does.
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ tmpPoint(1, 1, 1);
        tempCloud->points.push_back(tmpPoint);
        visu->addPointCloud(tempCloud, featureViewportString, featureViewport);
    }

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addPolygonMesh(inputMesh, originalViewportString, originalViewport);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the mesh to be measured. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsColored (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsSupervoxeled (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);

    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudPoints);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudPointsColored);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudWithNormals);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudNormals);

    /// Bounding box dimensions of plant
    // These are not scoped because bounding box dimensions are used for additional calculations downstream.
    BoundingBox boundingBox;
    BoundingBoxMaker<pcl::PointXYZRGBA> boundingBoxMaker;

    boundingBox = boundingBoxMaker.returnBoundingBoxOfCloud(cloudPointsColored);

    logStream << std::endl << "oriented bbox width: " << boundingBox.width << std::endl;
    logStream << "oriented bbox height: " << boundingBox.height << std::endl;
    logStream << "oriented bbox depth: " << boundingBox.depth;
    LOG.DEBUG(logStream.str()); logStream.str("");

    measurementData.addNameAndMeasurement("oriented_bbox_width", boundingBox.width);
    measurementData.addNameAndMeasurement("oriented_bbox_height", boundingBox.height);
    measurementData.addNameAndMeasurement("oriented_bbox_depth", boundingBox.depth);

    AxisAlignedBoundingBox axisAlignedBoundingBox;
    axisAlignedBoundingBox = boundingBoxMaker.returnAxisAlignedBoundingBoxOfCloud(cloudPointsColored);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visu->addCube(boundingBox.bboxTransform, boundingBox.bboxQuaternion, boundingBox.width, boundingBox.height, boundingBox.depth, "bbox", originalViewport);
        visu->addCube(axisAlignedBoundingBox.minX, axisAlignedBoundingBox.maxX,
                    axisAlignedBoundingBox.minY, axisAlignedBoundingBox.maxY,
                    axisAlignedBoundingBox.minZ, axisAlignedBoundingBox.maxZ, 1.0, 1.0, 1.0, "aligned_bbox", measurementViewport);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the oriented bounding box and the axis aligned bounding box. Press q to continue.");
            visu->spin();
        }
        else {
            visu->spinOnce();
        }
    }


    logStream << "axis aligned bbox x distance: " << axisAlignedBoundingBox.maxX - axisAlignedBoundingBox.minX << std::endl;
    logStream << "axis aligned bbox y distance: " << axisAlignedBoundingBox.maxY - axisAlignedBoundingBox.minY << std::endl;
    logStream  << "axis aligned bbox z distance: " << axisAlignedBoundingBox.maxZ - axisAlignedBoundingBox.minZ;
    LOG.DEBUG(logStream.str()); logStream.str("");
    measurementData.addNameAndMeasurement("axisAligned_bbox_xDistance", axisAlignedBoundingBox.maxX - axisAlignedBoundingBox.minX);
    measurementData.addNameAndMeasurement("axisAligned_bbox_yDistance", axisAlignedBoundingBox.maxY - axisAlignedBoundingBox.minY);
    measurementData.addNameAndMeasurement("axisAligned_bbox_zDistance", axisAlignedBoundingBox.maxZ - axisAlignedBoundingBox.minZ);

    /// Centroid
    { //scoping centroid calculation.
    Eigen::Vector4f centroid;
    compute3DCentroid(*cloudPoints, centroid);

    logStream << "Centroid coordinates: (" << centroid(0) << ", " << centroid(1) << ", " << centroid(2) << ")";
    LOG.DEBUG(logStream.str()); logStream.str("");

    measurementData.addNameAndMeasurement("centroid_xCoord", centroid(0));
    measurementData.addNameAndMeasurement("centroid_yCoord", centroid(1));
    measurementData.addNameAndMeasurement("centroid_zCoord", centroid(2));

    float centroid_zCoord_minus_aabbMinZ = centroid(2) - axisAlignedBoundingBox.minZ;
    logStream << "Distance in Z plane from centroid to aabb Z bottom: " << centroid_zCoord_minus_aabbMinZ;
    LOG.DEBUG(logStream.str()); logStream.str("");

    measurementData.addNameAndMeasurement("centroid_zCoord_minus_aabbMinZ", centroid_zCoord_minus_aabbMinZ);
    } //end scoping centroid calculation.

    /// Surface area of plant
    float elevatedScopeSurfaceArea = returnSurfaceAreaOfMesh(&inputMesh);
    { //scoping surface area calculations.
    logStream << "Number of polygons used for measuring surface area " << inputMesh.polygons.size() << std::endl;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    float surfaceArea = returnSurfaceAreaOfMesh(&inputMesh);
    logStream << "Total area: " << surfaceArea << " mm^2 " << " or " << surfaceArea/100.0 << " cm^2";
    LOG.DEBUG(logStream.str()); logStream.str("");
    measurementData.addNameAndMeasurement("total_surface_area", surfaceArea);
    } //end scoping of surface area calculations.

    /// Area of z layers up the canopy.
    { //scoping calculation of area of z layers up the canopy (i.e. leaf area density).

    float zStep = 200.0;  //This should probably be made into an input parameter.
    logStream << std::endl << "Fixed step surface area up the canopy. Using step units of " << zStep;
    LOG.DEBUG(logStream.str()); logStream.str("");
    float minimumZ = axisAlignedBoundingBox.minZ;
    float stoppingZ = 2000.0; //2 meters This needs to be changed at some point.
    float currentZ = minimumZ + zStep;
    float currentLayerArea = 0.0;
    float cumulativeAreaCurrent = 0.0;
    float cumulativeAreaPrevious = 0.0;
    int layerCounter = 0;

    // Area of Z layers at fixed step intervals up the plant.
    while (currentZ < stoppingZ) {
        layerCounter = layerCounter + 1;
        // Construct a cloud containing all of the points beneath the current Z axis
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointsBeneathZ (new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < cloudPoints->points.size(); i++) {
            pcl::PointXYZ currentPoint = cloudPoints->points[i];
            if (currentPoint.z <= currentZ) {
                pointsBeneathZ->points.push_back(currentPoint);
            }
        }
        pcl::PolygonMesh meshBeneathZ = extractMeshFromPolygonMeshGivenPointCloud(inputMesh, pointsBeneathZ);
        cumulativeAreaCurrent = returnSurfaceAreaOfMesh(&meshBeneathZ);
        currentLayerArea = cumulativeAreaCurrent - cumulativeAreaPrevious;

        logStream << "current Z level: " << currentZ <<
            " area of current layer in mm^2 : " << currentLayerArea << " cumulative area in mm^2: " << cumulativeAreaCurrent;
        LOG.DEBUG(logStream.str()); logStream.str("");
        std::stringstream ss;
        ss << layerCounter;
        std::string layerName = "surfaceArea_layer_" + ss.str();
        measurementData.addNameAndMeasurement(layerName, currentLayerArea);

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->addPolygonMesh(meshBeneathZ, "segmentedMesh2", measurementViewport);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the mesh beneath the z step. Press q to continue.");
                visu->spin();
            }
            else {
                visu->spinOnce();
            }
            visu->removePolygonMesh("segmentedMesh2");
        }

        cumulativeAreaPrevious = cumulativeAreaCurrent;
        currentZ = currentZ + zStep;
    }
    } // end scoping calculation of area of z layers up the canopy (i.e. leaf area density).

    { //scoping this
    // Area of Z layers at quarter proportions of the plant (e.g. upper 25% of plant, etc.)
    float proportion = 0.20;  // This should probably be made into an input parameter.
    float zStep = proportion * (axisAlignedBoundingBox.maxZ - axisAlignedBoundingBox.minZ);
    logStream << std::endl << "Proportional step surface area up the canopy. Using proportion of " << proportion <<
                " and zSteps of " << zStep;
    LOG.DEBUG(logStream.str()); logStream.str("");
    float minimumZ = axisAlignedBoundingBox.minZ;
    float currentZ = minimumZ + zStep;
    float currentLayerArea = 0.0;
    float cumulativeAreaCurrent = 0.0;
    float cumulativeAreaPrevious = 0.0;
    int layerCounter = 0;

    while (currentZ <= axisAlignedBoundingBox.maxZ) {
        layerCounter = layerCounter + 1;
        // Construct a cloud containing all of the points beneath the current Z axis
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointsBeneathZ (new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < cloudPoints->points.size(); i++) {
            pcl::PointXYZ currentPoint = cloudPoints->points[i];
            if (currentPoint.z <= currentZ) {
                pointsBeneathZ->points.push_back(currentPoint);
            }
        }
        pcl::PolygonMesh meshBeneathZ = extractMeshFromPolygonMeshGivenPointCloud(inputMesh, pointsBeneathZ);
        cumulativeAreaCurrent = returnSurfaceAreaOfMesh(&meshBeneathZ);
        currentLayerArea = cumulativeAreaCurrent - cumulativeAreaPrevious;

        logStream << "current Z level: " << currentZ <<
            " area of current layer in mm^2 : " << currentLayerArea << " cumulative area in mm^2: " << cumulativeAreaCurrent;
        LOG.DEBUG(logStream.str()); logStream.str("");
        std::stringstream ss;
        ss << layerCounter;
        std::string layerName = "surfaceArea_proportionalLayer_" + ss.str();
        measurementData.addNameAndMeasurement(layerName, currentLayerArea);
        //elevatedScopeSurfaceArea
        std::string percentageLayerName = "surfaceArea_propOfTotal_proportionalLayer_" + ss.str();
        measurementData.addNameAndMeasurement(percentageLayerName, (currentLayerArea / elevatedScopeSurfaceArea));

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->addPolygonMesh(meshBeneathZ, "segmentedMesh2", measurementViewport);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the mesh beneath the z step. Press q to continue.");
                visu->spin();
            }
            else {
                visu->spinOnce();
            }
            visu->removePolygonMesh("segmentedMesh2");
        }

        cumulativeAreaPrevious = cumulativeAreaCurrent;
        currentZ = currentZ + zStep;
    }
    } //end scope

    /* Unfinished implementation of surface area measurement from top down of plant.
    { // begin scope for measurement of surface area from top of plant down to point where 50% of the surface area is contained.
    float proportionStepSize = 0.05;  // We will check the amount of surface area at 5% increments of height
    float zStep = proportionStepSize * (axisAlignedBoundingBox.maxZ - axisAlignedBoundingBox.minZ);
    float proportionSurfaceArea = 0.5;
    logStream << std::endl << "Determining distance down the plant at which a proportion of the surface area is present. Using proportion of " << proportionSurfaceArea <<
                "; zSteps down will be " << zStep;
    LOG.DEBUG(logStream.str()); logStream.str("");
    float maximumZ = axisAlignedBoundingBox.maxZ;
    float currentZ = maximumZ - zStep;
    float currentLayerArea = 0.0;
    float cumulativeAreaCurrent = 0.0;
    float cumulativeAreaPrevious = 0.0;
    int layerCounter = 0;

    while (cumulativeAreaCurrent / elevatedScopeSurfaceArea <= 0.5 ) {
        layerCounter = layerCounter + 1;
        // Construct a cloud containing all of the points above the current Z axis
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointsAboveZ (new pcl::PointCloud<pcl::PointXYZ>);
        for (uint32_t i = 0; i < cloudPoints->points.size(); i++) {
            pcl::PointXYZ currentPoint = cloudPoints->points[i];
            if (currentPoint.z >= currentZ) {
                pointsAboveZ->points.push_back(currentPoint);
            }
        }

        /// RESUME FROM HERE 03/08/16
        pcl::PolygonMesh meshAboveZ = extractMeshFromPolygonMeshGivenPointCloud(inputMesh, pointsAboveZ);
        cumulativeAreaCurrent = returnSurfaceAreaOfMesh(&meshBeneathZ);
        currentLayerArea = cumulativeAreaCurrent - cumulativeAreaPrevious;

        logStream << "current Z level: " << currentZ <<
            " area of current layer in mm^2 : " << currentLayerArea << " cumulative area in mm^2: " << cumulativeAreaCurrent;
        LOG.DEBUG(logStream.str()); logStream.str("");
        std::stringstream ss;
        ss << layerCounter;
        std::string layerName = "surfaceArea_proportionalLayer_" + ss.str();
        measurementData.addNameAndMeasurement(layerName, currentLayerArea);
        //elevatedScopeSurfaceArea
        std::string percentageLayerName = "surfaceArea_propOfTotal_proportionalLayer_" + ss.str();
        measurementData.addNameAndMeasurement(percentageLayerName, (currentLayerArea / elevatedScopeSurfaceArea));

        if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
            visu->addPolygonMesh(meshBeneathZ, "segmentedMesh2", measurementViewport);
            if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
                LOG.DEBUG("Displaying the mesh beneath the z step. Press q to continue.");
                visu->spin();
            }
            else {
                visu->spinOnce();
            }
            visu->removePolygonMesh("segmentedMesh2");
        }

        cumulativeAreaPrevious = cumulativeAreaCurrent;
        currentZ = currentZ + zStep;
    }

    } // end scope for measurement of surface area from top of plant down to point where 50% of the surface area is contained.
    End unfinished implementation of surface ear from top down of plant. */

    /// Convex hull of plant
    { // scoping
    LOG.DEBUG("Constructing the convex hull of plant.");
    std::vector<pcl::Vertices> convexHullVertices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> convexHullMaker;
    convexHullMaker.setInputCloud(cloudPoints);
    convexHullMaker.setComputeAreaVolume(true);
    convexHullMaker.reconstruct(*convexHull, convexHullVertices);

    double convexHullArea = 0;
    double convexHullVolume = 0;
    convexHullArea = convexHullMaker.getTotalArea();
    convexHullVolume = convexHullMaker.getTotalVolume();

    logStream << "Area of the convex hull in mm^2: " << convexHullArea << std::endl;
    logStream << "Volume of the convex hull in mm^3: " << convexHullVolume;
    LOG.DEBUG(logStream.str()); logStream.str("");
    measurementData.addNameAndMeasurement("convexHullArea", convexHullArea);
    measurementData.addNameAndMeasurement("convexHullVolume", convexHullVolume);

    } // end scope

    // Diameter of mesh
    // To make sure we don't get any strange results with respect to unconnected plant pieces,
    // we enforce that we calculate diameter from a fully connected mesh.
    pcl::PolygonMesh connectedWholePlantMesh = returnLargestConnectedMesh(&inputMesh);
    Graph plantGraph(&connectedWholePlantMesh);
    DijkstraPathfinder plantPathfinder(plantGraph);

    PathDataContainer geodesicDiameter = plantPathfinder.findGeodesicDiameter();
    logStream << "Geodesic diameter of entire plant mesh: " << geodesicDiameter._distance;
    LOG.DEBUG(logStream.str()); logStream.str("");
    measurementData.addNameAndMeasurement("plant_maxGeodesicLength", geodesicDiameter._distance);

    ColorMap colorMap;
    PlantSegmentationDataContainer segmentationData(&inputMesh);

    makeSegmentedFeatureMeasurements(&inputMesh, measurementData, inputParams, visu);
    measurementData.writeMeasurementsToFile("PlantMeasurements.tsv");

    return 0;

}

int makeSegmentedFeatureMeasurements(pcl::PolygonMesh *inputMesh, MeasurementDataContainer &inputMeasurementData, InputParameters inputParams, pcl::visualization::PCLVisualizer *visualizer) {
    std::ostringstream logStream;
    /// I think we want to iterate over the individual segmented components, and choose how to handle them based on
    /// whether they are leaf or stem. For now, let's assume they are the largest connected piece.
    /// At the end we can get things like total leaf area.
    /// Average leaf area of the leaves beneath the top three leaves, etc.
    //writeIndividualComponentsOfSegmentedMesh(completeMeshToWrite, visualizer);
    int originalViewport = 1;
    int measurementViewport = 2;
    int featureViewport = 3;
    std::string originalViewportString = "orignalMesh";
    std::string measurementViewportString = "measurementMesh";
    std::string featureViewportString = "featureMesh";

    ColorMap colorMap;
    PlantSegmentationDataContainer segmentationData(inputMesh);

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
    logStream << "Found " << colorMapOfCurrentMesh.size() << " colors in the mesh.";
    LOG.DEBUG(logStream.str()); logStream.str("");

    //First, for each color, pull out the mesh corresponding to that color.
    std::vector<pcl::PolygonMesh> v_meshes;
    int numberOfLeaves = 0;
    std::map<TupleTriplet, int>::iterator colorMapItr;
    for (colorMapItr = colorMapOfCurrentMesh.begin(); colorMapItr != colorMapOfCurrentMesh.end(); colorMapItr++) {
        TupleTriplet currentColor = colorMapItr->first;
        LOG.DEBUG("Processing the following color from the input mesh: ", inputParams.debuggingParameters.getDebuggingLevel(), 1);
        printTupleTriplet(currentColor, inputParams.debuggingParameters.getDebuggingLevel(), 1);

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
        logStream << "Found " << pointCounter << " points that match the color.";
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");

        // Check to make sure that the color is expected. If not, skip it.
        if(colorMap._map_explicitlyHandledColors.find(currentColor) == colorMap._map_explicitlyHandledColors.end()) {
            LOG.DEBUG("WARNING:");
            LOG.DEBUG("WARNING: Encounted a color that is not explicitly handled. Make sure the mesh is as expected.");
            LOG.DEBUG("WARNING:");
            continue;
        }

        pcl::PolygonMesh currentMesh = extractMeshFromPolygonMeshGivenPointCloud(*inputMesh, currentCloud);
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

        if (currentCloudPointsColored->points.size() < 3) {
            LOG.DEBUG("Excluding this mesh. It contains too few points.", inputParams.debuggingParameters.getDebuggingLevel(), 1);
        }
        else {
            logStream << "Keeping a mesh that contains " << currentCloudPointsColored->points.size() << " vertices.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
            v_meshes.push_back(currentMesh);
            if (colorMap._leafColorMap_colorsToLabel.find(currentColor) != colorMap._leafColorMap_colorsToLabel.end()) {
                numberOfLeaves += 1;
            }
        }
    }

    // Then we process the mesh based on its color.
    for (uint32_t i = 0; i < v_meshes.size(); i++) {
        pcl::PolygonMesh currentMesh = v_meshes[i];
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

        pcl::fromPCLPointCloud2(currentMesh.cloud, *currentCloud);
        pcl::PointXYZRGBA singlePoint = currentCloud->points[0];

        TupleTriplet currentRGB(singlePoint.r, singlePoint.g, singlePoint.b);

        // If the mesh corresponds to a stem
        if (TupleTripletsAreEqual(currentRGB, colorMap._stem_color)) {
            pcl::PolygonMesh connectedMesh = returnLargestConnectedMesh(&currentMesh);
            makeStemMeasurements(&connectedMesh, inputMesh, inputMeasurementData, inputParams, visualizer);

            std::string fileOut = "segmentation_PLYs/stem.ply";
            logStream << "Writing stem out to " << fileOut;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");
            pcl::io::savePLYFile(fileOut, currentMesh);
            std::string fileOutConnected = "segmentation_PLYs/stem_largestConnected.ply";
            logStream << "Writing largest connected stem piece out to " << fileOutConnected;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");
            pcl::io::savePLYFile(fileOutConnected, connectedMesh);
            //visu->addPolygonMesh(currentMesh, fileOut, mesh_vp_2);
        } // Finished with stem processing.
        // If the mesh corresponds to a leaf
        else if (colorMap._leafColorMap_colorsToLabel.find(currentRGB) != colorMap._leafColorMap_colorsToLabel.end()) {
            pcl::PolygonMesh connectedMesh = returnLargestConnectedMesh(&currentMesh);
            makeIndividualLeafMeasurements(&connectedMesh, inputMesh, inputMeasurementData, inputParams, visualizer);

            uint32_t leafNumber = colorMap._leafColorMap_colorsToLabel[currentRGB];
            std::stringstream ss;
            ss << leafNumber;
            std::string fileOut = "segmentation_PLYs/leaf_" + ss.str() + ".ply";
            logStream << "Writing leaf out to " << fileOut;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");
            pcl::io::savePLYFile(fileOut, currentMesh);
            std::string fileOutConnected = "segmentation_PLYs/leaf_" + ss.str() + "_largestConnected.ply";
            logStream << "Writing largest connected leaf piece out to " << fileOutConnected;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");
            pcl::io::savePLYFile(fileOutConnected, connectedMesh);
            //visu->addPolygonMesh(currentMesh, fileOut, mesh_vp_2);
        } // Finished with leaf processing
        else if (currentRGB == colorMap._debug_color || currentRGB == colorMap._border_color) {
            LOG.DEBUG("WARNING: Debug color or border color encountered. Skipping this color and continuing.");
        }
        else if (colorMap._map_explicitlyHandledColors.find(currentRGB) != colorMap._map_explicitlyHandledColors.end()) {
            printTupleTriplet(currentRGB, inputParams.debuggingParameters.getDebuggingLevel(), 0);
            LOG.DEBUG("WARNING: The color in the mesh (printed above) does not yet have measurements implemented for it. Skipping this color and continuing.");
        }
        else {
            printTupleTriplet(currentRGB, inputParams.debuggingParameters.getDebuggingLevel(), 0);
            bool colorInTheMeshIsInColorMap = false;
            LOG.DEBUG("ERROR:");
            LOG.DEBUG("ERROR: The color in the mesh (printed above) is not explicitly handled. This likely indicates something is wrong.");
            LOG.DEBUG("ERROR:");
            assert(colorInTheMeshIsInColorMap == true && "The color in the mesh (printed above) is not expected in the color map. This likely indicates something is wrong. Aborting.");
        }
    }

    /// Find internode spacing using the base leaf points.
    { //begin scoping
    // Extract clouds from the input mesh for use downstream.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOfSegmentedMesh (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsOfSegmentedMeshColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::fromPCLPointCloud2(inputMesh->cloud, *cloudPointsOfSegmentedMesh);
    pcl::fromPCLPointCloud2(inputMesh->cloud, *cloudPointsOfSegmentedMeshColored);
    // Here, we estimate the internode distance by measuring the distance from each base leaf point. This desparately needs to be refactored.
    std::cout << "There are " << numberOfLeaves << " leaves to find internode distances for." << std::endl;
    for (uint32_t leafIndex = 0; leafIndex < numberOfLeaves; leafIndex++) {
        std::stringstream ss1;
        ss1 << leafIndex;
        std::stringstream ss2;
        ss2 << leafIndex + 1;
        std::string leafPair = ss1.str() + "-" + ss2.str();
        float internodeDistance = 0.0;
        if (leafIndex == 0) {  // If it's the first iteration, we need the bottom of the stem to the first leaf.
            TupleTriplet upperLeafColor = colorMap._leafColorMap[leafIndex + 1];
            pcl::PolygonMesh upperLeafMesh;
            // Find the meshes we need to operate on.
            for (uint32_t j = 0; j < v_meshes.size(); j++) {
                pcl::PolygonMesh currentMesh = v_meshes[j];
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::fromPCLPointCloud2(currentMesh.cloud, *currentCloud);
                pcl::PointXYZRGBA singlePoint = currentCloud->points[0];
                TupleTriplet currentRGB(singlePoint.r, singlePoint.g, singlePoint.b);
                if (currentRGB == upperLeafColor) {
                    upperLeafMesh = currentMesh;
                }
            }
            //Find the z distance between the base of the upper leaf mesh and the bottom of the stem.
            // First, we find the index on the minimum stem point.
            float minStemZCoord = 10000000.0; //Arbitrarily large magic number;
            uint32_t indexOfMinStemLabel = 0;
            for (uint32_t j = 0; j < cloudPointsOfSegmentedMeshColored->points.size(); j++) {
                pcl::PointXYZRGBA currentPoint = cloudPointsOfSegmentedMeshColored->points[j];
                TupleTriplet currentColor(currentPoint.r, currentPoint.g, currentPoint.b);
                float zCoord = currentPoint.z;
                if (currentColor == colorMap._stem_color) {
                    if (zCoord < minStemZCoord) {
                        minStemZCoord = zCoord;
                        indexOfMinStemLabel = j;
                    }
                }
            }
            // Get the connected mesh.
            pcl::PolygonMesh connectedMesh = returnLargestConnectedMesh(&upperLeafMesh);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOfSubMesh (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsOfSubMeshColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::fromPCLPointCloud2(connectedMesh.cloud, *cloudPointsOfSubMesh);
            pcl::fromPCLPointCloud2(connectedMesh.cloud, *cloudPointsOfSubMeshColored);
            // Identify the leaf "base".
            // The leaf base will be the point in the mesh to measure that is geodesically closest to the stem minimum.
            Graph meshGraph(inputMesh);
            DijkstraPathfinder leafBasePathfinder(meshGraph);
            uint32_t indexOfLeafMeshBase = leafBasePathfinder.returnIndexOfInputSubsetMeshClosestToStemBottom(&connectedMesh, inputMesh);
            pcl::PointXYZ bottomOfStem = cloudPointsOfSegmentedMesh->points[indexOfMinStemLabel];
            pcl::PointXYZ bottomOfLeaf = cloudPointsOfSubMesh->points[indexOfLeafMeshBase];
            internodeDistance = bottomOfLeaf.z - bottomOfStem.z;
            inputMeasurementData.addNameAndMeasurement("internodeDistance_" + leafPair, internodeDistance);
            std::cout << "Internode distance of leaf pair " << leafPair << " is : " << internodeDistance << std::endl;

        }
        else {
            TupleTriplet lowerLeafColor = colorMap._leafColorMap[leafIndex];
            TupleTriplet upperLeafColor = colorMap._leafColorMap[leafIndex + 1];
            pcl::PolygonMesh lowerLeafMesh;
            pcl::PolygonMesh upperLeafMesh;
            // Find the meshes we need to operate on.
            for (uint32_t j = 0; j < v_meshes.size(); j++) {
                pcl::PolygonMesh currentMesh = v_meshes[j];
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

                pcl::fromPCLPointCloud2(currentMesh.cloud, *currentCloud);
                pcl::PointXYZRGBA singlePoint = currentCloud->points[0];

                TupleTriplet currentRGB(singlePoint.r, singlePoint.g, singlePoint.b);
                if (currentRGB == lowerLeafColor) {
                    lowerLeafMesh = currentMesh;
                }
                if (currentRGB == upperLeafColor) {
                    upperLeafMesh = currentMesh;
                }
            }

            // First, we also need the min and max indices for the stem point.
            float minStemZCoord = 10000000.0; //Arbitrarily large magic number;
            float maxStemZCoord = -10000000.0; //Arbitrarily large magic number;
            uint32_t indexOfMinStemLabel = 0;
            uint32_t indexOfMaxStemLabel = 0;
            for (uint32_t j = 0; j < cloudPointsOfSegmentedMeshColored->points.size(); j++) {
                pcl::PointXYZRGBA currentPoint = cloudPointsOfSegmentedMeshColored->points[j];
                TupleTriplet currentColor(currentPoint.r, currentPoint.g, currentPoint.b);
                float zCoord = currentPoint.z;
                if (currentColor == colorMap._stem_color) {
                    if (zCoord < minStemZCoord) {
                        minStemZCoord = zCoord;
                        indexOfMinStemLabel = j;
                    }
                    if (zCoord > maxStemZCoord) {
                        maxStemZCoord = zCoord;
                        indexOfMaxStemLabel = j;
                    }
                }
            }

            // Get the connected mesh.
            pcl::PolygonMesh connectedLowerMesh = returnLargestConnectedMesh(&lowerLeafMesh);
            pcl::PolygonMesh connectedUpperMesh = returnLargestConnectedMesh(&upperLeafMesh);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOfLowerSubMesh (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsOfLowerSubMeshColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::fromPCLPointCloud2(connectedLowerMesh.cloud, *cloudPointsOfLowerSubMesh);
            pcl::fromPCLPointCloud2(connectedLowerMesh.cloud, *cloudPointsOfLowerSubMeshColored);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOfUpperSubMesh (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsOfUpperSubMeshColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::fromPCLPointCloud2(connectedUpperMesh.cloud, *cloudPointsOfUpperSubMesh);
            pcl::fromPCLPointCloud2(connectedUpperMesh.cloud, *cloudPointsOfUpperSubMeshColored);

            // Identify the leaf "base".
            // The leaf base will be the point in the mesh to measure that is geodesically closest to the stem minimum.
            Graph meshGraph(inputMesh);
            DijkstraPathfinder leafBasePathfinder(meshGraph);

            uint32_t indexOfLowerLeafMeshBase = leafBasePathfinder.returnIndexOfInputSubsetMeshClosestToStemBottom(&connectedLowerMesh, inputMesh);
            uint32_t indexOfUpperLeafMeshBase = leafBasePathfinder.returnIndexOfInputSubsetMeshClosestToStemBottom(&connectedUpperMesh, inputMesh);


            pcl::PointXYZ bottomOfLowerLeaf = cloudPointsOfLowerSubMesh->points[indexOfLowerLeafMeshBase];
            pcl::PointXYZ bottomOfUpperLeaf = cloudPointsOfUpperSubMesh->points[indexOfUpperLeafMeshBase];

            float bottomOfLowerLeafZ = bottomOfLowerLeaf.z;
            float bottomOfUpperLeafZ = bottomOfUpperLeaf.z;

            internodeDistance = bottomOfUpperLeafZ - bottomOfLowerLeafZ;

            // There are a few special cases we want to handle here.

            // One is where segmentation has created a case where leaf i + 1 has a smaller z coordinate than i.
            // We'll just set it the internode distance to 0.
            if (internodeDistance < 0.0) {
                std::cout << "Upper leaf base is lower than lower leaf base, setting distance to 0." << std::endl;
                internodeDistance = 0.0;
            }

            // A second case is where the bottom leaf is below the max of the stem, but the upper leaf is above it. In this case, we want to
            // set the upper leaf to the max stem height.
            if (bottomOfLowerLeafZ < maxStemZCoord && bottomOfUpperLeafZ > maxStemZCoord) {
                std::cout << "Upper leaf is above the stem max. Setting it to the stem max." << std::endl;
                bottomOfUpperLeafZ = maxStemZCoord;
                internodeDistance = bottomOfUpperLeafZ - bottomOfLowerLeafZ;
            }

            // A third case is where both the bottom leaf and the upper leaf are below the max of the stem (e.g. both whorl leaves).
            // in this case, we want to set the distance to 0.
            if (bottomOfLowerLeafZ > maxStemZCoord && bottomOfUpperLeafZ > maxStemZCoord) {
                std::cout << "Both leaves above the stem. Setting internode distance to 0." << std::endl;
                internodeDistance = 0.0;
            }

            inputMeasurementData.addNameAndMeasurement("internodeDistance_" + leafPair, internodeDistance);
            std::cout << "Internode distance of leaf pair " << leafPair << " is : " << internodeDistance << std::endl;
        }
    }
    } //end scoping for finding internode distance

    /// Let's take a few derived measurements, such as averaging leaf sets, etc.
    // To access the values from the measurement container, we'll need the phenotype names.
    // Perhaps phenotype names should be some sort of enum? For now we'll just use the string names until
    // the phenotypes we want to use are better defined.
    // We'll be most interested in leaves, and we also need to know the max leaf so we can go from top to bottom as well.
    // The derived phenotypes we want to take are:
    // Individual leaves measured in the reverse direction (e.g. leaf 1 is the max leaf, leaf 2 is max leaf - 1, etc.)
    // average length, width, etc. of leaves 2, 3, and 4, and 3, 4, and 5 for both directions.

    // This code assumes that the phenotype names stay constant from the leaf module; we'll need to write something better
    // in the long term since this will break if something changes there with respect to phenotype names.
    //inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_surface_area", surfaceArea);
    //inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_length", leafLength);
    //inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_width", leafWidth);
    //inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_angleAtPercentLength_" + proportionSS.str(), angleDeg);
    //inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_angleDiff1-025", angleDiffQuarterAndOne);
    //inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_angleAtFixedLength_" + fixedSS.str(), angleDeg);
    //inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_angleDiff1-Fixed", angleDiffFixedAndOne);

    // First, we'll get the leaf traits, but counting from the top down instead of the bottom up.
    // this is sometimes more developmentally relevant.
    // For example:
    // Bottom up: 1 2 3 4 5  <-- this is how they are coded in the color map and phenotype names.
    // Top down:  5 4 3 2 1
    { //begin scope for reverse counting.
    int topDownLeafNumber = 0;
    std::map<std::string, float> phenotypeMap = inputMeasurementData.getMeasurementData();
    for (uint32_t leafIndex = numberOfLeaves; leafIndex > 0; leafIndex--) {
        topDownLeafNumber += 1;
        // Leaf index is the value of the leaf we want.
        std::stringstream originalLeafSS;
        originalLeafSS << leafIndex;
        std::string originalLeafIDstr = originalLeafSS.str();
        std::stringstream reverseLeafSS;
        reverseLeafSS << topDownLeafNumber;
        std::string reverseLeafIDstr = reverseLeafSS.str();

        std::string surfaceAreaString = "leaf_" + originalLeafIDstr + "_surface_area";
        std::string leafLengthString = "leaf_" + originalLeafIDstr + "_length";
        std::string leafWidthString = "leaf_" + originalLeafIDstr + "_width";
        std::string leafAnglePropString = "leaf_" + originalLeafIDstr + "_angleAtPercentLength_25";
        std::string leafAngleFixedString = "leaf_" + originalLeafIDstr + "_angleAtFixedLength_76";

        if (phenotypeMap.find(surfaceAreaString) != phenotypeMap.end()) {
            inputMeasurementData.addNameAndMeasurement("leafRevOrder_" + reverseLeafIDstr + "_surface_area", phenotypeMap[surfaceAreaString]);
        }
        if (phenotypeMap.find(leafLengthString) != phenotypeMap.end()) {
            inputMeasurementData.addNameAndMeasurement("leafRevOrder_" + reverseLeafIDstr + "_length", phenotypeMap[leafLengthString]);
        }
        if (phenotypeMap.find(leafWidthString) != phenotypeMap.end()) {
            inputMeasurementData.addNameAndMeasurement("leafRevOrder_" + reverseLeafIDstr + "_width", phenotypeMap[leafWidthString]);
        }
        if (phenotypeMap.find(leafAnglePropString) != phenotypeMap.end()) {
            inputMeasurementData.addNameAndMeasurement("leafRevOrder_" + reverseLeafIDstr + "_angleAtPercentLength_25", phenotypeMap[leafAnglePropString]);
        }
        if (phenotypeMap.find(leafAngleFixedString) != phenotypeMap.end()) {
            inputMeasurementData.addNameAndMeasurement("leafRevOrder_" + reverseLeafIDstr + "_angleAtFixedLength_76", phenotypeMap[leafAngleFixedString]);
        }
    }
    } // end scope for reverse counting.

    // Next we'll take the average of leaves 2, 3, and 4 counting bottom up.
    { // begin scope for average of 2, 3, and 4
    std::map<std::string, float> phenotypeMap = inputMeasurementData.getMeasurementData();
    float areaSum = 0;
    float areaCounter = 0;
    float lengthSum = 0;
    float lengthCounter = 0;
    float widthSum = 0;
    float widthCounter = 0;
    float anglePropSum = 0;
    float anglePropCounter = 0;
    float angleFixedSum = 0;
    float angleFixedCounter = 0;
    for (uint32_t leafIndex = 2; leafIndex <= 4; leafIndex++) {
        std::stringstream originalLeafSS;
        originalLeafSS << leafIndex;
        std::string originalLeafIDstr = originalLeafSS.str();

        std::string surfaceAreaString = "leaf_" + originalLeafIDstr + "_surface_area";
        std::string leafLengthString = "leaf_" + originalLeafIDstr + "_length";
        std::string leafWidthString = "leaf_" + originalLeafIDstr + "_width";
        std::string leafAnglePropString = "leaf_" + originalLeafIDstr + "_angleAtPercentLength_25";
        std::string leafAngleFixedString = "leaf_" + originalLeafIDstr + "_angleAtFixedLength_76";

        if (phenotypeMap.find(surfaceAreaString) != phenotypeMap.end()) {
            areaSum += phenotypeMap[surfaceAreaString];
            areaCounter += 1;
        }
        if (phenotypeMap.find(leafLengthString) != phenotypeMap.end()) {
            lengthSum += phenotypeMap[leafLengthString];
            lengthCounter += 1;
        }
        if (phenotypeMap.find(leafWidthString) != phenotypeMap.end()) {
            widthSum += phenotypeMap[leafWidthString];
            widthCounter += 1;
        }
        if (phenotypeMap.find(leafAnglePropString) != phenotypeMap.end()) {
            anglePropSum += phenotypeMap[leafAnglePropString];
            anglePropCounter += 1;
        }
        if (phenotypeMap.find(leafAngleFixedString) != phenotypeMap.end()) {
            angleFixedSum += phenotypeMap[leafAngleFixedString];
            angleFixedCounter += 1;
        }
    }
    if (areaCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_234_surface_area", (areaSum / areaCounter) ); }
    if (lengthCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_234_length", (lengthSum / lengthCounter) ); }
    if (widthCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_234_width", (widthSum / widthCounter) ); }
    if (anglePropCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_234_angleAtPercentLength_25", (anglePropSum / anglePropCounter) ); }
    if (angleFixedCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_234_angleAtFixedLength_76", (angleFixedSum / angleFixedCounter) ); }
    } // end scope for average of 2, 3, and 4

    // Next we'll take the average of leaves 3, 4, and 5 counting bottom up.
    { // begin scope for average of 3, 4, and 5
    std::map<std::string, float> phenotypeMap = inputMeasurementData.getMeasurementData();
    float areaSum = 0;
    float areaCounter = 0;
    float lengthSum = 0;
    float lengthCounter = 0;
    float widthSum = 0;
    float widthCounter = 0;
    float anglePropSum = 0;
    float anglePropCounter = 0;
    float angleFixedSum = 0;
    float angleFixedCounter = 0;
    for (uint32_t leafIndex = 3; leafIndex <= 5; leafIndex++) {
        std::stringstream originalLeafSS;
        originalLeafSS << leafIndex;
        std::string originalLeafIDstr = originalLeafSS.str();

        std::string surfaceAreaString = "leaf_" + originalLeafIDstr + "_surface_area";
        std::string leafLengthString = "leaf_" + originalLeafIDstr + "_length";
        std::string leafWidthString = "leaf_" + originalLeafIDstr + "_width";
        std::string leafAnglePropString = "leaf_" + originalLeafIDstr + "_angleAtPercentLength_25";
        std::string leafAngleFixedString = "leaf_" + originalLeafIDstr + "_angleAtFixedLength_76";

        if (phenotypeMap.find(surfaceAreaString) != phenotypeMap.end()) {
            areaSum += phenotypeMap[surfaceAreaString];
            areaCounter += 1;
        }
        if (phenotypeMap.find(leafLengthString) != phenotypeMap.end()) {
            lengthSum += phenotypeMap[leafLengthString];
            lengthCounter += 1;
        }
        if (phenotypeMap.find(leafWidthString) != phenotypeMap.end()) {
            widthSum += phenotypeMap[leafWidthString];
            widthCounter += 1;
        }
        if (phenotypeMap.find(leafAnglePropString) != phenotypeMap.end()) {
            anglePropSum += phenotypeMap[leafAnglePropString];
            anglePropCounter += 1;
        }
        if (phenotypeMap.find(leafAngleFixedString) != phenotypeMap.end()) {
            angleFixedSum += phenotypeMap[leafAngleFixedString];
            angleFixedCounter += 1;
        }
    }
    if (areaCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_345_surface_area", (areaSum / areaCounter) ); }
    if (lengthCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_345_length", (lengthSum / lengthCounter) ); }
    if (widthCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_345_width", (widthSum / widthCounter) ); }
    if (anglePropCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_345_angleAtPercentLength_25", (anglePropSum / anglePropCounter) ); }
    if (angleFixedCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_345_angleAtFixedLength_76", (angleFixedSum / angleFixedCounter) ); }
    } // end scope for average of 3, 4, and 5

    // Now we'll get do averages again, but counting from the top down instead of the bottom up.
    // this is sometimes more developmentally relevant.
    // For example:
    // Bottom up: 1 2 3 4 5  <-- this is how they are coded in the color map and phenotype names.
    // Top down:  5 4 3 2 1
    { //begin scope for averages of reverse counting.
    int topDownLeafNumber = 0;
    std::map<std::string, float> phenotypeMap = inputMeasurementData.getMeasurementData();
    float areaSum = 0;
    float areaCounter = 0;
    float lengthSum = 0;
    float lengthCounter = 0;
    float widthSum = 0;
    float widthCounter = 0;
    float anglePropSum = 0;
    float anglePropCounter = 0;
    float angleFixedSum = 0;
    float angleFixedCounter = 0;
    for (uint32_t leafIndex = numberOfLeaves; leafIndex > 0; leafIndex--) {
        topDownLeafNumber += 1;
        // Leaf index is the value of the leaf we want.
        std::stringstream originalLeafSS;
        originalLeafSS << leafIndex;
        std::string originalLeafIDstr = originalLeafSS.str();
        std::stringstream reverseLeafSS;
        reverseLeafSS << topDownLeafNumber;
        std::string reverseLeafIDstr = reverseLeafSS.str();

        std::string surfaceAreaString = "leaf_" + originalLeafIDstr + "_surface_area";
        std::string leafLengthString = "leaf_" + originalLeafIDstr + "_length";
        std::string leafWidthString = "leaf_" + originalLeafIDstr + "_width";
        std::string leafAnglePropString = "leaf_" + originalLeafIDstr + "_angleAtPercentLength_25";
        std::string leafAngleFixedString = "leaf_" + originalLeafIDstr + "_angleAtFixedLength_76";

        if (topDownLeafNumber >= 4 && topDownLeafNumber <= 6) {
            if (phenotypeMap.find(surfaceAreaString) != phenotypeMap.end()) {
                areaSum += phenotypeMap[surfaceAreaString];
                areaCounter += 1;
            }
            if (phenotypeMap.find(leafLengthString) != phenotypeMap.end()) {
                lengthSum += phenotypeMap[leafLengthString];
                lengthCounter += 1;
            }
            if (phenotypeMap.find(leafWidthString) != phenotypeMap.end()) {
                widthSum += phenotypeMap[leafWidthString];
                widthCounter += 1;
            }
            if (phenotypeMap.find(leafAnglePropString) != phenotypeMap.end()) {
                anglePropSum += phenotypeMap[leafAnglePropString];
                anglePropCounter += 1;
            }
            if (phenotypeMap.find(leafAngleFixedString) != phenotypeMap.end()) {
                angleFixedSum += phenotypeMap[leafAngleFixedString];
                angleFixedCounter += 1;
            }
        }
    }
    if (areaCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder456_surface_area", (areaSum / areaCounter) ); }
    if (lengthCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder456_length", (lengthSum / lengthCounter) ); }
    if (widthCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder456_width", (widthSum / widthCounter) ); }
    if (anglePropCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder456_angleAtPercentLength_25", (anglePropSum / anglePropCounter) ); }
    if (angleFixedCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder456_angleAtFixedLength_76", (angleFixedSum / angleFixedCounter) ); }
    } // end scope for averages of reverse counting.

    { //begin scope for averages of reverse counting, but for 5, 6, and 7
    int topDownLeafNumber = 0;
    std::map<std::string, float> phenotypeMap = inputMeasurementData.getMeasurementData();
    float areaSum = 0;
    float areaCounter = 0;
    float lengthSum = 0;
    float lengthCounter = 0;
    float widthSum = 0;
    float widthCounter = 0;
    float anglePropSum = 0;
    float anglePropCounter = 0;
    float angleFixedSum = 0;
    float angleFixedCounter = 0;
    for (uint32_t leafIndex = numberOfLeaves; leafIndex > 0; leafIndex--) {
        topDownLeafNumber += 1;
        // Leaf index is the value of the leaf we want.
        std::stringstream originalLeafSS;
        originalLeafSS << leafIndex;
        std::string originalLeafIDstr = originalLeafSS.str();
        std::stringstream reverseLeafSS;
        reverseLeafSS << topDownLeafNumber;
        std::string reverseLeafIDstr = reverseLeafSS.str();

        std::string surfaceAreaString = "leaf_" + originalLeafIDstr + "_surface_area";
        std::string leafLengthString = "leaf_" + originalLeafIDstr + "_length";
        std::string leafWidthString = "leaf_" + originalLeafIDstr + "_width";
        std::string leafAnglePropString = "leaf_" + originalLeafIDstr + "_angleAtPercentLength_25";
        std::string leafAngleFixedString = "leaf_" + originalLeafIDstr + "_angleAtFixedLength_76";

        if (topDownLeafNumber >= 5 && topDownLeafNumber <= 7) {
            if (phenotypeMap.find(surfaceAreaString) != phenotypeMap.end()) {
                areaSum += phenotypeMap[surfaceAreaString];
                areaCounter += 1;
            }
            if (phenotypeMap.find(leafLengthString) != phenotypeMap.end()) {
                lengthSum += phenotypeMap[leafLengthString];
                lengthCounter += 1;
            }
            if (phenotypeMap.find(leafWidthString) != phenotypeMap.end()) {
                widthSum += phenotypeMap[leafWidthString];
                widthCounter += 1;
            }
            if (phenotypeMap.find(leafAnglePropString) != phenotypeMap.end()) {
                anglePropSum += phenotypeMap[leafAnglePropString];
                anglePropCounter += 1;
            }
            if (phenotypeMap.find(leafAngleFixedString) != phenotypeMap.end()) {
                angleFixedSum += phenotypeMap[leafAngleFixedString];
                angleFixedCounter += 1;
            }
        }
    }
    if (areaCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder567_surface_area", (areaSum / areaCounter) ); }
    if (lengthCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder567_length", (lengthSum / lengthCounter) ); }
    if (widthCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder567_width", (widthSum / widthCounter) ); }
    if (anglePropCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder567_angleAtPercentLength_25", (anglePropSum / anglePropCounter) ); }
    if (angleFixedCounter > 0) { inputMeasurementData.addNameAndMeasurement("leafAvg_revOrder567_angleAtFixedLength_76", (angleFixedSum / angleFixedCounter) ); }
    } // end scope for averages of reverse counting.

    return 0;
}

int makeStemMeasurements(pcl::PolygonMesh *meshToMeasure, pcl::PolygonMesh *originalMesh, MeasurementDataContainer &inputMeasurementData, InputParameters inputParams, pcl::visualization::PCLVisualizer *visualizer) {
    std::cout << "Making measurements of the stem" << std::endl;
    int originalViewport = 1;
    int measurementViewport = 2;
    int featureViewport = 3;
    std::string originalViewportString = "orignalMesh";
    std::string measurementViewportString = "measurementMesh";
    std::string featureViewportString = "featureMesh";

    // Extract clouds from the input mesh for use downstream.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::fromPCLPointCloud2(meshToMeasure->cloud, *cloudPoints);
    pcl::fromPCLPointCloud2(meshToMeasure->cloud, *cloudPointsColored);

    /// Geodesic diameter of stem
    Graph stemGraph(meshToMeasure);
    DijkstraPathfinder stemPathfinder(stemGraph);

    PathDataContainer geodesicDiameter = stemPathfinder.findGeodesicDiameter();

    std::cout << "Geodesic diameter of stem: " << geodesicDiameter._distance << std::endl;
    inputMeasurementData.addNameAndMeasurement("stem_maxGeodesicLength", geodesicDiameter._distance);

    /// Bounding box dimensions of stem
    BoundingBox boundingBox;
    BoundingBoxMaker<pcl::PointXYZRGBA> boundingBoxMaker;

    boundingBox = boundingBoxMaker.returnBoundingBoxOfCloud(cloudPointsColored);

    std::cout << std::endl << "oriented bbox width: " << boundingBox.width << std::endl;
    std::cout << "oriented bbox height: " << boundingBox.height << std::endl;
    std::cout << "oriented bbox depth: " << boundingBox.depth << std::endl;
    inputMeasurementData.addNameAndMeasurement("stem_oriented_bbox_width", boundingBox.width);
    inputMeasurementData.addNameAndMeasurement("stem_oriented_bbox_height", boundingBox.height);
    inputMeasurementData.addNameAndMeasurement("stem_oriented_bbox_depth", boundingBox.depth);

    //visu->addCube(boundingBox.bboxTransform, boundingBox.bboxQuaternion, boundingBox.width, boundingBox.height, boundingBox.depth, "bbox", mesh_vp_1);

    AxisAlignedBoundingBox axisAlignedBoundingBox;
    axisAlignedBoundingBox = boundingBoxMaker.returnAxisAlignedBoundingBoxOfCloud(cloudPointsColored);

    //visu->addCube(axisAlignedBoundingBox.minX, axisAlignedBoundingBox.maxX,
    //               axisAlignedBoundingBox.minY, axisAlignedBoundingBox.maxY,
    //                axisAlignedBoundingBox.minZ, axisAlignedBoundingBox.maxZ, 1.0, 1.0, 1.0, "aligned_bbox", mesh_vp_2);

    std::cout << std::endl << "axis aligned bbox x distance: " << axisAlignedBoundingBox.maxX - axisAlignedBoundingBox.minX << std::endl;
    std::cout << "axis aligned bbox y distance: " << axisAlignedBoundingBox.maxY - axisAlignedBoundingBox.minY << std::endl;
    std::cout << "axis aligned bbox z distance: " << axisAlignedBoundingBox.maxZ - axisAlignedBoundingBox.minZ << std::endl;
    inputMeasurementData.addNameAndMeasurement("stem_axisAligned_bbox_xDistance", axisAlignedBoundingBox.maxX - axisAlignedBoundingBox.minX);
    inputMeasurementData.addNameAndMeasurement("stem_axisAligned_bbox_yDistance", axisAlignedBoundingBox.maxY - axisAlignedBoundingBox.minY);
    inputMeasurementData.addNameAndMeasurement("stem_axisAligned_bbox_zDistance", axisAlignedBoundingBox.maxZ - axisAlignedBoundingBox.minZ);

    /// Surface area of stem
    { //scoping
    std::cout << "Number of polygons used for measuring surface area " << meshToMeasure->polygons.size() << std::endl;
    float surfaceArea = returnSurfaceAreaOfMesh(meshToMeasure);
    std::cout << "Total area: " << surfaceArea << " mm^2 " << " or " << surfaceArea/100.0 << " cm^2" << std::endl;
    inputMeasurementData.addNameAndMeasurement("stem_total_surface_area", surfaceArea);
    }

    /// Convex hull of stem
    { // scoping
    std::cout << std::endl << "Constructing the convex hull of stem. " << std::endl;
    std::vector<pcl::Vertices> convexHullVertices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> convexHullMaker;
    convexHullMaker.setInputCloud(cloudPoints);
    convexHullMaker.setComputeAreaVolume(true);
    convexHullMaker.reconstruct(*convexHull, convexHullVertices);

    double convexHullArea = 0;
    double convexHullVolume = 0;
    convexHullArea = convexHullMaker.getTotalArea();
    convexHullVolume = convexHullMaker.getTotalVolume();

    std::cout << "Area of the convex hull in mm^2: " << convexHullArea << std::endl;
    std::cout << "Volume of the convex hull in mm^3: " << convexHullVolume << std::endl;
    inputMeasurementData.addNameAndMeasurement("stem_convexHullArea", convexHullArea);
    inputMeasurementData.addNameAndMeasurement("stem_convexHullVolume", convexHullVolume);

    } // end scope

    /// RANSAC of stem to find radius. Let's consider modifying this to find multiple stem layers up the stem instead of one large cylinder.
    { // Begin scope for RANSAC
    // I don't think we should assume that the points have normals at this point, so we'll recalculate them.
    std::cout << "Estimating stem normals with K search with the following parameters." << std::endl;
    inputParams.normalEstimationParameters.printParameters();

    pcl::PointCloud<pcl::Normal>::Ptr stemNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> nest;
    nest.setSearchMethod(treeNormal);
    nest.setKSearch(inputParams.normalEstimationParameters.getKSearch());
    nest.setInputCloud(cloudPoints);
    nest.compute(*stemNormals);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (inputParams.sacSegmentationFromNormalsParameters.getMaxIterations());
    seg.setDistanceThreshold (inputParams.sacSegmentationFromNormalsParameters.getDistanceThreshold());
    seg.setNormalDistanceWeight (inputParams.sacSegmentationFromNormalsParameters.getNormalDistanceWeight());
    seg.setRadiusLimits (inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMin(),
                        inputParams.sacSegmentationFromNormalsParameters.getRadiusLimitsMax());
    seg.setInputCloud(cloudPoints);
    seg.setInputNormals(stemNormals);

    std::cout << std::endl << "Performing segmentation to identify a cylinder corresponding to the stem with the following parameters:" << std::endl;
    inputParams.sacSegmentationFromNormalsParameters.printParameters();

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visualizer->updatePointCloud(cloudPoints, featureViewportString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            std::cout << "Displaying the mesh to be RANSACed. Press q to continue." << std::endl;
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }


    seg.segment (*inliers, *coefficients);
    std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

    assert(inliers->indices.size() > 0 && "RANSAC unable to identify a cylinder for the stem. Verify stem and parameters.");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloudPoints);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_cylinder);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visualizer->updatePointCloud(cloud_cylinder, featureViewportString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            std::cout << "Displaying the stem cylinder identified with RANSAC. Press q to continue." << std::endl;
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    Eigen::VectorXf vCoefficients(7);
    std::cerr << "Model coefficients:" << std::endl;
    for (size_t i = 0; i < coefficients->values.size(); i++) {
            std::cerr << coefficients->values[i] << std::endl;
            vCoefficients[i] = coefficients->values[i];
    }

    std::cout << "stem radius and diameter calculated to be: " << vCoefficients[6] << " and " << vCoefficients[6] * 2 << std::endl;

    inputMeasurementData.addNameAndMeasurement("stem_radius", vCoefficients[6]);
    inputMeasurementData.addNameAndMeasurement("stem_diameter", vCoefficients[6] * 2);
    } // End scope for RANSAC
    return 0;
}

int makeIndividualLeafMeasurements(pcl::PolygonMesh *meshToMeasure, pcl::PolygonMesh *originalMesh, MeasurementDataContainer &inputMeasurementData, InputParameters inputParams, pcl::visualization::PCLVisualizer *visualizer) {
    std::ostringstream logStream;
    LOG.DEBUG("Making measurements of an individual leaf");

    ColorMap colorMap;
    int originalViewport = 1;
    int measurementViewport = 2;
    int featureViewport = 3;
    std::string originalViewportString = "orignalMesh";
    std::string measurementViewportString = "measurementMesh";
    std::string featureViewportString = "featureMesh";

    // Extract clouds from the input mesh for use downstream.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOfMeshToMeasure (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsOfMeshToMeasureColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::fromPCLPointCloud2(meshToMeasure->cloud, *cloudPointsOfMeshToMeasure);
    pcl::fromPCLPointCloud2(meshToMeasure->cloud, *cloudPointsOfMeshToMeasureColored);
    pcl::PointXYZRGBA singleColoredPoint = cloudPointsOfMeshToMeasureColored->points[0];
    TupleTriplet leafColor(singleColoredPoint.r, singleColoredPoint.g, singleColoredPoint.b);
    uint32_t leafID = colorMap._leafColorMap_colorsToLabel[leafColor];
    std::stringstream ss;
    ss << leafID;
    std::string leafIDstr = ss.str();


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsOfOriginalMesh (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsOfOriginalMeshColored (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::fromPCLPointCloud2(originalMesh->cloud, *cloudPointsOfOriginalMesh);
    pcl::fromPCLPointCloud2(originalMesh->cloud, *cloudPointsOfOriginalMeshColored);

    if (inputParams.debuggingParameters.getDebuggingLevel() > 0) {
        visualizer->updatePointCloud(cloudPointsOfMeshToMeasureColored, featureViewportString);
        if (inputParams.debuggingParameters.getDebuggingLevel() > 1) {
            LOG.DEBUG("Displaying the individual leaf to be processed. Press q to continue.");
            visualizer->spin();
        }
        else {
            visualizer->spinOnce();
        }
    }

    // I think the first task is to identify the leaf "base" so that we can use it to measure angle and length.
    // The leaf base will be the point in the mesh to measure that is geodesically closest to the stem minimum.
    Graph originalMeshGraph(originalMesh);
    DijkstraPathfinder leafBasePathfinder(originalMeshGraph);
    uint32_t indexOfLeafMeshBase = leafBasePathfinder.returnIndexOfInputSubsetMeshClosestToStemBottom(meshToMeasure, originalMesh);

    /// Surface area, length, width, and angles of leaf
    { //scoping

    logStream << "Number of polygons used for measuring surface area " << meshToMeasure->polygons.size();
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 1); logStream.str("");
    float surfaceArea = returnSurfaceAreaOfMesh(meshToMeasure);
    logStream << "Total area of leaf_" + leafIDstr + ": " << surfaceArea << " mm^2 " << " or " << surfaceArea/100.0 << " cm^2";
    LOG.DEBUG(logStream.str()); logStream.str("");
    inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_surface_area", surfaceArea);

    Graph leafGraph(meshToMeasure);
    DijkstraPathfinder leafPathfinder(leafGraph);
    PathDataContainer leafLengthPath = leafPathfinder.findPathToMostDistantNodeFromInputSource(indexOfLeafMeshBase);
    float leafLength = leafLengthPath._distance;
    logStream << "Leaf length estimated to be: " << leafLength;
    LOG.DEBUG(logStream.str()); logStream.str("");
    inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_length", leafLength);

    // For leaf width, we're going to make a very gross approximation, assuming that the leaf is a rectangle, and it's height is 1cm (10mm).
    // 2(w*h + w*l + l*h) = A
    // w = (A - 2lh) / (2h + 2l)
    float leafHeight = 10.0; // Assumed magic number based on hand measurements of meshes
    float numerator = surfaceArea - 2.0 * leafLength * leafHeight;
    float denomenator = 2 * leafHeight + 2 * leafLength;
    float leafWidth = numerator / denomenator;
    logStream << "Width estimated to be: " << leafWidth;
    LOG.DEBUG(logStream.str()); logStream.str("");
    inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_width", leafWidth);

    LOG.DEBUG("Attemtping to find angles.");
    logStream << "The original leaf length path is ";
    for (uint32_t i = 0; i < leafLengthPath._pathVectorOfLabels.size(); i++) {
        logStream << leafLengthPath._pathVectorOfLabels[i] << " ";
    }
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

    pcl::PointXYZ pclBaseLeafPoint = cloudPointsOfMeshToMeasure->points[leafLengthPath._pathVectorOfLabels[0]];

    std::vector<float> v_proportionsToFind = {0.10, 0.25, 0.50, 0.75, 1.0}; //These are hardcoded in for now.
    //Crude measure of curvature will be the difference between the angle at 25% up the leaf and 100% up the leaf.
    float angleQuarter = 0.0;
    float angleOne = 0.0;
    float angleFixed = 0.0;
    float angleDiffQuarterAndOne = 0.0;
    float angleDiffFixedAndOne = 0.0;
    // Find the points in the path where different proportions of distance have been covered.
    for (uint32_t i = 0; i < v_proportionsToFind.size(); i++) {
        pcl::PointXYZ zAlignedPointAboveBaseLeafPoint = pclBaseLeafPoint; // Initialize to the base point. We'll modify it below.
        float proportionToFind = v_proportionsToFind[i];
        float totalPathLength = leafLengthPath._distance;
        float currentPathLength = 0;
        float targetLength = proportionToFind * totalPathLength;
        for (uint32_t j = 0; j < leafLengthPath._pathVectorOfLabels.size() - 1; j++) {
            logStream << "Attempting to access elements j and j + 1:\t" << j << "\t" << j + 1<< ". path size is " << leafLengthPath._pathVectorOfLabels.size();
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            pcl::PointXYZ firstPoint = cloudPointsOfMeshToMeasure->points[leafLengthPath._pathVectorOfLabels[j]];
            pcl::PointXYZ secondPoint = cloudPointsOfMeshToMeasure->points[leafLengthPath._pathVectorOfLabels[j + 1]];
            float pairwiseDistance = pcl::euclideanDistance(firstPoint, secondPoint);
            currentPathLength = currentPathLength + pairwiseDistance;
            // Handle the edge case where the float comparison causes the 1.0 proportion to not be >= targetLength.
            if (j + 1 == leafLengthPath._pathVectorOfLabels.size() - 1) {
                LOG.DEBUG("Reached the end of the path. Setting current path length to the maximum.");
                currentPathLength = targetLength;
            }
            if (currentPathLength >= targetLength) {
                //Calculate curvature here. This is dependent on the stem being aligned with the z axis.
                pcl::PointXYZ targetLeafPoint = cloudPointsOfMeshToMeasure->points[leafLengthPath._pathVectorOfLabels[j + 1]];
                logStream << "Current path length: " << currentPathLength << " is greater than the target: " << targetLength << std::endl;
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                float zDifference = targetLeafPoint.z - pclBaseLeafPoint.z;
                // If zDifference is greater than or equal to 0, the target point is above the base point. Move the zAligned point in the same direction.
                // The angle will be less than or equal to 90 degrees.
                if (zDifference >= 0) {
                    zAlignedPointAboveBaseLeafPoint.z = pclBaseLeafPoint.z + zDifference;
                }
                // If zDifference is less than 0, the target point is below the base point. Move the zAligned point in the opposite direction.
                // The angle will be greater than 90 degrees.
                else {
                    zAlignedPointAboveBaseLeafPoint.z = pclBaseLeafPoint.z + (-1.0 * zDifference);
                }
                logStream << "Calculating curvature based on points: " << std::endl <<
                    "Base Point:\t\t" << pclBaseLeafPoint << std::endl <<
                    "zAligned Point:\t\t" << zAlignedPointAboveBaseLeafPoint << std::endl <<
                    "Target Point:\t\t" << targetLeafPoint;
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                // We have all three points and now we're going to find the angle between lines base-zAligned and base-target.
                float dist_basePoint_zAlignedPoint = pcl::euclideanDistance(pclBaseLeafPoint, zAlignedPointAboveBaseLeafPoint);
                float dist_basePoint_targetPoint = pcl::euclideanDistance(pclBaseLeafPoint, targetLeafPoint);
                float dist_zAlignedPoint_targetPoint = pcl::euclideanDistance(zAlignedPointAboveBaseLeafPoint, targetLeafPoint);

                // http://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
                //arccos( (P12 * P12 + P13 * P13 - P23 * P23) / (2 * P12 * P13) )
                float numerator = dist_basePoint_zAlignedPoint * dist_basePoint_zAlignedPoint +
                                        dist_basePoint_targetPoint * dist_basePoint_targetPoint -
                                        dist_zAlignedPoint_targetPoint * dist_zAlignedPoint_targetPoint;
                float denominator = 2.0 * dist_basePoint_zAlignedPoint * dist_basePoint_targetPoint;
                float angleDeg = acos(numerator / denominator) * 180.0 / M_PI;
                logStream << "Proportional angle (" << proportionToFind << ") between base-zAligned and base_target calculated to be: " << angleDeg << "." << std::endl;
                LOG.DEBUG(logStream.str()); logStream.str("");
                std::stringstream proportionSS;
                proportionSS << proportionToFind * 100;
                inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_angleAtPercentLength_" + proportionSS.str(), angleDeg);

                if (v_proportionsToFind[i] == 0.25) {
                    angleQuarter = angleDeg;
                }
                if (v_proportionsToFind[i] == 1.0) {
                    angleOne = angleDeg;
                }

                break;
            }
        }
    }
    angleDiffQuarterAndOne = angleOne - angleQuarter;
    inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_angleDiff1-025", angleDiffQuarterAndOne);

    // We repeat some of the code above to find an angle based on the absolute distance away from the base point.
    float targetLength = 76.2; // magic number of mm for leaf angle measurements; corresponds to 3 in.
    float currentPathLength = 0;
    for (uint32_t j = 0; j < leafLengthPath._pathVectorOfLabels.size() - 1; j++) {
        logStream << "Attempting to access elements j and j + 1:\t" << j << "\t" << j + 1 << ". path size is " << leafLengthPath._pathVectorOfLabels.size();
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        pcl::PointXYZ zAlignedPointAboveBaseLeafPoint = pclBaseLeafPoint; // Initialize to the base point. We'll modify it below.
        pcl::PointXYZ firstPoint = cloudPointsOfMeshToMeasure->points[leafLengthPath._pathVectorOfLabels[j]];
        pcl::PointXYZ secondPoint = cloudPointsOfMeshToMeasure->points[leafLengthPath._pathVectorOfLabels[j + 1]];
        float pairwiseDistance = pcl::euclideanDistance(firstPoint, secondPoint);
        currentPathLength = currentPathLength + pairwiseDistance;
        // Handle the edge case that the leaf is shorter than the target length.
        if (j + 1 == leafLengthPath._pathVectorOfLabels.size() - 1) {
            LOG.DEBUG("Reached the end of the path. Setting current path length to the maximum.");
            currentPathLength = targetLength;
        }
        if (currentPathLength >= targetLength) {
            //Calculate curvature here. This is dependent on the stem being aligned with the z axis.
            pcl::PointXYZ targetLeafPoint = cloudPointsOfMeshToMeasure->points[leafLengthPath._pathVectorOfLabels[j + 1]];
            logStream << "Pairwise distance: " << currentPathLength << " is greater than the target: " << targetLength << std::endl;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            float zDifference = targetLeafPoint.z - pclBaseLeafPoint.z;
            // If zDifference is greater than or equal to 0, the target point is above the base point. Move the zAligned point in the same direction.
            // The angle will be less than or equal to 90 degrees.
            if (zDifference >= 0) {
                zAlignedPointAboveBaseLeafPoint.z = pclBaseLeafPoint.z + zDifference;
            }
            // If zDifference is less than 0, the target point is below the base point. Move the zAligned point in the opposite direction.
            // The angle will be greater than 90 degrees.
            else {
                zAlignedPointAboveBaseLeafPoint.z = pclBaseLeafPoint.z + (-1.0 * zDifference);
            }
            logStream << "Calculating curvature based on points: " << std::endl <<
                "Base Point:\t\t" << pclBaseLeafPoint << std::endl <<
                "zAligned Point:\t\t" << zAlignedPointAboveBaseLeafPoint << std::endl <<
                "Target Point:\t\t" << targetLeafPoint;
            LOG.DEBUG(logStream.str()); logStream.str("");

            // We have all three points and now we're going to find the angle between lines base-zAligned and base-target.
            float dist_basePoint_zAlignedPoint = pcl::euclideanDistance(pclBaseLeafPoint, zAlignedPointAboveBaseLeafPoint);
            float dist_basePoint_targetPoint = pcl::euclideanDistance(pclBaseLeafPoint, targetLeafPoint);
            float dist_zAlignedPoint_targetPoint = pcl::euclideanDistance(zAlignedPointAboveBaseLeafPoint, targetLeafPoint);

            // http://stackoverflow.com/questions/1211212/how-to-calculate-an-angle-from-three-points
            //arccos( (P12 * P12 + P13 * P13 - P23 * P23) / (2 * P12 * P13) )
            float numerator = dist_basePoint_zAlignedPoint * dist_basePoint_zAlignedPoint +
                                        dist_basePoint_targetPoint * dist_basePoint_targetPoint -
                                        dist_zAlignedPoint_targetPoint * dist_zAlignedPoint_targetPoint;
            float denomenator = 2.0 * dist_basePoint_zAlignedPoint * dist_basePoint_targetPoint;
            float angleDeg = acos(numerator / denomenator) * 180.0 / M_PI;
            logStream << "Fixed distance angle between base-zAligned and base_target calculated to be: " << angleDeg << std::endl;
            LOG.DEBUG(logStream.str()); logStream.str("");
            std::stringstream fixedSS;
            fixedSS << round(targetLength);
            inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_angleAtFixedLength_" + fixedSS.str(), angleDeg);

            angleFixed = angleDeg;
            break;
        }
    }
    angleDiffFixedAndOne = angleOne - angleFixed;
    inputMeasurementData.addNameAndMeasurement("leaf_" + leafIDstr + "_angleDiff1-Fixed", angleDiffFixedAndOne);
    } //end scoping


    return 0;
}

int visualizeSpecificMeasurements(int argc, char** argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Visualizing measurements of the  plant.");
    LOG.DEBUG("Loading PLY to polygon mesh.");
    pcl::PolygonMesh inputMesh;
    pcl::io::loadPLYFile(argv[1], inputMesh);

    pcl::visualization::PCLVisualizer *visu;
    int originalViewport = 1;

    visu = new pcl::visualization::PCLVisualizer (argc, argv, "PlyViewer");

    visu->createViewPort (0.00, 0.0, 1.0, 1.0, originalViewport);
    visu->setBackgroundColor(0.5, 0.5, 0.5);
    visu->setSize(1700, 1000);
    //visu->addCoordinateSystem(50.0);
    visu->setCameraPosition(inputParams.cameraParameters.getxCoordLocation(),
                                inputParams.cameraParameters.getyCoordLocation(),
                                inputParams.cameraParameters.getzCoordLocation(),
                                inputParams.cameraParameters.getxViewComponent(),
                                inputParams.cameraParameters.getyViewComponent(),
                                inputParams.cameraParameters.getzViewComponent(),
                                inputParams.cameraParameters.getxUpComponent(),
                                inputParams.cameraParameters.getyUpComponent(),
                                inputParams.cameraParameters.getzUpComponent());

    visu->addPolygonMesh(inputMesh, "originalMesh", 1);

    LOG.DEBUG("Displaying the mesh to be measured. Press q to continue.");
    visu->spin();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsColored (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPointsSupervoxeled (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals (new pcl::PointCloud<pcl::Normal>);

    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudPoints);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudPointsColored);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudWithNormals);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *cloudNormals);

    /// Centroid
    { //scoping centroid calculation.
    Eigen::Vector4f centroid;
    compute3DCentroid(*cloudPoints, centroid);
    logStream << "Centroid coordinates: (" << centroid(0) << ", " << centroid(1) << ", " << centroid(2) << ")";
    LOG.DEBUG(logStream.str()); logStream.str("");
    float centroidX = centroid(0);
    float centroidY = centroid(1);
    float centroidZ = centroid(2);
    pcl::PointXYZRGBA centroidCoord;
    centroidCoord.x = centroidX;
    centroidCoord.y = centroidY;
    centroidCoord.z = centroidZ;
    centroidCoord.r = 0;
    centroidCoord.g = 0;
    centroidCoord.b = 255;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr centroidCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    centroidCloud->points.push_back(centroidCoord);
    visu->addPointCloud(centroidCloud, "centroid", 1);
    LOG.DEBUG("Displaying centroid. Press shift+ to enlarge the point. Press q to continue.");
    visu->spin();
    } //end scoping centroid calculation.

    // Diameter of mesh
    // To make sure we don't get any strange results with respect to unconnected plant pieces,
    // we enforce that we calculate diameter from a fully connected mesh.
    pcl::PolygonMesh connectedWholePlantMesh = returnLargestConnectedMesh(&inputMesh);
    Graph plantGraph(&connectedWholePlantMesh);
    DijkstraPathfinder plantPathfinder(plantGraph);

    PathDataContainer geodesicDiameter = plantPathfinder.findGeodesicDiameter();
    logStream << "Geodesic diameter of entire plant mesh: " << geodesicDiameter._distance;
    LOG.DEBUG(logStream.str()); logStream.str("");

    pcl::PointCloud<pcl::PointXYZ>::Ptr connectedMeshPoints (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(connectedWholePlantMesh.cloud, *connectedMeshPoints);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudPaths (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //int firstLabel = geodesicDiameter._sourceLabel;
    //int secondLabel = geodesicDiameter._targetLabel;
    //pcl::PointXYZ firstPoint = connectedMeshPoints->points[firstLabel];
    //pcl::PointXYZ lastPoint = connectedMeshPoints->points[secondLabel];
    //cloudPaths->points.push_back(firstPoint);
    //cloudPaths->points.push_back(lastPoint);
    //visu->addPointCloud(cloudPaths, "BeginAndEnd", 1);
    //LOG.DEBUG("Displaying the beginning and end of the maximum geodesic path. Press q to continue.");
    //visu->spin();

    for (uint32_t i = 0; i < geodesicDiameter._pathVectorOfLabels.size() - 1; i++) {
        std::stringstream ss;
        ss << i;
        //Using the basic addLine() function crashes with a confusing assertion.
        pcl::PointXYZ firstPathPoint = connectedMeshPoints->points[geodesicDiameter._pathVectorOfLabels[i]];
        pcl::PointXYZ secondPathPoint = connectedMeshPoints->points[geodesicDiameter._pathVectorOfLabels[i + 1]];
        addLineConnectionToViewer(firstPathPoint, secondPathPoint, ss.str(), visu, 1);
        pcl::PointXYZRGBA firstPathPointColored;
        pcl::PointXYZRGBA secondPathPointColored;
        firstPathPointColored.x = firstPathPoint.x; secondPathPointColored.x = secondPathPoint.x;
        firstPathPointColored.y = firstPathPoint.y; secondPathPointColored.y = secondPathPoint.y;
        firstPathPointColored.z = firstPathPoint.z; secondPathPointColored.z = secondPathPoint.z;
        firstPathPointColored.r = 0; firstPathPointColored.g = 0; firstPathPointColored.b = 255;
        secondPathPointColored.r = 0; secondPathPointColored.g = 0; secondPathPointColored.b = 255;
        cloudPaths->points.push_back(firstPathPointColored);
        cloudPaths->points.push_back(secondPathPointColored);
    }
    visu->updatePointCloud(cloudPaths, "centroid");
    LOG.DEBUG("Displaying the maximum geodesic length. Press q to continue.");
    visu->spin();

    /// Bounding box dimensions of plant
    BoundingBox boundingBox;
    BoundingBoxMaker<pcl::PointXYZRGBA> boundingBoxMaker;

    boundingBox = boundingBoxMaker.returnBoundingBoxOfCloud(cloudPointsColored);

    logStream << std::endl << "oriented bbox width: " << boundingBox.width << std::endl;
    logStream << "oriented bbox height: " << boundingBox.height << std::endl;
    logStream << "oriented bbox depth: " << boundingBox.depth;
    LOG.DEBUG(logStream.str()); logStream.str("");

    AxisAlignedBoundingBox axisAlignedBoundingBox;
    axisAlignedBoundingBox = boundingBoxMaker.returnAxisAlignedBoundingBoxOfCloud(cloudPointsColored);

    visu->addCube(axisAlignedBoundingBox.minX, axisAlignedBoundingBox.maxX,
                    axisAlignedBoundingBox.minY, axisAlignedBoundingBox.maxY,
                    axisAlignedBoundingBox.minZ, axisAlignedBoundingBox.maxZ, 1.0, 1.0, 1.0, "aligned_bbox", 1);

    LOG.DEBUG("Displaying the oriented bounding box and the axis aligned bounding box. Press q to continue.");
    visu->spin();




    return 0;
}
