
#ifndef MESHMEASUREMENTS_H
#define MESHMEASUREMENTS_H

#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "inputParams.h"
#include "measurementDataContainer.h"


/** \brief The primary function called by main() to make measurements of plant architecture.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file). The function only expects 1 .ply mesh.
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0], and the remaining index is the segmented .ply (i.e., colored vertices) that will be measured.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int makePlantMeasurements(int argc, char** argv, InputParameters inputParams);

/** \brief The secondary function called by makePlantMeasurements() to measure all of the segmented features in the mesh, including the stem and leaves.
  *
  * \param[in] inputMesh a pointer to a segmented (i.e., colored vertices) mesh for which individual portions of the mesh will be measured.
  * \param[in] inputMeasurementData the MeasurementDataContainer to which new measurements will be added.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \param[in] visualizer a pointer to a pcl::visualization::PCLVisualizer that will be used to display output if debugging level is sufficiently high.
  * \author Ryan McCormick
  */
int makeSegmentedFeatureMeasurements(pcl::PolygonMesh *inputMesh, MeasurementDataContainer &inputMeasurementData, InputParameters inputParams, pcl::visualization::PCLVisualizer *visualizer);

/** \brief A tertiary function called by makeSegmentedFeatureMeasurements() to measure the stem component of the mesh.
  *
  * \param[in] meshToMeasure a pointer to the stem subset of originalMesh for which measurements will be made.
  * \param[in] originalMesh a pointer to the original mesh from which mesh to Measure originally derived.
  * \param[in] inputMeasurementData the MeasurementDataContainer to which new measurements will be added.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \param[in] visualizer a pointer to a pcl::visualization::PCLVisualizer that will be used to display output if debugging level is sufficiently high.
  * \author Ryan McCormick
  */
int makeStemMeasurements(pcl::PolygonMesh *meshToMeasure, pcl::PolygonMesh *originalMesh, MeasurementDataContainer &inputMeasurementData, InputParameters inputParams, pcl::visualization::PCLVisualizer *visualizer);

/** \brief A tertiary function called by makeSegmentedFeatureMeasurements() to measure a leaf component of the mesh.
  *
  * \param[in] meshToMeasure a pointer to a leaf subset of originalMesh for which measurements will be made.
  * \param[in] originalMesh a pointer to the original mesh from which mesh to Measure originally derived.
  * \param[in] inputMeasurementData the MeasurementDataContainer to which new measurements will be added.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \param[in] visualizer a pointer to a pcl::visualization::PCLVisualizer that will be used to display output if debugging level is sufficiently high.
  * \author Ryan McCormick
  */
int makeIndividualLeafMeasurements(pcl::PolygonMesh *meshToMeasure, pcl::PolygonMesh *originalMesh, MeasurementDataContainer &inputMeasurementData, InputParameters inputParams, pcl::visualization::PCLVisualizer *visualizer);

/** \brief Calculates the area of the triangle formed by three points.
  *
  * \param[in] polygon a pcl::PointCloud of three pcl::PointXYZ points that comprise the triangle for which area will be calculated.
  * \return the area of the polygon.
  * \author Ryan McCormick, but the logic comes from the PCL forums.
  */
float calculateAreaPolygon(const pcl::PointCloud<pcl::PointXYZ> &polygon);

/** \brief Calculates the distance between two points. This should probably be removed, as I don't think it gets used.
  *
  * \param[in] edge a pcl::PointCloud of two pcl::PointXYZ points that represent the edge to be calculated.
  * \return the distance between the points.
  * \author Ryan McCormick, but the logic comes from the PCL forums.
  */
float calculateDistanceBetweenPoints(const pcl::PointCloud<pcl::PointXYZ> &edge);

/** \brief Calculates the length and width from area and perimeter for a rectangle. This should probably be removed, as I don't think it gets used.
  *
  * \param[in] area surface area of the 3D rectangle.
  * \param[in] perimeter perimeter of the 3D rectangle.
  * \return the length and width as a std::pair.
  * \author Ryan McCormick.
  */
std::pair<float,float> calculateLengthAndWidthFromAreaAndPerimeter(float area, float perimeter);

/** \brief Calculates the surface area of a mesh using calculateAreaPolygon();
  *
  * \param[in] inputMesh a pointer to a pcl::PolygonMesh for which the surface area will be calculated.
  * \return the surface area of the mesh.
  * \author Ryan McCormick.
  */
float returnSurfaceAreaOfMesh(pcl::PolygonMesh *inputMesh);

/** \brief Determines which of the input points has a greater z coordinate. This should probably be removed, as I don't think it gets used.
  *
  * \param[in] p1 a pcl::PointXYZ to compare with p2.
  * \param[in] p2 a pcl::PointXYZ to compare with p1.
  * \return bool for p1.z < p2.z
  * \author Ryan McCormick.
  */
bool zSortingFunction (pcl::PointXYZ p1, pcl::PointXYZ p2);

/** \brief Provides visual representation of some of the mesh measurements made in makePlantMeasurements();
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file). The function only expects 1 .ply mesh.
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0], and the remaining index is the segmented .ply (i.e., colored vertices) that will be measured.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int visualizeSpecificMeasurements(int argc, char** argv, InputParameters inputParams);

#endif
