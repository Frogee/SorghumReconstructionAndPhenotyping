
#ifndef POINTCLOUDFROMDEPTHIMAGE_H
#define POINTCLOUDFROMDEPTHIMAGE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "inputParams.h"

/** \brief Convience typedef to represent a std::vector of openCV matrices. Can be used to hold
  * a group of images that belong together.
  */
typedef std::vector<cv::Mat> VectorMats;

/** \brief Given a list of file names representing .png images, will read in the images to openCV matrices.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0]. All remaining indices should be the the files paths to .png images.
  * \param[in] v_mats the VectorMats container in which all of the loaded image data will be stored as openCV matrices.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
void loadDataFromImage(int argc, char **argv, VectorMats &v_mats, InputParameters inputParams);

/** \brief Performs post processing on depth images provided as openCV matrices, and writes them to file as point cloud data (.pcd).
  *
  * \param[in] v_mats the VectorMats container that contains depth image data to be processed and written to disk as point cloud data (.pcd).
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
void writePointClouds (VectorMats *v_mats, InputParameters inputParams);

/** \brief Function called by main() to convert depth images from input file names to point clouds and then write the point clouds to disk.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0]. All remaining indices should be the the files paths to .png images.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int convertDepthImagesToPointCloud(int argc, char** argv, InputParameters inputParams);

/** \brief Function to filter points that are statistical outliers from the input point cloud. This needs to be refactored to its own header file.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0]. argv[1] should contain the cloud to be filtered.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int removeStatisticalOutliers(int argc, char** argv, InputParameters inputParams);

#endif
