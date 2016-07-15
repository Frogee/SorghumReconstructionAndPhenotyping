

#ifndef LSYSTEM_FITTING_H
#define LSYSTEM_FITTING_H

#include <python2.7/Python.h>
#include "lsystemParameters.h"

std::pair<float, float> returnTurnAndPitchAnglesToMoveZAxisToNormal(Eigen::Vector3f vInputNormal, InputParameters inputParams);

float returnLeafPhyllotaxyAngleToMoveXAxisToNormal(Eigen::Vector3f vInputCylinderAxisNormal,  Eigen::Vector3f planeNormal, InputParameters inputParams);

int fitLSystemToPointCloud(int argc, char** argv, InputParameters inputParams);

std::pair<float, float> returnSecondLeafCurvatureControlPointEstimate(pcl::PointXYZ inputLeafPointOrigin, pcl::PointXYZ inputLeafPointEnd, InputParameters inputParams);

pcl::PolygonMesh buildLSystem(PyObject *inputLpyFunction, LSystemParameters inputLSystemParams, pcl::visualization::PCLVisualizer *visu, InputParameters inputParams);

int identifyPointsOfEmergingLeaves(pcl::PointCloud<pcl::PointXYZ>::Ptr inputStemCloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr inputPutativeLeafPoints,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr outputLeafPoints,
                                    InputParameters inputParams);

Eigen::Vector4f findPlaneToBisectStem(Eigen::Matrix3f inputLeafPCAEigenVectors,
                                        Eigen::Vector3f inputLeafPCAEigenValues,
                                        pcl::PointXYZ inputOriginPoint,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr nonCylinderPoints,
                                        Eigen::VectorXf inputCylinderCoefficients,
                                        InputParameters inputParams);
#endif
