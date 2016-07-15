
#ifndef ITERATIVECLOSESTPOINT_H
#define ITERATIVECLOSESTPOINT_H

#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include "inputParams.h"

/** \brief Performs pairwise iterative closest point for all input clouds in the order they are provided.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0], and the remaining indices are the point clouds in the order to be processed.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int registerPointCloudsICP_pairwise(int argc, char **argv, InputParameters inputParams);


/** \brief Performs pairwise iterative closest point for all input clouds in the order they are provided. After finishing a pair,
  * the pair is added to the global cloud. The global cloud is used for the next pairwise iteration.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0], and the remaining indices are the point clouds in the order to be processed.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int registerPointCloudsICP_oneAgainstGlobal(int argc, char **argv, InputParameters inputParams);

/** \brief Uses statistical outlier removal to filter the cloud, then performs pairwise iterative closest point for all
  * input clouds in the order they are provided. After finishing a pair, the pair is added to the global cloud. The
  * global cloud is used for the next pairwise iteration. This is nearly identical to the oneAgainstGlobal ICP.
  *
  * \param[in] argc the value of argc at runtime decremented by 2 (to account for removing the program name and the input .xml file)
  * \param[in] argv the argv given at runtime shifted by two indices (to account for removing the program name and the input .xml file).
  * argv will have the called processing option in argv[0], and the remaining indices are the point clouds in the order to be processed.
  * \param[in] inputParams the InputParameters object that contains values to be used, read in from the input .xml file.
  * \author Ryan McCormick
  */
int registerPointCloudsICP_refinement(int argc, char **argv, InputParameters inputParams);


int registerKinectFusionPLYs(int argc, char **argv, InputParameters inputParams);

pcl::CorrespondencesPtr registerPointCloudsICPAndReturnCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams);

int registerLSystemICP(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputSourceTransformedToTarget,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams);


Eigen::Matrix4f registerLSystemICPAndReturnTranslationMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputSourceTransformedToTarget,
                                                            pcl::visualization::PCLVisualizer* visu,
                                                            InputParameters inputParams);

float registerLSystemICPAndReturnFitness(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputSourceTransformedToTarget,
                                                            pcl::visualization::PCLVisualizer* visu,
                                                            InputParameters inputParams);

float registerLSystemICPAndReturnFitness_ForParallelCalling(pcl::PointCloud<pcl::PointXYZ>::Ptr inputSourceCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputTargetCloud,
                                                            pcl::PointCloud<pcl::PointXYZ>::Ptr outputSourceTransformedToTarget,
                                                            InputParameters inputParams);


#include "loggingHelper.h"

/** \brief This is a mock class with the sole purpose of accessing a protected member of a class it inherits from.
  *
  * Some of the relevant documentation: http://docs.pointclouds.org/trunk/correspondence_8h_source.html#l00092
  */
template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPointNonLinear_Exposed : public pcl::IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> {
    public:
        pcl::CorrespondencesPtr getCorrespondencesPtr() {
            for (uint32_t i = 0; i < this->correspondences_->size(); i++) {
                pcl::Correspondence currentCorrespondence = (*this->correspondences_)[i];
            }
            return this->correspondences_;
        }
};


/*
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPointNonLinearRedefinedForAccess : public pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar>
{
using IterativeClosestPoint<PointSource, PointTarget, Scalar>::min_number_correspondences_;
using IterativeClosestPoint<PointSource, PointTarget, Scalar>::reg_name_;
using IterativeClosestPoint<PointSource, PointTarget, Scalar>::transformation_estimation_;
using IterativeClosestPoint<PointSource, PointTarget, Scalar>::computeTransformation;

public:

typedef boost::shared_ptr< IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> > Ptr;
typedef boost::shared_ptr< const IterativeClosestPointNonLinear<PointSource, PointTarget, Scalar> > ConstPtr;

typedef typename Registration<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;


IterativeClosestPointNonLinear ()
{
min_number_correspondences_ = 4;
reg_name_ = "IterativeClosestPointNonLinear";

transformation_estimation_.reset (new pcl::registration::TransformationEstimationLM<PointSource, PointTarget, Scalar>);
}
};

*/

#endif
