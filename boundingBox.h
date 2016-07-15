#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include "boundingBox.h"

/** \brief Struct to contain data corresponding to a bounding box.
  * http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
  * http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
  * \author Ryan McCormick
  */
struct BoundingBox {
    double width, height, depth;
    Eigen::Matrix3f eigenVectorsPCA;
    Eigen::Quaternionf bboxQuaternion;
    Eigen::Vector3f bboxTransform;
};

/** \brief Struct to contain data corresponding to an axis aligned bounding box.
  * http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
  * http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
  * \author Ryan McCormick
  */
struct AxisAlignedBoundingBox {
    double minX, maxX, minY, maxY, minZ, maxZ;
};

/** \brief Class that can find an oriented bounding box and an axis aligned bounding box given a cloud.
  * http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html
  * http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
  * \author Ryan McCormick
  */
/// I don't really understand templates. I wanted to do returnBoundingBoxOfCloud(pcl::PointCloud<PointT>::Ptr)...
///     but that doesn't compile. Putting it into a class like this works since we typedef pcl::PointCloud<PointT> as Base.
template<typename PointT>
class BoundingBoxMaker {
    public:
        typedef pcl::PointCloud<PointT> Base;
        typedef typename Base::Ptr Ptr;

        ///I had to wrap the function in this class since I couldn't figure out passing dependent names to a function
        BoundingBox returnBoundingBoxOfCloud(Ptr inputCloud) {
            std::cout << "Determining BoundingBox" << std::endl;
            /// From user Nicola Fioraio on the PCL forums. http://www.pcl-users.org/Finding-oriented-bounding-box-of-a-cloud-td4024616.html
            /*
            1) compute the centroid (c0, c1, c2) and the normalized covariance
            2) compute the eigenvectors e0, e1, e2. The reference system will be (e0, e1, e0 X e1) --- note: e0 X e1 = +/- e2
            3) move the points in that RF --- note: the transformation given by the rotation matrix (e0, e1, e0 X e1) & (c0, c1, c2) must be inverted
            4) compute the max, the min and the center of the diagonal
            5) given a box centered at the origin with size (max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z) the transformation you have to apply is Rotation = (e0, e1, e0 X e1) & Translation = Rotation * center_diag + (c0, c1, c2)

            cheers
            --N
            */
            // Compute principal directions
            Eigen::Vector4f pcaCentroid;
            pcl::compute3DCentroid(*inputCloud, pcaCentroid);
            Eigen::Matrix3f covariance;
            computeCovarianceMatrixNormalized(*inputCloud, pcaCentroid, covariance);
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
            Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
            eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                            ///    the signs are different and the box doesn't get correctly oriented in some cases.
            //std::cout << "eigenVectorsPCA:\n" << eigenVectorsPCA << std::endl;
            //std::cout << "eigenValuesPCA:\n" << eigen_solver.eigenvalues() << std::endl;

            /*
            /// Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PCA<pcl::PointXYZ> pca;
            pca.setInputCloud(inputCloud);
            pca.project(*inputCloud, *cloudPCAprojection);
            std::cerr << std::endl << "EigenVectors:\n" << pca.getEigenVectors() << std::endl;
            std::cerr << std::endl << "EigenValues\n" << pca.getEigenValues() << std::endl;
            //In this case, pca.getEigenVectors returns something similar to eigenVectorsPCA.
            */

            // Transform the original cloud to the origin where the principal components correspond to the axes.
            Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
            projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
            projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
            Ptr cloudPointsProjected (new Base);
            pcl::transformPointCloud(*inputCloud, *cloudPointsProjected, projectionTransform);
            //std::cout << "Projection transform:\n" << projectionTransform << std::endl;

            //Get the minimum and maximum points of the transformed cloud.
            PointT minPoint, maxPoint;
            pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
            const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

            //Final transform
            const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
            const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

            BoundingBox boundingBox;
            boundingBox.eigenVectorsPCA = eigenVectorsPCA;
            boundingBox.bboxQuaternion = bboxQuaternion;
            boundingBox.bboxTransform = bboxTransform;
            boundingBox.width = maxPoint.x - minPoint.x;
            boundingBox.height = maxPoint.y - minPoint.y;
            boundingBox.depth = maxPoint.z - minPoint.z;

            return boundingBox;
        }

        AxisAlignedBoundingBox returnAxisAlignedBoundingBoxOfCloud(Ptr inputCloud) {

            PointT minPoint, maxPoint;
            pcl::getMinMax3D(*inputCloud, minPoint, maxPoint);
            AxisAlignedBoundingBox boundingBox;

            boundingBox.minX = minPoint.x;
            boundingBox.maxX = maxPoint.x;
            boundingBox.minY = minPoint.y;
            boundingBox.maxY = maxPoint.y;
            boundingBox.minZ = minPoint.z;
            boundingBox.maxZ = maxPoint.z;


            return boundingBox;
        }

};

#endif
