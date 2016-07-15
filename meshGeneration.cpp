#include <stdlib.h>
#include <cmath>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/marching_cubes_rbf.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/poisson.h>

#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

#include <boost/thread/thread.hpp>

#include "meshGeneration.h"
#include "inputParams.h"
#include "loggingHelper.h"

typedef pcl::PointNormal PointNT;

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> ColorHandlerXYZ;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerNT;
int meshGeneration(int argc, char** argv, InputParameters inputParams)
{
    PCL_INFO ("Entering mesh generation.\n");
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile (argv[1], cloud_blob);
  pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  //* the data should be available in cloud
  PCL_INFO ("Loaded %d points.\n", cloud->size());

        PCL_INFO ("Removing statistical outliers.\n");
        //Statistical removal of outliers
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (inputParams.statisticalOutlierRemovalParameters.getMeanK());
        sor.setStddevMulThresh (inputParams.statisticalOutlierRemovalParameters.getStdDevMulThresh());
        sor.filter (*cloud);

                PCL_INFO ("Filtering using pass through.\n");
        //PassThrough filter application
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThrough_Intermediate (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThrough (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cloud_filteredPassThrough_nonNormalized (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
        passThroughFilter.setInputCloud(cloud);
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
        passThroughFilter.filter(*cloud);

  pcl::visualization::PCLVisualizer *visu;
  visu = new pcl::visualization::PCLVisualizer (argc, argv, "MeshGeneration");

  int mesh_vp_1, mesh_vp_2;

  visu->createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
  visu->createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);
    visu->addPointCloud (cloud, ColorHandlerXYZ(cloud, 0.0, 255.0, 0.0), "modified", mesh_vp_1);
    visu->spinOnce();

            // Downsample
          pcl::console::print_highlight ("Downsampling...\n");
          pcl::VoxelGrid<pcl::PointXYZ> grid;
          const float voxel_grid_size = inputParams.voxelGridFilterParameters.getLeafSize();
          grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
          grid.setInputCloud (cloud);
          grid.filter (*cloud);


          visu->updatePointCloud(cloud, ColorHandlerXYZ(cloud, 0.0, 0.0, 255.0), "modified");
            visu->spinOnce();

        PCL_INFO ("Writing %d points to file.\n", cloud->size());
        std::stringstream ss;
        ss << "CloudToBeMeshed" << ".pcd";
        std::string filePath = "./" + ss.str();
        pcl::io::savePCDFileASCII(filePath, *cloud);


bool greedyProjectionTriangulation = true;
if (greedyProjectionTriangulation == true) {

/*
PCL_INFO ("Starting moving least squares\n");
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud (cloud);
    mls.setSearchRadius (inputParams.NormalEstimation_RadiusSearch);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    //mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    //mls.setUpsamplingRadius (0.5);
    //mls.setUpsamplingStepSize (0.3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
    mls.process (*cloud_smoothed);
*/
    PCL_INFO ("Estimating Normals\n");
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads (1);
    //ne.setInputCloud (cloud_smoothed);
    ne.setInputCloud (cloud);
    ne.setRadiusSearch (inputParams.normalEstimationParameters.getRadiusSearch());
    Eigen::Vector4f centroid;
    //compute3DCentroid (*cloud_smoothed, centroid);
    compute3DCentroid (*cloud, centroid);
    //ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
    std::cout << "Centroids: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
    ne.setViewPoint (centroid[0], 10000000, centroid[2]);
    float curViewX, curViewY, curViewZ;
    ne.getViewPoint(curViewX, curViewY, curViewZ);
    std::cout << "View Points Set: " << curViewX << " " << curViewY << " " << curViewZ << std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    ne.compute (*cloud_normals);

    for (size_t i = 0; i < cloud_normals->size (); ++i)
    {
    /*
        float sumOfPointAndNormal, differenceOfPointAndCentroid, differenceOfPointNormAndCentroid;
        sumOfPointAndNormal = 0.0 ; differenceOfPointAndCentroid = 0.0; differenceOfPointNormAndCentroid;
        float pointX, normalX;
        pointX = 0.0; normalX = 0.0;
        pointX = cloud->points[i].x;
        normalX = cloud_normals->points[i].normal_x;
        sumOfPointAndNormal = pointX + normalX;
        differenceOfPointAndCentroid = pointX - centroid[0];
        differenceOfPointNormAndCentroid = sumOfPointAndNormal - centroid[0];
        if (differenceOfPointNormAndCentroid < differenceOfPointAndCentroid) {   //If the normal is takes it closer to the centroid, need to flip.
            cloud_normals->points[i].normal_x *= -1;
            cloud_normals->points[i].normal_y *= -1;
            cloud_normals->points[i].normal_z *= -1;
        }
        else {
            //empty else statement;
        }
        */
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;

    }
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
    //concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
    concatenateFields (*cloud, *cloud_normals, *cloud_smoothed_normals);

        visu->updatePointCloud(cloud_smoothed_normals, ColorHandlerNT (cloud_smoothed_normals, 255.0, 0.0, 0.0), "modified");
        //visu->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_smoothed, cloud_normals, 10, 10, "normals", mesh_vp_1);
        visu->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals, 10, 20, "normals", mesh_vp_1);
        visu->spin();

    PCL_INFO ("Smoothing finished. Proceeding with triangulation\n");

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_smoothed_normals);
  //PCL_INFO ("KdTree MinPts: %d, Epsilon %f\n", tree2->getMinPts(), tree2->getEpsilon());


/*
http://pointclouds.org/documentation/tutorials/greedy_projection.php#greedy-triangulation
The method works by maintaining a list of points from which the mesh can be grown (“fringe” points) and extending it until all possible points are connected.
It can deal with unorganized points, coming from one or multiple scans, and having multiple connected parts. It works best if the surface is locally smooth
and there are smooth transitions between areas with different point densities.

Triangulation is performed locally, by projecting the local neighborhood of a point along the point’s normal, and connecting unconnected points.
Thus, the following parameters can be set:
    setMaximumNearestNeighbors(unsigned) and setMu(double) control the size of the neighborhood. The former defines how many neighbors are
        searched for, while the latter specifies the maximum acceptable distance for a point to be considered, relative to the distance of the nearest point
        (in order to adjust to changing densities). Typical values are 50-100 and 2.5-3 (or 1.5 for grids).
    setSearchRadius(double) is practically the maximum edge length for every triangle. This has to be set by the user such that to allow for the biggest
        triangles that should be possible.
    setMinimumAngle(double) and setMaximumAngle(double) are the minimum and maximum angles in each triangle. While the first is not guaranteed,
        the second is. Typical values are 10 and 120 degrees (in radians).
    setMaximumSurfaceAgle(double) and setNormalConsistency(bool) are meant to deal with the cases where there are sharp edges or corners and where two
        sides of a surface run very close to each other. To achieve this, points are not connected to the current point if their normals deviate more than the
        specified angle (note that most surface normal estimation methods produce smooth transitions between normal angles even at sharp edges). This angle is computed as the angle between the lines defined by the normals (disregarding the normal’s direction) if the normal-consistency-flag is not set, as not all normal estimation methods can guarantee consistently oriented normals. Typically, 45 degrees (in radians) and false works on most datasets.
*/

PCL_INFO ("Starting greedy projection triangulation\n");
  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());   //Hiding Normal radius search here for now.

  // Set typical values for the parameters
  gp3.setMu (inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());   //Temp
  gp3.setMaximumNearestNeighbors (inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());  //Temp
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  //  gp3.setMaximumSurfaceAngle(M_PI);
  //  gp3.setMinimumAngle(M_PI/4);
  //  gp3.setMaximumAngle(M_PI/2.0);
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_smoothed_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();
  pcl::io::saveVTKFile("mesh.vtk", triangles);
  pcl::io::savePLYFile("meshTri.ply", triangles);
  pcl::io::savePLYFile("mesh.ply", *cloud_smoothed_normals, false);
  visu->addPolygonMesh(triangles, "polygon", mesh_vp_2);
    visu->spin();
}
else {

PCL_INFO ("Starting marching cubes\n");
/*
PCL_INFO ("Starting moving least squares\n");
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    mls.setInputCloud (cloud);
    mls.setSearchRadius (inputParams.NormalEstimation_RadiusSearch);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.5);
    mls.setUpsamplingStepSize (0.3);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
    mls.process (*cloud_smoothed);
*/
    PCL_INFO ("Estimating Normals\n");
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNumberOfThreads (1);
    //ne.setInputCloud (cloud_smoothed);
    ne.setInputCloud (cloud);
    ne.setRadiusSearch (inputParams.normalEstimationParameters.getRadiusSearch());
    Eigen::Vector4f centroid;
    //compute3DCentroid (*cloud_smoothed, centroid);
    compute3DCentroid (*cloud, centroid);
    //ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
    std::cout << "Centroids: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
    ne.setViewPoint (centroid[0], 10000000, centroid[2]);
    float curViewX, curViewY, curViewZ;
    ne.getViewPoint(curViewX, curViewY, curViewZ);
    std::cout << "View Points Set: " << curViewX << " " << curViewY << " " << curViewZ << std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    ne.compute (*cloud_normals);

    for (size_t i = 0; i < cloud_normals->size (); ++i)
    {
    /*
        if (cloud_normals->points[i].normal_x > 0) {
            cloud_normals->points[i].normal_x *= -1;
        }
        if (cloud_normals->points[i].normal_y > 0) {
            cloud_normals->points[i].normal_y *= -1;
        }
        if (cloud_normals->points[i].normal_z > 0) {
            cloud_normals->points[i].normal_z *= -1;
        }
        */

        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
    //concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
    concatenateFields (*cloud, *cloud_normals, *cloud_smoothed_normals);

        visu->updatePointCloud(cloud_smoothed_normals, ColorHandlerNT (cloud_smoothed_normals, 255.0, 0.0, 0.0), "modified");
        //visu->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_smoothed, cloud_normals, 10, 10, "normals", mesh_vp_1);
        visu->addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals, 10, 20, "normals", mesh_vp_1);
        visu->spin();

    PCL_INFO ("Smoothing finished. Proceeding with triangulation\n");
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_smoothed_normals);

  // Initialize objects
  pcl::MarchingCubesHoppe<pcl::PointNormal> march;
  pcl::PolygonMesh triangles;

  // Get result
  /*
  PCL_INFO ("Setting input \n");
  march.setInputCloud (cloud_smoothed_normals);
  PCL_INFO ("Setting search method \n");
  march.setSearchMethod (tree2);
  march.setIsoLevel(inputParams.SACPrerejective_SimilarityThreshold);  //Temporarily hiding iso level in SAC_Similarity Threshold for testing.
  march.setGridResolution(inputParams.SACPrerejective_MaxCorrespondenceDistance,
                        inputParams.SACPrerejective_MaxCorrespondenceDistance, inputParams.SACPrerejective_MaxCorrespondenceDistance);   //Temp
  march.setPercentageExtendGrid(inputParams.SACPrerejective_InlierFraction);  //temp

  PCL_INFO ("reconstructing \n");
  march.reconstruct(triangles);
*/
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());
    poisson.setInputCloud(cloud_smoothed_normals);
    poisson.setDegree(2);
    poisson.setSamplesPerNode(inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());
    poisson.setScale(inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());
    poisson.setIsoDivide(inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());
    poisson.setConfidence(1);
    poisson.setManifold(0);
    poisson.setOutputPolygons(0);
    poisson.setSolverDivide(8);
    poisson.setPointWeight(0);

    poisson.reconstruct (triangles);


PCL_INFO ("Adding to visualizer\n");
  pcl::io::saveVTKFile("mesh.vtk", triangles);
  pcl::io::savePLYFile("meshTri.ply", triangles);
  pcl::io::savePLYFile("mesh.ply", *cloud_smoothed_normals, false);
  system("pwd");
  visu->addPolygonMesh(triangles, "polygon", mesh_vp_2);
    visu->spin();
}


  return (0);
}


int convertPCDToPLY(int argc, char** argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Not yet implemented");
    return(0);
}

int convertPLYToPCD(int argc, char** argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Converting PLY to PCD");

    logStream << "Loading PLY file " << argv[1] << ".";
    LOG.DEBUG(logStream.str()); logStream.str("");

    pcl::PolygonMesh inputMesh;
    pcl::io::loadPLYFile(argv[1], inputMesh);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputPointsColored (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointNormal>::Ptr inputWithNormals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr inputNormals (new pcl::PointCloud<pcl::Normal>);

    pcl::fromPCLPointCloud2(inputMesh.cloud, *inputPoints);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *inputPointsColored);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *inputWithNormals);
    pcl::fromPCLPointCloud2(inputMesh.cloud, *inputNormals);

    pcl::io::savePCDFile("convertedPLY.pcd", *inputPoints, true);  //To save as binary

    return(0);
}

int multiCloudMeshGeneration(int argc, char** argv, InputParameters inputParams) {
    PCL_INFO ("Entering mesh generation from multiple clouds.\n");




    pcl::visualization::PCLVisualizer visu(argc, argv, "MeshGeneration");

    int mesh_vp_1, mesh_vp_2;

    visu.createViewPort (0.0, 0, 0.5, 1.0, mesh_vp_1);
    visu.createViewPort (0.5, 0, 1.0, 1.0, mesh_vp_2);


    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_combined (new pcl::PointCloud<pcl::PointNormal> ());

    for (int i = 1; i < argc; i++) {
            std::cout << argv[i] << std::endl;

            // Load input file into a PointCloud<T> with an appropriate type
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PCLPointCloud2 cloud_blob;
            pcl::io::loadPCDFile (argv[i], cloud_blob);
            pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
            //the data should be available in cloud
            PCL_INFO ("Loaded %d points.\n", cloud->size());

/*
            PCL_INFO ("Starting moving least squares\n");
            pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
            mls.setInputCloud (cloud);
            mls.setSearchRadius (inputParams.movingLeastSquaresParameters.getSearchRadius());
            mls.setPolynomialFit (true);
            mls.setPolynomialOrder (2);
            //mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
            //mls.setUpsamplingRadius (1.5);
            //mls.setUpsamplingStepSize (1.0);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
            mls.process (*cloud_smoothed);
*/

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed (new pcl::PointCloud<pcl::PointXYZ> ());
            cloud_smoothed = cloud;

            PCL_INFO ("Estimating Normals\n");
            pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
            ne.setNumberOfThreads (1);
            ne.setInputCloud (cloud_smoothed);
            //ne.setInputCloud (cloud);
            ne.setRadiusSearch (inputParams.normalEstimationParameters.getRadiusSearch());
            Eigen::Vector4f centroid;
            compute3DCentroid (*cloud_smoothed, centroid);
            //compute3DCentroid (*cloud, centroid);
            ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
            std::cout << "Centroids: " << centroid[0] << " " << centroid[1] << " " << centroid[2] << std::endl;
            //ne.setViewPoint (centroid[0], 10000000, centroid[2]);
            float curViewX, curViewY, curViewZ;
            ne.getViewPoint(curViewX, curViewY, curViewZ);
            std::cout << "View Points Set: " << curViewX << " " << curViewY << " " << curViewZ << std::endl;
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
            ne.compute (*cloud_normals);

            for (size_t i = 0; i < cloud_normals->size (); ++i)
            {
                cloud_normals->points[i].normal_x *= -1;
                cloud_normals->points[i].normal_y *= -1;
                cloud_normals->points[i].normal_z *= -1;
            }

            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_smoothed_normals (new pcl::PointCloud<pcl::PointNormal> ());
            concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);
            *cloud_combined = *cloud_combined + *cloud_smoothed_normals;


            if (i == 1) {
                visu.addPointCloud (cloud_smoothed, ColorHandlerXYZ(cloud_smoothed, 0.0, 255.0, 0.0), "modified", mesh_vp_1);
                visu.addPointCloud (cloud_combined, ColorHandlerNT(cloud_combined, 0.0, 255.0, 0.0), "combined", mesh_vp_2);
                PCL_INFO ("Spinning. Press q to continue\n");
                visu.spin();
            }
            if (i == (argc - 1)) {
                visu.updatePointCloud (cloud_smoothed, ColorHandlerXYZ(cloud_smoothed, 0.0, 255.0, 0.0), "modified");
                visu.updatePointCloud (cloud_combined, ColorHandlerNT(cloud_combined, 0.0, 255.0, 0.0), "combined");
                visu.addPointCloudNormals<pcl::PointNormal>(cloud_combined, 10, 20, "normals", mesh_vp_2);
                PCL_INFO ("Spinning. Press q to continue\n");
                visu.spin();
            }
            else {
                visu.updatePointCloud (cloud_smoothed, ColorHandlerXYZ(cloud_smoothed, 0.0, 255.0, 0.0), "modified");
                visu.updatePointCloud (cloud_combined, ColorHandlerNT(cloud_combined, 0.0, 255.0, 0.0), "combined");
                visu.spinOnce();
            }
        }

        /*
http://pointclouds.org/documentation/tutorials/greedy_projection.php#greedy-triangulation
The method works by maintaining a list of points from which the mesh can be grown (“fringe” points) and extending it until all possible points are connected.
It can deal with unorganized points, coming from one or multiple scans, and having multiple connected parts. It works best if the surface is locally smooth
and there are smooth transitions between areas with different point densities.

Triangulation is performed locally, by projecting the local neighborhood of a point along the point’s normal, and connecting unconnected points.
Thus, the following parameters can be set:
    setMaximumNearestNeighbors(unsigned) and setMu(double) control the size of the neighborhood. The former defines how many neighbors are
        searched for, while the latter specifies the maximum acceptable distance for a point to be considered, relative to the distance of the nearest point
        (in order to adjust to changing densities). Typical values are 50-100 and 2.5-3 (or 1.5 for grids).
    setSearchRadius(double) is practically the maximum edge length for every triangle. This has to be set by the user such that to allow for the biggest
        triangles that should be possible.
    setMinimumAngle(double) and setMaximumAngle(double) are the minimum and maximum angles in each triangle. While the first is not guaranteed,
        the second is. Typical values are 10 and 120 degrees (in radians).
    setMaximumSurfaceAgle(double) and setNormalConsistency(bool) are meant to deal with the cases where there are sharp edges or corners and where two
        sides of a surface run very close to each other. To achieve this, points are not connected to the current point if their normals deviate more than the
        specified angle (note that most surface normal estimation methods produce smooth transitions between normal angles even at sharp edges). This angle is computed as the angle between the lines defined by the normals (disregarding the normal’s direction) if the normal-consistency-flag is not set, as not all normal estimation methods can guarantee consistently oriented normals. Typically, 45 degrees (in radians) and false works on most datasets.
*/

        PCL_INFO ("Smoothing finished. Proceeding with triangulation\n");
        // Create search tree*
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
        tree2->setInputCloud(cloud_combined);

        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
        pcl::PolygonMesh triangles;

        // Set the maximum distance between connected points (maximum edge length)
        gp3.setSearchRadius(inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());   //Hiding Normal radius search here for now.

        // Set typical values for the parameters
        gp3.setMu (inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());   //Temp
        gp3.setMaximumNearestNeighbors (inputParams.sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness());  //Temp
        gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
        gp3.setMinimumAngle(M_PI/18); // 10 degrees
        gp3.setMaximumAngle((90 * M_PI) / 180.0); // 90 degrees
        gp3.setNormalConsistency(true);
        //  gp3.setMaximumSurfaceAngle(M_PI);
        //  gp3.setMinimumAngle(M_PI/4);
        //  gp3.setMaximumAngle(M_PI/2.0);
        //gp3.setNormalConsistency(false);

        // Get result
        gp3.setInputCloud (cloud_combined);
        gp3.setSearchMethod (tree2);
        gp3.reconstruct (triangles);

        pcl::visualization::PCLVisualizer visuTri(argc, argv, "MeshGeneration");
        int tri_vp_1, tri_vp_2;
        visuTri.createViewPort (0.0, 0, 0.5, 1.0, tri_vp_1);
        visuTri.createViewPort (0.5, 0, 1.0, 1.0, tri_vp_2);
        visuTri.addPointCloud(cloud_combined, ColorHandlerNT(cloud_combined, 0.0, 255.0, 0.0), "combined", tri_vp_1);
        visuTri.addPolygonMesh(triangles, "polygon", tri_vp_2);
        PCL_INFO ("Spinning. Press q to continue\n");
        visuTri.spin();

        pcl::io::saveVTKFile("mesh.vtk", triangles);
        pcl::io::savePLYFile("meshTri.ply", triangles);
        pcl::io::savePLYFile("mesh.ply", *cloud_combined, false);
        return(0);
}
