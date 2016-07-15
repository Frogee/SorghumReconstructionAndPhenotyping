#include <stdlib.h>
#include <time.h>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>
#include "vtkSmartPointer.h"

#include "segmentation.h"
#include "supervoxel_construction.h"
#include "loggingHelper.h"

// Adds a line to the viewer. Inspired by http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php
void
addLineConnectionToViewer (pcl::PointXYZ pointBegin,
                                  pcl::PointXYZ pointEnd,
                                  std::string lineName,
                                  pcl::visualization::PCLVisualizer* viewer,
                                  int viewport)
{
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
      vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
      vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

      ////Iterate through all adjacent points, and add a center point to adjacent point pair
      //PointCloudT::iterator adjacent_itr = adjacent_supervoxel_centers.begin ();
      //for ( ; adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
      //{
        points->InsertNextPoint (pointBegin.x, pointBegin.y, pointBegin.z);
        points->InsertNextPoint (pointEnd.x, pointEnd.y, pointEnd.z);
      //}
      // Create a polydata to store everything in
      vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
      // Add the points to the dataset
      polyData->SetPoints (points);
      polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
      for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
        polyLine->GetPointIds ()->SetId (i,i);
      cells->InsertNextCell (polyLine);
      // Add the lines to the dataset
      polyData->SetLines (cells);
      viewer->addModelFromPolyData(polyData, lineName, viewport);
}


void
addSupervoxelAdjacencyToViewer (SupervoxelDataContainer inputSupervoxelData,
                                  pcl::visualization::PCLVisualizer* viewer,
                                  int viewport)
{
    std::multimap<uint32_t,uint32_t> supervoxel_adjacency = inputSupervoxelData._supervoxelAdjacency;
    std::map <uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters = inputSupervoxelData._supervoxelClusters;

    // To graph the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    std::multimap<uint32_t,uint32_t>::iterator label_itr;
    int lineCounter = 0;
    for (label_itr = supervoxel_adjacency.begin(); label_itr != supervoxel_adjacency.end(); ) { //We increment this iterator at the end of the loop

        //First get the label
        uint32_t supervoxel_label = label_itr->first;

        //Now get the supervoxel corresponding to the label
        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at(supervoxel_label);

        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        pcl::PointCloud<pcl::PointXYZRGBA> adjacent_supervoxel_centers;
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr;
        for ( adjacent_itr = supervoxel_adjacency.equal_range(supervoxel_label).first;
              adjacent_itr != supervoxel_adjacency.equal_range (supervoxel_label).second; adjacent_itr++) {
            pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr neighbor_supervoxel = supervoxel_clusters.at(adjacent_itr->second);
            adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
        }


        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
        vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
        vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

        //Iterate through all adjacent points, and add a center point to adjacent point pair
        pcl::PointCloud<pcl::PointXYZRGBA>::iterator cloudAdjacent_itr = adjacent_supervoxel_centers.begin ();
        for ( ; cloudAdjacent_itr != adjacent_supervoxel_centers.end (); ++cloudAdjacent_itr)
        {
            points->InsertNextPoint (supervoxel->centroid_.data);
            points->InsertNextPoint (cloudAdjacent_itr->data);
        }
        // Create a polydata to store everything in
        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
        // Add the points to the dataset
        polyData->SetPoints (points);
        polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
        for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
            polyLine->GetPointIds ()->SetId (i,i);
        cells->InsertNextCell (polyLine);
        // Add the lines to the dataset
        polyData->SetLines (cells);

        std::stringstream ss;
        ss << "supervoxel_" << supervoxel_label;
        viewer->addModelFromPolyData (polyData, ss.str(), viewport);

        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }
}

int updateSegmentedMeshInViewer(pcl::PolygonMesh *currentMesh,
                                PlantSegmentationDataContainer inputSegmentationData,
                                pcl::visualization::PCLVisualizer *visualizer,
                                std::string meshID,
                                int viewport) {

    // We can reconstitute the original polygon mesh from either the geodesic data structure and/or supervoxel data.
    int cloudIndex = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointsInOriginalMesh (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointsInMeshToDisplay (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PolygonMesh meshToDisplay = *currentMesh;



    pcl::fromPCLPointCloud2(meshToDisplay.cloud, *pointsInOriginalMesh);

    for (uint32_t i = 0; i < pointsInOriginalMesh->points.size(); i++) {
        pcl::PointXYZ currentPoint = pointsInOriginalMesh->points[i];

        TupleTriplet tupleXYZ = convertPclPointXYZtoTupleTriplet(currentPoint);
        TupleTriplet tupleRGB = inputSegmentationData._map_segmentedPoints[tupleXYZ];

        pcl::PointXYZRGBA newPoint;
        newPoint.x = currentPoint.x;
        newPoint.y = currentPoint.y;
        newPoint.z = currentPoint.z;
        newPoint.r = std::get<0>(tupleRGB);
        newPoint.g = std::get<1>(tupleRGB);
        newPoint.b = std::get<2>(tupleRGB);
        newPoint.a = 255.0;
        pointsInMeshToDisplay->points.push_back(newPoint);
    }

    pcl::PCLPointCloud2 segmentedMeshCloudPCL2;
    pcl::toPCLPointCloud2(*pointsInMeshToDisplay, segmentedMeshCloudPCL2);
    meshToDisplay.cloud = segmentedMeshCloudPCL2;

    visualizer->removePolygonMesh(meshID); //Although removing a point cloud doesn't seem to work, removing a PolygonMesh does.
	visualizer->addPolygonMesh(meshToDisplay, meshID, viewport);
    visualizer->spinOnce();
    return(0);
}

int screenshotPLY(int argc, char** argv, InputParameters inputParams) {
    std::ostringstream logStream;
    LOG.DEBUG("Using the following camera parameters: ");
    inputParams.cameraParameters.printParameters();

    logStream << "Loading PLY from file " << argv[1] << ".\n";
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");

    // This is a weak attempt at giving the visualizer window name a "unique" name so that the screen capture doesn't conflict
    // with other screencapture processes running at the same time.
    srand(time(NULL));
    std::stringstream windowNameStream;
    windowNameStream << "PlyViewer_" << rand()%1000;

    pcl::visualization::PCLVisualizer *visu;
    visu = new pcl::visualization::PCLVisualizer (argc, argv, windowNameStream.str());
    //visu->setFullScreen(true); This really crashes things.
    visu->setSize(1500, 1000);
    visu->addCoordinateSystem(50.0);


    visu->addModelFromPLYFile(argv[1], "PLY_model");
    visu->setBackgroundColor(0.5, 0.5, 0.5);
    visu->setCameraPosition(inputParams.cameraParameters.getxCoordLocation(),
                            inputParams.cameraParameters.getyCoordLocation(),
                            inputParams.cameraParameters.getzCoordLocation(),
                            inputParams.cameraParameters.getxViewComponent(),
                            inputParams.cameraParameters.getyViewComponent(),
                            inputParams.cameraParameters.getzViewComponent(),
                            inputParams.cameraParameters.getxUpComponent(),
                            inputParams.cameraParameters.getyUpComponent(),
                            inputParams.cameraParameters.getzUpComponent());

    //Save screenshot doesn't seem to work as intended, so we'll do it from outside the program using imageMagick
    //visu->saveScreenshot(argv[2]);

    std::string imageMagickString = "import -window " + windowNameStream.str() + " " + std::string(argv[2]);
    visu->spinOnce(2000);
    logStream << "Taking screenshot of window using the following command:\n" << imageMagickString;
    LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 0); logStream.str("");
    int systemReturn = system(imageMagickString.c_str());
    visu->spinOnce(2000);

    return(0);
}

