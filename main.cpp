#include <string>
#include <iostream>
#include <sstream>
#include <chrono>
#include <ctime>
#include "pointCloudFromDepthImage.h"
#include "SampleConsensusPrerejective.h"
#include "meshGeneration.h"
#include "segmentation.h"
#include "IterativeClosestPoint.h"
#include "inputParams.h"
#include "features.h"
#include "visualizer_helper.h"
#include "meshMeasurements.h"
#include "loggingHelper.h"
#include "lsystemFitting.h"

#define PROGRAMNAME "PlantMeshPhenotyper"

void printUsageMessage() {
    std::cout << "Usage:\n" <<
        "\t" << PROGRAMNAME << " inputParameters.xml --processingSelection file1 <file2> <file3> ..." << std::endl <<
        "Explanation:\n" <<
        "\tinputParameters.xml contains filter and algorithm parameters in xml format. This file necessary even if you want to use all default parameters.\n" <<
        "Options for \"--processingSelection\":\n" <<
        "\t\t--convertImageToCloud image1.png <image2.png> <image3.png> ...\n" <<
        "\t\t--registerPointCloudsICP_pairwise cloud1.pcd cloud2.pcd <cloud3.pcd> ... \n" <<
        "\t\t--registerPointCloudsICP_oneAgainstGlobal cloud1.pcd cloud2.pcd <cloud3.pcd> ... \n" <<
        "\t\t--registerPointCloudsICP_refinement cloud1.pcd cloud2.pcd <cloud3.pcd> ... \n" <<
        "\t\t--screenshotPLY mesh.ply \n" <<
        "\t\t--makePlantMeasurements segmentedMesh.ply \n" <<
        "\t\t--segmentationFromPLY mesh.ply \n" <<
        "\t\t--removeStatisticalOutliers cloud.pcd \n" <<
        "\t\t--computePointFeatures mesh.ply \n" <<
        "\t\t--segmentOutPot cloud.pcd \n" << std::endl;
}

/** \brief main
  *
  * \param[in] argc Number of command line arguments received. Prior to input to processing functions, this gets
  * decremented by 2 (to account for removing the program name and the input .xml file).
  * \param[in] argv Character array of arguments. Prior to input to processing functions, this gets shifted by
  * two indices (to account for removing the program name and the input .xml file). argv should contain the processing selection
  * and all of the input files to be used for that processing selection.
  * \author Ryan McCormick
  */
int main(int argc, char** argv) {
    std::cout << std::endl << "Beginning execution of " << PROGRAMNAME << std::endl;

    if (argc < 3) {
        std::cout << "Insufficient arguments." << std::endl;
        printUsageMessage();
        return(0);
    }

    /// Each of the specific options expects a variable number of input arguments. We trim the original input, which has the input parameters,
    /// so that each of the options behaves like its own main() function.
    int argBeginning = 2;
    char** argvSubset = &argv[argBeginning];
    int argcSubset = argc - argBeginning;

    InputParameters inputParams;
    XMLParser xmlParser;
    std::cout << "Using " << argv[1] << " as input parameter file." << std::endl;
    assert(xmlParser.parseXMLtoInputParameters(inputParams, argv[1]) == 0 && "Unable to succesfully parse XML file for input parameters. This file must exist, even if empty.");
    std::cout << "Finished loading input parameters. Proceeding..." << std::endl;

    LOG.SETLOGFILE(inputParams);
    std::ostringstream logStream;
    std::chrono::system_clock::time_point currentTime = std::chrono::system_clock::now();
    time_t timeTypeCurrentTime = std::chrono::system_clock::to_time_t(currentTime);
    logStream << std::endl << std::endl << "======================================" << std::endl <<
        "Initiating " << PROGRAMNAME << " at " << ctime(&timeTypeCurrentTime) << "Using input parameter file " << argv[1] << "." << std::endl;
    logStream << "\n\nWARNING\n";
    logStream << "WARNING\tThis program is still in an early stage of development.\n";
    logStream << "WARNING\tMany features are not implemented, are only partially working/developed, or are untested.\n";
    logStream << "WARNING\tThe developers are not liable for problems arising from use of this software. Use at your own risk.\n";
    logStream << "WARNING\n\n";
    logStream << "\nThe following arguments were provided:\n";
    for (int i = 0; i < argc; i++) {
        logStream << argv[i] << std::endl;
    }
    logStream << std::endl;
    LOG.DEBUG(logStream.str()); logStream.str("");
    logStream << "Logging output will be appended to " << inputParams.debuggingParameters.getLoggingPath() << std::endl;
    LOG.DEBUG(logStream.str()); logStream.str("");


    if (strcmp(argv[argBeginning], "--convertImageToCloud") == 0) {
        convertDepthImagesToPointCloud(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--registerPointCloudsICP_pairwise") == 0) {
        registerPointCloudsICP_pairwise(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--registerPointCloudsICP_oneAgainstGlobal") == 0) {
        registerPointCloudsICP_oneAgainstGlobal(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--registerPointCloudsICP_refinement") == 0) {
        registerPointCloudsICP_refinement(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--registerPointCloudsRANSACPrerejective") == 0) {
        registerPointCloudsRANSACPrerejective(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--registerKinectFusionPLYs") == 0) {
        registerKinectFusionPLYs(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--generateMesh") == 0) {
        meshGeneration(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--segmentationFromPLY") == 0) {
        segmentationFromPLY(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--screenshotPLY") == 0) {
        screenshotPLY(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--convertPCDToPLY") == 0) {
        convertPCDToPLY(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--convertPLYToPCD") == 0) {
        convertPLYToPCD(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--makePlantMeasurements") == 0) {
        makePlantMeasurements(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--removeStatisticalOutliers") == 0) {
        removeStatisticalOutliers(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--computePointFeatures") == 0) {
        computePointFeatures(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--constructSupervoxels") == 0) {
        supervoxelConstruction(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--segmentOutPot") == 0) {
        segmentOutPot(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--visualizeSpecificMeasurements") == 0) {
        visualizeSpecificMeasurements(argcSubset, argvSubset, inputParams);
    }
    else if (strcmp(argv[argBeginning], "--fitLSystemToPointCloud") == 0) {
        fitLSystemToPointCloud(argcSubset, argvSubset, inputParams);
    }
    else {
        std::cout << "Unable to find an expected processing selection." << std::endl;
        printUsageMessage();
    }
    return(0);
}
