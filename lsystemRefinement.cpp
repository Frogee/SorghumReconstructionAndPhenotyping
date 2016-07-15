
#include <algorithm>

#include <pcl/visualization/pcl_visualizer.h>


#include "loggingHelper.h"
#include "IterativeClosestPoint.h"
#include "lsystemFitting.h"
#include "lsystemRefinement.h"
#include "sampleMeshToPointCloud.h"

#include "tbb/parallel_for.h"
#include "tbb/blocked_range.h"


// Example adapted from: https://www.threadingbuildingblocks.org/docs/help/reference/algorithms/parallel_for_func.htm
class AverageWithVectors {
    public:
    std::vector<float> *input;
    std::vector<float> *output;
    void operator()( const tbb::blocked_range<int>& range ) const {
        for( int i=range.begin(); i!=range.end(); ++i )
            (*output)[i] = ( (*input)[i-1] + (*input)[i] + (*input)[i+1]) * (1/3.f);
    }
};

class LSystemFitnessResults {
    public:
        float registrationFitness;
        int numberPointsOfInputCloudAccountedFor;
        int numberPointsOfInputCloudUnaccountedFor;
        float proportionOfInputCloudAccounted;
        int numberPointsOfLSystemCloudAccounted;
        int numberPointsOfLSystemCloudUnaccounted;
        float proportionOfLSystemCloudUnaccounted;
        float combinedProportionScore;

};

class ParallelizedLSystemFitnessCalculator {
    public:
        Eigen::Matrix4f initialRotationMatrix;
        InputParameters inputParams;
        std::vector<LSystemParameters> *lSystemsToTest;
        std::vector<LSystemFitnessResults> *lSystemResultsContainer;
        std::vector<pcl::PolygonMesh> *lSystemMeshesToTest;
        pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo;

        void operator()( const tbb::blocked_range<int>& range ) const {
            for(int i = range.begin(); i != range.end(); ++i) {
                assert((*lSystemMeshesToTest).size() == (*lSystemResultsContainer).size() && "The size of the LSystem mesh vector and the size of the results vector must be the same. Aborting");

                pcl::PolygonMesh LSystemMeshToTest = (*lSystemMeshesToTest)[i];
                LSystemFitnessResults currentResults;

                // Because the random way points are sampled, sometimes you can get good fits from suboptimal parameters.
                // As such, we fit it an arbitrary number of times and take the average.
                float cumulativeRegistrationFitness = 0.0;
                float cumulativeProportionOfInputCloudAccounted = 0.0;
                float cumulativeProportionOfLSystemCloudUnaccounted = 0.0;
                float cumulativeCombinedProportionScore = 0.0;
                int cumulativeCombinedPointScore = 0;
                int cumulativeNumberPointsOfInputCloudAccountedFor = 0;
                int cumulativeNumberPointsOfInputCloudUnaccountedFor = 0;
                int cumulativeNumberPointsOfLSystemCloudAccounted = 0;
                int cumulativeNumberPointsOfLSystemCloudUnaccounted = 0;

                int numberOfTimesToFit = 2;
                for (uint32_t i = 0; i < numberOfTimesToFit; i++) {

                    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystemCloudToTest (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedLSystemCloudToTest  (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemCloudToTest  (new pcl::PointCloud<pcl::PointXYZ>);
                    sampleMeshToPointCloud_ForParallelCalling(&LSystemMeshToTest, *sampledLSystemCloudToTest, inputParams);
                    pcl::transformPointCloud(*sampledLSystemCloudToTest, *transformedLSystemCloudToTest, initialRotationMatrix);
                    float registrationFitness = registerLSystemICPAndReturnFitness_ForParallelCalling(transformedLSystemCloudToTest, inputCloudToFitLSystemTo, registeredLSystemCloudToTest, inputParams);

                    // I think we also need to be a bit more "greedy" than just correspondences. A nearest neighbor approach may work.
                    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchOriginalObject;
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsAcccounted (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsUnacccountedFor (new pcl::PointCloud<pcl::PointXYZ>);
                    kdtreeSearchOriginalObject.setInputCloud(registeredLSystemCloudToTest);

                    for (uint32_t j = 0; j < inputCloudToFitLSystemTo->points.size(); j++) {
                        pcl::PointXYZ currentPoint = inputCloudToFitLSystemTo->points[j];
                        std::vector<int> pointIndicesRadiusSearch;
                        std::vector<float> distanceRadiusSearch;
                        /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
                        /// This number determines the radius away from the L-System points that points in the original cloud will be considered accounted for in the L-system.
                        int numberNeighbors = kdtreeSearchOriginalObject.radiusSearch(currentPoint, 7.5, pointIndicesRadiusSearch, distanceRadiusSearch);
                        if (numberNeighbors >= 1) {
                            cloudOriginalObjectPointsAcccounted->points.push_back(currentPoint);
                        }
                        else {
                            cloudOriginalObjectPointsUnacccountedFor->points.push_back(currentPoint);
                        }
                    }

                    pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchLSystemCloud;
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLSystemPointsAcccounted (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLSystemPointsPointsUnacccountedFor (new pcl::PointCloud<pcl::PointXYZ>);
                    kdtreeSearchLSystemCloud.setInputCloud(inputCloudToFitLSystemTo);

                    for (uint32_t j = 0; j < registeredLSystemCloudToTest->points.size(); j++) {
                        pcl::PointXYZ currentPoint = registeredLSystemCloudToTest->points[j];
                        std::vector<int> pointIndicesRadiusSearch;
                        std::vector<float> distanceRadiusSearch;
                        /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
                        /// This number determines the search radius
                        int numberNeighbors = kdtreeSearchLSystemCloud.radiusSearch(currentPoint, 7.5, pointIndicesRadiusSearch, distanceRadiusSearch);
                        if (numberNeighbors >= 1) {
                            cloudLSystemPointsAcccounted->points.push_back(currentPoint);
                        }
                        else {
                            cloudLSystemPointsPointsUnacccountedFor->points.push_back(currentPoint);
                        }
                    }

                    float proportionOfInputCloudAccounted = (float)cloudOriginalObjectPointsAcccounted->size() / (float)inputCloudToFitLSystemTo->size();
                    cumulativeProportionOfInputCloudAccounted += proportionOfInputCloudAccounted;

                    float proportionOfLSystemCloudUnaccounted = (float)cloudLSystemPointsPointsUnacccountedFor->size() / (float)registeredLSystemCloudToTest->size();
                    cumulativeProportionOfLSystemCloudUnaccounted += proportionOfLSystemCloudUnaccounted;

                    cumulativeCombinedProportionScore += (proportionOfInputCloudAccounted - proportionOfLSystemCloudUnaccounted);

                    cumulativeCombinedPointScore += (int)cloudOriginalObjectPointsAcccounted->size() - (int)cloudLSystemPointsPointsUnacccountedFor->size();
                    cumulativeRegistrationFitness += registrationFitness;

                    cumulativeNumberPointsOfInputCloudAccountedFor += cloudOriginalObjectPointsAcccounted->size();
                    cumulativeNumberPointsOfInputCloudUnaccountedFor += cloudOriginalObjectPointsUnacccountedFor->size();
                    cumulativeNumberPointsOfLSystemCloudAccounted += cloudLSystemPointsAcccounted->size();
                    cumulativeNumberPointsOfLSystemCloudUnaccounted += cloudLSystemPointsPointsUnacccountedFor->size();

                }

                currentResults.registrationFitness = cumulativeRegistrationFitness / (float)numberOfTimesToFit;
                currentResults.combinedProportionScore = cumulativeCombinedProportionScore / (float)numberOfTimesToFit;
                currentResults.numberPointsOfInputCloudAccountedFor = cumulativeNumberPointsOfInputCloudAccountedFor / numberOfTimesToFit;
                currentResults.numberPointsOfInputCloudUnaccountedFor = cumulativeNumberPointsOfInputCloudUnaccountedFor / numberOfTimesToFit;
                currentResults.proportionOfInputCloudAccounted = cumulativeProportionOfInputCloudAccounted / (float)numberOfTimesToFit;
                currentResults.numberPointsOfLSystemCloudAccounted = cumulativeNumberPointsOfLSystemCloudAccounted / numberOfTimesToFit;
                currentResults.numberPointsOfLSystemCloudUnaccounted = cumulativeNumberPointsOfLSystemCloudUnaccounted / numberOfTimesToFit;
                currentResults.proportionOfLSystemCloudUnaccounted = cumulativeProportionOfLSystemCloudUnaccounted / (float)numberOfTimesToFit;
                (*lSystemResultsContainer)[i] = currentResults;

            }
        }

};


LSystemParameters refineLSystemParallel_ExpandedStem_InternodeLength(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams) {
    std::ostringstream logStream;
    LSystemParameters originalLSystemParameters = inputLsystemParams;
    LSystemParameters lSystemToExplore = inputLsystemParams;
    LSystemParameters lSystemToReturn = inputLsystemParams;
    LOG.DEBUG("Attempting to refine L-system Parameters for internode length using a parallelized implementation.");

    pcl::PolygonMesh LSystemMesh = buildLSystem(inputLpyFunction, inputLsystemParams, visu, inputParams);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystem (new pcl::PointCloud<pcl::PointXYZ>);
    sampleMeshToPointCloud(&LSystemMesh, *sampledLSystem, inputParams);

    pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f initialRotationMatrix = registerLSystemICPAndReturnTranslationMatrix(sampledLSystem, inputCloudToFitLSystemTo, registeredLSystemPointCloud, visu, inputParams);

    std::vector<float> retainedInternodeLengths = originalLSystemParameters.getInternodeLengths();
    for (uint32_t i = 0; i < phytomerIndicesToRefine_OneBased.size(); i++) {
        int currentPhytomerIndex_OneBased = phytomerIndicesToRefine_OneBased[i];
        int currentPhytomerIndex = currentPhytomerIndex_OneBased - 1;
        float currentInternodeLength = inputLsystemParams.getInternodeLengths()[currentPhytomerIndex];

        std::vector<LSystemParameters> parameterContainerToTest;
        std::vector<pcl::PolygonMesh> lsystemMeshesToTest;
        std::vector<LSystemFitnessResults> resultsFromTesting;

        // First, find the range of values we want to exhaustively test. How about a 50% variation in either direction in 5 % intervals?
        // These need to be used to construct a vector of L systems that we can test in parallel.

        for (float currentProportion = 0.7; currentProportion <= 1.301; currentProportion = currentProportion + 0.1) {
            LSystemParameters lsystemParametersToTest;
            LSystemFitnessResults lsystemFitness;
            lsystemParametersToTest = inputLsystemParams;

            logStream << "Adding the proportion of " << currentProportion << " for internode height of phytomer " << currentPhytomerIndex_OneBased << " for testing in parallel.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            std::vector<float> currentInternodeLengths = retainedInternodeLengths;
            std::vector<float> originalInternodeLengths = originalLSystemParameters.getInternodeLengths();
            // The total height of the plant needs to remain the same. All phytomers below the one being changed can remain the same,
            //  but the phytomer above it needs to have the difference added. e.g. plant is 2 units tall. phytomer 1 is .8,
            //  phytomer 2 is .5, phytomer 3 is .7. Phytomer two + phytomer 3 always need to add to 1.2
            // This doesn't apply for the bottom phytomer.
            // So, if not the top or bottom phytomer:
            if (currentPhytomerIndex < originalInternodeLengths.size() && currentPhytomerIndex != 0) {
                float originalCombinedHeight = originalInternodeLengths[currentPhytomerIndex + 1] + originalInternodeLengths[currentPhytomerIndex];
                float bottomPhytomerValue = currentProportion * originalInternodeLengths[currentPhytomerIndex];
                float upperPhytomerValue = originalCombinedHeight - bottomPhytomerValue;
                currentInternodeLengths[currentPhytomerIndex + 1] = upperPhytomerValue;
                currentInternodeLengths[currentPhytomerIndex] = bottomPhytomerValue;

            }
            //else if the top or bottom phytomer:
            else {
                currentInternodeLengths[currentPhytomerIndex] = currentProportion * originalInternodeLengths[currentPhytomerIndex];
            }
            logStream << "The following internode lengths for phytomer " << currentPhytomerIndex << " will be added for this proportion:\t";
            for (uint32_t j = 0; j < currentInternodeLengths.size(); j++) {
                logStream << currentInternodeLengths[j] << "\t";
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            lsystemParametersToTest.setInternodeLengths(currentInternodeLengths);

            pcl::PolygonMesh LSystemMeshToTest = buildLSystem(inputLpyFunction, lsystemParametersToTest, visu, inputParams);

            parameterContainerToTest.push_back(lsystemParametersToTest);
            lsystemMeshesToTest.push_back(LSystemMeshToTest);
            resultsFromTesting.push_back(lsystemFitness);

        }

        ParallelizedLSystemFitnessCalculator fitnessCalculator;
        fitnessCalculator.lSystemsToTest = &parameterContainerToTest;
        fitnessCalculator.lSystemResultsContainer = &resultsFromTesting;
        fitnessCalculator.initialRotationMatrix = initialRotationMatrix;
        fitnessCalculator.inputCloudToFitLSystemTo = inputCloudToFitLSystemTo;
        fitnessCalculator.lSystemMeshesToTest = &lsystemMeshesToTest;
        fitnessCalculator.inputParams = inputParams;

        LOG.DEBUG("\nBeginning parallel search over Lsystems. This may take a moment.\n");
        tbb::parallel_for( tbb::blocked_range<int>( 0, (*fitnessCalculator.lSystemResultsContainer).size() ), fitnessCalculator );
        LOG.DEBUG("Finished parallel search.");

        int bestInternodeFitIndex = 0;
        int bestPointsAccounted = 0;
        float bestRegistrationFitness = 9999999;
        int bestCombinedPointScore = -9999999;
        float bestCombinedProportionScore = -9999999;

        for (uint32_t i = 0; i < (*fitnessCalculator.lSystemResultsContainer).size(); i++) {
            LSystemFitnessResults currentResult = (*fitnessCalculator.lSystemResultsContainer)[i];
            if (currentResult.combinedProportionScore > bestCombinedProportionScore) {
                bestInternodeFitIndex = i;
                bestCombinedProportionScore = currentResult.combinedProportionScore;
            }
        }

        logStream << "For derivation " << currentPhytomerIndex_OneBased << ", the original internode lengths were:\n";
        for (uint32_t j = 0; j < originalLSystemParameters.getInternodeLengths().size(); j++) {
            logStream << originalLSystemParameters.getInternodeLengths()[j] << "\t";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        logStream << "The best combined proportion score was " << bestCombinedProportionScore << " for the following internode set:\n";
        LSystemParameters bestLSystemParams = (*fitnessCalculator.lSystemsToTest)[bestInternodeFitIndex];
        std::vector<float> bestInternodeLengths = bestLSystemParams.getInternodeLengths();
        for (uint32_t j = 0; j < bestInternodeLengths.size(); j++) {
                logStream << bestInternodeLengths[j] << "\t";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        retainedInternodeLengths = bestInternodeLengths;
        lSystemToReturn.setInternodeLengths(bestInternodeLengths);

    }

    return lSystemToReturn;
}


LSystemParameters refineLSystemParallel_ExpandedStem_LeafPhyllotaxyAngle(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams) {
    std::ostringstream logStream;
    LSystemParameters originalLSystemParameters = inputLsystemParams;
    LSystemParameters lSystemToExplore = inputLsystemParams;
    LSystemParameters lSystemToReturn = inputLsystemParams;
    LOG.DEBUG("Attempting to refine L-system Parameters for leaf phyllotaxy angle using a parallelized implementation.");

    pcl::PolygonMesh LSystemMesh = buildLSystem(inputLpyFunction, inputLsystemParams, visu, inputParams);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystem (new pcl::PointCloud<pcl::PointXYZ>);
    sampleMeshToPointCloud(&LSystemMesh, *sampledLSystem, inputParams);

    pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f initialRotationMatrix = registerLSystemICPAndReturnTranslationMatrix(sampledLSystem, inputCloudToFitLSystemTo, registeredLSystemPointCloud, visu, inputParams);

    std::vector<float> retainedLeafPhyllotaxyAngles = originalLSystemParameters.getLeafPhyllotaxyAngles();
    for (uint32_t i = 0; i < phytomerIndicesToRefine_OneBased.size(); i++) {
        int currentPhytomerIndex_OneBased = phytomerIndicesToRefine_OneBased[i];
        int currentPhytomerIndex = currentPhytomerIndex_OneBased - 1;
        float currentLeafPhyllotaxyAngles = inputLsystemParams.getLeafPhyllotaxyAngles()[currentPhytomerIndex];

        std::vector<LSystemParameters> parameterContainerToTest;
        std::vector<pcl::PolygonMesh> lsystemMeshesToTest;
        std::vector<LSystemFitnessResults> resultsFromTesting;

        // First, find the range of values we want to exhaustively test. How about a 50% variation in either direction in 5 % intervals?
        // These need to be used to construct a vector of L systems that we can test in parallel.

        for (float currentProportion = 0.7; currentProportion <= 1.301; currentProportion = currentProportion + 0.1) {
            LSystemParameters lsystemParametersToTest;
            LSystemFitnessResults lsystemFitness;
            lsystemParametersToTest = inputLsystemParams;

            logStream << "Adding the proportion of " << currentProportion << " for leaf phyllotaxy of phytomer " << currentPhytomerIndex_OneBased << " for testing in parallel.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            std::vector<float> currentLeafPhyllotaxyAngles = retainedLeafPhyllotaxyAngles;
            std::vector<float> originalLeafPhyllotaxyAngles = originalLSystemParameters.getLeafPhyllotaxyAngles();

            currentLeafPhyllotaxyAngles[currentPhytomerIndex] = currentProportion * originalLeafPhyllotaxyAngles[currentPhytomerIndex];

            logStream << "The following phyllotaxy angles for phytomer " << currentPhytomerIndex << " will be added for this proportion:\t";
            for (uint32_t j = 0; j < currentLeafPhyllotaxyAngles.size(); j++) {
                logStream << currentLeafPhyllotaxyAngles[j] << "\t";
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            lsystemParametersToTest.setLeafPhyllotaxyAngles(currentLeafPhyllotaxyAngles);

            pcl::PolygonMesh LSystemMeshToTest = buildLSystem(inputLpyFunction, lsystemParametersToTest, visu, inputParams);

            parameterContainerToTest.push_back(lsystemParametersToTest);
            lsystemMeshesToTest.push_back(LSystemMeshToTest);
            resultsFromTesting.push_back(lsystemFitness);

        }

        ParallelizedLSystemFitnessCalculator fitnessCalculator;
        fitnessCalculator.lSystemsToTest = &parameterContainerToTest;
        fitnessCalculator.lSystemResultsContainer = &resultsFromTesting;
        fitnessCalculator.initialRotationMatrix = initialRotationMatrix;
        fitnessCalculator.inputCloudToFitLSystemTo = inputCloudToFitLSystemTo;
        fitnessCalculator.lSystemMeshesToTest = &lsystemMeshesToTest;
        fitnessCalculator.inputParams = inputParams;

        LOG.DEBUG("\nBeginning parallel search over Lsystems. This may take a moment.\n");
        tbb::parallel_for( tbb::blocked_range<int>( 0, (*fitnessCalculator.lSystemResultsContainer).size() ), fitnessCalculator );
        LOG.DEBUG("Finished parallel search.");

        int bestLeafPhyllotaxyAnglesFitIndex = 0;
        int bestPointsAccounted = 0;
        float bestRegistrationFitness = 9999999;
        int bestCombinedPointScore = -9999999;
        float bestCombinedProportionScore = -9999999;

        for (uint32_t i = 0; i < (*fitnessCalculator.lSystemResultsContainer).size(); i++) {
            LSystemFitnessResults currentResult = (*fitnessCalculator.lSystemResultsContainer)[i];
            if (currentResult.combinedProportionScore > bestCombinedProportionScore) {
                bestLeafPhyllotaxyAnglesFitIndex = i;
                bestCombinedProportionScore = currentResult.combinedProportionScore;
            }
        }

        logStream << "For derivation " << currentPhytomerIndex_OneBased << ", the original phyllotaxy angles were:\n";
        for (uint32_t j = 0; j < originalLSystemParameters.getLeafPhyllotaxyAngles().size(); j++) {
            logStream << originalLSystemParameters.getLeafPhyllotaxyAngles()[j] << "\t";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        logStream << "The best combined proportion score was " << bestCombinedProportionScore << " for the following internode set:\n";
        LSystemParameters bestLSystemParams = (*fitnessCalculator.lSystemsToTest)[bestLeafPhyllotaxyAnglesFitIndex];
        std::vector<float> bestLeafPhyllotaxyAngles = bestLSystemParams.getLeafPhyllotaxyAngles();
        for (uint32_t j = 0; j < bestLeafPhyllotaxyAngles.size(); j++) {
                logStream << bestLeafPhyllotaxyAngles[j] << "\t";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        retainedLeafPhyllotaxyAngles = bestLeafPhyllotaxyAngles;
        lSystemToReturn.setLeafPhyllotaxyAngles(bestLeafPhyllotaxyAngles);

    }

    return lSystemToReturn;
}

LSystemParameters refineLSystemParallel_ExpandedStem_LeafCurvaturesControlPointTwo(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams) {
    std::ostringstream logStream;
    LSystemParameters originalLSystemParameters = inputLsystemParams;
    LSystemParameters lSystemToExplore = inputLsystemParams;
    LSystemParameters lSystemToReturn = inputLsystemParams;
    LOG.DEBUG("Attempting to refine L-system Parameters for leaf curvature using a parallelized implementation.");

    pcl::PolygonMesh LSystemMesh = buildLSystem(inputLpyFunction, inputLsystemParams, visu, inputParams);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystem (new pcl::PointCloud<pcl::PointXYZ>);
    sampleMeshToPointCloud(&LSystemMesh, *sampledLSystem, inputParams);

    pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f initialRotationMatrix = registerLSystemICPAndReturnTranslationMatrix(sampledLSystem, inputCloudToFitLSystemTo, registeredLSystemPointCloud, visu, inputParams);

    std::vector< std::vector < std::pair<float, float> > > retainedLeafCurvatures = originalLSystemParameters.getLeafCurvatures();
    for (uint32_t i = 0; i < phytomerIndicesToRefine_OneBased.size(); i++) {
        int currentPhytomerIndex_OneBased = phytomerIndicesToRefine_OneBased[i];
        int currentPhytomerIndex = currentPhytomerIndex_OneBased - 1;

        std::vector<LSystemParameters> parameterContainerToTest;
        std::vector<pcl::PolygonMesh> lsystemMeshesToTest;
        std::vector<LSystemFitnessResults> resultsFromTesting;


        // First, find the range of values we want to exhaustively test. How about a 50% variation in either direction in 5 % intervals?
        // Leaf curvatures are going to be a bit different since they are a pair of values for each control point.
        // Since we're only looking at the extended cylinder around the plant, let's take advantage of that and assume very little curvature
        // has occurred, and that only control point 2 matters. Then, we can rotate a line of fixed length around the origin (0 to 90 degrees) and
        // see which fits best.
        // First, we'll need an estimate of how long the control point vector should be. We can get that based off of the stem radius.
        float internodeRadius = inputLsystemParams.getInternodeRadii()[currentPhytomerIndex];
        float lengthOfControlPointVector = (internodeRadius + (inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue()) * 1.1) * 7.0;  // magic numbers obtained from cylinder fitting in "lsystemFitting.cpp"
        //sin(theta) / cos (theta) = y / x

        for (float angleToTest = -70.0; angleToTest <= 70.0; angleToTest+= 10.0) {

            LSystemParameters lsystemParametersToTest;
            LSystemFitnessResults lsystemFitness;
            lsystemParametersToTest = inputLsystemParams;

            logStream << "Adding angle of " << angleToTest << " for leaf curvature of phytomer " << currentPhytomerIndex_OneBased << " for testing in parallel.";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            float angleRad = angleToTest * (M_PI / 180.0);
            float xCoord = cos(angleRad) * lengthOfControlPointVector;
            float yCoord = sin(angleRad) * lengthOfControlPointVector;

            std::vector< std::vector < std::pair<float, float> > > currentLeafCurvatures = retainedLeafCurvatures;
            std::vector< std::vector < std::pair<float, float> > > originalLeafCurvatures = originalLSystemParameters.getLeafCurvatures();

            // Set the 2nd, 3rd, and 4th control points to be nearly the same.
            (currentLeafCurvatures[currentPhytomerIndex])[1].first = xCoord;
            (currentLeafCurvatures[currentPhytomerIndex])[1].second = yCoord;
            (currentLeafCurvatures[currentPhytomerIndex])[2].first = xCoord * 1.000001;
            (currentLeafCurvatures[currentPhytomerIndex])[2].second = yCoord * 1.000001;
            (currentLeafCurvatures[currentPhytomerIndex])[3].first = xCoord * 1.000002;
            (currentLeafCurvatures[currentPhytomerIndex])[3].second = yCoord * 1.000002;

            logStream << "The following leaf curvatures for phytomer " << currentPhytomerIndex << " will be added for this angle:\n";
            for (uint32_t j = 0; j < currentLeafCurvatures.size(); j++) {
                for (uint32_t k = 0; k < currentLeafCurvatures[j].size(); k++) {
                    logStream << "(" << currentLeafCurvatures[j][k].first << ", " << currentLeafCurvatures[j][k].second << ")\t";
                }
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            }

            lsystemParametersToTest.setLeafCurvatures(currentLeafCurvatures);

            pcl::PolygonMesh LSystemMeshToTest = buildLSystem(inputLpyFunction, lsystemParametersToTest, visu, inputParams);

            parameterContainerToTest.push_back(lsystemParametersToTest);
            lsystemMeshesToTest.push_back(LSystemMeshToTest);
            resultsFromTesting.push_back(lsystemFitness);

        }

        ParallelizedLSystemFitnessCalculator fitnessCalculator;
        fitnessCalculator.lSystemsToTest = &parameterContainerToTest;
        fitnessCalculator.lSystemResultsContainer = &resultsFromTesting;
        fitnessCalculator.initialRotationMatrix = initialRotationMatrix;
        fitnessCalculator.inputCloudToFitLSystemTo = inputCloudToFitLSystemTo;
        fitnessCalculator.lSystemMeshesToTest = &lsystemMeshesToTest;
        fitnessCalculator.inputParams = inputParams;

        LOG.DEBUG("\nBeginning parallel search over Lsystems. This may take a moment.\n");
        tbb::parallel_for( tbb::blocked_range<int>( 0, (*fitnessCalculator.lSystemResultsContainer).size() ), fitnessCalculator );
        LOG.DEBUG("Finished parallel search.");

        int bestLeafCurvaturesFitIndex = 0;
        int bestPointsAccounted = 0;
        float bestRegistrationFitness = 9999999;
        int bestCombinedPointScore = -9999999;
        float bestCombinedProportionScore = -9999999;

        for (uint32_t i = 0; i < (*fitnessCalculator.lSystemResultsContainer).size(); i++) {
            LSystemFitnessResults currentResult = (*fitnessCalculator.lSystemResultsContainer)[i];
            if (currentResult.combinedProportionScore > bestCombinedProportionScore) {
                bestLeafCurvaturesFitIndex = i;
                bestCombinedProportionScore = currentResult.combinedProportionScore;
            }
        }

        logStream << "For derivation " << currentPhytomerIndex_OneBased << ", the original curvature set was:\n";
        for (uint32_t j = 0; j < originalLSystemParameters.getLeafCurvatures().size(); j++) {
            for (uint32_t k= 0; k < originalLSystemParameters.getLeafCurvatures()[j].size(); k++) {
                logStream << "(" << originalLSystemParameters.getLeafCurvatures()[j][k].first << ", " <<  originalLSystemParameters.getLeafCurvatures()[j][k].second << ")\t";
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        }

        logStream << "The best combined proportion score was " << bestCombinedProportionScore << " for the following curvature set:\n";
        LSystemParameters bestLSystemParams = (*fitnessCalculator.lSystemsToTest)[bestLeafCurvaturesFitIndex];
        std::vector< std::vector < std::pair<float, float> > > bestLeafCurvatures = bestLSystemParams.getLeafCurvatures();
        for (uint32_t j = 0; j < bestLeafCurvatures.size(); j++) {
            for (uint32_t k= 0; k < bestLeafCurvatures[j].size(); k++) {
                logStream << "(" << bestLeafCurvatures[j][k].first << ", " << bestLeafCurvatures[j][k].second << ")\t";
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        }
        retainedLeafCurvatures = bestLeafCurvatures;
        lSystemToReturn.setLeafCurvatures(bestLeafCurvatures);

    }

    return lSystemToReturn;
}



LSystemParameters refineLSystem_ExpandedStem_InternodeLength(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams) {
    std::ostringstream logStream;
    LSystemParameters originalLSystemParameters = inputLsystemParams;
    LSystemParameters lSystemToExplore = inputLsystemParams;
    LSystemParameters lSystemToReturn = inputLsystemParams;
    LOG.DEBUG("Attempting to refine L-system Parameters.");

    /// What should be the strategy for refinement?
    /// A decent first approximation could be to maximize point correspondences between the LSystem and this part of the cloud.
    /// We already have a basic template of the L-System. We could modify it over a range of values, check correspondences for each value, and choose the max.
    pcl::PolygonMesh LSystemMesh = buildLSystem(inputLpyFunction, inputLsystemParams, visu, inputParams);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystem (new pcl::PointCloud<pcl::PointXYZ>);
    sampleMeshToPointCloud(&LSystemMesh, *sampledLSystem, inputParams);

    pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f initialRotationMatrix = registerLSystemICPAndReturnTranslationMatrix(sampledLSystem, inputCloudToFitLSystemTo, registeredLSystemPointCloud, visu, inputParams);

    // Let's try refining internode length first.
    std::vector<float> retainedInternodeLengths = originalLSystemParameters.getInternodeLengths();
    for (uint32_t i = 0; i < phytomerIndicesToRefine_OneBased.size(); i++) {
        int currentPhytomerIndex_OneBased = phytomerIndicesToRefine_OneBased[i];
        int currentPhytomerIndex = currentPhytomerIndex_OneBased - 1;
        float currentInternodeLength = inputLsystemParams.getInternodeLengths()[currentPhytomerIndex];

        // First, find the range of values we want to exhaustively test. How about a 50% variation in either direction in 5 % intervals?
        float currentProportion = 0.7;
        std::vector<float> bestInternodeLengths;
        int bestInternodeFit = 0;
        int bestPointsAccounted = 0;
        float bestRegistrationFitness = 9999999;
        int bestCombinedPointScore = -9999999;
        float bestCombinedProportionScore = -9999999;
        while (currentProportion < 1.3) {
            logStream << "Testing the current proportion of " << currentProportion << " for internode height of phytomer " << currentPhytomerIndex_OneBased;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            std::vector<float> currentInternodeLengths = retainedInternodeLengths;
            std::vector<float> originalInternodeLengths = originalLSystemParameters.getInternodeLengths();
            // The total height of the plant needs to remain the same. All phytomers below the one being changed can remain the same,
            //  but the phytomer above it needs to have the difference added. e.g. plant is 2 units tall. phytomer 1 is .8,
            //  phytomer 2 is .5, phytomer 3 is .7. Phytomer two + phytomer 3 always need to add to 1.2
            // This doesn't apply for the bottom phytomer.
            // So, if not the top or bottom phytomer:
            if (currentPhytomerIndex < originalInternodeLengths.size() && currentPhytomerIndex != 0) {
                float originalCombinedHeight = originalInternodeLengths[currentPhytomerIndex + 1] + originalInternodeLengths[currentPhytomerIndex];
                float bottomPhytomerValue = currentProportion * originalInternodeLengths[currentPhytomerIndex];
                float upperPhytomerValue = originalCombinedHeight - bottomPhytomerValue;
                currentInternodeLengths[currentPhytomerIndex + 1] = upperPhytomerValue;
                currentInternodeLengths[currentPhytomerIndex] = bottomPhytomerValue;

            }
            //else if the top or bottom phytomer:
            else {
                currentInternodeLengths[currentPhytomerIndex] = currentProportion * originalInternodeLengths[currentPhytomerIndex];
            }
            logStream << "Testing internode lengths for phytomer " << currentPhytomerIndex << " using:\t";
            for (uint32_t j = 0; j < currentInternodeLengths.size(); j++) {
                logStream << currentInternodeLengths[j] << "\t";
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");


            lSystemToExplore.setInternodeLengths(currentInternodeLengths);

            pcl::PolygonMesh LSystemMeshToTest = buildLSystem(inputLpyFunction, lSystemToExplore, visu, inputParams);
            pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystemCloudToTest (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedLSystemCloudToTest  (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemCloudToTest  (new pcl::PointCloud<pcl::PointXYZ>);
            sampleMeshToPointCloud(&LSystemMeshToTest, *sampledLSystemCloudToTest, inputParams);
            pcl::transformPointCloud(*sampledLSystemCloudToTest, *transformedLSystemCloudToTest, initialRotationMatrix);
            float registrationFitness = registerLSystemICPAndReturnFitness(transformedLSystemCloudToTest, inputCloudToFitLSystemTo, registeredLSystemCloudToTest, visu, inputParams);

            // I think we also need to be a bit more "greedy" than just correspondences. A nearest neighbor approach may work.
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchOriginalObject;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsAcccounted (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsUnacccountedFor (new pcl::PointCloud<pcl::PointXYZ>);
            kdtreeSearchOriginalObject.setInputCloud(registeredLSystemCloudToTest);

            for (uint32_t j = 0; j < inputCloudToFitLSystemTo->points.size(); j++) {
                pcl::PointXYZ currentPoint = inputCloudToFitLSystemTo->points[j];
                std::vector<int> pointIndicesRadiusSearch;
                std::vector<float> distanceRadiusSearch;
                /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
                /// This number determines the radius away from the L-System points that points in the original cloud will be considered accounted for in the L-system.
                int numberNeighbors = kdtreeSearchOriginalObject.radiusSearch(currentPoint, 7.5, pointIndicesRadiusSearch, distanceRadiusSearch);
                if (numberNeighbors >= 1) {
                    cloudOriginalObjectPointsAcccounted->points.push_back(currentPoint);
                }
                else {
                    cloudOriginalObjectPointsUnacccountedFor->points.push_back(currentPoint);
                }
            }

            pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchLSystemCloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLSystemPointsAcccounted (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLSystemPointsPointsUnacccountedFor (new pcl::PointCloud<pcl::PointXYZ>);
            kdtreeSearchLSystemCloud.setInputCloud(inputCloudToFitLSystemTo);

            for (uint32_t j = 0; j < registeredLSystemCloudToTest->points.size(); j++) {
                pcl::PointXYZ currentPoint = registeredLSystemCloudToTest->points[j];
                std::vector<int> pointIndicesRadiusSearch;
                std::vector<float> distanceRadiusSearch;
                /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
                /// This number determines the search radius
                int numberNeighbors = kdtreeSearchLSystemCloud.radiusSearch(currentPoint, 7.5, pointIndicesRadiusSearch, distanceRadiusSearch);
                if (numberNeighbors >= 1) {
                    cloudLSystemPointsAcccounted->points.push_back(currentPoint);
                }
                else {
                    cloudLSystemPointsPointsUnacccountedFor->points.push_back(currentPoint);
                }
            }

            float proportionOfInputCloudAccounted = (float)cloudOriginalObjectPointsAcccounted->size() / (float)inputCloudToFitLSystemTo->size();
            float proportionOfLSystemCloudUnaccounted = (float)cloudLSystemPointsPointsUnacccountedFor->size() / (float)registeredLSystemCloudToTest->size();

            logStream << "Of the " << inputCloudToFitLSystemTo->size() << " points in the input cloud, the L-system accounts for " <<
                                cloudOriginalObjectPointsAcccounted->size() << " (" << proportionOfInputCloudAccounted << "), and fails to account for " << cloudOriginalObjectPointsUnacccountedFor->size() << "." << std::endl;
            logStream << "Of the " << registeredLSystemCloudToTest->size() << " points in the L-system, the input cloud accounts for " <<
                                cloudLSystemPointsAcccounted->size() << ", and fails to account for " << cloudLSystemPointsPointsUnacccountedFor->size() <<
                                    " (" << proportionOfLSystemCloudUnaccounted << ")." << std::endl;
            float combinedProportionScore = proportionOfInputCloudAccounted - proportionOfLSystemCloudUnaccounted;
            int combinedPointScore= (int)cloudOriginalObjectPointsAcccounted->size() - (int)cloudLSystemPointsPointsUnacccountedFor->size();
            logStream << "Combined point score of this fit is " << combinedPointScore << ", combined proportion score is " << combinedProportionScore <<
                            " and registration fitness is " << registrationFitness << ".";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            if (combinedProportionScore >= bestCombinedProportionScore) {
                logStream << "Recording new combined proportion score of. " << combinedProportionScore;
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                bestRegistrationFitness = registrationFitness;
                bestPointsAccounted = cloudOriginalObjectPointsAcccounted->size();
                bestCombinedPointScore = cloudOriginalObjectPointsAcccounted->size() - cloudLSystemPointsPointsUnacccountedFor->size();
                bestInternodeLengths = currentInternodeLengths;
                bestCombinedProportionScore = combinedProportionScore;
                retainedInternodeLengths = bestInternodeLengths;

            }
            currentProportion += 0.1;
        }
        logStream << "The original internode set was:\n";
        for (uint32_t j = 0; j < originalLSystemParameters.getInternodeLengths().size(); j++) {
            logStream << originalLSystemParameters.getInternodeLengths()[j] << "\t";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        logStream << "The best fitting internode set for phytomer " << currentPhytomerIndex << " was found to be:\n";
        for (uint32_t j = 0; j < bestInternodeLengths.size(); j++) {
            logStream << bestInternodeLengths[j] << "\t";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    }

    lSystemToReturn.setInternodeLengths(retainedInternodeLengths);
    return(lSystemToReturn);
}


LSystemParameters refineLSystem_ExpandedStem_LeafPhyllotaxyAngle(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams) {
    std::ostringstream logStream;
    LSystemParameters originalLSystemParameters = inputLsystemParams;
    LSystemParameters lSystemToExplore = inputLsystemParams;
    LSystemParameters lSystemToReturn = inputLsystemParams;
    LOG.DEBUG("Attempting to refine L-system Parameters.");

    /// What should be the strategy for refinement?
    /// A decent first approximation could be to maximize point correspondences between the LSystem and this part of the cloud.
    /// We already have a basic template of the L-System. We could modify it over a range of values, check correspondences for each value, and choose the max.
    pcl::PolygonMesh LSystemMesh = buildLSystem(inputLpyFunction, inputLsystemParams, visu, inputParams);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystem (new pcl::PointCloud<pcl::PointXYZ>);
    sampleMeshToPointCloud(&LSystemMesh, *sampledLSystem, inputParams);

    pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f initialRotationMatrix = registerLSystemICPAndReturnTranslationMatrix(sampledLSystem, inputCloudToFitLSystemTo, registeredLSystemPointCloud, visu, inputParams);

    std::vector<float> retainedLeafPhyllotaxyAngles = originalLSystemParameters.getLeafPhyllotaxyAngles();
    for (uint32_t i = 0; i < phytomerIndicesToRefine_OneBased.size(); i++) {
        int currentPhytomerIndex_OneBased = phytomerIndicesToRefine_OneBased[i];
        int currentPhytomerIndex = currentPhytomerIndex_OneBased - 1;
        float currentLeafPhyllotaxyAngles = inputLsystemParams.getLeafPhyllotaxyAngles()[currentPhytomerIndex];

        // First, find the range of values we want to exhaustively test. How about a 50% variation in either direction in 5 % intervals?
        float currentProportion = 0.7;
        std::vector<float> bestLeafPhyllotaxyAngles;
        int bestLeafPhyllotaxyAnglesFit = 0;
        int bestPointsAccounted = 0;
        float bestRegistrationFitness = 9999999;
        int bestCombinedPointScore = -9999999;
        float bestCombinedProportionScore = -9999999;
        while (currentProportion <= 1.3) {
            logStream << "Testing the current proportion of " << currentProportion << " for leaf phyllotaxy angle of phytomer " << currentPhytomerIndex_OneBased;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            std::vector<float> currentLeafPhyllotaxyAngles = retainedLeafPhyllotaxyAngles;
            std::vector<float> originalLeafPhyllotaxyAngles = originalLSystemParameters.getLeafPhyllotaxyAngles();

            currentLeafPhyllotaxyAngles[currentPhytomerIndex] = currentProportion * originalLeafPhyllotaxyAngles[currentPhytomerIndex];

            logStream << "Testing leaf phyllotaxy angles for phytomer " << currentPhytomerIndex << " using:\t";
            for (uint32_t j = 0; j < currentLeafPhyllotaxyAngles.size(); j++) {
                logStream << currentLeafPhyllotaxyAngles[j] << "\t";
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");


            lSystemToExplore.setLeafPhyllotaxyAngles(currentLeafPhyllotaxyAngles);

            pcl::PolygonMesh LSystemMeshToTest = buildLSystem(inputLpyFunction, lSystemToExplore, visu, inputParams);
            pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystemCloudToTest (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedLSystemCloudToTest  (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemCloudToTest  (new pcl::PointCloud<pcl::PointXYZ>);
            sampleMeshToPointCloud(&LSystemMeshToTest, *sampledLSystemCloudToTest, inputParams);
            pcl::transformPointCloud(*sampledLSystemCloudToTest, *transformedLSystemCloudToTest, initialRotationMatrix);
            float registrationFitness = registerLSystemICPAndReturnFitness(transformedLSystemCloudToTest, inputCloudToFitLSystemTo, registeredLSystemCloudToTest, visu, inputParams);

            // I think we also need to be a bit more "greedy" than just correspondences. A nearest neighbor approach may work.
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchOriginalObject;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsAcccounted (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsUnacccountedFor (new pcl::PointCloud<pcl::PointXYZ>);
            kdtreeSearchOriginalObject.setInputCloud(registeredLSystemCloudToTest);

            for (uint32_t j = 0; j < inputCloudToFitLSystemTo->points.size(); j++) {
                pcl::PointXYZ currentPoint = inputCloudToFitLSystemTo->points[j];
                std::vector<int> pointIndicesRadiusSearch;
                std::vector<float> distanceRadiusSearch;
                /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
                /// This number determines the radius away from the L-System points that points in the original cloud will be considered accounted for in the L-system.
                int numberNeighbors = kdtreeSearchOriginalObject.radiusSearch(currentPoint, 7.5, pointIndicesRadiusSearch, distanceRadiusSearch);
                if (numberNeighbors >= j) {
                    cloudOriginalObjectPointsAcccounted->points.push_back(currentPoint);
                }
                else {
                    cloudOriginalObjectPointsUnacccountedFor->points.push_back(currentPoint);
                }
            }

            pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchLSystemCloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLSystemPointsAcccounted (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLSystemPointsPointsUnacccountedFor (new pcl::PointCloud<pcl::PointXYZ>);
            kdtreeSearchLSystemCloud.setInputCloud(inputCloudToFitLSystemTo);

            for (uint32_t j = 0; j < registeredLSystemCloudToTest->points.size(); j++) {
                pcl::PointXYZ currentPoint = registeredLSystemCloudToTest->points[j];
                std::vector<int> pointIndicesRadiusSearch;
                std::vector<float> distanceRadiusSearch;
                /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
                /// This number determines the search radius
                int numberNeighbors = kdtreeSearchLSystemCloud.radiusSearch(currentPoint, 7.5, pointIndicesRadiusSearch, distanceRadiusSearch);
                if (numberNeighbors >= 1) {
                    cloudLSystemPointsAcccounted->points.push_back(currentPoint);
                }
                else {
                    cloudLSystemPointsPointsUnacccountedFor->points.push_back(currentPoint);
                }
            }

            float proportionOfInputCloudAccounted = (float)cloudOriginalObjectPointsAcccounted->size() / (float)inputCloudToFitLSystemTo->size();
            float proportionOfLSystemCloudUnaccounted = (float)cloudLSystemPointsPointsUnacccountedFor->size() / (float)registeredLSystemCloudToTest->size();

            logStream << "Of the " << inputCloudToFitLSystemTo->size() << " points in the input cloud, the L-system accounts for " <<
                                cloudOriginalObjectPointsAcccounted->size() << " (" << proportionOfInputCloudAccounted << "), and fails to account for " << cloudOriginalObjectPointsUnacccountedFor->size() << "." << std::endl;
            logStream << "Of the " << registeredLSystemCloudToTest->size() << " points in the L-system, the input cloud accounts for " <<
                                cloudLSystemPointsAcccounted->size() << ", and fails to account for " << cloudLSystemPointsPointsUnacccountedFor->size() <<
                                    " (" << proportionOfLSystemCloudUnaccounted << ")." << std::endl;
            float combinedProportionScore = proportionOfInputCloudAccounted - proportionOfLSystemCloudUnaccounted;
            int combinedPointScore= (int)cloudOriginalObjectPointsAcccounted->size() - (int)cloudLSystemPointsPointsUnacccountedFor->size();
            logStream << "Combined point score of this fit is " << combinedPointScore << ", combined proportion score is " << combinedProportionScore <<
                            " and registration fitness is " << registrationFitness << ".";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            if (combinedProportionScore >= bestCombinedProportionScore) {
                logStream << "Recording new combined proportion score of. " << combinedProportionScore;
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                bestRegistrationFitness = registrationFitness;
                bestPointsAccounted = cloudOriginalObjectPointsAcccounted->size();
                bestCombinedPointScore = cloudOriginalObjectPointsAcccounted->size() - cloudLSystemPointsPointsUnacccountedFor->size();
                bestLeafPhyllotaxyAngles = currentLeafPhyllotaxyAngles;
                bestCombinedProportionScore = combinedProportionScore;
                retainedLeafPhyllotaxyAngles = bestLeafPhyllotaxyAngles;

            }
            currentProportion += 0.1;
        }
        logStream << "The original phyllotaxy set was:\n";
        for (uint32_t j = 0; j < originalLSystemParameters.getLeafPhyllotaxyAngles().size(); j++) {
            logStream << originalLSystemParameters.getLeafPhyllotaxyAngles()[j] << "\t";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

        logStream << "The best fitting phyllotaxy set for phytomer " << currentPhytomerIndex << " was found to be:\n";
        for (uint32_t j = 0; j < bestLeafPhyllotaxyAngles.size(); j++) {
            logStream << bestLeafPhyllotaxyAngles[j] << "\t";
        }
        LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
    }

    lSystemToReturn.setLeafPhyllotaxyAngles(retainedLeafPhyllotaxyAngles);
    return(lSystemToReturn);
}

LSystemParameters refineLSystem_ExpandedStem_LeafCurvaturesControlPointTwo(PyObject *inputLpyFunction,
                            LSystemParameters inputLsystemParams,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudToFitLSystemTo,
                            std::vector<int> phytomerIndicesToRefine_OneBased,
                            pcl::visualization::PCLVisualizer* visu,
                            InputParameters inputParams) {
    std::ostringstream logStream;
    LSystemParameters originalLSystemParameters = inputLsystemParams;
    LSystemParameters lSystemToExplore = inputLsystemParams;
    LSystemParameters lSystemToReturn = inputLsystemParams;
    LOG.DEBUG("Attempting to refine L-system Parameters.");

    /// What should be the strategy for refinement?
    /// A decent first approximation could be to maximize point correspondences between the LSystem and this part of the cloud.
    /// We already have a basic template of the L-System. We could modify it over a range of values, check correspondences for each value, and choose the max.
    pcl::PolygonMesh LSystemMesh = buildLSystem(inputLpyFunction, inputLsystemParams, visu, inputParams);
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystem (new pcl::PointCloud<pcl::PointXYZ>);
    sampleMeshToPointCloud(&LSystemMesh, *sampledLSystem, inputParams);

    pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemPointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix4f initialRotationMatrix = registerLSystemICPAndReturnTranslationMatrix(sampledLSystem, inputCloudToFitLSystemTo, registeredLSystemPointCloud, visu, inputParams);

    std::vector< std::vector < std::pair<float, float> > > retainedLeafCurvatures = originalLSystemParameters.getLeafCurvatures();
    for (uint32_t i = 0; i < phytomerIndicesToRefine_OneBased.size(); i++) {
        int currentPhytomerIndex_OneBased = phytomerIndicesToRefine_OneBased[i];
        int currentPhytomerIndex = currentPhytomerIndex_OneBased - 1;

        // First, find the range of values we want to exhaustively test. How about a 50% variation in either direction in 5 % intervals?
        // Leaf curvatures are going to be a bit different since they are a pair of values for each control point.
        // Since we're only looking at the extended cylinder around the plant, let's take advantage of that and assume very little curvature
        // has occurred, and that only control point 2 matters. Then, we can rotate a line of fixed length around the origin (0 to 90 degrees) and
        // see which fits best.
        // First, we'll need an estimate of how long the control point vector should be. We can get that based off of the stem radius.
        float internodeRadius = inputLsystemParams.getInternodeRadii()[currentPhytomerIndex];
        float lengthOfControlPointVector = (internodeRadius + (inputParams.sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue()) * 1.1) * 5.0;  // magic numbers obtained from cylinder fitting in "lsystemFitting.cpp"

        //sin(theta) / cos (theta) = y / x

        std::vector< std::vector < std::pair<float, float> > > bestLeafCurvatures;
        int bestLeafControlPointsFit = 0;
        int bestPointsAccounted = 0;
        float bestRegistrationFitness = 9999999;
        int bestCombinedPointScore = -9999999;
        float bestCombinedProportionScore = -9999999;
        for (float angleToTest = -70.0; angleToTest <= 70.0; angleToTest+= 10.0) {
            logStream << "Testing the current angle of " << angleToTest << " for leaf curvature of of phytomer " << currentPhytomerIndex_OneBased;
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            float angleRad = angleToTest * (M_PI / 180.0);
            float xCoord = cos(angleRad) * lengthOfControlPointVector;
            float yCoord = sin(angleRad) * lengthOfControlPointVector;

            std::vector< std::vector < std::pair<float, float> > > currentLeafCurvatures = retainedLeafCurvatures;
            std::vector< std::vector < std::pair<float, float> > > originalLeafCurvatures = originalLSystemParameters.getLeafCurvatures();

            // Set the 2nd, 3rd, and 4th control points to be nearly the same.
            (currentLeafCurvatures[currentPhytomerIndex])[1].first = xCoord;
            (currentLeafCurvatures[currentPhytomerIndex])[1].second = yCoord;
            (currentLeafCurvatures[currentPhytomerIndex])[2].first = xCoord * 1.000001;
            (currentLeafCurvatures[currentPhytomerIndex])[2].second = yCoord * 1.000001;
            (currentLeafCurvatures[currentPhytomerIndex])[3].first = xCoord * 1.000002;
            (currentLeafCurvatures[currentPhytomerIndex])[3].second = yCoord * 1.000002;

            logStream << "Testing leaf curvatures for phytomer " << currentPhytomerIndex << " using:\n";
            for (uint32_t j = 0; j < currentLeafCurvatures.size(); j++) {
                for (uint32_t k = 0; k < currentLeafCurvatures[j].size(); k++) {
                    logStream << "(" << currentLeafCurvatures[j][k].first << ", " << currentLeafCurvatures[j][k].second << ")\t";
                }
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
            }



            lSystemToExplore.setLeafCurvatures(currentLeafCurvatures);

            pcl::PolygonMesh LSystemMeshToTest = buildLSystem(inputLpyFunction, lSystemToExplore, visu, inputParams);
            pcl::PointCloud<pcl::PointXYZ>::Ptr sampledLSystemCloudToTest (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformedLSystemCloudToTest  (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr registeredLSystemCloudToTest  (new pcl::PointCloud<pcl::PointXYZ>);
            sampleMeshToPointCloud(&LSystemMeshToTest, *sampledLSystemCloudToTest, inputParams);
            pcl::transformPointCloud(*sampledLSystemCloudToTest, *transformedLSystemCloudToTest, initialRotationMatrix);
            float registrationFitness = registerLSystemICPAndReturnFitness(transformedLSystemCloudToTest, inputCloudToFitLSystemTo, registeredLSystemCloudToTest, visu, inputParams);

            // I think we also need to be a bit more "greedy" than just correspondences. A nearest neighbor approach may work.
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchOriginalObject;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsAcccounted (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOriginalObjectPointsUnacccountedFor (new pcl::PointCloud<pcl::PointXYZ>);
            kdtreeSearchOriginalObject.setInputCloud(registeredLSystemCloudToTest);

            for (uint32_t j = 0; j < inputCloudToFitLSystemTo->points.size(); j++) {
                pcl::PointXYZ currentPoint = inputCloudToFitLSystemTo->points[j];
                std::vector<int> pointIndicesRadiusSearch;
                std::vector<float> distanceRadiusSearch;
                /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
                /// This number determines the radius away from the L-System points that points in the original cloud will be considered accounted for in the L-system.
                int numberNeighbors = kdtreeSearchOriginalObject.radiusSearch(currentPoint, 7.5, pointIndicesRadiusSearch, distanceRadiusSearch);
                if (numberNeighbors >= 1) {
                    cloudOriginalObjectPointsAcccounted->points.push_back(currentPoint);
                }
                else {
                    cloudOriginalObjectPointsUnacccountedFor->points.push_back(currentPoint);
                }
            }

            pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeSearchLSystemCloud;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLSystemPointsAcccounted (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLSystemPointsPointsUnacccountedFor (new pcl::PointCloud<pcl::PointXYZ>);
            kdtreeSearchLSystemCloud.setInputCloud(inputCloudToFitLSystemTo);

            for (uint32_t j = 0; j < registeredLSystemCloudToTest->points.size(); j++) {
                pcl::PointXYZ currentPoint = registeredLSystemCloudToTest->points[j];
                std::vector<int> pointIndicesRadiusSearch;
                std::vector<float> distanceRadiusSearch;
                /// IMPORTANT MAGIC NUMBER HERE, CONSIDER REFACTORING TO INPUT VARIABLE IF THIS METHOD IS USED.
                /// This number determines the search radius
                int numberNeighbors = kdtreeSearchLSystemCloud.radiusSearch(currentPoint, 7.5, pointIndicesRadiusSearch, distanceRadiusSearch);
                if (numberNeighbors >= 1) {
                    cloudLSystemPointsAcccounted->points.push_back(currentPoint);
                }
                else {
                    cloudLSystemPointsPointsUnacccountedFor->points.push_back(currentPoint);
                }
            }

            float proportionOfInputCloudAccounted = (float)cloudOriginalObjectPointsAcccounted->size() / (float)inputCloudToFitLSystemTo->size();
            float proportionOfLSystemCloudUnaccounted = (float)cloudLSystemPointsPointsUnacccountedFor->size() / (float)registeredLSystemCloudToTest->size();

            logStream << "Of the " << inputCloudToFitLSystemTo->size() << " points in the input cloud, the L-system accounts for " <<
                                cloudOriginalObjectPointsAcccounted->size() << " (" << proportionOfInputCloudAccounted << "), and fails to account for " << cloudOriginalObjectPointsUnacccountedFor->size() << "." << std::endl;
            logStream << "Of the " << registeredLSystemCloudToTest->size() << " points in the L-system, the input cloud accounts for " <<
                                cloudLSystemPointsAcccounted->size() << ", and fails to account for " << cloudLSystemPointsPointsUnacccountedFor->size() <<
                                    " (" << proportionOfLSystemCloudUnaccounted << ")." << std::endl;
            float combinedProportionScore = proportionOfInputCloudAccounted - proportionOfLSystemCloudUnaccounted;
            int combinedPointScore= (int)cloudOriginalObjectPointsAcccounted->size() - (int)cloudLSystemPointsPointsUnacccountedFor->size();
            logStream << "Combined point score of this fit is " << combinedPointScore << ", combined proportion score is " << combinedProportionScore <<
                            " and registration fitness is " << registrationFitness << ".";
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

            if (combinedProportionScore >= bestCombinedProportionScore) {
                logStream << "Recording new combined proportion score of. " << combinedProportionScore;
                LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");

                bestRegistrationFitness = registrationFitness;
                bestPointsAccounted = cloudOriginalObjectPointsAcccounted->size();
                bestCombinedPointScore = cloudOriginalObjectPointsAcccounted->size() - cloudLSystemPointsPointsUnacccountedFor->size();
                bestLeafCurvatures = currentLeafCurvatures;
                bestCombinedProportionScore = combinedProportionScore;
                retainedLeafCurvatures = bestLeafCurvatures;

            }

        }
        logStream << "The original curvature set was:\n";
        for (uint32_t j = 0; j < originalLSystemParameters.getLeafCurvatures().size(); j++) {
            for (uint32_t k= 0; k < originalLSystemParameters.getLeafCurvatures()[j].size(); k++) {
                logStream << "(" << originalLSystemParameters.getLeafCurvatures()[j][k].first << ", " <<  originalLSystemParameters.getLeafCurvatures()[j][k].second << ")\t";
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        }


        logStream << "The best fitting curvature set for phytomer " << currentPhytomerIndex << " was found to be:\n";
        for (uint32_t j = 0; j < bestLeafCurvatures.size(); j++) {
            for (uint32_t k= 0; k < bestLeafCurvatures[j].size(); k++) {
                logStream << "(" << bestLeafCurvatures[j][k].first << ", " << bestLeafCurvatures[j][k].second << ")\t";
            }
            LOG.DEBUG(logStream.str(), inputParams.debuggingParameters.getDebuggingLevel(), 2); logStream.str("");
        }

    }

    lSystemToReturn.setLeafCurvatures(retainedLeafCurvatures);
    return(lSystemToReturn);
}

