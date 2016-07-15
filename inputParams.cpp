#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <vector>
#include "rapidxml-1.13/rapidxml.hpp"
#include "inputParams.h"
#include "loggingHelper.h"

// Begin PassThroughFilterParameters class
PassThroughFilterParameters::PassThroughFilterParameters() {
    setXmin(-5000); setXmax(5000);
    setYmin(-5000); setYmax(5000);
    setZmin(-5000); setZmax(5000);
}
PassThroughFilterParameters::~PassThroughFilterParameters(){ //Empty destructor
}

int PassThroughFilterParameters::getXmin() { return _Xmin; }
void PassThroughFilterParameters::setXmin(int inputXmin) { _Xmin = inputXmin; }
int PassThroughFilterParameters::getXmax() { return _Xmax; }
void PassThroughFilterParameters::setXmax(int inputXmax) { _Xmax = inputXmax; }
int PassThroughFilterParameters::getYmin() { return _Ymin; }
void PassThroughFilterParameters::setYmin(int inputYmin) { _Ymin = inputYmin; }
int PassThroughFilterParameters::getYmax() { return _Ymax; }
void PassThroughFilterParameters::setYmax(int inputYmax) { _Ymax = inputYmax; }
int PassThroughFilterParameters::getZmin() { return _Zmin; }
void PassThroughFilterParameters::setZmin(int inputZmin) { _Zmin = inputZmin; }
int PassThroughFilterParameters::getZmax() { return _Zmax; }
void PassThroughFilterParameters::setZmax(int inputZmax) { _Zmax = inputZmax; }


void PassThroughFilterParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tPassThroughFilter Xmin:\t\t" << this->getXmin() << std::endl <<
                    "\t\tPassThroughFilter Xmax:\t\t" << this->getXmax() << std::endl <<
                    "\t\tPassThroughFilter Ymin:\t\t" << this->getYmin() << std::endl <<
                    "\t\tPassThroughFilter Ymax:\t\t" << this->getYmax() << std::endl <<
                    "\t\tPassThroughFilter Zmin:\t\t" << this->getZmin() << std::endl <<
                    "\t\tPassThroughFilter Zmax:\t\t" << this->getZmax();
    LOG.DEBUG(logStream.str());
}

// End PassThroughFilterParameters class



// Begin StatisticalOutlierRemovalParameters class
StatisticalOutlierRemovalParameters::StatisticalOutlierRemovalParameters() {
    setMeanK(30);
    setStdDevMulThresh(2.0);
}
StatisticalOutlierRemovalParameters::~StatisticalOutlierRemovalParameters() { //Empty destructor
}

int StatisticalOutlierRemovalParameters::getMeanK() { return _MeanK; }
void StatisticalOutlierRemovalParameters::setMeanK(int inputMeanK) { _MeanK = inputMeanK; }
float StatisticalOutlierRemovalParameters::getStdDevMulThresh() {return _StdDevMulThresh; }
void StatisticalOutlierRemovalParameters::setStdDevMulThresh(float inputStdDevMulThresh) { _StdDevMulThresh = inputStdDevMulThresh; }

void StatisticalOutlierRemovalParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tStatOutlierRemoval MeanK:\t\t" << this->getMeanK() << std::endl <<
                    "\t\tStatOutlierRemoval StddevMulThresh:\t\t" << this->getStdDevMulThresh();
    LOG.DEBUG(logStream.str());
}

// End StatisticalOutlierRemovalParameters class



// Begin NormalEstimationParameters class
NormalEstimationParameters::NormalEstimationParameters() {
    setRadiusSearch(20.0);
    setKSearch(10);
}
NormalEstimationParameters::~NormalEstimationParameters() {
//Empty destructor
}

float NormalEstimationParameters::getRadiusSearch() { return _RadiusSearch; }
void NormalEstimationParameters::setRadiusSearch(float inputRadiusSearch) { _RadiusSearch = inputRadiusSearch; }
int NormalEstimationParameters::getKSearch() { return _KSearch; }
void NormalEstimationParameters::setKSearch(int inputKSearch) { _KSearch = inputKSearch; }

void NormalEstimationParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tNormalEstimation RadiusSearch:\t\t" << getRadiusSearch() << std::endl <<
                    "\t\tNormalEstimation KSearch:\t\t" << getKSearch();
    LOG.DEBUG(logStream.str());
}
// End NormalEstimationParameters class



// Begin FeatureEstimationParameters class
FeatureEstimationParameters::FeatureEstimationParameters() {
    setRadiusSearch(30.0);
    setKSearch(20);
}
FeatureEstimationParameters::~FeatureEstimationParameters() {
//Empty destructor
}
float FeatureEstimationParameters::getRadiusSearch() { return _RadiusSearch; }
void FeatureEstimationParameters::setRadiusSearch(float inputRadiusSearch) { _RadiusSearch = inputRadiusSearch; }
int FeatureEstimationParameters::getKSearch() { return _KSearch; }
void FeatureEstimationParameters::setKSearch(int inputKSearch) { _KSearch = inputKSearch; }

void FeatureEstimationParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tFeatureEstimation RadiusSearch:\t\t" << this->getRadiusSearch() << std::endl <<
                    "\t\tFeatureEstimation KSearch:\t\t" << this->getKSearch();
    LOG.DEBUG(logStream.str());
}
// End FeatureEstimationParameters class



// Begin SampleConsensusPrerejectiveParameters class
SampleConsensusPrerejectiveParameters::SampleConsensusPrerejectiveParameters() {
    setMaximumIterations(20000);
    setNumberOfSamples(3);
    setCorrespondenceRandomness(2);
    setSimilarityThreshold(0.50);
    setMaxCorrespondenceDistance(10.0);
    setInlierFraction(0.20);
}
SampleConsensusPrerejectiveParameters::~SampleConsensusPrerejectiveParameters(){ //Empty destructor
}

int SampleConsensusPrerejectiveParameters::getMaximumIterations() { return _MaximumIterations; }
void SampleConsensusPrerejectiveParameters::setMaximumIterations(int inputMaximumIterations) { _MaximumIterations = inputMaximumIterations; }
int SampleConsensusPrerejectiveParameters::getNumberOfSamples() { return _NumberOfSamples; }
void SampleConsensusPrerejectiveParameters::setNumberOfSamples(int inputNumberOfSamples) { _NumberOfSamples = inputNumberOfSamples; }
int SampleConsensusPrerejectiveParameters::getCorrespondenceRandomness() { return _CorrespondenceRandomness; }
void SampleConsensusPrerejectiveParameters::setCorrespondenceRandomness(int inputCorrespondenceRandomness) { _CorrespondenceRandomness = inputCorrespondenceRandomness; }
float SampleConsensusPrerejectiveParameters::getSimilarityThreshold() { return _SimilarityThreshold; }
void SampleConsensusPrerejectiveParameters::setSimilarityThreshold(float inputSimilarityThreshold) { _SimilarityThreshold = inputSimilarityThreshold; }
float SampleConsensusPrerejectiveParameters::getMaxCorrespondenceDistance() { return _MaxCorrespondenceDistance; }
void SampleConsensusPrerejectiveParameters::setMaxCorrespondenceDistance(float inputMaxCorrespondenceDistance) { _MaxCorrespondenceDistance = inputMaxCorrespondenceDistance; }
float SampleConsensusPrerejectiveParameters::getInlierFraction() { return _InlierFraction; }
void SampleConsensusPrerejectiveParameters::setInlierFraction(float inputInlierFraction) { _InlierFraction = inputInlierFraction; }

void SampleConsensusPrerejectiveParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tSACPrerejective MaximumIterations:\t\t" << this->getMaximumIterations() << std::endl <<
                    "\t\tSACPrerejective NumberOfSamples:\t\t" << this->getNumberOfSamples() << std::endl <<
                    "\t\tSACPrerejective CorrespondenceRandomness:\t\t" << this->getCorrespondenceRandomness() << std::endl <<
                    "\t\tSACPrerejective SimilarityThreshold:\t\t" << this->getSimilarityThreshold() << std::endl <<
                    "\t\tSACPrerejective MaxCorrespondenceDistance:\t\t" << this->getMaxCorrespondenceDistance() << std::endl <<
                    "\t\tSACPrerejective InlierFraction:\t\t" << this->getInlierFraction();
    LOG.DEBUG(logStream.str());
}
// End SampleConsensusPrerejectiveParameters class


// Begin VoxelGridFilterParameters
VoxelGridFilterParameters::VoxelGridFilterParameters() {
    setLeafSize(0.10);
}
VoxelGridFilterParameters::~VoxelGridFilterParameters() {
//Empty destructor
}

float VoxelGridFilterParameters::getLeafSize() { return _LeafSize; }
void VoxelGridFilterParameters::setLeafSize(float inputLeafSize) { _LeafSize = inputLeafSize; }

void VoxelGridFilterParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tVoxelGridFilter LeafSize:\t\t" << this->getLeafSize();
    LOG.DEBUG(logStream.str());
}
// End VoxelGridFilterParameters


// Begin IterativeClosestPointParameters
IterativeClosestPointParameters::IterativeClosestPointParameters() {
    setMaximumIterations(1000);
    setMaxCorrespondenceDistance(100.0);
    setMinCorrespondenceDistance(5.0);
    setLargeCorrespondenceDistanceStepReduction(20.0);
    setSmallCorrespondenceDistanceStepReduction(2.5);
    setThresholdSwitchFromLargeToSmallDistanceSteps(100.0);
    setFitnessThresholdDefiningFailure(1000.0);

}
IterativeClosestPointParameters::~IterativeClosestPointParameters() {
//Empty destructor
}

int IterativeClosestPointParameters::getMaximumIterations() { return _MaximumIterations; }
void IterativeClosestPointParameters::setMaximumIterations(int inputMaximumIterations) { _MaximumIterations = inputMaximumIterations; }
float IterativeClosestPointParameters::getMaxCorrespondenceDistance() { return _MaxCorrespondenceDistance; }
void IterativeClosestPointParameters::setMaxCorrespondenceDistance(float inputMaxCorrespondenceDistance) { _MaxCorrespondenceDistance = inputMaxCorrespondenceDistance; }
float IterativeClosestPointParameters::getMinCorrespondenceDistance() { return _MinCorrespondenceDistance; }
void IterativeClosestPointParameters::setMinCorrespondenceDistance(float inputMinCorrespondenceDistance) { _MinCorrespondenceDistance = inputMinCorrespondenceDistance; }
float IterativeClosestPointParameters::getLargeCorrespondenceDistanceStepReduction() { return _LargeCorrespondenceDistanceStepReduction; }
void IterativeClosestPointParameters::setLargeCorrespondenceDistanceStepReduction(float inputLargeCorrespondenceDistanceStepReduction) { _LargeCorrespondenceDistanceStepReduction = inputLargeCorrespondenceDistanceStepReduction; }
float IterativeClosestPointParameters::getSmallCorrespondenceDistanceStepReduction() { return _SmallCorrespondenceDistanceStepReduction; }
void IterativeClosestPointParameters::setSmallCorrespondenceDistanceStepReduction(float inputSmallCorrespondenceDistanceStepReduction) { _SmallCorrespondenceDistanceStepReduction = inputSmallCorrespondenceDistanceStepReduction; }
float IterativeClosestPointParameters::getThresholdSwitchFromLargeToSmallDistanceSteps() { return _ThresholdSwitchFromLargeToSmallDistanceSteps; }
void IterativeClosestPointParameters::setThresholdSwitchFromLargeToSmallDistanceSteps(float inputThresholdSwitchFromLargeToSmallDistanceSteps) { _ThresholdSwitchFromLargeToSmallDistanceSteps = inputThresholdSwitchFromLargeToSmallDistanceSteps; }
float IterativeClosestPointParameters::getFitnessThresholdDefiningFailure() { return _FitnessThresholdDefiningFailure; }
void IterativeClosestPointParameters::setFitnessThresholdDefiningFailure(float inputFitnessThresholdDefiningFailure) { _FitnessThresholdDefiningFailure = inputFitnessThresholdDefiningFailure; }

void IterativeClosestPointParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tIterativeClosestPoint MaximumIterations:\t\t" << this->getMaximumIterations() << std::endl <<
                    "\t\tIterativeClosestPoint MaxCorrespondenceDistance:\t\t" << this->getMaxCorrespondenceDistance() << std::endl <<
                    "\t\tIterativeClosestPoint MinCorrespondenceDistance:\t\t" << this->getMinCorrespondenceDistance() << std::endl <<
                    "\t\tIterativeClosestPoint LargeCorrespondenceDistanceStepReduction:\t\t" << this->getLargeCorrespondenceDistanceStepReduction() << std::endl <<
                    "\t\tIterativeClosestPoint SmallCorrespondenceDistanceStepReduction:\t\t" << this->getSmallCorrespondenceDistanceStepReduction() << std::endl <<
                    "\t\tIterativeClosestPoint ThresholdSwitchFromLargeToSmallDistanceSteps:\t\t" << this->getThresholdSwitchFromLargeToSmallDistanceSteps() << std::endl <<
                    "\t\tIterativeClosestPoint FitnessThresholdDefiningFailure:\t\t" << this->getFitnessThresholdDefiningFailure();
    LOG.DEBUG(logStream.str());

}
// End IterativeClosestPointParameters

// Begin RandomSampleParameters
RandomSampleParameters::RandomSampleParameters() {
    setSampleProportion(0.75);
}
RandomSampleParameters::~RandomSampleParameters() {
//Empty destructor
}

float RandomSampleParameters::getSampleProportion() { return _SampleProportion; }
void RandomSampleParameters::setSampleProportion(float inputSampleProportion) { _SampleProportion = inputSampleProportion; }

void RandomSampleParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tRandomSample SampleProportion:\t\t" << this->getSampleProportion();
    LOG.DEBUG(logStream.str());
}
// End RandomSampleParameters



// Begin MovingLeastSquaresParameters
MovingLeastSquaresParameters::MovingLeastSquaresParameters() {
    setSearchRadius(10.0);
    setUpsampleFlag(0);
    setPolynomialOrder(2);
    setUpsamplingRadius(1.0);
    setUpsamplingStepSize(0.5);
}
MovingLeastSquaresParameters::~MovingLeastSquaresParameters() {
//Empty destructor
}

float MovingLeastSquaresParameters::getSearchRadius() { return _SearchRadius; }
void MovingLeastSquaresParameters::setSearchRadius(float inputSearchRadius) { _SearchRadius = inputSearchRadius; }
int MovingLeastSquaresParameters::getUpsampleFlag() { return _UpsampleFlag; }
void MovingLeastSquaresParameters::setUpsampleFlag(int inputUpsampleFlag) { _UpsampleFlag = inputUpsampleFlag; }
int MovingLeastSquaresParameters::getPolynomialOrder() { return _PolynomialOrder; }
void MovingLeastSquaresParameters::setPolynomialOrder(int inputPolynomialOrder) { _PolynomialOrder = inputPolynomialOrder; }
float MovingLeastSquaresParameters::getUpsamplingRadius() { return _UpsamplingRadius; }
void MovingLeastSquaresParameters::setUpsamplingRadius(float inputUpsamplingRadius) { _UpsamplingRadius = inputUpsamplingRadius; }
float MovingLeastSquaresParameters::getUpsamplingStepSize() { return _UpsamplingStepSize; }
void MovingLeastSquaresParameters::setUpsamplingStepSize(float inputUpsamplingStepSize) { _UpsamplingStepSize = inputUpsamplingStepSize; }

void MovingLeastSquaresParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tMovingLeastSquares SearchRadius:\t\t" << this->getSearchRadius() << std::endl <<
                    "\t\tMovingLeastSquares UpsampleFlag:\t\t" << this->getUpsampleFlag() << std::endl <<
                    "\t\tMovingLeastSquares PolynomialOrder:\t\t" << this->getPolynomialOrder() << std::endl <<
                    "\t\tMovingLeastSquares UpsamplingRadius:\t\t" << this->getUpsamplingRadius() << std::endl <<
                    "\t\tMovingLeastSquares UpsamplingStepSize:\t\t" << this->getUpsamplingStepSize();
    LOG.DEBUG(logStream.str());
}
// End MovingLeastSquaresParameters


// Begin RegionGrowingParameters
RegionGrowingParameters::RegionGrowingParameters() {
    setMinClusterSize(1000);
    setMaxClusterSize(10000000);
    setNumberOfNeighbours(400);
    setSmoothnessThreshold(3.0);
    setCurvatureThreshold(3.0);
}
RegionGrowingParameters::~RegionGrowingParameters() {
//Empty destructor
}

int RegionGrowingParameters::getMinClusterSize() { return _MinClusterSize; }
void RegionGrowingParameters::setMinClusterSize(int inputMinClusterSize) { _MinClusterSize = inputMinClusterSize; }
int RegionGrowingParameters::getMaxClusterSize() { return _MaxClusterSize; }
void RegionGrowingParameters::setMaxClusterSize(int inputMaxClusterSize) { _MaxClusterSize = inputMaxClusterSize; }
int RegionGrowingParameters::getNumberOfNeighbours() { return _NumberOfNeighbours; }
void RegionGrowingParameters::setNumberOfNeighbours(int inputNumberOfNeighbours) { _NumberOfNeighbours = inputNumberOfNeighbours; }
float RegionGrowingParameters::getSmoothnessThreshold() { return _SmoothnessThreshold; }
void RegionGrowingParameters::setSmoothnessThreshold(float inputSmoothnessThreshold) { _SmoothnessThreshold = inputSmoothnessThreshold; }
float RegionGrowingParameters::getCurvatureThreshold() { return _CurvatureThreshold; }
void RegionGrowingParameters::setCurvatureThreshold(float inputCurvatureThreshold) { _CurvatureThreshold = inputCurvatureThreshold; }

void RegionGrowingParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tRegionGrowing MinClusterSize:\t\t" << this->getMinClusterSize() << std::endl <<
                    "\t\tRegionGrowing MaxClusterSize:\t\t" << this->getMaxClusterSize() << std::endl <<
                    "\t\tRegionGrowing NumberOfNeighbours:\t\t" << this->getNumberOfNeighbours() << std::endl <<
                    "\t\tRegionGrowing SmoothnessThreshold:\t\t" << this->getSmoothnessThreshold() << std::endl <<
                    "\t\tRegionGrowing CurvatureThreshold:\t\t" << this->getCurvatureThreshold();
    LOG.DEBUG(logStream.str());
}
// End RegionGrowingParameters



// Begin SACSegmentationFromNormalsParameters
SACSegmentationFromNormalsParameters::SACSegmentationFromNormalsParameters() {
    setMaxIterations(20000);
    setDistanceThreshold(100.0);
    setNormalDistanceWeight(0.2);
    setRadiusLimitsMin(0.01);
    setRadiusLimitsMax(50.0);
    setSelectWithinDistanceValue(15.0);
}
SACSegmentationFromNormalsParameters::~SACSegmentationFromNormalsParameters() {
//Empty destructor
}

int SACSegmentationFromNormalsParameters::getMaxIterations() { return _MaxIterations; }
void SACSegmentationFromNormalsParameters::setMaxIterations(int inputMaxIterations) { _MaxIterations = inputMaxIterations; }
float SACSegmentationFromNormalsParameters::getDistanceThreshold() { return _DistanceThreshold; }
void SACSegmentationFromNormalsParameters::setDistanceThreshold(float inputDistanceThreshold) { _DistanceThreshold = inputDistanceThreshold; }
float SACSegmentationFromNormalsParameters::getNormalDistanceWeight() { return _NormalDistanceWeight; }
void SACSegmentationFromNormalsParameters::setNormalDistanceWeight(float inputNormalDistanceWeight) { _NormalDistanceWeight = inputNormalDistanceWeight; }
float SACSegmentationFromNormalsParameters::getRadiusLimitsMin() { return _RadiusLimitsMin; }
void SACSegmentationFromNormalsParameters::setRadiusLimitsMin(float inputRadiusLimitsMin) { _RadiusLimitsMin = inputRadiusLimitsMin; }
float SACSegmentationFromNormalsParameters::getRadiusLimitsMax() { return _RadiusLimitsMax; }
void SACSegmentationFromNormalsParameters::setRadiusLimitsMax(float inputRadiusLimitsMax) { _RadiusLimitsMax = inputRadiusLimitsMax; }
float SACSegmentationFromNormalsParameters::getSelectWithinDistanceValue() { return _SelectWithinDistanceValue; }
void SACSegmentationFromNormalsParameters::setSelectWithinDistanceValue(float inputSelectWithinDistanceValue) { _SelectWithinDistanceValue = inputSelectWithinDistanceValue; }

void SACSegmentationFromNormalsParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tSACSegmentationFromNormals MaxIterations:\t\t" << this->getMaxIterations() << std::endl <<
                    "\t\tSACSegmentationFromNormals DistanceThreshold:\t\t" << this->getDistanceThreshold() << std::endl <<
                    "\t\tSACSegmentationFromNormals NormalDistanceWeight:\t\t" << this->getNormalDistanceWeight() << std::endl <<
                    "\t\tSACSegmentationFromNormals RadiusLimitsMin:\t\t" << this->getRadiusLimitsMin() << std::endl <<
                    "\t\tSACSegmentationFromNormals RadiusLimitsMax:\t\t" << this->getRadiusLimitsMax() << std::endl <<
                    "\t\tSACSegmentationFromNormals SelectWithinDistanceValue:\t\t" << this->getSelectWithinDistanceValue();
    LOG.DEBUG(logStream.str());

}
// End SACSegmentationFromNormalsParameters



// Begin CircleRANSACParameters
CircleRANSACParameters::CircleRANSACParameters() {
    setMaxIterations(20000);
    setDistanceThreshold(100.0);
    setSelectWithinDistanceValue(15.0);
}
CircleRANSACParameters::~CircleRANSACParameters() {
//Empty destructor
}

int CircleRANSACParameters::getMaxIterations() { return _MaxIterations; }
void CircleRANSACParameters::setMaxIterations(int inputMaxIterations) { _MaxIterations = inputMaxIterations; }
float CircleRANSACParameters::getDistanceThreshold() { return _DistanceThreshold; }
void CircleRANSACParameters::setDistanceThreshold(float inputDistanceThreshold) { _DistanceThreshold = inputDistanceThreshold; }
float CircleRANSACParameters::getSelectWithinDistanceValue() { return _SelectWithinDistanceValue; }
void CircleRANSACParameters::setSelectWithinDistanceValue(float inputSelectWithinDistanceValue) { _SelectWithinDistanceValue = inputSelectWithinDistanceValue; }

void CircleRANSACParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tCircleRANSACParameters MaxIterations:\t\t" << this->getMaxIterations() << std::endl <<
                    "\t\tCircleRANSACParameters DistanceThreshold:\t\t" << this->getDistanceThreshold() << std::endl <<
                    "\t\tCircleRANSACParameters SelectWithinDistanceValue:\t\t" << this->getSelectWithinDistanceValue();
    LOG.DEBUG(logStream.str());

}
// End CircleRANSACParameters


// Begin PlaneRANSACParameters
PlaneRANSACParameters::PlaneRANSACParameters() {
    setMaxIterations(20000);
    setDistanceThreshold(100.0);
    setSelectWithinDistanceValue(15.0);
}
PlaneRANSACParameters::~PlaneRANSACParameters() {
//Empty destructor
}

int PlaneRANSACParameters::getMaxIterations() { return _MaxIterations; }
void PlaneRANSACParameters::setMaxIterations(int inputMaxIterations) { _MaxIterations = inputMaxIterations; }
float PlaneRANSACParameters::getDistanceThreshold() { return _DistanceThreshold; }
void PlaneRANSACParameters::setDistanceThreshold(float inputDistanceThreshold) { _DistanceThreshold = inputDistanceThreshold; }
float PlaneRANSACParameters::getSelectWithinDistanceValue() { return _SelectWithinDistanceValue; }
void PlaneRANSACParameters::setSelectWithinDistanceValue(float inputSelectWithinDistanceValue) { _SelectWithinDistanceValue = inputSelectWithinDistanceValue; }

void PlaneRANSACParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\tPlaneRANSACParameters MaxIterations:\t\t" << this->getMaxIterations() << std::endl <<
                    "\tPlaneRANSACParameters DistanceThreshold:\t\t" << this->getDistanceThreshold() << std::endl <<
                    "\tPlaneRANSACParameters SelectWithinDistanceValue:\t\t" << this->getSelectWithinDistanceValue();
    LOG.DEBUG(logStream.str());

}
// End CircleRANSACParameters



// Begin SupervoxelClusteringParameters
SupervoxelClusteringParameters::SupervoxelClusteringParameters() {
    setVoxelResolution(0.02);
    setSeedResolution(0.001);
    setColorImportance(0.0);
    setSpatialImportance(0.40);
    setNormalImportance(1.0);
}
SupervoxelClusteringParameters::~SupervoxelClusteringParameters() {
//Empty destructor
}

float SupervoxelClusteringParameters::getVoxelResolution() { return _VoxelResolution; }
void SupervoxelClusteringParameters::setVoxelResolution(float inputVoxelResolution) { _VoxelResolution = inputVoxelResolution; }
float SupervoxelClusteringParameters::getSeedResolution() { return _SeedResolution; }
void SupervoxelClusteringParameters::setSeedResolution(float inputSeedResolution) { _SeedResolution = inputSeedResolution; }
float SupervoxelClusteringParameters::getColorImportance() { return _ColorImportance; }
void SupervoxelClusteringParameters::setColorImportance(float inputColorImportance) { _ColorImportance = inputColorImportance; }
float SupervoxelClusteringParameters::getSpatialImportance() { return _SpatialImportance; }
void SupervoxelClusteringParameters::setSpatialImportance(float inputSpatialImportance) { _SpatialImportance = inputSpatialImportance; }
float SupervoxelClusteringParameters::getNormalImportance() { return _NormalImportance; }
void SupervoxelClusteringParameters::setNormalImportance(float inputNormalImportance) { _NormalImportance = inputNormalImportance; }

void SupervoxelClusteringParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tSupervoxelClustering VoxelResolution:\t\t" << this->getVoxelResolution() << std::endl <<
                        "\t\tSupervoxelClustering SeedResolution:\t\t" << this->getSeedResolution() << std::endl <<
                        "\t\tSupervoxelClustering ColorImportance:\t\t" << this->getColorImportance() << std::endl <<
                        "\t\tSupervoxelClustering SpatialImportance:\t\t" << this->getSpatialImportance() << std::endl <<
                        "\t\tSupervoxelClustering NormalImportance:\t\t" << this->getNormalImportance();
    LOG.DEBUG(logStream.str());

}
// End SupervoxelClusteringParameters

// Begin DebuggingParameters
DebuggingParameters::DebuggingParameters() {
    setDebuggingLevel(1);
    setLoggingPath("./plantMeshPhenotyper.log");
}
DebuggingParameters::~DebuggingParameters() {
    // Empty destructor
}

float DebuggingParameters::getDebuggingLevel() {return _DebuggingLevel; }
void DebuggingParameters::setDebuggingLevel(int inputDebuggingLevel) { _DebuggingLevel = inputDebuggingLevel; }

std::string DebuggingParameters::getLoggingPath() {return _loggingPath; }
void DebuggingParameters::setLoggingPath(std::string inputLoggingPath) { _loggingPath = inputLoggingPath; }

void DebuggingParameters::printParameters() {
    std::ostringstream logStream;
    logStream << "\t\tDebugging level:\t\t" << this->getDebuggingLevel() << std::endl <<
                    "\t\tLogging path:\t\t" << this->getLoggingPath();
    LOG.DEBUG(logStream.str());
}

// End DebuggingParameters

// Begin CameraParameters
CameraParameters::CameraParameters() {
    setxCoordLocation(0);
    setyCoordLocation(0);
    setzCoordLocation(0);

    setxViewComponent(0);
    setyViewComponent(0);
    setzViewComponent(0);

    setxUpComponent(0);
    setyUpComponent(0);
    setzUpComponent(0);
}

CameraParameters::~CameraParameters() {
    //Empty destructor
}

double CameraParameters::getxCoordLocation() { return _xCoordLocation; }
void CameraParameters::setxCoordLocation(double inputxCoordLocation) { _xCoordLocation = inputxCoordLocation; }

double CameraParameters::getyCoordLocation() { return _yCoordLocation; }
void CameraParameters::setyCoordLocation(double inputyCoordLocation) { _yCoordLocation = inputyCoordLocation; }

double CameraParameters::getzCoordLocation() { return _zCoordLocation; }
void CameraParameters::setzCoordLocation(double inputzCoordLocation) { _zCoordLocation = inputzCoordLocation; }

double CameraParameters::getxViewComponent() { return _xViewComponent; }
void CameraParameters::setxViewComponent(double inputxViewComponent) { _xViewComponent = inputxViewComponent; }

double CameraParameters::getyViewComponent() { return _yViewComponent; }
void CameraParameters::setyViewComponent(double inputyViewComponent) { _yViewComponent = inputyViewComponent; }

double CameraParameters::getzViewComponent() { return _zViewComponent; }
void CameraParameters::setzViewComponent(double inputzViewComponent) { _zViewComponent = inputzViewComponent; }

double CameraParameters::getxUpComponent() { return _xUpComponent; }
void CameraParameters::setxUpComponent(double inputxUpComponent) { _xUpComponent = inputxUpComponent; }

double CameraParameters::getyUpComponent() { return _yUpComponent; }
void CameraParameters::setyUpComponent(double inputyUpComponent) { _yUpComponent = inputyUpComponent; }

double CameraParameters::getzUpComponent() { return _zUpComponent; }
void CameraParameters::setzUpComponent(double inputzUpComponent) { _zUpComponent = inputzUpComponent; }

void CameraParameters::printParameters() {
    std::cout << "\t\tCameraParameters xCoordinateLocation:\t\t" << this->getxCoordLocation() << std::endl;
    std::cout << "\t\tCameraParameters yCoordinateLocation:\t\t" << this->getyCoordLocation() << std::endl;
    std::cout << "\t\tCameraParameters zCoordinateLocation:\t\t" << this->getzCoordLocation() << std::endl;
    std::cout << "\t\tCameraParameters xViewComponent:\t\t" << this->getxViewComponent() << std::endl;
    std::cout << "\t\tCameraParameters yViewComponent:\t\t" << this->getyViewComponent() << std::endl;
    std::cout << "\t\tCameraParameters zViewComponent:\t\t" << this->getyViewComponent() << std::endl;
    std::cout << "\t\tCameraParameters xUpComponent:\t\t" << this->getxUpComponent() << std::endl;
    std::cout << "\t\tCameraParameters yUpComponent:\t\t" << this->getyUpComponent() << std::endl;
    std::cout << "\t\tCameraParameters zUpComponent:\t\t" << this->getzUpComponent() << std::endl;
}

//End CameraParameters


InputParameters::InputParameters() { //Empty constructor
}
InputParameters::~InputParameters() { //Empty destructor
}


/**This thing is getting a little out of hand. Maybe we should only print parameters relevant to the chosen option at runtime?
  */
void InputParameters::printInputParameters() {
        std::cout << "\tPassThroughFilter Xmin:\t\t" << this->passThroughFilterParameters.getXmin() << std::endl;
        std::cout << "\tPassThroughFilter Xmax:\t\t" << this->passThroughFilterParameters.getXmax() << std::endl;
        std::cout << "\tPassThroughFilter Ymin:\t\t" << this->passThroughFilterParameters.getYmin() << std::endl;
        std::cout << "\tPassThroughFilter Ymax:\t\t" << this->passThroughFilterParameters.getYmax() << std::endl;
        std::cout << "\tPassThroughFilter Zmin:\t\t" << this->passThroughFilterParameters.getZmin() << std::endl;
        std::cout << "\tPassThroughFilter Zmax:\t\t" << this->passThroughFilterParameters.getZmax() << std::endl;
        std::cout << "\tStatOutlierRemoval MeanK:\t\t" << this->statisticalOutlierRemovalParameters.getMeanK() << std::endl;
        std::cout << "\tStatOutlierRemoval StddevMulThresh:\t\t" << this->statisticalOutlierRemovalParameters.getStdDevMulThresh() << std::endl;
        std::cout << "\tNormalEstimation RadiusSearch:\t\t" << this->normalEstimationParameters.getRadiusSearch() << std::endl;
        std::cout << "\tNormalEstimation KSearch:\t\t" << this->normalEstimationParameters.getKSearch() << std::endl;
        std::cout << "\tFeatureEstimation RadiusSearch:\t\t" << this->featureEstimationParameters.getRadiusSearch() << std::endl;
        std::cout << "\tFeatureEstimation KSearch:\t\t" << this->featureEstimationParameters.getKSearch() << std::endl;
        std::cout << "\tSACPrerejective MaximumIterations:\t\t" << this->sampleConsensusPrerejectiveParameters.getMaximumIterations() << std::endl;
        std::cout << "\tSACPrerejective NumberOfSamples:\t\t" << this->sampleConsensusPrerejectiveParameters.getNumberOfSamples() << std::endl;
        std::cout << "\tSACPrerejective CorrespondenceRandomness:\t\t" << this->sampleConsensusPrerejectiveParameters.getCorrespondenceRandomness() << std::endl;
        std::cout << "\tSACPrerejective SimilarityThreshold:\t\t" << this->sampleConsensusPrerejectiveParameters.getSimilarityThreshold() << std::endl;
        std::cout << "\tSACPrerejective MaxCorrespondenceDistance:\t\t" << this->sampleConsensusPrerejectiveParameters.getMaxCorrespondenceDistance() << std::endl;
        std::cout << "\tSACPrerejective InlierFraction:\t\t" << this->sampleConsensusPrerejectiveParameters.getInlierFraction() << std::endl;
        std::cout << "\tVoxelGridFilter LeafSize:\t\t" << this->voxelGridFilterParameters.getLeafSize() << std::endl;
        std::cout << "\tIterativeClosestPoint MaximumIterations:\t\t" << this->iterativeClosestPointParameters.getMaximumIterations() << std::endl;
        std::cout << "\tIterativeClosestPoint MaxCorrespondenceDistance:\t\t" << this->iterativeClosestPointParameters.getMaxCorrespondenceDistance() << std::endl;
        std::cout << "\tIterativeClosestPoint MinCorrespondenceDistance:\t\t" << this->iterativeClosestPointParameters.getMinCorrespondenceDistance() << std::endl;
        std::cout << "\tIterativeClosestPoint LargeCorrespondenceDistanceStepReduction:\t\t" << this->iterativeClosestPointParameters.getLargeCorrespondenceDistanceStepReduction() << std::endl;
        std::cout << "\tIterativeClosestPoint SmallCorrespondenceDistanceStepReduction:\t\t" << this->iterativeClosestPointParameters.getSmallCorrespondenceDistanceStepReduction() << std::endl;
        std::cout << "\tIterativeClosestPoint ThresholdSwitchFromLargeToSmallDistanceSteps:\t\t" << this->iterativeClosestPointParameters.getThresholdSwitchFromLargeToSmallDistanceSteps() << std::endl;
        std::cout << "\t\tIterativeClosestPoint FitnessThresholdDefiningFailure:\t\t" << this->iterativeClosestPointParameters.getFitnessThresholdDefiningFailure() << std::endl;
        std::cout << "\tRandomSample SampleProportion:\t\t" << this->randomSampleParameters.getSampleProportion() << std::endl;
        std::cout << "\tMovingLeastSquares SearchRadius:\t\t" << this->movingLeastSquaresParameters.getSearchRadius() << std::endl;
        std::cout << "\tMovingLeastSquares UpsampleFlag:\t\t" << this->movingLeastSquaresParameters.getUpsampleFlag() << std::endl;
        std::cout << "\tMovingLeastSquares PolynomialOrder:\t\t" << this->movingLeastSquaresParameters.getPolynomialOrder() << std::endl;
        std::cout << "\tMovingLeastSquares UpsamplingRadius:\t\t" << this->movingLeastSquaresParameters.getUpsamplingRadius() << std::endl;
        std::cout << "\tMovingLeastSquares UpsamplingStepSize:\t\t" << this->movingLeastSquaresParameters.getUpsamplingStepSize() << std::endl;
        std::cout << "\tRegionGrowing MinClusterSize:\t\t" << this->regionGrowingParameters.getMinClusterSize() << std::endl;
        std::cout << "\tRegionGrowing MaxClusterSize:\t\t" << this->regionGrowingParameters.getMaxClusterSize() << std::endl;
        std::cout << "\tRegionGrowing NumberOfNeighbours:\t\t" << this->regionGrowingParameters.getNumberOfNeighbours() << std::endl;
        std::cout << "\tRegionGrowing SmoothnessThreshold:\t\t" << this->regionGrowingParameters.getSmoothnessThreshold() << std::endl;
        std::cout << "\tRegionGrowing CurvatureThreshold:\t\t" << this->regionGrowingParameters.getCurvatureThreshold() << std::endl;
        std::cout << "\tSACSegmentationFromNormals MaxIterations:\t\t" << this->sacSegmentationFromNormalsParameters.getMaxIterations() << std::endl;
        std::cout << "\tSACSegmentationFromNormals DistanceThreshold:\t\t" << this->sacSegmentationFromNormalsParameters.getDistanceThreshold() << std::endl;
        std::cout << "\tSACSegmentationFromNormals NormalDistanceWeight:\t\t" << this->sacSegmentationFromNormalsParameters.getNormalDistanceWeight() << std::endl;
        std::cout << "\tSACSegmentationFromNormals RadiusLimitsMin:\t\t" << this->sacSegmentationFromNormalsParameters.getRadiusLimitsMin() << std::endl;
        std::cout << "\tSACSegmentationFromNormals RadiusLimitsMax:\t\t" << this->sacSegmentationFromNormalsParameters.getRadiusLimitsMax() << std::endl;
        std::cout << "\tSACSegmentationFromNormals SelectWithinDistanceValue:\t\t" << this->sacSegmentationFromNormalsParameters.getSelectWithinDistanceValue() << std::endl;
        std::cout << "\tCircleRANSACParameters MaxIterations:\t\t" << this->circleRANSACParameters.getMaxIterations() << std::endl;
        std::cout << "\tCircleRANSACParameters DistanceThreshold:\t\t" << this->circleRANSACParameters.getDistanceThreshold() << std::endl;
        std::cout << "\tCircleRANSACParameters SelectWithinDistanceValue:\t\t" << this->circleRANSACParameters.getSelectWithinDistanceValue() << std::endl;
        std::cout << "\tPlaneRANSACParameters MaxIterations:\t\t" << this->planeRANSACParameters.getMaxIterations() << std::endl;
        std::cout << "\tPlaneRANSACParameters DistanceThreshold:\t\t" << this->planeRANSACParameters.getDistanceThreshold() << std::endl;
        std::cout << "\tPlaneRANSACParameters SelectWithinDistanceValue:\t\t" << this->planeRANSACParameters.getSelectWithinDistanceValue() << std::endl;
        std::cout << "\tSupervoxelClustering VoxelResolution:\t\t" << this->supervoxelClusteringParameters.getVoxelResolution() << std::endl;
        std::cout << "\tSupervoxelClustering SeedResolution:\t\t" << this->supervoxelClusteringParameters.getSeedResolution() << std::endl;
        std::cout << "\tSupervoxelClustering ColorImportance:\t\t" << this->supervoxelClusteringParameters.getColorImportance() << std::endl;
        std::cout << "\tSupervoxelClustering SpatialImportance:\t\t" << this->supervoxelClusteringParameters.getSpatialImportance() << std::endl;
        std::cout << "\tSupervoxelClustering NormalImportance:\t\t" << this->supervoxelClusteringParameters.getNormalImportance() << std::endl;
        std::cout << "\t\tCameraParameters xCoordinateLocation:\t\t" << this->cameraParameters.getxCoordLocation() << std::endl;
        std::cout << "\t\tCameraParameters yCoordinateLocation:\t\t" << this->cameraParameters.getyCoordLocation() << std::endl;
        std::cout << "\t\tCameraParameters zCoordinateLocation:\t\t" << this->cameraParameters.getzCoordLocation() << std::endl;
        std::cout << "\t\tCameraParameters xViewComponent:\t\t" << this->cameraParameters.getxViewComponent() << std::endl;
        std::cout << "\t\tCameraParameters yViewComponent:\t\t" << this->cameraParameters.getyViewComponent() << std::endl;
        std::cout << "\t\tCameraParameters zViewComponent:\t\t" << this->cameraParameters.getyViewComponent() << std::endl;
        std::cout << "\t\tCameraParameters xUpComponent:\t\t" << this->cameraParameters.getxUpComponent() << std::endl;
        std::cout << "\t\tCameraParameters yUpComponent:\t\t" << this->cameraParameters.getyUpComponent() << std::endl;
        std::cout << "\t\tCameraParameters zUpComponent:\t\t" << this->cameraParameters.getzUpComponent() << std::endl;
        std::cout << "\t\tDebugging level:\t\t" << this->debuggingParameters.getDebuggingLevel() << std::endl;
        std::cout << "\t\tLogging path:\t\t" << this->debuggingParameters.getLoggingPath() << std::endl;
}

///////////////////

XMLParser::XMLParser() { //Empty constructor
}
XMLParser::~XMLParser() { //Empty constructor
}

int XMLParser::parseXMLtoInputParameters(InputParameters &inputParameters, std::string inputFileName) {
    std::cerr << "Parsing XML..." << std::endl;
    /// Example adapted from https://gist.github.com/JSchaenzle/2726944
	rapidxml::xml_document<> doc;
	rapidxml::xml_node<> * root_node;

	std::ifstream xmlFile(inputFileName);
	assert(xmlFile.is_open());

	std::vector<char> buffer((std::istreambuf_iterator<char>(xmlFile)), std::istreambuf_iterator<char>());
	buffer.push_back('\0'); //Terminate the vector
	assert(buffer.size() > 10);
	doc.parse<0>(&buffer[0]);


	root_node = doc.first_node("InputParameters");
	for (rapidxml::xml_node<> * parameterSetNode = root_node->first_node("ParameterSet"); parameterSetNode; parameterSetNode = parameterSetNode->next_sibling()) {

        if (strcmp(parameterSetNode->first_attribute("name")->value(), "PassThroughFilter") == 0) {
            std::cout << "Reading PassThroughFilter parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "Xmin") == 0) {
                    inputParameters.passThroughFilterParameters.setXmin(atoi(parameterNode->first_attribute("value")->value()));
                    //std::cout << "Xmin set to: " << inputParameters.passThroughFilterParameters.getXmin() << std::endl;
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "Xmax") == 0) {
                    inputParameters.passThroughFilterParameters.setXmax(atoi(parameterNode->first_attribute("value")->value()));
                    //std::cout << "Xmax set to: " << inputParameters.passThroughFilterParameters.getXmax() << std::endl;
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "Ymin") == 0) {
                    inputParameters.passThroughFilterParameters.setYmin(atoi(parameterNode->first_attribute("value")->value()));
                    //std::cout << "Ymin set to: " << inputParameters.passThroughFilterParameters.getYmin() << std::endl;
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "Ymax") == 0) {
                    inputParameters.passThroughFilterParameters.setYmax(atoi(parameterNode->first_attribute("value")->value()));
                    //std::cout << "Ymax set to: " << inputParameters.passThroughFilterParameters.getYmax() << std::endl;
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "Zmin") == 0) {
                    inputParameters.passThroughFilterParameters.setZmin(atoi(parameterNode->first_attribute("value")->value()));
                    //std::cout << "Zmin set to: " << inputParameters.passThroughFilterParameters.getZmin() << std::endl;
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "Zmax") == 0) {
                    inputParameters.passThroughFilterParameters.setZmax(atoi(parameterNode->first_attribute("value")->value()));
                    //std::cout << "Zmax set to: " << inputParameters.passThroughFilterParameters.getZmax() << std::endl;
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }

        }


        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "StatisticalOutlierRemoval") == 0) {
            std::cout << "Reading StatisticalOutlierRemoval parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "MeanK") == 0) {
                    inputParameters.statisticalOutlierRemovalParameters.setMeanK(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "StdDevMulThresh") == 0) {
                    inputParameters.statisticalOutlierRemovalParameters.setStdDevMulThresh(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }


        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "NormalEstimation") == 0) {
            std::cout << "Reading Normal Estimation parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "RadiusSearch") == 0) {
                    inputParameters.normalEstimationParameters.setRadiusSearch(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "KSearch") == 0) {
                    inputParameters.normalEstimationParameters.setKSearch(atoi(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }

        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "FeatureEstimation") == 0) {
            std::cout << "Reading Feature Estimation parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "RadiusSearch") == 0) {
                    inputParameters.featureEstimationParameters.setRadiusSearch(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "KSearch") == 0) {
                    inputParameters.featureEstimationParameters.setKSearch(atoi(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }

        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "SampleConsensusPrerejective") == 0) {
            std::cout << "Reading Sample Consensus Prerejective parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "MaximumIterations") == 0) {
                    inputParameters.sampleConsensusPrerejectiveParameters.setMaximumIterations(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "NumberOfSamples") == 0) {
                    inputParameters.sampleConsensusPrerejectiveParameters.setNumberOfSamples(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "CorrespondenceRandomness") == 0) {
                    inputParameters.sampleConsensusPrerejectiveParameters.setCorrespondenceRandomness(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "SimilarityThreshold") == 0) {
                    inputParameters.sampleConsensusPrerejectiveParameters.setSimilarityThreshold(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "MaxCorrespondenceDistance") == 0) {
                    inputParameters.sampleConsensusPrerejectiveParameters.setMaxCorrespondenceDistance(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "InlierFraction") == 0) {
                    inputParameters.sampleConsensusPrerejectiveParameters.setInlierFraction(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }


        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "VoxelGridFilter") == 0) {
            std::cout << "Reading Voxel Grid Filter parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "LeafSize") == 0) {
                    inputParameters.voxelGridFilterParameters.setLeafSize(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }


        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "IterativeClosestPoint") == 0) {
            std::cout << "Reading Iterative Closest Point parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "MaximumIterations") == 0) {
                    inputParameters.iterativeClosestPointParameters.setMaximumIterations(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "MaxCorrespondenceDistance") == 0) {
                    inputParameters.iterativeClosestPointParameters.setMaxCorrespondenceDistance(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "MinCorrespondenceDistance") == 0) {
                    inputParameters.iterativeClosestPointParameters.setMinCorrespondenceDistance(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "LargeCorrespondenceDistanceStepReduction") == 0) {
                    inputParameters.iterativeClosestPointParameters.setLargeCorrespondenceDistanceStepReduction(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "SmallCorrespondenceDistanceStepReduction") == 0) {
                    inputParameters.iterativeClosestPointParameters.setSmallCorrespondenceDistanceStepReduction(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "ThresholdSwitchFromLargeToSmallDistanceSteps") == 0) {
                    inputParameters.iterativeClosestPointParameters.setThresholdSwitchFromLargeToSmallDistanceSteps(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "FitnessThresholdDefiningFailure") == 0) {
                    inputParameters.iterativeClosestPointParameters.setFitnessThresholdDefiningFailure(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }


        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "RandomSample") == 0) {
            std::cout << "Reading Random Sample parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "SampleProportion") == 0) {
                    inputParameters.randomSampleParameters.setSampleProportion(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }

        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "MovingLeastSquares") == 0) {
            std::cout << "Reading Moving Least Squares parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "SearchRadius") == 0) {
                    inputParameters.movingLeastSquaresParameters.setSearchRadius(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "UpsampleFlag") == 0) {
                    inputParameters.movingLeastSquaresParameters.setUpsampleFlag(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "PolynomialOrder") == 0) {
                    inputParameters.movingLeastSquaresParameters.setPolynomialOrder(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "UpsamplingRadius") == 0) {
                    inputParameters.movingLeastSquaresParameters.setUpsamplingRadius(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "UpsamplingStepSize") == 0) {
                    inputParameters.movingLeastSquaresParameters.setUpsamplingStepSize(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }


        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "RegionGrowing") == 0) {
            std::cout << "Reading Region Growing parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "MinClusterSize") == 0) {
                    inputParameters.regionGrowingParameters.setMinClusterSize(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "MaxClusterSize") == 0) {
                    inputParameters.regionGrowingParameters.setMaxClusterSize(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "NumberOfNeighbours") == 0) {
                    inputParameters.regionGrowingParameters.setNumberOfNeighbours(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "SmoothnessThreshold") == 0) {
                    inputParameters.regionGrowingParameters.setSmoothnessThreshold(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "CurvatureThreshold") == 0) {
                    inputParameters.regionGrowingParameters.setCurvatureThreshold(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }



        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "SACFromNormals") == 0) {
            std::cout << "Reading SAC from normals parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "MaxIterations") == 0) {
                    inputParameters.sacSegmentationFromNormalsParameters.setMaxIterations(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "DistanceThreshold") == 0) {
                    inputParameters.sacSegmentationFromNormalsParameters.setDistanceThreshold(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "NormalDistanceWeight") == 0) {
                    inputParameters.sacSegmentationFromNormalsParameters.setNormalDistanceWeight(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "RadiusLimitsMin") == 0) {
                    inputParameters.sacSegmentationFromNormalsParameters.setRadiusLimitsMin(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "RadiusLimitsMax") == 0) {
                    inputParameters.sacSegmentationFromNormalsParameters.setRadiusLimitsMax(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "SelectWithinDistanceValue") == 0) {
                    inputParameters.sacSegmentationFromNormalsParameters.setSelectWithinDistanceValue(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }

        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "CircleRANSACParameters") == 0) {
            std::cout << "Reading RANSAC from a circle model parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "MaxIterations") == 0) {
                    inputParameters.circleRANSACParameters.setMaxIterations(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "DistanceThreshold") == 0) {
                    inputParameters.circleRANSACParameters.setDistanceThreshold(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "SelectWithinDistanceValue") == 0) {
                    inputParameters.circleRANSACParameters.setSelectWithinDistanceValue(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }

        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "PlaneRANSACParameters") == 0) {
            std::cout << "Reading RANSAC from a plane model parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "MaxIterations") == 0) {
                    inputParameters.planeRANSACParameters.setMaxIterations(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "DistanceThreshold") == 0) {
                    inputParameters.planeRANSACParameters.setDistanceThreshold(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "SelectWithinDistanceValue") == 0) {
                    inputParameters.planeRANSACParameters.setSelectWithinDistanceValue(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }


        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "SupervoxelClustering") == 0) {
            std::cout << "Reading supervoxel clustering parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "VoxelResolution") == 0) {
                    inputParameters.supervoxelClusteringParameters.setVoxelResolution(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "SeedResolution") == 0) {
                    inputParameters.supervoxelClusteringParameters.setSeedResolution(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "ColorImportance") == 0) {
                    inputParameters.supervoxelClusteringParameters.setColorImportance(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "SpatialImportance") == 0) {
                    inputParameters.supervoxelClusteringParameters.setSpatialImportance(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "NormalImportance") == 0) {
                    inputParameters.supervoxelClusteringParameters.setNormalImportance(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }

        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "CameraParameters") == 0) {
            std::cout << "Reading camera parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "xCoordinateLocation") == 0) {
                    inputParameters.cameraParameters.setxCoordLocation(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "yCoordinateLocation") == 0) {
                    inputParameters.cameraParameters.setyCoordLocation(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "zCoordinateLocation") == 0) {
                    inputParameters.cameraParameters.setzCoordLocation(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "xViewComponent") == 0) {
                    inputParameters.cameraParameters.setxViewComponent(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "yViewComponent") == 0) {
                    inputParameters.cameraParameters.setyViewComponent(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "zViewComponent") == 0) {
                    inputParameters.cameraParameters.setzViewComponent(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "xUpComponent") == 0) {
                    inputParameters.cameraParameters.setxUpComponent(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "yUpComponent") == 0) {
                    inputParameters.cameraParameters.setyUpComponent(atof(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "zUpComponent") == 0) {
                    inputParameters.cameraParameters.setzUpComponent(atof(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }

        else if (strcmp(parameterSetNode->first_attribute("name")->value(), "Debugging") == 0) {
            std::cout << "Reading debugging parameter set" << std::endl;
            for(rapidxml::xml_node<> * parameterNode = parameterSetNode->first_node("Parameter"); parameterNode; parameterNode = parameterNode->next_sibling()) {
                if (strcmp(parameterNode->first_attribute("name")->value(), "DebuggingLevel") == 0) {
                    inputParameters.debuggingParameters.setDebuggingLevel(atoi(parameterNode->first_attribute("value")->value()));
                }
                else if (strcmp(parameterNode->first_attribute("name")->value(), "LoggingPath") == 0) {
                    inputParameters.debuggingParameters.setLoggingPath(std::string(parameterNode->first_attribute("value")->value()));
                }
                else {
                    std::cout << "Unrecognized parameter name: " << parameterNode->first_attribute("name")->value() << ". Aborting." << std::endl;
                    return(1);
                }
            }
        }


        else {
            std::cout << "Unrecognized parameter set name: " << parameterSetNode->first_attribute("name")->value() << ". Aborting." << std::endl;
            return(1);
        }
	}

	return(0);
}
