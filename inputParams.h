#ifndef INPUTPARAMS_H
#define INPUTPARAMS_H
#include <string>

/** \brief Parameters obtained from the input .xml file for an x, y, z pass through filter with which point clouds are filtered.
  */
class PassThroughFilterParameters{
    public:
        PassThroughFilterParameters();
        ~PassThroughFilterParameters();

        int getXmin();
        void setXmin(int inputXmin);
        int getXmax();
        void setXmax(int inputXmax);
        int getYmin();
        void setYmin(int inputYmin);
        int getYmax();
        void setYmax(int inputYmax);
        int getZmin();
        void setZmin(int inputZmin);
        int getZmax();
        void setZmax(int inputZmax);

        void printParameters();

    private:
        int _Xmin;
        int _Xmax;
        int _Ymin;
        int _Ymax;
        int _Zmin;
        int _Zmax;
};

/** \brief Parameters obtained from the input .xml file for the removal of outlier points.
  */
class StatisticalOutlierRemovalParameters{
    public:
        StatisticalOutlierRemovalParameters();
        ~StatisticalOutlierRemovalParameters();

        int getMeanK();
        void setMeanK(int inputMeanK);
        float getStdDevMulThresh();
        void setStdDevMulThresh(float inputStdDevMulThresh);

        void printParameters();

    private:
        int _MeanK;
        float _StdDevMulThresh;
};

/** \brief Parameters obtained from the input .xml file for estimation of point normals.
  */
class NormalEstimationParameters{
    public:
        NormalEstimationParameters();
        ~NormalEstimationParameters();

        float getRadiusSearch();
        void setRadiusSearch(float inputRadiusSearch);
        int getKSearch();
        void setKSearch(int inputKSearch);

        void printParameters();

    private:

        float _RadiusSearch;
        int _KSearch;
};

/** \brief Parameters obtained from the input .xml file for estimation of point features.
  */
class FeatureEstimationParameters{
    public:
        FeatureEstimationParameters();
        ~FeatureEstimationParameters();

        float getRadiusSearch();
        void setRadiusSearch(float inputRadiusSearch);
        int getKSearch();
        void setKSearch(int inputKSearch);

        void printParameters();

    private:

        float _RadiusSearch;
        int _KSearch;
};

/** \brief Parameters obtained from the input .xml file for prerejective RANSAC.
  */
class SampleConsensusPrerejectiveParameters{
    public:
        SampleConsensusPrerejectiveParameters();
        ~SampleConsensusPrerejectiveParameters();

        int getMaximumIterations();
        void setMaximumIterations(int inputMaximumIterations);
        int getNumberOfSamples();
        void setNumberOfSamples(int inputNumberOfSamples);
        int getCorrespondenceRandomness();
        void setCorrespondenceRandomness(int inputCorrespondenceRandomness);
        float getSimilarityThreshold();
        void setSimilarityThreshold(float inputSimilarityThreshold);
        float getMaxCorrespondenceDistance();
        void setMaxCorrespondenceDistance(float inputMaxCorrespondenceDistance);
        float getInlierFraction();
        void setInlierFraction(float inputInlierFraction);

        void printParameters();

    private:
        int _MaximumIterations;
        int _NumberOfSamples;
        int _CorrespondenceRandomness;
        float _SimilarityThreshold;
        float _MaxCorrespondenceDistance;
        float _InlierFraction;
};

/** \brief Parameters obtained from the input .xml file used to downsample a cloud using a voxel grid.
  */
class VoxelGridFilterParameters{
    public:
        VoxelGridFilterParameters();
        ~VoxelGridFilterParameters();

        float getLeafSize();
        void setLeafSize(float inputLeafSize);

        void printParameters();

    private:

        float _LeafSize;
};

/** \brief Parameters obtained from the input .xml file used to perform iterative closest point registration.
  */
class IterativeClosestPointParameters{
    public:
        IterativeClosestPointParameters();
        ~IterativeClosestPointParameters();

        int getMaximumIterations();
        void setMaximumIterations(int inputMaximumIterations);
        float getMaxCorrespondenceDistance();
        void setMaxCorrespondenceDistance(float inputMaxCorrespondenceDistance);
        float getMinCorrespondenceDistance();
        void setMinCorrespondenceDistance(float inputMinCorrespondenceDistance);
        float getLargeCorrespondenceDistanceStepReduction();
        void setLargeCorrespondenceDistanceStepReduction(float inputLargeCorrespondenceDistanceStepReduction);
        float getSmallCorrespondenceDistanceStepReduction();
        void setSmallCorrespondenceDistanceStepReduction(float inputSmallCorrespondenceDistanceStepReduction);
        float getThresholdSwitchFromLargeToSmallDistanceSteps();
        void setThresholdSwitchFromLargeToSmallDistanceSteps(float inputThresholdSwitchFromLargeToSmallDistanceSteps);
        float getFitnessThresholdDefiningFailure();
        void setFitnessThresholdDefiningFailure(float inputFitnessThresholdDefiningFailure);

        void printParameters();

    private:

        float _MaximumIterations;
        float _MaxCorrespondenceDistance;
        float _MinCorrespondenceDistance;
        float _LargeCorrespondenceDistanceStepReduction;
        float _SmallCorrespondenceDistanceStepReduction;
        float _ThresholdSwitchFromLargeToSmallDistanceSteps;
        float _FitnessThresholdDefiningFailure;

};

/** \brief Parameters obtained from the input .xml file used to randomly sample points from a cloud
  */
class RandomSampleParameters{
    public:
        RandomSampleParameters();
        ~RandomSampleParameters();

        float getSampleProportion();
        void setSampleProportion(float inputSampleProportion);

        void printParameters();

    private:

        float _SampleProportion;
};

/** \brief Parameters obtained from the input .xml file used for MLS.
  */
class MovingLeastSquaresParameters{
    public:
        MovingLeastSquaresParameters();
        ~MovingLeastSquaresParameters();

        float getSearchRadius();
        void setSearchRadius(float inputSearchRadius);
        int getUpsampleFlag();
        void setUpsampleFlag(int inputUpsampleFlag);
        int getPolynomialOrder();
        void setPolynomialOrder(int inputPolynomialOrder);
        float getUpsamplingRadius();
        void setUpsamplingRadius(float inputUpsamplingRadius);
        float getUpsamplingStepSize();
        void setUpsamplingStepSize(float inputUpsamplingStepSize);

        void printParameters();

    private:

        float _SearchRadius;
        int _UpsampleFlag;
        int _PolynomialOrder;
        float _UpsamplingRadius;
        float _UpsamplingStepSize;
};

/** \brief Parameters obtained from the input .xml file used for region growing.
  */
class RegionGrowingParameters{
    public:
        RegionGrowingParameters();
        ~RegionGrowingParameters();

        int getMinClusterSize();
        void setMinClusterSize(int inputMinClusterSize);
        int getMaxClusterSize();
        void setMaxClusterSize(int inputMaxClusterSize);
        int getNumberOfNeighbours();
        void setNumberOfNeighbours(int inputNumberOfNeighbours);
        float getSmoothnessThreshold();
        void setSmoothnessThreshold(float inputSmoothnessThreshold);
        float getCurvatureThreshold();
        void setCurvatureThreshold(float inputCurvatureThreshold);

        void printParameters();

    private:
        int _MinClusterSize;
        int _MaxClusterSize;
        int _NumberOfNeighbours;
        float _SmoothnessThreshold;
        float _CurvatureThreshold;
};

/** \brief Parameters obtained from the input .xml file used for RANSAC with normals to fit a cylinder model.
  */
class SACSegmentationFromNormalsParameters{
    public:
        SACSegmentationFromNormalsParameters();
        ~SACSegmentationFromNormalsParameters();

        int getMaxIterations();
        void setMaxIterations(int inputMaxIterations);
        float getDistanceThreshold();
        void setDistanceThreshold(float inputDistanceThreshold);
        float getNormalDistanceWeight();
        void setNormalDistanceWeight(float inputNormalDistanceWeight);
        float getRadiusLimitsMin();
        void setRadiusLimitsMin(float inputRadiusLimitsMin);
        float getRadiusLimitsMax();
        void setRadiusLimitsMax(float inputRadiusLimitsMax);
        float getSelectWithinDistanceValue();
        void setSelectWithinDistanceValue(float inputSelectWithinDistanceValue);

        void printParameters();

    private:
        int _MaxIterations;
        float _DistanceThreshold;
        float _NormalDistanceWeight;
        float _RadiusLimitsMin;
        float _RadiusLimitsMax;
        float _SelectWithinDistanceValue;

};

/** \brief Parameters obtained from the input .xml file used for RANSAC to fit a 2D circle model.
  */
class CircleRANSACParameters{
    public:
        CircleRANSACParameters();
        ~CircleRANSACParameters();

        int getMaxIterations();
        void setMaxIterations(int inputMaxIterations);
        float getDistanceThreshold();
        void setDistanceThreshold(float inputDistanceThreshold);
        float getSelectWithinDistanceValue();
        void setSelectWithinDistanceValue(float inputSelectWithinDistanceValue);

        void printParameters();

    private:
        int _MaxIterations;
        float _DistanceThreshold;
        float _SelectWithinDistanceValue;

};

/** \brief Parameters obtained from the input .xml file used for RANSAC to fit a plane model.
  */
class PlaneRANSACParameters{
    public:
        PlaneRANSACParameters();
        ~PlaneRANSACParameters();

        int getMaxIterations();
        void setMaxIterations(int inputMaxIterations);
        float getDistanceThreshold();
        void setDistanceThreshold(float inputDistanceThreshold);
        float getSelectWithinDistanceValue();
        void setSelectWithinDistanceValue(float inputSelectWithinDistanceValue);

        void printParameters();

    private:
        int _MaxIterations;
        float _DistanceThreshold;
        float _SelectWithinDistanceValue;

};

/** \brief Parameters obtained from the input .xml file used to cluster a point cloud into supervoxels.
  */
class SupervoxelClusteringParameters{
    public:
        SupervoxelClusteringParameters();
        ~SupervoxelClusteringParameters();

        float getVoxelResolution();
        void setVoxelResolution(float inputVoxelResolution);
        float getSeedResolution();
        void setSeedResolution(float inputSeedResolution);
        float getColorImportance();
        void setColorImportance(float inputColorImportance);
        float getSpatialImportance();
        void setSpatialImportance(float inputSpatialImportance);
        float getNormalImportance();
        void setNormalImportance(float inputNormalImportance);

        void printParameters();

    private:
        float _VoxelResolution;
        float _SeedResolution;
        float _ColorImportance;
        float _SpatialImportance;
        float _NormalImportance;

};

/** \brief Parameters obtained from the input .xml file used to define the debugging level. Higher levels provide more information.
  */
class DebuggingParameters{
    public:
        DebuggingParameters();
        ~DebuggingParameters();

        float getDebuggingLevel();
        void setDebuggingLevel(int inputDebuggingLevel);

        std::string getLoggingPath();
        void setLoggingPath(std::string inputLoggingPath);

        void printParameters();

    private:
        int _DebuggingLevel;
        std::string _loggingPath;

};

/** \brief Parameters obtained from the input .xml file used to define where the location and orientation of the scene camera (if debugging level is
  * sufficiently high).
  */
class CameraParameters{
    public:
        CameraParameters();
        ~CameraParameters();

        double getxCoordLocation();
        void setxCoordLocation(double inputxCoordLocation);

        double getyCoordLocation();
        void setyCoordLocation(double inputyCoordLocation);

        double getzCoordLocation();
        void setzCoordLocation(double inputzCoordLocation);

        double getxViewComponent();
        void setxViewComponent(double inputxViewComponent);

        double getyViewComponent();
        void setyViewComponent(double inputyViewComponent);

        double getzViewComponent();
        void setzViewComponent(double inputzViewComponent);

        double getxUpComponent();
        void setxUpComponent(double inputxUpComponent);

        double getyUpComponent();
        void setyUpComponent(double inputyUpComponent);

        double getzUpComponent();
        void setzUpComponent(double inputzUpComponent);

        int getDebuggingLevel();
        void setDebuggingLevel(int inputDebuggingLevel);

        void printParameters();

    private:
        double _xCoordLocation;
        double _yCoordLocation;
        double _zCoordLocation;
        double _xViewComponent;
        double _yViewComponent;
        double _zViewComponent;
        double _xUpComponent;
        double _yUpComponent;
        double _zUpComponent;
        int _DebuggingLevel;

};

/** \brief Container that holds all input parameters so that they can be passed to processing functions.
  */
class InputParameters {
public:
    InputParameters();
    ~InputParameters();

    PassThroughFilterParameters passThroughFilterParameters;
    StatisticalOutlierRemovalParameters statisticalOutlierRemovalParameters;
    NormalEstimationParameters normalEstimationParameters;
    FeatureEstimationParameters featureEstimationParameters;
    SampleConsensusPrerejectiveParameters sampleConsensusPrerejectiveParameters;
    VoxelGridFilterParameters voxelGridFilterParameters;
    IterativeClosestPointParameters iterativeClosestPointParameters;
    RandomSampleParameters randomSampleParameters;
    MovingLeastSquaresParameters movingLeastSquaresParameters;
    RegionGrowingParameters regionGrowingParameters;
    SACSegmentationFromNormalsParameters sacSegmentationFromNormalsParameters;
    CircleRANSACParameters circleRANSACParameters;
    PlaneRANSACParameters planeRANSACParameters;
    SupervoxelClusteringParameters supervoxelClusteringParameters;
    DebuggingParameters debuggingParameters;
    CameraParameters cameraParameters;

    void printInputParameters();

};

/** \brief Class to parse XML files.
  */
class XMLParser{
    public:
        XMLParser();
        ~XMLParser();

        int parseXMLtoInputParameters(InputParameters &inputParameters, std::string inputFileName);

};

#endif
