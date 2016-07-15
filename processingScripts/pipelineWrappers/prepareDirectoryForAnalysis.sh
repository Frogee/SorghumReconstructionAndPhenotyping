SOURCEDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/SorghumMeshPhenotyper_v5/

### You shouldn't need to modify anything else below provided you have write permissions to the working directory in which this is run.

if [ ! -d ${SOURCEDIR} ]
then
	echo "Unable to find source directory. Check that the variable is pointing to the right location."
fi

if [ ! -d "./ProcessedData" ]
then
	mkdir ./ProcessedData
fi

if [ ! -d "./ProcessedData/ProcessedClouds" ]
then
	mkdir ./ProcessedData/ProcessedClouds
	mkdir ./ProcessedData/ProcessedClouds/AdaBoostLearning
fi

if [ ! -d "./ProcessedData/SegmentedMeshes_Measurements" ]
then
	mkdir ./ProcessedData/SegmentedMeshes_Measurements
fi

if [ ! -d "./ProcessedData/SegmentedMeshes_Screenshots" ]
then
	mkdir ./ProcessedData/SegmentedMeshes_Screenshots
fi

if [ ! -d "./ProcessedData/ProcessedClouds_Screenshots" ]
then
	mkdir ./ProcessedData/ProcessedClouds_Screenshots
fi

if [ ! -d "./sample_cloud_PLYs" ]
then
	mkdir ./sample_cloud_PLYs
fi

if [ ! -d "./sample_clouds" ]
then
	mkdir ./sample_clouds
fi

if [ ! -d "./sample_cloud_PLYs" ]
then
	mkdir ./sample_cloud_PLYs
fi

if [ ! -d "./sample_clouds_unprocessed" ]
then
	mkdir ./sample_clouds_unprocessed
fi

if [ ! -d "./sample_globalICP" ]
then
	mkdir ./sample_globalICP
fi

if [ ! -d "./sample_pairwiseICP" ]
then
	mkdir ./sample_pairwiseICP
fi

if [ ! -d "./sample_RANSACclouds" ]
then
	mkdir ./sample_RANSACclouds
fi

if [ ! -d "./sample_registeredClouds" ]
then
	mkdir ./sample_registeredClouds
fi

if [ ! -d "./segmentation_PLYs" ]
then
	mkdir ./segmentation_PLYs
fi

if [ ! -d "./processingConfigFiles" ]
then
	mkdir ./processingConfigFiles
	cp ${SOURCEDIR}defaultConfigFiles/*.xml ./processingConfigFiles/
	cp ${SOURCEDIR}defaultConfigFiles/*.sh ./processingConfigFiles/
	cp ${SOURCEDIR}defaultConfigFiles/*.mlx ./processingConfigFiles/
fi

if [ ! -d "./PhenoPlots" ]
then
	mkdir ./PhenoPlots
fi

cp ${SOURCEDIR}processingScripts/pipelineWrappers/pipelineLauncher* ./
cp ${SOURCEDIR}processingScripts/pipelineWrappers/reprocessing* ./

echo
echo "Finished preparing the directory. Read the pipeline launcher scripts to launch the pipeline."
echo

