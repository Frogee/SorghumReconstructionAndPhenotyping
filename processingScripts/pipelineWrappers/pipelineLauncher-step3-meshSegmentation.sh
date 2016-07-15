# This file is used to launch processing scripts. The following paths need to be specified:

# The path to the processing scripts that will be launched
SOURCEDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/SorghumMeshPhenotyper_v5/processingScripts/

# Path to the multiboost binary.
# http://www.multiboost.org/
MULTIBOOSTBINARY=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/MultiBoost-1.2.02/MultiBoost-Build/multiboost

# The path to the processing binary
PROCESSINGBINARY=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/SorghumMeshPhenotyper_v5/bin/Debug/SorghumMeshPhenotyper_v5

# Path to the configuration files. This shouldn't need to be modified.
CONFIGFILEDIR=processingConfigFiles/

# Path to the directory to put processed meshes before and after segmentation. This shouldn't need to be modified.
UNSEGMENTEDMESHDIR=ProcessedData/ProcessedClouds/
LEARNEDLABELSDIR=ProcessedData/ProcessedClouds/AdaBoostLearning/
SEGMENTEDMESHDIR=ProcessedData/SegmentedMeshes_Measurements/
SEGMENTEDMESHSCREENSHOTDIR=ProcessedData/SegmentedMeshes_Screenshots/


if [ -x ${SOURCEDIR}executeSegmentation.sh ]
then
	${SOURCEDIR}executeSegmentation.sh ${SOURCEDIR} ${CONFIGFILEDIR} ${PROCESSINGBINARY} ${UNSEGMENTEDMESHDIR} ${LEARNEDLABELSDIR} ${SEGMENTEDMESHDIR}
else 
	echo "Unable to execute ${SOURCEDIR}executeSegmentation.sh. Make sure permissions are set correctly or that it's in the correct location. Aborting."
	exit 1
fi

if [ -x ${SOURCEDIR}screenshotSegmentedPLYs.sh ]
then
	${SOURCEDIR}screenshotSegmentedPLYs.sh ${SOURCEDIR} ${CONFIGFILEDIR} ${PROCESSINGBINARY} ${SEGMENTEDMESHDIR} ${SEGMENTEDMESHSCREENSHOTDIR}
else
	echo "Unable to execute ${SOURCEDIR}screenshotSegmentedPLYs.sh. Make sure permissions are set correctly or that it's in the correct location. Aborting."
	exit 1
fi

