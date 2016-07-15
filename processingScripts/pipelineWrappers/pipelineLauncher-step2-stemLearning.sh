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

#TODO: We probably need to check if MeshLab and ImageMagick are in the path, and abort if not.

if [ -x ${MULTIBOOSTBINARY} ]
then
	echo "Found the Multiboost binary. Proceeding."
else
	echo "Unable to find the executable Multiboost binary ${MULTIBOOSTBINARY}. Aborting."
	exit 1
fi

if [ -x ${SOURCEDIR}executeFeatures.sh ]
then
	${SOURCEDIR}executeFeatures.sh ${SOURCEDIR} ${CONFIGFILEDIR} ${PROCESSINGBINARY} ${UNSEGMENTEDMESHDIR} ${LEARNEDLABELSDIR}
else 
	echo "Unable to execute ${SOURCEDIR}executeFeatures.sh. Make sure permissions are set correctly or that it's in the correct location. Aborting."
	exit 1
fi

if [ -x ${SOURCEDIR}multiBoost.sh ]
then
	${SOURCEDIR}multiBoost.sh ${SOURCEDIR} ${MULTIBOOSTBINARY} ${UNSEGMENTEDMESHDIR} ${LEARNEDLABELSDIR}
else
	echo "Unable to execute ${SOURCEDIR}imagesToMesh.sh. Make sure permissions are set correctly or that it's in the correct location. Aborting."
	exit 1
fi

