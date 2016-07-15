# This file is used to launch processing scripts. The following paths need to be specified:

# ID of the sample for which the combinedCloudFiltered.ply file has been manually corrected.
inputSampleIDs[0]=19_3
inputSampleIDs[1]=72_3
inputSampleIDs[2]=97_3

# The path to the processing scripts that will be launched
SOURCEDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/SorghumMeshPhenotyper_v5/processingScripts/

# The path to the processing binary
PROCESSINGBINARY=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/SorghumMeshPhenotyper_v5/bin/Debug/SorghumMeshPhenotyper_v5

# Path to the directory containing the two Screened Poisson Surface Reconstruction binaries
# http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version7.0/
POISSONBINARYDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/PoissonRecon/Bin/Linux/

# Path to the configuration files. This shouldn't need to be modified.
CONFIGFILEDIR=processingConfigFiles/

# Path to the directory to put processed meshes before and after segmentation. This shouldn't need to be modified.
UNSEGMENTEDMESHDIR=ProcessedData/ProcessedClouds/
SEGMENTEDMESHDIR=ProcessedData/SegmentedMeshes_Measurements/
SEGMENTEDMESHSCREENSHOTDIR=ProcessedData/SegmentedMeshes_Screenshots/
MESHSCREENSHOTDIR=ProcessedData/ProcessedClouds_Screenshots/

#TODO: We probably need to check if MeshLab and ImageMagick are in the path, and abort if not.

if [ -x ${POISSONBINARYDIR}PoissonRecon ]
then
	echo "Found the Poisson Reconstruction binary. Proceeding."
else
	echo "Unable to find the executable Poisson Reconstruction binary ${POISSONBINARYDIR}PoissonRecon. Aborting."
	exit 1
fi

if [ -x ${SOURCEDIR}postManualIntervention/meshAfterCurationOfCombinedCloudFiltered.sh ]
then
	for inputSampleID in ${inputSampleIDs[@]}
	do
		rm ${UNSEGMENTEDMESHDIR}${inputSampleID}_combinedCloudFiltered_postProcessed.ply
		rm ${UNSEGMENTEDMESHDIR}${inputSampleID}_Poisson*.ply
		echo "Reprocessing ${inputSampleID}_combinedCloudFiltered.ply"
		${SOURCEDIR}postManualIntervention/meshAfterCurationOfCombinedCloudFiltered.sh ${SOURCEDIR} ${CONFIGFILEDIR} ${PROCESSINGBINARY} ${POISSONBINARYDIR} ${UNSEGMENTEDMESHDIR} ${inputSampleID}
	done
else
	echo "Unable to execute ${SOURCEDIR}postManualIntervention/meshAfterCurationOfCombinedCloudFiltered.sh. Aborting."
	exit 1
fi

echo
echo "All image directories have a corresponding mesh. Continuing."
echo

if [ -x ${SOURCEDIR}smoothClouds.sh ]
then
	${SOURCEDIR}smoothClouds.sh ${SOURCEDIR} ${CONFIGFILEDIR} ${UNSEGMENTEDMESHDIR}
else
	echo "Unable to execute ${SOURCEDIR}smoothClouds.sh. Make sure permissions are set correctly or that it's in the correct location. Aborting."
	exit 1
fi

echo
echo "All meshes have been smoothed. Continuing."
echo

if [ -x ${SOURCEDIR}screenshotMeshPLYs.sh ]
then
	${SOURCEDIR}screenshotMeshPLYs.sh ${SOURCEDIR} ${CONFIGFILEDIR} ${PROCESSINGBINARY} ${UNSEGMENTEDMESHDIR} ${MESHSCREENSHOTDIR}
else
	echo "Unable to execute ${SOURCEDIR}screenshotMeshPLYs.sh. Make sure permissions are set correctly or that it's in the correct location. Aborting."
	exit 1
fi
