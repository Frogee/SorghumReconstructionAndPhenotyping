# Launches the depth image processing pipeline

SOURCEDIR=$1

# Path to the image processing binary
#PROCESSINGBINARY=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/DepthCameraPhenotyping_Prototype4/bin/Debug/DepthCameraPhenotyping_v4
PROCESSINGBINARY=$3

# Path to the directory containing the two Screened Poisson Surface Reconstruction binaries
# http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version7.0/
#POISSONBINARYDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/PoissonRecon/Bin/Linux/
POISSONBINARYDIR=$4

# Path to the directory containing the configuration files used for processing
#CONFIGFILEDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite/processingConfigFiles/devTestDebug/
CONFIGFILEDIR=$2

# Path to the directory to which files will be output. 
#OUTPUTDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite/ProcessedData/ProcessedClouds/
OUTPUTDIR=$5

inputSampleID=$6

echo
echo
echo "Executing meshAfterCurationOfCombinedCloudFiltered with the following settings:"
echo
echo "Directory with shell scripts: ${SOURCEDIR}"
echo "Directory with configuration files: ${CONFIGFILEDIR}"
echo "Path to processing binary: ${PROCESSINGBINARY}"
echo "Path to input PLY: ${inputSampleID}"
echo "Directory to write output to: ${OUTPUTDIR}"
echo


		# Perform some additional post registration filtering.
		${SOURCEDIR}postProcess.sh ${OUTPUTDIR}${inputSampleID}_combinedCloudFiltered.ply \
			${CONFIGFILEDIR}postRegistration_preMeshing_processing_step1.mlx \
			${CONFIGFILEDIR}postRegistration_preMeshing_processing_step2.mlx

		# Use Screened Poisson Surface Reconstruction to mesh the cloud.
		inputPly=${OUTPUTDIR}combinedCloudFiltered_postProcessed.ply
		echo
		echo "Constructing surface"
		echo

		source ${CONFIGFILEDIR}ScreenedPoissonReconstructionParams.sh

		${POISSONBINARYDIR}PoissonRecon --in ${OUTPUTDIR}combinedCloudFiltered_postProcessed.ply --out ${OUTPUTDIR}PoissonReconout.ply --density \
			--threads ${threads} \
			--depth ${depth} \
			--fullDepth ${fullDepth} \
			--voxelDepth ${voxelDepth} \
			--samplesPerNode ${samplesPerNode} \
			--pointWeight ${pointWeight}

		echo
		echo "Trimming surface"
		echo

		${POISSONBINARYDIR}SurfaceTrimmer --in ${OUTPUTDIR}PoissonReconout.ply --out ${OUTPUTDIR}PoissonReconTrimmed.ply \
			--trim ${trim} \
			--smooth ${smooth} \
			--aRatio ${aRatio}
	
		rm ${OUTPUTDIR}PoissonReconout.ply
		mv ${OUTPUTDIR}combinedCloudFiltered_postProcessed.ply ${OUTPUTDIR}${inputSampleID}_combinedCloudFiltered_postProcessed.ply
		mv ${OUTPUTDIR}PoissonReconTrimmed.ply ${OUTPUTDIR}${inputSampleID}_PoissonReconTrimmed.ply
