# Launches the depth image processing pipeline

# Path to the image processing binary
PROCESSINGBINARY=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/DepthCameraPhenotyping_Prototype4/bin/Debug/DepthCameraPhenotyping_v4
# Path to the directory containing the two Screened Poisson Surface Reconstruction binaries
# http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version7.0/
POISSONBINARYDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/PoissonRecon/Bin/Linux/

# Path to the directory containing the configuration files used for processing
CONFIGFILEDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite/processingConfigFiles/devTestDebug/

# Path to the directory to which files will be output. 
OUTPUTDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite/ProcessedData/ProcessedClouds/

# Array containing the paths to directories that hold directories with image data.
PARENTDIR[0]=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite/images/*
#PARENTDIR[1]=/home/rfm_node03/Documents/Data/Image-BasedPhenotyping/ImageData/BTx623xIS3620C/BTx623xIS3620C_08-03/Rep_2/*_2
#PARENTDIR[2]=/home/rfm_node03/Documents/Data/Image-BasedPhenotyping/ImageData/BTx623xIS3620C/BTx623xIS3620C_08-03/Rep_3/*_3

checkAssertionStatus () {
	returnedValue=$1
	assertValue=134  # This is what is returned (POSIX I think?) on a failed assert.
	if [ $returnedValue -eq $assertValue ]
	then
		echo "Process failed. We should probably log something here. Or maybe that should be the job of the program."
	fi
}

# First check the output directory for already processed samples (e.g. if need to restart after a computer lock up) 
PROCESSEDSAMPLELIST=()
processedSamples=${OUTPUTDIR}*PoissonReconTrimmed.ply
for file in ${processedSamples}
do
	strippedPath=${file##*/}
	sampleName=${strippedPath%_PoissonReconTrimmed.ply}
	PROCESSEDSAMPLELIST+=($sampleName)
done

IMAGEDIR=()
for dir in ${PARENTDIR[@]}
do
	IMAGEDIR+=($dir)
done

for dir in ${IMAGEDIR[@]}
do
        parentDir=$(basename ${dir})
	bool_sampleAlreadyProcessed="false"
	for sampleName in ${PROCESSEDSAMPLELIST[@]}
	do
		if [ ${parentDir} = ${sampleName} ]
		then
			bool_sampleAlreadyProcessed="true"
			continue
		fi
	done

	if [ ${bool_sampleAlreadyProcessed} = "false" ]
	then
		echo "Processing sample ${parentDir}"

		cloudDirFiles=sample_clouds/*.pcd
		cloudDir=sample_clouds

		inputCloudString=""
		i=1
		for file in ${cloudDirFiles}
		do
	       		inputCloudString="$inputCloudString ${cloudDir}/${i}.pcd"
			i=$(($i+1))
		done

		echo
		echo "Input string will be    $inputCloudString"
		echo

		# Perform an initial pairwise registration
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}registrationParametersInitialPairwise.xml \
			--registerPointCloudsICP_pairwise $inputCloudString
	fi
done
