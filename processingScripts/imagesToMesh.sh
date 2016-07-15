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
OUTPUTDIR=$6

# Array containing the paths to directories that hold directories with image data.
#PARENTDIR[0]=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite/images/*
#PARENTDIR[1]=/home/rfm_node03/Documents/Data/Image-BasedPhenotyping/ImageData/BTx623xIS3620C/BTx623xIS3620C_08-03/Rep_2/*_2
PARENTDIR[0]=$5*

echo
echo
echo "Executing images to mesh with the following settings:"
echo
echo "Directory with shell scripts: ${SOURCEDIR}"
echo "Directory with configuration files: ${CONFIGFILEDIR}"
echo "Path to processing binary: ${PROCESSINGBINARY}"
echo "Path to images: ${PARENTDIR[0]}"
echo "Directory to write output to: ${OUTPUTDIR}"
echo

checkAssertionStatus () {
	returnedValue=$1
	assertValue=134  # This is what is returned (POSIX I think?) on a failed assert.
	if [ $returnedValue -eq $assertValue ]
	then
		echo "Processed failed. Aborting."
		exit 1
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

		rm sample_clouds/*.pcd
		rm sample_cloud_PLYs/*.ply
		rm sample_clouds_unprocessed/*.pcd
		rm sample_registeredClouds/*.pcd
		rm sample_pairwiseICP/*.pcd
		rm sample_globalICP/*.pcd

		images=${dir}/DepthImage*.png
		echo ${images}
		echo $dir

		inputImageString=""
		i=0
		for file in ${images}
		do
			if [ $i -lt 12 ]   # Expecting only 12 images per directory.
			then
				inputImageString="$inputImageString $file"
				i=$(($i+1))
			fi
		done
		
		# Convert the depth images to point clouds.
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}convertImageToCloudParameters.xml \
			--convertImageToCloud $inputImageString
		checkAssertionStatus $?

		cloudDirFiles=sample_clouds/*.pcd
		cloudDir=sample_clouds

		inputCloudString=""
		i=1
		for file in ${cloudDirFiles}
		do
	       		inputCloudString="$inputCloudString ${cloudDir}/${i}.pcd"
			i=$(($i+1))
		done

		# Perform an initial pairwise registration
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}registrationParametersInitialPairwise.xml \
			--registerPointCloudsICP_pairwise $inputCloudString
		checkAssertionStatus $?	

		inputCloudString=""
		cloudDirFiles=sample_pairwiseICP/*.pcd
		cloudDir=sample_pairwiseICP

		i=1
		for file in ${cloudDirFiles}
		do
			inputCloudString="$inputCloudString ${cloudDir}/${i}_ICP.pcd"
			i=$(($i+1))
		done


		echo
		echo "Input string will be    $inputCloudString"
		echo
	
		# Refine the initial pairwise registration with an initial global registration
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}registrationParametersInitialGlobal.xml \
			--registerPointCloudsICP_oneAgainstGlobal $inputCloudString
		checkAssertionStatus $?

		mv sample_registeredClouds/*_ICP.pcd sample_globalICP/
	
		inputCloudString=""
		cloudDirFiles=sample_globalICP/*_ICP.pcd
		cloudDir=sample_globalICP

		i=1
		for file in ${cloudDirFiles}
		do
			inputCloudString="$inputCloudString ${cloudDir}/${i}_ICP.pcd"
			i=$(($i+1))
		done

		echo
		echo "Input string will be    $inputCloudString"
		echo

		# Use the initial registration to segment the pot out.
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}potSegmentation.xml \
			--segmentOutPot $inputCloudString
		checkAssertionStatus $?

		inputCloudString=""
		cloudDirFiles=sample_globalICP/*_ICP_potRemoved.pcd
		cloudDir=sample_globalICP

		i=1
		for file in ${cloudDirFiles}
		do
			inputCloudString="$inputCloudString ${cloudDir}/${i}_ICP_potRemoved.pcd"
			i=$(($i+1))
		done

		echo
		echo "Input string will be    $inputCloudString"
		echo

		# Once the pot is segmented out, refine the registration.
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}registrationParametersRefinement.xml \
			--registerPointCloudsICP_refinement $inputCloudString
		checkAssertionStatus $?

		combinedCloudDir=./sample_registeredClouds/
		combinedCloud=./sample_registeredClouds/combinedCloud.pcd
		
		# Filter the registered cloud.
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}postRegistrationFilteringParameters.xml \
			--removeStatisticalOutliers $combinedCloud
		checkAssertionStatus $?

		# Perform some additional post registration filtering.
		${SOURCEDIR}postProcess.sh sample_registeredClouds/combinedCloudFiltered.ply \
			${CONFIGFILEDIR}postRegistration_preMeshing_processing_step1.mlx \
			${CONFIGFILEDIR}postRegistration_preMeshing_processing_step2.mlx
		checkAssertionStatus $?

		# Use Screened Poisson Surface Reconstruction to mesh the cloud.
		inputPly=./sample_registeredClouds/combinedCloudFiltered_postProcessed.ply
		#--depth had previously been set to 7; trying to set to 8 to prevent smaller leaves from getting cut out.
		echo
		echo "Constructing surface"
		echo

		source ${CONFIGFILEDIR}ScreenedPoissonReconstructionParams.sh

		${POISSONBINARYDIR}PoissonRecon --in $inputPly --out sample_registeredClouds/PoissonReconout.ply --density \
			--threads ${threads} \
			--depth ${depth} \
			--fullDepth ${fullDepth} \
			--voxelDepth ${voxelDepth} \
			--samplesPerNode ${samplesPerNode} \
			--pointWeight ${pointWeight}
		checkAssertionStatus $?

		echo
		echo "Trimming surface"
		echo

		${POISSONBINARYDIR}SurfaceTrimmer --in sample_registeredClouds/PoissonReconout.ply --out sample_registeredClouds/PoissonReconTrimmed.ply \
			--trim ${trim} \
			--smooth ${smooth} \
			--aRatio ${aRatio}
		checkAssertionStatus $?

		${SOURCEDIR}moveFiles.sh ${OUTPUTDIR} ${parentDir}
		checkAssertionStatus $?
	fi

done
