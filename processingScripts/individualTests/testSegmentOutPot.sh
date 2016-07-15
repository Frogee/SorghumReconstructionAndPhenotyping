# Launches the depth image processing pipeline

# Path to the image processing binary
PROCESSINGBINARY=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/DepthCameraPhenotyping_Prototype4/bin/Debug/DepthCameraPhenotyping_v4
# Path to the directory containing the two Screened Poisson Surface Reconstruction binaries
# http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version7.0/
POISSONBINARYDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/PoissonRecon/Bin/Linux/

# Path to the directory containing the configuration files used for processing
CONFIGFILEDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite/processingConfigFiles/devTest/

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
			--segmentOutPotPrototype $inputCloudString
	
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


		combinedCloudDir=./sample_registeredClouds/
		combinedCloud=./sample_registeredClouds/combinedCloud.pcd
		
		# Filter the registered cloud.
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}postRegistrationFilteringParameters.xml \
			--removeStatisticalOutliers $combinedCloud
	
		# Perform some additional post registration filtering.
		cd $combinedCloudDir
		./postProcess.sh combinedCloudFiltered.ply \
			${CONFIGFILEDIR}postRegistration_preMeshing_processing_step1.mlx \
			${CONFIGFILEDIR}postRegistration_preMeshing_processing_step2.mlx

		# Use Screened Poisson Surface Reconstruction to mesh the cloud.
		inputPly=combinedCloudFiltered_postProcessed.ply
		#--depth had previously been set to 7; trying to set to 8 to prevent smaller leaves from getting cut out.
		echo
		echo "Constructing surface"
		echo

		${POISSONBINARYDIR}PoissonRecon --in $inputPly --out PoissonReconout.ply --density \
			--threads 4 \
			--depth 7 \
			--fullDepth 5 \
			--voxelDepth 8 \
			--samplesPerNode 1.0 \
			--pointWeight 6 #Values between 6 and 10 seem to work pretty well

		echo
		echo "Trimming surface"
		echo

		${POISSONBINARYDIR}SurfaceTrimmer --in PoissonReconout.ply --out PoissonReconTrimmed.ply \
			--trim 7 \
			--smooth 5 \
			--aRatio 0.001

		cd ..

		./processingScripts/moveFiles.sh ${OUTPUTDIR} ${parentDir} 
	fi

done
