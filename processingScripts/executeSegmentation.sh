PROCESSINGBINARY=$3

CONFIGFILEDIR=$2

PLYFILES=$4*_PoissonReconTrimmed_smoothedDecimated_geodesicready.ply

LEARNEDFILEDIR=$5

OUTPUTDIR=$6

PLYSUFFIX="_PoissonReconTrimmed_smoothedDecimated_geodesicready.ply"
LEARNEDPREFIX="learned"
LEARNEDSUFFIX="_PoissonReconTrimmed_smoothedDecimated_geodesicready.ply"

checkAssertionStatus () {
	returnedValue=$1
	assertValue=134  # This is what is returned (POSIX I think?) on a failed assert.
	if [ $returnedValue -eq $assertValue ]
	then
		echo "Processed failed. Aborting."
		exit 1
	fi
}

for file in ${PLYFILES}
do
	fileName=$(basename ${file})
	inputPLY=${file}
	inputLearnedPLY=""
	while IFS='_' read -ra nameArray ; do
		sampleName=${nameArray[0]}
	       	sampleNumber=${nameArray[1]}
	done <<< "$fileName"

	#First, check that the directory doesn't already have a segmented mesh so that we don't need to repeat it.
	
	inputLearnedPLY="${LEARNEDFILEDIR}${LEARNEDPREFIX}${sampleName}_${sampleNumber}${LEARNEDSUFFIX}"
	individualDir="${OUTPUTDIR}${sampleName}_${sampleNumber}/"

	if [ ! -f "${individualDir}segmentedMesh.ply" ]
	then

		${PROCESSINGBINARY} \
			${CONFIGFILEDIR}segmentationParameters.xml \
			--segmentationFromPLY ${inputPLY} ${inputLearnedPLY}
		
		checkAssertionStatus $?
	
		if [ ! -d "${individualDir}" ]
			then
				mkdir ${individualDir}
			fi
	
		rm ${individualDir}segmentedMesh.ply
		rm ${individualDir}segmentedMeshPreTransformation.ply

		mv segmentation_PLYs/*ply ${individualDir}
	fi

done


