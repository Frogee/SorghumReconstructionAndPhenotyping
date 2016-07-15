OUTPUTDIR=$3
PLYFILES=$3*PoissonReconTrimmed.ply
MESHLABSCRIPTDIR=$2

# First check the output directory for already processed samples (e.g. if need to restart after a computer lock up) 
PROCESSEDSAMPLELIST=()
processedSamples=${OUTPUTDIR}*_smoothedDecimated_geodesicready.ply
for file in ${processedSamples}
do
	strippedPath=${file##*/}
	PROCESSEDSAMPLELIST+=($sampleName) 
	sampleName=${strippedPath%_PoissonReconTrimmed_smoothedDecimated_geodesicready.ply}
	PROCESSEDSAMPLELIST+=($sampleName)
done

for file in $PLYFILES
do
	strippedPath=${file##*/}                        
	sampleName=${strippedPath%_PoissonReconTrimmed.ply} 
        bool_sampleAlreadyProcessed="false"
	for sampleNameFromList in ${PROCESSEDSAMPLELIST[@]}
	do
		if [ ${sampleNameFromList} = ${sampleName} ]
		then
			bool_sampleAlreadyProcessed="true"
			continue
		fi
	done

        if [ ${bool_sampleAlreadyProcessed} = "false" ]
	then

		filePrefix="${strippedPath%.*}"
		meshlabserver -i $file -o ${OUTPUTDIR}${filePrefix}_smoothedDecimated.ply -s ${MESHLABSCRIPTDIR}MLSsmoothing_decimation.mlx -om vn
		meshlabserver -i ${OUTPUTDIR}${filePrefix}_smoothedDecimated.ply -o ${OUTPUTDIR}${filePrefix}_smoothedDecimated_geodesicready.ply -s ${MESHLABSCRIPTDIR}prepareForGeodesicDistance.mlx -om vc vn
	fi
done

									
