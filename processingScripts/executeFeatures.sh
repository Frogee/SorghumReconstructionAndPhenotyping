PROCESSINGBINARY=$3

CONFIGFILEDIR=$2

INPUTDIR=$4

OUTPUTDIR=$5

INPUTFILES=${INPUTDIR}*geodesicready.ply

for file in ${INPUTFILES}
do
	
	outFileName=$(basename ${file})
	outFileName=${outFileName%.ply}

	if [ ! -f "${OUTPUTDIR}${outFileName}.arf" ]
	then

		time ${PROCESSINGBINARY} \
        		${CONFIGFILEDIR}featureEstimationParameters.xml \
			--computePointFeatures ${file}

		mv featuresOutputFile.arf ${OUTPUTDIR}${outFileName}.arf
	fi

done


INPUTFILES=${INPUTDIR}*geodesicready_labeled.ply

for file in ${INPUTFILES}
do
	outFileName=$(basename ${file})
	outFileName=${outFileName%.ply}

	if [ ! -f "${OUTPUTDIR}${outFileName}.arf" ]
	then

		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}featureEstimationParameters.xml \
			--computePointFeatures ${file}

		mv featuresOutputFile.arf ${OUTPUTDIR}${outFileName}.arf
	fi

done
