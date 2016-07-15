PROCESSINGBINARY=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/DepthCameraPhenotyping_Prototype4/bin/Debug/DepthCameraPhenotyping_v4

CONFIGFILEDIR=/home/rfm_node03/Documents/Analyses/ImageBasedPhenotyping/BTx623xIS3620C_Fall2015/ImageAnalysis/processingConfigFiles/BTx623xIS3620C_08-03/

INPUTDIR=/home/rfm_node03/Documents/Analyses/ImageBasedPhenotyping/BTx623xIS3620C_Fall2015/ImageAnalysis/ProcessedData/BTx623xIS3620C_08-03_ProcessedClouds_Debug/

OUTPUTDIR=/home/rfm_node03/Documents/Analyses/ImageBasedPhenotyping/BTx623xIS3620C_Fall2015/ImageAnalysis/ProcessedData/BTx623xIS3620C_08-03_ProcessedClouds_Debug/AdaBoostLearning/

INPUTFILES=${INPUTDIR}*geodesicready.ply

for file in ${INPUTFILES}
do
	
	outFileName=$(basename ${file})
	outFileName=${outFileName%.ply}

	time ${PROCESSINGBINARY} \
        	${CONFIGFILEDIR}featureEstimationParameters.xml \
		--computePointFeatures ${file}

	mv featuresOutputFile.arf ${OUTPUTDIR}${outFileName}.arf

done


INPUTFILES=${INPUTDIR}*geodesicready_labeled.ply

for file in ${INPUTFILES}
do
	outFileName=$(basename ${file})
	outFileName=${outFileName%.ply}

	time ${PROCESSINGBINARY} \
		${CONFIGFILEDIR}featureEstimationParameters.xml \
		--computePointFeatures ${file}

	mv featuresOutputFile.arf ${OUTPUTDIR}${outFileName}.arf

done
