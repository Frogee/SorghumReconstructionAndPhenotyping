
PROCESSINGBINARY=$3

CONFIGFILEDIR=$2

MESHDIR=$4

SCREENSHOTDIR=$5

plyFiles=${MESHDIR}*_geodesicready.ply

for file in ${plyFiles}
do
	sampleNameStripPath=${file##*/}
	sampleName=${sampleNameStripPath%%.*}
	inputPLY=$file
	time ${PROCESSINGBINARY} \
		${CONFIGFILEDIR}cameraParameters.xml \
		--screenshotPLY $inputPLY ${SCREENSHOTDIR}${sampleName}.png
done
