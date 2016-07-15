
PROCESSINGBINARY=$3

CONFIGFILEDIR=$2

MESHDIRS=$4*_[1-3]

SCREENSHOTDIR=$5

for dir in ${MESHDIRS[@]}
do
	echo ${dir}
	sampleName=${dir##*/}
	inputString=""
	inputPLY=${dir}/segmentedMesh.ply
	time ${PROCESSINGBINARY} \
		${CONFIGFILEDIR}cameraParameters.xml \
		--screenshotPLY $inputPLY ${SCREENSHOTDIR}${sampleName}.png

done
