CONFIGFILEDIR=$2

PROCESSINGBINARY=$3

MESHDIRS=$4/*_[1-3]


for dir in ${MESHDIRS[@]}
do
	#First, check that the measurements file doesn't already exist so we don't need to repeat it.
	if [ ! -f "${dir}/PlantMeasurements.tsv" ]
	then	
	
		rm ${dir}/PlantMeasurements.tsv

		echo ${dir}

		inputString=""
		inputPLY=${dir}/segmentedMesh.ply
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}measurementParameters.xml \
			--makePlantMeasurements $inputPLY
	
		mv segmentation_PLYs/* $dir/
		mv PlantMeasurements.tsv $dir/
	fi
done



