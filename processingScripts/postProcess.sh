INPUTPLY=$1
FIRSTINPUTMESHLABFILTERS=$2
SECONDINPUTMESHLABFILTERS=$3

parent="$(dirname "${INPUTPLY}")"

meshlabserver -i $INPUTPLY -o ${parent}/intermediate.ply -s ${FIRSTINPUTMESHLABFILTERS} -om vn
meshlabserver -i ${parent}/intermediate.ply -o ${parent}/combinedCloudFiltered_postProcessed.ply -s ${SECONDINPUTMESHLABFILTERS} -om vn
rm ${parent}/intermediate.ply

