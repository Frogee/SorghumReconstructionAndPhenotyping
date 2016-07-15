CONFIGFILEDIR=$2
MESHDIRS=$4*_[1-3]

for dir in ${MESHDIRS}
do
	rm ${dir}/*scaled.ply

	plyFiles=${dir}/*.ply
	for file in ${plyFiles}
	do
		fileprefix="${file%.*}"
		meshlabserver -i ${file} -o ${fileprefix}_scaled.ply -s ${CONFIGFILEDIR}ScaleMesh.mlx -om vc vn
	done
done
