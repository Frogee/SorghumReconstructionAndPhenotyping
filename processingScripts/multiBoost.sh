SCRIPTDIR=$1

MULTIBOOSTBIN=$2

INPUTMESHDIR=$3

OUTPUTFILEDIR=$4

if [ ! -f "${OUTPUTFILEDIR}combinedFeaturesToTrainWith.arf" ]
then
	FILES=${OUTPUTFILEDIR}*_labeled.arf
	bool_firstFile="True"
	for file in ${FILES}
	do

		if [ $bool_firstFile = "True" ]
		then	
			grep "^%\|^@\|^$\|^[[:space:]]*$" $file >> ${OUTPUTFILEDIR}combinedFeaturesToTrainWith.arf
			bool_firstFile="False"
		fi
		# The training file was getting pretty monsterous with ~ 100 meshes. Since the unlabeled points typically outnumber
		# the stem points by a factor of 10, we'll just take every 10 unlabeled points.
		grep -v "^%" $file | grep -v "^@" | grep -v "^$" | grep -v "^[[:space:]]*$" | awk -F',' \
			'BEGIN { unlabeledCounter = 0 } {
				if ($NF == "UNLABELED") {
					unlabeledCounter = unlabeledCounter + 1
					if (unlabeledCounter == 10) {
			 			print $0
						unlabeledCounter = 0
					}
				}
				else {
					print $0
				}
			}' >> ${OUTPUTFILEDIR}combinedFeaturesToTrainWith.arf
		# This would write all of the features to a file.
		#grep -v "^%" $file | grep -v "^@" | grep -v "^$" | grep -v "^[[:space:]]*$" >> ${OUTPUTFILEDIR}combinedFeaturesToTrainWith.arf
	done
fi

if [ ! -f "${OUTPUTFILEDIR}modelOut.xml" ]
then
	${MULTIBOOSTBIN} \
		--fileformat arff \
		--stronglearner AdaBoostMH \
		--train ${OUTPUTFILEDIR}combinedFeaturesToTrainWith.arf 60 \
		--shypname ${OUTPUTFILEDIR}modelOut.xml \
		--verbose 3 \
		--outputinfo ${OUTPUTFILEDIR}outTest.dta
fi

echo
echo
echo

echo "Learning points and converting learned files to PLYs."
FILES=${OUTPUTFILEDIR}*_geodesicready.arf
for file in ${FILES}
do
	filePathWithoutExtension=${file%.*}
	fileNameWithoutExtensionAndPath=${filePathWithoutExtension##*/}

	if [ ! -f "${filePathWithoutExtension}.dta" ]
	then
		${MULTIBOOSTBIN} \
			--stronglearner AdaBoostMH \
			--fileformat arff \
			--test $file ${OUTPUTFILEDIR}modelOut.xml 60 ${filePathWithoutExtension}.dta
	fi

	if [ ! -f "${OUTPUTFILEDIR}learned${fileNameWithoutExtensionAndPath}.ply" ]
	then
		python ${SCRIPTDIR}convertARFFandDTAToPLY.py ${file} ${filePathWithoutExtension}.dta > ${OUTPUTFILEDIR}learned${fileNameWithoutExtensionAndPath}.ply
	fi
done

echo "Converting labeled feature files to PLYs."
FILES=${OUTPUTFILEDIR}*_geodesicready_labeled.arf
for file in ${FILES}
do
	filePathWithoutExtension=${file%.*}
	fileNameWithoutExtensionAndPath=${filePathWithoutExtension##*/}
	fileNameStripLabeled=${fileNameWithoutExtensionAndPath%_labeled}
	python ${SCRIPTDIR}convertLearnedARFFtoPLY.py ${file} > ${OUTPUTFILEDIR}learned${fileNameStripLabeled}.ply
done

