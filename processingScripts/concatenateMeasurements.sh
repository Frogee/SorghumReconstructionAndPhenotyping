
SOURCEDIR=$1
CONFIGFILEDIR=$2
DIRS=$4*_[1-3]

inputString=""
for dir in ${DIRS}
do
	inputString="${inputString} $dir"
done

workingDirectory=$(pwd)
experimentSuffix=$(basename "${workingDirectory}")
experimentSuffix=${experimentSuffix%_*}
echo "Processing phenotypes for ${experimentSuffix}"
python ${SOURCEDIR}phenotypeReportGenerator.py ${experimentSuffix} ${inputString}
