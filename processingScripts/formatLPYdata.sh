DIRS=./*_[1-3]

inputString=""
for dir in ${DIRS}
do
	inputString="${inputString} $dir"
done
#echo ${inputString}
python constructLPYinputData.py ${inputString}
