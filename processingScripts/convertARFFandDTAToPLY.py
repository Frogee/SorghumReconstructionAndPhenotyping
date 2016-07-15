#dta file format
'''
Instance	Forecast	Labels
0	WHORL	| WHORL
1	WHORL	| WHORL
2	LEAF	| LEAF
3	LEAF	| LEAF
4	WHORL	| WHORL
5	LEAF	| LEAF
6	LEAF	| LEAF
7	LEAF	| LEAF
8	WHORL	| WHORL
'''

#arff file format
'''
%.arff format features.

@RELATION pointLabels

@ATTRIBUTE X NUMERIC
@ATTRIBUTE Y NUMERIC
@ATTRIBUTE Z NUMERIC
@ATTRIBUTE FeatureBin1 NUMERIC
@ATTRIBUTE FeatureBin2 NUMERIC
@ATTRIBUTE FeatureBin3 NUMERIC
@ATTRIBUTE FeatureBin4 NUMERIC
@ATTRIBUTE FeatureBin5 NUMERIC
@ATTRIBUTE FeatureBin6 NUMERIC
@ATTRIBUTE FeatureBin7 NUMERIC
@ATTRIBUTE FeatureBin8 NUMERIC
@ATTRIBUTE FeatureBin9 NUMERIC
@ATTRIBUTE FeatureBin10 NUMERIC
@ATTRIBUTE FeatureBin11 NUMERIC
@ATTRIBUTE FeatureBin12 NUMERIC
@ATTRIBUTE FeatureBin13 NUMERIC
@ATTRIBUTE FeatureBin14 NUMERIC
@ATTRIBUTE FeatureBin15 NUMERIC
@ATTRIBUTE FeatureBin16 NUMERIC
@ATTRIBUTE FeatureBin17 NUMERIC
@ATTRIBUTE FeatureBin18 NUMERIC
@ATTRIBUTE FeatureBin19 NUMERIC
@ATTRIBUTE FeatureBin20 NUMERIC
@ATTRIBUTE FeatureBin21 NUMERIC
@ATTRIBUTE FeatureBin22 NUMERIC
@ATTRIBUTE FeatureBin23 NUMERIC
@ATTRIBUTE FeatureBin24 NUMERIC
@ATTRIBUTE FeatureBin25 NUMERIC
@ATTRIBUTE FeatureBin26 NUMERIC
@ATTRIBUTE FeatureBin27 NUMERIC
@ATTRIBUTE FeatureBin28 NUMERIC
@ATTRIBUTE FeatureBin29 NUMERIC
@ATTRIBUTE FeatureBin30 NUMERIC
@ATTRIBUTE FeatureBin31 NUMERIC
@ATTRIBUTE FeatureBin32 NUMERIC
@ATTRIBUTE FeatureBin33 NUMERIC
@ATTRIBUTE class {STEM,LEAF,WHORL}

@DATA
47.1422,-58.4355,999.534,5.86012,5.18631,5.39768,7.67375,13.8219,22.3032,14.1048,7.87964,5.12095,5.72019,6.93142,13.5678,9.60446,8.17607,7.54443,7.31408,7.05061,6.89519,7.28395,8.07167,9.53288,14.9589,9.43509,11.1556,12.3511,9.74161,5.18773,1.37486,5.4773,10.2102,13.0633,12.4818,9.52145,WHORL
117.792,-17.2061,940.242,5.32474,4.58169,5.84104,9.18944,14.6061,22.2107,14.8972,8.67985,5.40611,4.38359,4.87952,12.9829,9.2318,7.86322,7.02808,6.56975,6.60862,7.02677,7.51693,8.72636,10.8358,15.6098,12.0122,12.1892,11.4125,8.85525,4.95931,1.23686,4.7126,8.69901,11.3365,12.0094,12.5771,WHORL
'''

'''
ply
format ascii 1.0
comment VCGLIB generated
element vertex 16870
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
property uchar alpha
end_header
53.1911 9.8178 968.982 0 0 255 255
13.3387 185.977 943.944 255 0 0 255
'''

import sys

def usage():
    sys.stderr.write("\tconvertARFFandDTAtoPCD.py input.arff input.dta\n")
    sys.stderr.write("\tExample usage: convertARFFandDTAtoPCD.py features.arff predictions.dta\n")
    sys.exit()


try:
    arffFile = open(sys.argv[1])
    dtaFile = open(sys.argv[2])
except IndexError:  # Check if arguments were given
    sys.stderr.write("No arguments received. Please check your input:\n")
    usage()
except IOError:  # Check if file is unabled to be opened.
    sys.stderr.write("Cannot open target file. Please check your input:\n")
    usage()

def returnPointXYZbyRow():
    pointXYZbyRow = []
    numAttributes = 0
    Xindex = 0
    Yindex = 0
    Zindex = 0
    for line in arffFile:
        if (line.find("@ATTRIBUTE") >= 0):
            if (line.find("@ATTRIBUTE X") >= 0):
                Xindex = numAttributes
            elif (line.find("@ATTRIBUTE Y") >= 0):
                Yindex = numAttributes
            elif (line.find("@ATTRIBUTE Z") >= 0):
                Zindex = numAttributes
            numAttributes = numAttributes + 1
        if (line.find("@") < 0 and line.find("&") < 0):
            featureVector = line.split(",")
            if (len(featureVector) > 1):
                pointXYZ = (featureVector[Xindex], featureVector[Yindex], featureVector[Zindex])
                pointXYZbyRow.append(pointXYZ)
    return pointXYZbyRow

def returnLabelByRow():
    labelByRow = []
    for line in dtaFile:
        if (line.find("Instance") >= 0):
            headerRow = True
        else:
            classificationList = line.split("\t")
            if (len(classificationList) > 1):
                classification = classificationList[1]
                labelByRow.append(classification)

    return labelByRow

def printPLYheader(numberOfPoints):
    sys.stdout.write("ply\n")
    sys.stdout.write("format ascii 1.0\n")
    sys.stdout.write("comment constructed from MultiBoost output\n")
    sys.stdout.write("element vertex " + str(numberOfPoints) + "\n")
    sys.stdout.write("property float x\n")
    sys.stdout.write("property float y\n")
    sys.stdout.write("property float z\n")
    sys.stdout.write("property uchar red\n")
    sys.stdout.write("property uchar green\n")
    sys.stdout.write("property uchar blue\n")
    sys.stdout.write("property uchar alpha\n")
    sys.stdout.write("end_header\n")

pointXYZlist = returnPointXYZbyRow()
labelList = returnLabelByRow()

if (len(pointXYZlist) != len(labelList)):
    sys.stderr.write("The number of points and the number of labels do not match.\n")
    sys.stderr.write("Number labels: " + str(len(labelList)) + "\t Number points: " + str(len(pointXYZlist)) + "\n")
    sys.exit()


printPLYheader(len(pointXYZlist))
for i in range(len(pointXYZlist)):
    pointXYZ = pointXYZlist[i]
    sys.stdout.write(pointXYZ[0] + " " + pointXYZ[1] + " " + pointXYZ[2] + " ")
    if (labelList[i] == "STEM"):
        sys.stdout.write("0 255 255 255\n")
    elif (labelList[i] == "INFLORESCENCE"):
        sys.stdout.write("255 215 0 255\n")
    elif (labelList[i] == "LEAF"):
        sys.stdout.write("0 255 0 255\n")
    elif (labelList[i] == "WHORL"):
        sys.stdout.write("0 0 255 255\n")
    elif (labelList[i] == "JUNCTION"):
        sys.stdout.write("0 255 255 255\n")
    elif (labelList[i] == "TIP"):
        sys.stdout.write("255 255 0 255\n")
    else:
        sys.stdout.write("255 255 255 255\n")
