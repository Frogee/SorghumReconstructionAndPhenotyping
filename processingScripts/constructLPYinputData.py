#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import random
from collections import defaultdict
import matplotlib.pyplot as plt

PHENOLIST_FILENAME="lpyPhenotypeNamesToKeep.txt"
LEAF_FILENAME="LeafMeasurements.tsv"
li_phenotypesToKeep = []
# These classes are the "rows" of the output. Each class must be a unique
# substring of the name in "phenotypes to keep" that it groups together
li_phenotypeClasses = ["internodeDistance", "radius", "angle", "length", "width"]
li_phenotypeClassesCorrespondingRowName = ["internode_length", "internode_radius", "leaf_angle", "leaf_length", "leaf_width"]


class SampleDataContainer:
    def __init__(self, inputName):
        self.sampleName = inputName
        self.phenoDict = {}

    def printContents(self):
        sys.stdout.write(str(self.sampleName))
        sys.stdout.write("\n")
        print self.phenoDict

    def addPhenoNameAndValue(self, phenoName, phenoValue):
        self.phenoDict[phenoName] = phenoValue

# We maintain a global container because not every sample will have the same
# phenotypes
class GlobalPhenotypeNameContainer:
    def __init__(self):
        self.phenoDict = {}

    def printContents(self):
        print self.phenoDict

    def addPhenoNameAndValue(self, phenoName, phenoValue):
        self.phenoDict[phenoName] = 1

# User safety check
if len(sys.argv) < 2:
    sys.stderr.write("Insufficient arguments. Please check your input.\n")

try:
    li_phenotypesToKeep = [line.strip() for line in open(PHENOLIST_FILENAME)]
except IOError:
    sys.stderr.write("Unable to open " + PHENOLIST_FILENAME + ". Aborting.")
    sys.exit()

allPhenotypeContainer = GlobalPhenotypeNameContainer()
sampleList = []
sampleReplicateDict = defaultdict(list)

# For each input directory, get the directory's measurements for individual
# leaves and phenotypes. We also want to combine sample replicates
for directoryIndex in range(len(sys.argv)):
    if (directoryIndex == 0):
        continue
    directoryName = sys.argv[directoryIndex]
    sampleName = directoryName.split("/")[-1]
    sampleCohort = sampleName.split("_")[0]
    inputLeafFilenameString = directoryName + "/" + LEAF_FILENAME

    try:
        li_leafMeasurements = [line.strip() for line in open(inputLeafFilenameString)]
        li_leafMeasurements = [element.split('\t') for element in li_leafMeasurements]
    except IOError:
        sys.stderr.write("Unable to open " + inputLeafFilenameString + ". Skipping sample " + sampleName + "\n")
        continue

    currentSample = SampleDataContainer(sampleName)

    for columnIndex in range(len(li_leafMeasurements[0])):
        phenoName = li_leafMeasurements[0][columnIndex]
        phenoValue = li_leafMeasurements[1][columnIndex]

        allPhenotypeContainer.addPhenoNameAndValue(phenoName, phenoValue)
        currentSample.addPhenoNameAndValue(phenoName, phenoValue)

    currentSample.printContents()

    sampleList.append(currentSample)

    if (sampleCohort in sampleReplicateDict):
        sampleReplicateDict[sampleCohort].append(currentSample)
    else:
        sampleReplicateDict[sampleCohort] = [currentSample]

    # Since we don't have internode radius implemented yet, we're going to
    # repeat the stem radius for the number of internodes. To do so, we first
    # need to count the number of internodes.
    numInternodes = 0
    numLeaves = 0
    for phenotypeKey in currentSample.phenoDict:
        if ("internodeDistance" in phenotypeKey):
            numInternodes = numInternodes + 1
        if ("FixedLength" in phenotypeKey):
            numLeaves = numLeaves + 1

    outfileName = directoryName + "/lpyData.csv"
    print("Writing data to " + outfileName)
    outfile = open(outfileName, 'w')
    numTimesOutputInternodeRadius = 0
    #Set up the header
    for leafIndex in range(1, numLeaves + 1):
        outfile.write("," + str(leafIndex))
    outfile.write("\n")
    # Write the rest of the data.
    for phenotypeClassIndex in range(len(li_phenotypeClasses)):
        phenotypeClass = li_phenotypeClasses[phenotypeClassIndex]
        outfile.write(li_phenotypeClassesCorrespondingRowName[phenotypeClassIndex])
        for phenotype in li_phenotypesToKeep:
            print phenotype
            # The phenotype class must be a unique substring of the name)
            if (phenotypeClass in phenotype):
                if (phenotype in currentSample.phenoDict):
                    if (phenotype == "stem_radius" and numTimesOutputInternodeRadius >= numInternodes):
                        sys.stderr.write("Maximum number of stem_radius values output. Skipping.\n")
                        continue
                    #Convert to meters
                    try:
                        if (phenotypeClass != "angle"):
                            convertedValue = float(currentSample.phenoDict[phenotype]) / 1000.0
                        else:
                            convertedValue = currentSample.phenoDict[phenotype]
                        outfile.write("," + str(convertedValue))
                        if (phenotype == "stem_radius"):
                            numTimesOutputInternodeRadius = numTimesOutputInternodeRadius + 1
                    except TypeError:
                        sys.stderr.write("Type error. Aborting\n")
                        sys.exit()

                else:
                    sys.stderr.write(phenotype + " is not in the dictionary of " + currentSample.sampleName + "\n")
        outfile.write("\n")
    outfile.close()

'''
allPhenotypeContainer.printContents()
print sampleList

phenoList = []
for key in allPhenotypeContainer.phenoDict:
    phenoList.append(key)

phenoList.sort()

print phenoList

outputList = []
# Set up the header
headerList = []
headerList.append("\t")
for phenotype in phenoList:
    headerList.append(phenotype)

outputList.append(headerList)

for sample in sampleList:
    currentRow = []
    currentRow.append(sample.sampleName)
    for phenotype in phenoList:
        if (phenotype in sample.phenoDict):
            value = sample.phenoDict[phenotype]
        else:
            value = -9
        currentRow.append(value)

    outputList.append(currentRow)

print outputList


for rowIndex in range(len(outputList)):
    for colIndex in range(len(outputList[rowIndex])):
        outfile.write(str(outputList[rowIndex][colIndex]))
        outfile.write("\t")
    outfile.write("\n")

outfile.close()
'''
'''
# Generating plots to visualize data
#boxplots=True
boxplots=False
if (boxplots==True):
    for phenotype in phenoList:
        listData = []
        nameList = []
        sampleGroupList = []
        # Make an ordered sample group list
        for sampleGroup in sampleReplicateDict:
            sampleGroupList.append(sampleGroup)
        sampleGroupList.sort()

        for sampleGroup in sampleGroupList:
            nameList.append(sampleGroup)
            repDataList = []
            replicateList = sampleReplicateDict[sampleGroup]
            for sample in replicateList:
                if (phenotype in sample.phenoDict):
                    repDataList.append(float(sample.phenoDict[phenotype]))
            listData.append(repDataList)
        print listData
        fig, axes = plt.subplots(nrows=1, ncols=1)
        plt.setp(axes, xticklabels=nameList)
        plt.title(phenotype)
        plots = plt.scatter(listData, sym='bo')
        #plt.show()
        fileName = "./PhenoPlots/" + phenotype + ".png"
        plt.savefig(fileName, dpi=150, format="png")
        plt.close(fig)

#scatter=True
scatter=False
if (scatter==True):
    for phenotype in phenoList:
        listData = []
        nameList = []
        sampleGroupList = []
        # Make an ordered sample group list
        for sampleGroup in sampleReplicateDict:
            sampleGroupList.append(sampleGroup)
        sampleGroupList.sort()

        fig, axes = plt.subplots(nrows=1, ncols=1)


        sampleGroupNumber = 0
        for sampleGroup in sampleGroupList:
            sampleGroupNumber = sampleGroupNumber + 1
            nameList.append(sampleGroup)
            repDataList = []
            xAxis = []
            replicateList = sampleReplicateDict[sampleGroup]
            for sample in replicateList:
                if (phenotype in sample.phenoDict):
                    repDataList.append(float(sample.phenoDict[phenotype]))
                    xAxis.append(sampleGroupNumber + random.uniform(-0.05, 0.05))
            plt.scatter(xAxis, repDataList)

        plt.xticks(range(1, sampleGroupNumber + 1), nameList)
        plt.title(phenotype)
        fileName = "./PhenoPlots/" + phenotype + ".png"
        plt.savefig(fileName, dpi=150, format="png")
        plt.close(fig)

scatterRepColor=True
#scatterRepColor=False
colorList = ["#CC0000", "#009900", "#3366FF"]
if (scatterRepColor==True):
    for phenotype in phenoList:
        listData = []
        nameList = []
        sampleGroupList = []
        # Make an ordered sample group list
        for sampleGroup in sampleReplicateDict:
            sampleGroupList.append(sampleGroup)
        sampleGroupList.sort()

        fig, axes = plt.subplots(nrows=1, ncols=1)


        sampleGroupNumber = 0
        for sampleGroup in sampleGroupList:
            sampleGroupNumber = sampleGroupNumber + 1
            nameList.append(sampleGroup)
            replicateList = sampleReplicateDict[sampleGroup]
            sampleNumber = 0
            for sample in replicateList:
                sampleNumber = sampleNumber + 1
                if (phenotype in sample.phenoDict):
                    repDataList = [float(sample.phenoDict[phenotype])]
                    xAxis = [sampleGroupNumber + random.uniform(-0.05, 0.05)]
                    plt.scatter(xAxis, repDataList, c=colorList[sampleNumber - 1])

        plt.xticks(range(1, sampleGroupNumber + 1), nameList)
        plt.title(phenotype)
        fileName = "./PhenoPlots/" + phenotype + ".png"
        plt.savefig(fileName, dpi=150, format="png")
        plt.close(fig)
'''
