#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import random
from collections import defaultdict
import matplotlib.pyplot as plt
import scipy.stats as sp
import math
import numpy as np

MEASUREMENT_FILENAME="PlantMeasurements.tsv"
ORDERED_SAMPLE_IDS_FILENAME="OrderedSampleIDs.tsv"
NUM_REPLICATES = 3
MISSING_DATA = "-"

# User safety check
if len(sys.argv) < 3:
    sys.stderr.write("Insufficient arguments. Please check your input.\n")
    
EXPERIMENT_NAME=sys.argv[1]

class SampleDataContainer:
#Contains data for each rep of a sample
    def __init__(self, inputName):
        self.sampleName = inputName
        self.phenoDict = defaultdict(list)

    def printContents(self):
        sys.stdout.write(str(self.sampleName))
        sys.stdout.write("\n")
        print self.phenoDict

    def addPhenoValue(self, phenoName, phenoValue, replicateNumber):
        if (phenoName in self.phenoDict):
             self.phenoDict[phenoName][replicateNumber - 1] = phenoValue
        else:
            for numReps in range(1, NUM_REPLICATES + 1):
                self.phenoDict[phenoName].append(MISSING_DATA)
            self.phenoDict[phenoName][replicateNumber - 1] = phenoValue

class GlobalSampleIDContainer:
    def __init__(self):
        self.sampleDict = {}

    def printContents(self):
        print self.sampleDict

    def addSampleID(self, sampleID):
        self.sampleDict[sampleID] = 1

    def addSample(self, sampleID, sample):
        self.sampleDict[sampleID] = sample

# We maintain a global container because not every sample will have the same
# phenotypes
class GlobalPhenotypeNameContainer:
    def __init__(self):
        self.phenoDict = {}

    def printContents(self):
        print self.phenoDict

    def addPhenoNameAndValue(self, phenoName, phenoValue):
        self.phenoDict[phenoName] = 1

def AddTransformedColumns(inputList):
    li_originalIDorder = []
    d_IDsToPhenos = defaultdict(list)

    for i in range(len(inputList)):
        str_ID = str(inputList[i][0])
        li_originalIDorder.append(str_ID)
        d_IDsToPhenos[str_ID] = []

    for col_index in range(len(inputList[0])):
        d_PhenosToTransform = defaultdict(float)
        d_TransformedPhenos = defaultdict(float)
        if col_index < 1:
            continue
        str_phenoName = "NULL"
        for row_index in range(len(inputList)):
            key = li_originalIDorder[row_index]
            if row_index == 0:
                str_phenoName = inputList[row_index][col_index]
                d_IDsToPhenos[key].append(str_phenoName)
                continue
            try:
                d_IDsToPhenos[key].append(inputList[row_index][col_index])
            except IndexError:
                sys.stderr.write("Index error at row_index " + str(row_index) + " and col_index " + str(col_index) + "\n")
            if inputList[row_index][col_index] == MISSING_DATA:
                continue
            try:
                float(inputList[row_index][col_index])
            except ValueError:
                sys.stderr.write(inputList[row_index][col_index] + " at row_index " + str(row_index) + " and col_index " + str(col_index) + " is not a valid numeric value.\n")
                continue
            d_PhenosToTransform[key] = float(inputList[row_index][col_index])

        li_x = []
        key_list = []
        for key in d_PhenosToTransform:
            key_list.append(key)
            li_x.append(d_PhenosToTransform[key])

        if len(li_x) == 0:
            continue
        na_x = np.array(li_x)

#Normalize li_x
    #The Empirical Normal Quantile Transformation (ENQT) as described in Peng et al. 2007
    #"Normalizing a large number of quantitative traits using empirical normal quantile transformation"
    #The actual transform can be found at:
    #www.mayo.edu/research/documents/biostat-78pdf/doc-10027528
    #"Estimating genetic components of variants for quantitative traits in family studies using
    #the MULTIC routines"

    #This routine found at stackoverflow.com/questions/5284646/rank-items-in-an-array-using-python-numpy
        na_rankIndices = na_x.argsort()  #Get the indices of elements as they would be sorted
        na_xRanks = np.empty(len(na_x), float)
        na_xRanks[na_rankIndices] = np.arange(len(na_x))   #Get the element rankings

        for i in range(len(na_xRanks)):
            na_xRanks[i] = na_xRanks[i] + 1

    #y_i = phi^-1 * (r_i / 1 + total observations) where phi^-1 is the inverse of the cumulative
    #function of the standard normal distribution
        na_xNormed = np.empty(len(na_x), float)

    #This routine found at stackoverflow.com/questions/20626994/how-to-calculate-inverse-normal-distribution-in-python

        for i in range(len(na_xRanks)):
            na_xNormed[i] = sp.norm.ppf(na_xRanks[i] / float((1 + len(na_xRanks))))

        for i in range(len(key_list)):
            d_TransformedPhenos[key_list[i]] = na_xNormed[i]

        for i in range(len(li_originalIDorder)):
            key = li_originalIDorder[i]
            if i == 0:
                d_IDsToPhenos[key].append(str_phenoName + "_Normalized")
            elif key in d_TransformedPhenos:
                d_IDsToPhenos[key].append(str(d_TransformedPhenos[key]))
            else:
                d_IDsToPhenos[key].append(MISSING_DATA)

    listToReturn = []
    for i in range(len(li_originalIDorder)):
        listToAdd = []
        key = li_originalIDorder[i]
        IDstring = key
        listToAdd.append(IDstring)
        for value in d_IDsToPhenos[key]:
            listToAdd.append(value)
        listToReturn.append(listToAdd)
    return listToReturn
## End def AddTransformedColumns

allPhenotypeContainer = GlobalPhenotypeNameContainer()
#contains all possible sample IDs
allSampleIDsContainer = GlobalSampleIDContainer()
#contains sample IDs that have data
availableSampleData = GlobalSampleIDContainer()
sampleList = []
sampleReplicateDict = defaultdict(list)

# First, get the list of all samples in the population so that we can output
# the data even for missing samples.

print("Generating report for plant measurements. This may take a while.")

try:
    li_sampleIDs = [line.strip() for line in open(ORDERED_SAMPLE_IDS_FILENAME)]
except IOError:
    sys.stderr.write("Unable to open file " + ORDERED_SAMPLE_IDS_FILENAME + ". Make sure that it exists and contains Sample IDs.\n")

for ID in li_sampleIDs:
    allSampleIDsContainer.addSampleID(ID)

# For each input directory, get the directory's measurements for individual
# leaves and phenotypes. We also want to combine sample replicates
for directoryIndex in range(len(sys.argv)):
    # We skip the first and second arguments of the argv. Arg 0 is the program name,
    # and arg 2 contains the experiment name designation.    
    if (directoryIndex == 0):
        continue
    if (directoryIndex == 1):
        continue
    directoryName = sys.argv[directoryIndex]
    sampleRepName = directoryName.split("/")[-1]
    sampleCohort = sampleRepName.split("_")[0]
    sampleRepNumber = sampleRepName.split("_")[1]

    inputMeasurementFilenameString = directoryName + "/" + MEASUREMENT_FILENAME

    try:
        li_plantMeasurements = [line.strip() for line in open(inputMeasurementFilenameString)]
        li_plantMeasurements = [element.split('\t') for element in li_plantMeasurements]

    except IOError:
        sys.stderr.write("Unable to open a file. Skipping sample " + sampleRepName + "\n")
        continue

    #If we already have a replicate from the same cohort in the availableSampleData
    if (sampleCohort in availableSampleData.sampleDict):
        #We can use the available container
        currentSample = availableSampleData.sampleDict[sampleCohort]
    #If not, we need to make a new container
    else:
        currentSample = SampleDataContainer(sampleCohort)
    #print(li_wholePlantMeasurements)

    for rowIndex in range(len(li_plantMeasurements)):
        phenoName = EXPERIMENT_NAME + "_" + li_plantMeasurements[rowIndex][0]
        phenoValue = li_plantMeasurements[rowIndex][1]
        
        allPhenotypeContainer.addPhenoNameAndValue(phenoName, phenoValue)
        currentSample.addPhenoValue(phenoName, phenoValue, int(sampleRepNumber))
        
    #currentSample.printContents()
    #If we already have a replicate from the same cohort in the availableSampleData
    if (sampleCohort in availableSampleData.sampleDict):
        #We can update the sample container
        availableSampleData.sampleDict[sampleCohort] = currentSample
    #If not, we add the new container
    else:
        availableSampleData.addSample(sampleCohort, currentSample)

#allPhenotypeContainer.printContents()
#print sampleList

phenoList = []
for key in allPhenotypeContainer.phenoDict:
    phenoList.append(key)

phenoList.sort()

#print phenoList

outputList = []
# Set up the header
headerList = []
headerList.append("SampleName")
for phenotype in phenoList:
    #Each phenotype gets the a replicate column and an average column
    for repNumber in range(1, NUM_REPLICATES + 1):
        headerList.append(phenotype + "_rep" + str(repNumber))
    #the average column
    headerList.append(phenotype + "_Average")

outputList.append(headerList)

# For each of the sample IDs in the global list
for sampleID in li_sampleIDs:
    currentRow = []
    currentRow.append(sampleID)

    #Check if the sample from the global list has a corresponding ID in phenotyped samples
    bool_localSampleFound = False
    if sampleID in availableSampleData.sampleDict:
        bool_localSampleFound = True
        #Then output available reps for each phenotype
        for phenotype in phenoList:
            #For each possible replicate
            repAverageList = []
            for repNumber in range(1, NUM_REPLICATES + 1):
                sample = availableSampleData.sampleDict[sampleID]
                if (phenotype in sample.phenoDict):
                    value = sample.phenoDict[phenotype][repNumber - 1]
                    currentRow.append(value)
                else:
                    value = MISSING_DATA
                    currentRow.append(value)
                if (value != MISSING_DATA):
                    repAverageList.append(float(value))
            #If there are some data in the rep average list, average it and add it
            if (len(repAverageList) != 0):
                currentRow.append(str(np.mean(repAverageList)))
            #Else add it as missing.
            else:
                currentRow.append(MISSING_DATA)
        #End for each phenotype

    if bool_localSampleFound == False:
        for phenotype in phenoList:
            #For each possible replicate
            for repNumber in range(1, NUM_REPLICATES + 1):
                value = MISSING_DATA
                currentRow.append(value)
            #and for the average value.
            value = MISSING_DATA
            currentRow.append(value)


    outputList.append(currentRow)

#print outputList

outputList = AddTransformedColumns(outputList)

outfile = open("phenotypeMeasurementsReport.tsv", 'w')

for rowIndex in range(len(outputList)):
    for colIndex in range(len(outputList[rowIndex])):
        outfile.write(str(outputList[rowIndex][colIndex]))
        outfile.write("\t")
    outfile.write("\n")

outfile.close()

outfile = open("phenotypeMeasurementsReport.tsvr", 'w')

for colIndex in range(len(outputList[0])):
    for rowIndex in range(len(outputList)):
        outfile.write(str(outputList[rowIndex][colIndex]))
        outfile.write("\t")
    outfile.write("\n")

outfile.close()
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
        for sampleGroupName in availableSampleData.sampleDict:
            sampleGroupList.append(sampleGroupName)
        sampleGroupList.sort()

        fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(18,8))

        sampleGroupNumber = 0
        for sampleGroup in sampleGroupList:
            nameList.append(sampleGroup)
            sampleGroupNumber = sampleGroupNumber + 1
            sample = availableSampleData.sampleDict[sampleGroup]
            if (phenotype in sample.phenoDict):
                repDataList = sample.phenoDict[phenotype]
                cleanedData = []
                xAxis = []
                for repNumber in range(1, NUM_REPLICATES + 1):
                    if (repDataList[repNumber - 1] != MISSING_DATA):
                        xAxis = [sampleGroupNumber + random.uniform(-0.02, 0.02)]
                        plt.scatter(xAxis, [repDataList[repNumber - 1]], c=colorList[repNumber - 1])
			
        plt.xticks(range(1, sampleGroupNumber + 1), nameList, rotation='vertical')
	plt.xlim(-1.0, sampleGroupNumber + 1)
	ylims = axes.get_ylim()
	sampleGroupNumber = 0
	for sampleGroup in sampleGroupList:
		sampleGroupNumber = sampleGroupNumber + 1
		plt.plot([sampleGroupNumber, sampleGroupNumber], [ylims[0], ylims[1]], color='black', alpha=0.2, linewidth=1.0)
        plt.ylim(ylims[0], ylims[1])
	plt.title(phenotype)
        fileName = "./PhenoPlots/" + phenotype + ".png"
        plt.savefig(fileName, dpi=150, format="png")
        plt.close(fig)
