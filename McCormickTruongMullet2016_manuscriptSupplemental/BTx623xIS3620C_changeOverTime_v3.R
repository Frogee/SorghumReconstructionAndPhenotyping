# R/qtl QTL mapping for image based phenotyping
# Written by Sandra Truong and Ryan McCormick - 02/01/16
#   Latest update: 06/03/16
# The purpose of this script is for generating QTL maps with respect to time, and for doing
#   some post-hoc analyses of some of the phenotypes.
# This assumes the existence of an "overTimePlots" directory in the working directory.
# This assumes that permutations have already been run and the scanone object saved to a "permutations25000_RObject" file.
# There is some code in here that doesn't appear to do anything; it remains there as a reminder to myself
#   for how to interact with R data structures.

#
# Global variables
#
NULL_VALUE = -9999999
NULL_STRING = "NOT_APPLICABLE"
TIMEPOINTONESTEM = "X08.03"
TIMEPOINTTWOSTEM = "X08.10"
TIMEPOINTTHREESTEM = "X08.15"
TIMEPOINTFOURSTEM = "X08.20"
FIRSTCOLOR = "#1B325F"
SECONDCOLOR = "#F26C4F"
QTLOVERTIMEWIDTH = 20
QTLOVERTIMEHEIGHT= 8

#
# Utility functions 
#

# To be used with an apply function to parse the phenotype string names (including how R sanitizes them).
splitPhenotypeIDsToDaysAndName = function(rowElement) { 
  valueToReturn = NULL_VALUE
  nameToReturn = NULL_STRING
  vectorToReturn = c(valueToReturn, nameToReturn)
  
  v_splitRow = strsplit(toString(rowElement), "[_]")  # I think this returns a list? and all lists are vectors?
  dateUnconverted = v_splitRow[[1]][1]  # Because it returns a list, we have to access the list element first, then the character vector.
  # The dates have an X added and the "-" converted to a "." upon being read into R.
  # These correspond to specific days after planting, and are assigned accordingly.
  if (dateUnconverted == TIMEPOINTONESTEM) {
    valueToReturn = 27
  }
  else if (dateUnconverted == TIMEPOINTTWOSTEM) {
    valueToReturn = 34
  }
  else if (dateUnconverted == TIMEPOINTTHREESTEM) {
    valueToReturn = 39
  }
  else if (dateUnconverted == TIMEPOINTFOURSTEM) {
    valueToReturn = 44
  }
  else {
    valueToReturn = NULL_VALUE
  }
  
  if (valueToReturn == NULL_VALUE) {
    nameToReturn = NULL_STRING
  }
  else {
    nameVector = v_splitRow[[1]][-1]
    nameToReturn = paste(nameVector, collapse="_")
  }
  
  vectorToReturn = c(valueToReturn, nameToReturn)
  return(vectorToReturn)
}

# http://stackoverflow.com/questions/8197559/emulate-ggplot2-default-color-palette
# function to recreate the ggplot color palette; credit goes to stack overflow
gg_color_hue <- function(n) {
  hues = seq(15, 375, length=n+1)
  hcl(h=hues, l=65, c=100)[1:n]
}

#
# Primary script
#

#Change these file paths to reflect your system:
input_file_directory = file.path("./")
input_file_name = file.path("./BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_allPhenotypes.csv")
baseWorkingDirectory = "/home/rfm_node03/Documents/Analyses/RStudioProjects/RIG/Sbi3/BTx623xIS3620C/QTLmapping/ImageBasedPhenotyping/BTx623xIS3620C_v5_1"
individualPhenoQTLimagePath = "/home/rfm_node03/Documents/Analyses/RStudioProjects/RIG/Sbi3/BTx623xIS3620C/QTLmapping/ImageBasedPhenotyping/BTx623xIS3620C_v5_1/overTimePlots"

options(warn=1)
library(qtl)
library(qtlcharts)
library(reshape2)
library(ggplot2)
library(scales)

# These permutations were calculated for 25,000 permutations for 97 normalized phenotypes of 398 samples and 10787 markers as RIL
load("permutations25000_RObject")
alpha_value = 0.05
permutationThreshold = c(summary(scanone_filteredcross_perm, alpha=alpha_value))

# Choose the distance (in cM) to thin out. This is necessary even with the pruned dataset; just set to 0.8.
#   For some reason, if this step is not done, ggplot tries to allocate 10+ Tb of memory and
#   crashes. It seems to be related to having two markers very close to each other (e.g. < 0.000001 cM apart); maybe it
#   causes the geom_raster from ggplot problems. Even if set to too low of a value, ggplot still has problems rendering
#   and messes up dislaying the density of the lines. 0.8 seems to be a good balance.
 marker_distance = 0.8

## Begin script


#Set the working directory so files are output to correct location.
setwd(file.path(baseWorkingDirectory))

# Read cross in, convert it to RIL. Here we pull out genotype groupings after unning sim.geno.
#   We read in the cross a second time below since there appears to be some interaction
#   with the plotting such that the plotting crashes if its run on the cross that has had sim.geno run on it.
cross_inputcross = read.cross("csvr", input_file_directory, input_file_name, genotypes=c("AA","AB","BB","D","C"))
# http://kbroman.org/qtlcharts/assets/vignettes/userGuide.html
# Pull out genotypes for specific markers. This needs to be run prior to dropping markers since it could get dropped.
cross_filteredcross = convert2riself(cross_inputcross)
cross_filteredcross = jittermap(cross_filteredcross)
cross_filteredcross = calc.genoprob(cross_filteredcross, map.function="haldane")
cross_filteredcross = sim.geno(cross_filteredcross, map.function="haldane")
cross_filteredcross = fill.geno(cross_filteredcross)
genos = pull.geno(cross_filteredcross)
genotypeGroupings = genos[, "7_59847033_indel"]
genotypeGroupingsDw3 = genos[, "7_59847033_indel"]
genotypeGroupingsChr10 = genos[, "10_7478172_SNP"]
genotypeGroupingsChr4 = genos[ , "4_62448585_indel"]
# genos[, c("7_63757921_SNP", "10_60160484_indel")] # We can also pick out multiple variants.
# If we need to assign groups based on more than one variant
for (i in 1:length(genotypeGroupings)) {
  # If the IS3620C allele is present at Dw3 and the BTx623 allele at the chr 10 locus,
  # assign it to group 1, otherwise group 2.
  if (genotypeGroupingsDw3[i] == 2 && genotypeGroupingsChr10[i] == 1) {
    genotypeGroupings[i] = 1
  }
  else {
    genotypeGroupings[i] = 2
  }
}

colorsGG = gg_color_hue(2)

# If you don't want to thin markers, then comment out with "#"
 cross_filteredcross_map = pull.map(cross_filteredcross) 
 markers2keep = lapply(cross_filteredcross_map, pickMarkerSubset, min.distance=marker_distance) 
 cross_sub = pull.markers(cross_filteredcross, unlist(markers2keep))
 cross_filteredcross = cross_sub

# I think we need to do this again after we've dropped markers.
 cross_filteredcross = jittermap(cross_filteredcross)
 cross_filteredcross = calc.genoprob(cross_filteredcross, map.function="haldane")
 cross_filteredcross = sim.geno(cross_filteredcross, map.function="haldane")

print(summary(cross_filteredcross))

#
# To save time by not loading the data files above into memory, everything below this point can be run
#   after running the above once.
#

#
## Segmentation independent phenotypes:
#

phenoNameSuffixList = c("convexHullArea_Average_Normalized", 
                        "axisAligned_bbox_zDistance_Average_Normalized",
                        "plant_maxGeodesicLength_Average_Normalized",
                        "centroid_zCoord_minus_aabbMinZ_Average_Normalized",
                        "total_surface_area_Average_Normalized")

# Generate a character vector of phenotype names to run scanone with.
phenotypeNameList = character()
for (i in 1:(length(phenoNameSuffixList))) {
  phenoNameSuffix = phenoNameSuffixList[i]
  # http://kbroman.org/qtlcharts/assets/vignettes/userGuide.html
  timepointOneName = paste(TIMEPOINTONESTEM, phenoNameSuffix, sep="_")
  timepointTwoName = paste(TIMEPOINTTWOSTEM, phenoNameSuffix, sep="_")
  timepointThreeName = paste(TIMEPOINTTHREESTEM, phenoNameSuffix, sep="_")
  timepointFourName = paste(TIMEPOINTFOURSTEM, phenoNameSuffix, sep="_")
  phenotypeNameList <- c(phenotypeNameList, timepointOneName, timepointTwoName, timepointThreeName, timepointFourName)
}

# Scanone can be called with a phenotype name list to put all results into the output object.
scanoneResults = scanone(cross_filteredcross, phe=phenotypeNameList)

# make a new data frame from the results that we can interact with.
resultsMelted = melt(scanoneResults, id.vars=c("chr", "pos"))
maxLODvalue = max(resultsMelted$value)
if (maxLODvalue < 6) {
  maxLODvalue = 6
}
# Plot the results as a heat map; each phenotype x timepoint combination is along the y and markers along the x.
# Each cell is labeled with the LOD score. #D55E00", "#0072B2"  #0020C2 #C2A200 #1B325F #F26C4F
# + scale_fill_gradientn("LOD", colours=c("#F0F0F0", "#F0F0F0", "#0020C2", "#E42217", "#E42217"), 
(p <- ggplot(resultsMelted, aes(pos, variable)) + geom_raster(aes(fill = value), interpolate=TRUE) + scale_fill_gradientn("LOD", colours=c("#F0F0F0", "#F0F0F0", FIRSTCOLOR, SECONDCOLOR, SECONDCOLOR), 
                       values=rescale(c(0, 2.0, 3.8, 8, 12)), na.value = "transparent")
+ facet_wrap ( ~chr, nrow = 1, scales = "free_x")
+ theme(line = element_blank(),
        line = element_blank(),
        panel.background = element_blank()))

ggsave("phenotypeByTimepointLODscores_SegmentationIndependent.png", width = QTLOVERTIMEWIDTH, height = QTLOVERTIMEHEIGHT, units = "in")

#
# End segmentation independent phenotypes

#
# Segmentation dependent phenotypes
#

phenoNameSuffixList = c("stem_axisAligned_bbox_zDistance_Average_Normalized",
                        "leafAvg_345_angleAtFixedLength_76_Average_Normalized",
                        "leafAvg_345_width_Average_Normalized", 
                        "leafAvg_345_length_Average_Normalized",
                        "leafAvg_345_surface_area_Average_Normalized")

# Generate a character vector of phenotype names to run scanone with.
phenotypeNameList = character()
for (i in 1:(length(phenoNameSuffixList))) {
  phenoNameSuffix = phenoNameSuffixList[i]
  # http://kbroman.org/qtlcharts/assets/vignettes/userGuide.html
  timepointOneName = paste(TIMEPOINTONESTEM, phenoNameSuffix, sep="_")
  timepointTwoName = paste(TIMEPOINTTWOSTEM, phenoNameSuffix, sep="_")
  timepointThreeName = paste(TIMEPOINTTHREESTEM, phenoNameSuffix, sep="_")
  timepointFourName = paste(TIMEPOINTFOURSTEM, phenoNameSuffix, sep="_")
  phenotypeNameList <- c(phenotypeNameList, timepointOneName, timepointTwoName, timepointThreeName, timepointFourName)
}

# Scanone can be called with a phenotype name list to put all results into the output object.
scanoneResults = scanone(cross_filteredcross, phe=phenotypeNameList)

# make a new data frame from the results that we can interact with.
resultsMelted = melt(scanoneResults, id.vars=c("chr", "pos"))
maxLODvalue = max(resultsMelted$value)
if (maxLODvalue < 6) {
  maxLODvalue = 6
}
# Plot the results as a heat map; each phenotype x timepoint combination is along the y and markers along the x.
# Each cell is labeled with the LOD score. #D55E00", "#0072B2"  #0020C2 #C2A200 #1B325F #F26C4F
# + scale_fill_gradientn("LOD", colours=c("#F0F0F0", "#F0F0F0", "#0020C2", "#E42217", "#E42217"), 
(p <- ggplot(resultsMelted, aes(pos, variable)) + geom_raster(aes(fill = value), interpolate=TRUE)
+ scale_fill_gradientn("LOD", colours=c("#F0F0F0", "#F0F0F0", FIRSTCOLOR, SECONDCOLOR, SECONDCOLOR), 
                       values=rescale(c(0, 2.0, 3.8, 8, 12)), na.value = "transparent")
+ facet_wrap ( ~chr, nrow = 1, scales = "free_x")
+ theme(line = element_blank(),
        line = element_blank(),
        panel.background = element_blank()))

ggsave("phenotypeByTimepointLODscores_SegmentationDependent.png", width = QTLOVERTIMEWIDTH, height = QTLOVERTIMEHEIGHT, units = "in")

#
# End segmentation dependent phenotypes
#

##
#
# Now some post hoc analyses of individual phenotypes by individual genotypes.
#
##

# Use this to make the results more reproducible.
load("BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_allPhenotypes_filledGenos_crossObjectForGrowthOverTimeModels.RData")
genos = pull.geno(cross_filteredcross)
genotypeGroupings = genos[, "7_59847033_indel"]
genotypeGroupingsDw3 = genos[, "7_59847033_indel"]
genotypeGroupingsChr10 = genos[, "10_7478172_SNP"]
genotypeGroupingsChr4 = genos[ , "4_62448585_indel"]

print(summary(cross_filteredcross))

phenos = pull.pheno(cross_filteredcross)  # phenos is a data frame
genos = pull.geno(cross_filteredcross)   # genos is a matrix; column names are markers, which is why genos[,"markerName"] works. We can also pass it a vector of names.

dataFrameForPlotting = data.frame(indel_7_59847033 = genotypeGroupingsDw3, 
                                  SNP_10_7478172 = genotypeGroupingsChr10,
                                  indel_4_62448585 = genotypeGroupingsChr4,
                                  phenos)

resultsMelted = melt(dataFrameForPlotting, id.vars=c("SampleName", "indel_4_62448585", "indel_7_59847033", "SNP_10_7478172"))
resultsMeltedCompleteCases = complete.cases(resultsMelted[, "value"])
resultsMelted = resultsMelted[resultsMeltedCompleteCases, ]

DAPandNameColumns = sapply(resultsMelted$variable, splitPhenotypeIDsToDaysAndName) # splitPhenotypeIDsToDaysAndName is a utility function
resultsMeltedRefined = data.frame(daysAfterPlanting = DAPandNameColumns[1, ], phenotypeName = DAPandNameColumns[2, ], resultsMelted)
resultsMeltedFinal = resultsMeltedRefined[resultsMeltedRefined$daysAfterPlanting != NULL_VALUE, ]

# Need to modify the data classes so that plotting makes the right assumptions.
sapply(resultsMeltedFinal, class)
sapply(resultsMeltedFinal, mode)
resultsMeltedFinal$daysAfterPlanting = as.numeric(as.character(resultsMeltedFinal$daysAfterPlanting)) # must convert it to a character first otherwise sets to 1, 2, 3, etc.)
resultsMeltedFinal$SampleName = as.factor(resultsMeltedFinal$SampleName)
resultsMeltedFinal$indel_4_62448585 = as.factor(resultsMeltedFinal$indel_4_62448585)
resultsMeltedFinal$indel_7_59847033 = as.factor(resultsMeltedFinal$indel_7_59847033)
resultsMeltedFinal$SNP_10_7478172 = as.factor(resultsMeltedFinal$SNP_10_7478172)

resultsMeltedFinal[resultsMeltedFinal$SampleName == 175, "indel_7_59847033"] # Can check the genotype of various individuals.
resultsMeltedFinal[resultsMeltedFinal$SampleName == 398, "indel_7_59847033"] 
resultsMeltedFinal[resultsMeltedFinal$SampleName == 387, "indel_7_59847033"]
#Candidates for the dw3 picture:
resultsMeltedFinal[resultsMeltedFinal$SampleName == 437, "indel_7_59847033"] 
resultsMeltedFinal[resultsMeltedFinal$SampleName == 19, "indel_7_59847033"] 

cbPalette <- c("#999999", "#E69F00", "#56B4E9", "#009E73", "#F0E442", "#0072B2", "#D55E00", "#CC79A7")
modelType = "lm"
resultsMeltedFinalSubsetAABBz = resultsMeltedFinal[resultsMeltedFinal$phenotypeName == "axisAligned_bbox_zDistance_Average", ]
# without smoothing
p = ggplot(data = resultsMeltedFinalSubsetAABBz, aes(x=daysAfterPlanting, y=value, group=SampleName)) + 
      geom_line(aes(color=indel_7_59847033)) +
      scale_color_manual(values = c(SECONDCOLOR, FIRSTCOLOR)) + theme_classic()

# With smoothing
p = ggplot(data = resultsMeltedFinalSubsetAABBz, aes(x=daysAfterPlanting, y=value, group=SampleName, color=indel_7_59847033)) + geom_line(alpha = 0.3) +
    scale_color_manual(values = c(SECONDCOLOR, FIRSTCOLOR)) +
    theme_classic() + 
    stat_smooth(aes(group = indel_7_59847033), method=modelType, level = 0.95, size = 3, alpha = 0.5) +
    coord_cartesian(xlim = c(27, 44)) 
p

ggsave("ggplot_axisAligned_bbox_z_Average.png", width = 14, height = 6, units = "in")


resultsMeltedFinalSubsetLlen345 = resultsMeltedFinal[resultsMeltedFinal$phenotypeName == "leafAvg_345_length_Average", ]
# without smoothing
p = ggplot(data = resultsMeltedFinalSubsetLlen345, aes(x=daysAfterPlanting, y=value, group=SampleName)) + geom_line(aes(color=indel_4_62448585)) +
      scale_color_manual(values = c(SECONDCOLOR, FIRSTCOLOR)) +
      theme_classic()

# With smoothing
p = ggplot(data = resultsMeltedFinalSubsetLlen345, aes(x=daysAfterPlanting, y=value, group=SampleName, color=indel_4_62448585)) + geom_line(alpha = 0.3) +
  scale_color_manual(values = c(SECONDCOLOR, FIRSTCOLOR)) +
  theme_classic() + 
  stat_smooth(aes(group = indel_4_62448585), method=modelType, level = 0.95, size = 3, alpha = 0.5) +
  coord_cartesian(xlim = c(27, 44)) 
p

ggsave("ggplot_leafAvg_345_length_Average.png", width = 14, height = 6, units = "in")

# Trying to pull out individuals with large and small leaf length
largestLeafLengthSubset = resultsMeltedFinalSubsetLlen345[resultsMeltedFinalSubsetLlen345[ , "value"] > 650, ]
#257_2 is decent
smallestLeafLengthSubset = resultsMeltedFinalSubsetLlen345[resultsMeltedFinalSubsetLlen345[ , "value"] < 400, ]
# 18_3 48_1 166_any 178_any 223_any are decent
# 306_any 398_any are great
tailsLeafLengthSubset = resultsMeltedFinalSubsetLlen345[which(resultsMeltedFinalSubsetLlen345[ , "value"] < 500 | resultsMeltedFinalSubsetLlen345[ , "value"] > 650), ]

# Trying to get the mean of plants; we want the average for each group at the first time point, and at the last.
library(plyr)
aabb_z_averages = ddply(resultsMeltedFinalSubsetAABBz, .(indel_7_59847033, daysAfterPlanting), summarise, mean = mean(value))
leafLenAverages = ddply(resultsMeltedFinalSubsetLlen345, .(indel_4_62448585, daysAfterPlanting), summarise, mean = mean(value))

# Fitting linear models to each of the genotypes
aabb_z_allele1Data = resultsMeltedFinalSubsetAABBz[resultsMeltedFinalSubsetAABBz$indel_7_59847033 == 1, ]
aabb_z_allele1Fit = lm(value ~ daysAfterPlanting, data = aabb_z_allele1Data)
summary(aabb_z_allele1Fit)
plot(value ~ daysAfterPlanting, data = aabb_z_allele1Data)
abline(aabb_z_allele1Fit)

aabb_z_allele2Data = resultsMeltedFinalSubsetAABBz[resultsMeltedFinalSubsetAABBz$indel_7_59847033 == 2, ]
aabb_z_allele2Fit = lm(value ~ daysAfterPlanting, data = aabb_z_allele2Data)
summary(aabb_z_allele2Fit)
plot(value ~ daysAfterPlanting, data = aabb_z_allele2Data)
abline(aabb_z_allele2Fit)

leafLen_allele1Data = resultsMeltedFinalSubsetLlen345[resultsMeltedFinalSubsetLlen345$indel_4_62448585 == 1, ]
leafLen_allele1Fit = lm(value ~ daysAfterPlanting, data = leafLen_allele1Data)
summary(leafLen_allele1Fit)
plot(value ~ daysAfterPlanting, data = leafLen_allele1Data)
abline(leafLen_allele1Fit)

leafLen_allele2Data = resultsMeltedFinalSubsetLlen345[resultsMeltedFinalSubsetLlen345$indel_4_62448585 == 2, ]
leafLen_allele2Fit = lm(value ~ daysAfterPlanting, data = leafLen_allele2Data)
summary(leafLen_allele2Fit)
plot(value ~ daysAfterPlanting, data = leafLen_allele2Data)
abline(leafLen_allele2Fit)
dev.off()



