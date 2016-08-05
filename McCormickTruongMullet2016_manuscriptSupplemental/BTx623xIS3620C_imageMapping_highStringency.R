# R/qtl QTL mapping for image based phenotyping
# Written by Sandra Truong and Ryan McCormick - 02/01/16
#   Latest update: 06/04/16
# The purpose of this script is to QTL map all normalized image-based phenotypes, including
#   individual replicates. Two file paths are included, one for a low-stringency "discovery" run, and one
#   for a high-stringency "analysis" run; see the commented path variables below.
# It assumes that permutations have already been run and the scanone object saved to a "permutations25000_RObject" file.
# It also assumes a few directories already exist in the current working directory to which it will generate
#   new directories and write files.

#Change these file paths to reflect your system:
input_file_directory = file.path("./")
input_file_name = file.path("./BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_allPhenotypes.csv")
baseWorkingDirectory = "/home/rfm_node03/Documents/Analyses/RStudioProjects/RIG/Sbi3/BTx623xIS3620C/QTLmapping/ImageBasedPhenotyping/BTx623xIS3620C_v5_1"
#individualPhenoQTLimagePath = "/home/rfm_node03/Documents/Analyses/RStudioProjects/RIG/Sbi3/BTx623xIS3620C/QTLmapping/ImageBasedPhenotyping/BTx623xIS3620C_08-03-2015_v5/individualPhenotypeQTLPlots"
#combinedQTLimagePath = "/home/rfm_node03/Documents/Analyses/RStudioProjects/RIG/Sbi3/BTx623xIS3620C/QTLmapping/ImageBasedPhenotyping/BTx623xIS3620C_08-03-2015_v5/scanone_images"
# These two are used for filepaths for a higher stringency run for alpha_value of 0.05 or higher.
 individualPhenoQTLimagePath = "/home/rfm_node03/Documents/Analyses/RStudioProjects/RIG/Sbi3/BTx623xIS3620C/QTLmapping/ImageBasedPhenotyping/BTx623xIS3620C_v5_1/individualPhenotypeQTLPlots_highStringency"
 combinedQTLimagePath = "/home/rfm_node03/Documents/Analyses/RStudioProjects/RIG/Sbi3/BTx623xIS3620C/QTLmapping/ImageBasedPhenotyping/BTx623xIS3620C_v5_1/scanone_images_highStringency"
 globalQTLSignificantIntervalFileName = "/home/rfm_node03/Documents/Analyses/RStudioProjects/RIG/Sbi3/BTx623xIS3620C/QTLmapping/ImageBasedPhenotyping/BTx623xIS3620C_v5_1/SignificantSingleQTLIntervals.txt"
 
output_file_stem = "BTx623xIS3620C_imageBased"
image_name_stem = "BTx623xIS3620C_imageBased"

# Use this vector if want to specify the phenotype(s) to map. Otherwise, uncomment out the empty vector line.
#phenotype_list=c("X08.03_leaf_3_angleAtPercentLength_25_Average_Normalized")
phenoNameSuffixList = c("convexHullArea_Average_Normalized", 
                        "axisAligned_bbox_zDistance_Average_Normalized",
                        "plant_maxGeodesicLength_Average_Normalized",
                        "centroid_zCoord_minus_aabbMinZ_Average_Normalized",
                        "total_surface_area_Average_Normalized",
                        "stem_axisAligned_bbox_zDistance_Average_Normalized",
                        "leafAvg_345_angleAtFixedLength_76_Average_Normalized",
                        "leafAvg_345_length_Average_Normalized",
                        "leafAvg_345_width_Average_Normalized",
                        "leafAvg_345_surface_area_Average_Normalized")

options(warn=1)
library(qtl)

# These permutations were calculated for 25,000 permutations for 97 normalized phenotypes of 398 samples and 10787 markers as RIL
load("permutations25000_RObject")
alpha_value = 0.05
permutationThreshold = c(summary(scanone_filteredcross_perm, alpha=alpha_value))

str_method = "scanone"

#
# Begin script
#

#Set the working directory so files are output to correct location.
setwd(file.path(baseWorkingDirectory))

#Read cross in, convert it to RIL
cross_inputcross = read.cross("csvr", input_file_directory, input_file_name, genotypes=c("AA","AB","BB","D","C"))

# Choose the distance (in cM) to thin out
#marker_distance = 0.1
# If you don't want to thin markers, then comment out with "#"
#cross_inputcross_map = pull.map(cross_inputcross) 
#markers2keep = lapply(cross_inputcross_map, pickMarkerSubset, min.distance=marker_distance) 
#cross_sub = pull.markers(cross_inputcross, unlist(markers2keep))
#cross_inputcross = cross_sub

cross_filteredcross = convert2riself(cross_inputcross)
cross_filteredcross = jittermap(cross_filteredcross)

cross_filteredcross = calc.genoprob(cross_filteredcross, map.function="haldane")
cross_filteredcross = sim.geno(cross_filteredcross, map.function="haldane")

print(summary(cross_filteredcross))

phenos = pull.pheno(cross_filteredcross)
phenoNames = colnames(phenos)
phenotype_list = c()
for (i in 1:length(phenoNameSuffixList)) {
  for (j in 1:length(phenoNames)) {
    if (grepl(paste("[3,5,0]_", phenoNameSuffixList[i], sep=""), phenoNames[j])) {   # This works because the dates are 03, 10, 15, 20
      phenotype_list <- c(phenotype_list, phenoNames[j])
    }
  }
}
print(phenotype_list)

########################
########################
########################

# QTL mapping loop
# For each phenotype:


sink(globalQTLSignificantIntervalFileName, append=FALSE)
cat("Significant LOD intervals from single QTL mapping.\n\n")
sink()

for (i in 1:(length(phenotype_list))) {   # Begin for each phenotype in phenotype list
  
  setwd(file.path(combinedQTLimagePath))
  
  #Get the number of individuals phenotyped; skip if lower than an arbitrary threshold.
  print("On phenotype:")
  print(phenotype_list[i])
  currentPhenoValues <- pull.pheno(cross_filteredcross, phenotype_list[i])
  numPhenotyped <- 0
  for (j in 1:(length(currentPhenoValues))) {
    if (!is.na(currentPhenoValues[j])) {
      numPhenotyped <- numPhenotyped + 1
    }
  }
  if (numPhenotyped < 70) {   # Magic number of 70 individuals required to scan for QTL.
    next 
  }
  
  # Check if file exists and skip if so.
  sampleNumString <- paste(numPhenotyped, "samples", sep="")
  plotname <- paste(image_name_stem, sampleNumString, phenotype_list[i], sep="-")
  QTL_image_name_stem <- paste(image_name_stem, phenotype_list[i], sep="-")
  QTL_image_name <- paste(QTL_image_name_stem, "jpg", sep=".")
  
  if (file.exists(QTL_image_name) == TRUE) {
    next
  }
  
  # Scanone and and plots:
  ptm <- proc.time()
  scanone_filteredcross <- scanone(cross_filteredcross, pheno.col=phenotype_list[i])
  print(proc.time() - ptm)
  
  # Store an appropriate y limit for the plot based on the scanone results.
  ylimit <- permutationThreshold
  for (j in 1:(length(scanone_filteredcross[,3])))
  {
    if (scanone_filteredcross[j,3] >= ylimit)
    {
      ylimit <- c(scanone_filteredcross[j,3]+1)
    }
  }
  
  # Plot entire genome; sys.sleep() is called since occasionally run into trouble with the 
  #   devices otherwise. Not sure if this is an R problem or RStudio problem.
  layout(matrix(c(1, 1, 2, 3), 2, 2, byrow = TRUE))
  plot(scanone_filteredcross, col=c("blue"), lty=c(1), ylim=c(0, ylimit), main=plotname)
  add.threshold(scanone_filteredcross, perms=scanone_filteredcross_perm, alpha=alpha_value, col="red", lty=2)
  normedPhenoName = phenotype_list[i]
  splitNormedName = unlist(strsplit(normedPhenoName, "_"))
  joinedNonNormedName = paste(splitNormedName[1:(length(splitNormedName) - 1)], collapse="_")
  phenoVals = pull.pheno(cross_filteredcross, pheno.col=joinedNonNormedName)
  plot(phenoVals, ylab=joinedNonNormedName)
  hist(phenoVals, xlab=joinedNonNormedName, main=joinedNonNormedName)
  Sys.sleep(3)
  
  # Write the genome wide plot to a file.
  jpeg(QTL_image_name, width=1400, height=1000, res=200)
  layout(matrix(c(1, 1, 2, 3), 2, 2, byrow = TRUE))
  plot(scanone_filteredcross, col=c("blue"), lty=c(1), ylim=c(0, ylimit), main=plotname)
  add.threshold(scanone_filteredcross, perms=scanone_filteredcross_perm, alpha=alpha_value, col="red", lty=2)
  normedPhenoName = phenotype_list[i]
  splitNormedName = unlist(strsplit(normedPhenoName, "_"))
  joinedNonNormedName = paste(splitNormedName[1:(length(splitNormedName) - 1)], collapse="_")
  phenoVals = pull.pheno(cross_filteredcross, pheno.col=joinedNonNormedName)
  plot(phenoVals, ylab=joinedNonNormedName)
  hist(phenoVals, xlab=joinedNonNormedName, main=joinedNonNormedName)
  dev.off()
  
  #If there were QTL above threshold, make a new directory and save some additional information
  #First determine if there were significant markers above alpha.
  markersAboveAlpha = FALSE
  for (j in 1:(length(scanone_filteredcross[,3]))) {
    if (scanone_filteredcross[j,3] >= permutationThreshold) {
      markersAboveAlpha = TRUE
      break
    }
  }
  
  if (markersAboveAlpha == TRUE) {  # Begin if markersAboveAlpha
    dir.create(file.path(individualPhenoQTLimagePath, phenotype_list[i]))
    setwd(file.path(individualPhenoQTLimagePath, phenotype_list[i]))
    
    # Plot additional information
    layout(matrix(c(1, 1, 2, 3), 2, 2, byrow = TRUE))
    plot(scanone_filteredcross, col=c("blue"), lty=c(1), ylim=c(0, ylimit), main=plotname)
    add.threshold(scanone_filteredcross, perms=scanone_filteredcross_perm, alpha=alpha_value, col="red", lty=2)
    normedPhenoName = phenotype_list[i]
    splitNormedName = unlist(strsplit(normedPhenoName, "_"))
    joinedNonNormedName = paste(splitNormedName[1:(length(splitNormedName) - 1)], collapse="_")
    phenoVals = pull.pheno(cross_filteredcross, pheno.col=joinedNonNormedName)
    plot(phenoVals, ylab=joinedNonNormedName)
    hist(phenoVals, xlab=joinedNonNormedName, main=joinedNonNormedName)
    dev.off()
    Sys.sleep(3)
    
    #Save file with additional information on distribution
    QTLwithDist_image_name_stem <- paste(image_name_stem, phenotype_list[i], "distInfo", sep="-")
    QTLwithDist_image_name <- paste(QTLwithDist_image_name_stem, "jpg", sep=".")
    jpeg(QTLwithDist_image_name, width=1400, height=1000, res=200)
    layout(matrix(c(1, 1, 2, 3), 2, 2, byrow = TRUE))
    plot(scanone_filteredcross, col=c("blue"), lty=c(1), ylim=c(0, ylimit), main=plotname)
    add.threshold(scanone_filteredcross, perms=scanone_filteredcross_perm, alpha=alpha_value, col="red", lty=2)
    normedPhenoName = phenotype_list[i]
    splitNormedName = unlist(strsplit(normedPhenoName, "_"))
    joinedNonNormedName = paste(splitNormedName[1:(length(splitNormedName) - 1)], collapse="_")
    phenoVals = pull.pheno(cross_filteredcross, pheno.col=joinedNonNormedName)
    plot(phenoVals, ylab=joinedNonNormedName)
    hist(phenoVals, xlab=joinedNonNormedName, main=joinedNonNormedName)
    dev.off()
    Sys.sleep(3)
    
    # Save file with just the scanone results
    jpeg(QTL_image_name, width=1400, height=700, res=200)
    par(mfrow=c(1,1), las=1)
    plot(scanone_filteredcross, col=c("blue"), lty=c(1), ylim=c(0, ylimit), main=plotname)
    add.threshold(scanone_filteredcross, perms=scanone_filteredcross_perm, alpha=alpha_value, col="red", lty=2)
    dev.off()
    Sys.sleep(3)
    
    # Save an effect plot.
    par(mfrow=c(1,1), las=1)
    effectscan(cross_filteredcross, pheno.col=phenotype_list[i], get.se=FALSE, draw=TRUE, main=plotname)
    Sys.sleep(3)
    
    effects_image_name <- paste(QTL_image_name_stem, "effects", sep="_")
    effects_image_name <- paste(effects_image_name, "jpg", sep=".")
    
    jpeg(effects_image_name, width=1600, height=800, res=200)
    effectscan(cross_filteredcross, pheno.col=phenotype_list[i], get.se=FALSE, draw=TRUE, main=plotname)
    dev.off()
    
    # For all significant markers above the permutation threshold, write them to a text file along with their LOD intervals.
    file_QTLstem <- paste(phenotype_list[i],"QTL",sep="_")
    file_QTLname <- paste(file_QTLstem, "txt", sep=".")
    sink(file=file_QTLname, append=FALSE)
    cat("For phenotype", phenotype_list[i], "\n")
    cat("Marker","\t", "Chr", "\t", "cM","\t","lod","\n")
    sink()
    
    for (j in 1:(length(scanone_filteredcross[,3]))) {
      if (scanone_filteredcross[j,3] >= permutationThreshold) {
        sink(file=file_QTLname, append=TRUE)
        cat(rownames(scanone_filteredcross)[j],"\t", scanone_filteredcross[j,1],"\t", scanone_filteredcross[j,2], "\t",scanone_filteredcross[j,3],"\n")
        sink()

        #PXG_name <- paste(phenotype_list[i], rownames(scanone_filteredcross)[j], sep="_")
        #PXG_name <- paste(PXG_name, "jpg", sep=".")
        #jpeg(PXG_name, width=1600, height=800, res=200)
        #plotPXG(cross_filteredcross, rownames(scanone_filteredcross)[j], pheno.col=phenotype_list[i], jitter=1, infer=TRUE, main=rownames(scanone_filteredcross)[j])
        #dev.off()
      }
    }
    # Save the lod intervals. Note that this assumes 10 chromosomes.
    chromosomesWithSignificantMarkers = integer()
    sink(file=file_QTLname, append=TRUE)
    cat("\nLOD-2 intervals for all chromosomes:\n")
    for (k in 1:10) {
      print(lodint(scanone_filteredcross, drop=2, chr=k, expandtomarkers=TRUE))
      lodInterval = lodint(scanone_filteredcross, drop=2, chr=k, expandtomarkers=TRUE)
      for (lodValue in lodInterval[['lod']]) {
        if (lodValue >= permutationThreshold) {
          if (is.na(match(k, chromosomesWithSignificantMarkers))) {
            chromosomesWithSignificantMarkers = c(chromosomesWithSignificantMarkers, k)
          }
        }
      }
    }
    sink()
    
    sink(globalQTLSignificantIntervalFileName, append=TRUE)
    cat(phenotype_list[i], "\n")
    for (k in chromosomesWithSignificantMarkers) {
      print(lodint(scanone_filteredcross, drop=2, chr=k, expandtomarkers=TRUE))
    }
    cat("\n")
    sink()
    
  } #End if markersAboveAlpha
  
  setwd(file.path(baseWorkingDirectory))
  
} # End for each phenotype in phenotype list

