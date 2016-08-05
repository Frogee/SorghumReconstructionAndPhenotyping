
# Notes on reproducibility
Please note that if you attempt an exact reproduction of the results that there are a number of steps in the analysis that depend on genotype probabilities, and the results will not be exactly the same due to sampling. If you start from the .csv files, you will find that QTL shift positions slightly, and some QTL are sometimes over the significance threshold, sometimes not. In an effort to make the analysis as reproducible as possible, RData files containing R/qtl cross objects that were used in the analysis are provided. If you are attempting replication, load() those instead of using the "raw data" provided as .csv files. Note also that, if you are attempting replication, file paths specified in the R-scripts are absolute paths and will not work out of the box, and they will need to be adjusted.

# Workflow
The workflow was as follows. First, a low stringency (i.e. low specificity, high sensitivity) analysis was performed using single QTL mapping for all of the phenotypes with the entire marker set (BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_allPhenotypes.csv.gz or BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_allPhenotypes_crossObjectForSingleQTLMapping.RData). The results were manually inspected. A subset of phenotypes that could be reliably measured and had biologically interesting results were identified and remapped at higher stringency (i.e. 95% threshold) with a single QTL model (BTx623xIS3620C_imageMapping_highStringency.R).

For multiple-QTL mapping, the marker set was reduced. Once single-QTL mapping had identified QTL for each phenotype by timepoint combination, the significant markers were added back if they had been dropped from the reduced set. Change over time for these phenotypes was examined (BTx623xIS3620C_changeOverTime_v3.R), and multiple-QTL mapping was performed (see the seg_dependent and seg_independent directories within the multipleQTLMapping directory). The file multiple-QTL-mappingLOD-2Ints.ods contains the collated intervals from multiple-QTL mapping within the multipleQTLMapping directory.

# File desciptions

  - BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing.csv.gz
    - Contains genotypes for the entire RIL population for AA, AB, BB markers. Physical positions are relative to Sbi3.

  - BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_allPhenotypes.csv.gz
    - Contains genotypes for the entire RIL population and ~9000 rows of phenotypes measured from the RIL population. The phenotype rows include measurements for each individual RIL replicate, normalized values, and RIL averages for the 4 timepoints.

  - BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_analysisSubsetPhenotypes.csv.gz
    - Contains genotypes for the entire RIL population and 40 rows of phenotypes (10 phenotypes at 4 timepoints). 

  - BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_allPhenotypes_crossObjectForSingleQTLMapping.RData
    - The R data for the R/qtl cross object used for single QTL mapping. Using this *should* make the single QTL mapping results deterministic since sim.geno() has already been run for the cross object.

  - BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_allPhenotypes_filledGenos_crossObjectForGrowthOverTimeModels.RData
    - The R data for the R/qtl cross object used for fitting linear models of growth over time. Using this *should* make the single QTL mapping results deterministic since sim.geno() and fill.geno() has already been run for the cross object.

  - BTx623xIS3620c_01-02-14_v0007_Sbi3_MapAfterSDCOsleq2ToMissing_analysisSubsetPhenotypes_crossObjectForMultiQTLMapping.RData
    - The R data for the R/qtl cross object used for multiple QTL mapping. Using this *should* make the single QTL mapping results deterministic since sim.geno() has already been run for the cross object.

  - BTx623xIS3620C_imageMapping.R
    - The R script used for exploratory single-QTL analyses. The subset of phenotypes for the final analyses were chosen based on these results.

  - BTx623xIS3620C_imageMapping_highStringency.R
    - The R script used for single-QTL analyses of the specific subset of phenotypes. The results of this analysis were used to seed multiple QTL mapping analyses. Peak LOD markers from this analysis were added back to the thinned marker set if they were dropped during thinning.

  - BTx623xIS3620C_changeOverTime_v3.R
    - The R script used to examine growth over time.

# Contact
Ryan McCormick at ryanabashbash@tamu.edu
