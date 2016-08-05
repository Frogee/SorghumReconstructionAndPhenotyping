# Estimate QTL using multiple mapping with R/qtl package
# (Much of this code originates from rqtl.org tutorials.)    
working_directory = getwd()
if (!is.null(working_directory)) setwd(input_file_directory)
# load in cross (cross_inputcross)
load("/home/skt/Documents/Image-based_phenotyping_RFM/2016-06/01_cross_object/BTx623_IS3620c_2016-06-04")
# time points
timepoint_one = "X08.03"
timepoint_two = "X08.10"
timepoint_three = "X08.15"
timepoint_four= "X08.20"

library(qtl)

#############################################################################################
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>#
phenotype_suffix = c("leafAvg_345_angleAtFixedLength_76_Average_Normalized")
phenotype_timepoint = timepoint_one

# Initialize the multiple-QTL-model with the results from single QTL analysis 

init_qtl = makeqtl(cross_inputcross, 
                   chr=c(7),
                   pos=c(72.89654))

init_qtl_formula = y ~ Q1
# <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<#
#############################################################################################

phenotype = paste(phenotype_timepoint, phenotype_suffix, sep="_")

# Penalities calcluated from scantwo (permutations = 25,000, alpha = 0.05)
stepout2 = stepwiseqtl(cross_inputcross, 
                       pheno.col=phenotype,
                       penalties=c(3.199771, 4.375354, 1.942041), 
                       qtl=init_qtl,          
                       formula=init_qtl_formula,
                       max.qtl=6,
                       scan.pairs=FALSE,
                       refine.locations=FALSE,
                       keeptrace=TRUE,
                       verbose=TRUE)

phenotype_directory = paste0(phenotype, "_multiple_qtl_mapping_folder")
dir.create(file.path(phenotype_directory))
setwd(file.path(phenotype_directory))

# all models traversed
thetrace = attr(stepout2, "trace")

# Make file for model selection of best fit model
all_pLOD_model_file_name = paste0("pLOD_models_", phenotype, ".txt")
sink(file = all_pLOD_model_file_name)
cat("QTL models traversed with associated pLOD for phenotype: \n")
cat(phenotype)
cat("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n")
print(thetrace)
cat("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n")
sink()


qtl_formula = attr(stepout2, "formula")

# write .txt lod2interval file
lod2int_file_name = paste0("lod2interval_for_model_",
                           phenotype,
                           ".txt")
sink(file = lod2int_file_name)
cat("Refined QTL from chosen model (within model selection) with pLOD for phenotype: \n")
cat(phenotype)
cat("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n")
cat("Fit model statistics\n\n")
print(summary(fitqtl(cross=cross_inputcross, pheno.col=phenotype, qtl=stepout2, formula = qtl_formula)))
cat("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n")
cat("LOD 2 intervals of the QTL model\n\n")
for(qtl in 1:length(stepout2$name)){ 
  cat("Q")
  cat(qtl)
  cat("\n")
  print(lodint(stepout2, qtl.index=qtl , drop=2, expandtomarkers=TRUE))
  cat("\n")
}
sink()

# write lod score of entire model
lod2int_file_name = paste0("lods_for_multiple-QTL_model_",
                           phenotype,
                           ".txt")
sink(file = lod2int_file_name)
print(attr(stepout2, "lodprofile"))
sink()

# plot the qtl model's lod scores
qtl_model_lod_file_name = paste0("lod_plot_for_multiple-QTL_model_",
                                 phenotype,
                                 ".png")
png(file = qtl_model_lod_file_name,  
    width = 20, 
    height = 10, 
    units ="in", 
    res = 300,
    g = "white")
par(mfrow=c(1,1))
plotLodProfile(stepout2, showallchr=TRUE)
dev.off()

setwd("../")