# Launches the depth image processing pipeline

# Path to the directory containing the two Screened Poisson Surface Reconstruction binaries
# http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version7.0/
POISSONBINARYDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/PoissonRecon/Bin/Linux/

# Use Screened Poisson Surface Reconstruction to mesh the cloud.
#inputPly=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/117_1_combinedCloudFiltered_postProcessed.ply
inputPly=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/110_3_combinedCloudFiltered_postProcessed.ply
#inputPly=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/139_3_combinedCloudFiltered_postProcessed.ply
#inputPly=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/177_3_combinedCloudFiltered_postProcessed.ply
#inputPly=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/279_3_combinedCloudFiltered_postProcessed.ply
#inputPly=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/384_1_combinedCloudFiltered_postProcessed.ply


#--depth had previously been set to 7; trying to set to 8 to prevent smaller leaves from getting cut out.
		echo
		echo "Constructing surface"
		echo

		${POISSONBINARYDIR}PoissonRecon --in $inputPly --out TestPoissonReconout.ply --density \
			--threads 4 \
			--depth 7 \
			--fullDepth 5 \
			--voxelDepth 9 \
			--samplesPerNode 5.0 \
			--pointWeight 6 #Values between 6 and 10 seem to work pretty well

		echo
		echo "Trimming surface"
		echo

		${POISSONBINARYDIR}SurfaceTrimmer --in TestPoissonReconout.ply --out TestPoissonReconTrimmed.ply \
			--trim 5 \
			--smooth 5 \
			--aRatio 0.001


