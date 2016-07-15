# Launches the depth image processing pipeline

# Path to the image processing binary
PROCESSINGBINARY=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/DepthCameraPhenotyping_Prototype4/bin/Debug/DepthCameraPhenotyping_v4
# Path to the directory containing the two Screened Poisson Surface Reconstruction binaries
# http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version7.0/
POISSONBINARYDIR=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/PoissonRecon/Bin/Linux/

# Path to the configuration files. This shouldn't need to be modified.
CONFIGFILEDIR=processingConfigFiles/

# Array containing the paths to directories that hold directories with image data.
inputPCD=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/313_2_combinedCloud.pcd
#inputPCD=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/59_1_combinedCloud.pcd
#inputPCD=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/72_2_combinedCloud.pcd
#inputPCD=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/110_3_combinedCloud.pcd
#inputPCD=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/117_1_combinedCloud.pcd
#inputPCD=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/139_3_combinedCloud.pcd
#inputPCD=/home/rfm_node03/Documents/Development/Image-BasedPhenotyping/SorghumMeshPhenotyper/testSuite_v5/ProcessedData/ProcessedClouds/177_3_combinedCloud.pcd

		# Filter the registered cloud.
		time ${PROCESSINGBINARY} \
			${CONFIGFILEDIR}postRegistrationFilteringParameters.xml \
			--removeStatisticalOutliers $inputPCD


