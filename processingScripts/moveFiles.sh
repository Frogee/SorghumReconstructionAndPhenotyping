
TARGETDIRECTORY=$1
FILEPREFIX=$2

mv sample_registeredClouds/PoissonReconTrimmed.ply ${TARGETDIRECTORY}/${FILEPREFIX}_PoissonReconTrimmed.ply
mv sample_registeredClouds/combinedCloud.pcd ${TARGETDIRECTORY}/${FILEPREFIX}_combinedCloud.pcd
mv sample_registeredClouds/combinedCloudFiltered.pcd ${TARGETDIRECTORY}/${FILEPREFIX}_combinedCloudFiltered.pcd
mv sample_registeredClouds/combinedCloudFiltered.ply ${TARGETDIRECTORY}/${FILEPREFIX}_combinedCloudFiltered.ply
mv sample_registeredClouds/combinedCloudFiltered_postProcessed.ply ${TARGETDIRECTORY}/${FILEPREFIX}_combinedCloudFiltered_postProcessed.ply

