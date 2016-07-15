# Source file for Screened Poisson Surface reconstruction parameters

# Documentation for the parameters can be found at:
# http://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version7.0/

:<<"COMMENT"
The program is called in the following manner. Change the values below the comment accordingly.

${POISSONBINARYDIR}PoissonRecon --in $inputPly --out TestPoissonReconout.ply --density \
	--threads 4 \
	--depth 8 \
	--fullDepth 5 \
	--voxelDepth 9 \
	--samplesPerNode 10.0 \
	--pointWeight 6 #Values between 6 and 10 seem to work pretty well

${POISSONBINARYDIR}SurfaceTrimmer --in TestPoissonReconout.ply --out TestPoissonReconTrimmed.ply \
	--trim 6 \
	--smooth 5 \
	--aRatio 0.001
COMMENT

echo
echo "Sourcing input parameters for Screened Poisson Surface reconstruction."
echo

threads=4
depth=9
fullDepth=5
voxelDepth=9
samplesPerNode=10.0
pointWeight=6

trim=5
smooth=5
aRatio=0.001



