
#include <vector>
#include "lsystemParameters.h"

LSystemParameters::LSystemParameters() {
    // Set up some default L system parameters, assuming measurements in mm. This defaults to a 3 phytomer L-system
    std::vector<float> internodeLengths = {182.104, 42.25, 65.32};
    std::vector<float> internodeRadii = {7.903, 7.903, 7.903};
    std::vector<float> internodeTurnAngles = {0.0, 0.0, 0.0};
    std::vector<float> internodePitchAngles = {0.0, 0.0, 0.0};

    std::vector<float> leafPhyllotaxyAngles = {0.0, 0.0, 0.0};
    std::vector<float> leafWidths = {20.0, 30.0, 40.0};
    std::pair<float,float> leafCurveFirst_P1(0.0, 0.0);
    std::pair<float,float> leafCurveFirst_P2(50.0, 120.0);
    std::pair<float,float> leafCurveFirst_P3(120.0, 130.0);
    std::pair<float,float> leafCurveFirst_P4(200.0, 80.0);
    std::vector< std::pair<float,float> > leafCurveFirst = {leafCurveFirst_P1, leafCurveFirst_P2, leafCurveFirst_P3, leafCurveFirst_P4};
    std::pair<float,float> leafCurveSecond_P1(0.0, 0.0);
    std::pair<float,float> leafCurveSecond_P2(10.0, 10.0);
    std::pair<float,float> leafCurveSecond_P3(28.0, 5.0);
    std::pair<float,float> leafCurveSecond_P4(32.0, -5.0);
    std::vector< std::pair<float,float> > leafCurveSecond = {leafCurveSecond_P1, leafCurveSecond_P2, leafCurveSecond_P3, leafCurveSecond_P4};
    std::pair<float,float> leafCurveThird_P1(0.0, 0.0);
    std::pair<float,float> leafCurveThird_P2(30.0, 20.0);
    std::pair<float,float> leafCurveThird_P3(50.0, 20.0);
    std::pair<float,float> leafCurveThird_P4(90.0, -10.0);
    std::vector< std::pair<float,float> > leafCurveThird = {leafCurveThird_P1, leafCurveThird_P2, leafCurveThird_P3, leafCurveThird_P4};
    std::vector< std::vector< std::pair <float, float> > > leafCurvatures = {leafCurveFirst, leafCurveSecond, leafCurveThird};

    setNumberDerivations(3);

    setInternodeLengths(internodeLengths);
    setInternodeRadii(internodeRadii);
    setInternodePitchAngles(internodePitchAngles);
    setInternodeTurnAngles(internodeTurnAngles);

    setLeafPhyllotaxyAngles(leafPhyllotaxyAngles);
    setLeafWidths(leafWidths);
    setLeafCurvatures(leafCurvatures);

}
LSystemParameters::~LSystemParameters() {
    //empty constructor
}


void LSystemParameters::setNumberDerivations(int inputNumberDerivations) { _numberDerivations = inputNumberDerivations; }
int LSystemParameters::getNumberDerivations() { return _numberDerivations; }

void LSystemParameters::setInternodeLengths(std::vector<float> inputInternodeLengths) { _internodeLengths = inputInternodeLengths; }
void LSystemParameters::appendInternodeLength(float inputInternodeLength) { _internodeLengths.push_back(inputInternodeLength); }
std::vector<float> LSystemParameters::getInternodeLengths() { return _internodeLengths; }

void LSystemParameters::setInternodeRadii(std::vector<float> inputInternodeRadii) { _internodeRadii = inputInternodeRadii; }
void LSystemParameters::appendInternodeRadius(float inputInternodeRadius) { _internodeRadii.push_back(inputInternodeRadius); }
std::vector<float> LSystemParameters::getInternodeRadii() { return _internodeRadii; }

void LSystemParameters::setInternodePitchAngles(std::vector<float> inputInternodePitchAngles) { _internodePitchAngles = inputInternodePitchAngles; }
void LSystemParameters::appendInternodePitchAngle(float inputInternodePitchAngle) { _internodePitchAngles.push_back(inputInternodePitchAngle); }
std::vector<float> LSystemParameters::getInternodePitchAngles() { return _internodePitchAngles; }

void LSystemParameters::setInternodeTurnAngles(std::vector<float> inputInternodeTurnAngles) { _internodeTurnAngles = inputInternodeTurnAngles; }
void LSystemParameters::appendInternodeTurnAngle(float inputInternodeTurnAngle) { _internodeTurnAngles.push_back(inputInternodeTurnAngle); }
std::vector<float> LSystemParameters::getInternodeTurnAngles() { return _internodeTurnAngles; }

void LSystemParameters::setLeafPhyllotaxyAngles(std::vector<float> inputLeafPhyllotaxyAngles) { _leafPhyllotaxyAngles = inputLeafPhyllotaxyAngles; }
void LSystemParameters::appendLeafPhyllotaxyAngle(float inputLeafPhyllotaxyAngle) { _leafPhyllotaxyAngles.push_back(inputLeafPhyllotaxyAngle); }
std::vector<float> LSystemParameters::getLeafPhyllotaxyAngles() { return _leafPhyllotaxyAngles; }

void LSystemParameters::setLeafWidths(std::vector<float> inputLeafWidths) { _leafWidths = inputLeafWidths; }
void LSystemParameters::appendLeafWidth(float inputLeafWidth) { _leafWidths.push_back(inputLeafWidth); }
std::vector<float> LSystemParameters::getLeafWidths() { return _leafWidths; }

void LSystemParameters::setLeafCurvatures(std::vector< std::vector< std::pair<float, float> > > inputLeafCurvatures) { _leafCurvatures = inputLeafCurvatures; }
void LSystemParameters::appendLeafCurvature(std::vector< std::pair<float, float> > inputLeafCurvature) { _leafCurvatures.push_back(inputLeafCurvature); }
std::vector< std::vector< std::pair<float, float> > > LSystemParameters::getLeafCurvatures() { return _leafCurvatures; }


LSystemParameters returnPredefinedLSystem_debugSystemOne_twoPhytomers() {

    LSystemParameters lsystemParams;
    int debugNumberDerivations = 2;  //('Number of derivations:', 2)
    lsystemParams.setNumberDerivations(debugNumberDerivations);
    std::vector<float> debugInternodeLengths = {147.85301208496094, 54.37424087524414}; //('Internode lengths:', [147.85301208496094, 45.62873077392578])
    lsystemParams.setInternodeLengths(debugInternodeLengths);
    std::vector<float> debugInternodeRadii = {6.921778202056885, 6.921778202056885}; // ('Internode radii:', [6.921778202056885, 6.921778202056885])
    lsystemParams.setInternodeRadii(debugInternodeRadii);
    std::vector<float> debugInternodePitchAngles = {-6.08206844329834, 0.0}; // ('Internode pitch angles (rotation around Y axis):', [-6.08206844329834, 0.0])
    lsystemParams.setInternodePitchAngles(debugInternodePitchAngles);
    std::vector<float> debugInternodeTurnAngles = {-4.473091125488281, 0.0}; // ('Internode turn angles (rotation around X axis):', [-4.473091125488281, 0.0])
    lsystemParams.setInternodeTurnAngles(debugInternodeTurnAngles);
    std::vector<float> debugLeafPhyllotaxyAngles = {-94.10169982910156, 84.54471588134766};
    lsystemParams.setLeafPhyllotaxyAngles(debugLeafPhyllotaxyAngles); //('Leaf phyllotaxy angles:', [-94.10169982910156, 105.68089294433594])
    std::vector<float> debugLeafWidths = {30.0, 30.0};
    lsystemParams.setLeafWidths(debugLeafWidths); //('Leaf widths: ', [30.0, 30.0])
    std::pair<float,float> leafCurveFirst_P1(0.0, 0.0);
    std::pair<float,float> leafCurveFirst_P2(80.63927459716797, 96.1021499633789);
    std::pair<float,float> leafCurveFirst_P3(80.63935852050781, 96.10224914550781);
    std::pair<float,float> leafCurveFirst_P4(80.63943481445312, 96.10234069824219);
    std::vector< std::pair<float,float> > leafCurveFirst = {leafCurveFirst_P1, leafCurveFirst_P2, leafCurveFirst_P3, leafCurveFirst_P4}; // ('Control Points:', [(0.0, 0.0, 1), (57.5994873046875, 68.64439392089844, 1), (57.599544525146484, 68.64446258544922, 1), (57.59960174560547, 68.64453125, 1)])
    std::pair<float,float> leafCurveSecond_P1(0.0, 0.0);
    std::pair<float,float> leafCurveSecond_P2(80.63927459716797, 96.1021499633789);
    std::pair<float,float> leafCurveSecond_P3(80.63935852050781, 96.10224914550781);
    std::pair<float,float> leafCurveSecond_P4(80.63943481445312, 96.10234069824219);
    std::vector< std::pair<float,float> > leafCurveSecond = {leafCurveSecond_P1, leafCurveSecond_P2, leafCurveSecond_P3, leafCurveSecond_P4}; // ('Control Points:', [(0.0, 0.0, 1), (68.64439392089844, 57.5994873046875, 1), (68.64446258544922, 57.599544525146484, 1), (68.64453125, 57.59960174560547, 1)])
    std::vector< std::vector< std::pair <float, float> > > debugLeafCurvatures = {leafCurveFirst, leafCurveSecond};
    lsystemParams.setLeafCurvatures(debugLeafCurvatures);
    return lsystemParams;
}

LSystemParameters returnPredefinedLSystem_debugSystemOne_threePhytomers() {

    LSystemParameters lsystemParams;
    int debugNumberDerivations = 3;
    lsystemParams.setNumberDerivations(debugNumberDerivations);
    std::vector<float> debugInternodeLengths = {147.85301208496094, 54.37424087524414, 73.88188171386719};
    lsystemParams.setInternodeLengths(debugInternodeLengths);
    std::vector<float> debugInternodeRadii = {6.921778202056885, 6.921778202056885, 8.010336875915527};
    lsystemParams.setInternodeRadii(debugInternodeRadii);
    std::vector<float> debugInternodePitchAngles = {-6.08206844329834, 0.0, -0.9762353897094727};
    lsystemParams.setInternodePitchAngles(debugInternodePitchAngles);
    std::vector<float> debugInternodeTurnAngles = {-4.473091125488281, 0.0, -3.230930805206299};
    lsystemParams.setInternodeTurnAngles(debugInternodeTurnAngles);
    std::vector<float> debugLeafPhyllotaxyAngles = {-82.33898162841797, 114.1353759765625, -68.3261947631836};
    lsystemParams.setLeafPhyllotaxyAngles(debugLeafPhyllotaxyAngles);
    std::vector<float> debugLeafWidths = {30.0, 30.0, 30.0};
    lsystemParams.setLeafWidths(debugLeafWidths);
    std::pair<float,float> leafCurveFirst_P1(0.0, 0.0);
    std::pair<float,float> leafCurveFirst_P2(80.63927459716797, 96.1021499633789);
    std::pair<float,float> leafCurveFirst_P3(80.63935852050781, 96.10224914550781);
    std::pair<float,float> leafCurveFirst_P4(80.63943481445312, 96.10234069824219);
    std::vector< std::pair<float,float> > leafCurveFirst = {leafCurveFirst_P1, leafCurveFirst_P2, leafCurveFirst_P3, leafCurveFirst_P4};
    std::pair<float,float> leafCurveSecond_P1(0.0, 0.0);
    std::pair<float,float> leafCurveSecond_P2(80.63927459716797, 96.1021499633789);
    std::pair<float,float> leafCurveSecond_P3(80.63935852050781, 96.10224914550781);
    std::pair<float,float> leafCurveSecond_P4(80.63943481445312, 96.10234069824219);
    std::vector< std::pair<float,float> > leafCurveSecond = {leafCurveSecond_P1, leafCurveSecond_P2, leafCurveSecond_P3, leafCurveSecond_P4};
    std::pair<float,float> leafCurveThird_P1(0.0, 0.0);
    std::pair<float,float> leafCurveThird_P2(101.93933868408203, 85.53726196289062);
    std::pair<float,float> leafCurveThird_P3(101.93943786621094, 85.53734588623047);
    std::pair<float,float> leafCurveThird_P4(101.93954467773438, 85.53742980957031);
    std::vector< std::pair<float,float> > leafCurveThird = {leafCurveThird_P1, leafCurveThird_P2, leafCurveThird_P3, leafCurveThird_P4};
    std::vector< std::vector< std::pair <float, float> > > debugLeafCurvatures = {leafCurveFirst, leafCurveSecond, leafCurveThird};
    lsystemParams.setLeafCurvatures(debugLeafCurvatures);

    return lsystemParams;
}
