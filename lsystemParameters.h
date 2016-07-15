
#ifndef LSYSTEM_PARAMETERS_H
#define LSYSTEM_PARAMETERS_H

class LSystemParameters {
    public:
        LSystemParameters();
        ~LSystemParameters();

        void setNumberDerivations(int inputNumberDerivations);
        int getNumberDerivations();

        void setInternodeLengths(std::vector<float> inputInternodeLengths);
        void appendInternodeLength(float inputInternodeLength);
        std::vector<float> getInternodeLengths();

        void setInternodeRadii(std::vector<float> inputInternodeRadii);
        void appendInternodeRadius(float inputInternodeRadius);
        std::vector<float> getInternodeRadii();

        void setInternodePitchAngles(std::vector<float> inputPitchAngles); /*!< \brief Pitches up by angle around the L axis (seems to be Y). Corresponds to "^" instruction.> */
        void appendInternodePitchAngle(float inputPitchAngle);
        std::vector<float> getInternodePitchAngles(); /*!< \brief Pitches up by angle around the L axis (seems to be Y). Corresponds to "^" instruction. > */

        void setInternodeTurnAngles(std::vector<float> inputTurnAngles); /*!< \brief Turns right by angle around the U axis (seems to be X). Corresponds to "-" instruction. > */
        void appendInternodeTurnAngle(float inputTurnAngle);
        std::vector<float> getInternodeTurnAngles(); /*!< \brief Turns right by angle around the U axis (seems to be X). Corresponds to "-" instruction. > */

        void setLeafPhyllotaxyAngles(std::vector<float> inputLeafPhyllotaxyAngles); /*!< \brief Turns counter-clockwise around Z axis, starting along X axis. Corresponds to "/" instruction. > */
        void appendLeafPhyllotaxyAngle(float inputLeafPhyllotaxyAngle);
        std::vector<float> getLeafPhyllotaxyAngles(); /*!< \brief Turns counter-clockwise around Z axis, starting along X axis. Corresponds to "/" instruction. > */

        void setLeafWidths(std::vector<float> inputLeafWidths);
        void appendLeafWidth(float inputLeafWidth);
        std::vector<float> getLeafWidths();

        void setLeafCurvatures(std::vector< std::vector< std::pair<float, float> > > inputLeafCurvatures);
        void appendLeafCurvature(std::vector< std::pair<float, float> > inputLeafCurvature);
        std::vector< std::vector< std::pair<float, float> > > getLeafCurvatures();

    private:

        int _numberDerivations;
        std::vector<float> _internodeLengths;
        std::vector<float> _internodeRadii;
        std::vector<float> _internodePitchAngles; /*!< \brief Pitches up by angle around the L axis (seems to be Y). Corresponds to "^" instruction. > */
        std::vector<float> _internodeTurnAngles; /*!< \brief Turns right by angle around the U axis (seems to be X). Corresponds to "-" instruction. > */

        std::vector<float> _leafPhyllotaxyAngles; /*!< \brief Turns counter-clockwise around Z axis, starting along X axis. Corresponds to "/" instruction. > */
        std::vector<float> _leafWidths;
        std::vector< std::vector< std::pair<float, float> > > _leafCurvatures;

};

LSystemParameters returnPredefinedLSystem_debugSystemOne_twoPhytomers();
LSystemParameters returnPredefinedLSystem_debugSystemOne_threePhytomers();

#endif
