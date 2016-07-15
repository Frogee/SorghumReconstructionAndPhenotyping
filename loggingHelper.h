
#ifndef LOGGINGHELPER_H
#define LOGGINGHELPER_H

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

#include "inputParams.h"

/** \brief A class for a global logging object for some logging and debugging functionality.
  *
  * \author Ryan McCormick
  */
class ProgramLogger{
    public:
        ProgramLogger();
        ~ProgramLogger();

        int SETLOGFILE(InputParameters inputParams);
        int DEBUG(std::string inputMessageToLog, uint32_t debuggingLevel = 2, uint32_t debuggingLevelThresholdOverWhichToLog = 2);
        int LOG(std::string inputMessageToLog);

        std::ofstream _outputFileStream;
};

/** \brief The global logging object that will be instantiated in the .cpp file.
  *
  * \author Ryan McCormick
  */
extern ProgramLogger LOG; // Global logger

#endif
