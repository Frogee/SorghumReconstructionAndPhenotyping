
#include <string>
#include <iostream>
#include <assert.h>
#include <fstream>

#include "loggingHelper.h"
#include "inputParams.h"

ProgramLogger::ProgramLogger() {
    // Empty constructor
}

ProgramLogger::~ProgramLogger() {
    _outputFileStream.close();
}

/** \brief Opens the file to which logging information will be appended.
  * \param[in] inputParams An InputParameters object created from the input .xml file.
  * \author Ryan McCormick
  */
int ProgramLogger::SETLOGFILE(InputParameters inputParams) {
    _outputFileStream.open(inputParams.debuggingParameters.getLoggingPath(), std::ofstream::out | std::ofstream::app);
    assert(_outputFileStream.is_open() == true && "Unable to open the logging file. Check that the path is valid.");
    return(0);
}

/** \brief Writes content to the logging file and stdout if the debugging level is set sufficiently high by the user.
  * \param[in] inputMessageToLog The message to log as a string.
  * \param[in] debuggingLevel The current debugging level, typically obtained from inputParams.debuggingParameters.getDebuggingLevel()
  * \param[in] debuggingThresholdOverWhichToLog The debugging level at which the input message should be logged.
  * \author Ryan McCormick
  */
int ProgramLogger::DEBUG(std::string inputMessageToLog, uint32_t debuggingLevel, uint32_t debuggingLevelThresholdOverWhichToLog) {
    if (debuggingLevel >= debuggingLevelThresholdOverWhichToLog) {
        std::cout << inputMessageToLog << std::endl;
        this->LOG(inputMessageToLog);
    }
    return(0);
}

/** \brief Write content to the logging file.
  * \param[in] inputMessageToLog The message to log as a string.
  * \author Ryan McCormick
  */
int ProgramLogger::LOG(std::string inputMessageToLog) {
    assert(_outputFileStream.is_open() == true && "Unable to open the logging file. Check that the path is valid.");
    _outputFileStream << inputMessageToLog << std::endl;
    return(0);
}

/** \brief Global logging object.
  *
  * \author Ryan McCormick
  */
ProgramLogger LOG; // Global logger
