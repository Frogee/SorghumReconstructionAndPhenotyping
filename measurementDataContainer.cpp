
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <assert.h>
#include "measurementDataContainer.h"
#include "dijkstraPathfinding.h"

MeasurementDataContainer::MeasurementDataContainer() {
    //Empty constructor
}
MeasurementDataContainer::~MeasurementDataContainer() {
    //Empty destructor
}


void MeasurementDataContainer::addNameAndMeasurement(std::string inputString, float inputValue) {
    _map_measurementData.insert(std::pair<std::string, float>(inputString, inputValue));
}

std::map<std::string, float> MeasurementDataContainer::getMeasurementData() {
    return _map_measurementData;
}

void MeasurementDataContainer::writeMeasurementsToFile(std::string fileName) {
    std::cout << "Writing data in measurement data container to file " << fileName << std::endl;
    std::ofstream fileOut;
    fileOut.open(fileName.c_str());
    assert(fileOut.is_open());
    std::map<std::string, float>::iterator itr;
    for (itr = _map_measurementData.begin(); itr != _map_measurementData.end(); itr++) {
        std::string measurementName = itr->first;
        float measurementValue = itr->second;
        fileOut << measurementName << "\t" << measurementValue << std::endl;
    }
}
