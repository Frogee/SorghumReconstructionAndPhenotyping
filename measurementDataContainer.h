
#ifndef MEASUREMENTDATACONTAINER_H
#define MEASUREMENTDATACONTAINER_H

#include <string>
#include <vector>
#include <map>

/** \brief Class that holds measurements made for a given mesh as they are being made.
  */
class MeasurementDataContainer {
    public:
        MeasurementDataContainer();
        ~MeasurementDataContainer();

        void addNameAndMeasurement(std::string inputString, float inputValue);

        std::map<std::string, float> getMeasurementData();

        void writeMeasurementsToFile(std::string fileName);

    private:
        std::vector<std::string> _measurementNameContainer;
        std::vector<float> _measurementValueContainer;
        std::map<std::string, float> _map_measurementData;
};

#endif
