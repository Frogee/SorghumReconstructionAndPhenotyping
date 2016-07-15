#ifndef TUPLETRIPLET_H
#define TUPLETRIPLET_H

#include <tuple>
#include <map>

#include <pcl/point_types.h>


/** \brief Container to stores 3 floats, intended for either (x, y, z) or (r, g, b).
  *
  * The TupleTriplet is a tuple of 3 floats used as a hacky way to be able to store data from types pcl::PointXYZ, geodesic::SurafcePoint, and RGB
  * values into std::map and std::unordered_map for fast searches. The better way (I think) would be to create hash functions for the pcl::PointXYZ,
  * geodesic::SurfacePoint, etc, but using TupleTriplets as an intermediate has worked for now. TupleTriplets are of the form (x, y, z) and (r, g, b).
  */
typedef std::tuple<float, float, float> TupleTriplet;

/** \brief Tests if two TupleTriplets are equal. It's pretty unsafe since it tests for exact equality on floats (e.g. rounding from 5 to 4 decimals
  * will cause two TupleTriplets to evaluate as not equal).
  */
bool TupleTripletsAreEqual(TupleTriplet point1, TupleTriplet point2);

/** \brief Converts at pcl::PointXYZ to a TupleTriplet
  */
TupleTriplet convertPclPointXYZtoTupleTriplet(pcl::PointXYZ inputPoint);

/** \brief Converts a TupleTriplet to a pcl::PointXYZ
  */
pcl::PointXYZ convertTupleTriplettoPclPointXYZ(TupleTriplet inputTuple);

/** \brief Prints the contents of a TupleTriplet
  */
void printTupleTriplet(TupleTriplet inputTuple);

/** \brief Prints the contents of a TupleTriplet
  */
void printTupleTriplet(TupleTriplet inputTuple, uint32_t debuggingLevel, uint32_t debuggingThreshold);

/** \brief Container for all RGB colors that have meaning during segmentation.
  *
  * This class exists to contain a set of hard-coded RGB color values used during segmentation. The colors are defined in the constructor. If
  * a hand-segmented or machine learning segmented plant is provided and colors don't correspond to these values, behavior will likely be unexpected.
  */
class ColorMap {
    public:
        ColorMap();

        std::map<int, TupleTriplet> _leafColorMap;
        std::map<TupleTriplet, int> _leafColorMap_colorsToLabel;
        std::map<TupleTriplet, int> _map_explicitlyHandledColors;
        TupleTriplet _stem_color;
        TupleTriplet _border_color;
        TupleTriplet _unsegmented_color;
        TupleTriplet _debug_color;
        TupleTriplet _inflorescence_color;
};

//Need to get rid of PointXYZ
typedef std::tuple<float, float, float> TuplePointXYZ;

#endif
