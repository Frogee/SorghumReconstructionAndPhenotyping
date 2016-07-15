#include <tuple>
#include <map>
#include <pcl/point_types.h>
#include <sstream>
#include <iostream>
#include "tupleTriplet.h"
#include "loggingHelper.h"

/** This tests for exact equality of floats, so doesn't work in cases where, say, one (x, y, z) point has been rounded to 3 decimal places
  * and the other one is at 4 decimal places. We should consider modifying this to work within some sort of rounding error.
  */
bool TupleTripletsAreEqual(TupleTriplet point1, TupleTriplet point2) {
    bool allCoordsAreEqual = true;
    if (std::get<0>(point1) != std::get<0>(point2)) {
        allCoordsAreEqual = false;
    }
    else if (std::get<1>(point1) != std::get<1>(point2)) {
        allCoordsAreEqual = false;
    }
    else if (std::get<2>(point1) != std::get<2>(point2)) {
        allCoordsAreEqual = false;
    }
    else {
        //empty else
    }
    return allCoordsAreEqual;
}

//Probably should refactor this to make the TupleTriplet a class with this as constructor.
TupleTriplet convertPclPointXYZtoTupleTriplet(pcl::PointXYZ inputPoint) {
    TupleTriplet constructedTuple(inputPoint.x, inputPoint.y, inputPoint.z);
    return constructedTuple;
}

//Probably should refactor this to make the TupleTriplet a class with this as constructor.
pcl::PointXYZ convertTupleTriplettoPclPointXYZ(TupleTriplet inputTuple) {
    pcl::PointXYZ constructedPCLpoint(std::get<0>(inputTuple), std::get<1>(inputTuple), std::get<2>(inputTuple));
    return constructedPCLpoint;
}

//Refactor this to member function?
void printTupleTriplet(TupleTriplet inputTuple) {
    std::cout << "(" << std::get<0>(inputTuple) << ", " << std::get<1>(inputTuple) << ", " << std::get<2>(inputTuple) << ")" << std::endl;
}

void printTupleTriplet(TupleTriplet inputTuple, uint32_t debuggingLevel, uint32_t debuggingThreshold) {
    std::ostringstream logStream;
    logStream << "(" << std::get<0>(inputTuple) << ", " << std::get<1>(inputTuple) << ", " << std::get<2>(inputTuple) << ")";
    LOG.DEBUG(logStream.str(), debuggingLevel, debuggingThreshold); logStream.str("");
}


/** The ColorMap constructor defines many hardcoded RGB values that are used during segmentation. The most important of these with respect to inputs
  * are the STEM_COLOR, BORDER_COLOR, UNSEGMENTED_COLOR. The remainders correspond to leaves. If a hand-segmented or machine learning segmented plant
  * is provided and colors don't correspond to these values, behavior will likely be unexpected.
  */
ColorMap::ColorMap() {

    TupleTriplet STEM_COLOR(0.0, 255.0, 255.0);
    TupleTriplet BORDER_COLOR(80.0, 80.0, 80.0);
    TupleTriplet UNSEGMENTED_COLOR(255.0, 255.0, 255.0);
    TupleTriplet DEBUG_COLOR(255.0, 20.0, 147.0);
    TupleTriplet INFLORESCENCE_COLOR(255.0, 215.0, 0.0); // gold

    _stem_color = STEM_COLOR;
    _border_color = BORDER_COLOR;
    _unsegmented_color = UNSEGMENTED_COLOR;
    _debug_color = DEBUG_COLOR;
    _inflorescence_color = INFLORESCENCE_COLOR;

    // We're emulating the ggplot2 colors.
    // http://stackoverflow.com/questions/8197559/emulate-ggplot2-default-color-palette
    // and converted the hex to rgb
    TupleTriplet color1(248.0, 118.0, 109.0); //salmon-y #F8766D
    TupleTriplet color2(183.0, 159.0, 0.0); //tan-y #B79F00
    TupleTriplet color3(0.0, 186.0, 56.0); //green-y #00BA38
    //TupleTriplet color4(0.0, 191.0, 196.0); //sea-green-y #00BFC4
    TupleTriplet color4(0.0, 134.0, 125.0); //sea-green-y
    TupleTriplet color5(97.0, 156.0, 255.0); // sky blue-y #619CFF
    TupleTriplet color6(245.0, 100.0, 227.0); // pink-y #F564E3
    TupleTriplet color7(229.0, 135.0, 0.0); //orange-y #E58700
    TupleTriplet color8(185.0, 131.0, 255.0); // purple-y #B983FF

    TupleTriplet color9(222.0, 42.0, 29.0); // red-ish
    TupleTriplet color10(222.0, 29.0, 186.0);
    TupleTriplet color11(103.0, 29.0, 222.0);
    TupleTriplet color12(20.0, 42.0, 222.0);
    /*
    TupleTriplet color5(29.0, 135.0, 222.0);
    TupleTriplet color6(29.0, 222.0, 96.0);
    TupleTriplet color7(148.0, 222.0, 29.0);
    TupleTriplet color8(222.0, 125.0, 29.0);
    TupleTriplet color9(240.0, 178.0, 179.0);
    TupleTriplet color10(176.0, 237.0, 161.0);
    TupleTriplet color11(250.0, 250.0, 17.0);
    TupleTriplet color12(247.0, 168.0, 229.0);
    */
    //New colors added to account for manual segmentation
    TupleTriplet color13(150.0, 150.0, 150.0);
    TupleTriplet color14(0.0, 0.0, 255.0);
    TupleTriplet color15(170.0, 170.0, 255.0);
    TupleTriplet color16(255.0, 0.0, 0.0);
    TupleTriplet color17(255.0, 0.0, 127.0);
    TupleTriplet color18(0.0, 170.0, 0.0);
    TupleTriplet color19(85.0, 170.0, 255.0);
    TupleTriplet color20(255.0, 170.0, 0.0);
    TupleTriplet color21(153.0, 102.0, 51.0);
    TupleTriplet color22(255.0, 170.0, 255.0);
    TupleTriplet color23(170.0, 255.0, 127.0);
    TupleTriplet color24(255.0, 255.0, 127.0);
    TupleTriplet color25(108.0, 108.0, 108.0);
    TupleTriplet color26(237.0, 146.0, 15.0);
    TupleTriplet color27(170.0, 255.0, 0.0);

    _leafColorMap.insert(std::pair<int, TupleTriplet> (1, color1));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (2, color2));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (3, color3));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (4, color4));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (5, color5));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (6, color6));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (7, color7));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (8, color8));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (9, color9));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (10, color10));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (11, color11));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (12, color12));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (13, color13));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (14, color14));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (15, color15));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (16, color16));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (17, color17));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (18, color18));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (19, color19));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (20, color20));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (21, color21));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (22, color22));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (23, color23));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (24, color24));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (25, color25));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (26, color26));
    _leafColorMap.insert(std::pair<int, TupleTriplet> (27, color27));


    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color1, 1));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color2, 2));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color3, 3));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color4, 4));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color5, 5));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color6, 6));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color7, 7));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color8, 8));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color9, 9));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color10, 10));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color11, 11));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color12, 12));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color13, 13));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color14, 14));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color15, 15));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color16, 16));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color17, 17));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color18, 18));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color19, 19));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color20, 20));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color21, 21));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color22, 22));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color23, 23));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color24, 24));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color25, 25));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color26, 26));
    _leafColorMap_colorsToLabel.insert(std::pair<TupleTriplet, int> (color27, 27));

    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color1, 1));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color2, 2));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color3, 3));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color4, 4));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color5, 5));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color6, 6));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color7, 7));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color8, 8));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color9, 9));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color10, 10));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color11, 11));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color12, 12));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color13, 13));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color14, 14));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color15, 15));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color16, 16));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color17, 17));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color18, 18));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color19, 19));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color20, 20));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color21, 21));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color22, 22));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color23, 23));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color24, 24));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color25, 25));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color26, 26));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (color27, 27));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (STEM_COLOR, 28));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (BORDER_COLOR, 29));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (UNSEGMENTED_COLOR, 30));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (DEBUG_COLOR, 31));
    _map_explicitlyHandledColors.insert(std::pair<TupleTriplet, int> (INFLORESCENCE_COLOR, 32));

}


