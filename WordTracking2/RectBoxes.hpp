//
//  RectBoxes.hpp
//  WordTracking2
//
//  Created by boyang on 5/10/17.
//  Copyright © 2017 boyang. All rights reserved.
//

#ifndef RectBoxes_hpp
#define RectBoxes_hpp

#include <stdio.h>
#include <iostream>
#include <queue>
#include "opencv2/core/core.hpp"

class RectBoxes {
public:
    static void addCorner(CvPoint pt);
    static int getRectCornerSize();
    static CvPoint popFromRectCorner();
    static bool insideTheBox(CvPoint topLeft, CvPoint bottomRight, CvPoint pointPreFram);
    static void pushToInBoxPointsPreFrame(CvPoint pt);
    static void pushToInBoxPointsCurFrame(CvPoint pt);
    static CvPoint calculateMedianPointPrePoints();
    static CvPoint calculateMedianPointCurPoints();
    
private:
    static std::queue<CvPoint> rectBoxCorners;
    static std::vector<CvPoint> inBoxPointsPre;
    static std::vector<CvPoint> inBoxPointsCur;
    

};

#endif /* RectBoxes_hpp */
