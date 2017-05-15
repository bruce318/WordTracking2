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
    static void pushDiff(CvPoint pt);
    static CvPoint calculateMedianTranslationVec();
    
private:
    static std::queue<CvPoint> rectBoxCorners;
    static std::vector<CvPoint> pointDiff;
    

};

#endif /* RectBoxes_hpp */
