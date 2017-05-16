//
//  RectBoxes.cpp
//  WordTracking2
//
//  Created by boyang on 5/10/17.
//  Copyright © 2017 boyang. All rights reserved.
//

#include "RectBoxes.hpp"

int a = 0;
std::queue<CvPoint> RectBoxes::rectBoxCorners;
std::vector<CvPoint> RectBoxes::pointDiff;

struct mySortClass {
    bool operator() (CvPoint pt1, CvPoint pt2) {
        if(pt1.x == pt2.x) {
            return (pt1.y < pt2.y);
        }
        return (pt1.x < pt2.x);
    }
} mySort;

//add rectangle's corners to the list(top left and bottom right corner)
void RectBoxes::addCorner(CvPoint pt) {
    rectBoxCorners.push(pt);
    std::cout<<std::to_string(a++)+"rectBoxCorners Size:"<<rectBoxCorners.size()<<std::endl;
}

//get current size of the queue(rectBoxCorners)
int RectBoxes::getRectCornerSize() {
    return rectBoxCorners.size();
}

CvPoint RectBoxes::popFromRectCorner() {
    CvPoint pt = rectBoxCorners.front();
    rectBoxCorners.pop();
    return pt;
}

//check the point is inside the box or not
bool RectBoxes::insideTheBox(CvPoint topLeft, CvPoint bottomRight, CvPoint pointPreFram) {
    if(pointPreFram.x >= topLeft.x && pointPreFram.x <= bottomRight.x
       && pointPreFram.y >= topLeft.y && pointPreFram.y <=bottomRight.y) {
        return true;
    } else {
        return false;
    }
}

//put the tranlation vector to the list
void RectBoxes::pushDiff(CvPoint pt) {
    pointDiff.push_back(pt);
}

CvPoint RectBoxes::calculateMedianTranslationVec() {
    int size = pointDiff.size();
    if (size < 5) {
        std::cout<<"Not enough features found in box"<<std::endl;
        return CvPoint(-2000,-2000);
    }
    std::sort(pointDiff.begin(), pointDiff.end(), mySort);
    CvPoint ans = pointDiff[size/2];
    pointDiff.clear();
    return ans;
}

