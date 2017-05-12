//
//  RectBoxes.cpp
//  WordTracking2
//
//  Created by boyang on 5/10/17.
//  Copyright © 2017 boyang. All rights reserved.
//

#include "RectBoxes.hpp"


std::queue<CvPoint> RectBoxes::rectBoxCorners;
std::vector<CvPoint> RectBoxes::inBoxPointsPre;
std::vector<CvPoint> RectBoxes::inBoxPointsCur;

//add rectangle's corners to the list(top left and bottom right corner)
void RectBoxes::addCorner(CvPoint pt) {
    rectBoxCorners.push(pt);
    std::cout<<"rectBoxCorners Size:"<<rectBoxCorners.size()<<std::endl;
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

//put the point(previous frame) in to the inBoxPoints vector
void RectBoxes::pushToInBoxPointsPreFrame(CvPoint pt) {
    inBoxPointsPre.push_back(pt);
}

//put the point(current frame) in to the inBoxPoints vector
void RectBoxes::pushToInBoxPointsCurFrame(CvPoint pt) {
    inBoxPointsCur.push_back(pt);
}

CvPoint RectBoxes::calculateMedianPointPrePoints() {
    int size = inBoxPointsPre.size();
    if (size == 0) {
        std::cout<<"No features found in box"<<std::endl;
        return CvPoint(-1,-1);
    }
    int x = 0;
    int y = 0;
    //sum up
    for(int k = 0 ; k < size ; k++) {
        x += inBoxPointsPre[k].x;
        y += inBoxPointsPre[k].y;
    }
    x = x/size;
    y = y/size;
    return CvPoint(x,y);
}

CvPoint RectBoxes::calculateMedianPointCurPoints() {
    int size = inBoxPointsCur.size();
    if (size == 0) {
        std::cout<<"No features found in box"<<std::endl;
        return CvPoint(-1,-1);
    }
    int x = 0;
    int y = 0;
    //sum up
    for(int k = 0 ; k < size ; k++) {
        x += inBoxPointsCur[k].x;
        y += inBoxPointsCur[k].y;
    }
    x = x/size;
    y = y/size;
    return CvPoint(x,y);
}

