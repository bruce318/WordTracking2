//
//  main.cpp
//  WordTracking2
//
//  Created by boyang on 4/19/17.
//  Copyright © 2017 boyang. All rights reserved.
//

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "stdio.h"
#include "math.h"
#include <iostream>
#include <map>

using namespace cv;

//global var
const int MAX_CORNERS = 2000;
const int TOLERENCE_WINSIZE = 10;//half of the winsize eg. 3 means winsize is 7
const int SSD_WINSIZE = 3;//half of the winsize eg. 5 means winsize is 11
const double SSD_THRESHOLD = 4;
const Size imgSize = Size(640,480);//640, 480


int cntAddByTolerance = 0;
int count = 0;
int cntTolerancePerformance = 0;
int cnt_total_valid_point = 0;

Scalar chainLengthColor[8] = {Scalar(0,0,255),Scalar(0,153,255),Scalar(0,255,255),Scalar(0,255,0),Scalar(255,255,0),Scalar(255,0,0),Scalar(255,0,153),Scalar(0,0,0)};//rainbow order

Mat imgPrePre;
Mat imgPre;
Mat imgCur;
std::vector<std::vector<CvPoint>> featureList(MAX_CORNERS , std::vector<CvPoint>(0,0));
std::map<CvPoint , int > map;
std::vector<CvPoint> reuse2;
std::vector<std::vector<int> > trackingTable;//a table to keep a record of tracking


//functions
//For hashmap
bool operator<(cv::Point const& a, cv::Point const& b)
{
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
}

//square function
inline static double square(int a)
{
    return a * a;
}

CvPoint add_tolerance(int x, int y ){
    return CvPoint(x,y);
}

bool checkFeasibility (CvPoint newPoint, int i, std::vector<CvPoint> & reuse, int reuseIt, std::vector<int> & trackingTableThisFrame){
    if (map.find(newPoint) != map.end()
        && featureList[map[newPoint]].size() < i*2) {
        int index2 = map[newPoint];
        featureList[index2].push_back(reuse[reuseIt]);
        featureList[index2].push_back(reuse[reuseIt + 1]);
        cntTolerancePerformance++;
        //keep record on the tracking table
        trackingTableThisFrame[index2] = trackingTable[i - 2][index2] + 1;
        return true;
    } else {
        return false;
    }
}

//second round check:check the tracking point by adding some tolerance
void second_round_check (std::vector<CvPoint> & reuse,std::vector<CvPoint> & temp, int i, std::vector<int> & trackingTableThisFrame){
    for (int reuseIt = 0 ; reuseIt < reuse.size() ; reuseIt+=2) {
        CvPoint originalPoint = reuse[reuseIt];
        CvPoint newPoint;
        newPoint = add_tolerance(originalPoint.x + 1, originalPoint.y);
        if (checkFeasibility(newPoint, i, reuse, reuseIt, trackingTableThisFrame)){
            continue;
        }
        newPoint = add_tolerance(originalPoint.x - 1, originalPoint.y);
        if (checkFeasibility(newPoint, i, reuse, reuseIt, trackingTableThisFrame)){
            continue;
        }
        newPoint = add_tolerance(originalPoint.x, originalPoint.y + 1);
        if (checkFeasibility(newPoint, i, reuse, reuseIt, trackingTableThisFrame)){
            continue;
        }
        newPoint = add_tolerance(originalPoint.x, originalPoint.y - 1);
        if (checkFeasibility(newPoint, i, reuse, reuseIt, trackingTableThisFrame)){
            continue;
        }
        //didn't find a point to continue tracking even add 1 pixel tolerence
        //first frame no need third round check since don't have previous frame
        if(i == 1){
            temp.push_back(reuse[reuseIt]);
            temp.push_back(reuse[reuseIt + 1]);
        } else {//ready to process third round check
            reuse2.push_back(reuse[reuseIt]);
            reuse2.push_back(reuse[reuseIt + 1]);
        }

    }
}

bool checkOutOfBound (CvPoint thisPoint) {
    if(thisPoint.x - SSD_WINSIZE < 0
       || thisPoint.x + SSD_WINSIZE >= imgSize.width
       || thisPoint.y -SSD_WINSIZE < 0
       || thisPoint.y + SSD_WINSIZE >= imgSize.height
       ) {
        return true;
    }
    return false;
}

int ssd(CvPoint firstPoint, CvPoint secondPoint, int flag){
    Mat img1 = imgPrePre;
    Mat img2;
    if(flag == 1) {
        img2 = imgPre;
    } else if (flag == 2) {
        img2 = imgCur;
    }
    int sum = 0;
    if(checkOutOfBound(firstPoint) || checkOutOfBound(secondPoint)){
        return -1;
    }
    for(int y = - SSD_WINSIZE ; y <= SSD_WINSIZE ; y++) {
        for(int x = -SSD_WINSIZE ; x <= SSD_WINSIZE ; x++) {
            Scalar intensity1 = img1.at<uchar>(firstPoint.x + x, firstPoint.y + y);
            Scalar intensity2 = img2.at<uchar>(secondPoint.x + x, secondPoint.y + y);
            
            sum += square(intensity1.val[0] - intensity2.val[0]);
            //check
            //std::cout<<"1:"<<intensity1.val[0]<<" 2:"<<intensity2.val[0]<<std::endl;
        }
    }
    return sum;
}

void thirdRoundCheck(int i, std::vector<CvPoint> & temp, std::vector<int> & trackingTableThisFrame) {
    //loop through list:reuse2 which are non tracking new points. check them whether can they connect to the tracking chain
    for(int reuse2It = 0 ; reuse2It < reuse2.size() ; reuse2It+=2) {
        CvPoint startPoint = reuse2[reuse2It];
        CvPoint endPoint = reuse2[reuse2It + 1];
        double minRatio = DBL_MAX;
        int indexToBeUse = -1;
        for(int dy = startPoint.y - TOLERENCE_WINSIZE ; dy <= startPoint.y + TOLERENCE_WINSIZE ; dy++ ) {
            for (int dx = startPoint.x - TOLERENCE_WINSIZE ; dx <= startPoint.x + TOLERENCE_WINSIZE ; dx++) {
                CvPoint pointWithTolerance = CvPoint(dx,dy);//also the end point in the previous frame
                CvPoint preStartPoint;
                if(map.find(pointWithTolerance) != map.end()) {
                    int index3 = map[pointWithTolerance];
                    int size = featureList[index3].size();
                    //check whether already have tracking chain or not
                    if (size < i*2){
                        //check if it is the right one
                        if (featureList[index3][size - 1].x == pointWithTolerance.x
                            && featureList[index3][size - 1].y == pointWithTolerance.y) {
                            //index of the start point in the previous frame is size - 2
                            preStartPoint = featureList[index3][size - 2];
                            int U_0 = ssd(preStartPoint, pointWithTolerance, 1);
                            int U_1 = ssd(preStartPoint, endPoint, 2);
                            double ratio = DBL_MAX;
                            if(U_0 == 0) {
                                if(U_1 == 0) {
                                    ratio = 1;
                                }
                            } else if (U_0 != -1 && U_1 != -1) {
                                ratio = U_1/U_0;
                            }
                            if ( ratio < SSD_THRESHOLD && ratio < minRatio) {
                                minRatio = ratio;
                                indexToBeUse = index3;
                            }
                            
                        } else {
                            std::cout<<"not find the right point in previous frame"<<std::endl;
                        }
                    }
                    
                    
                }
            }
        }
        //if been modified -> have good matches. check whether it already have tracking point or not
        if (minRatio != DBL_MAX && featureList[indexToBeUse].size() < i*2) {
            featureList[indexToBeUse].push_back(startPoint);
            featureList[indexToBeUse].push_back(endPoint);
            cntAddByTolerance++;
            trackingTableThisFrame[indexToBeUse] = trackingTable[i - 2][indexToBeUse] + 1;
            
        } else {
            temp.push_back(startPoint);
            temp.push_back(endPoint);
        }
    }
}

//analysis: static of tracking chain
void static_of_tracking_chain (std::vector<std::vector<CvPoint>> featureList) {
    for (int i = 0 ; i < featureList.size() ; i++) {
        int cnt_tracking_chain = 1;
        for (int j = 2 ; j < featureList[i].size() ; j+=2) {
            if (featureList[i][j].x != -1
                && (featureList[i][j].x >= featureList[i][j-1].x-1
                    && featureList[i][j].x <= featureList[i][j-1].x+1)
                && (featureList[i][j].y >= featureList[i][j-1].y-1)
                    && featureList[i][j].y <= featureList[i][j-1].y+1) {
                cnt_tracking_chain++;
            } else {
                if (cnt_tracking_chain != 1){
                    std::cout<<cnt_tracking_chain<<std::endl;
                }
                cnt_tracking_chain = 1;
            }
        }
    }
}

//analysis: static of tracking chain by tracking table
void analysis() {
    for (int j = 0 ; j < trackingTable[0].size() ; j++) {
        int cntChainLength = 1;//start from 1. since one keypoint count as 1
        for (int i = 0 ; i < trackingTable.size() ; i++) {
            if(trackingTable[i][j] == 1) {
                cntChainLength++;
            } else {
                if(cntChainLength != 1){//don't output the chain length == 1
                    std::cout<<cntChainLength<<std::endl;
                }
                cntChainLength = 1;
            }
        }
    }
}


int main(int argc, const char * argv[]) {
    //some var
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);

    
    //read file
    std::vector<cv::String> fileNames;
    String folder = "/Users/boyang/workspace/WordTracking2/src6";
    cv::glob(folder, fileNames);
    
    //loop through all the images in the file
    for(int i = 1 ; i < fileNames.size() - 1 ; i++) {
        //counting all output key point both good and bad one
        int keypoint_cnt = 0;
        //count the numbers of features which are in track
        int cnt_tracking_feature_each_frame = 0;
        //put the duplicate and new key point
        std::vector<CvPoint> temp;
        //new feature to track record it in the reuse array then use some tolerance to check again
        std::vector<CvPoint> reuse;
        //create tracking table for this frame
        std::vector<int> trackingTableThisFrame(MAX_CORNERS, 0);
        
        //load image
        imgPre = imread(fileNames[i], IMREAD_GRAYSCALE );
        resize(imgPre, imgPre, imgSize);
        imgCur = imread(fileNames[i+1], IMREAD_GRAYSCALE);
        resize(imgCur, imgCur, imgSize);
        //load a color image to show
        Mat imgShow = imread(fileNames[i], IMREAD_COLOR);
        resize(imgShow, imgShow, imgSize);
        
        int corner_count=MAX_CORNERS;
        //set it later
        int win_size;
        //vector to put output feature points
        std::vector<Point2f> featuresPre(MAX_CORNERS);
        std::vector<Point2f> featuresCur(MAX_CORNERS);
        
        //find good features to track
        goodFeaturesToTrack(imgPre,
                            featuresPre,
                            MAX_CORNERS,
                            0.01,
                            5.0,
                            Mat(),
                            3,
                            0,
                            0.04
                            );
        
        cornerSubPix(imgPre,
                     featuresPre,
                     subPixWinSize,
                     Size(-1,-1),
                     termcrit
                     );
        
        //output the status and errors of feature point
        std::vector<uchar> status;
        std::vector<float> error;
        
        //optical flow
        calcOpticalFlowPyrLK(imgPre,
                             imgCur,
                             featuresPre,
                             featuresCur,
                             status,
                             error
                             );
        
        for(int j=0;j<corner_count;j++)
        {
            keypoint_cnt++;
            //start point and end point of the optical flow
            CvPoint p0=cvPoint(cvRound(featuresPre[j].x),cvRound(featuresPre[j].y));
            CvPoint p1=cvPoint(cvRound(featuresCur[j].x),cvRound(featuresCur[j].y));
            //draw line of the optical flow
//            line(imgShow,p0,p1,CV_RGB(255,0,0),2);
            
            //if is the first frame
            if(i == 1) {
                //not found in second frame or large error or already recorded -> mark(-1,-1)
                if(status[j]==0 || error[j]>50 || map.find(p1) != map.end())
                {
                    featureList[j].push_back(CvPoint(-1 , -1));
                    featureList[j].push_back(CvPoint(-1 , -1));
                } else {
                    //mark it in the map
                    map.emplace(p1 , j);
                    //record the feature's coordinate
                    featureList[j].push_back(p0);
                    featureList[j].push_back(p1);
                }
            //not the first frame
            } else {
                //if not found in second frame or large error->record it in the temp array first and establish the lost feature by them at the end of each frame so that the total number of featureList won't change.(consistancy)
                if(status[j]==0|| error[j]>50) {
                    temp.push_back(CvPoint(-1 , -1));
                    temp.push_back(CvPoint(-1 , -1));
                } else if (map.find(p0) != map.end()) {//if the feature coordinate match one of the feature's end point in last frame -> connect them
                    int index = map[p0];
                    //avoid duplicate point(not duplicate)
                    if(featureList[index].size() < i*2) {
                        count++;
                        cnt_tracking_feature_each_frame++;
                        featureList[index].push_back(p0);
                        featureList[index].push_back(p1);
//                        std::cout<<index<<"-"<<featureList[index].size() - 2<<std::endl;
//                        std::cout<<"x="<<p0.x<<"y="<<p0.y<<std::endl;
                        //record on the tracking table
                        trackingTableThisFrame[index] = trackingTable[i - 2][index] + 1;
                    
                    } else {//duplicate
                        temp.push_back(CvPoint(-1 , -1));
                        temp.push_back(CvPoint(-1 , -1));
                    }
                    
                } else {//new feature to track
                    //record it in the reuse array then use some tolerance to check again
                    reuse.push_back(p0);
                    reuse.push_back(p1);
                }
            }
        }
        

        if (i > 1) {
            //second round add tolerance seek tracking point
            second_round_check(reuse, temp, i, trackingTableThisFrame);
            reuse.clear();
            
            thirdRoundCheck(i, temp, trackingTableThisFrame);
            reuse2.clear();
        }
        
        //clear map
        map.clear();
        
        //replace the lost tracking point by the temp array's point(new feature to track and some invalid points). Also renew the hash map
        int tempIt = 0;//temp iterator
        size_t tempSize = temp.size();//mark the temp size
        for (int k = 0 ; k < featureList.size() ; k++) {
            //size!=i*2 means didn't renew in this frame
            if(featureList[k].size() != i*2) {
                //sometimes we set max corner to detect, but computer didn't find so many corner feature
                if (tempIt < tempSize) {
                    //avoid duplicate
                    if(map.find(tempIt + 1) == map.end()){
                        featureList[k].push_back(temp[tempIt++]);
                        featureList[k].push_back(temp[tempIt++]);
                    } else {
                        featureList[k].push_back(CvPoint(-1 , -1));
                        featureList[k].push_back(CvPoint(-1 , -1));
                        tempIt += 2;
                    }
                    
                } else {
                    featureList[k].push_back(CvPoint(-1 , -1));
                    featureList[k].push_back(CvPoint(-1 , -1));
                    
                }
                
            }
            //put the feature coordinate(not (-1,-1) one) into the map
            //and draw circles on the feature points. Color depends on the chain length
            if (featureList[k][i*2-1].x != -1 || featureList[k][i*2-1].y != -1) {
                map.emplace(featureList[k][i*2-1] , k);
                int colorIndex = trackingTableThisFrame[k]>8?8:trackingTableThisFrame[k];
                Scalar circleColor = chainLengthColor[colorIndex];
                circle(imgShow, featureList[k][i*2 - 2], 3, circleColor, 1);
                
            }
            
        }
        if (tempIt != tempSize) {
            std::cout<<"size not match:"<<tempIt<<"-"<<tempSize<<std::endl;
        }
        //check temp size
//        std::cout<<temp.size()<<std::endl;
        
        //push back the tracking table for this frame
        trackingTable.push_back(trackingTableThisFrame);
        
        imgPrePre.release();
        //copy to previous frame
        imgPre.copyTo(imgPrePre);
        //clear
        temp.clear();
        trackingTableThisFrame.clear();
        imgPre.release();
        imgCur.release();
        
        
        //check the number of tracked key point
//        std::cout<<"tracked key point:"<<cnt_tracking_feature_each_frame<<" keypointNum"<<keypoint_cnt<<std::endl;
        //check the number of valid keypoint
//        std::cout<<"valid key point:"<<map.size()<<std::endl;
        cnt_total_valid_point += map.size();
        
        
        //For testing - search
    //    for(int i = 0 ; i < featureList.size() ; i++){
    //        for(int j = 0 ; j < featureList[i].size() ; j++){
    //            if(featureList[i][j].x == 817 && featureList[i][j].y == 549){
    //                std::cout<< "found:" <<i<<","<<j<< std::endl;
    //            }
    //        }
    //    }

        namedWindow("LKpyr_opticalFlow");
        imshow("LKpyr_opticalFlow",imgShow);
        cvWaitKey(1);
    }

    
    //analysis: static of tracking chain
    //static_of_tracking_chain (featureList);

    
    //analysis: static of tracking chain by tracking table
//    analysis();
    
    std::cout<<"addBy1PixelTorlerance"<<cntTolerancePerformance<<std::endl;
    std::cout<<"addByTolerance"<<cntAddByTolerance<<std::endl;
    //    std::cout<<"total tracked keypoint"<<count<<std::endl;
    std::cout<<"total valid keypoint"<<cnt_total_valid_point<<std::endl;

    return 0;
    
}

/*
 int line_thickness;  line_thickness=1;
 
 CvScalar line_color;  line_color = CV_RGB(255, 0, 0);
 
 CvPoint p,q;
 p.x = (int) cornersA[i].x;
 p.y = (int) cornersA[i].y;
 q.x = (int) cornersB[i].x;
 q.y = (int) cornersB[i].y;
 
 double angle;  angle= atan2((double) p.y-q.y, (double) p.x-q.x);
 double hypotenuse; hypotenuse= sqrt(square(p.y-q.y) + square(p.x-q.x));
 double sum;
 int n;
 n=n+1;
 sum=sum+hypotenuse;
 //        printf("sum is %f/n",sum);
 //        printf("num is %d/n",n);
 q.x = (int) (p.x-1.5*hypotenuse*cos(angle));
 q.y = (int) (p.y-1.5*hypotenuse*sin(angle));
 
 cvLine(imgC, p, q, CV_RGB(0, 0, 250), line_thickness, CV_AA, 0);
 
 //  p.x = (int) (q.x+9*cos(angle+pi/4));
 //  p.y = (int) (q.y+9*sin(angle+pi/4));
 //  cvLine(imgC, p, q, line_color, line_thickness, CV_AA, 0);
 //  p.x = (int) (q.x+9*cos(angle-pi/4));
 //  p.y = (int) (q.y+9*sin(angle-pi/4));
 //  cvLine(imgC, p, q, line_color, line_thickness, CV_AA, 0);
 */
