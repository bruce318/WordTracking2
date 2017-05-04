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
int count = 0;
int cnt_total_valid_point = 0;
const int MAX_CORNERS = 1000;
std::vector<std::vector<CvPoint>> featureList(MAX_CORNERS , std::vector<CvPoint>(0,0));
std::map<CvPoint , int > map;

//functions
//For hashmap
bool operator<(cv::Point const& a, cv::Point const& b)
{
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
}

CvPoint add_tolerance(int x, int y ){
    return CvPoint(x,y);
}

bool checkFeasibility (CvPoint newPoint, int i, std::vector<CvPoint> & reuse, int reuseIt){
    if (map.find(newPoint) != map.end()
        && featureList[map[newPoint]].size() < i*2) {
        int index2 = map[newPoint];
        featureList[index2].push_back(reuse[reuseIt]);
        featureList[index2].push_back(reuse[reuseIt + 1]);
        return true;
    } else {
        return false;
    }
}

//second round check:check the tracking point by adding some tolerance
void second_round_check (std::vector<CvPoint> & reuse,std::vector<CvPoint> & temp, int i){
    for (int reuseIt = 0 ; reuseIt < reuse.size() ; reuseIt+=2) {
        CvPoint originalPoint = reuse[reuseIt];
        CvPoint newPoint;
        newPoint = add_tolerance(originalPoint.x + 1, originalPoint.y);
        if (checkFeasibility(newPoint, i, reuse, reuseIt)){
            continue;
        }
        newPoint = add_tolerance(originalPoint.x - 1, originalPoint.y);
        if (checkFeasibility(newPoint, i, reuse, reuseIt)){
            continue;
        }
        newPoint = add_tolerance(originalPoint.x, originalPoint.y + 1);
        if (checkFeasibility(newPoint, i, reuse, reuseIt)){
            continue;
        }
        newPoint = add_tolerance(originalPoint.x, originalPoint.y - 1);
        if (checkFeasibility(newPoint, i, reuse, reuseIt)){
            continue;
        }
        temp.push_back(reuse[reuseIt]);
        temp.push_back(reuse[reuseIt + 1]);

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


//square function
inline static double square(int a)
{
    return a * a;
}


int main(int argc, const char * argv[]) {
    //read file
    std::vector<cv::String> fileNames;
    String folder = "/Users/boyang/workspace/WordTracking2/src2";
    cv::glob(folder, fileNames);
    
    //loop through all the images in the file
    for(int i = 1 ; i < fileNames.size() - 1 ; i++) {
        //counting all output key point both good and bad one
        int keypoint_cnt = 0;
        //count the numbers of features which are in track
        int cnt_tracking_feature_each_frame = 0;
        //put the duplicate key point
        std::vector<CvPoint> temp;
        //new feature to track record it in the reuse array then use some tolerance to check again
        std::vector<CvPoint> reuse;
        
        //load image
        Mat imgPre = imread(fileNames[i], IMREAD_GRAYSCALE );
        resize(imgPre, imgPre, Size(640, 480));
        Mat imgCur = imread(fileNames[i+1], IMREAD_GRAYSCALE);
        resize(imgCur, imgCur, Size(640, 480));
        //load a color image to show
        Mat imgShow = imread(fileNames[i], IMREAD_COLOR);
        resize(imgShow, imgShow, Size(640, 480));
        
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
            CvPoint p0=cvPoint(cvRound(featuresPre[j].x),cvRound(featuresPre[j].y));
            CvPoint p1=cvPoint(cvRound(featuresCur[j].x),cvRound(featuresCur[j].y));
            line(imgShow,p0,p1,CV_RGB(255,0,0),2);
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
        
        //second round add tolerance seek tracking point
        second_round_check(reuse, temp, i);
        reuse.clear();
        
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
                    featureList[k].push_back(temp[tempIt++]);
                    featureList[k].push_back(temp[tempIt++]);
                } else {
                    featureList[k].push_back(CvPoint(-1 , -1));
                    featureList[k].push_back(CvPoint(-1 , -1));
                    
                }
                
            }
            //put the feature coordinate(not (-1,-1) one) into the map
            if (featureList[k][i*2-1].x != -1 || featureList[k][i*2-1].y != -1) {
                map.emplace(featureList[k][i*2-1] , k);
                
            }
            
        }
        if (tempIt != tempSize) {
            std::cout<<"size not match:"<<tempIt<<"-"<<tempSize<<std::endl;
        }
        //check temp size
//        std::cout<<temp.size()<<std::endl;
        //temp list clear
        temp.clear();
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
    //      cvNamedWindow("ImageA",cv::WINDOW_AUTOSIZE);
    //      cvNamedWindow("ImageB",cv::WINDOW_AUTOSIZE);
        namedWindow("LKpyr_opticalFlow");
    //      cvShowImage("ImageA",imgA);
    //      cvShowImage("ImageB",imgB);
        imshow("LKpyr_opticalFlow",imgShow);
    //    cvSaveImage( "/Users/boyang/Downloads/boyang/1.jpg", imgC );
        
        
        cvWaitKey(0);
    }
//    std::cout<<"total tracked keypoint"<<count<<std::endl;
    std::cout<<"total valid keypoint"<<cnt_total_valid_point<<std::endl;
    //analysis: static of tracking chain
    static_of_tracking_chain (featureList);

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
