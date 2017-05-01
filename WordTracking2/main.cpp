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
const int MAX_CORNERS=1000;
std::vector<std::vector<CvPoint>> featureList(MAX_CORNERS , std::vector<CvPoint>(0,0));
std::map<CvPoint , int > map;

//functions

bool operator<(cv::Point const& a, cv::Point const& b)
{
    return (a.x < b.x) || (a.x == b.x && a.y < b.y);
}

//analysis: static of tracking chain
void static_of_tracking_chain (std::vector<std::vector<CvPoint>> featureList) {
    for (int i = 0 ; i < featureList.size() ; i++) {
        int cnt_tracking_chain = 1;
        for (int j = 2 ; j < featureList[i].size() ; j+=2) {
            if (featureList[i][j].x != -1 && featureList[i][j].x == featureList[i][j-1].x && featureList[i][j].y == featureList[i][j-1].y) {
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
    std::string folder = "/Users/boyang/workspace/WordTracking2/src5";
    cv::glob(folder, fileNames);
    
    for(size_t i = 1 ; i < fileNames.size() - 1 ; i++) {
        int keypoint_cnt = 0;
        int cnt_tracking_feature_each_frame = 0;
        std::vector<CvPoint> temp;
        const char* ch1 = fileNames[i].c_str();
        const char* ch2 = fileNames[i + 1].c_str();
        IplImage* imgA=cvLoadImage(ch1,CV_LOAD_IMAGE_GRAYSCALE);
        IplImage* imgB=cvLoadImage(ch2,CV_LOAD_IMAGE_GRAYSCALE);

        CvSize img_sz=cvGetSize(imgA);
        int win_size=10;
        
        IplImage* imgC=cvLoadImage(ch1,CV_LOAD_IMAGE_UNCHANGED);
        
        
        IplImage* eig_image=cvCreateImage(img_sz,IPL_DEPTH_32F,1);
        IplImage* tmp_image=cvCreateImage(img_sz,IPL_DEPTH_32F,1);
        
        int corner_count=MAX_CORNERS;
        CvPoint2D32f* cornersA=new CvPoint2D32f[MAX_CORNERS];
        cvGoodFeaturesToTrack(//检测角点
                              imgA,
                              eig_image,//两个临时图像
                              tmp_image,
                              cornersA,//函数的输出，即检测到的角点数组
                              &corner_count,//最大角点数，调用函数后返回的角点的数目
                              0.01,
                              5.0,//返回角点之间的最短距离不应小于min_distance
                              0,
                              3,
                              0,
                              0.04
                              );
        cvFindCornerSubPix(//根据上一步精确角点位置，确定亚像素角点
                           imgA,
                           cornersA,//整数值的像素位置
                           corner_count,//角点数目
                           cvSize(win_size,win_size),//等式产生窗口的尺寸
                           cvSize(-1,-1),//禁区，不需要时设置cvSize（-1,-1）
                           cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03)
                           );
        
        char features_found[MAX_CORNERS];
        float feature_errors[MAX_CORNERS];
        CvSize pyr_sz=cvSize(imgA->width+8,imgB->height/3);
        IplImage* pyrA=cvCreateImage(pyr_sz,IPL_DEPTH_8U,1);
        IplImage* pyrB=cvCreateImage(pyr_sz,IPL_DEPTH_8U,1);
        CvPoint2D32f* cornersB=new CvPoint2D32f[MAX_CORNERS];
        cvCalcOpticalFlowPyrLK(
                               imgA,//初始图像
                               imgB,//最终图像
                               pyrA,//申请存放两幅输入图像金字塔的缓存，大小至少为(img.width-8)*img.height/3字节
                               pyrB,
                               cornersA,//用于寻找运动的点
                               cornersB,//存放featureA中点的新的位置
                               corner_count,//featureA中点的数目
                               cvSize(win_size,win_size),
                               5,//金字塔层数
                               features_found,//对应点是否在第二副图像中发现
                               feature_errors,
                               cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.3),
                               0
                               );
        
        for(int j=0;j<corner_count;j++)
        {
            keypoint_cnt++;
            CvPoint p0=cvPoint(cvRound(cornersA[j].x),cvRound(cornersA[j].y));
            CvPoint p1=cvPoint(cvRound(cornersB[j].x),cvRound(cornersB[j].y));
            cvLine(imgC,p0,p1,CV_RGB(255,0,0),2);
            if(i == 1) {
                //not found in second frame or large error or already recorded -> mark(-1,-1)
                if(features_found[j]==0 || feature_errors[j]>50 || map.find(p1) != map.end())
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
                if(features_found[j]==0|| feature_errors[j]>50) {
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
                    //record it in the temp array first and use them to replace the lost feature at the end of each frame
                    temp.push_back(p0);
                    temp.push_back(p1);
                }
            }
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
        std::cout<<"tracked key point:"<<cnt_tracking_feature_each_frame<<" keypointNum"<<keypoint_cnt<<std::endl;
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
        cvNamedWindow("LKpyr_opticalFlow",cv::WINDOW_AUTOSIZE);
    //      cvShowImage("ImageA",imgA);
    //      cvShowImage("ImageB",imgB);
        cvShowImage("LKpyr_opticalFlow",imgC);
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
