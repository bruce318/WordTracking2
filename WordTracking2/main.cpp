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


//global var
const int MAX_CORNERS=550;
std::vector<std::vector<CvPoint>> featureList(MAX_CORNERS , std::vector<CvPoint>(0,0));

//square function
inline static double square(int a)
{
    return a * a;
}


int main(int argc, const char * argv[]) {
    
    IplImage* imgA=cvLoadImage("/Users/boyang/workspace/WordTracking2/src1/1.JPG",CV_LOAD_IMAGE_GRAYSCALE);
    IplImage* imgB=cvLoadImage("/Users/boyang/workspace/WordTracking2/src1/2.JPG",CV_LOAD_IMAGE_GRAYSCALE);
    
    CvSize img_sz=cvGetSize(imgA);
    int win_size=10;
    
    IplImage* imgC=cvLoadImage("/Users/boyang/workspace/WordTracking2/src1/2.JPG",CV_LOAD_IMAGE_UNCHANGED);
    
    
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
    
    for(int i=0;i<corner_count;i++)
    {
        if(features_found[i]==0||feature_errors[i]>550)
        {
            printf("error is %f/n",feature_errors[i]);
            featureList[i].push_back(CvPoint(-1 , -1));
            continue;
        }
         printf("got it/n");
        CvPoint p0=cvPoint(cvRound(cornersA[i].x),cvRound(cornersA[i].y));
        CvPoint p1=cvPoint(cvRound(cornersB[i].x),cvRound(cornersB[i].y));
        cvLine(imgC,p0,p1,CV_RGB(255,0,0),2);
        
        
        featureList[i].push_back(p0);
        
        
        
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
        
    }
    //  cvNamedWindow("ImageA",cv::WINDOW_AUTOSIZE);
    //  cvNamedWindow("ImageB",cv::WINDOW_AUTOSIZE);
    cvNamedWindow("LKpyr_opticalFlow",cv::WINDOW_AUTOSIZE);
    //  cvShowImage("ImageA",imgA);
    //  cvShowImage("ImageB",imgB);
    cvShowImage("LKpyr_opticalFlow",imgC);
    cvSaveImage( "/Users/boyang/Downloads/boyang/1.jpg", imgC );
    
    
    cvWaitKey(0);
    return 0;
    
}
