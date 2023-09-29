#include <bits/stdc++.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;
int color=10;
Mat src,nor;
void onTrackbar(int,void*)
{
    src = imread("../test.png");
    // Mat channels[3];
    // split(src.clone(),channels);
    // channels[1]=channels[2]-channels[1];
    // normalize(channels[1],nor,0.,255,NORM_MINMAX);
    // Mat src1;
    // merge(channels,3,src1);
    //hsv
    Mat hsv;
    cvtColor(src,hsv,COLOR_BGR2HSV);
    Mat red1,red2,yellow;
    inRange(hsv,Scalar(0,43,46),Scalar(25,255,255),red1);
    inRange(hsv,Scalar(156,43,46),Scalar(180,255,255),red2);
    inRange(hsv,Scalar(11,43,46),Scalar(25,255,255),yellow);
    Mat ones_mat = Mat::ones(Size(src.cols,src.rows),CV_8UC1);
    Mat result = red1|red2|yellow ;//255*(ones_mat - (ones_mat-red/255).mul(ones_mat-yellow/255));
    // imshow("hsv",result);
    //threshold
    // Mat thre;
    // threshold(result,thre,thres,255,THRESH_BINARY);
    //open
    Mat open,close;
    Mat kernel = getStructuringElement(MORPH_RECT,Size(3,3));
    morphologyEx(result,open,MORPH_OPEN,kernel,Point(-1,-1),18);
    //morphologyEx(open,open,MORPH_CLOSE,kernel,Point(-1,-1),iter2);
    //contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierachy;
    findContours(open,contours,RETR_LIST,CHAIN_APPROX_NONE);
    //select
    int maxi=-1;
    double maxs=0;
    for(int i=0;i<contours.size();++i){
        double s=contourArea(contours[i]);
        if(s>maxs){
            maxi=i;
            maxs=s;
        }
    }
    if(maxi==-1)cout<<"Not found"<<endl;
    else{
        //draw
        Rect rect = boundingRect(contours[maxi]);
        drawContours(src,contours,maxi,Scalar(0,255,255),1);
        rectangle(src,rect,Scalar(255,255,0),2);
        imshow("img",src);
        // waitKey(0);
    }
}
int main()
{
    //slide bar
    namedWindow("img");
    createTrackbar("color","img",&color,255,onTrackbar);
    onTrackbar(0,0);
    waitKey(0);
    return 0;
}