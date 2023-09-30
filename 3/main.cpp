#include <bits/stdc++.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;
bool check(const std::vector<Point> &contour)
{
    RotatedRect rect = minAreaRect(contour);
    double  rect_area, contour_area, rect_length, contour_length;
    rect_area = rect.size.area();
    contour_area = contourArea(contour);
    if(contour_area < 200) return false; 
    if(rect_area > 1.5 * contour_area) return false;
    rect_length = (rect.size.height + rect.size.width) * 2;
    contour_length = arcLength(contour, true);
    // cout<<fabs(rect_length - contour_length)/min(rect_length,contour_length)<<" "<<max(rect.size.width, rect.size.height) / min(rect.size.width, rect.size.height) <<"/";
    if(fabs(rect_length - contour_length)/min(rect_length,contour_length)>0.2) return false;
    double time=rect.size.width/rect.size.height;
    if(rect.size.width>10 && rect.size.width<40 && rect.size.height>5 && rect.size.height<20 && fabs(time-2)<0.5) return true;
    return false;
}
int main()
{
    VideoCapture vc("../armor.mp4");
    Mat src;
    vc >> src;
    assert(!src.empty());
    VideoWriter writer("../output.avi",VideoWriter::fourcc('M','J','P','G'),24,Size(1280, src.size[0] / (src.size[1] / 1280.0)),true);
    while(vc.read(src))
    {
        // Mat src=imread("../1223.png");
        Mat res;
        float times = src.size[1] / 1280.0;
        resize(src, res, Size(1280, src.size[0] / times));
        Mat channels[3],red,red2;
        split(res,channels);
        red=channels[2]-(channels[0]+channels[1])/2;
        
        Mat thre;
        threshold(red,thre,30,255,THRESH_BINARY);
        Mat blur;
        GaussianBlur(thre,blur,Size(7,7),0,0);
        Mat ero;
        Mat kernel=getStructuringElement(MORPH_RECT,Size(3,3));
        erode(blur,ero,kernel,Point(-1,-1),1);
        morphologyEx(ero,ero,MORPH_CLOSE,kernel);
        // imshow("ero",ero);
        vector<vector<Point>> contours;
        findContours(ero,contours,RETR_LIST,CHAIN_APPROX_SIMPLE);
        int tri[20],cnt=0,pair[10][2],cntp=0,paired[20];
        for(int i=0;i<contours.size();++i)
            if(check(contours[i]))tri[++cnt]=i;
        for(int i=1;i<=cnt;++i){
            Rect coordination1=boundingRect(contours[tri[i]]);
            int x1=coordination1.x,y1=coordination1.y;
            // cout<<x1<<" "<<y1<<"//";
            // rectangle(res,Rect(x1,y1,coordination1.width,coordination1.height),Scalar(255,0,0),1);
        }
        memset(paired,0,sizeof(paired));
        for(int i=1;i<=cnt;++i){
            if(paired[i])continue;
            bool found=false;
            Rect coordination1=boundingRect(contours[tri[i]]);
            int x1=coordination1.x,y1=coordination1.y;
            for(int j=i+1;j<=cnt && !found;++j){
                Rect coordination2=boundingRect(contours[tri[j]]);
                int x2=coordination2.x,y2=coordination2.y;
                if(abs(x1-x2)<70 && abs(x1-x2)>20 && abs(y1-y2)<20 && coordination1.height+coordination2.height<100){
                    cntp++;
                    if(x1<x2){
                        pair[cntp][0]=tri[i];
                        pair[cntp][1]=tri[j];
                    }
                    else{
                        pair[cntp][0]=tri[j];
                        pair[cntp][1]=tri[i];
                    }
                    found=true;
                    paired[i]=true;
                    paired[j]=true;
                }
            }
        }
        for(int i=1;i<=cntp;++i){
            // Mat hull;
            // approxPolyDP(contours[tri[i]],hull,2,1);
            // polylines(res, hull, 1, (0, 0, 255), 2);
            Rect coordination1=boundingRect(contours[pair[i][0]]);
            Rect coordination2=boundingRect(contours[pair[i][1]]);
            int x1=coordination1.x,y1=coordination1.y,x2=coordination2.x+coordination2.width,y2=coordination2.y+coordination2.height;
            rectangle(res,Rect(x1,y1-10,x2-x1,y2-y1+20),Scalar(0,255,0),2);
        }
        writer<<res;
        //imshow("src",res);
        // waitKey(30);
    }
    writer.release();
    return 0;
}