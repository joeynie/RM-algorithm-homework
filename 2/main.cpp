#include <bits/stdc++.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
using namespace std;
void process(int t)
{
    string str = " ";
    str[0] = t + '0';
    Mat src = imread("../plates/00" + str + ".jpg");
    Mat res;
    float times = src.size[1] / 640.0;
    resize(src, res, Size(640, src.size[0] / times));
    Mat channels[3];
    split(res.clone(), channels);
    channels[0] = channels[0] - channels[2];
    channels[1] -= channels[1];
    channels[2] -= channels[2];
    Mat src1;
    merge(channels, 3, src1);
    Mat thre;
    threshold(channels[0], thre, 100, 255, THRESH_BINARY);
    // Mat hsv;
    // cvtColor(res,hsv,COLOR_BGR2HSV);
    // Mat blue;
    // inRange(hsv,Scalar(100,43,46),Scalar(124,255,255),blue);
    Mat blur;
    medianBlur(thre, blur, 5);
    Mat close;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(blur, close, MORPH_OPEN, kernel, Point(-1, -1), 1);
    morphologyEx(close, close, MORPH_CLOSE, kernel, Point(-1, -1), 7);
    vector<vector<Point>> contours;
    findContours(close, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    int maxi = 0;
    double maxs = 0;
    for (int i = 0; i < contours.size(); ++i)
    {
        double s = contourArea(contours[i]);
        if (s > maxs)
        {
            maxi = i;
            maxs = s;
        }
    }
    Mat hull;
    approxPolyDP(contours[maxi], hull, 12, true);
    polylines(res, hull, 1, (255, 0, 255), 2);
    imshow("res", res);
    waitKey(0);
}
int main()
{
    for (int i = 1; i <= 5; ++i)
        process(i);
    return 0;
}