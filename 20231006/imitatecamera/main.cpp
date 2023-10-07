#include<bits/stdc++.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
using namespace cv;
using namespace Eigen;
using namespace std;
const int N = 4000;
int times=1;
void onTrackbar(int,void*){
    double cx=2., cy=2., cz=2.;
    Quaterniond q={-0.5,0.5,0.5,-0.5};
    Matrix<double,3,4>camera;
    camera <<  400., 0., 190., 0.,
                0., 400., 160., 0.,
                0., 0., 1., 0.;
    Matrix4d converter = [&q,&cx,&cy,&cz](){
        Vector3d tvec = {cx,cy,cz};
        Matrix3d rvec = q.matrix();
        Matrix4d converter = Matrix4d::Zero();
        converter.block(0, 0, 3, 3) = rvec.transpose().cast<double>();
        converter.block(0, 3, 3, 1) = -rvec.transpose().cast<double>()*tvec;
        converter(3,3) = 1;
        return converter;
    }();
    Mat res(720,1280,CV_8UC3);
    imshow("img",res);
    freopen("../points.txt","r",stdin);
    int n;
    cin>>n;
    for(int i=1;i<=n;++i)
    {
        double x,y,z;
        cin>>x>>y>>z;
        Vector4d pt(x,y,z,1);
        Vector4d c4=converter*pt;
        Vector3d cpt = camera*c4;
        circle(res,Point2d(cpt(0,0)/cpt(2,0),cpt(1,0)/cpt(2,0)),1,Scalar(255,255,255),1);
    }
    imshow("img",res);
    imwrite("../jiaolong.jpg",res);
}
int main()
{
    //slide bar
    namedWindow("img");
    createTrackbar("times","img",&times,255,onTrackbar);
    onTrackbar(0,0);
    waitKey(0);
    return 0;
}