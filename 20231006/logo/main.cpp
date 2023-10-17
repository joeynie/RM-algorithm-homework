#include<bits/stdc++.h>
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<eigen3/Eigen/Geometry>
using namespace std;
using namespace Eigen;
using namespace cv;
int n;
vector<Vector4d> pt;
int x=0,y=0,z=0;
int rx=0,ry=0,rz=0;
void onTrackbar(int, void*)
{
    Vector3d tvec = {x/50.,y/50.,z/50.};
    Quaterniond q1={-0.5,0.5,0.5,-0.5};
    Quaterniond q2=AngleAxisd(rz*1.*M_PI/180, Vector3d::UnitZ())*
                AngleAxisd(ry*1.*M_PI/180, Vector3d::UnitY())*
                AngleAxisd(rx*1.*M_PI/180, Vector3d::UnitX());
    Matrix<double,3,4>camera;
    camera <<  400., 0., 190., 0.,
                0., 400., 160., 0.,
                0., 0., 1., 0.;
    
    Matrix4d converter = Matrix4d::Zero();
    Matrix3d rvec = (q2*q1).matrix();
    converter.block(0, 0, 3, 3) = rvec.transpose().cast<double>();
    converter.block(0, 3, 3, 1) = -rvec.transpose().cast<double>()*tvec;
    converter(3,3) = 1;
    Mat res=Mat::zeros(720,1280,CV_8UC3);
    for(int i=0;i<n;++i)
    {
        Vector4d c4 = converter*pt[i];
        Vector3d cpt = camera*c4;
        circle(res,Point2d(cpt(0,0)/cpt(2,0),cpt(1,0)/cpt(2,0)),1,Scalar(255,255,255),1);
    }
    imshow("control",res);
    waitKey(50);
}
int main()
{
    freopen("../1.txt","r",stdin);
    cin>>n;
    for(int i=1;i<=n;++i)
    {
        double x,y,z;
        cin>>x>>y>>z;
        pt.push_back(Vector4d(x,y,z,1));
    }
    namedWindow("control");
    createTrackbar("x","control",&x,200,onTrackbar);
    createTrackbar("y","control",&y,200,onTrackbar);
    createTrackbar("z","control",&z,200,onTrackbar);
    createTrackbar("rx","control",&rx,360,onTrackbar);
    createTrackbar("ry","control",&ry,360,onTrackbar);
    createTrackbar("rz","control",&rz,360,onTrackbar);
    // onTrackbar(0,0);
    // waitKey();
    VideoWriter writer("../logo2.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 20, Size(1280, 720), true);
    Vector3d start_t(0,-2,-1);
    Vector3d start_r(30,180,60);
    Vector3d end_t(2,2,2);
    Vector3d end_r(0,0,0);
    for(double t=0.;t<=300;++t){
        if(t<100)t+=1.5;
        else if(t>295) t-=0.95;
        else if(t>250) t-=0.5;
        else if(t>175) t-=0.1;
        //internal reference
        Matrix<double,3,4>camera;
        camera <<  400., 0., 190., 0.,
                    0., 400., 160., 0.,
                    0., 0., 1., 0.;
        //external reference
        Quaterniond q0={-0.5,0.5,0.5,-0.5};
        Vector3d euler=((300-t)*start_r+t*end_r)/300;
        Quaterniond q1=AngleAxisd(euler[2]*M_PI/180, Vector3d::UnitZ())*
                        AngleAxisd(euler[1]*M_PI/180, Vector3d::UnitY())*
                        AngleAxisd(euler[0]*M_PI/180, Vector3d::UnitX());;
        Vector3d tvec = ((300-t)*start_t+t*end_t)/300.;
        Matrix3d rvec = (q1*q0).matrix();
        Matrix4d converter = Matrix4d::Zero();
        converter.block(0, 0, 3, 3) = rvec.transpose().cast<double>();
        converter.block(0, 3, 3, 1) = -rvec.transpose().cast<double>()*tvec;
        converter(3,3) = 1;
        Mat res=Mat::zeros(720,1280,CV_8UC3);
        for(int i=0;i<n;++i)
        {
            Vector4d c4 = converter*pt[i];
            Vector3d cpt = camera*c4;
            circle(res,Point2d(cpt(0,0)/cpt(2,0),cpt(1,0)/cpt(2,0)),1,Scalar(255,255,255),1);
        }
        // imshow("img",res);
        // waitKey(30);
        writer << res;
    }
    writer.release();
    return 0;
    // imwrite("../jiaolong.jpg",res);
}