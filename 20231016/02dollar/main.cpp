#include<bits/stdc++.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>
using namespace std;
using namespace Eigen;
using namespace cv;
int main()
{
    freopen("../dollar.txt","r",stdin);
    srand(time(0));
    const int N = 50;
    Matrix<double,1,N> data;
    for(int i=0;i<30;++i)cin>>data[i];
    const int Z_N = 1, X_N = 2;
    Matrix<double, X_N, 1> X;
    Matrix<double, X_N, X_N> A; 
    Matrix<double, X_N, X_N> P; 
    Matrix<double, X_N, X_N> R;  
    Matrix<double, X_N, Z_N> K;  
    Matrix<double, Z_N, X_N> C;
    Matrix<double, Z_N, Z_N> Q;
    X << data[0],0;
    A << 1,1,0,1;
    C << 1,0;
    R << 2,0,0,2;
    Q << 10;
    cout<<"amounting rate:"<<endl;
    for(int i=1;i<30;++i)
    {
        //prediction
        Matrix<double, X_N, 1> X_k = A * X;
        P = A*P*A.transpose()+R;
        //update
        K = P * C.transpose() * (C * P * C.transpose() + Q).inverse();
        Matrix<double, Z_N, 1> Z{data[i]};
        X = X_k + K * (Z - C * X_k);
        P = (Matrix<double,X_N,X_N>::Identity() - K * C) * P;
        cout<<"["<<i<<"] "<<X[1]<<endl;
    }
    Matrix<double, X_N, 1> X_k = A * X;
    cout<<"predicted dollar:"<<X_k[0]<<endl;
    return 0;
}
