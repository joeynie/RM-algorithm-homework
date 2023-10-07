#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
using namespace std;
using namespace cv;
int main() {
    const int board_w = 9, board_h = 6;
    const int board_n = board_w * board_h;
    Size board_size( 9, 6 );//number of corner points, not checks
    Mat gray_img, drawn_img;
    vector< Point2f > point_pix_pos_buf;
    vector< vector<Point2f> > point_pix_pos;
    int found, successes = 0;
    Size img_size;
    int cnt = 0;
    int k = 0, n = 0;
    for (int i = 0; i <= 40; i++){
        string str;
        if(i<10){str+=char(i+'0');}
        else{
            str+=char(i/10+'0');
            str+=char(i%10+'0');
        }
        cv::Mat src0 = cv::imread("../chess/"+str+".jpg");
        if ( !cnt ) {
            img_size.width = src0.cols;
            img_size.height = src0.rows;
        }
        found = findChessboardCorners( src0, board_size, point_pix_pos_buf );//look for corners
        if ( found && point_pix_pos_buf.size() == board_n ) {//match
            successes++;
            cvtColor( src0, gray_img, COLOR_BGR2GRAY );
            find4QuadCornerSubpix( gray_img, point_pix_pos_buf, Size( 5, 5 ) );//break down pixels into several smaller parts
            point_pix_pos.push_back( point_pix_pos_buf );
            //draw
            drawn_img = src0.clone();
            drawChessboardCorners( drawn_img, board_size, point_pix_pos_buf, found );
            imshow( "corners", drawn_img );
            waitKey( 50 );
        } else
            cout << "\tbut failed to found all chess board corners in this image" << endl;
        point_pix_pos_buf.clear();
        cnt++;
    };
    cout << successes << " useful chess boards" << endl;
    //calibrate
    Size square_size( 10, 10 );
    vector< vector< Point3f > > point_grid_pos;
    vector< Point3f > point_grid_pos_buf;
    vector< int > point_count;
    Mat camera_matrix( 3, 3, CV_32FC1, Scalar::all( 0 ) );
    Mat dist_coeffs( 1, 5, CV_32FC1, Scalar::all( 0 ) );
    vector< Mat > rvecs;
    vector< Mat > tvecs;
    for (int i = 0; i < successes; i++ ) {
        for (int j = 0; j < board_h; j++ ) {
            for (int k = 0; k < board_w; k++ ){
                Point3f pt;
                pt.x = k * square_size.width;//coordination
                pt.y = j * square_size.height;
                pt.z = 0;
                point_grid_pos_buf.push_back( pt );
            }
        }
        point_grid_pos.push_back( point_grid_pos_buf );
        point_grid_pos_buf.clear();
        point_count.push_back( board_h * board_w );
    }
    cout << calibrateCamera( point_grid_pos, point_pix_pos, img_size, camera_matrix, dist_coeffs, rvecs, tvecs ) << endl;
    cout << camera_matrix << endl << dist_coeffs << endl;
    return 0;
}
