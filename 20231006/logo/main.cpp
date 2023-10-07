#include <assert.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>

namespace constants {
const double FPS = 60.0;
const double VIDEO_T = 7.;
}  // namespace constants

namespace math {

double sq(const double& x) {
  return x * x;
}

double get_dis(const cv::Point2d& pt1, const cv::Point2d& pt2) {
  return std::sqrt(math::sq(pt1.x - pt2.x) + math::sq(pt1.y - pt2.y));
}

/*
x1, y1, z1: A 坐标系的坐标轴在 A 系下的表示
x2, y2, z2: B 坐标系的坐标轴在 A 系下的表示
得到 B 系下某坐标转到 A 系下坐标的四元数
*/
Eigen::Quaterniond quaternion_rot(const Eigen::Vector3d& x1,
                                  const Eigen::Vector3d& y1,
                                  const Eigen::Vector3d& z1,
                                  const Eigen::Vector3d& x2,
                                  const Eigen::Vector3d& y2,
                                  const Eigen::Vector3d& z2) {
  Eigen::Matrix3d M =
      x1 * x2.transpose() + y1 * y2.transpose() + z1 * z2.transpose();
  Eigen::Matrix4d N;
  N << M(0, 0) + M(1, 1) + M(2, 2), M(1, 2) - M(2, 1), M(2, 0) - M(0, 2),
      M(0, 1) - M(1, 0), M(1, 2) - M(2, 1), M(0, 0) - M(1, 1) - M(2, 2),
      M(0, 1) + M(1, 0), M(2, 0) + M(0, 2), M(2, 0) - M(0, 2),
      M(0, 1) + M(1, 0), -M(0, 0) + M(1, 1) - M(2, 2), M(1, 2) + M(2, 1),
      M(0, 1) - M(1, 0), M(2, 0) + M(0, 2), M(1, 2) + M(2, 1),
      -M(0, 0) - M(1, 1) + M(2, 2);

  Eigen::EigenSolver<Eigen::Matrix4d> N_es(N);
  Eigen::Vector4d::Index maxIndex;
  N_es.eigenvalues().real().maxCoeff(&maxIndex);

  Eigen::Vector4d ev_max = N_es.eigenvectors().col(maxIndex).real();

  Eigen::Quaterniond quat(ev_max(0), ev_max(1), ev_max(2), ev_max(3));
  quat.normalize();

  return quat;
}

Eigen::Matrix4d get_rotation(Eigen::Vector3d axis, double angle) {
  // Use Rodrigues rotation formula
  Eigen::Matrix4d model = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d M;
  Eigen::Matrix3d Rk;
  Rk << 0, -axis[2], axis[1], axis[2], 0, -axis[0], -axis[1], axis[0], 0;
  M = I + (1 - std::cos(angle)) * Rk * Rk + std::sin(angle) * Rk;
  model << M(0, 0), M(0, 1), M(0, 2), 0, M(1, 0), M(1, 1), M(1, 2), 0, M(2, 0),
      M(2, 1), M(2, 2), 0, 0, 0, 0, 1;
  return model;
}

inline Eigen::Vector2d rotate(const Eigen::Vector2d& vec, const double& angle) {
  Eigen::Matrix2d mat;
  mat << std::cos(angle), -std::sin(angle), std::sin(angle), std::cos(angle);
  return mat * vec;
}

inline double randr(const double& low, const double& up) {
  int randmax = 32768;
  return low + (up - low) * (std::rand() % randmax) / double(randmax);
}

inline bool in_contours(const std::vector<std::vector<cv::Point>>& contours,
                        const cv::Point& pt) {
  for (int i = 0; i < contours.size(); ++i)
    if (cv::pointPolygonTest(contours[i], pt, false) == 1)
      return true;
  return false;
}
}  // namespace math

namespace camera {

class Camera {
 public:
  Camera() {
    this->cam_f << 400., 0., 190., 0., /*\*/
        0., 400., 160., 0.,            /*\*/
        0., 0., 1., 0.;
  }
  const Eigen::Matrix<double, 3, 4>& get_cam_f() const { return this->cam_f; }
  double x(const double& p) const { return 4. - 2. * p * p; }
  double y(const double& p) const { return 3. - 2. * p + 1. * p * p; }
  double z(const double& p) const { return 4. - 4. * p + 2. * p * p; }
  double x_prime(const double& p) const { return -4. * p; }
  double y_prime(const double& p) const { return -2. + 2. * p; }
  double z_prime(const double& p) const { return -2. + 2. * p; }
  // double cam_z_x(const double& p) const { return 2; }
  // double cam_z_y(const double& p) const { return 4. - p; }
  // double cam_z_z(const double& p) const { return 2; }

  // double cam_z_x_prime(const double& p) const { return -1; }
  // double cam_z_y_prime(const double& p) const { return 0.; }
  // double cam_z_z_prime(const double& p) const { return 0.; }

 private:
  Eigen::Matrix<double, 3, 4> cam_f;
};

class Proportion {
 public:
  double p(const double& t) const {
    double res =
        (1. - std::exp(-0.4 * (0.25 * t + t * t) / this->t0)) / this->ful;
    return res > 1. ? 1. : res;
  }

 private:
  double t0 = 1.8;
  double ful = 0.9997;
};

Eigen::Matrix4d get_converter(const camera::Proportion& prop,
                              const camera::Camera& cam,
                              const double& t) {
  double p = prop.p(t);
  Eigen::Vector3d z_cam_w =
      Eigen::Vector3d(cam.x_prime(p), cam.y_prime(p), cam.z_prime(p))
          .normalized();
  Eigen::Vector3d x_cam_w = [&z_cam_w]() {
    Eigen::Vector2d z2 = {z_cam_w(0, 0), z_cam_w(1, 0)};
    z2 = math::rotate(z2, -M_PI / 2.);
    Eigen::Vector3d z3 = {z2(0, 0), z2(1, 0), 0.};
    return z3.normalized();
  }();
  Eigen::Vector3d y_cam_w = [&z_cam_w, &x_cam_w]() {
    Eigen::Matrix4d rot = math::get_rotation(x_cam_w, -M_PI / 2);
    Eigen::Vector4d z4;
    z4 << z_cam_w, 1;
    Eigen::Vector4d y4 = rot * z4;
    return Eigen::Vector3d(y4(0, 0), y4(1, 0), y4(2, 0)).normalized();
  }();
  // q 是相机系坐标转世界系坐标的四元数
  // std::cout << Eigen::Vector3d(cam.x(p), cam.y(p), cam.z(p)) << '\n';
  Eigen::Quaterniond q = math::quaternion_rot(
      {1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}, x_cam_w, y_cam_w, z_cam_w);
  Eigen::Matrix4d converter = [&q, &cam, &p]() {
    Eigen::Vector3d cam_w = {cam.x(p), cam.y(p), cam.z(p)};
    // std::cout << "[cam_w] " << cam_w << '\n' << "[q] " << q << '\n';
    Eigen::Matrix4d converter = Eigen::Matrix4d::Zero();
    Eigen::Matrix3d rot_c_to_w = q.matrix();
    converter.block(0, 0, 3, 3) = rot_c_to_w.transpose().cast<double>();
    converter.block(0, 3, 3, 1) =
        -rot_c_to_w.transpose().cast<double>() * cam_w;
    converter(3, 3) = 1.;
    return converter;
  }();
  return converter;
}
}  // namespace camera

int main() {
  std::srand(std::time(0));
  std::vector<std::vector<cv::Point>> contours = []() {
    cv::Mat src = cv::imread("../test.jpg");
    cv::Mat grey = [&src]() {
      cv::Mat grey;
      cv::cvtColor(src, grey, cv::COLOR_BGR2GRAY);
      return grey;
    }();

    cv::Mat binary;
    cv::threshold(grey, binary, 100, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(binary, contours, hierachy, cv::RETR_TREE,
                     cv::CHAIN_APPROX_NONE);

    cv::Mat with_contours;
    cv::cvtColor(binary, with_contours, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < contours.size(); ++i) {
      cv::drawContours(with_contours, contours, i, {0, 255, 0}, 2);
    }
    return contours;
  }();
  camera::Proportion prop;
  camera::Camera cam;
  // contours.clear();
  std::vector<Eigen::Vector3d> contours_w = [&contours, &cam, &prop]() {
    std::vector<Eigen::Vector3d> contours_w;
    int cnt = 0;
    for (int i = 0; i < contours.size(); ++i) {
      cv::Point2f center = [&contours, &i]() {
        cv::Rect rect = cv::boundingRect(contours[i]);
        return cv::Point2f(rect.x + rect.width / 2., rect.y + rect.height / 2.);
      }();
      for (int j = 0; j < contours[i].size(); ++j) {
        // 534 * 944
        ++cnt;
        if (math::randr(0., 1.) > 1.)
          continue;
        // 像素结果反向生成世界坐标
        // 已知最后 x_w, x_u, y_u，求 x_w, y_w, z_w
        // 以相机为球心，拉一条线出去
        bool extended = false;
        bool out_of_vision = false;
        double ex_x = contours[i][j].x, ex_y = contours[i][j].y;
        double ex_rate = math::randr(1., 1.5);
        if (math::randr(1. - ex_rate, 1.5 - ex_rate) > 0.) {
          double t_x = center.x + (ex_x - center.x) * ex_rate;
          double t_y = center.y + (ex_y - center.y) * ex_rate;
          if (!math::in_contours(contours, cv::Point(t_x, t_y)) &&
              math::get_dis({t_x, t_y}, contours[i][j]) < 150.) {
            extended = true;
            ex_x = t_x, ex_y = t_y;
          }
        }
        double x = ex_x / 300., y = ex_y / 300.;
        double fin_z_c = 1.5;
        double depth = math::randr(-4, 3) + fin_z_c;
        double rate = depth / fin_z_c;
        if (0 < depth && depth < 0.25)
          continue;
        if (depth < 0)
          out_of_vision = true;
        Eigen::Vector4d pc = {x * rate, y * rate, fin_z_c * rate, 1.};
        Eigen::Matrix4d converter = camera::get_converter(prop, cam, 100);
        Eigen::Vector4d pw = converter.inverse() * pc;
        Eigen::Vector3d pw3 = {pw(0, 0), pw(1, 0), pw(2, 0)};
        if (out_of_vision || extended) {
          if (math::randr(0, 1) < 0.5 || depth < 0.5)
            pw3(1, 0) += math::randr(-3, 3), pw3(2, 0) += math::randr(-3, 3);
        }
        contours_w.push_back(pw3);
      }
    }
    return contours_w;
  }();
  {
    std::ofstream ofile;
    ofile.open("../1.txt");
    ofile << contours_w.size() << '\n';
    for (int i = 0; i < contours_w.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
        ofile << contours_w[i](j, 0);
        if (j == 2)
          ofile << '\n';
        else
          ofile << ' ';
      }
    }
  }
  cv::VideoWriter writer("../logo.avi",
                         cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                         constants::FPS, cv::Size(1280, 768), false);
  double t_tot = 0.;
  double t_interval = 1. / constants::FPS;
  while (true) {
    t_tot += t_interval;
    Eigen::Matrix4d converter = camera::get_converter(prop, cam, t_tot);
    cv::Mat img = cv::Mat::zeros(768, 1280, CV_8UC1);
    for (int i = 0; i < contours_w.size(); ++i) {
      Eigen::Vector4d w4;
      w4 << contours_w[i], 1;
      Eigen::Vector4d c4 = converter * w4;
      Eigen::Vector3d u3 = cam.get_cam_f() * c4;
      double depth = u3(2, 0);
      u3 /= u3(2, 0);
      // std::cout << "w4: " << w4 << "\n";
      // std::cout << "c4: " << c4 << "\n";
      // std::cout << "u3: " << u3 << "\n\n";
      if (depth <= 0.)
        continue;
      double radius = depth < 0.5 ? 6 : int(3 / depth);
      cv::circle(img, cv::Point2f(u3(0, 0), u3(1, 0)), radius, {255, 255, 255},
                 -1, cv::LINE_AA);
      // cv::waitKey(1000);
    }
    writer << img;
    if (t_tot >= constants::VIDEO_T)
      break;
    // cv::imshow("res", img);
    // cv::waitKey(int(t_interval * 1000));
  }
  writer.release();
  // Eigen::Vector3d pos_w = {0., 1., 2.};
  // Eigen::Quaterniond q = []() {
  //   Eigen::Vector3d x_w = {1., 0., 0.};
  //   Eigen::Vector3d y_w = {0., 1., 0.};
  //   Eigen::Vector3d z_w = {0., 0., 1.};
  //   Eigen::Vector3d camera_x_w = {0., 1., 0.};
  //   Eigen::Vector3d camera_y_w = {0., 0., -1.};
  //   Eigen::Vector3d camera_z_w = {-1., 0., 0.};
  //   return math::quaternion_rot(x_w, y_w, z_w, camera_x_w, camera_y_w,
  //                               camera_z_w);
  // }();
  // 首先坐标系不旋转，从 o_a_w 移动到 o_b_w
  // 坐标系移动为 o_b_w - o_a_w，其中坐标移动为 o_a_w - o_b_w
  // 已知物体的 pos_w，要得到它的 pos_c
  // 先平移，后旋转
  // Eigen::Vector3d pos_c = q.matrix().transpose().cast<double>() * pos_w;
  // std::cout << "* " << pos_w << "\n* " << pos_c << '\n';
  // cv::waitKey(0);
  return 0;
}