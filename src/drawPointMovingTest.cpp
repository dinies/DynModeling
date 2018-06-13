#include "../include/defs.hpp"
#include "../include/gnuplot-iostream.h"

#include<iostream>
#include <cmath>
#include <boost/tuple/tuple.hpp>
#include <opencv2/plot.hpp>

using namespace dyn_modeling;
using namespace std;


void colorPoint(RGBImage& img, int row , int col){
  int v = 255;
  img.at<cv::Vec3b>(row,col) = cv::Vec3b(v,v,v);
}


int main(int argc, char** argv) {

//  std::cout << "debug\n";
//  std::cout << "PATH=" << getenv("PATH") << std::endl;
  int rows = 80;
  int cols = 80;
  int num_pos = 100;

  Vector2iVector pointPositions(num_pos);

//  cout << "debug\n";
  for (size_t i = 0; i < pointPositions.size(); i++) {
    pointPositions[i] = Eigen::Vector2i(rows * drand48(), cols * drand48());
  }

  RGBImage shown_image;
  shown_image.create(rows, cols);
  shown_image = cv::Vec3b(255, 0, 0);

  cv::namedWindow("distance map");

  //test boost
//  std::cout << "debug\n" << endl;
  std::vector<boost::tuple<double, double, double, double> > pts_A;
  pts_A.push_back(boost::make_tuple(
          1,
          0,
          1,
          1
                  )
  );

  for (size_t j = 0; j < pointPositions.size(); j++) {
    const Eigen::Vector2i &point = pointPositions[j];
    int r = point.x();
    int c = point.y();
    colorPoint(shown_image, r, c);
    cout <<  j;
    cout <<  "\n";
    cv::waitKey(1);
    cv::imshow("distance map", shown_image);
  }


  //Mat usages
  cv::Mat M = cv::Mat( 4 ,4, CV_8UC3, cv::Scalar(0,0,155));
  cout << M << endl;



  //gnuplot
  Gnuplot gp;
  std::vector<std::pair<double, double> > xy_pts_A;
  for(double x=-2; x<2; x+=0.01) {
    double y = x*x*x;
    xy_pts_A.push_back(std::make_pair(x, y));
  }

  std::vector<std::pair<double, double> > xy_pts_B;
  for(double alpha=0; alpha<1; alpha+=1.0/24.0) {
    double theta = alpha*2.0*3.14159;
    xy_pts_B.push_back(std::make_pair(cos(theta), sin(theta)));
  }

  gp << "set xrange [-2:2]\nset yrange [-2:2]\n";
  gp << "plot '-' with lines title 'cubic', '-' with points title 'circle'\n";
  gp.send1d(xy_pts_A);
  gp.send1d(xy_pts_B);





  //plotting
//  vector<double> data = { 3,1,8,9,1,3,3,1};
//  cv::Mat plot_result;
//
//  try {
//    cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(data);
//    plot->render( plot_result);
//    cv::imshow("plot", plot_result);
//    cv::waitKey();
//
//  }
//  catch( cv::Exception ce) {
//    cout << ce.what() << endl;
//  }
}
