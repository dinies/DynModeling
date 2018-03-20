#include "../include/defs.hpp"


using namespace dyn_modeling;
using namespace std;


void colorPoint(RGBImage& img, int row , int col){
  int v = 255;
  img.at<cv::Vec3b>(row,col) = cv::Vec3b(v,v,v);
}


int main(int argc, char** argv){
  int rows=480;
  int cols=640;
  int num_pos=100;

  Vector2iVector pointPositions(num_pos);

  for (size_t i=0; i<pointPositions.size(); i++){
    pointPositions[i]=Eigen::Vector2i(rows*drand48(), cols*drand48());
  }

  RGBImage shown_image;
  shown_image.create(rows, cols);
  shown_image=cv::Vec3b(255,0,0);

  cv::namedWindow("distance map");

  for (size_t j=0; j<pointPositions.size(); j++){
    const Eigen::Vector2i& point=pointPositions[j];
    int r=point.x();
    int c=point.y();
    colorPoint(shown_image, r, c);
    cv::imshow("distance map", shown_image);
    cv::waitKey(1);
  }
}
