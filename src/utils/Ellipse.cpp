// code taken from http://www.visiondummy.com/wp-content/uploads/2014/04/error_ellipse.cpp
#include "Ellipse.hpp"

namespace dyn_modeling {


  cv::RotatedRect Ellipse::getEllipse(double chisquare_val, cv::Point2f mean, cv::Mat covmat){

    //Get the eigenvalues and eigenvectors
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(covmat, eigenvalues, eigenvectors);

    //Calculate the angle between the largest eigenvector and the x-axis
    double angle = atan2(eigenvectors.at<double>(0,1), eigenvectors.at<double>(0,0));

    //Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
    if(angle < 0)
      angle += 6.28318530718;

    //Conver to degrees instead of radians
    angle = 180*angle/3.14159265359;

    //Calculate the size of the minor and major axes
    double halfmajoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(0));
    double halfminoraxissize=chisquare_val*sqrt(eigenvalues.at<double>(1));

    //Return the oriented ellipse
    //The -angle is used because OpenCV defines the angle clockwise instead of anti-clockwise
    return cv::RotatedRect(mean, cv::Size2f(halfmajoraxissize, halfminoraxissize), -angle);

  }
}
