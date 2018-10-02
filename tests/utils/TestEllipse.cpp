// Created by Edoardo Ghini on 30/08/2018.

#define BOOST_TEST_MODULE EllipseTests

#include <boost/test/unit_test.hpp>

#include "../../src/utils/Ellipse.hpp"

namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( EllipseTest)

  BOOST_AUTO_TEST_CASE( DrawEllipse) {
    //Covariance matrix of our data
    cv::Mat covmat = (cv::Mat_<double>(2,2) << 500.5886, 400.6111, 400.6111, 500.7801);
    //The mean of our data
    cv::Point2f mean(160,120);

    //Calculate the error ellipse for a 95% confidence intervanl
    cv::RotatedRect ellipse = Ellipse::getEllipse(2.4477, mean, covmat);

    //Show the result
    cv::Mat visualizeimage(240, 320, CV_8UC1, cv::Scalar::all(0));
    cv::ellipse(visualizeimage, ellipse, cv::Scalar::all(255), 2);
    cv::imshow("EllipseDemo", visualizeimage);
    cv::waitKey();
  }

  BOOST_AUTO_TEST_SUITE_END()
}
