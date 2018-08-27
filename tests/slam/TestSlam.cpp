// Created by dinies on 26/06/2018.


#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>

#include "src/slam/Slam.hpp"
#include "opencv2/opencv.hpp"

namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( SlamClass)

  BOOST_AUTO_TEST_CASE(behaviouralTest) {
    std::string absolutePath = "/home/dinies/gitrepos/DynModeling/files/datasets/realLaserScans.txt";
    std::vector<double> initial_state = { 0, 0, 0 };
    Slam slam = Slam(absolutePath, initial_state);
    slam.cycle();
    cv::waitKey();
  }
  BOOST_AUTO_TEST_SUITE_END()
}
