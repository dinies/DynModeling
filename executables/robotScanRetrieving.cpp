// Created by Dinies on 03/07/2018.

#include "src/slam/Robot.hpp"


using namespace dyn_modeling;
int main(int argc, char **argv) {

  std::string relativePath= "../files/datasets/dummyDataSet.txt";
  Robot r = Robot(relativePath);
  double borderRatio = 0.05;
  std::vector< scanPoint > scanPoints_robotFrame = r.retrieveScanPointsRobotFrame(0, borderRatio);
  for ( auto point : scanPoints_robotFrame){
    std::cout << point.coords(0) << " ; "<< point.coords(1) << "\n";
  }
}

