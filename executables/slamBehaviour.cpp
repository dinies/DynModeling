// Created by Dinies on 04/07/2018.

#include "opencv2/opencv.hpp"
#include "src/slam/Slam.hpp"

using namespace dyn_modeling;
int main(int argc, char **argv) {

  // std::string absolutePath = "/home/dinies/GitRepos/DynModeling/files/datasets/exampleDataSetTwoLines.txt";
  std::string absolutePath = "/home/dinies/GitRepos/DynModeling/files/datasets/realLaserScans.txt";
  std::vector<double> initial_state = { 0, 0, 0 };
  Slam slam = Slam(absolutePath, initial_state);
  slam.cycle();
  
  cv::waitKey();
}
