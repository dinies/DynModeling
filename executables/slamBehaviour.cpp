// Created by Dinies on 04/07/2018.

#include "opencv2/opencv.hpp"
#include "src/slam/Slam.hpp"

using namespace dyn_modeling;
int main(int argc, char **argv) {

  std::string absolutePath = "/home/dinies/GitRepos/DynModeling/files/datasets/exampleDataSetTwoLines.txt";
  std::vector<double> initial_state = { 0, 0, M_PI/2};
  Slam slam = Slam(absolutePath, initial_state);
  slam.cycle();
  cv::waitKey();
}
