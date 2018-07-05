// Created by Dinies on 04/07/2018.

#include "src/slam/Slam.hpp"

using namespace dyn_modeling;
int main(int argc, char **argv) {

  std::string absolutePath = "/Users/dinies33/GitRepos/DynModeling/files/datasets/dummyDataSet.txt";
  std::vector<double> initial_state = { 0, 0, 0};
  Slam slam = Slam(absolutePath, initial_state);
  slam.cycle();
}
