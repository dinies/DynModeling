// Created by Edoardo Ghini on 13/06/2018.

#include "src/slam/DatasetManager.hpp"
#include <iostream>
#include <fstream>
#include <string>
using namespace dyn_modeling;
int main(int argc, char **argv) {

  std::string absolutePath = "/home/dinies/GitRepos/DynModeling/files/datasets/exampleDataSetOneline.txt";
  datasetParams params =  DatasetManager::parseStaticParameters( absolutePath);
  std::cout << params.tag;
  return 0;
}


