// Created by Edoardo Ghini on 13/06/2018.

#include "src/slam/DatasetManager.hpp"
#include <iostream>
#include <fstream>
#include <string>
using namespace dyn_modeling;
int main(int argc, char **argv) {

  std::string relativePath= "../files/datasets/exampleDataSetOneline.txt";
  datasetParams params =  DatasetManager::parseStaticParameters( relativePath);
  std::cout << params.tag;
  return 0;
}


