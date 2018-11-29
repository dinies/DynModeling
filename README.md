# DynModeling

## Overview
This project focuses mainly on a autonomous robotic implementation of SLAM navigation and Loop recognition.
## Requirements and Compatibility
There are some dependencies required for compilation: mainly cmake, Eigen library, boost library and gnuplot.
There also other dependencies that will be automatically downloaded at building time, such as googleTest.

This project supports Unix systems but is tested only on Ubuntu ( version 18.04).
It features an off-source build with cmake config files.
This is not an installable library.

## Usage
After cloning the repo you can build and run the executables:

mkdir build && cd build && cmake .. && make -j8

after that you can navigate to the bin folder and execute the binaries:

cd ../bin\\
./TestSlam

## Known issues
Currently it works only with the CMAKE\_BUILD\_TYPE=Debug ( which is the default).
With other flags, for example optimization flags such as -O2 -O3 -Ofast it will have unexpected behaviours.


