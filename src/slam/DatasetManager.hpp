// Created by Edoardo Ghini on 26/06/2018.

#pragma once
#include <unistd.h>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

namespace dyn_modeling {
  class DatasetManager {
  public:

    typedef struct datasetParams {
      std::string tag;
      std::string topic;
      std::string frame_id;
      double min_range;
      double max_range;
      double min_angle;
      double max_angle;
      double angle_increment;
      double time_increment;
      double scan_time;
      double ranges_size;
      double intensities;
    } datasetParams;


    // std::string tag;
    // std::string topic;
    // std::string frame_id;
    // double sequence_num;
    // double timing_count;
    // std::vector<double> imu_odom;
    // double min_range;
    // double max_range;
    // double min_angle;
    // double max_angle;
    // double angle_increment;
    // double time_increment;
    // double scan_time;
    // double ranges_size;
    // std::vector<double> ranges;
    // double intensities;
    datasetParams m_staticParams;

    DatasetManager( std::string t_dataSetPath);

    static datasetParams parseStaticParameters( std::string t_dataSetPath);


  };
}
