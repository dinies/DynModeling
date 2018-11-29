// Created by dinies on 26/06/2018.

#pragma once
#include <unistd.h>
#include <cmath>
#include <vector>
#include <cstring>
#include <fstream>
#include <iostream>

namespace dyn_modeling {
  typedef struct datasetParams_tag {
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

  typedef struct dataNode_tag {
    int sequence_number;
    double timing_count;
    std::vector<double> ranges;
  }dataNode;

  typedef std::vector< dataNode > dataSet;



  class DatasetManager {
  private:
    std::string m_dataSetPath;
    dataSet m_dataSet;
  public:
   datasetParams m_staticParams;

    DatasetManager( const std::string &t_dataSetPath);

    static datasetParams parseStaticParameters
    (const std::string &t_dataSetPath);

    static dataSet parseDataSet( const std::string &t_dataSetPath,
                                 const datasetParams &t_dSetParams);

    static void collectDataFromString( const std::string &t_dataString,
                                       dataNode &t_returning_struct,
                                       int t_rangesNum);

    std::vector<double> getSpanningAngles();

    inline int getNumDataEntries() { return m_dataSet.size(); }
    inline int getNumRanges() { return m_staticParams.ranges_size; }

    std::vector<double> getDataNodeRanges(const int t_index_datanode);

  };
}
