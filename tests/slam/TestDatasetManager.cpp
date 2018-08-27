// Created by dinies on 12/07/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "../../src/slam/DatasetManager.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( DManager)

  BOOST_AUTO_TEST_CASE(parseOnelineFile) {

    std::string absolutePath = "/home/dinies/gitrepos/DynModeling/files/datasets/exampleDataSetOneline.txt";
    datasetParams params =  DatasetManager::parseStaticParameters(absolutePath);
    BOOST_CHECK_EQUAL( params.tag  , "LASER_MESSAGE");
    if (params.tag == "LASER_MESSAGE"){
      BOOST_CHECK_EQUAL( params.topic  , "/scan");
      BOOST_CHECK_EQUAL( params.frame_id  , "/laser_frame");
      BOOST_CHECK_EQUAL( params.min_range , 0.02);
      BOOST_CHECK_EQUAL( params.max_range , 30);
      BOOST_CHECK_EQUAL( params.min_angle , -1.5708);
      BOOST_CHECK_EQUAL( params.max_angle , 1.5708);
      BOOST_CHECK_EQUAL( params.angle_increment, 0.00436332);
      BOOST_CHECK_EQUAL( params.time_increment, 0);
      BOOST_CHECK_EQUAL( params.scan_time  , 0);
      BOOST_CHECK_EQUAL( params.ranges_size  , 721);
      BOOST_CHECK_EQUAL( params.intensities  , 0);
    }
  }

  BOOST_AUTO_TEST_CASE( parseDatasetTwoLines) {
    std::string absolutePath = "/home/dinies/gitrepos/DynModeling/files/datasets/exampleDataSetTwoLines.txt";
    datasetParams params;
    params.tag =  "LASER_MESSAGE";
    params.topic = "/scan";
    params.frame_id = "/laser_frame";
    params.min_range = 0.02;
    params.max_range = 30;
    params.min_angle = -1.5708;
    params.max_angle = 1.5708;
    params.angle_increment= 0.00436332;
    params.time_increment= 0;
    params.scan_time  = 0;
    params.ranges_size  = 721;
    params.intensities  = 0;

    dataSet result_dS = DatasetManager::parseDataSet(absolutePath, params);

    BOOST_CHECK_EQUAL( result_dS.size() , 2);

    if (result_dS.size() == 2){
      BOOST_CHECK_EQUAL( result_dS.at(0).sequence_number , 30690);
      BOOST_CHECK_EQUAL( result_dS.at(0).timing_count , 1482340237.09809);
      BOOST_CHECK_EQUAL( result_dS.at(0).ranges.size(),721);
      BOOST_CHECK_EQUAL( result_dS.at(0).ranges.at(0),1.103);
      BOOST_CHECK_EQUAL( result_dS.at(0).ranges.at(720),7.935);
      BOOST_CHECK_EQUAL( result_dS.at(1).sequence_number , 30691);
      BOOST_CHECK_EQUAL( result_dS.at(1).timing_count , 1482340237.12284);
      BOOST_CHECK_EQUAL( result_dS.at(1).ranges.size(),721);
      BOOST_CHECK_EQUAL( result_dS.at(1).ranges.at(0),1.114);
      BOOST_CHECK_EQUAL( result_dS.at(1).ranges.at(720),7.94);
    }
  }
  BOOST_AUTO_TEST_CASE( DMconstructorWithDummyFile) {

    std::string absolutePath = "/home/dinies/gitrepos/DynModeling/files/datasets/dummyDataSet.txt";
    DatasetManager dm = DatasetManager(absolutePath);
    BOOST_CHECK_EQUAL( dm.m_staticParams.tag  , "LASER_MESSAGE");

    if (dm.m_staticParams.tag == "LASER_MESSAGE"){
      BOOST_CHECK_EQUAL( dm.m_staticParams.topic  , "/scan");
      BOOST_CHECK_EQUAL( dm.m_staticParams.frame_id  , "/laser_frame");
      BOOST_CHECK_EQUAL( dm.m_staticParams.min_range , 0.02);
      BOOST_CHECK_EQUAL( dm.m_staticParams.max_range , 30);
      BOOST_CHECK_EQUAL( dm.m_staticParams.min_angle , -1.0472);
      BOOST_CHECK_EQUAL( dm.m_staticParams.max_angle , 1.0472);
      BOOST_CHECK_EQUAL( dm.m_staticParams.angle_increment, 0.5236);
      BOOST_CHECK_EQUAL( dm.m_staticParams.time_increment, 0);
      BOOST_CHECK_EQUAL( dm.m_staticParams.scan_time  , 0);
      BOOST_CHECK_EQUAL( dm.m_staticParams.ranges_size  , 5);
      BOOST_CHECK_EQUAL( dm.m_staticParams.intensities  , 0);
    }
  }

  BOOST_AUTO_TEST_CASE( DMconstructorWithSimilRealfile ) {

    std::string absolutePath = "/home/dinies/gitrepos/DynModeling/files/datasets/exampleDataSetOneline.txt";
    DatasetManager dm = DatasetManager(absolutePath);
    BOOST_CHECK_EQUAL( dm.m_staticParams.tag  , "LASER_MESSAGE");

    if (dm.m_staticParams.tag == "LASER_MESSAGE"){
      BOOST_CHECK_EQUAL( dm.m_staticParams.topic  , "/scan");
      BOOST_CHECK_EQUAL( dm.m_staticParams.frame_id  , "/laser_frame");
      BOOST_CHECK_EQUAL( dm.m_staticParams.min_range , 0.02);
      BOOST_CHECK_EQUAL( dm.m_staticParams.max_range , 30);
      BOOST_CHECK_EQUAL( dm.m_staticParams.min_angle , -1.5708);
      BOOST_CHECK_EQUAL( dm.m_staticParams.max_angle , 1.5708);
      BOOST_CHECK_EQUAL( dm.m_staticParams.angle_increment, 0.00436332);
      BOOST_CHECK_EQUAL( dm.m_staticParams.time_increment, 0);
      BOOST_CHECK_EQUAL( dm.m_staticParams.scan_time  , 0);
      BOOST_CHECK_EQUAL( dm.m_staticParams.ranges_size  , 721);
      BOOST_CHECK_EQUAL( dm.m_staticParams.intensities  , 0);
    }
  }
  BOOST_AUTO_TEST_CASE(SpanningAngles ) {

    std::string absolutePath = "/home/dinies/gitrepos/DynModeling/files/datasets/dummyDataSet.txt";
    DatasetManager dm = DatasetManager(absolutePath);
    std::vector<double> spanning_angles = dm.getSpanningAngles();


    BOOST_CHECK_EQUAL( spanning_angles.size() , 5);
    if (spanning_angles.size() == 5){
      double alfa = 0.5236;
      std::vector<double> truth_spanning = { -alfa*2, -alfa, 0 , alfa , alfa*2};
      double threshold = 0.01;
      for ( int i= 0 ; i <5 ; ++i ){
        BOOST_CHECK_SMALL( spanning_angles.at(i) - truth_spanning.at(i), threshold);
      }
    }
  }

  BOOST_AUTO_TEST_CASE(nodeRanges) {

    std::string absolutePath = "/home/dinies/gitrepos/DynModeling/files/datasets/dummyDataSet.txt";
    DatasetManager dm = DatasetManager(absolutePath);

    int num_data_entries = dm.getNumDataEntries();
    BOOST_CHECK_EQUAL( num_data_entries , 1);

    if (num_data_entries == 1){
      std::vector<double> ranges = dm.getDataNodeRanges(0);
      std::vector<double> truth_ranges = {10,10,10,5.5,1};

      BOOST_CHECK_EQUAL( ranges.size() , 5);

      if ( ranges.size() == 5 ){
        for ( int i = 0 ; i < 5; ++i ){
          BOOST_CHECK_EQUAL( ranges.at(i) , truth_ranges.at(i));
        }
      }
    }
  }
  BOOST_AUTO_TEST_SUITE_END()
}
