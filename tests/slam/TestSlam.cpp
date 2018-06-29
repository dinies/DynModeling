// Created by Edoardo Ghini on 26/06/2018.

#define BOOST_TEST_MODULE SlamTests

#include <boost/test/unit_test.hpp>
#include "../../src/slam/DatasetManager.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( DManager)

  // TODO

  // BOOST_AUTO_TEST_CASE( parsingExampleFile) {
  //   dM = DatasetManager( "exampleDataSet.txt");
  //   BOOST_CHECK_EQUAL( 2+2, 4);
  // }

  BOOST_AUTO_TEST_CASE(parseOnelineFile) {

    std::string absolutePath = "/Users/dinies33/GitRepos/DynModeling/files/datasets/exampleDataSetOneline.txt";
    DatasetManager::datasetParams params =  DatasetManager::parseStaticParameters(absolutePath);
    BOOST_CHECK_EQUAL( params.tag  , "LASER_MESSAGE");
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


  BOOST_AUTO_TEST_CASE( DMconstructor ) {

    std::string absolutePath = "/Users/dinies33/GitRepos/DynModeling/files/datasets/exampleDataSetOneline.txt";
    DatasetManager dm = DatasetManager(absolutePath);
    BOOST_CHECK_EQUAL( dm.m_staticParams.tag  , "LASER_MESSAGE");
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
  BOOST_AUTO_TEST_SUITE_END()

}
