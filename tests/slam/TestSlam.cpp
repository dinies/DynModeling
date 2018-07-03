// Created by Edoardo Ghini on 26/06/2018.

//#define BOOST_TEST_TOOLS_UNDER_DEBUGGER it seems not to solve the porblem,
// ( the real responsability is that all the compilation is managed by xcode mess)
// on UNUNTU maybe it should be fine.  I am referring to clang build system of OSX
#define BOOST_TEST_MODULE SlamTests


#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "../../src/slam/DatasetManager.hpp"
#include "../../src/slam/Robot.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( DManager)


  // BOOST_AUTO_TEST_CASE( parsingExampleFile) {
  //   dM = DatasetManager( "exampleDataSet.txt");
  //   BOOST_CHECK_EQUAL( 2+2, 4);
  // }

  BOOST_AUTO_TEST_CASE(parseOnelineFile) {

    std::string absolutePath = "/Users/dinies33/GitRepos/DynModeling/files/datasets/exampleDataSetOneline.txt";
    datasetParams params =  DatasetManager::parseStaticParameters(absolutePath);
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

  BOOST_AUTO_TEST_CASE( parseDatasetTwoLines) {
    std::string absolutePath = "/Users/dinies33/GitRepos/DynModeling/files/datasets/exampleDataSetTwoLines.txt";
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
  BOOST_AUTO_TEST_CASE( DMconstructorWithDummyFile) {

    std::string absolutePath = "/Users/dinies33/GitRepos/DynModeling/files/datasets/dummyDataSet.txt";
    DatasetManager dm = DatasetManager(absolutePath);
    BOOST_CHECK_EQUAL( dm.m_staticParams.tag  , "LASER_MESSAGE");
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


  BOOST_AUTO_TEST_CASE( DMconstructorWithSimilRealfile ) {

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

  BOOST_AUTO_TEST_CASE(SpanningAngles ) {

    std::string absolutePath = "/Users/dinies33/GitRepos/DynModeling/files/datasets/dummyDataSet.txt";
    DatasetManager dm = DatasetManager(absolutePath);
    std::vector<double> spanning_angles = dm.getSpanningAngles();


    BOOST_CHECK_EQUAL( spanning_angles.size() , 5);
    double thirtyDeg_inRad = 0.5236;
    BOOST_CHECK_EQUAL( std::round(10000*spanning_angles.at(0))/10000, -thirtyDeg_inRad*2);
    BOOST_CHECK_EQUAL( std::round(10000*spanning_angles.at(1))/10000, -thirtyDeg_inRad);
    BOOST_CHECK_EQUAL( std::round(10000*spanning_angles.at(2))/10000, 0);
    BOOST_CHECK_EQUAL( std::round(10000*spanning_angles.at(3))/10000, thirtyDeg_inRad);
    BOOST_CHECK_EQUAL( std::round(10000*spanning_angles.at(4))/10000, thirtyDeg_inRad*2);
  }

  BOOST_AUTO_TEST_CASE(nodeRanges) {

    std::string absolutePath = "/Users/dinies33/GitRepos/DynModeling/files/datasets/dummyDataSet.txt";
    DatasetManager dm = DatasetManager(absolutePath);
    std::vector<double> ranges = dm.getDataNodeRanges(0);

    BOOST_CHECK_EQUAL( ranges.size() , 5);
    BOOST_CHECK_EQUAL( ranges.at(0) , 10);
    BOOST_CHECK_EQUAL( ranges.at(1) , 10);
    BOOST_CHECK_EQUAL( ranges.at(2) , 10);
    BOOST_CHECK_EQUAL( ranges.at(3) , 5.5);
    BOOST_CHECK_EQUAL( ranges.at(4) , 1);
  }
  BOOST_AUTO_TEST_SUITE_END()


  BOOST_AUTO_TEST_SUITE( Rob)

  BOOST_AUTO_TEST_CASE( retrieveScanPoints) {
    std::string absolutePath = "/Users/dinies33/GitRepos/DynModeling/files/datasets/exampleDataSetOneline.txt";
    std::vector<double> initial_state = { 0, 0, 0};
    Robot r = Robot(absolutePath, initial_state);
    std::vector< scanPoint > scanPoints_robotFrame = r.retrieveScanPoints(0);
    double angle = 0.5235984;
    Eigen::Matrix2d R;
    R << cos(angle ), -sin(angle),
      sin(angle), cos(angle);
    Eigen::Vector2d origin_range( 1.417 ,0);
    Eigen::Vector2d coords = R* origin_range;

    scanPoint sP = scanPoints_robotFrame.at(120);
    BOOST_CHECK_EQUAL(sP.coords(0) ,coords(0));
    BOOST_CHECK_EQUAL(sP.coords(1) ,coords(1));
  }
  BOOST_AUTO_TEST_SUITE_END()


}
