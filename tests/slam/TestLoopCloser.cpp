// Created by dinies on 10/10/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

// #include "opencv2/opencv.hpp"
// #include "../../src/slam/Robot.hpp"
#include "../../include/structs.hpp"
#include "../../src/slam/Graph.hpp"
#include "../../src/slam/LoopCloser.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( LoopCloserClass )

  struct LoopRecognitionFixture {
    Graph graph;
    double maxDistCenters;
    double maxLinesLengthDiff;
    double maxLinesOrientDiff;

    LoopRecognitionFixture(){
    }


    // Graph g;
    // SimpleGraphFixture(){
    //   scanPoint sp1(1,1);
    //   scanPoint sp2(3,1);
    //   scanPoint sp3(3,3);
    //   scanPoint sp4(5,3);
    //   scanPoint sp5(1,1);
    //   scanPoint sp6(6,1);
    //   scanPoint sp7(6,3);
    //   scanPoint sp8(8,3);
    //   scanPoint sp9(1,-3);
    //   scanPoint sp10(1,1);
    //   scanPoint sp11(4,1);
    //   scanPoint sp12(4,4);

    //   std::vector<scanPoint> points_a { sp1, sp2, sp3, sp4};
    //   std::vector<scanPoint> points_b { sp5, sp6, sp7, sp8};
    //   std::vector<scanPoint> points_c { sp9, sp10, sp11, sp12};

    //   line l1a( 0, 1);
    //   line l2a( 1, 2);
    //   line l3a( 2, 3);
    //   line l1b( 0, 1);
    //   line l2b( 1, 2);
    //   line l3b( 2, 3);
    //   line l1c( 0, 1);
    //   line l2c( 1, 2);
    //   line l3c( 2, 3);

    //   std::vector<line> lines_a { l1a, l2a, l3a};
    //   std::vector<line> lines_b { l1b, l2b, l3b};
    //   std::vector<line> lines_c { l1c, l2c, l3c};

    //   dataAssociation d1_ab( 0,0,100);
    //   dataAssociation d2_ab( 1,1,100);
    //   dataAssociation d1_bc( 0,1,100);

    //   std::vector< dataAssociation> associations_ab { d1_ab, d2_ab};
    //   std::vector< dataAssociation> associations_bc { d1_bc};

    //   state s_a( 0, 0 ,0);
    //   state s_b( 0.1, 0 ,0.1);
    //   state s_c( 0.1, 0.1 ,0.2);

    //   node node_a( s_a, points_a, lines_a);
    //   node node_b( s_b, points_b, lines_b);
    //   node node_c( s_c, points_c, lines_c);

    //   edge edge_zero = edge();
    //   edge edge_ab( 0.1, 0 , 0.1, associations_ab);
    //   edge edge_bc( 0, 0.1 , 0.1, associations_bc);

    //   const int dataEntriesNum {3};
    //   const int maxPointsNum {4};
    //   g= Graph( dataEntriesNum, maxPointsNum );

    //   g.insertNode( node_a);
    //   g.insertNode( node_b);
    //   g.insertNode( node_c);


    //   g.insertEdge( edge_zero);
    //   g.insertEdge( edge_ab);
    //   g.insertEdge( edge_bc);
 
    // DataAssociator dA;
    // std::vector<scanPoint> oldscanPoints;
    // std::vector<scanPoint> newscanPoints;
    // PlainDataAssociatorFixture(){
    //   scanPoint sp1(1,1);
    //   scanPoint sp2(3,1);
    //   scanPoint sp3(5,3);
    //   scanPoint sp4(5.5,3);
    //   scanPoint sp5(1,1.5);
    //   scanPoint sp6(2.73,2.5);
    //   scanPoint sp7(4,4.2);
    //   scanPoint sp8(4.73,4.5);
    //   scanPoint sp9(5.23,4.5);

    //   oldscanPoints ={ sp1, sp2, sp3, sp4};
    //   newscanPoints ={ sp5, sp6, sp7, sp8, sp9};

    //   line l1(0,1);
    //   line l2(1,2);
    //   line l3(2,3);
    //   line l4(0,1);
    //   line l5(1,2);
    //   line l6(2,3);
    //   line l7(3,4);


    //   std::vector<line> oldLines = { l1, l2, l3};
    //   std::vector<line> newLines = { l4, l5, l6, l7};


    //   const int max_candidates = 2;
    //   const double lengthDiffThreshold = 0.5;
    //   const double absoluteOrientationDiffThreshold = 1.4;
    //   const double nearLinesOrientationDiffThreshold = 0.3;
    //   const double nearLinesBonusScoreMultiplier= 1.3;

    //   dA = DataAssociator( max_candidates,
    //                        lengthDiffThreshold,
    //                        absoluteOrientationDiffThreshold,
    //                        nearLinesOrientationDiffThreshold,
    //                        nearLinesBonusScoreMultiplier,
    //                        oldLines,
    //                        oldscanPoints,
    //                        newLines,
    //                        newscanPoints);

    // }
    // ~PlainDataAssociatorFixture(){}
 };

  BOOST_AUTO_TEST_CASE( testCloseLoopSimple) {

    // PlainDataAssociatorFixture fixture;
    // std::vector< dataAssociation> result = fixture.dA.associateLines();
    // BOOST_CHECK_EQUAL( 2 , result.size());
    // if ( result.size() ==2){
    //   dataAssociation d1 = result.at(0);
    //   dataAssociation d2 = result.at(1);
    //   BOOST_CHECK_EQUAL( d1.old_line_index, 2);
    //   BOOST_CHECK_EQUAL( d1.new_line_index, 3);
    //   BOOST_CHECK( d1.confidence_score > 0);
    //   BOOST_CHECK_EQUAL( d2.old_line_index, 0);
    //   BOOST_CHECK_EQUAL( d2.new_line_index, 0);
    //   BOOST_CHECK( d2.confidence_score > 0);
    // }
  }

  BOOST_AUTO_TEST_CASE( testFindIndexesOptimization) {

    // PlainDataAssociatorFixture fixture;
    // std::vector< dataAssociation> result = fixture.dA.associateLines();
    // BOOST_CHECK_EQUAL( 2 , result.size());
    // if ( result.size() ==2){
    //   dataAssociation d1 = result.at(0);
    //   dataAssociation d2 = result.at(1);
    //   BOOST_CHECK_EQUAL( d1.old_line_index, 2);
    //   BOOST_CHECK_EQUAL( d1.new_line_index, 3);
    //   BOOST_CHECK( d1.confidence_score > 0);
    //   BOOST_CHECK_EQUAL( d2.old_line_index, 0);
    //   BOOST_CHECK_EQUAL( d2.new_line_index, 0);
    //   BOOST_CHECK( d2.confidence_score > 0);
    // }
  }


  BOOST_AUTO_TEST_CASE( testSanitizeClosures) {

   //  PlainDataAssociatorFixture fixture;
   //  std::vector< dataAssociation> result = fixture.dA.associateLines();
   //  BOOST_CHECK_EQUAL( 2 , result.size());
   //  if ( result.size() ==2){
   //    dataAssociation d1 = result.at(0);
   //    dataAssociation d2 = result.at(1);
   //    BOOST_CHECK_EQUAL( d1.old_line_index, 2);
   //    BOOST_CHECK_EQUAL( d1.new_line_index, 3);
   //    BOOST_CHECK( d1.confidence_score > 0);
   //    BOOST_CHECK_EQUAL( d2.old_line_index, 0);
   //    BOOST_CHECK_EQUAL( d2.new_line_index, 0);
   //    BOOST_CHECK( d2.confidence_score > 0);
   // }
  }

  BOOST_AUTO_TEST_CASE( testFindClosures) {

    // PlainDataAssociatorFixture fixture;
    // std::vector< dataAssociation> result = fixture.dA.associateLines();
    // BOOST_CHECK_EQUAL( 2 , result.size());
    // if ( result.size() ==2){
    //   dataAssociation d1 = result.at(0);
    //   dataAssociation d2 = result.at(1);
    //   BOOST_CHECK_EQUAL( d1.old_line_index, 2);
    //   BOOST_CHECK_EQUAL( d1.new_line_index, 3);
    //   BOOST_CHECK( d1.confidence_score > 0);
    //   BOOST_CHECK_EQUAL( d2.old_line_index, 0);
    //   BOOST_CHECK_EQUAL( d2.new_line_index, 0);
    //   BOOST_CHECK( d2.confidence_score > 0);
    // }
  }


  BOOST_AUTO_TEST_SUITE_END()
}

