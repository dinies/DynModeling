// Created by dinies on 05/09/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "../../src/slam/LineMatcher.hpp"
#include "../../src/slam/Robot.hpp"
#include "../../include/structs.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( DataAssociatorClass)

  struct PlainDataAssociatorFixture {
    PlainDataAssociatorFixture(){
      const scanPoint sp1;
      const scanPoint sp2;
      const scanPoint sp3;
      const scanPoint sp4;
      const scanPoint sp5;
      const scanPoint sp6;
      const scanPoint sp7;
      const scanPoint sp8;
      const scanPoint sp9;

      Eigen::Vector2d c1(1,1);
      Eigen::Vector2d c2(3,1);
      Eigen::Vector2d c3(5,3);
      Eigen::Vector2d c4(5.5,3);
      Eigen::Vector2d c5(1,1.5);
      Eigen::Vector2d c6(2.73,2);
      Eigen::Vector2d c7(4,3.7);
      Eigen::Vector2d c8(4.73,4);
      Eigen::Vector2d c9(5.23,4);

      sp1.coords = c1;
      sp2.coords = c2;
      sp3.coords = c3;
      sp4.coords = c4;
      sp5.coords = c5;
      sp6.coords = c6;
      sp7.coords = c7;
      sp8.coords = c8;
      sp9.coords = c9;

      oldscanPoints ={ sp1, sp2, sp3, sp4};
      newscanPoints ={ sp5, sp6, sp7, sp8, sp9};

      const line l1;
      l1.first_index = 0;
      l1.second_index = 1;
      const line l2;
      l2.first_index = 1;
      l2.second_index = 2;
      const line l3;
      l3.first_index = 2;
      l3.second_index = 3;
      const line l4;
      l4.first_index = 0;
      l4.second_index = 1;
      const line l5;
      l5.first_index = 1;
      l5.second_index = 2;
      const line l6;
      l6.first_index = 2;
      l6.second_index = 3;
      const line l7;
      l7.first_index = 3;
      l7.second_index = 4;


      const std::vector<line> oldLines = { l1, l2, l3};
      const std::vector<line> newLines = { l4, l5, l6, l7};


      const int max_candidates = 2;
      const double lengthDiffThreshold = 0.5;
      const double orientationDiffThreshold = 0.2;

      dA = DataAssociator( max_candidates, lengthDiffThreshold, orientationDiffThreshold, oldLines,oldscanPoints, newLines, newscanPoints);

    }
    ~PlainDataAssociatorFixture(){}
    DataAssociator dA;
    std::vector<scanpoint> oldscanPoints;
    std::vector<scanpoint> newscanPoints;
  }

  struct DataAssociationsFixture {
    DataAssociationsFixture(){
      association.old_line_index = 0;
      association.new_line_index = 0;
      association.confidence_score = 100;

      dataAssociation d1;
      d1.old_line_index = 0;
      d1.new_line_index = 0;
      d1.confidence_score = 110;

      dataAssociation d2;
      d2.old_line_index = 0;
      d2.new_line_index = 1;
      d2.confidence_score = 70;

      dataAssociation d3;
      d3.old_line_index = 0;
      d3.new_line_index = 2;
      d3.confidence_score = 50;

      dataAssociation d4;
      d4.old_line_index = 1;
      d4.new_line_index = 1;
      d4.confidence_score = 20;

      dataAssociation d5;
      d5.old_line_index = 2;
      d5.new_line_index = 1;
      d5.confidence_score = 20;

      dataAssociation d6;
      d6.old_line_index = 2;
      d6.new_line_index = 3;
      d6.confidence_score = 120;

      associationsVec = { d1, d2, d3};
      std::vector< dataAssociation> vecNewline1 = { d1 };
      std::vector< dataAssociation> vecNewline2 = { d2, d4, d5 };
      std::vector< dataAssociation> vecNewline3 = { d3 };
      std::vector< dataAssociation> vecNewline4 = { d6 };
      associationsMatrix = { vecNewline1, vecNewline2, vecNewline3, vecNewline4};

      std::vector< dataAssociation> emptyVec = {};
      associationsMatrixEmpty = {emptyVec, emptyVec, emptyVec, emptyVec};

    }
    ~DataAssociationsFixture(){}
    dataAssociation association;
    std::vector<dataAssociation> associationsVec;
    std::vector< std::vector< dataAssociation>> associationsMatrix;
    std::vector< std::vector< dataAssociation>> associationsMatrixEmpty;
  }



  BOOST_AUTO_TEST_CASE( testAssociateLines ) {

    PlainDataAssociatorFixture fixture;
    std::vector< dataAssociation> result = fixture.dA.associateLines();
    BOOST_CHECK_EQUAL( 2 , result.size());

    // RGBImage img;
    // img.create( 900,900);
    // img = cv::Vec3b(227, 246, 253);
    // cv::namedWindow("Lines");
    // cv::moveWindow("Lines", 40, 40);
    // cv::Scalar dark_red = {20,0,255};
    // Drawer drawer(15);
    // cv::imshow("Lines",img);
    // for ( auto line : lines){
    //   scanPoint sP1 = scanPoints_robotFrame.at( line.first_index);
    //   scanPoint sP2 = scanPoints_robotFrame.at( line.second_index);
    //   cv::Point2d p1( sP1.coords(0), sP1.coords(1));
    //   cv::Point2d p2( sP2.coords(0), sP2.coords(1));
    //   drawer.drawLine( img, p1, p2 , dark_red);
    // }

    // cv::imshow("Lines",img);
    // cv::waitKey(500);
  }

  BOOST_AUTO_TEST_CASE( testFindCandidates) {
    PlainDataAssociatorFixture fixture;
    std::vector< dataAssociation> result = fixture.dA.associateLines();
    BOOST_CHECK_EQUAL(2 , result.size());
    if ( result.size() >=2){
      dataAssociation d1 = result.at(0);
      dataAssociation d2 = result.at(1);
      BOOST_CHECK_EQUAL( d1.old_line_index= 0);
      BOOST_CHECK_EQUAL( d1.new_line_index = 0);
      BOOST_CHECK( d1.confidence_score > 0);
      BOOST_CHECK_EQUAL( d2.old_line_index = 2);
      BOOST_CHECK_EQUAL( d2.new_line_index = 3);
      BOOST_CHECK( d2.confidence_score > 0);
    }
  }

  BOOST_AUTO_TEST_CASE( testInsertOrderedDataAssociationHighScore) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    fixtureClass.dA.insertOrderedDataAssociation( fixtureStructs.association, fixtureStructs.associationsVec );
    BOOST_CHECK_EQUAL( 4 , fixtureStructs.associationsVec.size());
    dataAssociation d = fixtureStructs.associationsVec.at(1);
    BOOST_CHECK_EQUAL( 100 , d.confidence_score);
  }

  BOOST_AUTO_TEST_CASE( testInsertOrderedDataAssociationLowScore) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    fixtureStructs.association.confidence_score = 10;
    fixtureClass.dA.insertOrderedDataAssociation( fixtureStructs.association, fixtureStructs.associationsVec );
    BOOST_CHECK_EQUAL( 4 , fixtureStructs.associationsVec.size());
    dataAssociation d = fixtureStructs.associationsVec.at(3);
    BOOST_CHECK_EQUAL( 10 , d.confidence_score);
  }

  BOOST_AUTO_TEST_CASE( testInsertOrderedDataAssociationInvalid) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    fixtureStructs.association.confidence_score = -1;
    fixtureClass.dA.insertOrderedDataAssociation( fixtureStructs.association, fixtureStructs.associationsVec );
    BOOST_CHECK_EQUAL( 3 , fixtureStructs.associationsVec.size());
  }


  BOOST_AUTO_TEST_CASE( testCompareLinesValid) {
    PlainDataAssociatorFixture fixtureClass;
    dataAssociation result = fixture.dA.compareLines(0,0);
    BOOST_CHECK( result.confidence_score > 0);
  }

  BOOST_AUTO_TEST_CASE( testCompareLinesInvalid) {
    PlainDataAssociatorFixture fixtureClass;
    dataAssociation result = fixture.dA.compareLines(0,2);
    BOOST_CHECK( result.confidence_score < 0);
  }


  BOOST_AUTO_TEST_CASE( testGetLineLength) {
    PlainDataAssociatorFixture fixture;
    const line line;
    line.first_index = 0;
    line.second_index = 1;
    double result = fixture.dA.getLineLength( newLineIndex);
    double truth = 2;
    BOOST_CHECK_SMALL( fabs(result - truth), 0.01);
  }

  BOOST_AUTO_TEST_CASE( testGetLineOrientation) {
    PlainDataAssociatorFixture fixture;
    const line line1;
    line1.first_index = 0;
    line1.second_index = 1;
    double result = fixture.dA.getLineOrientation( line1);
    double truth = 0;
    BOOST_CHECK_SMALL( fabs(result - truth), 0.01);
    const line line2;
    line2.first_index = 1;
    line2.second_index = 2;
    double result = fixture.dA.getLineOrientation( line2);
    double truth = 0.785;
    BOOST_CHECK_SMALL( fabs(result - truth), 0.01);
  }

  BOOST_AUTO_TEST_CASE( testGetPossibleCandidatesIndexesFirstElem) {
    PlainDataAssociatorFixture fixture;
    const int newLineIndex = 0;
    std::vector< int> result = fixture.dA.getPossibleCandidateIndexes( newLineIndex);
    std::vector< int> truth = { 0,1 };
    BOOST_CHECK_EQUAL( truth.size() , result.size());

    if (truth.size() == result.size()){
      for (int i = 0; i < result.size(); ++i){
        BOOST_CHECK_EQUAL( truth.at(i) , result.at(i));
      }
    }
  }

  BOOST_AUTO_TEST_CASE( testGetPossibleCandidatesIndexesMiddleElem) {
    PlainDataAssociatorFixture fixture;
    const int newLineIndex = 1;
    std::vector< int> result = fixture.dA.getPossibleCandidateIndexes( newLineIndex);
    std::vector< int> truth = { 0,1,2 };
    BOOST_CHECK_EQUAL( truth.size() , result.size());

    if (truth.size() == result.size()){
      for (int i = 0; i < result.size(); ++i){
        BOOST_CHECK_EQUAL( truth.at(i) , result.at(i));
      }
    }
  }

  BOOST_AUTO_TEST_CASE( testChooseBestAssociations) {
    //TODO
    BOOST_CHECK( true);
  }

  BOOST_AUTO_TEST_CASE( testRemoveTakenAssociations) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    fixtureStructs.association.first_index = 0;
    fixtureStructs.association.second_index = 2;
    fixtureClass.dA.removeTakenAssociations(fixtureStructs.association,fixtureStructs.associationsMatrix);
    std::vector<int> truth_elemsInEveryVecOfMat = { 0 , 1, 0, 0};
    int count = 0;
    for ( auto vec : fixtureStructs.associationsMatrix ){
      BOOST_CHECK_EQUAL( truth_elemsInEveryVecOfMat.at(count) , vec.size());
      ++count;
    }
  }

  BOOST_AUTO_TEST_CASE( testChooseMaxScoreAssociationsEmptyMatrix) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    dataAssociation result = fixtureClass.dA.chooseMaxScoreAssociation(fixtureStructs.associationsMatrixEmpty);
    BOOST_CHECK_SMALL( result.confidence_score + 100, 0.01 );
  }


  BOOST_AUTO_TEST_CASE( testChooseMaxScoreAssociations) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    dataAssociation result = fixtureClass.dA.chooseMaxScoreAssociation(fixtureStructs.associationsMatrix);
    BOOST_CHECK_EQUAL( result.old_line_index , 2 );
    BOOST_CHECK_EQUAL( result.new_line_index , 3 );
    BOOST_CHECK_SMALL( result.confidence_score - 120, 0.01 );
  }

  BOOST_AUTO_TEST_CASE( testAssociationsAllTakenTrue) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    bool result = fixtureClass.dA.associationsAllTaken(fixtureStructs.associationsMatrixEmpty);
    BOOST_CHECK( result );
  }

  BOOST_AUTO_TEST_CASE( testAssociationsAllTakenFalse) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    bool result = fixtureClass.dA.associationsAllTaken(fixtureStructs.associationsMatrix);
    BOOST_CHECK( !result );
  }



  BOOST_AUTO_TEST_SUITE_END()
}
