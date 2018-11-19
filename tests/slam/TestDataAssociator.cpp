// Created by dinies on 05/09/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "../../src/slam/LineMatcher.hpp"
#include "../../src/slam/Robot.hpp"
#include "../../include/structs.hpp"
#include "../../src/slam/DataAssociator.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( DataAssociatorClass)

  struct PlainDataAssociatorFixture {
    DataAssociator dA;
    std::vector<scanPoint> oldscanPoints;
    std::vector<scanPoint> newscanPoints;
    PlainDataAssociatorFixture(){
      scanPoint sp1(1,1);
      scanPoint sp2(3,1);
      scanPoint sp3(5,3);
      scanPoint sp4(5.5,3);
      scanPoint sp5(1,1.5);
      scanPoint sp6(2.73,2.5);
      scanPoint sp7(4,4.2);
      scanPoint sp8(4.73,4.5);
      scanPoint sp9(5.23,4.5);

      oldscanPoints ={ sp1, sp2, sp3, sp4};
      newscanPoints ={ sp5, sp6, sp7, sp8, sp9};

      line l1(0,1);
      line l2(1,2);
      line l3(2,3);
      line l4(0,1);
      line l5(1,2);
      line l6(2,3);
      line l7(3,4);


      std::vector<line> oldLines = { l1, l2, l3};
      std::vector<line> newLines = { l4, l5, l6, l7};


      const int max_candidates = 2;
      const double lengthDiffThreshold = 0.5;
      const double absoluteOrientationDiffThreshold = 1.4;
      const double nearLinesOrientationDiffThreshold = 0.3;
      const double nearLinesBonusScoreMultiplier= 1.3;

      dA = DataAssociator( max_candidates,
                           lengthDiffThreshold,
                           absoluteOrientationDiffThreshold,
                           nearLinesOrientationDiffThreshold,
                           nearLinesBonusScoreMultiplier,
                           oldLines,
                           oldscanPoints,
                           newLines,
                           newscanPoints);

    }
    ~PlainDataAssociatorFixture(){}
 };

  struct DataAssociationsFixture {
    dataAssociation association;
    std::vector<dataAssociation> associationsVec;
    std::vector< std::vector< dataAssociation>> associationsMatrix;
    std::vector< std::vector< dataAssociation>> associationsMatrixEmpty;
    DataAssociationsFixture(){
      association.old_line_index = 0;
      association.new_line_index = 0;
      association.confidence_score = 100;

      dataAssociation d1(0,0,110);

      dataAssociation d2(0,1,70);

      dataAssociation d3(0,2,50);

      dataAssociation d4(1,1,20);

      dataAssociation d5(2,1,20);

      dataAssociation d6(2,3,120);

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
 };



  BOOST_AUTO_TEST_CASE( testAssociateLines ) {

    PlainDataAssociatorFixture fixture;
    std::vector< dataAssociation> result = fixture.dA.associateLines();
    BOOST_CHECK_EQUAL( 2 , result.size());
    if ( result.size() ==2){
      dataAssociation d1 = result.at(0);
      dataAssociation d2 = result.at(1);
      BOOST_CHECK_EQUAL( d1.old_line_index, 2);
      BOOST_CHECK_EQUAL( d1.new_line_index, 3);
      BOOST_CHECK( d1.confidence_score > 0);
      BOOST_CHECK_EQUAL( d2.old_line_index, 0);
      BOOST_CHECK_EQUAL( d2.new_line_index, 0);
      BOOST_CHECK( d2.confidence_score > 0);
   }
  }

  BOOST_AUTO_TEST_CASE( testFindCandidatesHighToleranceDistanceDiff) {
    PlainDataAssociatorFixture fixture;
    const int newLineIndex = 1;
    fixture.dA.setLengthDiffThreshold( 3);
    std::vector< dataAssociation> result = fixture.dA.findCandidates( newLineIndex);
    BOOST_CHECK_EQUAL(3 , result.size());
    if ( result.size() ==3){
      dataAssociation d1 = result.at(0);
      dataAssociation d2 = result.at(1);
      dataAssociation d3 = result.at(2);
      BOOST_CHECK_EQUAL( d1.old_line_index, 1);
      BOOST_CHECK_EQUAL( d1.new_line_index, 1);
      BOOST_CHECK( d1.confidence_score > 0);
      BOOST_CHECK_EQUAL( d2.old_line_index, 0);
      BOOST_CHECK_EQUAL( d2.new_line_index, 1);
      BOOST_CHECK( d2.confidence_score > 0);
      BOOST_CHECK_EQUAL( d3.old_line_index, 2);
      BOOST_CHECK_EQUAL( d3.new_line_index, 1);
      BOOST_CHECK( d3.confidence_score > 0);
    }
  }

  BOOST_AUTO_TEST_CASE( testFindCandidatesLowToleranceDistanceDiff) {
    PlainDataAssociatorFixture fixture;
    const int newLineIndex = 1;
    std::vector< dataAssociation> result = fixture.dA.findCandidates( newLineIndex);
    BOOST_CHECK_EQUAL(1 , result.size());
    if ( result.size() ==1){
      dataAssociation d1 = result.at(0);
      BOOST_CHECK_EQUAL( d1.old_line_index, 0);
      BOOST_CHECK_EQUAL( d1.new_line_index, 1);
      BOOST_CHECK( d1.confidence_score > 0);
      }
  }

  BOOST_AUTO_TEST_CASE( testInsertOrderedDataAssociationHighScore) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    fixtureClass.dA.insertOrderedDataAssociation( fixtureStructs.association,
                                                  fixtureStructs.associationsVec );
    BOOST_CHECK_EQUAL( 4 , fixtureStructs.associationsVec.size());
    dataAssociation d = fixtureStructs.associationsVec.at(1);
    BOOST_CHECK_EQUAL( 100 , d.confidence_score);
  }

  BOOST_AUTO_TEST_CASE( testInsertOrderedDataAssociationLowScore) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    fixtureStructs.association.confidence_score = 10;
    fixtureClass.dA.insertOrderedDataAssociation( fixtureStructs.association,
                                                  fixtureStructs.associationsVec );
    BOOST_CHECK_EQUAL( 4 , fixtureStructs.associationsVec.size());
    dataAssociation d = fixtureStructs.associationsVec.at(3);
    BOOST_CHECK_EQUAL( 10 , d.confidence_score);
  }

  BOOST_AUTO_TEST_CASE( testInsertOrderedDataAssociationInvalid) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    fixtureStructs.association.confidence_score = -1;
    fixtureClass.dA.insertOrderedDataAssociation( fixtureStructs.association,
                                                  fixtureStructs.associationsVec );
    BOOST_CHECK_EQUAL( 3 , fixtureStructs.associationsVec.size());
  }


  BOOST_AUTO_TEST_CASE( testCompareLinesValid1) {
    PlainDataAssociatorFixture fixtureClass;
    dataAssociation result = fixtureClass.dA.compareLines(0,0);
    BOOST_CHECK( result.confidence_score > 0);
  }

  BOOST_AUTO_TEST_CASE( testCompareLinesValid2) {
    PlainDataAssociatorFixture fixtureClass;
    dataAssociation result = fixtureClass.dA.compareLines(0,1);
    BOOST_CHECK( result.confidence_score > 0);
  }



  BOOST_AUTO_TEST_CASE( testCompareLinesInvalid) {
    PlainDataAssociatorFixture fixtureClass;
    dataAssociation result = fixtureClass.dA.compareLines(0,2);
    BOOST_CHECK( result.confidence_score < 0);
  }


  BOOST_AUTO_TEST_CASE( testGetLineLength) {
    PlainDataAssociatorFixture fixture;
    line line;
    line.first_index = 0;
    line.second_index = 1;
    double result = fixture.dA.getLineLength(line, fixture.oldscanPoints);
    double truth = 2;
    BOOST_CHECK_SMALL( fabs(result - truth), 0.01);
  }

  BOOST_AUTO_TEST_CASE( testGetLineOrientation) {
    PlainDataAssociatorFixture fixture;
    line line1(0,1);
    double result = fixture.dA.getLineOrientation( line1, fixture.oldscanPoints);
    double truth = 0;
    double diff = MyMath::boxMinusAngleRad(result , truth);
    BOOST_CHECK_SMALL( diff, 0.01);
    line line2(1,2);
    result = fixture.dA.getLineOrientation( line2, fixture.oldscanPoints);
    truth = M_PI/4;
    diff = MyMath::boxMinusAngleRad(result , truth);
    BOOST_CHECK_SMALL( diff , 0.01);
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
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    std::vector<dataAssociation> result = fixtureClass.dA.chooseBestAssociations( fixtureStructs.associationsMatrix );
    BOOST_CHECK_EQUAL( result.size() , 3);
    if ( result.size() == 3){
      dataAssociation d1 = result.at(0);
      dataAssociation d2 = result.at(1);
      dataAssociation d3 = result.at(2);
      BOOST_CHECK_EQUAL( d1.old_line_index, 2);
      BOOST_CHECK_EQUAL( d1.new_line_index, 3);
      BOOST_CHECK_EQUAL( d1.confidence_score, 120);
      BOOST_CHECK_EQUAL( d2.old_line_index, 0);
      BOOST_CHECK_EQUAL( d2.new_line_index, 0);
      BOOST_CHECK_EQUAL( d2.confidence_score, 110);
      BOOST_CHECK_EQUAL( d3.old_line_index, 1);
      BOOST_CHECK_EQUAL( d3.new_line_index, 1);
      BOOST_CHECK_EQUAL( d3.confidence_score, 20);
   }
  }

  BOOST_AUTO_TEST_CASE( testRemoveTakenAssociations) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    fixtureStructs.association.old_line_index = 0;
    fixtureStructs.association.new_line_index = 2;
    fixtureClass.dA.removeTakenAssociations(fixtureStructs.association,fixtureStructs.associationsMatrix);
    std::vector<int> truth_elemsInEveryVecOfMat = { 0 , 2, 0, 1};
    int count = 0;
    for ( auto vec : fixtureStructs.associationsMatrix ){
      BOOST_CHECK_EQUAL( truth_elemsInEveryVecOfMat.at(count) , vec.size());
      ++count;
    }
  }

  BOOST_AUTO_TEST_CASE( testChooseMaxScoreAssociationEmptyMatrix) {
    PlainDataAssociatorFixture fixtureClass;
    DataAssociationsFixture fixtureStructs;
    dataAssociation result = fixtureClass.dA.chooseMaxScoreAssociation(fixtureStructs.associationsMatrixEmpty);
    BOOST_CHECK_SMALL( result.confidence_score + 100, 0.01 );
  }


  BOOST_AUTO_TEST_CASE( testChooseMaxScoreAssociation) {
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

