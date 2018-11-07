// Created by dinies on 17/10/2018.

#include <gtest/gtest.h>
#include "../../src/slam/LoopCloser.hpp"
#include "../../src/slam/Graph.hpp"
#include "../../tests/slam/MockGraph.hpp"

using namespace testing;
namespace dyn_modeling {


  class LoopCloserFixture : public testing::Test {

    protected:
      virtual void SetUp(){
        scanPoint sp1(1,1);
        scanPoint sp2(3,1);
        scanPoint sp3(3,3);
        scanPoint sp4(5,3);

        scanPoint sp5(1,1);
        scanPoint sp6(6,1);
        scanPoint sp7(6,3);
        scanPoint sp8(8,3);

        scanPoint sp9(1,-3);
        scanPoint sp10(1,1);
        scanPoint sp11(4,1);
        scanPoint sp12(4,4);

        std::vector<scanPoint> points_a { sp1, sp2, sp3, sp4};
        std::vector<scanPoint> points_b { sp5, sp6, sp7, sp8};
        std::vector<scanPoint> points_c { sp9, sp10, sp11, sp12};

        line l1a( 0, 1);
        line l2a( 1, 2);
        line l3a( 2, 3);
        line l1b( 0, 1);
        line l2b( 1, 2);
        line l3b( 2, 3);
        line l1c( 0, 1);
        line l2c( 1, 2);
        line l3c( 2, 3);

        std::vector<line> lines_a { l1a, l2a, l3a};
        std::vector<line> lines_b { l1b, l2b, l3b};
        std::vector<line> lines_c { l1c, l2c, l3c};

        dataAssociation d1_ab( 0,0,100);
        dataAssociation d2_ab( 1,1,100);
        dataAssociation d1_bc( 0,1,100);

        std::vector< dataAssociation> associations_ab { d1_ab, d2_ab};
        std::vector< dataAssociation> associations_bc { d1_bc};

        state s_a( 0, 0 ,0);
        state s_b( 0.1, 0 ,0.1);
        state s_c( 0.1, 0.1 ,0.2);

        node node_a( s_a, points_a, lines_a);
        node node_b( s_b, points_b, lines_b);
        node node_c( s_c, points_c, lines_c);

        edge edge_zero = edge();
        edge edge_ab( 0.1, 0 , 0.1, associations_ab);
        edge edge_bc( 0, 0.1 , 0.1, associations_bc);

        const int dataEntriesNum {3};
        const int maxPointsNum {4};
        g= Graph( dataEntriesNum, maxPointsNum );

        g.insertNode( node_a);
        g.insertNode( node_b);
        g.insertNode( node_c);


        g.insertEdge( edge_zero);
        g.insertEdge( edge_ab);
        g.insertEdge( edge_bc);


      }

      Graph g;

  };

  class FourStepClosure : public testing::Test {

    protected:
      MockGraph graphMock;
      Graph g;
      LoopCloser<MockGraph>* lC;
      std::vector<trail> trailsTree;
      std::vector<trail> trailsQuery;
  
      virtual void SetUp(){
        
        const double maxLinesLengthDiff{0.1};
        const double maxLinesOrientDiff{0.1};
        const double leafRangeKdtree{0.1};
        const double maxDistanceKdtree{0.2};
   
        lC= new LoopCloser<MockGraph>( 
            graphMock,
            maxLinesLengthDiff,
            maxLinesOrientDiff,
            leafRangeKdtree,
            maxDistanceKdtree
            );

        scanPoint sp1(3,5);
        scanPoint sp2(5,5);
        scanPoint sp3(5,4);
        scanPoint sp4(5,0);
        scanPoint sp5(5,-2);
        scanPoint sp6(4,-2);

        std::vector<scanPoint> points_n1 { sp1, sp2, sp3};
        std::vector<scanPoint> points_n2 {};
        std::vector<scanPoint> points_n3 {};
        std::vector<scanPoint> points_n4 { sp4, sp5, sp6};

        line l1n1( 0, 1);
        line l2n1( 1, 2);
        line l1n4( 0, 1);
        line l2n4( 1, 2);

        std::vector<line> lines_n1 { l1n1, l2n1};
        std::vector<line> lines_n2 {};
        std::vector<line> lines_n3 {};
        std::vector<line> lines_n4 { l1n4, l2n4};


        state s1( 0, 0 ,0);
        state s2( 0, -3 ,-M_PI/2);
        state s3( 3, -3 , 0);
        state s4( 3, -3 , M_PI/2);

        node node_1( s1, points_n1, lines_n1);
        node node_2( s2, points_n2, lines_n2);
        node node_3( s3, points_n3, lines_n3);
        node node_4( s4, points_n4, lines_n4);

        std::vector< dataAssociation> assoc_1_2 { };
        std::vector< dataAssociation> assoc_2_3 { };
        std::vector< dataAssociation> assoc_3_4 { };

        edge edge_1 = edge();
        edge edge_2(0,-3 ,-M_PI/2, assoc_1_2);
        edge edge_3(0, 3 , M_PI/2, assoc_2_3);
        edge edge_4(0, 3 , M_PI/2, assoc_3_4);

        const int dataEntriesNum {3};
        const int maxPointsNum {4};
        g= Graph( dataEntriesNum, maxPointsNum );

        g.insertNode( node_1);
        g.insertNode( node_2);
        g.insertNode( node_3);
        g.insertNode( node_4);
        g.insertEdge( edge_1);
        g.insertEdge( edge_2);
        g.insertEdge( edge_3);
        g.insertEdge( edge_4);

        trail t1 = trail( Eigen::Vector2d(4,5),  0,0,1);
        trail t2 = trail( Eigen::Vector2d(5,4.5),0,1,1);
        trail t3 = trail( Eigen::Vector2d(4,5),  3,0,1);
        trail t4 = trail( Eigen::Vector2d(5,4.5),3,1,1);
        trailsTree = { t1, t2};
        trailsQuery= { t3, t4};

      }

      virtual void TearDown(){
        delete lC;
      }
     };



  TEST_F( LoopCloserFixture , fixtureCheck){
    EXPECT_EQ( 3, g.getNodes().size());
  }

  TEST_F( FourStepClosure, ClosuresFourStepLoop){
    

   EXPECT_CALL( graphMock, findTrails(0, 1))
      .Times(1)
      .WillOnce( Return( trailsTree));

     EXPECT_CALL( graphMock, findTrails(2, 3))
      .Times(1)
      .WillOnce( Return( trailsQuery));

     std::list<closure> resultingClosures = lC->findClosures(3,3,1);
     EXPECT_EQ( 2, resultingClosures.size());

     EXPECT_CALL( graphMock, getNode(3))
      .Times(2)
      .WillRepeatedly( Return( g.getNode(3)));

     EXPECT_CALL( graphMock, getNode(0))
      .Times(2)
      .WillRepeatedly( Return( g.getNode(0)));

     lC->sanitizeClosures( resultingClosures);
     EXPECT_EQ( 2, resultingClosures.size());

     std::pair<int,int> truthIndexes(0,3);
     EXPECT_EQ( truthIndexes, lC->findIndexesOptimization( resultingClosures, 3));

  }
}
