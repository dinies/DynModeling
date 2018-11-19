// Created by dinies on 26/10/2018.

#include <gtest/gtest.h>
#include "../../src/kdtree/kdTreeAltered.hpp"


using namespace testing;

namespace dyn_modeling {

  class kdTrailsFixture: public testing::Test {


    protected:
      std::vector<trail> trailsList;
 
      virtual void SetUp(){
        trail t1 = trail( Eigen::Vector2d(4,5),  4,5,7);
        trail t2 = trail( Eigen::Vector2d(5,4.5),0,1,1);
        trail t3 = trail( Eigen::Vector2d(4,5),  3,0,1);
        trail t4 = trail( Eigen::Vector2d(5,4.5),3,1,1);
        trailsList= { t1, t2, t3, t4};
      }

      virtual void TearDown(){
      }
     };


 TEST_F( kdTrailsFixture, kdtreeSimple){

    const double leafRange{ 0.2};
    const double maxDistance{ 0.5};
    BaseTreeNode* root= buildTree(trailsList, leafRange);

    trail answer;
    trail t = trail( Eigen::Vector2d(4,5),  0,0,1);
    double kd_dist = root->findNeighbor(answer, t, maxDistance);
    EXPECT_EQ( 0, kd_dist);
    EXPECT_EQ( 4, answer.nodeIndex);
    EXPECT_EQ( 5, answer.lineIndex);
    EXPECT_EQ( 7, answer.length);
  }

  TEST_F( kdTrailsFixture, kdtreeNoResults){

    const double leafRange{ 0.2};
    const double maxDistance{ 0.5};
    BaseTreeNode* root= buildTree(trailsList, leafRange);

    trail answer;
    trail t = trail( Eigen::Vector2d(4.6,5),  0,0,1);
    double kd_dist = root->findNeighbor(answer, t, maxDistance);
    EXPECT_EQ( -1, kd_dist);
  }
}
