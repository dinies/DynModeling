// Created by dinies on 12/07/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>
#include <Eigen/Core>

#include "../../src/slam/ScanMatcher.hpp"
#include "../../src/slam/Robot.hpp"
#include "../../include/structs.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( ScanMatcherClass)

  BOOST_AUTO_TEST_CASE(translationalICPscanMatchingFromOrigin) {

    ScanMatcher sM = ScanMatcher();
    std::vector<double> state = { 0 ,0 ,0};

    Eigen::Vector2d t(-3,0);
    scanPoint sp1;
    scanPoint sp2;
    scanPoint sp3;
    scanPoint sp4;
    scanPoint sp5;
    scanPoint sp6;
    scanPoint sp7;
    scanPoint sp8;
    scanPoint sp9;
    scanPoint sp10;
    Eigen::Vector2d c1(5,2);
    Eigen::Vector2d c2(4,1);
    Eigen::Vector2d c3(5,0);
    Eigen::Vector2d c4(4,-1);
    Eigen::Vector2d c5(4,-2);
    Eigen::Vector2d c6 = c1+t;
    Eigen::Vector2d c7 = c2+t;
    Eigen::Vector2d c8 = c3+t;
    Eigen::Vector2d c9 = c4+t;
    Eigen::Vector2d c10 = c5+t;
    sp1.coords = c1;
    sp2.coords = c2;
    sp3.coords = c3;
    sp4.coords = c4;
    sp5.coords = c5;
    sp6.coords = c6;
    sp7.coords = c7;
    sp8.coords = c8;
    sp9.coords = c9;
    sp10.coords = c10;

    const std::vector<scanPoint> oldSPoints_robot ={ sp1,sp2,sp3,sp4,sp5};
    const std::vector<scanPoint> newSPoints_robot ={ sp6,sp7,sp8,sp9,sp10};
    roundResult icpRes = sM.icpRound(5,state,oldSPoints_robot, newSPoints_robot);
    std::cout << " translational num of iters " << icpRes.chi.size() <<"\n";
    std::vector<double> truthTransf = { - t(0), - t(1),0};
    double threshold = 0.02;
    for ( int i = 0; i < 3; ++i){
      BOOST_CHECK_SMALL(icpRes.delta_x.at(i) - truthTransf.at(i),threshold);
    }
  }

  BOOST_AUTO_TEST_CASE(rotationalICPscanMatchingFromOrigin) {

    ScanMatcher sM = ScanMatcher();
    std::vector<double> state = { 0 ,0 ,0};
    double angle = M_PI/3;
    Eigen::Matrix2d R;
    R << cos(angle ), -sin(angle),
      sin(angle), cos(angle);

    scanPoint sp1;
    scanPoint sp2;
    scanPoint sp3;
    scanPoint sp4;
    scanPoint sp5;
    scanPoint sp6;
    scanPoint sp7;
    scanPoint sp8;
    scanPoint sp9;
    scanPoint sp10;
    Eigen::Vector2d c1(5,2);
    Eigen::Vector2d c2(4,1);
    Eigen::Vector2d c3(5,0);
    Eigen::Vector2d c4(4,-1);
    Eigen::Vector2d c5(4,-2);
    Eigen::Vector2d c6 = R * c1;
    Eigen::Vector2d c7 = R * c2;
    Eigen::Vector2d c8 = R * c3;
    Eigen::Vector2d c9 = R * c4;
    Eigen::Vector2d c10 = R * c5;
    sp1.coords = c1;
    sp2.coords = c2;
    sp3.coords = c3;
    sp4.coords = c4;
    sp5.coords = c5;
    sp6.coords = c6;
    sp7.coords = c7;
    sp8.coords = c8;
    sp9.coords = c9;
    sp10.coords = c10;

    const std::vector<scanPoint> oldSPoints_robot ={ sp1,sp2,sp3,sp4,sp5};
    const std::vector<scanPoint> newSPoints_robot ={ sp6,sp7,sp8,sp9,sp10};

    roundResult icpRes = sM.icpRound(5,state,oldSPoints_robot, newSPoints_robot);
    std::cout << " rotational num of iters " << icpRes.chi.size() <<"\n";
    std::vector<double> truthTransf = {0,0, - angle};
    double threshold = 0.02;
    for ( int i = 0; i < 3; ++i){
      BOOST_CHECK_SMALL( icpRes.delta_x.at(i) - truthTransf.at(i),threshold);
    }
  }

  // BOOST_AUTO_TEST_CASE(rotationalICPscanMatching) {

  //   ScanMatcher sM = ScanMatcher();
  //   double bias_x = 3;
  //   double bias_y = -2;
  //   double bias_theta = 0.3;
  //   std::vector<double> state = { bias_x ,bias_y ,bias_theta};
  //   double angle = M_PI/3;
  //   Eigen::Matrix2d R;
  //   R << cos(angle ), -sin(angle),
  //     sin(angle), cos(angle);

  //   scanPoint sp1;
  //   scanPoint sp2;
  //   scanPoint sp3;
  //   scanPoint sp4;
  //   scanPoint sp5;
  //   scanPoint sp6;
  //   scanPoint sp7;
  //   scanPoint sp8;
  //   scanPoint sp9;
  //   scanPoint sp10;
  //   Eigen::Vector2d c1(5,2);
  //   Eigen::Vector2d c2(4,1);
  //   Eigen::Vector2d c3(5,0);
  //   Eigen::Vector2d c4(4,-1);
  //   Eigen::Vector2d c5(4,-2);
  //   Eigen::Vector2d c6 = R * c1;
  //   Eigen::Vector2d c7 = R * c2;
  //   Eigen::Vector2d c8 = R * c3;
  //   Eigen::Vector2d c9 = R * c4;
  //   Eigen::Vector2d c10 = R * c5;
  //   sp1.coords = c1;
  //   sp2.coords = c2;
  //   sp3.coords = c3;
  //   sp4.coords = c4;
  //   sp5.coords = c5;
  //   sp6.coords = c6;
  //   sp7.coords = c7;
  //   sp8.coords = c8;
  //   sp9.coords = c9;
  //   sp10.coords = c10;

  //   const std::vector<scanPoint> oldSPoints_robot ={ sp1,sp2,sp3,sp4,sp5};
  //   const std::vector<scanPoint> newSPoints_robot ={ sp6,sp7,sp8,sp9,sp10};

  //   roundResult icpRes = sM.icpRound(5,state,oldSPoints_robot, newSPoints_robot);
  //   std::cout << " rotational num of iters " << icpRes.chi.size() <<"\n";
  //   std::vector<double> truthTransf = {0,0,angle};
  //   double threshold = 0.02;
  //   for ( int i = 0; i < 3; ++i){
  //     BOOST_CHECK_SMALL( icpRes.delta_x.at(i) - truthTransf.at(i),threshold);
  //   }
  // }

  BOOST_AUTO_TEST_SUITE_END()
}
