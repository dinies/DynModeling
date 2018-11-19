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

    const double epsilon { 0.1};
    ScanMatcher sM = ScanMatcher( epsilon );
    Eigen::Vector3d initialGuess(0.0 ,0.0 ,0.0);

    Eigen::Vector2d t(-3,0);
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

    scanPoint sp1(c1);
    scanPoint sp2(c2);
    scanPoint sp3(c3);
    scanPoint sp4(c4);
    scanPoint sp5(c5);
    scanPoint sp6(c6);
    scanPoint sp7(c7);
    scanPoint sp8(c8);
    scanPoint sp9(c9);
    scanPoint sp10(c10);

    const std::vector<scanPoint> oldSPoints_robot ={ sp1,sp2,sp3,sp4,sp5};
    const std::vector<scanPoint> newSPoints_robot ={ sp6,sp7,sp8,sp9,sp10};
    roundResult icpRes = sM.icpRound(5,
                                     initialGuess,
                                     oldSPoints_robot,
                                     newSPoints_robot);
    std::cout << " translational num of iters " << icpRes.chi.size() <<"\n";
    std::vector<double> truthTransf = { - t(0), - t(1), - 0};
    double threshold = 0.02;
    for ( int i = 0; i < 3; ++i){
      BOOST_CHECK_SMALL(icpRes.delta_x(i) - truthTransf.at(i),threshold);
    }
  }

  BOOST_AUTO_TEST_CASE(rotationalICPscanMatchingFromOrigin) {

    const double epsilon { 0.1};
    ScanMatcher sM = ScanMatcher(epsilon);
    Eigen::Vector3d initialGuess(0.0 ,0.0 ,0.0);
    double angle = M_PI/3;
    Eigen::Matrix2d R;
    R << cos(angle ), -sin(angle),
      sin(angle), cos(angle);

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

    scanPoint sp1(c1);
    scanPoint sp2(c2);
    scanPoint sp3(c3);
    scanPoint sp4(c4);
    scanPoint sp5(c5);
    scanPoint sp6(c6);
    scanPoint sp7(c7);
    scanPoint sp8(c8);
    scanPoint sp9(c9);
    scanPoint sp10(c10);

    const std::vector<scanPoint> oldSPoints_robot ={ sp1,sp2,sp3,sp4,sp5};
    const std::vector<scanPoint> newSPoints_robot ={ sp6,sp7,sp8,sp9,sp10};


    roundResult icpRes = sM.icpRound(5,
                                     initialGuess,
                                     oldSPoints_robot,
                                     newSPoints_robot);
    std::cout << " rotational num of iters " << icpRes.chi.size() <<"\n";
    std::vector<double> truthTransf = { -0, - 0, -angle};
    double threshold = 0.02;
    for ( int i = 0; i < 3; ++i){
      BOOST_CHECK_SMALL( icpRes.delta_x(i) - truthTransf.at(i),threshold);
    }
  }

  BOOST_AUTO_TEST_CASE(rototranslationalICPscanMatching) {

    const double epsilon { 0.1};
    ScanMatcher sM = ScanMatcher( epsilon);
    Eigen::Vector3d initialGuess(0.0 ,0.0 ,0.0);
    double angle = M_PI/3;
    Eigen::Matrix2d R;
    R << cos(angle ), -sin(angle),
      sin(angle), cos(angle);
    Eigen::Vector2d t(-3,0);

        Eigen::Vector2d c1(5,2);
    Eigen::Vector2d c2(4,1);
    Eigen::Vector2d c3(5,0);
    Eigen::Vector2d c4(4,-1);
    Eigen::Vector2d c5(4,-2);
    Eigen::Vector2d c6 = (R * c1) + t;
    Eigen::Vector2d c7 = (R * c2) + t;
    Eigen::Vector2d c8 = (R * c3) + t;
    Eigen::Vector2d c9 = (R * c4) + t;
    Eigen::Vector2d c10 =(R * c5) + t;

    scanPoint sp1(c1);
    scanPoint sp2(c2);
    scanPoint sp3(c3);
    scanPoint sp4(c4);
    scanPoint sp5(c5);
    scanPoint sp6(c6);
    scanPoint sp7(c7);
    scanPoint sp8(c8);
    scanPoint sp9(c9);
    scanPoint sp10(c10);

    const std::vector<scanPoint> oldSPoints_robot ={ sp1,sp2,sp3,sp4,sp5};
    const std::vector<scanPoint> newSPoints_robot ={ sp6,sp7,sp8,sp9,sp10};

    roundResult icpRes = sM.icpRound(5,
                                     initialGuess,
                                     oldSPoints_robot,
                                     newSPoints_robot);

    std::cout << " rototraslational num of iters " << icpRes.chi.size() <<"\n";
    std::vector<double> truthTransf = { - t(0), - t(1), -angle };
    double threshold = 0.02;
    for ( int i = 0; i < 3; ++i){
      BOOST_CHECK_SMALL( icpRes.delta_x(i) - truthTransf.at(i),threshold);
    }
  }

  BOOST_AUTO_TEST_SUITE_END()
}
