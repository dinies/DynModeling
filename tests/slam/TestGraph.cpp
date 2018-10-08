// Created by dinies on 4/10/2018.

#define BOOST_TEST_MODULE SlamTests

#include <iostream>
#include <boost/test/unit_test.hpp>

#include "../../src/slam/Graph.hpp"
#include "../../src/slam/DataAssociator.hpp"


namespace dyn_modeling{
  BOOST_AUTO_TEST_SUITE( GraphClass)

  struct SimpleGraphFixture {

    Graph g;
    SimpleGraphFixture(){
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
    ~SimpleGraphFixture(){}

  };

  BOOST_AUTO_TEST_CASE(updateNodeStatesSimpleGraph) {
    SimpleGraphFixture fixture;
    edge& e_ab = fixture.g.getEdgeRef(1);
    edge& e_bc = fixture.g.getEdgeRef(2);

    e_ab.delta_x = Eigen::Vector3d( 2, 3, M_PI/2);
    e_bc.delta_x = Eigen::Vector3d( -2, 2, -M_PI/4);
    fixture.g.updateNodeStates();

    node n_a = fixture.g.getNode(0);
    node n_b = fixture.g.getNode(1);
    node n_c = fixture.g.getNode(2);
    state s_a = n_a.q;
    state s_b = n_b.q;
    state s_c = n_c.q;

    const double threshold = 0.01;

    BOOST_CHECK_SMALL( s_a.mu(0) - 0 , threshold);
    BOOST_CHECK_SMALL( s_a.mu(1) - 0 , threshold);
    BOOST_CHECK_SMALL( s_a.mu(2) - 0 , threshold);
    BOOST_CHECK_SMALL( s_b.mu(0) - 2 , threshold);
    BOOST_CHECK_SMALL( s_b.mu(1) - 3 , threshold);
    BOOST_CHECK_SMALL( s_b.mu(2) - M_PI/2, threshold);
    BOOST_CHECK_SMALL( s_c.mu(0) - 0 , threshold);
    BOOST_CHECK_SMALL( s_c.mu(1) - 1 , threshold);
    BOOST_CHECK_SMALL( s_c.mu(2) - M_PI/4, threshold);

  };

  BOOST_AUTO_TEST_CASE(findSimpleTrailsTest) {
    SimpleGraphFixture fixture;
    std::vector<trail> result = fixture.g.findTrails(0 , 2);

    BOOST_CHECK_EQUAL( 6 , result.size());

    const double threshold = 0.01;
    if ( result.size() == 6) {
      trail curr_trail;

      curr_trail = result.at(0);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(0) - 4 , threshold);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(1) - 3 , threshold);
      BOOST_CHECK_EQUAL( curr_trail.nodeIndex , 0);
      BOOST_CHECK_EQUAL( curr_trail.lineIndex , 2);
      BOOST_CHECK_EQUAL( curr_trail.length, 1);

      curr_trail = result.at(1);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(0) - 3 , threshold);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(1) - 2 , threshold);
      BOOST_CHECK_EQUAL( curr_trail.nodeIndex , 0);
      BOOST_CHECK_EQUAL( curr_trail.lineIndex , 1);
      BOOST_CHECK_EQUAL( curr_trail.length, 2);

      curr_trail = result.at(2);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(0) - 7 , threshold);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(1) - 3 , threshold);
      BOOST_CHECK_EQUAL( curr_trail.nodeIndex , 1);
      BOOST_CHECK_EQUAL( curr_trail.lineIndex , 2);
      BOOST_CHECK_EQUAL( curr_trail.length, 1);

      curr_trail = result.at(3);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(0) - 1 , threshold);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(1)-(-1), threshold);
      BOOST_CHECK_EQUAL( curr_trail.nodeIndex , 2);
      BOOST_CHECK_EQUAL( curr_trail.lineIndex , 0);
      BOOST_CHECK_EQUAL( curr_trail.length, 1);

      curr_trail = result.at(4);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(0) - 2, threshold);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(1) - 1, threshold);
      BOOST_CHECK_EQUAL( curr_trail.nodeIndex , 0);
      BOOST_CHECK_EQUAL( curr_trail.lineIndex , 0);
      BOOST_CHECK_EQUAL( curr_trail.length, 3);

      curr_trail = result.at(5);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(0) - 4 , threshold);
      BOOST_CHECK_SMALL( curr_trail.lineCenterCoords(1) - 2.5 , threshold);
      BOOST_CHECK_EQUAL( curr_trail.nodeIndex , 2);
      BOOST_CHECK_EQUAL( curr_trail.lineIndex , 2);
      BOOST_CHECK_EQUAL( curr_trail.length, 1);
    }
  };

  BOOST_AUTO_TEST_SUITE_END()
}
