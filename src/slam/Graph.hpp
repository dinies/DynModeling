// Created by Dinies on 10/09/2018.

#pragma once
#include <unistd.h>
#include <vector>

// #include <limits>
// #include <string>
// #include <fstream>
// #include <iostream>
// #include <Eigen/Dense>

#include "../../include/structs.hpp"
// #include "../utils/MyMath.hpp"
#include "Robot.hpp"



namespace dyn_modeling {

  typedef struct node_tag{
    state q;
    std::vector<scanPoint> scanPoints_robotFrame;
    std::vector<line> lines;

    node_tag( state t_1,
              std::vector<scanPoint> t_2,
              std::vector<line> t_3):
      q( t_1),
      scanPoints_robotFrame( t_2),
      lines( t_3)
    {}
    node_tag():
      q()
    {
      scanPoints_robotFrame = {};
      lines = {};
    }
  } node;

  typedef struct edge_tag{
    Eigen::Vector3d delta_x;
    std::vector<dataAssociation> associations;

    edge_tag( Eigen::Vector3d t_1,
              std::vector<dataAssociation> t_2):
      delta_x( t_1),
      associations( t_2)
    {}
    edge_tag( double t_x,
              double t_y,
              double t_theta,
              std::vector<dataAssociation> t_associations):
      delta_x( t_x, t_y, t_theta),
      associations( t_associations)
    {}
    edge_tag(){
      delta_x = Eigen::Vector3d::Zero();
      associations = {};
    }
  } edge;

  typedef struct trail_tag{
    Eigen::VectorXd lineCenterCoords;
    int nodeIndex;
    int lineIndex;
    int length;

    trail_tag( Eigen::VectorXd t_1,
               int t_2,
               int t_3,
               int t_4):
      lineCenterCoords( t_1),
      nodeIndex( t_2),
      lineIndex( t_3),
      length( t_4)
    {}
    trail_tag():
      nodeIndex(-1),
      lineIndex(-1),
      length(-1)
   {
      lineCenterCoords = Eigen::Vector2d::Zero();
    }
  } trail;

  class Graph{

  private:
    int m_maxPointsNum;
    std::vector<node> m_nodes;
    std::vector<edge> m_edges;


  public:
    Graph( const int t_dataEntriesNum, const int t_maxPointsNum );

    Graph();

    inline void insertNode( const node &t_node){ m_nodes.push_back(t_node); };

    inline void insertEdge( const edge &t_edge){ m_edges.push_back(t_edge); };

    inline edge getEdge( const int t_index ){return m_edges.at(t_index); };

    inline node getNode( const int t_index ){return m_nodes.at(t_index); };

    inline edge& getEdgeRef( const int t_index ){return m_edges.at(t_index); };

    inline node& getNodeRef( const int t_index ){return m_nodes.at(t_index); };

    inline void changeNode( const int t_index, const node &t_node )
    { m_nodes.at(t_index) = t_node; };

    inline void changeEdge( const int t_index, const edge &t_edge )
    { m_edges.at(t_index) = t_edge; };

    inline std::vector<node> getNodes(){return m_nodes; };

    inline std::vector<edge> getEdges(){return m_edges; };

    void updateNodeStates();

    std::vector<trail> findTrails(const int t_nodeFromIndex,
                                  const int t_nodeToIndex );

    void addOldTrails( std::vector<trail> &oldTrails,
                       std::map< int, trail> &t_unmatchedTrails);

    std::map<int,trail> matchActiveTrails (std::map<int,trail> &t_activeTrails,
                                           const int t_nodeIndex);

  };
}

