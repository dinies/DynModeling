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
  } node;

  typedef struct edge_tag{
    Eigen::Vector3d delta_x;
    std::vector<dataAssociation> associations;
  } edge;

  class Graph{

  private:
    std::vector<node> m_nodes;
    std::vector<edge> m_edges;

  public:
    Graph( const int t_dataEntries );

    Graph();

    inline void insertNode( const node &t_node){ m_nodes.push_back(t_node); };

    inline void insertEdge( const edge &t_edge){ m_edges.push_back(t_edge); };

    inline edge getEdge( const int t_index ){return m_edges.at(t_index); };

    inline node getNode( const int t_index ){return m_nodes.at(t_index); };

    inline void changeNode( const int t_index, const node &t_node ){ m_nodes.at(t_index) = t_node; };

    inline void changeEdge( const int t_index, const edge &t_edge ){ m_edges.at(t_index) = t_edge; };

    inline std::vector<node> getNodes(){return m_nodes; };

    inline std::vector<edge> getEdges(){return m_edges; };

  };
}

