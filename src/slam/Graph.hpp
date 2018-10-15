// Created by Dinies on 10/09/2018.

#pragma once
#include "GraphInterface.hpp"

namespace dyn_modeling {

   class Graph: GraphInterface {

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

