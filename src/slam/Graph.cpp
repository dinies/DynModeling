// Created by Dinies on 10/09/2018.

#include "Graph.hpp"

namespace dyn_modeling {
  Graph::Graph( const int t_dataEntries) {
    m_nodes.reserve( t_dataEntries );
    m_edges.reserve( t_dataEntries );
  };

  Graph::Graph(){};

}

