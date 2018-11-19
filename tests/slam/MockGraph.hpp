// Created by dinies on 15/10/2018.

#pragma once
#include <gmock/gmock.h>

#include "../../src/slam/GraphInterface.hpp"

namespace dyn_modeling {

  class MockGraph: public GraphInterface{

    public:
      MOCK_METHOD1(insertNode, void(const node &t_node));
      MOCK_METHOD1(insertEdge, void(const edge &t_edge));
      MOCK_METHOD1(getEdge, edge( const int t_index ));
      MOCK_METHOD1(getNode, node( const int t_index ));

      MOCK_METHOD1(getEdgeRef, edge&( const int t_index ));
      MOCK_METHOD1(getNodeRef, node&( const int t_index ));

      MOCK_METHOD2(changeNode, void(const int t_index,const node &t_node));
      MOCK_METHOD2(changeEdge, void(const int t_index,const edge &t_edge));
      MOCK_METHOD0(getNodes,std::vector<node>());
      MOCK_METHOD0(getEdges,std::vector<edge>());
      MOCK_METHOD0(updateNodeStates, void());

      MOCK_METHOD2(findTrails,
          std::vector<trail>(
            const int t_nodeFromIndex,
            const int t_nodeToIndex ));

      MOCK_METHOD2(addOldTrails,
          void(
            std::vector<trail> &oldTrails,
            std::map< int, trail> &t_unmatchedTrails ));

      MOCK_METHOD2(matchActiveTrails,
          std::map<int,trail>(
            std::map<int,trail> &t_activeTrails,
            const int t_nodeIndex ));

      virtual ~MockGraph() {};
  };
}
