// Created by Dinies on 10/09/2018.

#include "Graph.hpp"
#include "Robot.hpp"

namespace dyn_modeling {
  Graph::Graph( const int t_dataEntriesNum, const int t_maxPointsNum):
    m_maxPointsNum( t_maxPointsNum)
  {
    m_nodes.reserve( t_dataEntriesNum );
    m_edges.reserve( t_dataEntriesNum );
  };

  Graph::Graph(){};


  void Graph::updateNodeStates(){
    node prev_n;
    edge e;

    for(int i=1; i< m_nodes.size();++i){
      prev_n = getNode(i-1);
      node& curr_n = getNodeRef(i);
      e = getEdge(i);
      curr_n.q.mu = Robot::boxPlus( prev_n.q.mu, e.delta_x);
    }

  }


  std::map<int,trail> Graph::matchActiveTrails
  (std::map<int,trail> &t_activeTrails,
   const int t_nodeIndex){

    node n = getNode(t_nodeIndex);
    edge e = getEdge(t_nodeIndex);

    std::map< int,trail> newActiveTrails;

    std::vector<int> linesUsed( n.lines.size(),0);

    for( int i = 0; i< e.associations.size(); ++i){
      dataAssociation dA = e.associations.at(i);
      std::map< int,trail>::iterator it = t_activeTrails.find( dA.old_line_index);

      if(it != t_activeTrails.end()){
        trail t = it->second;
        ++t.length;
        newActiveTrails.insert( std::pair<int,trail>( dA.new_line_index, t ));
        t_activeTrails.erase( dA.old_line_index);
        linesUsed.at(dA.new_line_index) = 1;
      }
      else{

        line l = n.lines.at( dA.new_line_index);
        std::vector<scanPoint> middlePointVec =
          Robot::computeMiddleScanPoints(  n.scanPoints_robotFrame.at(l.first_index),
                                           n.scanPoints_robotFrame.at(l.second_index),
                                           1);
        scanPoint middlePoint= middlePointVec.at(0);
        // std::cout << "p1 "<< n.scanPoints_robotFrame.at(l.first_index).coords(0)<<","<< n.scanPoints_robotFrame.at(l.first_index).coords(1)<< " ";
        // std::cout << ", p2 "<< n.scanPoints_robotFrame.at(l.second_index).coords(0)<<","<< n.scanPoints_robotFrame.at(l.second_index).coords(1)<< " ";
        // std::cout << " middle : " << middlePoint.coords(0) << ", "<< middlePoint.coords(1) << "\n";

        Eigen::VectorXd lineCenterCoords(2);
        lineCenterCoords <<
          middlePoint.coords(0), middlePoint.coords(1);

        trail t(lineCenterCoords, t_nodeIndex, dA.new_line_index, 1 );
        newActiveTrails.insert( std::pair<int,trail>( dA.new_line_index, t ));
        linesUsed.at(dA.new_line_index) = 1;
      }
    }
    line l;
    for (int j= 0; j<linesUsed.size(); ++j){
      if( linesUsed.at(j) == 0){
        l = n.lines.at(j);
        std::vector<scanPoint> middlePointVec =
          Robot::computeMiddleScanPoints(  n.scanPoints_robotFrame.at(l.first_index),
                                           n.scanPoints_robotFrame.at(l.second_index),
                                           1);
        scanPoint middlePoint= middlePointVec.at(0);
        // std::cout << "p1 "<< n.scanPoints_robotFrame.at(l.first_index).coords(0)<<","<< n.scanPoints_robotFrame.at(l.first_index).coords(1)<< " ";
        // std::cout << ", p2 "<< n.scanPoints_robotFrame.at(l.second_index).coords(0)<<","<< n.scanPoints_robotFrame.at(l.second_index).coords(1)<< " ";
        // std::cout << " middle : " << middlePoint.coords(0) << ", "<< middlePoint.coords(1) << "\n";

        Eigen::VectorXd lineCenterCoords(2);
        lineCenterCoords <<
          middlePoint.coords(0), middlePoint.coords(1);

        trail t(lineCenterCoords, t_nodeIndex, j, 1 );
        newActiveTrails.insert( std::pair<int,trail>( j, t ));
      }
    }
    return newActiveTrails;
  }


  void Graph::addOldTrails( std::vector<trail> &t_oldTrails,
                     std::map< int, trail> &t_unmatchedTrails){

    trail curr_trail;
    for( std::map<int,trail>::iterator it=t_unmatchedTrails.begin(); it!=t_unmatchedTrails.end();++it ){
      t_oldTrails.push_back(it->second);
      curr_trail = it->second;
      // std::cout << "trail : center " << curr_trail.lineCenterCoords(0) << "," << curr_trail.lineCenterCoords(1) << "node "<< curr_trail.nodeIndex <<" ";
      // std::cout << "line " << curr_trail.lineIndex << "length "<< curr_trail.length << "\n";
    }
  }

  std::vector<trail> Graph::findTrails(const int t_nodeFromIndex,
                                       const int t_nodeToIndex ){
    std::vector<trail> oldTrails;
    oldTrails.reserve( (t_nodeToIndex - t_nodeToIndex) * m_maxPointsNum);
    std::map< int, trail> currActiveTrails;
    std::map< int, trail> newActiveTrails;

    for ( int i= t_nodeFromIndex; i<= t_nodeToIndex;++i){

      newActiveTrails = matchActiveTrails( currActiveTrails, i );
      addOldTrails( oldTrails, currActiveTrails);

      currActiveTrails = newActiveTrails;
    }

    addOldTrails( oldTrails, newActiveTrails);
    return oldTrails;
  }
}

