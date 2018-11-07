// Created by Dinies on 8/10/2018.

namespace dyn_modeling {

  template< class T>
    LoopCloser<T>::LoopCloser( T &t_graph,
        const double t_maxLinesLengthDiff,
        const double t_maxLinesOrientDiff,
        const double t_leafRangeKdtree,
        const double t_maxDistanceKdtree,
        const double t_thresholdLoopRecognition
        ):
      m_graph( t_graph),
      m_maxLinesLengthDiff( t_maxLinesLengthDiff),
      m_maxLinesOrientDiff( t_maxLinesOrientDiff),
      m_leafRangeKdtree( t_leafRangeKdtree),
      m_maxDistanceKdtree( t_maxDistanceKdtree),
      m_thresholdLoopRecognition( t_thresholdLoopRecognition)
  {}


  template< class T>
    void LoopCloser<T>::closeLoop( const int t_currIteration,
        const int t_backRange,
        const int t_querySetRange ){


      std::cout << "begin closure iter:"<< t_currIteration << "\n";
      std::list<closure> closures=
        findClosures( t_currIteration,
            t_backRange,
            t_querySetRange);

      if ( closures.size() > 0){
        sanitizeClosures( closures);
      }
      if ( closures.size() > 0){
        std::pair<int,int> optimIndexes =
          LoopCloser::findIndexesOptimization(closures,t_currIteration);
      }

      std::cout << "end  closure \n";
    }

  template< class T>
    std::pair<int,int> LoopCloser<T>::findIndexesOptimization
    (const std::list< closure > &t_closures,
     const int t_currIteration){
      int minIndex = -1;
      int maxIndex = t_currIteration;

      for (std::list<closure>::const_iterator it=t_closures.begin();
          it != t_closures.end(); ++it){
        if ( it->newerTrail.nodeIndex < maxIndex ){
          maxIndex = it->newerTrail.nodeIndex;
        }
        if ( it->olderTrail.nodeIndex > minIndex ){
          minIndex = it->olderTrail.nodeIndex;
        }
      }
      return std::pair<int,int>( minIndex, maxIndex);
    }

  template< class T>
    void LoopCloser<T>::sanitizeClosures
    (std::list< closure > &t_closures){

      trail t1;
      trail t2;
      node n1;
      node n2;
      line l1;
      line l2;
      std::vector< scanPoint> pointsWorld1;
      std::vector< scanPoint> pointsWorld2;
      double length1;
      double length2;
      double orient1;
      double orient2;
      std::list< closure >::iterator it=t_closures.begin();

      while( it != t_closures.end()){

        t1 = it->newerTrail;
        t2 = it->olderTrail;
       // std::cout << "Closure between node "<< t1.nodeIndex << " and " << t2.nodeIndex << "\n";
        n1 = m_graph.getNode( t1.nodeIndex);
        n2 = m_graph.getNode( t2.nodeIndex);
        l1 = n1.lines.at( t1.lineIndex);
        l2 = n2.lines.at( t2.lineIndex);
        pointsWorld1 =
          Robot::changeCoordsRobotToWorld( n1.scanPoints_robotFrame,
              n1.q);
        pointsWorld2 =
          Robot::changeCoordsRobotToWorld( n2.scanPoints_robotFrame,
              n2.q);
        length1 = DataAssociator::getLineLength( l1, pointsWorld1);
        length2 = DataAssociator::getLineLength( l2, pointsWorld2);
        orient1 = DataAssociator::getLineOrientation( l1, pointsWorld1);
        orient2 = DataAssociator::getLineOrientation( l2, pointsWorld2);

        if ( fabs( length1 - length2) > m_maxLinesLengthDiff ||
            fabs( orient1 - orient2) > m_maxLinesOrientDiff ){

          it = t_closures.erase(it); 
          //automatically increments, since all the following
          //elements are shifted to the left !!
        }
        else{
          int temporalDistance = t1.nodeIndex - t2.nodeIndex;
          double scoreClosure = temporalDistance * (
              fabs(fabs(length1- length2)-
                m_maxLinesLengthDiff)*0.5/m_maxLinesLengthDiff+
              fabs(fabs(orient1- orient2)-
                m_maxLinesOrientDiff)*0.5/m_maxLinesOrientDiff);


          if ( scoreClosure <= m_thresholdLoopRecognition) {
            it = t_closures.erase(it); 
          }
          else{
            it->score = scoreClosure;
            std::cout << "closure detected \n ";
            ++it;
          }
        }
      }
    }

  template< class T>
    std::list< closure > LoopCloser<T>::findClosures
    (const int t_currIteration,
     const int t_backRange,
     const int t_querySetRange){

      std::list< closure > closures;

      const int treeIndexFrom = t_currIteration - t_backRange;
      const int treeIndexTo = t_currIteration - 2*t_querySetRange;
      const int queryIndexFrom = t_currIteration - t_querySetRange;

      if ( treeIndexFrom >= 0 && treeIndexTo>=0 && queryIndexFrom >=0){
        std::vector<trail> treeTrails =
          m_graph.findTrails( treeIndexFrom,treeIndexTo);

        std::vector<trail> queryTrails =
          m_graph.findTrails( queryIndexFrom,t_currIteration);

        BaseTreeNode* root= buildTree(treeTrails, m_leafRangeKdtree);

        closure c;
        trail answer;
        for ( auto t: queryTrails){
          double kd_dist = root->findNeighbor(answer, t, m_maxDistanceKdtree);
          if ( kd_dist >= 0){
            c = closure( t, answer);
            closures.push_back( c );
          }
        }
      }
      return closures;
    }
}

