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
    std::vector<loopDrawingData> LoopCloser<T>::closeLoop(
        const int t_currIteration,
        const double t_ratioQuerySet){

      std::cout << "begin closure iter:"<< t_currIteration << "\n";

      std::vector<loopDrawingData> loopDrawings;
      std::list<closure> closures=
        findClosures( t_currIteration,t_ratioQuerySet);

      if ( closures.size() > 0){
        loopDrawings.reserve( closures.size());
        sanitizeClosures(closures, loopDrawings, t_currIteration);
      }
      if ( closures.size() > 0){
        std::pair<int,int> optimIndexes =
          LoopCloser::findIndexesOptimization(closures,t_currIteration);
      }

      std::cout << "end  closure \n";

      return loopDrawings;
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
    (std::list< closure > &t_closures,
     std::vector<loopDrawingData> &t_loopDrawings,
     const int t_currentIteration
     ){

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
          // This takes advantage that the iterator automatically will
          // point to the next element , since all the following
          // elements have been shifted to the left
        }
        else{
          int temporalDistance = t1.nodeIndex - t2.nodeIndex;
          double scoreClosure = temporalDistance * (
              fabs(fabs(length1- length2)-
                m_maxLinesLengthDiff)*0.5/m_maxLinesLengthDiff+
              fabs(fabs(orient1- orient2)-
                m_maxLinesOrientDiff)*0.5/m_maxLinesOrientDiff) / t_currentIteration;

          std::cout << scoreClosure << "\n";



          if ( scoreClosure <= m_thresholdLoopRecognition) {
            it = t_closures.erase(it); 
          }
          else{
            it->score = scoreClosure;

            scanPoint curr_edge1 = pointsWorld1.at(l1.first_index);
            scanPoint curr_edge2 = pointsWorld1.at(l1.second_index);
            std::vector<scanPoint> curr_middle_vec = Robot::computeMiddleScanPoints(
                curr_edge1,
                curr_edge2,
                1);

            scanPoint curr_middle = curr_middle_vec.at(0);

            scanPoint prev_edge1 = pointsWorld2.at(l2.first_index);
            scanPoint prev_edge2 = pointsWorld2.at(l2.second_index);
            std::vector<scanPoint> prev_middle_vec =  Robot::computeMiddleScanPoints(
                prev_edge1,
                prev_edge2,
                1);

            scanPoint prev_middle = prev_middle_vec.at(0);

            loopDrawingData loopDrawing(
                curr_edge1.coords,
                curr_edge2.coords,
                curr_middle.coords,
                prev_edge1.coords,
                prev_edge2.coords,
                prev_middle.coords
                );

            t_loopDrawings.push_back(loopDrawing);
            std::cout << "closure detected \n ";
            std::cout << "Closure between node "<< t1.nodeIndex << " and " << t2.nodeIndex << "\n";
            ++it;
          }
        }
      }
    }

  template< class T>
    std::list< closure > LoopCloser<T>::findClosures
    (const int t_currIteration,
     const double t_ratioQuerySet){

      std::list< closure > closures;
      const int querySetDim = (int) floor(t_currIteration* t_ratioQuerySet);

      const int treeIndexFrom = 3;
      const int treeIndexTo = t_currIteration - 2*querySetDim;
      const int queryIndexFrom = t_currIteration - querySetDim;

      if ( treeIndexFrom >= 0 && treeIndexTo > treeIndexFrom && queryIndexFrom >= treeIndexTo){
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

