// Created by Edoardo Ghini on 26/06/2018.

#include "Slam.hpp"

namespace dyn_modeling {


  Slam::Slam( Eigen::Vector3d &t_initRState,
      paramsSlam &t_params,
      Robot &t_robot,
      ScanMatcher &t_scanM,
      Map &t_map,
      LineMatcher &t_lineM,
      Graph &t_graph,
      LoopCloser<Graph> &t_loopC):
    m_initialRobotState( t_initRState),
    m_params( t_params),
    m_robot( t_robot),
    m_scanMatcher( t_scanM),
    m_map( t_map),
    m_lineMatcher( t_lineM),
    m_graph( t_graph),
    m_loopCloser( t_loopC)
  {};



  void Slam::cycle(){
    const int num_ranges = m_robot.getNumRanges();
    const int num_dataEntries = m_robot.getNumDataEntries();


    m_scanMatcher.setKernelThreshold( m_params.kernelThresholdScanMatching);

    std::vector<scanPoint> drawingPoints_worldFrame;
    drawingPoints_worldFrame.reserve(num_ranges);

    resultMatchAssociations resMatch;
    resMatch.prevAssociationPoints.reserve( num_ranges);
    resMatch.currAssociationPoints.reserve( num_ranges);

    std::vector<scanPoint> prevAssociationSP_worldFrame;
    prevAssociationSP_worldFrame.reserve(num_ranges);

    std::vector<scanPoint> currAssociationSP_worldFrame;
    currAssociationSP_worldFrame.reserve(num_ranges);


    node currNode;
    currNode.scanPoints_robotFrame.reserve(num_ranges);
    currNode.lines.reserve(num_ranges);
    node prevNode;
    edge currEdge;

    std::vector<loopDrawingData>  currLoopDrawings;
    std::vector<loopDrawingData>  prevLoopDrawings;
    m_map.showImg();

    for (int i = 0; i < num_dataEntries; ++i) {

      currNode.scanPoints_robotFrame =
        m_robot.retrieveScanPointsRobotFrame(i, m_params.borderRatio);
      currNode.lines =
        m_lineMatcher.generateLines( currNode.scanPoints_robotFrame );

      // std::cout << currNode.lines.size() << " lines \n" ;

      if ( i == 0 ){
        currNode.q = m_initialRobotState;

      }
      else{
        prevNode = m_graph.getNode(i -1);

        DataAssociator associator
          (m_params.maxCandidatesAssociation,
           m_params.maxLengthDiffAssociation,
           m_params.maxAbsoluteOrientationDiffThreshold,
           m_params.maxNearLinesOrientationDiffThreshold,
           m_params.nearLinesBonusScoreMultiplier,
           prevNode.lines,prevNode.scanPoints_robotFrame,
           currNode.lines, currNode.scanPoints_robotFrame );

        currEdge.associations = associator.associateLines();
        // std::cout << currEdge.associations.size() << " associations \n" ;
        if ( currEdge.associations.size() > 0){
          resMatch = matchAssociatedData( prevNode,
                                          currEdge,
                                          currNode,
                                          m_params.icpIterationsCap,
                                          m_params.numMiddleScanPoints);
          currEdge.delta_x = resMatch.icpResult.delta_x;

          currNode.q= Robot::updateState( prevNode.q, currEdge.delta_x );
          // std::cout  << currEdge.delta_x(0) << "; "<< currEdge.delta_x(1) << "; " << currEdge.delta_x(2) << " dX \n";
          // Eigen::Vector3d curr_state= currNode.q.mu;
          // if ( std::isnan( curr_state(0)) || std::isnan( curr_state(1)) || std::isnan( curr_state(2)) ){
          //   std::cout << "NAN \n";
          // }
        }
        else{
          std::cout << "No associations at iteration : "<< i <<"\n";
        }
      }

      m_graph.insertNode( currNode);
      m_graph.insertEdge( currEdge);



      //loop checker
      if (i%m_params.everyNumIterTryLoopClosure ==0) {
        prevLoopDrawings = currLoopDrawings;
        currLoopDrawings=
          m_loopCloser.closeLoop(i, i-1, m_params.dimQuerySetLoopClosure);
      }

      drawingPoints_worldFrame =
        Robot::changeCoordsRobotToWorld(  currNode.scanPoints_robotFrame,
                                           currNode.q);
     prevAssociationSP_worldFrame =
        Robot::changeCoordsRobotToWorld( resMatch.prevAssociationPoints,
                                          prevNode.q);
      currAssociationSP_worldFrame =
        Robot::changeCoordsRobotToWorld( resMatch.currAssociationPoints,
                                          currNode.q);

      preDrawingManagement(i-1);
      m_map.drawImages( drawingPoints_worldFrame,
                        prevAssociationSP_worldFrame,
                        currAssociationSP_worldFrame,
                        m_params.numMiddleScanPoints,
                        currNode.q,
                        i);
      if (i%m_params.everyNumIterTryLoopClosure ==0) {
        m_map.drawLoopClosures( prevLoopDrawings, false);
        m_map.drawLoopClosures( currLoopDrawings, true);
      }

      postDrawingManagement(i);
      // std::cout << i << " iteration \n";
    }
    plotStateEvolution(0.01);
  }

  resultMatchAssociations Slam::matchAssociatedData
  (const node &t_prevNode,
   const edge &t_currEdge,
   const node &t_currNode,
   const int t_icpIterations_cap,
   const int t_numMidPoints){

    resultMatchAssociations result;
    result.prevAssociationPoints.reserve
      (t_currEdge.associations.size()*t_numMidPoints);
    result.currAssociationPoints.reserve
      (t_currEdge.associations.size()*t_numMidPoints);

    line linePrev;
    line lineCurr;
    Eigen::Vector3d icpInitialGuess;
    icpInitialGuess << 0 , 0, 0;

    scanPoint s1;
    scanPoint s2;
    scanPoint s3;


    for ( auto association : t_currEdge.associations){

      linePrev = t_prevNode.lines.at( association.old_line_index);
      s1 = t_prevNode.scanPoints_robotFrame.at( linePrev.first_index);
      s2 = t_prevNode.scanPoints_robotFrame.at( linePrev.second_index);
      for( auto s : Robot::computeMiddleScanPoints( s1, s2, t_numMidPoints)){
        result.prevAssociationPoints.push_back( s);
      }

      lineCurr= t_currNode.lines.at( association.new_line_index);
      s1 = t_currNode.scanPoints_robotFrame.at( lineCurr.first_index);
      s2 = t_currNode.scanPoints_robotFrame.at( lineCurr.second_index);
      for( auto s : Robot::computeMiddleScanPoints( s1, s2, t_numMidPoints)){
        result.currAssociationPoints.push_back( s);
      }


    }
    result.icpResult = m_scanMatcher.icpRound(t_icpIterations_cap,
                                              icpInitialGuess,
                                              result.prevAssociationPoints,
                                              result.currAssociationPoints);
    return result;
  }



  void  Slam::preDrawingManagement( const int t_index){
    const int i = t_index;
    if (i-10 >= 0){
      m_map.fadeRobot(i-10);
    }
    if (i-30 >= 0){
      m_map.deleteRobot(i-30);
    }

    if (i-1 >= 0){
      m_map.fadeScanPoints(i-1);
    }
    if (i-5 >= 0){
      m_map.deleteScanPoints(i-5);
    }

    if (i-1 >= 0){
      m_map.deleteLineAssociations(i-1);
    }

    if (i-10 >= 0){
      m_map.fadeWorldMap(i-10);
    }
  }

  void  Slam::postDrawingManagement( const int t_index){
    const int i = t_index;
    if (i%30==0) {
      m_map.drawTrail(0,i);
    }
    else{
      m_map.drawTrail(i,i);
    }
    m_map.showImg();
    cv::waitKey(1);
  }


  void Slam::plotStateEvolution(const double t_delta_t){
    double curr_t = 0;
    std::vector< boost::tuple<double,double>> x;
    std::vector< boost::tuple<double,double>> y;
    std::vector< boost::tuple<double,double>> theta;
    std::vector< boost::tuple<double,double>> path;
    std::vector< node> nodes =  m_graph.getNodes();

    for ( auto n : nodes){
      x.push_back( boost::make_tuple( curr_t, n.q.mu(0)));
      y.push_back( boost::make_tuple( curr_t, n.q.mu(1)));
      theta.push_back( boost::make_tuple( curr_t,n.q.mu(2)));
      curr_t += t_delta_t;
      path.push_back( boost::make_tuple( n.q.mu(0),n.q.mu(1)));
    }
    Gnuplot gp;
    gp << "set terminal qt 1\n";
    gp << "plot";
    gp << gp.binFile1d(x, "record") << "with lines title 'x'" << ",";
    gp << gp.binFile1d(y, "record") << "with lines title 'y'" << "\n";
    gp << "set terminal qt 2\n";
    gp << "plot";
    gp << gp.binFile1d(theta, "record") << "with lines title 'theta'" << "\n";
    gp << "set terminal qt 3\n";
    gp << "plot";
    gp << gp.binFile1d(path, "record") << "with lines title 'path'" << "\n";
    cv::waitKey(1);
  }
}
