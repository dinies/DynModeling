// Created by Edoardo Ghini on 26/06/2018.

#include "Slam.hpp"
namespace dyn_modeling {


  Slam::Slam( const std::string &t_dataSet_AbsolPath, const Eigen::Vector3d &t_initialRobotState, const double maxDistBetweenSPoints, const double maxAngularCoeff ):
    m_robot(  Robot( t_dataSet_AbsolPath, t_initialRobotState) ),
    m_scanMatcher( ScanMatcher()),
    m_map( Map()),
    m_lineMatcher( LineMatcher( maxDistBetweenSPoints, maxAngularCoeff))
  {
    m_graph = Graph( m_robot.getNumDataEntries());
  }


  void Slam::cycle(){
    const int num_ranges = m_robot.getNumRanges();
    const int num_dataEntries = m_robot.getNumDataEntries();


    const int icpIterations_cap = 4;
    const int maxCandidatesAssociation = 5;
    const double maxLengthDiffAssociation = 0.1;
    const double maxOrientationDiffAssociation = 0.2;

    std::vector<scanPoint> drawingPoints_worldFrame;
    drawingPoints_worldFrame.reserve(num_ranges);
    roundResult icpResult;
    m_scanMatcher.setKernelThreshold(0.5);




    node currNode;
    currNode.scanPoints_robotFrame.reserve(num_ranges);
    currNode.lines.reserve(num_ranges);
    node prevNode;
    edge currEdge;

    m_map.showImg();

    for (int i = 0; i < num_dataEntries; ++i) {

      currNode.scanPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i);
      currNode.lines = m_lineMatcher.generateLines( currNode.scanPoints_robotFrame );

      std::cout << currNode.lines.size() << " lines \n" ;

      if ( i == 0 ){
        currNode.q = m_robot.getState();

      }
      else{
        prevNode = m_graph.getNode(i -1);

        DataAssociator associator( maxCandidatesAssociation, maxLengthDiffAssociation, maxOrientationDiffAssociation, prevNode.lines, prevNode.scanPoints_robotFrame, currNode.lines, currNode. scanPoints_robotFrame );
        currEdge.associations = associator.associateLines();
        std::cout << currEdge.associations.size() << " associations \n" ;
        if ( currEdge.associations.size() > 0){
          icpResult = matchAssociatedData( prevNode, currEdge, currNode, icpIterations_cap );
          m_robot.updateState( icpResult.delta_x);
          currNode.q= m_robot.getState();
          currEdge.delta_x = icpResult.delta_x;
        }
        else{
          std::cout << "No associations\n";
        }
      }

      m_graph.insertNode( currNode);
      m_graph.insertEdge( currEdge);

      //loop checker
      //TODO

      drawingPoints_worldFrame = m_robot.changeCoordsRobotToWorld(  currNode.scanPoints_robotFrame );
      preDrawingManagement(i-1);
      m_map.drawImages( drawingPoints_worldFrame , currNode.q , i);
      //TODO add lines to the drawing;  create line struct inside line generator class and implement a new line matching
      postDrawingManagement(i);
      std::cout << i << " iteration \n";
    }
    plotStateEvolution(0.01);
  }

  roundResult Slam::matchAssociatedData(const node &t_prevNode, const edge &t_currEdge, const node &t_currNode, const int t_icpIterations_cap){
    std::vector<scanPoint> prevSPointsAssociated;
    prevSPointsAssociated.reserve( t_currEdge.associations.size()*2);
    std::vector<scanPoint> currSPointsAssociated;
    currSPointsAssociated.reserve( t_currEdge.associations.size()*2);
    line linePrev;
    line lineCurr;
    Eigen::Vector3d icpInitialGuess;
    icpInitialGuess << 0 , 0, 0;

    for ( auto association : t_currEdge.associations){

      linePrev = t_prevNode.lines.at( association.old_line_index);
      prevSPointsAssociated.push_back( t_prevNode.scanPoints_robotFrame.at( linePrev.first_index));
      prevSPointsAssociated.push_back( t_prevNode.scanPoints_robotFrame.at( linePrev.second_index));

      lineCurr= t_currNode.lines.at( association.new_line_index);
      currSPointsAssociated.push_back( t_currNode.scanPoints_robotFrame.at( lineCurr.first_index));
      currSPointsAssociated.push_back( t_currNode.scanPoints_robotFrame.at( lineCurr.second_index));
    }
    return m_scanMatcher.icpRound(t_icpIterations_cap ,icpInitialGuess, prevSPointsAssociated, currSPointsAssociated);
  }



  void  Slam::preDrawingManagement( const int t_index){
    const int i = t_index;
    if (i-1> 0){
      m_map.fadeRobot(i-1);
    }
    if (i-5> 0){
      m_map.deleteRobot(i-5);
    }

    if (i-20 > 0){
      m_map.fadeScanPoints(i-20);
    }
    if (i-200 > 0){
      m_map.deleteScanPoints(i-200);
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
      y.push_back( boost::make_tuple( curr_t,n.q.mu(1)));
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
  }
}
