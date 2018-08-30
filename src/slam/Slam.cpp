// Created by Edoardo Ghini on 26/06/2018.

#include "Slam.hpp"
namespace dyn_modeling {


  Slam::Slam( const std::string &t_dataSet_AbsolPath, const std::vector<double> &t_initialRobotState):
    m_robot(  Robot( t_dataSet_AbsolPath, t_initialRobotState) ),
    m_scanMatcher( ScanMatcher()),
    m_map( Map())
  {}


  void Slam::cycle(){
    const int num_ranges = m_robot.getNumRanges();
    const int num_dataEntries = m_robot.getNumDataEntries();
    m_nodes.reserve(num_dataEntries);



    // const std::vector<double> initialGuessState = { 0, 0, 0};
    const int icpIterations_cap = 2;


    // std::vector<double> old_robotState;
    // old_robotState.reserve(3);
    // std::vector<scanPoint> oldSPoints_robotFrame;
    // oldSPoints_robotFrame.reserve(num_ranges);
    // std::vector<scanPoint> newSPoints_robotFrame;
    // newSPoints_robotFrame.reserve(num_ranges);
    // std::vector<double> new_robotState;
    // new_robotState.reserve(3);
    std::vector<scanPoint> drawingPoints_worldFrame;
    drawingPoints_worldFrame.reserve(num_ranges);
    roundResult icpRes;
    m_scanMatcher.setKernelThreshold(0.5);




    node currNode;
    currNode.state.reserve(3);
    currNode.transf2currState.reserve(3);
    currNode.scanPoints_robotFrame.reserve(num_ranges);
    currNode.lines.reserve(num_ranges);
    Map::showImg();


    for (int i = 0; i < num_dataEntries; ++i) {

      currNode.scanPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i);
      currNode.lines = m_lineGenerator.generateLines( currNode.scanPoints_robotFrame );

      if ( i == 0 ){
        currNode.state = m_robot.getState();
        currNode.transf2currState = {0,0,0};
      }
      else{
        // oldSPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i - 1);
        // oldSPoints_robotFrame = m_nodes.at(i - 1).scanPoints_robotFrame;
        // newSPoints_robotFrame = m_robot.retrieveScanPointsRobotFrame(i);

        icpRes = m_scanMatcher.icpRound(icpIterations_cap,initialGuessState,oldSPoints_robotFrame,newSPoints_robotFrame);
        m_robot.updateState( icpRes.delta_x);
        currNode.state = m_robot.getState();
        currNode.transf2currState = icpRes.delta_x;
      }
      m_nodes.push_back(currNode);

      //loop checker
      //TODO

      drawingPoints_worldFrame = m_robot.changeCoordsRobotToWorld(  currNode.scanPoints_robotFrame );
      preDrawingManagement(i-1);
      m_map.drawImages( newDrawingPoints_worldFrame , new_robotState , i);
      //TODO add lines to the drawing;  create line struct inside line generator class and implement a new line matching
      postDrawingManagement(i);
   }
    m_robot.plotStateEvolution(0.01);
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
    if (i%10==0) {
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
    for ( auto n : m_nodes){
      x.push_back( boost::make_tuple( curr_t, n.state.at(0)));
      y.push_back( boost::make_tuple( curr_t,n.state.at(1)));
      theta.push_back( boost::make_tuple( curr_t,n.state.at(2)));
      curr_t += t_delta_t;
      path.push_back( boost::make_tuple( n.state.at(0),n.state.at(1)));
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
