// Created by Dinies on 02/07/2018.

#include "Robot.hpp"

namespace dyn_modeling {

  Robot::Robot( const std::string &t_dataSet_AbsolPath , const std::vector<double> &t_initial_state):
    m_datasetManager( DatasetManager( t_dataSet_AbsolPath))
  {
    m_state.q = t_initial_state;
    m_old_states.push_back(m_state);
  };



  std::vector<scanPoint> Robot::retrieveScanPointsRobotFrame( int t_index_datanode){
    std::vector<double> spanning_angles = m_datasetManager.getSpanningAngles();
    std::vector<double> data_ranges = m_datasetManager.getDataNodeRanges( t_index_datanode);
    std::vector<scanPoint> scan_points;
    scan_points.reserve(data_ranges.size());
    int index = 0;
    for ( auto angle : spanning_angles){
      scanPoint sP;
      Eigen::Matrix2d R;
      R << cos(angle ), -sin(angle),
        sin(angle), cos(angle);
      Eigen::Vector2d origin_range( data_ranges.at(index) ,0);
      Eigen::Vector2d coords = R* origin_range;
      sP.coords = coords;
      scan_points.push_back(sP);
      ++index;
    }
    return scan_points;
  };


  std::vector<scanPoint> Robot::changeCoordsRobotToWorld( const std::vector<scanPoint> &t_scanPoints_robotFrame){
    Eigen::Isometry2d transf = MyMath::v2t(m_state.q);
    std::vector<scanPoint> scanPvec_worldFrame;
    scanPvec_worldFrame.reserve( t_scanPoints_robotFrame.size());

    for ( auto sPoint : t_scanPoints_robotFrame){
      Eigen::Vector2d  sPvec_robot = sPoint.coords;
      scanPoint sPoint_world;
      sPoint_world.coords = transf * sPvec_robot;
      scanPvec_worldFrame.push_back( sPoint_world);
    }
    return scanPvec_worldFrame;
  };

  std::vector<double> Robot::boxPlus(const std::vector<double> t_first,const std::vector<double> t_second){
    std::vector<double> result;
    result.reserve(3);
    result.push_back( t_first.at(0) + t_second.at(0));
    result.push_back( t_first.at(1) + t_second.at(1));
    result.push_back( MyMath::boxPlusAngleRad(t_first.at(2), t_second.at(2)));
    return result;
  }

  std::vector<double> Robot::boxMinus(const std::vector<double> t_first,const std::vector<double> t_second){
    std::vector<double> result;
    result.reserve(3);
    result.push_back( t_first.at(0) - t_second.at(0));
    result.push_back( t_first.at(1) - t_second.at(1));
    result.push_back( MyMath::boxMinusAngleRad(t_first.at(2), t_second.at(2)));
    return result;
  }


  void Robot::updateState(const std::vector<double> &t_deltaState){
    m_old_states.push_back(m_state);
    m_state.q = Robot::boxPlus(m_state.q, t_deltaState);
  }

  void Robot::plotStateEvolution(const double t_delta_t){
    double curr_t = 0;
    std::vector< boost::tuple<double,double>> x;
    std::vector< boost::tuple<double,double>> y;
    std::vector< boost::tuple<double,double>> theta;
    std::vector< boost::tuple<double,double>> path;
    for ( auto s : m_old_states){
      x.push_back( boost::make_tuple( curr_t,s.q.at(0)));
      y.push_back( boost::make_tuple( curr_t,s.q.at(1)));
      theta.push_back( boost::make_tuple( curr_t,s.q.at(2)));
      curr_t += t_delta_t;
      path.push_back( boost::make_tuple( s.q.at(0),s.q.at(1)));
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
  };
}


