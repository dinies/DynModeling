// Created by Dinies on 09/08/2018.

#include "LineMatcher.hpp"

namespace dyn_modeling {
  LineMatcher::LineMatcher( double t_distanceBetweenSPointsThreshold, double t_angularCoeffThreshold ):
    m_distanceBetweenSPointsThreshold(t_distanceBetweenSPointsThreshold),
    m_angularCoeffThreshold( t_angularCoeffThreshold)
  {};



  std::vector<line> LineMatcher::generateLines(const std::vector<scanPoint> &t_scanPoints_robotFrame){
    std::vector<line> lines;
    int currState = 0;
    scanPoint prevSPoint;
    double prev_laserRange;
    double curr_laserRange;
    double curr_segmentInclination;
    double curr_avg_lineInclination = 0;
    scanPoint sP;
    for (int i = 0; i < t_scanPoints_robotFrame.size(); ++i){

      sP = t_scanPoints_robotFrame.at(i);
      switch( currState){
      case 0:
        line curr_line;
        curr_line.first_index = i;
        prev_laserRange = sqrt( pow(sP.coords(0),2) + pow(sP.coords(1),2) );
        prevSPoint = sP;
        currState = 1;
        break;

      case 1:
        curr_laserRange = sqrt( pow(sP.coords(0),2) + pow(sP.coords(1),2) );
        if (fabs( prev_laserRange - curr_laserRange ) < m_distanceBetweenSPointsThreshold ){
          curr_line.second_index = i;
          curr_segmentInclination = atan2( sP.coords(1) - prevSPoint.coords(1), sP.coords(0) - prevSPoint.coords(0) );
          curr_avg_lineInclination = MyMath::computeAvg( curr_avg_lineInclination, i - curr_line.first_index, curr_segmentInclination);
          prevSPoint = sP;
          prev_laserRange = curr_laserRange;
          currState = 2;
          if ( i == t_scanPoints_robotFrame.size() -1){
            lines.push_back( curr_line);
          }
        }
        else{
          currState = 1;
        }
        break;
      case 2:
        curr_laserRange = sqrt( pow(sP.coords(0),2) + pow(sP.coords(1),2) );
        curr_segmentInclination = atan2( sP.coords(1) - prevSPoint.coords(1), sP.coords(0) - prevSPoint.coords(0) );
        if ( fabs(curr_segmentInclination - curr_avg_lineInclination) < m_angularCoeffThreshold && fabs( prev_laserRange - curr_laserRange ) < m_distanceBetweenSPointsThreshold ){

          curr_avg_lineInclination = MyMath::computeAvg( curr_avg_lineInclination, i - curr_line.first_index, curr_segmentInclination);
          prevSPoint = sP;
          prev_laserRange = curr_laserRange;
          if ( i == t_scanPoints_robotFrame.size() -1){
            lines.push_back( curr_line);
          }
        }
        else {
          if ( ! (fabs( prev_laserRange - curr_laserRange ) < m_distanceBetweenSPointsThreshold )){
            currState = 0;
          }
          else {
            lines.push_back( curr_line);
            line curr_line;
            curr_line.first_index = i-1;
            curr_line.second_index = i;
            prevSPoint = t_scanPoints_robotFrame.at(i-1);
            curr_segmentInclination = atan2( sP.coords(1) - prevSPoint.coords(1), sP.coords(0) - prevSPoint.coords(0) );
            curr_avg_lineInclination = MyMath::computeAvg( curr_avg_lineInclination, i - curr_line.first_index, curr_segmentInclination);
            prevSPoint = sP;
            prev_laserRange = sqrt( pow(sP.coords(0),2) + pow(sP.coords(1),2) );
            if ( i == t_scanPoints_robotFrame.size() -1){
              lines.push_back( curr_line);
            }
          }
        }
        break;
      }
    }
    return lines;
  }
}

