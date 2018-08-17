// Created by Dinies on 09/08/2018.

#include "LineMatcher.hpp"

namespace dyn_modeling {
  LineMatcher::LineMatcher( const double t_kernelThreshold = 0.2, const double t_distanceBetweenSPointsThreshold = 0.5, const double t_angularCoeffThreshold = 0.1):
    m_kernelThreshold( t_kernelThreshold),
    m_distanceBetweenSPointsThreshold(t_distanceBetweenSPointsThreshold),
    m_angularCoeffThreshold( t_angularCoeffThreshold)
  {};

  std::vector<line> LineMatcher::generateLines(const std::vector<scanPoint> t_scanPoints_robotFrame){
    std::vector<line> lines;

    bool flag_just_eliminated_segment = false;
    bool flag_secondIndex_undefined = true;
    line curr_line;
    curr_line.first_index = 0;

    double curr_avg_lineInclination;
    scanPoint prevSPoint = t_scanPoints_robotFrame.at(0);
    double prev_laserRange =  sqrt( pow(prevSPoint.coords.at(0),2) + pow(prevSPoint.coords.at(1),2) );
    double curr_segmentInclination;
    double curr_laserRange;


    for (int i = 1; i < t_scanPoints_robotFrame.size()-2; ++i){

      if ( flag_just_eliminated_segment){
        flag_secondIndex_undefined = true;
        line curr_line;
        curr_line.first_index = i;
      }
      else{
        scanPoint sP = t_scanPoints_robotFrame.at(i);
        curr_laserRange = sqrt( pow(sP.coords.at(0),2) + pow(sP.coords.at(1),2) );
        curr_segmentInclination = atan2( sP.coords.at(1) - prevSPoint.coords.at(1), sP.coords.at(0) - prevSPoint.coords.at(0) );

        // if ( flag_secondIndex_undefined ){
        //   //add point to line
        //   curr_line.second_index = i;
        //   flag_secondIndex_undefined = false;
        //   curr_avg_lineInclination = MyMath.computeAvg( curr_avg_lineInclination, i - curr_line.first_index, curr_segmentInclination);

        // }
        // else{

       if ( abs(curr_segmentInclination - curr_avg_lineInclination) < m_angularCoeffThreshold && abs( prev_laserRange - curr_laserRange ) < m_distanceBetweenSPointsThreshold ){

            //add point to line
            curr_line.second_index = i;
            flag_secondIndex_undefined = false;
            curr_avg_lineInclination = MyMath.computeAvg( curr_avg_lineInclination, i - curr_line.first_index, curr_segmentInclination);

          }
          else {
            //close line
            if (!flag_secondIndex_undefined){
              lines.push_back( curr_line);
            }
            if ( ! abs( prev_laserRange - curr_laserRange ) < m_distanceBetweenSPointsThreshold ){
              // eliminating the segment
              flag_just_eliminated_segment = true;
            }
            else{
              // mantain current segment
              line curr_line;
              curr_line.first_index = i;
              flag_secondIndex_undefined = true;
            }
          }
        }
        //end
        prev_laserRange =  sqrt( pow(sP.coords.at(0),2) + pow(sP.coords.at(1),2) );
        prevSPoint = sP;
      }
      return lines;
      //TODO     finish and debug and test
    }
  }
}

