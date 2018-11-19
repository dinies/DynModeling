// Created by Dinies on 09/08/2018.

#include "LineMatcher.hpp"

namespace dyn_modeling {
  LineMatcher::LineMatcher( double t_distanceBetweenRangesThreshold,
      double t_angularCoeffThreshold,
      const double t_minLength):
    m_distanceBetweenRangesThreshold(t_distanceBetweenRangesThreshold),
    m_angularCoeffThreshold( t_angularCoeffThreshold),
    m_minLengthThreshold( t_minLength)
  {};


  double LineMatcher::computeLength
    (const std::vector<scanPoint> &t_scanPoints,
     const int t_firstIndex,
     const int t_secondIndex ){
      scanPoint s1 = t_scanPoints.at( t_firstIndex);
      scanPoint s2 = t_scanPoints.at( t_secondIndex);
      return  sqrt( pow(s2.coords(0) - s1.coords(0), 2) +
          pow(s2.coords(1) - s1.coords(1), 2));
    }

  std::vector<line> LineMatcher::generateLines
    (const std::vector<scanPoint> &t_scanPoints_robotFrame){

      std::vector<line> lines;
      int currState = 0;
      scanPoint prevSPoint;
      double prev_laserRange;
      double curr_laserRange;
      double curr_segmentInclination;
      double curr_avg_lineInclination = 0;
      scanPoint sP;
      int first_index;
      int second_index;
      for (int i = 0; i < t_scanPoints_robotFrame.size(); ++i){

        sP = t_scanPoints_robotFrame.at(i);
        switch( currState){
          case 0:
            first_index = i;
            prev_laserRange = sqrt( pow(sP.coords(0),2) + pow(sP.coords(1),2) );
            prevSPoint = sP;
            currState = 1;
            break;

          case 1:
            curr_laserRange = sqrt( pow(sP.coords(0),2) + pow(sP.coords(1),2) );
            if (fabs( prev_laserRange - curr_laserRange ) <
                m_distanceBetweenRangesThreshold ){

              second_index = i;
              curr_segmentInclination =
                atan2( sP.coords(1) - prevSPoint.coords(1),
                    sP.coords(0) - prevSPoint.coords(0) );
              curr_avg_lineInclination=
                MyMath::computeAvg( curr_avg_lineInclination,
                    i - first_index,
                    curr_segmentInclination);
              prevSPoint = sP;
              prev_laserRange = curr_laserRange;
              currState = 2;
            }
            else{
              currState = 0;
            }
            break;
          case 2:
            curr_laserRange= sqrt(pow(sP.coords(0),2)+pow(sP.coords(1),2));
            curr_segmentInclination=atan2(sP.coords(1)-prevSPoint.coords(1),
                sP.coords(0) - prevSPoint.coords(0) );
            if ( fabs(curr_segmentInclination - curr_avg_lineInclination) < m_angularCoeffThreshold
                && fabs( prev_laserRange - curr_laserRange ) < m_distanceBetweenRangesThreshold ){

              second_index = i;
              curr_avg_lineInclination = MyMath::computeAvg( curr_avg_lineInclination,
                  i - first_index,
                  curr_segmentInclination);
              prevSPoint = sP;
              prev_laserRange = curr_laserRange;
            }
            else {
              if ( ! (fabs( prev_laserRange - curr_laserRange ) < m_distanceBetweenRangesThreshold )){
                if ( second_index - 1 > first_index &&
                    computeLength( t_scanPoints_robotFrame,
                      first_index,
                      second_index -1) > m_minLengthThreshold){
                  line new_line;
                  new_line.first_index = first_index;
                  new_line.second_index = second_index - 1;
                  lines.push_back( new_line);
                }
                currState = 0;
              }
              else {
                if ( computeLength( t_scanPoints_robotFrame,
                      first_index, second_index) > m_minLengthThreshold){
                  line new_line;
                  new_line.first_index = first_index;
                  new_line.second_index = second_index;
                  lines.push_back( new_line);
                }

                first_index = i-1;
                second_index = i;
                prevSPoint = t_scanPoints_robotFrame.at(i-1);
                curr_segmentInclination = atan2( sP.coords(1) - prevSPoint.coords(1),
                    sP.coords(0) - prevSPoint.coords(0) );
                curr_avg_lineInclination = MyMath::computeAvg( curr_avg_lineInclination,
                    i - first_index,
                    curr_segmentInclination);
                prevSPoint = sP;
                prev_laserRange = sqrt( pow(sP.coords(0),2) + pow(sP.coords(1),2) );
              }
            }
            break;
        }
      }
      if ( currState == 2 &&
          computeLength( t_scanPoints_robotFrame,
            first_index,
            second_index) > m_minLengthThreshold){
        line new_line;
        new_line.first_index = first_index;
        new_line.second_index = second_index;
        lines.push_back( new_line);
      }

      return lines;
    }
}

