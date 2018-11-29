// Created by Dinies on 04/09/2018.

#include "DataAssociator.hpp"

namespace dyn_modeling {

  DataAssociator::DataAssociator( const int t_maxCandidates,
                  const double t_lengthDifferenceThreshold,
                  const double t_absoluteOrientationDiffThreshold,
                  const double t_nearLinesOrientationDiffThreshold,
                  const double t_nearLinesBonusScoreMultiplier,
                  const std::vector<line> &t_old_lines,
                  const std::vector<scanPoint> &t_old_sPoints,
                  const std::vector<line> &t_new_lines,
                  const std::vector<scanPoint> &t_new_sPoints):
    m_maxCandidates(t_maxCandidates),
    m_lengthDifferenceThreshold( t_lengthDifferenceThreshold),
    m_absoluteOrientationDiffThreshold( t_absoluteOrientationDiffThreshold),
    m_nearLinesOrientationDiffThreshold( t_nearLinesOrientationDiffThreshold),
    m_nearLinesBonusScoreMultiplier( t_nearLinesBonusScoreMultiplier),
    m_oldLines( t_old_lines),
    m_oldSPoints( t_old_sPoints),
    m_newLines( t_new_lines),
    m_newSPoints( t_new_sPoints)
  {};

  DataAssociator::DataAssociator(){};



  std::vector<dataAssociation> DataAssociator::associateLines(){

    std::vector< std::vector< dataAssociation> > matrix_associations;
    matrix_associations.reserve( m_newLines.size());

    for (int i = 0; i < m_newLines.size(); ++i){
      matrix_associations.push_back( findCandidates( i ));
    }
    return chooseBestAssociations( matrix_associations);
  }

  std::vector<dataAssociation> DataAssociator::chooseBestAssociations
  (std::vector< std::vector< dataAssociation>> &t_matrix){

    std::vector<dataAssociation> associations;
    associations.reserve( m_newLines.size());
    while ( !associationsAllTaken(t_matrix) ){
      dataAssociation curr_bestAssociation =
        chooseMaxScoreAssociation( t_matrix);
      removeTakenAssociations( curr_bestAssociation, t_matrix);
      associations.push_back( curr_bestAssociation);
    }
    return associations;
  }


  bool DataAssociator::associationsAllTaken
  (const std::vector< std::vector< dataAssociation>> &t_matrix){

    bool allTaken = true;
    for (auto vec : t_matrix){
      if ( !vec.empty()){
        allTaken = false;
      }
    }
    return allTaken;
  }


  dataAssociation DataAssociator::chooseMaxScoreAssociation
  (const std::vector< std::vector< dataAssociation>> &t_matrix){

    dataAssociation bestAssociation;
    bestAssociation.confidence_score = -100;
    for (auto vec : t_matrix){
      if ( !vec.empty()){
        dataAssociation currAssociation = vec.front();
        if ( currAssociation.confidence_score >
             bestAssociation.confidence_score){
          bestAssociation = currAssociation;
        }
      }
    }
    return bestAssociation;
  }

  void DataAssociator::removeTakenAssociations
  (const dataAssociation &t_takenAssociation,
    std::vector< std::vector< dataAssociation>> &t_matrix ){

    for (auto &vec : t_matrix){
      vec.erase
        ( std::remove_if
          (vec.begin(),
           vec.end(),
           [t_takenAssociation](dataAssociation dA) -> bool{
             return dA.old_line_index == t_takenAssociation.old_line_index ||
               dA.new_line_index == t_takenAssociation.new_line_index;
           }
           ), vec.end() );
    }
  }


  std::vector<dataAssociation> DataAssociator::findCandidates
  (const int t_choosen_newLine_index){

    std::vector< dataAssociation> candidates;
    candidates.reserve( m_maxCandidates);
    std::vector< int> good_indexes =
      getPossibleCandidateIndexes( t_choosen_newLine_index );
    for( auto i : good_indexes){
      insertOrderedDataAssociation(compareLines(i, t_choosen_newLine_index),
                                   candidates);
    }
    return candidates;
  }


  void DataAssociator::insertOrderedDataAssociation
  (const dataAssociation &t_lineComparison,
   std::vector<dataAssociation> &t_candidates){

    if ( t_lineComparison.confidence_score >= 0){
      bool inserted = false;
      int i = 0;
      while ( !inserted && i < t_candidates.size() ){
        if ( t_candidates.at(i).confidence_score <=
             t_lineComparison.confidence_score){
          t_candidates.insert( t_candidates.begin()+i, t_lineComparison);
          inserted = true;
        }
        ++i;
      }
      if (!inserted){
        t_candidates.insert( t_candidates.end(), t_lineComparison);
      }
    }
  }

  dataAssociation DataAssociator::compareLines( const int t_oldLine_index,
                                                const int t_newLine_index ){
    dataAssociation result;
    result.old_line_index = t_oldLine_index;
    result.new_line_index = t_newLine_index;
    double oldLength =
      DataAssociator::getLineLength( m_oldLines.at(t_oldLine_index),
                                     m_oldSPoints);
    double newLength =
      DataAssociator::getLineLength( m_newLines.at(t_newLine_index),
                                     m_newSPoints);
    double lengthDiff = fabs(oldLength - newLength);
    line old_line = m_oldLines.at(t_oldLine_index );
    line new_line = m_newLines.at(t_newLine_index );
    double old_line_orientation =
      DataAssociator::getLineOrientation(old_line, m_oldSPoints);
    double new_line_orientation =
      DataAssociator::getLineOrientation(new_line, m_newSPoints);
    double absoluteOriDiff = fabs(old_line_orientation -new_line_orientation);

    if (lengthDiff > m_lengthDifferenceThreshold ||
        absoluteOriDiff > m_absoluteOrientationDiffThreshold){
      result.confidence_score = -1;
    }
    else{
      result.confidence_score = 100 -
        (60 * lengthDiff/ m_lengthDifferenceThreshold)-
        (40 * absoluteOriDiff/ m_absoluteOrientationDiffThreshold);
    }
    if ( (t_newLine_index -1) >= 0 && (t_oldLine_index -1) >= 0){
      line old_lower_neighboor = m_oldLines.at(t_oldLine_index -1);
      line new_lower_neighboor = m_newLines.at(t_newLine_index -1);

      if ( old_lower_neighboor.second_index == old_line.first_index &&
           new_lower_neighboor.second_index == new_line.first_index){
        double old_low_neigh_ori =
          DataAssociator::getLineOrientation( old_lower_neighboor,
                                              m_oldSPoints);
        double new_low_neigh_ori =
          DataAssociator::getLineOrientation( new_lower_neighboor,
                                              m_newSPoints);

        if (fabs(MyMath::boxMinusAngleRad
                 ( MyMath::boxMinusAngleRad( old_low_neigh_ori,
                                             old_line_orientation),
                   MyMath::boxMinusAngleRad( new_low_neigh_ori,
                                             new_line_orientation))
                 ) < m_nearLinesOrientationDiffThreshold){
          result.confidence_score = result.confidence_score *
            m_nearLinesBonusScoreMultiplier;
        }
      }
    }

    if ((t_newLine_index +1) < m_newLines.size() &&
        (t_oldLine_index +1) < m_oldLines.size()){

      line old_upper_neighboor = m_oldLines.at(t_oldLine_index +1);
      line new_upper_neighboor = m_newLines.at(t_newLine_index +1);

      if ( old_upper_neighboor.second_index == old_line.first_index &&
           new_upper_neighboor.second_index == new_line.first_index){

        double old_up_neigh_ori =
          DataAssociator::getLineOrientation( old_upper_neighboor,
                                              m_oldSPoints);
        double new_up_neigh_ori =
          DataAssociator::getLineOrientation( new_upper_neighboor,
                                              m_newSPoints);

        if ( fabs(MyMath::boxMinusAngleRad
                  ( MyMath::boxMinusAngleRad( old_line_orientation,
                                              old_up_neigh_ori),
                    MyMath::boxMinusAngleRad( new_line_orientation,
                                              new_up_neigh_ori ))
                  ) < m_nearLinesOrientationDiffThreshold){
          result.confidence_score = result.confidence_score *
            m_nearLinesBonusScoreMultiplier;
        }
      }
    }

    return result;
  }

  double DataAssociator::getLineLength
  (const line &t_line,
   const std::vector<scanPoint> &t_sPoints ){

    scanPoint sP1 = t_sPoints.at( t_line.first_index);
    scanPoint sP2 = t_sPoints.at( t_line.second_index);

    return sqrt( pow( (sP1.coords(0) - sP2.coords(0)),2) +
                 pow( (sP1.coords(1) - sP2.coords(1)),2) );
  }

  double DataAssociator::getLineOrientation
  (const line &t_line,
   const std::vector<scanPoint> &t_sPoints ){

    scanPoint sP1 = t_sPoints.at( t_line.first_index);
    scanPoint sP2 = t_sPoints.at( t_line.second_index);
    return atan2( sP2.coords(1) - sP1.coords(1),
                  sP2.coords(0) - sP1.coords(0) );
  }


  std::vector<int> DataAssociator::getPossibleCandidateIndexes
  (const int t_newLine_index ){

    std::vector< int> indexes;
    indexes.reserve( m_maxCandidates);
    int first_index;
    int last_index;
    if( (t_newLine_index- m_maxCandidates/2) >= 0){
      first_index = t_newLine_index - m_maxCandidates/2;
    }else{
      first_index = 0;
    }
    if( (t_newLine_index +  m_maxCandidates/2) < m_oldLines.size())
      last_index = t_newLine_index +  m_maxCandidates/2 ;
    else{
      last_index = m_oldLines.size() -1;
    }

    int i = first_index;
    while( i <= last_index){
      indexes.push_back(i);
      ++i;
    }
    return indexes;
  }

}

