// Created by Dinies on 04/09/2018.

#pragma once
#include <unistd.h>
#include <vector>

#include "../../include/structs.hpp"
#include "../utils/MyMath.hpp"
#include "LineMatcher.hpp"



namespace dyn_modeling {

  typedef struct dataAssociation_tag{
    int old_line_index;
    int new_line_index;
    double confidence_score;
  } dataAssociation;


  class DataAssociator{
  private:
    int m_maxCandidates;
    double m_lengthDifferenceThreshold;
    double m_orientationDiffThreshold;
    std::vector<line> m_oldLines;
    std::vector<line> m_newLines;
    std::vector<scanPoint> m_oldSPoints;
    std::vector<scanPoint> m_newSPoints;

  public:
    DataAssociator( int t_maxCandidates, double t_lengthDifferenceThreshold, double t_orientationDiffThreshold, const std::vector<line> &t_old_lines,const std::vector<scanPoint> &t_old_sPoints,const std::vector<line> &t_new_lines,const std::vector<scanPoint> &t_new_sPoints);


    DataAssociator();

    std::vector<dataAssociation> associateLines();

    std::vector<dataAssociation> findCandidates(const int t_choosen_newLine_index);

    void insertOrderedDataAssociation(const dataAssociation &t_lineComparison, std::vector<dataAssociation> &t_candidates);

    dataAssociation compareLines( const int t_oldLine_index,  const int t_newLine_index  );

    double getLineLength( const line &t_line, const std::vector<scanPoint> &t_sPoints );

    double getLineOrientation( const line &t_line, const std::vector<scanPoint> &t_sPoints );

    std::vector< int> getPossibleCandidateIndexes(const int t_newLine_index );

    std::vector<dataAssociation> chooseBestAssociations( std::vector< std::vector< dataAssociation>> &t_matrix);

    void removeTakenAssociations( const dataAssociation &t_takenAssociation, std::vector< std::vector< dataAssociation>> &t_matrix);

    dataAssociation chooseMaxScoreAssociation( const std::vector< std::vector< dataAssociation>> &t_matrix);

    bool associationsAllTaken( const std::vector< std::vector< dataAssociation>> &t_matrix);

    inline double getLengthDiffThreshold() { return m_lengthDifferenceThreshold;};

    inline void setLengthDiffThreshold( const double t_lengthDifferenceThreshold) { m_lengthDifferenceThreshold = t_lengthDifferenceThreshold; };


  };
}
