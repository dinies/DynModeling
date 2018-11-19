// Created by Dinies on 26/06/2018.

#include "DatasetManager.hpp"
namespace dyn_modeling {

  DatasetManager::DatasetManager(const std::string &t_dataSetPath):
    m_dataSetPath( t_dataSetPath)
  {
    m_staticParams = DatasetManager::parseStaticParameters(t_dataSetPath);
    m_dataSet = DatasetManager::parseDataSet( t_dataSetPath, m_staticParams);
  };



  datasetParams DatasetManager::parseStaticParameters
  (const std::string &t_dataSetPath){

    std::ifstream fileStream( t_dataSetPath);
    std::string firstLine;

    if (fileStream.is_open()) {
      std::getline(fileStream,firstLine);

      fileStream.close();
      std::vector<std::string> params;

      char * cstr = new char [firstLine.length()+1];
      std::strcpy (cstr, firstLine.c_str());

      char * pch;
      int i = 0;
      pch = std::strtok( cstr, " ");
      while ( pch != NULL && i<=20) {
        params.push_back( pch );
        pch = std::strtok( NULL, " ");
        ++i;
      }

      datasetParams pS;
      pS.tag = params.at(0);
      pS.topic = params.at(1);
      pS.frame_id = params.at(2);
      pS.min_range = std::stod( params.at(13));
      pS.max_range = std::stod( params.at(14));
      pS.min_angle = std::stod( params.at(15));
      pS.max_angle = std::stod( params.at(16));
      pS.angle_increment = std::stod( params.at(17));
      pS.time_increment = std::stod( params.at(18));
      pS.scan_time = std::stod( params.at(19));
      pS.ranges_size = std::stod( params.at(20));
      pS.intensities = 0;
      return pS;
    }
    else{
      datasetParams pS;
      pS.tag = "something goes wrong";
      return pS;
    }
  }
  void DatasetManager::collectDataFromString( const std::string &t_dataString,
                                              dataNode &t_returning_struct,
                                              int t_rangesNum){
    char * cstr = new char [t_dataString.length()+1];
    std::strcpy (cstr, t_dataString.c_str());

    char * pch;
    int i = 0;
    pch = std::strtok( cstr, " ");
    while ( pch != NULL) {
      switch(i){
      case 3:{
        t_returning_struct.sequence_number = std::stoi( pch);
        break;
      }
      case 4:{
        t_returning_struct.timing_count= std::stod( pch);
        break;
      }
      default:{
        if(i>= 21 && i<= 21+ t_rangesNum -1){
          t_returning_struct.ranges.push_back(std::stod(pch));
          break;
        }
      }
      }
      pch = std::strtok( NULL, " ");
      ++i;
    }
  };


  dataSet DatasetManager::parseDataSet( const std::string &t_dataSetPath,
                                        const datasetParams &t_dSetParams){
    dataSet dS;

    std::ifstream fileStream( t_dataSetPath);
    std::string current_line;
    if (!fileStream.is_open()) {
      return dS;
    }
    const int ranges_num = static_cast<int>(t_dSetParams.ranges_size);
    while ( getline (fileStream,current_line) ){

      dataNode dataOfNode;
      dataOfNode.ranges.reserve(ranges_num);
      DatasetManager::collectDataFromString(current_line,
                                            dataOfNode,
                                            ranges_num);
      dS.push_back(dataOfNode);
    }
    fileStream.close();
    return dS;
  };

  std::vector<double> DatasetManager::getSpanningAngles(){

    int num_span = static_cast<int>(m_staticParams.ranges_size);
    std::vector<double> span_angles;
    span_angles.reserve(num_span);
    double angle = m_staticParams.min_angle;
    double delta_span = m_staticParams.angle_increment;
    for (int i = 0; i < num_span; ++i) {
      span_angles.push_back( angle );
      angle += delta_span;
    }
    return span_angles;
  };


  std::vector<double> DatasetManager::getDataNodeRanges
  (const int t_index_datanode){

    dataNode dN = m_dataSet.at(t_index_datanode);
    return dN.ranges;
  };

}
