// Created by Edoardo Ghini on 26/06/2018.

#include "DatasetManager.hpp"
namespace dyn_modeling {

  DatasetManager::datasetParams DatasetManager::parseStaticParameters( std::string t_dataSetPath){

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

  DatasetManager::DatasetManager( std::string t_dataSetPath){
    // datasetParams data = {
    //   "LASER_MESSAGE",
    //   "/scan",
    //   "/laser_frame",
    //   0.02,
    //   30,
    //   -1.5708,
    //   1.5708,
    //   0.00436332,
    //   0,
    //   0,
    //   721,
    //   0
    // };
    m_staticParams = DatasetManager::parseStaticParameters(t_dataSetPath);
  };
}
