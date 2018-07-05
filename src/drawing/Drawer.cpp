// Created by dinies on 20/06/2018.

#include "Drawer.hpp"
namespace dyn_modeling {

  void Drawer::drawPoligon(const RGBImage &t_drawing, const std::vector< cv::Point2d> &t_consecPoints, const cv::Scalar &t_color){

    int poligon_edgesNum = t_consecPoints.size();

    for (int i = 0; i < poligon_edgesNum ; ++i) {
      if ( i == 0 ){
        Drawer::drawLine( t_drawing, t_consecPoints.at(poligon_edgesNum -1), t_consecPoints.at(0),t_color);
      }
      else{
        Drawer::drawLine( t_drawing, t_consecPoints.at(i-1), t_consecPoints.at(i),t_color);
      }
    }
  };

  void Drawer::drawLine(const RGBImage &t_drawing, const cv::Point2d &t_firstP, const cv::Point2d &t_secondP, const cv::Scalar &t_color){

    cv::Point2d firstP_imgFrame = Drawer::convertFrowWorldToImg(t_drawing, t_firstP);
    cv::Point2d secondP_imgFrame = Drawer::convertFrowWorldToImg(t_drawing, t_secondP);
    cv::line(t_drawing, firstP_imgFrame, secondP_imgFrame,t_color);
  };



  cv::Point2d Drawer::convertFrowWorldToImg(const RGBImage &t_drawing, const cv::Point2d &t_point){
    int heigth = t_drawing.rows;
    int width = t_drawing.cols;
    int middle_x = std::round(width/2);
    int middle_y = - std::round(heigth/2);
    cv::Point2d p_imgFrame( middle_x + t_point.x, middle_y - t_point.y);
    return p_imgFrame;
  };


}
