// Created by dinies on 20/06/2018.

#include "Drawer.hpp"
namespace dyn_modeling {

  Drawer::Drawer(const double scale):m_scale(scale){};


  void Drawer::drawHollowPoligon(const RGBImage &t_drawing, const std::vector< cv::Point2d> &t_consecPoints, const cv::Scalar &t_color){

    int poligon_edgesNum = t_consecPoints.size();

    for (int i = 0; i < poligon_edgesNum ; ++i) {
      if ( i == 0 ){
        drawLine( t_drawing, t_consecPoints.at(poligon_edgesNum -1), t_consecPoints.at(0),t_color);
      }
      else{
        drawLine( t_drawing, t_consecPoints.at(i-1), t_consecPoints.at(i),t_color);
      }
    }
  };

  void Drawer::drawPatch(RGBImage &t_drawing, const cv::Point2d &t_point,const cv::Scalar &t_color){

    cv::Point2d P_imgFrame = convertFrowWorldToImg(t_drawing, t_point);
    if ( isInsideBoundaries(t_drawing, P_imgFrame )){
    t_drawing.at<cv::Vec3b>(P_imgFrame.y,P_imgFrame.x) = cv::Vec3b(t_color[0],t_color[1],t_color[2]);
    }
 };

  void Drawer::drawLine(const RGBImage &t_drawing, const cv::Point2d &t_firstP, const cv::Point2d &t_secondP, const cv::Scalar &t_color){
    cv::Point2d firstP_imgFrame = convertFrowWorldToImg(t_drawing, t_firstP);
    cv::Point2d secondP_imgFrame = convertFrowWorldToImg(t_drawing, t_secondP);
    if ( isInsideBoundaries(t_drawing, firstP_imgFrame) && isInsideBoundaries(t_drawing, secondP_imgFrame)){
      cv::line(t_drawing, firstP_imgFrame, secondP_imgFrame,t_color,2);
    }
  };


  cv::Point2d Drawer::convertFrowWorldToImg(const RGBImage &t_drawing, const cv::Point2d &t_point){
    int heigth = t_drawing.rows;
    int width = t_drawing.cols;
    int middle_x = std::round(width/2);
    int middle_y = std::round(heigth/2);
    cv::Point2d p_imgFrame( middle_x + t_point.x * m_scale, middle_y - t_point.y * m_scale);
    return p_imgFrame;
  };

  bool Drawer::isInsideBoundaries(const RGBImage &t_drawing, const cv::Point2d &t_point){
    int heigth = t_drawing.rows;
    int width = t_drawing.cols;
    return t_point.x >= 0 && t_point.x <= width && t_point.y >= 0 && t_point.y <= heigth;
  };

}
