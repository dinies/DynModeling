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
    t_drawing.at<cv::Vec3b>(P_imgFrame.y,P_imgFrame.x) = cv::Vec3b(t_color[0],t_color[1],t_color[2]);
    // t_drawing.at<cv::Vec3b>(P_imgFrame.y+1,P_imgFrame.x) = cv::Vec3b(t_color[0],t_color[1],t_color[2]);
    // t_drawing.at<cv::Vec3b>(P_imgFrame.y-1,P_imgFrame.x) = cv::Vec3b(t_color[0],t_color[1],t_color[2]);
    // t_drawing.at<cv::Vec3b>(P_imgFrame.y,P_imgFrame.x+1) = cv::Vec3b(t_color[0],t_color[1],t_color[2]);
    // t_drawing.at<cv::Vec3b>(P_imgFrame.y,P_imgFrame.x-1) = cv::Vec3b(t_color[0],t_color[1],t_color[2]);
    // // need to implement a guard that checks that all the coords are inside the mat ranges: if it happens then segFault
 };

  void Drawer::drawLine(const RGBImage &t_drawing, const cv::Point2d &t_firstP, const cv::Point2d &t_secondP, const cv::Scalar &t_color){
    cv::Point2d firstP_imgFrame = convertFrowWorldToImg(t_drawing, t_firstP);
    cv::Point2d secondP_imgFrame = convertFrowWorldToImg(t_drawing, t_secondP);
    cv::line(t_drawing, firstP_imgFrame, secondP_imgFrame,t_color);
  };


  cv::Point2d Drawer::convertFrowWorldToImg(const RGBImage &t_drawing, const cv::Point2d &t_point){
    int heigth = t_drawing.rows;
    int width = t_drawing.cols;
    int middle_x = std::round(width/2);
    int middle_y = std::round(heigth/2);
    cv::Point2d p_imgFrame( middle_x + t_point.x * m_scale, middle_y - t_point.y * m_scale);
    return p_imgFrame;
  };


}
