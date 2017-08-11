#include <algorithm>
#include <iostream>
#include <cmath>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgcodecs.hpp>
#include "features.h"
using namespace cv;

const int circleRadius = 50;


void processImage(cv::Mat& src, CirclesMessage& msg) {

  int thresh = 100;
  int max_thresh = 255;
  RNG rng(12345);
  Point2f camCenter(src.cols / 2, src.rows / 2);
  
  Mat threshold_output;
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;

  Mat imgHSV;
  cvtColor(src, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
  Mat red_hue_range;
  Mat green_hue_range;
  Mat blue_hue_range;
  
  // Refuse only pixels in target's colors range
  cv::inRange(imgHSV, cv::Scalar(128, 81, 32), cv::Scalar(175, 255, 255), red_hue_range);
  cv::inRange(imgHSV, cv::Scalar(102, 86, 49), cv::Scalar(143, 255, 255), blue_hue_range);
  cv::inRange(imgHSV, cv::Scalar(51, 89, 60), cv::Scalar(103, 255, 255), green_hue_range);
  
  // Summ all ranges
  cv::Mat summ;
  cv::addWeighted(red_hue_range, 1.0, green_hue_range, 1.0, 0.0, summ);
  cv::addWeighted(blue_hue_range, 1.0, summ, 1.0, 0.0, summ);
  
  // smooth it, otherwise a lot of false circles may be detected
  //cv::GaussianBlur( summ, summ, cv::Size(9, 9), 2, 2 );

  //morphological opening (remove small objects from the foreground)
  erode(summ, summ, getStructuringElement(MORPH_ELLIPSE, Size(15, 15)) );
  dilate( summ, summ, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 
  //morphological closing (fill small holes in the foreground)
  dilate( summ, summ, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 
  erode(summ, summ, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );

  /// Detect edges using Threshold
  threshold( summ, threshold_output, thresh, 255, THRESH_BINARY );
  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Find the rotated rectangles and ellipses for each contour
  std::vector<RotatedRect> minRect( contours.size() );
  std::vector<RotatedRect> minEllipse( contours.size() );

  for( int i = 0; i < contours.size(); i++ )
     { minRect[i] = minAreaRect( Mat(contours[i]) );
       if( contours[i].size() > 5 )
         { minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
     }

// Preparing the message
  msg.circles = minEllipse;
  msg.inTheBox = std::vector<bool>(minEllipse.size(), false);

  /// Draw contours + rotated rects + ellipses + center and coords
  Mat drawing = src;



      // Estimate central lines and box
       Point2f leftCenter(0, drawing.rows / 2), rightCenter(drawing.cols, drawing.rows / 2);
       Point2f topCenter(drawing.cols / 2, 0), bottomCenter(drawing.cols / 2, drawing.rows);
       float boxTop = (drawing.rows * 0.4) / 2,
	         boxBottom = drawing.rows - boxTop,
	         boxLeft = (drawing.cols * 0.5) / 2,
	         boxRight = drawing.cols - boxLeft;

       msg.box.left = boxLeft;
       msg.box.right = boxRight;
       msg.box.top = boxTop;
       msg.box.bottom = boxBottom;

       Point2f boxTopLeft(boxLeft, boxTop), boxTopRight(boxRight, boxTop);
       Point2f boxBottomLeft(boxLeft, boxBottom), boxBottomRight(boxRight, boxBottom);
    
      //Draw central lines and good coordinates box
       line( drawing, leftCenter, rightCenter, Scalar(255, 0, 0), 3, 8 );
       line( drawing, bottomCenter, topCenter, Scalar(255, 0, 0), 3, 8 );
       rectangle( drawing, boxBottomLeft, boxTopRight, Scalar(100, 0, 255, 0.1), 2, CV_AA);

  for( int i = 0; i< contours.size(); i++ )
     {
	// find centers of ellipses
       Point2f center = minEllipse[i].center;
       Size2f size = minEllipse[i].size;

       //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       Scalar color = Scalar( 0, 255, 0 );
       // contour
       drawContours( drawing, contours, i, color, 1, 8, std::vector<Vec4i>(), 0, Point() );
       // ellipse
       ellipse( drawing, minEllipse[i], color, 2, 8 );
       // rotated rectangle
       Point2f rect_points[4]; minRect[i].points( rect_points );
	// center
       cv::circle(drawing, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );

	// Print coordinates
         cv::Scalar color2(1, 1, 1);
         Point2f y_p(center.x, center.y + 20);
         std::string x_coord = std::to_string((int)center.x), 
                     y_coord = std::to_string((int)center.y);
         cv::putText(drawing, "x: " + x_coord, center, cv::FONT_HERSHEY_DUPLEX, 0.8, color2);
         cv::putText(drawing, "y: " + y_coord, y_p, cv::FONT_HERSHEY_DUPLEX, 0.8, color2);
      

        // draw a line to the circle if it is out the box
       if (center.x > boxRight ||
           center.x < boxLeft ||
           center.y < boxTop ||
           center.y > boxBottom)
       {
           arrowedLine(drawing, camCenter, center, Scalar(255, 0, 0), 2, CV_AA);
           msg.inTheBox[i] = true;
       }

       for( int j = 0; j < 4; j++ )
          line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
     }
}

