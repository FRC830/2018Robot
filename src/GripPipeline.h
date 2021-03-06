/*
 * GripPipeline.h
 *
 *  Created on: Jan 9, 2018
 *      Author: RatPack
 */

#ifndef SRC_GRIPPIPELINE_H_
#define SRC_GRIPPIPELINE_H_

#pragma once
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>

using namespace std;
namespace grip {

/**
* GripPipeline class.
*
* An OpenCV pipeline generated by GRIP.
*/
class GripPipeline {
	private:
		cv::Mat hslThresholdOutput;
		std::vector<std::vector<cv::Point> > findContoursOutput;
		void hslThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);
		void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);

	public:
		GripPipeline();
		void Process(cv::Mat& source0);
		void EndProcess(float mid, string vision_error);
		cv::Mat* GetHslThresholdOutput();
		std::vector<std::vector<cv::Point> >* GetFindContoursOutput();
		static bool compareContourArea(vector<cv::Point> &contour_1,vector<cv::Point> &contour_2 );

		float getHeight(cv::Rect &rect);
		float getWidth(cv::Rect &rect);
		float boundRectArea (cv::Rect &rect);
		float getRectRatio(cv::Rect &rect);
};


} // end namespace grip



#endif /* SRC_GRIPPIPELINE_H_ */
