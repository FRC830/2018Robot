/*
 * GripPipeline.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: RatPack
 */

#include "GripPipeline.h"
#include "Lib830.h"

using namespace std;

namespace grip {

GripPipeline::GripPipeline() {
}

void GripPipeline::Process(cv::Mat& source0){
	//Step HSL_Threshold0:
	//input
	cv::Mat hslThresholdInput = source0;
	double hslThresholdHue[] = {0, 180};
	double hslThresholdSaturation[] = {0, 255};
	double hslThresholdLuminance[] = {218, 255.0};
	hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, this->hslThresholdOutput);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = hslThresholdOutput;

	bool findContoursExternalOnly = false;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);

	if (findContoursOutput.size() == 0) {
		return;
	}

	cv::Scalar color_1 = {0,255,0};
	cv::Scalar color_2 {255,0,0};
	cv::Scalar color_3 {0,0,0};


	vector<rectDimensions> rectVect;
	//drawContours(source0, findContoursOutput, -1, color_1, 5);
	for (int i = 0; i < (int)findContoursOutput.size(); i++){
		cv::Rect temp = cv::boundingRect(findContoursOutput[i]);
		rectDimensions rectDim(temp);
		if (rectDim.ratio < 9 && rectDim.ratio > 6) {
			rectVect.push_back(rectDim);
			cout << "ratio: " << rectDim.ratio << endl;

		}
	}
	if (rectVect.size() == 0) {
		return;
	}


	sort(rectVect.rbegin(), rectVect.rend());
	cv::rectangle(source0, rectVect[0].rect.tl(), rectVect[0].rect.br(), color_2);
	cv::Point midpoint = (rectVect[0].rect.tl()+rectVect[0].rect.br())/2;
	cv:line(source0, midpoint, midpoint, color_3, 5);
}

/**
 * This method is a generated getter for the output of a HSL_Threshold.
 * @return Mat output from HSL_Threshold.
 */

GripPipeline::rectDimensions::rectDimensions(cv::Rect& rect) : rect(rect) {
	float width = rect.width;
	float height = rect.height;
	ratio = width/height;
}



cv::Mat* GripPipeline::GetHslThresholdOutput(){
	return &(this->hslThresholdOutput);
}

std::vector<std::vector<cv::Point> >* GripPipeline::GetFindContoursOutput(){
	return &(this->findContoursOutput);
}

	void GripPipeline::hslThreshold(cv::Mat &input, double hue[], double sat[], double lum[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HLS);
		cv::inRange(out, cv::Scalar(hue[0], lum[0], sat[0]), cv::Scalar(hue[1], lum[1], sat[1]), out);
	}


	void GripPipeline::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}

	bool GripPipeline::compareContourArea(vector<cv::Point> &contour_1,vector<cv::Point> &contour_2) {
		double i = fabs( cv::contourArea(cv::Mat(contour_1)));
		double j = fabs( cv::contourArea(cv::Mat(contour_2)));
		return i > j;
	}



} // end grip namespace




