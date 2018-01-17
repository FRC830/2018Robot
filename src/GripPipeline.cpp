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
/**
* Runs an iteration of the pipeline and updates outputs.
*/
void GripPipeline::Process(cv::Mat& source0){
	//Step HSL_Threshold0:
	//input
	cv::Mat hslThresholdInput = source0;
	double hslThresholdHue[] = {0, 180};
	double hslThresholdSaturation[] = {0, 255};
	double hslThresholdLuminance[] = {217.85071942446044, 255.0};
	hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, this->hslThresholdOutput);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = hslThresholdOutput;

	bool findContoursExternalOnly = false;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);

	if (findContoursOutput.size() == 0) {
		SmartDashboard::PutString("vision error", "no contours");
		return;
	}

	cv::Scalar color_1 = {0,255,0};
	cv::Scalar color_2 {255,0,0};


	sort(findContoursOutput.begin(), findContoursOutput.end(), compareContourArea);

	vector <cv::Rect> boundRect;


	for (int i = 0; i < (int)findContoursOutput.size(); i++) {
		cv::Rect temp_boundRect = cv::boundingRect(cv::Mat(findContoursOutput[i]));
		float ratio = getRectRatio(temp_boundRect);
		//float area = boundRectArea(temp_boundRect);
		//might cause an issue
		if ((ratio < 9 && ratio > 4)) {
			boundRect.push_back(temp_boundRect);
		}
	}
	if (boundRect.size() < 2) {
		SmartDashboard::PutString("vision error", "rectangles with wrong ratio");
		return;
	}


	SmartDashboard::PutNumber("ratio" , getRectRatio(boundRect[0]));
	SmartDashboard::PutNumber("ratio 2",  getRectRatio(boundRect[1]));

	cout << "rec size " << boundRect.size() <<endl;

	vector <float> rectRatio;
	for (int i = 0; i < (int)(boundRect.size()-1); i++) {
		float area_ratio = boundRectArea(boundRect[i])/boundRectArea(boundRect[i+1]);
		if (area_ratio < 1 && area_ratio != 0) {
			area_ratio = 1/area_ratio;
		}
		rectRatio.push_back(area_ratio);
		cout <<area_ratio <<endl;
	}


	int index = distance(rectRatio.begin(), min_element(rectRatio.begin(), rectRatio.end()));

	cout << "smallest: " << *min_element(rectRatio.begin(), rectRatio.end()) <<endl;
	cout << "index :" << index <<endl;

	vector <cv::Rect> finalboundRect;

	finalboundRect.push_back(boundRect[index]);
	finalboundRect.push_back(boundRect[index + 1]);


	for (int i = 0; i < 2; i++) {
		cv::rectangle(source0, finalboundRect[i].tl(), finalboundRect[i].br(), color_1, 5);

	}

	SmartDashboard::PutString("vision error", "");

}

/**
 * This method is a generated getter for the output of a HSL_Threshold.
 * @return Mat output from HSL_Threshold.
 */
cv::Mat* GripPipeline::GetHslThresholdOutput(){
	return &(this->hslThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* GripPipeline::GetFindContoursOutput(){
	return &(this->findContoursOutput);
}
	/**
	 * Segment an image based on hue, saturation, and luminance ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param lum The min and max luminance.
	 * @param output The image in which to store the output.
	 */
	//void hslThreshold(Mat *input, double hue[], double sat[], double lum[], Mat *out) {
	void GripPipeline::hslThreshold(cv::Mat &input, double hue[], double sat[], double lum[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HLS);
		cv::inRange(out, cv::Scalar(hue[0], lum[0], sat[0]), cv::Scalar(hue[1], lum[1], sat[1]), out);
	}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
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

	float GripPipeline::getHeight(cv::Rect &rect) {
		return  fabs(rect.tl().y - rect.br().y);
	}
	float GripPipeline::getWidth(cv::Rect &rect) {
		return fabs(rect.br().x - rect.tl().x);
	}
	float GripPipeline::boundRectArea(cv::Rect &rect) {
		float height = getHeight(rect);
		float width = getWidth(rect);
		return height * width;
	}
	float GripPipeline::getRectRatio(cv::Rect &rect) {
		return getHeight(rect)/getWidth(rect);
	}



} // end grip namespace




