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

	double hue_min = SmartDashboard::GetNumber("hue min", 60);
	double hue_max = SmartDashboard::GetNumber("hue max", 108);
	double saturation_min = SmartDashboard::GetNumber("sat min", 131);
	double saturation_max = SmartDashboard::GetNumber("sat max", 255);
	double luminance_min = SmartDashboard::GetNumber("lum min", 140);
	double luminance_max = SmartDashboard::GetNumber("lum max", 220);
	cv::Mat hslThresholdInput = source0;
	double hslThresholdHue[] = {hue_min, hue_max};
	double hslThresholdSaturation[] = {saturation_min, saturation_max};
	double hslThresholdLuminance[] = {luminance_min, luminance_max};

//	double hslThresholdHue[] = {60, 108};
//	double hslThresholdSaturation[] = {131, 255};
//	double hslThresholdLuminance[] = {140, 220};
	hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, this->hslThresholdOutput);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = hslThresholdOutput;

	bool findContoursExternalOnly = false;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);

//	source0 = hslThresholdOutput;
//	return;
	if (findContoursOutput.size() == 0) {
		SmartDashboard::PutString("vision error", "no contours");
		SmartDashboard::PutNumber("mid point x", 160);
		return;
	}

	cv::Scalar color_1 = {0,255,0};
	cv::Scalar color_2 {255,0,0};


	sort(findContoursOutput.begin(), findContoursOutput.end(), compareContourArea);

	vector <cv::Rect> boundRect;


	for (int i = 0; i < (int)findContoursOutput.size(); i++) {
		cv::Rect temp_boundRect = cv::boundingRect(cv::Mat(findContoursOutput[i]));
		float ratio = getRectRatio(temp_boundRect);
		float area = temp_boundRect.area();
		//might cause an issue
		if ((ratio < 10 && ratio > 2) && area > 100) {
			boundRect.push_back(temp_boundRect);
		}
	}
//	for (int i = 0; i < (int) boundRect.size(); i++) {
//		cv::rectangle(source0, boundRect[i].tl(), boundRect[i].br(), color_1,1);
//	}
//	return;
	if (boundRect.size() < 2) {
		EndProcess(160,"rectangles with wrong ratio" );
		return;
	}


	SmartDashboard::PutNumber("ratio" , getRectRatio(boundRect[0]));
	SmartDashboard::PutNumber("ratio 2",  getRectRatio(boundRect[1]));

	vector <float> rectRatio;
	for (int i = 0; i < (int)(boundRect.size()-1); i++) {
		float area_ratio = boundRectArea(boundRect[i])/boundRectArea(boundRect[i+1]);
		if (area_ratio < 1 && area_ratio != 0) {
			area_ratio = 1/area_ratio;
		}
		rectRatio.push_back(area_ratio);
	}


	int index = distance(rectRatio.begin(), min_element(rectRatio.begin(), rectRatio.end()));

	//cout << "smallest: " << *min_element(rectRatio.begin(), rectRatio.end()) <<endl;
	//cout << "index :" << index <<endl;

	vector <cv::Rect> finalboundRect;

	finalboundRect.push_back(boundRect[index]);
	finalboundRect.push_back(boundRect[index + 1]);

	cv::Rect rect_1 = finalboundRect[0];
	cv::Rect rect_2 = finalboundRect[1];
	if (rect_1.tl().x > rect_2.tl().x )
		swap(rect_1,rect_2);

	double rect_length = rect_2.br().x - rect_1.tl().x;
	double x = getHeight(rect_1)/rect_length;
	double range = 1.5;
	if (x >= 2*range || x <= 2/range) {
		SmartDashboard::PutNumber("mid point x", 160);
		return;
	}
	for (int i = 0; i < 2; i++) {
		cv::rectangle(source0, finalboundRect[i].tl(), finalboundRect[i].br(), color_1, 5);

	}

	//cv::line(source0, rect_1.tl(), rect_2.br(), color_2,5);
	cv::Scalar color_3 {100,100,255};
	cv::Point mid_point ((rect_1.tl() + rect_2.br()) / 2);

	cv::line(source0, mid_point, mid_point, color_3, 5);

	cv::line(source0, cv::Point(160,0), cv::Point(160,240), {0,255,255}, 3 );

	EndProcess(mid_point.x, "");

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
	 * @para lum The min and max luminance.
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

	void GripPipeline::EndProcess(float mid, string vision_error) {
		SmartDashboard::PutString("vision error", vision_error);
		SmartDashboard::PutNumber("mid point x", mid);
	}



} // end grip namespace




