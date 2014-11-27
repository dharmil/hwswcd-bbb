#ifndef _MARKER_DETECTOR_H_
#define _MARKER_DETECTOR_H_
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "pattern.h"

using namespace std;
using namespace cv;


class markerDetector : Pattern
{
public:

    //constructor
	markerDetector(const double param1, const double param2, const int param3, const  double param4, const int param5, const int thresh_mode, const double markSize, const char*, int patCount);

	//distractor
	~markerDetector(){};

enum THRES_MODE {
		FIXED_THRESHOLD,
		ADAPTIVE_THRESHOLD, 
	}; 
//detect patterns in the input frame
bool detect(Mat &frame, const Mat& cameraMatrix, const Mat& distortions, vector<Pattern>& foundPatterns, Point3f& position);

private:

	int mode, normSize, block_size;
	double confThreshold, thresh1, thresh2, markerSize;
  vector<Mat> library; //possible to save different marker patterns
  int patternCount; //number of patterns
	struct patInfo{
		int index;
		int ori;
		double maxCor;
	};
	Mat binImage, grayImage, normROI, patMask, patMaskInt;
	Point2f norm2DPts[4];


	void convertAndBinarize(const Mat& src, Mat& dst1, Mat& dst2, int thresh_mode = 1);
	void normalizePattern(const Mat& src, const Point2f roiPoints[], Rect& rec, Mat& dst);
	int identifyPattern(const Mat& src, std::vector<cv::Mat>& loadedPatterns, patInfo& out);

};

#endif // _MARKER_DETECTOR_H_
