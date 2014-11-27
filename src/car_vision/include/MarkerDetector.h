#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "Marker.h"

using namespace cv;
using std::pair;

class MarkerDetector
{
public:
	MarkerDetector(void);
	~MarkerDetector(void);
//	void processingFrame(const Mat& _frame, Mat frame);   //function process the current frame
	bool processingFrame(const Mat& _frame, Mat framem, Point3f &MarkerPosition, bool DebugMode);
	void getTransformations(void);			   //function process transformation
private:
	bool findMarkers(const Mat& _frame, vector<Marker>& _detectedMarkers);												// function find the marker, return a boolean result, which represent if the marker is found
	void findMarkerContours(const Mat& _imgThreshold, vector<vector<Point> >& _contours, int _minContourPointsAllowed); // function find the contour
	void findMarkerCandidates(const vector<vector<Point> >& _contours, vector<Marker>& _detectedMarkers);               // function find the candidates of the marker
	void detectMarkers(const Mat& _imgGray, vector<Marker>& _detectedMarkers);                                          // function detect the marker
	void estimatePosition(vector<Marker>& _detectedMarkers);															// function estimate the position of the marker
private:
	float minContourLengthAllowed;	// the minimum length of the marker allowed to be detected
	Size markerSize;
	vector<Point2f> vec_markerCorners2d;	//the coordinate of the marker in 2D
	vector<Point3f> vec_markerCorners3d;	//the coordinate of the marker in 3D
	Mat mat_camMat;
	Mat mat_distCoeff;
	bool MarkerFound;
public:
	Mat mat_imgGray;
	Mat mat_imgThreshold;
	vector<vector<Point> > vec_contours;
	vector<Marker> vec_markers;
	vector<Marker> all_vec_markers;
	vector<Marker> Temp_markers;
};
