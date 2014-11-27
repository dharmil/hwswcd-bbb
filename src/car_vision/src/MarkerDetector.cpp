#include "MarkerDetector.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <bitset>
#include <algorithm>

MarkerDetector::MarkerDetector(void)
{
	//set the the minimum length of the marker allowed to be detected
	minContourLengthAllowed = 100.0f;
	//initial the cammat in matrix format
	mat_camMat = (Mat_<float>(3,3) << 0,0,0,
									0,0,0,
									0,0,0);
	//initial the distance coefficient
	mat_distCoeff = (Mat_<float>(4,1) << 0,0,0,0);
	//initial the 2d rectangle size as 100*100
	markerSize = Size(100, 100);
	vec_markerCorners2d.push_back(Point2f(0, 0));
	vec_markerCorners2d.push_back(Point2f(markerSize.width-1, 0));
	vec_markerCorners2d.push_back(Point2f(markerSize.width-1, markerSize.height-1));
	vec_markerCorners2d.push_back(Point2f(0, markerSize.height-1));

	vec_markerCorners3d.push_back(cv::Point3f(-0.5f,-0.5f,0));
	vec_markerCorners3d.push_back(cv::Point3f(+0.5f,-0.5f,0));
	vec_markerCorners3d.push_back(cv::Point3f(+0.5f,+0.5f,0));
	vec_markerCorners3d.push_back(cv::Point3f(-0.5f,+0.5f,0));
	MarkerFound=false;
}

MarkerDetector::~MarkerDetector(void)
{
}

bool MarkerDetector::processingFrame(const Mat& _frame, Mat frame, Point3f &MarkerPosition, bool DebugMode)
{
        int scaleMarker = 1;
        if( _frame.cols == 320 ){
          scaleMarker = 2;
        }else if( frame.cols == 160 ){
          scaleMarker = 4;
        }else if( frame.cols == 640 ){
          scaleMarker = 1;
        }else{
          std::cout << "[Warning] MarkerDetector -> detect(): wrong scale level to calculate depth!" << std::endl;
        }
	vec_markers.clear();
	MarkerFound=findMarkers(_frame, vec_markers);
	if (MarkerFound==true)
	{
	//give the position
	float SumX = 0;
	float SumY = 0;
	for (int i= 0; i<4; i++)
		{
			SumX += vec_markers[0].vec_points[i].x;
			SumY += vec_markers[0].vec_points[i].y;
		}
	float CenX=SumX /vec_markers[0].vec_points.size();
	float CenY=SumY /vec_markers[0].vec_points.size();
	float RectangleHeight=sqrt(pow((vec_markers[0].vec_points[0].y-vec_markers[0].vec_points[3].y),2)+pow((vec_markers[0].vec_points[0].x-vec_markers[0].vec_points[3].x),2));
//	std::cout << "The number of points is"<<vec_markers[0].vec_points.size()<<std::endl;
//		std::cout << "The Sum X is"<<SumX<<";The Sum Y is"<<SumY<<std::endl;
//		std::cout << "The center X is"<<CenX<<";The center Y is"<<CenY<<std::endl;
//	std::cout << "The length of the border is"<<RectangleHeight<<std::endl;

		//caculate the distance in z axis
	const float fStandardDistance = 110.0;
	const float fStandardHeight = 55.0;
	//const float fZ = fStandardDistance - fStandardHeight * RectangleHeight/fStandardDistance;
        const float fZ = (71.*70.)/(RectangleHeight*scaleMarker);
//	   std::cout << "The distance in Z axis is"<<fZ<<std::endl;
//	    output Marker Position
	MarkerPosition.x=CenX;
	MarkerPosition.y=CenY;
	MarkerPosition.z=fZ;
//	std::cout <<  " X = " << MarkerPosition.x << " Y = " << MarkerPosition.y << " Z = " << MarkerPosition.z << std::endl;
	}
	if (DebugMode==true)
	{
			// show threshold
			//imshow("thresholdImg", this->mat_imgThreshold);

			// show contours
			vector<Vec4i> hierarchy;
			Mat contourImg = Mat::zeros(frame.size(), CV_8UC3);
			for(int i=0; i<this->vec_contours.size(); i++)
			{
				drawContours(contourImg, this->vec_contours, i, Scalar(255,255,255), 2, 8, hierarchy, 0, Point());
			}
			//imshow("contours", contourImg);

			// show marker
			for(int i=0; i<this->vec_markers.size(); i++)
			{
				int sizeNum = this->vec_markers[i].vec_points.size();
				for (int j=0; j<sizeNum; j++)
				{
					line(frame, this->vec_markers[i].vec_points[j], this->vec_markers[i].vec_points[(j+1)%sizeNum], Scalar(0,255,0), 2, 8);
				}
//				circle(frame, this->vec_markers[i].vec_points[0], 3, Scalar(0,255,255), 2, 8);
			}
			//imshow("Target Marker", frame);
			//waitKey (1);
			// show all marker
			for(int i=0; i<all_vec_markers.size(); i++)
			{
				int sizeNum = all_vec_markers[i].vec_points.size();
				for (int j=0; j<sizeNum; j++)
				{
					line(frame, all_vec_markers[i].vec_points[j], all_vec_markers[i].vec_points[(j+1)%sizeNum], Scalar(0,0,255), 5, 8);
				}
//				circle(frame, all_vec_markers[i].vec_points[0], 3, Scalar(0,255,255), 2, 8);
			}
			//imshow("all detected markers", frame);
			//waitKey (1);

			// print markers information
//			int numMarker = this->vec_markers.size();
////		 	std::cout << "new frame---------------" << std::endl;
//			if (numMarker >0)
//			{
//				std::cout << "detect marker number: " << numMarker << std::endl;
//				for (int i=0; i<numMarker; i++)
//				{
//					std::cout << "marker index: " << i << "   " << "marker ID: " << std::bitset<10>(this->vec_markers[i].m_id) << std::endl;
//				}
//			}
//			else
//			{
//				std::cout << "no marker!" << std::endl;
//			}
	}
	if (this->vec_markers.size() >0)
	{
		return true;
	}
	else
	{
		return false;
	}

}

void MarkerDetector::getTransformations(void)
{
}

bool MarkerDetector::findMarkers(const Mat& _frame, vector<Marker>& _detectedMarkers)
{
	// convert the image into grayscale image
	cvtColor(_frame, mat_imgGray, CV_BGR2GRAY);

	// convert the gray scale image into binary image using adaptive threshold
//	threshold(mat_imgGray, mat_imgThreshold, 128, 255, cv::THRESH_BINARY_INV);
 	adaptiveThreshold(mat_imgGray, mat_imgThreshold, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 7, 7);


	// find the markercontours
	findMarkerContours(mat_imgThreshold, vec_contours, mat_imgGray.cols/5);
	// find the candidates of the marker
	findMarkerCandidates(vec_contours, _detectedMarkers);
	all_vec_markers=_detectedMarkers;

	// detect the Marker ID
	detectMarkers(mat_imgGray, _detectedMarkers);
	if (_detectedMarkers.size()>0)
		{
		return true;
		}
	else
	{
		return false;
	}



}



// find contours from the markers
void MarkerDetector::findMarkerContours(const Mat& _imgThreshold, vector<vector<Point> >& _contours, int _minContourPointsAllowed)
{
	// create a copy of the binary image
	Mat imgTemp = _imgThreshold.clone();
	// create a vector to store all found contours
	vector<vector<Point> > allContours;
	// find the contours
	findContours(imgTemp, allContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	// delete the contour if the contours detected is too few
	_contours.clear();
	for (size_t i=0; i<allContours.size(); i++)
	{
		int contourSize = allContours[i].size();
		if (contourSize > _minContourPointsAllowed)
		{
			_contours.push_back(allContours[i]);
		}
	}
}

void MarkerDetector::findMarkerCandidates(const vector<vector<Point> >& _contours, vector<Marker>& _detectedMarkers)
{
	vector<Point> approxCurve;
	vector<Marker> markerPossible;
	
	for (size_t i=0; i<_contours.size(); i++)
	{
		// approximates contour or a curve using Douglas-Peucker algorithm
		approxPolyDP(_contours[i], approxCurve, double(_contours[i].size())*0.05, true);
		// only access the polygon with 4 sides
		if (approxCurve.size() != 4)
			continue;
		// only access the convex polygen
		if (!isContourConvex(approxCurve))
			continue;
		// find the shorted side of this polygen
		float minDist = FLT_MAX;
		for (int i=0; i<4; i++)
		{
			Point vecTemp = approxCurve[i] - approxCurve[(i+1)%4];
			float distSquared = vecTemp.dot(vecTemp);
			minDist = std::min(minDist, distSquared);
		}
		// if this side is too short, this polygen is not a marker
		if (minDist > minContourLengthAllowed)
		{
			Marker markerTemp;
			for (int i=0; i<4; i++)
			{
				markerTemp.vec_points.push_back(Point2f(approxCurve[i].x, approxCurve[i].y));
			}
			markerPossible.push_back(markerTemp);
		}
	}
	//order the markerpossible as reverse clockwise
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		Point v1 = markerPossible[i].vec_points[1] - markerPossible[i].vec_points[0];
		Point v2 = markerPossible[i].vec_points[2] - markerPossible[i].vec_points[0];
		double theta = (v1.x * v2.y) - (v1.y * v2.x);
		if (theta < 0.0)
		{
			std::swap(markerPossible[i].vec_points[1], markerPossible[i].vec_points[3]);
		}
	}
	vector<pair<int, int> > tooNearCandidates;
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		for (size_t j=i+1; j<markerPossible.size(); j++)
		{
			float distSquared = 0.0f;
			for (int k=0; k<4; k++)
			{
				Point vec = markerPossible[i].vec_points[k] - markerPossible[j].vec_points[k];
				distSquared += vec.dot(vec);
			}
			if (distSquared < 400)
			{
				tooNearCandidates.push_back(pair<int, int>(i,j));
			}
		}
	}
	vector<bool> markerRemoveIndex(markerPossible.size(), false);
	for (size_t i=0; i<tooNearCandidates.size(); i++)
	{
		float length1 = markerPossible[tooNearCandidates[i].first].calPerimeter();
		float length2 = markerPossible[tooNearCandidates[i].second].calPerimeter();
		markerRemoveIndex[(length1>length2) ? tooNearCandidates[i].second : tooNearCandidates[i].first] = true;
	}
	// remove the candidates with low certainty, get the final markers
	_detectedMarkers.clear();
	for (size_t i=0; i<markerPossible.size(); i++)
	{
		if (!markerRemoveIndex[i])
		{
			_detectedMarkers.push_back(markerPossible[i]);
		}
	}
}

void MarkerDetector::detectMarkers(const Mat& _imgGray, vector<Marker>& _detectedMarkers)
{
	Mat canonicalImg;	// a standard image
	vector<Marker> goodMarkers;

	// matching the marker according to ID
	for (size_t i=0; i<_detectedMarkers.size(); i++)
	{
		// get the perspective transform matrix
		Mat M = getPerspectiveTransform(_detectedMarkers[i].vec_points, vec_markerCorners2d);
		// warps the image using perspective transformation
		warpPerspective(_imgGray, canonicalImg, M, markerSize);
		// decode the marker
		int nRotations;
		int idMarker = Marker::decode(canonicalImg, nRotations);
		if (idMarker != -1)
		{
			_detectedMarkers[i].m_id = idMarker;
			// rotate the corner points
			std::rotate(_detectedMarkers[i].vec_points.begin(), _detectedMarkers[i].vec_points.begin()+4-nRotations, _detectedMarkers[i].vec_points.end());
			goodMarkers.push_back(_detectedMarkers[i]);
		}
	}

	// use subpixel to improve the accuracy of the corner
	if (goodMarkers.size() > 0)
	{
		vector<Point2f> preciseCorners(4*goodMarkers.size());

		for (size_t i=0; i<goodMarkers.size(); i++)
		{
			for (int j=0; j<4; j++)
			{
				preciseCorners[4*i+j] = goodMarkers[i].vec_points[j];
			}
		}

		cornerSubPix(_imgGray, preciseCorners, Size(5,5), Size(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER, 30, 0.1));

		for (size_t i=0; i<goodMarkers.size(); i++)
		{
			for (int j=0; j<4; j++)
			{
				goodMarkers[i].vec_points[j] = preciseCorners[4*i+j];
			}
		}
	}

	_detectedMarkers = goodMarkers;
}

