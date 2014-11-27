/************************************************************************
* Copyright(c) 2013  Yang Xian
* All rights reserved.
*
* File:	Marker.h
* Brief: ��ǩ��,ʵ��markerͼ�Ͷ�ӦID֮���ת��
* Version: 1.0
* Author: Yang Xian
* Email: yang_xian521@163.com
* Date:	2013/1/25 11:45
* History:
************************************************************************/
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include "Marker.h"

using namespace cv;

class Marker
{
public:
	Marker(void);
	~Marker(void);	
	static int decode(Mat& _input, int& _numRotation);
	float calPerimeter();
private:
	static Mat rotate(const Mat& _input);
	static int hammDistMarker(const Mat& _code);
	static int code2ID(const Mat& _code);
private:
	static const int m_idVerify[4][5];	// marker ID
public:
	int m_id;	// marker decoded ID
	vector<Point2f> vec_points;	// countour points of the detected marker
	Matx33f m_rotation;
	Vec3f m_translation;
};
