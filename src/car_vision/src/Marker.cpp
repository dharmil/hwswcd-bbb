#include "Marker.h"
#include <iostream>
#include <stdlib.h>
//const int Marker::m_idVerify[4][5] = {{1,0,0,0,0},
//						{1,0,1,1,1},
//						{0,1,0,0,1},
//						{0,1,1,1,0}};	// id information for 00、01、10、11
const int Marker::m_idVerify[4][5] = {{1,0,0,1,0},
						{1,0,1,1,1},
						{0,1,0,0,1},
						{0,1,1,1,0}};	// id information for 00、01、10、11

Marker::Marker(void)
{
	m_id = -1;
	m_rotation.eye();
	m_translation.zeros();
}

Marker::~Marker(void)
{
}

Mat Marker::rotate(const Mat& _input)
{
	// rotate the code for 90 degrees
	Mat output;
	_input.copyTo(output); 
	for (int i=0; i<_input.rows; i++)
	{
		for (int j=0; j<_input.cols; j++)
		{
			output.at<uchar>(i,j) = _input.at<uchar>(_input.cols-j-1, i);
		}
	}
	return output;
}

int Marker::hammDistMarker(const Mat& _code)
{
	int dist = 0;

	for (int i=0; i<5; i++)
	{
		int minDistWord = INT_MAX;
		for (int x=0; x<4; x++)	// match each word with current line of ID code, and return the most similar result
		{
			int sumTemp = 0;
			for (int j=0; j<5; j++)
			{
				sumTemp += (_code.at<uchar>(i,j) == m_idVerify[x][j]) ? 0 : 1;
			}
			if (minDistWord > sumTemp)
			{
				minDistWord = sumTemp;
			}
		}
		//the sum of the similarity of all 5 lines of code
		dist += minDistWord;
	}
	
	return dist;
}

int Marker::code2ID(const Mat& _code)
{
	// 因为只有4个wordID，取第2位和第4位作为id，4个word的id分别为00,01,10,11
	int val = 0;
	for (int i=0; i<5; i++)
	{
//		std::cout << "current i is"<<i<<"cunrrent val is"<<val <<std::endl;
//		system ("PAUSE");
		val <<= 1;
		if (_code.at<uchar>(i,1))
			val |= 1;
		val <<= 1;
		if (_code.at<uchar>(i,4))
		{
			val |= 1;
		}
	}
	return val;
}

int Marker::decode(Mat& _input, int& _numRotation)
{
	CV_Assert(_input.rows == _input.cols);
	CV_Assert(_input.type() == CV_8UC1);

	// convert the input image into binary image
	Mat grey;
	threshold(_input, grey, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
// 	imshow("improvedMarker", grey);
	
	// Maker is a 7*7 marker, but the id information is only contained in the central 5*5 region
	int patchSize = _input.rows / 7;
	// check if the boundary is all filled
	for (int i=0; i<7; i++)
	{
		int step = 6;
		if (i==0 || i==6)
			step = 1;
		for (int j=0; j<7; j+=step)
		{
			int x = j * patchSize;
			int y = i * patchSize;
			Mat patch = grey(Rect(x, y, patchSize, patchSize));
			int numNoZero = countNonZero(patch);
			if (numNoZero > (patchSize*patchSize/2) )
			{
				return -1;	// if the boundary is not totally filled, its not a marker
			}
		}
	}

	// get the id code information
	Mat codemap = Mat::zeros(5, 5, CV_8UC1);
	for (int i=0; i<5; i++)
	{
		for (int j=0; j<5; j++)
		{
			int x = (j+1) * patchSize;
			int y = (i+1) * patchSize;
			Mat patch = grey(Rect(x, y, patchSize, patchSize));
			int numNoZero = countNonZero(patch);
			if (numNoZero > (patchSize*patchSize/2) )
			{
				codemap.at<uchar>(i,j) = 1;
			}
		}
	}
	//decode, caculate the minimum distance to ensure the robust of the code
	int minDist = INT_MAX;
	int minDistIndex = -1;
	Mat codemapRotate[4];
	codemapRotate[0] = codemap;
	for (int i=0; i<4; i++)	// rote 4 times
	{
		int distTemp = hammDistMarker(codemapRotate[i]);
		if (distTemp < minDist)
		{
			minDist = distTemp;
			minDistIndex = i;
		}
		codemapRotate[(i+1)%4] = rotate(codemapRotate[i]);
	}

	_numRotation = minDistIndex;
	if (minDist == 0)	// matching and return the id
	{
		return code2ID(codemapRotate[minDistIndex]);
	}

	return -1;	// not match with the ID, not a marker
}

float Marker::calPerimeter()
{
	float sum = 0.0f;
	for (size_t i=0; i<vec_points.size(); i++)
	{
		size_t j = (i+1) % vec_points.size();
		Point2f vec = vec_points[i] - vec_points[j];
		sum += sqrt(vec.dot(vec));
	}
	return sum;
}
