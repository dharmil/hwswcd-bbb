#ifndef MARKERTRACKING_H_
#define MARKERTRACKING_H_

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

class markerTracking{
  
  public:
    markerTracking();
    /*
     * Main function of marker tracking
     * -> detects colored rectangles
     */ 
    bool processMarkerTrackingColor(Mat &originalImage, Point3f &markerPosition);
    /*
     * Main function of marker tracking
     * -> detects marker
     */
    bool processMarkerTracking(Mat &originalImage, Point3f &markerPosition);

  private:
    int iThreshold;
    int SliderPos;
    int ThresholdSliderPos;
    
    vector< vector< Point > > sContour;
    vector< Point > Rectangle;
    CvMemStorage* memStorage;

    bool bFirstStripe;
    bool bFirstMarker;

    //Point3f markerPosition; // current marker position

    Mat mOriginalImage;
    Mat mGrayImage;
    Mat mThresholdImage;

    int SubPixelAccuracy( const Mat &pSrc, const Point2f &p);

};
#endif //LINEFINDER_H_
