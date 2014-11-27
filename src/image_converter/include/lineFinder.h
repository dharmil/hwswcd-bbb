#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <iostream>
class lineFinder{
  
  public:
    //lineFinder() : _deltaRho(3), _deltaTheta((M_PI/180)), _minVote(60), _minLength(60), _maxGap(15), _drawLines(true), _foundInitLine(false), _minLineTrust(0.5), _posLineArea(15), _saveLinePos(4), _DEBUG(false){}
    lineFinder();
    lineFinder(cv::Mat &inImg);
    /*
     * Set image center
     */
    void setImageParameter(cv::Mat &image);
    /*
     * Set resolution
     */
    void setAccResolution(double dRho, double dTheta);
    /*
     * Set the minimum number of votes
     */
    void setMinVote(int minv);
    /*
     * Set line length and gap
     */
    void setLineLengthAndGap(double length, double gap);
    /*
     * Set all parameters
     */
    void setAllLineParameters(double dRho, double dTheta, int minv, double length, double gap);
    /*
     * Run lineFinder: Main function of lineFinder
     * return: position of line (pixel coordinates)
     */
    //int processLineFinder(cv::Mat &binaryImage, cv::Mat &origImage);
    int processLineFinder(cv::Mat &origImage);
    /*
     * Calculate sedond derivative of Gaussian
     */
    cv::Mat get2DerivGaussKernel(unsigned char w, float sigma);

  private:
    cv::Mat img; //original image
    //double _count;
    // line parameters
    // ------------------------------------
    std::vector<cv::Vec4i> _lines; // vector containing the end points of detected line
    // resolution parameters
    double _deltaRho; // The resolution of the parameter r in pixels
    double _deltaTheta; // The resolution of the parameter /theta in radians
    int _minVote; // minimum number of votes that a line must receive
    double _minLength; // min length for a line
    double _maxGap; //allowed gap along the line
    double _x_pos;
    bool _drawLines; // boolean to draw line (default is false as it is not possible to visualize something on the beagle bone)
    bool _foundInitLine; // parameter is initialized with false. Gets true if line is found for the first time
    float _minLineTrust; // lower bound of line trust parameter (0-1)

    // position parameter
    // ------------------------------------
    int _curLinePos; // saves current line position
    int _imgCenterX; // image center in x direction
    int _posLineArea; // area where the line should be
    int _updatePos; // time steps after updating position
    int _countPosTime;
    int _saveLinNumb; // number of lines to calculate current position
    std::vector<int> _saveLinePos; // save defined number of lines
   // image preprocessing parameter
    cv::Size _imgSize;
    // ------------------------------------
    
    bool _DEBUG; // parameter for debugging

    /*
     * Image preprocessing
     * -> such as Canny, convert to HSV
     */
    void imagePreprocess(cv::Mat &origImage, cv::Mat &binary, cv::Mat &binaryHSV);
    /*
     * Apply probabilistic Hough Transform
     */
    void findHoughLines(cv::Mat& binary); // save line in global variable
    void findHoughLines(cv::Mat& binary, std::vector<cv::Vec4i> &lines);
    /*
     * Filter out lines that are not in a defined angle (area)
     */
    void filterLinesAngle(std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &filteredLines);
    /*
     * Filter out lines that are in the upper part of the image
     */
    void filterLinesUpImgPart(std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &filteredLines, cv::Size imgSize);
    /*
     * Draw the detected lines on an image
     * -> global defined lines
     */
    void drawDetectedLines(cv::Mat &image, cv::Scalar color);
    /*
     * Draw detected lines on an image
     */
    void drawDetectedLines(cv::Mat &image, std::vector<cv::Vec4i> &lines, cv::Scalar color);
    /*
     * Draw bounding boxes
     */
    void drawDetectedBBs(cv::Mat &image, std::vector<cv::Rect> &bboxes, cv::Scalar color);
    /*
     * Calculates angle of a line
     * -> this is necessary to restrict a defined area 
     */
    float calcAngle(cv::Point &p1, cv::Point &p2);
    /*
     * Converts a line defined by its two end-points into its r and theta (origin is at top-left corner with x right and y don and theta measured positive clockise -i < theta < pi)
     */
    void lineFromXY2Theta(const cv::Vec4i &line, float &r, float &theta);
    /*
     * Checks if point is inside the bounding box (BB)
     */
    bool isPointInsideBB(cv::Point2f point, cv::Size bbox);
    /*
     * Checks if the input line (given r and theta) intersects with the given bounding box.
     * The line is represented by: x cos(theta) + y sin(theta) = r
     */
    void intersectLineRThetaWithBB(float r, float theta, const cv::Size bbox, cv::Vec4i *outLine);
    /*
     * Groups nearby lines
     */
    void groupNearbyLines(std::vector<cv::Vec4i> &lines, float groupThreshold, cv::Size bbox);   
    /*
     * Extracts bounding boxes from lines
     */ 
    void calcLinesBoundingBoxes(cv::vector<cv::Vec4i> &lines, CvSize size, cv::vector<cv::Rect> &boxes);
    /*
     * Groups together bounding boxes
     */
    void groupBoundingBoxes(cv::vector<cv::Rect> &boxes, float groupThreshold);
    /*
     * Evaluate line
     * -> precise description follows 
     */
    float evaluateLines(std::vector<cv::Vec4i> &line, cv::Vec4i &evLine);
    /*
     * Calculate current line position
     */
    // only from one line
    bool calcLinePosition(std::vector<cv::Vec4i> &lines, int &posLine);
    // combine two different measurements
    int calcLinePosition(std::vector<cv::Vec4i> &line1, std::vector<cv::Vec4i> &line2);
    /*
     * Calculate line position, including the last n-steps in the path
     */
    int calcLinePosPast(std::vector<int> &linePos);

};
