#include "lineFinder.h"

/*
 * Line constructor
 */
lineFinder::lineFinder(){
  // line parameter
  _deltaRho = 3;
  _deltaTheta = M_PI/180;
  _minVote = 60; //60
  _minLength = 60; //60
  _maxGap = 15;
  _drawLines = true;
  _foundInitLine = false;
  _minLineTrust = 0.5;
  _posLineArea = 15;
  _saveLinNumb = 4;
  _drawLines = true;
  _showImages = false;
  _DEBUG = false;
  std::cout << "Initializing lineFinder ... " << std::endl;
  std::cout << " -> make sure function - setImageParameter - is called for initialization " << std::endl;
  std::cout << std::endl;
  std::cout << "[lineFinder]: line initialization ... " << std::endl;
}

lineFinder::lineFinder(cv::Mat &inImg){
  // line parameter
  _deltaRho = 3;
  _deltaTheta = M_PI/180;
  _minVote = 60;
  _minLength = 60;
  _maxGap = 15;
  _drawLines = true;
  _foundInitLine = false;
  _minLineTrust = 0.5;
  _posLineArea = 15;
  _saveLinNumb = 4;
  _drawLines = true;
  _showImages = false;

// image parameter
_imgSize = inImg.size();
std::cout << "Initializing lineFinder ... " << std::endl;
//std::cout << "Video size: cols:" << inImg.cols << " rows: " << inImg.rows << std::endl;
_imgCenterX = inImg.cols/2;

_DEBUG = false;
}

/*
* Set image center
*/
void lineFinder::setImageParameter(cv::Mat &image){
_imgSize = image.size();
_imgCenterX = image.cols/2;
//std::cout << "Video size: cols: " << image.cols << " rows: " << image.rows << std::endl;
  if(_imgSize.width == 160){
    _minVote = 30;
    _minLength = 30;
  }
}

/*
* Set resolution
*/
void lineFinder::setAccResolution(double dRho, double dTheta){
_deltaRho = dRho;
_deltaTheta = dTheta;
}

/*
* Set the minimum number of votes
*/
void lineFinder::setMinVote(int minv){
_minVote = minv;
}
/*
* Set line length and gap
*/
void lineFinder::setLineLengthAndGap(double length, double gap){
_minLength = length;
_maxGap = gap;
}
/*
* Set all line parameters
*/
void lineFinder::setAllLineParameters(double dRho, double dTheta, int minv, double length, double gap){
_deltaRho = dRho;
_deltaTheta = dTheta;
_minVote = minv;
_minLength = length;
_maxGap = gap;
}

/*
* Image preprocessing
* -> such as Canny, convert to HSV
*/
void lineFinder::imagePreprocess(cv::Mat &origImage, cv::Mat &binary, cv::Mat &binaryHSV){
cv::Mat imgBinary, imgHSV, imgGRAY;
if(origImage.channels() ==3){
  cv::GaussianBlur(origImage, origImage, cv::Size(5,5),3);
  cv::cvtColor(origImage, imgGRAY, CV_RGB2GRAY);
  cv::cvtColor(origImage, imgHSV, CV_RGB2HSV);
  cv::inRange(imgHSV,cv::Scalar(0,0,240),cv::Scalar(255,255,255), binaryHSV);
}

cv::Canny(binaryHSV, binaryHSV, 20, 80, 3); //20, 80, 3
cv::Canny(imgGRAY, binary, 20, 80, 3);
}

/*
* Filter out lines that are not in a defined angle (area)
*/
void lineFinder::filterLinesAngle(std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &filteredLines){
std::vector<cv::Vec4i>::iterator it = lines.begin();
float r,theta, degTheta;
// filter out horizontal lines 
while(it!=lines.end()) {

  lineFromXY2Theta((*it), r, theta);
  degTheta = abs((theta*180)/M_PI);
  if ((degTheta > 120 || degTheta < 60)) { //vertical line
    filteredLines.push_back((*it));
  }
  ++it;
}
}

/*
* Filter out lines that are in the upper part of the image
*/
void lineFinder::filterLinesUpImgPart(std::vector<cv::Vec4i> &lines, std::vector<cv::Vec4i> &filteredLines, cv::Size imgSize){
std::vector<cv::Vec4i>::iterator it = lines.begin();
int range = (int)(float)imgSize.height*(3./5.);
// filter out lines that are in the upper part of the image
while(it!=lines.end()) {
  if ((*it)[3]> range) {
    filteredLines.push_back((*it));
  }
  ++it;
}
}
/*
* Apply probabilistic Hough Transform
*/
// save lines in global variable
void lineFinder::findHoughLines(cv::Mat& binary){
  _lines.clear();
  cv::HoughLinesP(binary, _lines, _deltaRho, _deltaTheta, _minVote, _minLength, _maxGap);
}

void lineFinder::findHoughLines(cv::Mat& binary, std::vector<cv::Vec4i> &lines){
  cv::HoughLinesP(binary, lines, _deltaRho, _deltaTheta, _minVote, _minLength, _maxGap);
}
/*
* Draw the detected lines on an image
* -> global lines
*/
void lineFinder::drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(255,255,0)){
  std::vector<cv::Vec4i>::const_iterator it= _lines.begin();
  while (it!=_lines.end()){
    cv::Point pt1((*it)[0],(*it)[1]);
    cv::Point pt2((*it)[2],(*it)[3]);

    cv::line(image, pt1,pt2, color, 2);
    ++it;
  }
}

/*
* Draw the detected lines on an image
*/
void lineFinder::drawDetectedLines(cv::Mat &image, std::vector<cv::Vec4i> &lines, cv::Scalar color=cv::Scalar(255,0,0)){
  std::vector<cv::Vec4i>::const_iterator it= lines.begin();
  while (it!=lines.end()){
    cv::Point pt1((*it)[0],(*it)[1]);
    cv::Point pt2((*it)[2],(*it)[3]);
    cv::line(image, pt1,pt2, color,2);
    ++it;
  }
}

/*
* Draw the detected lines on an image
*/
void lineFinder::drawDetectedLines(cv::Mat &image, cv::Vec4i &line, cv::Scalar color=cv::Scalar(255,0,0)){
    cv::Point pt1(line[0], line[1]);
    cv::Point pt2(line[2], line[3]);

    cv::line(image, pt1,pt2, color,1);
}
/*
* Draw bounding boxes
*/
void lineFinder::drawDetectedBBs(cv::Mat &image, std::vector<cv::Rect> &bboxes, cv::Scalar color=cv::Scalar(255,0,255)){
std::vector<cv::Rect>::iterator it= bboxes.begin();
for(;it!=bboxes.end(); it++){
  cv::rectangle(image,cv::Point((*it).x,(*it).y),cv::Point((*it).x+(*it).width,(*it).y+(*it).height), color); 
}
}
/*
* Calculates angle of a line
* -> this is necessary to restrict a defined area for calculation of Hough Transformation 
*/
float lineFinder::calcAngle(cv::Point &p1, cv::Point &p2){
cv::Point p_center(1,0);
cv::Point p_zero;
p_zero =  p2-p1;
float angle = 180/M_PI * atan2(p_zero.y - p_center.y, p_zero.x - p_center.x);
return angle;
}

/*
* Converts a line defined by its two end-points into its r and theta 
* (origin is at top-left corner with x right and y down
* and theta measured positive clockise -i < theta < pi)
*/
void lineFinder::lineFromXY2Theta(const cv::Vec4i &line, float &r, float &theta){
//check if vertical line x1==x2
if(line[0] == line [2]){
  r = fabs(line[0]);
  //theta is 0 or pi
  theta = line[0]>=0 ? 0. : M_PI;

//check if horizontal line y1==y2
}else if (line[1] == line[3]){
  //r is y
  r = fabs(line[1]);
  //theta is pi/2 or -pi/2
  theta = (float) line[1]>=0 ? M_PI/2 : -M_PI/2;
//check general line
}else {
  // tan(theta) = (x2-x1)/(y1-y2)
  theta = atan2(line[2]-line[0],line[1]-line[3]);
  // r = x*cos(theta)+y*sin(theta)
  float r1 = line[0]*cos(theta) + line[1]*sin(theta);
  r = line[2]*cos(theta) + line[3]*sin(theta);
  // adjust to add pi if necessary
  if(r1<0 || r<0){
    //add pi
    theta += M_PI;
    if(theta > M_PI)
      theta -= 2*M_PI;
    r = fabs(r);
  }
}
}


/*
* Checks if point is inside the bounding box (BB)
*/
bool lineFinder::isPointInsideBB(cv::Point2f point, cv::Size bbox){
return (point.x>=0 && point.x<=bbox.width
    && point.y>=0 && point.y<=bbox.height) ? true : false;
}

/*
* Checks if the input line (given r and theta) intersects with the given bounding box.
* The line is represented by: x cos(theta) + y sin(theta) = r
*/
void lineFinder::intersectLineRThetaWithBB(float r, float theta, const cv::Size bbox, cv::Vec4i *outLine){
//hold parameters
double xup, xdown, yleft, yright;
//intersect with top and bottom borders: y=0 and y=bbox.height-1
if (cos(theta)==0){ //horizonal lines
  xup = xdown = bbox.width+2;
}else{
  xup = r / cos(theta);
  xdown = (r-bbox.height*sin(theta))/cos(theta);
}
//intersect with left and right borders: x=0 and x=bbox.widht-1
if (sin(theta)==0){
  yleft = yright = bbox.height+2;
}else{
  yleft = r/sin(theta);
  yright = (r-bbox.width*cos(theta))/sin(theta);
}

//points of intersection
cv::Point2f pts[4];
pts[0].x = xup; pts[0].y = 0; 
pts[1].x = xdown; pts[1].y = bbox.height; 
pts[2].x = 0; pts[2].y = yleft; 
pts[3].x = bbox.width; pts[3].y = yright; 
//get the starting point
int i;
for (i=0; i<4; i++){
  //if point inside, then put it  
  if(isPointInsideBB(pts[i], bbox)){
    outLine[0][0] = pts[i].x;
    outLine[0][1] = pts[i].y;
    //get out of for loop
    break;
  }
}
//get the ending point
for (i++; i<4; i++){
  //if point inside, then put it
  if(isPointInsideBB(pts[i], bbox)){
    outLine[0][2] = pts[i].x;
    outLine[0][3] = pts[i].y;
    //get out of for loop
    break;
  }
}

}
/*
* Groups nearby lines
*/
void lineFinder::groupNearbyLines(std::vector<cv::Vec4i> &lines, float groupThreshold, cv::Size bbox){
//converts the lines into r-theta parameters
int numInLines = lines.size();
std::vector<float> rs(numInLines);
std::vector<float> thetas(numInLines);
for(int i=0; i<numInLines; i++)
  lineFromXY2Theta(lines[i], rs[i], thetas[i]);
bool stop = false; // to stop loop
while(!stop){
  //minimum distance so far
  float minDist = groupThreshold+5, dist;
  std::vector<float>::iterator ir, jr, itheta, jtheta, minIr, minJr, minItheta, minJtheta;
  //compute pairwise distance between detected maxima
  for (ir=rs.begin(), itheta=thetas.begin(); ir!=rs.end(); ir++, itheta++)
    for (jr=ir+1, jtheta=itheta+1; jr!=rs.end(); jr++, jtheta++){
      //add pi if neg
      float t1 = *itheta<0 ? *itheta : *itheta+M_PI;
      float t2 = *jtheta<0 ? *jtheta : *jtheta+M_PI;
      //get distance
      dist = 1 * fabs(*ir - *jr) + 1 * fabs(t1 - t2);//fabs(*itheta - *jtheta);
      //check if minimum
      if (dist<minDist){
        minDist = dist;
        minIr = ir; minItheta = itheta;
        minJr = jr; minJtheta = jtheta;
      }
    }

  //check if minimum distance is less than groupThreshold
  if (minDist >= groupThreshold)
    stop = true;
  else
  {
    //put into the first
    *minIr = (*minIr + *minJr)/2;
    *minItheta = (*minItheta + *minJtheta)/2;
    //delete second one
    rs.erase(minJr);
    thetas.erase(minJtheta); 
  }
}
// put back lines 
lines.clear();
for(int i=0; i<(int)rs.size(); i++){
  cv::Vec4i line;
  intersectLineRThetaWithBB(rs[i], thetas[i], bbox, &line);
  lines.push_back(line);
}

}

/*
* Extracts bounding boxes from lines
* - TODO: at the moment this function is only for vertical lines: maybe some lineType to differ
*/ 
void lineFinder::calcLinesBoundingBoxes(cv::vector<cv::Vec4i> &lines, CvSize size, cv::vector<cv::Rect> &boxes){
int start, end; // start- end point
boxes.clear();
//vertical lines
for(unsigned int i=0; i<lines.size(); ++i){
  //get min and max x and add the bounding box covering the whole height
  start = (int)fmin(lines[i][0], lines[i][2]); // [0] = startPoint, [2] = endPoint
  end = (int)fmax(lines[i][0], lines[i][2]);
  boxes.push_back(cv::Rect(start, 0, end-start+1, size.height-1));
}

}

/*
* Groups together bounding boxes
* - TODO: at the moment only fertical lines -> no horizontal lines
*/
void lineFinder::groupBoundingBoxes(cv::vector<cv::Rect> &boxes, float groupThreshold){
bool cont = true;
//TODO: check if to intersect with bounding box or not
//loop to get the largs ovelap and check the overlap ratio
float overlap, maxOverlap;
while(cont){
  maxOverlap = overlap = -1e5;
  // get max overlap
  cv::vector<cv::Rect>::iterator i, j, maxI, maxJ;
  for(i = boxes.begin(); i != boxes.end(); i++){
    for(j=i+1; j!=boxes.end(); j++){
      // smallest x, and compute the x2 - x1 / width of smallest
      // (x12 - x21) / (x22 - x21)
      //std::cout << " x: " << i->x << " y: " << j->x << " i width: " << i->width << " j width: " << j->width << std::endl;

      overlap = i->x < j->x ?
        (i->x + i->width - j->x) / (float)j->width :
        (j->x + j->width - i->x) / (float)i->width;

      //get maximum
      if(_DEBUG)
        std::cout << "maximum" << std::endl;
      if(overlap > maxOverlap){
        maxI = i;
        maxJ = j;
        maxOverlap = overlap;
      }
    }
  }
  if(_DEBUG)
    std::cout << "check max overlap" << std::endl;
  // check the max overlap against the threshold
  if(maxOverlap >= groupThreshold){
    *maxI = cv::Rect(cv::min((*maxI).x, (*maxJ).x),
        cv::min((*maxI).y, (*maxJ).y),
        cv::max((*maxI).width, (*maxJ).width),
        cv::max((*maxI).height, (*maxJ).height));
    boxes.erase(maxJ);
  }else
    cont = false; // stop
}
}

/*
* Normal distribution (1D): calculate belief of a line, depends on position, angle
*  and numbe of lines
*/
float lineFinder::normalDist(int x, int mu, float sigma){
float normDist =  exp(-((x-mu)*(x-mu))/(2*sigma*sigma));
return normDist;
}


float lineFinder::normalDist(float x, float mu, float sigma){
float normDist =  exp(-((x-mu)*(x-mu))/(2*sigma*sigma));
return normDist;
}

/*
* Evaluate line
* -> precise description follows
* return: highest belief state 
*/
float lineFinder::evaluateLines(std::vector<cv::Vec4i> &lines, cv::Vec4i &evLine){
  // belief state from 0 (no) to 1 (trust)
  int lineSize = lines.size();
  float belief = 0.;
  float posBelief = 0.;
  float degBelief = 0.;
  float numBelief = 0.;
  float sigma = 10.;
  
  if(lineSize>=1){
    std::vector<float> lineBelief; // vector to store all belief states
    std::vector<cv::Vec4i>::iterator it_lines, maxBeliefLine; // iterator to store line with highest score

    /*
     * going through all lines and calculate the belief of each line
     */
    it_lines = lines.begin();
    float r,theta;
    // Get line with minimum distance to previous line
    int tempDist = 0; int minDist = 1000;
    for(;it_lines!=lines.end(); it_lines++){
      belief = 0.;
      cv::Vec4i line = (*it_lines); // set line for evaluation
      // calculate belief of position (normal distribution)
      posBelief = normalDist(line[2], _curLinePos, 15.);
      
      // calculate belief of degree
      lineFromXY2Theta(line, r,theta);
      theta = (float)(180.*theta)/M_PI;
      r = (float)(180.*_curLineAngle)/M_PI;
      degBelief = normalDist(theta, r, 15.);
      
      // calculate belief depend on current number of lines
      numBelief = (1./(float)lines.size());
      //calculate total belief
      belief = (1./5.)*(3.*posBelief+degBelief+numBelief);

      //At the moment take only the nearest line to position before
      /*
       * tempDist = (int)abs(_curLinePos - line[2]);
       * if(tempDist<minDist){
       *   minDist = tempDist;
       *   maxBeliefLine = it_lines;
       * }
       */  

      lineBelief.push_back(belief);
    }
    /*
     * return line with highest belief value
     */
    it_lines = lines.begin();
    float maxBelief = 0.;
    for(std::vector<float>::iterator itScore = lineBelief.begin(); itScore!=lineBelief.end(); itScore++){

      float curBelief = (*itScore);
      // evaluate if current belief is higher than maxBelief
      if(curBelief>=maxBelief){
        maxBelief = curBelief;
        maxBeliefLine = it_lines;
      }

      if(it_lines!=lines.end())
        it_lines++;
    }
    //TODO: if(maxBelief != 0.) -> attention segmentation fault
    if(maxBelief != 0.)
      evLine = (*maxBeliefLine);
  }
  return belief;
}

/*
 * Calculate current line position
 */
// only from one line
int lineFinder::calcLinePosition(std::vector<cv::Vec4i> &lines, cv::Vec4i &bestLine, float &curBelief){
  // return true if line is found
  bool foundLine = false;
  int linePos = 0;
  int lineSize = lines.size();
  if(!_foundInitLine){
    // Only for line initializing step
    // Check if only one line exist and if it is at the image center (x)
    if(lineSize == 1){
      if(lines[0][2]>= _imgCenterX-_posLineArea && lines[0][2] <= _imgCenterX+_posLineArea){
        linePos = (int)lines[0][2];

        _foundInitLine = true;
        foundLine = true;
        // save init line position: for further calculation we need all position values of _saveLinePos
        for(int i=0; i<_saveLinNumb; i++){
          _saveLinePos.push_back(linePos);
        }
      }
    }
  }else{

    if(lineSize >= 1){
      curBelief = evaluateLines(lines, bestLine);

      linePos = (int)bestLine[2];
    }else{
      // set value as not possible
      linePos = 8000;
    }
  }
  return linePos;
}
// combine two different measurements
int lineFinder::calcLinePosition(std::vector<cv::Vec4i> &line1, std::vector<cv::Vec4i> &line2){
  int pos = 0;

  return pos;
}

/*
 * Calculate line position, including the last n-steps in the path
 */
int lineFinder::calcLinePosPast(std::vector<int> &linePos){
  int lineSize = linePos.size();
  if(lineSize == 0){
    std::cout << "[Warning]: [lineFinder].calcLinePosition -> there exist no line" << std::endl;
    return 0;
  }else{
    int avgPos = 0; // average line position
    for(std::vector<int>::iterator itPos = linePos.begin(); itPos!=linePos.end(); itPos++){
      avgPos += (*itPos);
    }
    
    return avgPos/lineSize;
  }
}
/*
 * Run lineFinder: Main function of lineFinder
 * return: position of line (pixel coordinates)
 */

int lineFinder::processLineFinder(cv::Mat &origImage){
  if(_DEBUG)
    std::cout << "[lineFinder] -> processLineFinder" << std::endl;
  //float cur_degree = 0; // Temporal variable to save current degree of Hough Transformation
  /*
   * Image preprocessing, Canny, HSV,...
   * ---------------------------------------------------------------
   */
  cv::Mat binaryImage, binaryImgHSV;
  imagePreprocess(origImage, binaryImage, binaryImgHSV);
  //int x_center = binaryImage.cols/2; // Initialize image x center position
  //_imgCenterX = x_center; // set image center of x direction global
  /*
   * Apply Probabilistic Hough Transformation
   */
  std::vector<cv::Vec4i> linesHSV, linesBin;
  findHoughLines(binaryImage, linesBin); // binary image
  findHoughLines(binaryImgHSV, linesHSV); // HSV image
  /*
   * Preprocessing of calculated lines
   * ---------------------------------------------------------------
   */
  std::vector<cv::Vec4i> filteredLines, filterdBinLines, filterdHSVLines, filterdBinLinesAngle, filterdHSVLinesAngle;
  // Filter out horizontal lines
  // Take only lines of a defined angle range
  filterLinesAngle(linesBin, filterdBinLinesAngle);
  filterLinesAngle(linesHSV, filterdHSVLinesAngle);
  // filter out lines in upper image part
  filterLinesUpImgPart(filterdBinLinesAngle, filterdBinLines, cv::Size(binaryImage.cols, binaryImage.rows));
  filterLinesUpImgPart(filterdHSVLinesAngle, filterdHSVLines, cv::Size(binaryImage.cols, binaryImage.rows));
 
  /*
   * Grouping founded lines
   * --------------------------------------------------------------
   */
  if(_DEBUG)
    std::cout << "[lineFiner] -> groupLines" << std::endl;
  // grouping lines into regions;
  float groupThreshold =45.; 
  if(origImage.rows == 160.){
    groupThreshold = 10.;
  }else if (origImage.rows == 320.){
    groupThreshold = 25.;
  }
  
  groupNearbyLines(filterdBinLines, groupThreshold, cv::Size(binaryImage.cols-1, binaryImage.rows-1));
  groupNearbyLines(filterdHSVLines, groupThreshold, cv::Size(binaryImage.cols-1, binaryImage.rows-1));

  
  // calculate bounding box around lines
  float overlapThreshold = 0.5;
  std::vector<cv::Rect> boxes;
  /*
   * calcLinesBoundingBoxes(filteredLines, cv::Size(binaryImage.cols, binaryImage.rows), boxes);
   * if(boxes.size()>=2){
   *   groupBoundingBoxes(boxes, overlapThreshold);
   * }
   */

  /*
   * RANSAC
   * --------------------------------------------------------------
   */

  /*
   * Calculate line position
   * --------------------------------------------------------------
   */
  if(_DEBUG)
    std::cout << "[lineFiner] -> calcLinePosition" << std::endl;
  float curBeliefBin = 0.;
  float curBeliefHSV = 0.;
  float tempr = 0.;
  cv::Vec4i tempLineBin, tempLineHSV;
  if(!_foundInitLine){
    // Initialization step
    _curLinePos = _imgCenterX; //
    int tmpPos = calcLinePosition(filterdBinLines, _curLine, curBeliefBin);
    //std::cout << " ... " << std::endl;
    std::cerr << " ... ";
    if(_foundInitLine){
      _curLinePos = tmpPos;
      std::cout << std::endl;
      std::cout << "[lineFinder]: line initialization -> finished " << std::endl;
    }
  }else{
    int curLinePosBin = _curLinePos;
    int curLinePosHSV = _curLinePos;
    curLinePosBin = calcLinePosition(filterdBinLines, tempLineBin, curBeliefBin);
    curLinePosHSV = calcLinePosition(filterdHSVLines, tempLineHSV, curBeliefHSV);

    // set best line global
    if(curBeliefBin>curBeliefHSV){
      if(curLinePosBin < origImage.size().width){ // additional check if line position is possible
        _curLinePos = curLinePosBin;
        _curLine = tempLineBin;
        lineFromXY2Theta(_curLine, tempr, _curLineAngle);
      }
    }else{
      if(curLinePosHSV < origImage.size().width){ // additional check if line position is possible
        _curLinePos = curLinePosHSV;
        _curLine = tempLineHSV;
      }
    }

    if(_curLinePos < origImage.size().width){ // additional check if line position is possible
      // If a line was found it is saved in _saveLinePos and the first entry is deleted
      // Afterwards calculate line position from remaining entries
      // _curLinePos == 8000 if something goes wrong
      _saveLinePos.push_back(_curLinePos);
      _saveLinePos.erase(_saveLinePos.begin());
      _curLinePos = calcLinePosPast(_saveLinePos);
    }
    
  }


  /*
   * Show lines
   * --------------------------------------------------------------
   */
  // draw lines and bounding boxes
  if(_showImages && _drawLines){

    cv::Mat clone_inputImg;// = cv::Mat(&origImage, true);
    clone_inputImg = origImage.clone();
    //drawDetectedLines(clone_inputImg, filterdHSVLines);
    
    cv::Point pt_avg11(_curLinePos, origImage.rows);
    cv::Point pt_avg12(_curLinePos, (int)origImage.rows-(origImage.rows/10));
    cv::line(origImage,pt_avg12, pt_avg11, cv::Scalar(128),3);
    drawDetectedLines(binaryImage, filterdBinLines);
    drawDetectedLines(binaryImgHSV, filterdHSVLines);

    drawDetectedLines(origImage, _curLine);
    //drawDetectedLines(binaryImgHSV, linesHSV);
    //drawDetectedLines(clone_inputImg, filterdBinLines);
    //drawDetectedBBs(origImage, boxes);
    //drawDetectedLines(binaryImage);
    //cv::imshow("clone", clone_inputImg);
    cv::imshow("binary", origImage);
    cv::imshow("HSV_bin", binaryImgHSV);
    cv::imshow("binary_bin", binaryImage);

  }else if(!_showImages && _drawLines){
    cv::Point pt_avg11(_curLinePos, origImage.rows);
    cv::Point pt_avg12(_curLinePos, (int)origImage.rows-(origImage.rows/10));
    cv::line(origImage,pt_avg12, pt_avg11, cv::Scalar(128),3);
    drawDetectedLines(origImage, _curLine);
  }

 return _curLinePos;
}

/*
 * Calculate second derivative of Gaussian
 */
cv::Mat get2DerivGaussKernel(unsigned char w, float sigma){   
  /*
   * for (double i=-w; i<=w; i++)
   *     CV_MAT_ELEM(*kernel, CV_32FC1, int(i+w), 0) =
   *         (CV_32FC1)
   *         (exp(-.5*i*i)/sigma - (i*i)*exp(-(.5/sigma)*i*i)/(sigma*sigma));  
   */
  cv::Mat gaussKernel = cv::getGaussianKernel(2*w, sigma);
  cv::Point anchor = cvPoint(-1,-1);
  int type = cv::getKernelType(gaussKernel, anchor);
  cv::Mat LoG;
  cv::Laplacian(gaussKernel, LoG, type, 2*w, 1, 0, cv::BORDER_DEFAULT);
  return LoG;
}
