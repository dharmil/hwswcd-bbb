#include "markerdetector.h"

markerDetector::markerDetector(const double param1, const double param2, const int param3, const double param4, const int param5, const int thresh_mode, const double markSize, const char* filename, int patCount)
{
  std::cout << "[MarkerDetector]: marker initialization ... " << std::endl;
  thresh1 = param1;//for image thresholding
  thresh2 = param2;// for adaptive thresholding
  block_size = param3;//for adaptive image thresholding
  mode = thresh_mode;//1 means fixed threshol, otherwise the adaptive algorithm is enabled
  confThreshold = param4;//bound for accepted similarities between detected patterns and loaded patterns
  normSize = param5;// the size of normalized ROI 
  normROI = Mat(normSize,normSize,CV_8UC1);//normalized ROI
  markerSize = markSize; //size of the marker to calculate depth (only approximation)

  //Masks for exterior(black) and interior area inside the pattern
  patMask = Mat::ones(normSize,normSize,CV_8UC1);
  Mat submat = patMask(cv::Range(normSize/4,3*normSize/4), cv::Range(normSize/4, 3*normSize/4));
  submat = Scalar(0);

  patMaskInt = Mat::zeros(normSize,normSize,CV_8UC1);
  submat = patMaskInt(cv::Range(normSize/4,3*normSize/4), cv::Range(normSize/4, 3*normSize/4));
  submat = Scalar(1);


  //corner of normalized area
  norm2DPts[0] = Point2f(0,0);
  norm2DPts[1] = Point2f(normSize-1,0);
  norm2DPts[2] = Point2f(normSize-1,normSize-1);
  norm2DPts[3] = Point2f(0,normSize-1);

  //load pattern
  //Pattern pattern;
  Pattern pattern;
  pattern.loadPattern(filename, normSize, library, patCount);
}

bool markerDetector::detect(Mat& frame, const Mat& cameraMatrix, const Mat& distortions, vector<Pattern>& foundPatterns, Point3f& position)
{

	patInfo out;
	Point2f roi2DPts[4];
	Mat binImage2;

	//binarize image
	convertAndBinarize(frame, binImage, grayImage, mode);
	binImage.copyTo(binImage2);

	//imshow("binaryImage" , binImage2);
  //calculate scaling factor to calculate depth of marker
  int scaleMarker = 1;
  if( frame.cols == 320 ){
    scaleMarker = 2;
  }else if ( frame.cols == 160 ){
    scaleMarker = 4;
  }else if( frame.cols == 640 ){
    scaleMarker = 1;
  }else{
    cout << "[Warning] MarkerDetector -> detect(): wrong scale level to calculate depth!" << endl;
  }

	int avsize = (binImage.rows+binImage.cols)/2;

	vector<vector<Point> > contours;
	vector<Point> polycont;

	//find contours in binary image
	cv::findContours(binImage2, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	unsigned int i;
	Point p;
	int pMinX, pMinY, pMaxY, pMaxX;

	for(i=0; i<contours.size(); i++){
		Mat contourMat = Mat (contours[i]);
		const double per = arcLength( contourMat, true);
		//check the perimeter
		if (per>(avsize/4) && per<(4*avsize)) {
			polycont.clear();
			approxPolyDP( contourMat, polycont, per*0.02, true);

			//check rectangularity and convexity
			if (polycont.size()==4 && isContourConvex(Mat (polycont))){

				//locate the 2D box of contour,
				p = polycont.at(0);
				pMinX = pMaxX = p.x;
				pMinY = pMaxY = p.y;
				int j;
				for(j=1; j<4; j++){
					p = polycont.at(j);
					if (p.x<pMinX){
						pMinX = p.x;
						}
					if (p.x>pMaxX){
						pMaxX = p.x;
						}
					if (p.y<pMinY){
						pMinY = p.y;
						}
					if (p.y>pMaxY){
						pMaxY = p.y;
						}
				}
				Rect box(pMinX, pMinY, pMaxX-pMinX+1, pMaxY-pMinY+1);
				
				//find the upper left vertex
				double d;
				double dmin=(4*avsize*avsize);
				int v1=-1;
				for (j=0; j<4; j++){
					d = norm(polycont.at(j));
					if (d<dmin) {
					dmin=d;
					v1=j;
					}
				}

				//store vertices in refinedVertices and enable sub-pixel refinement if you want
				vector<Point2f> refinedVertices;
				refinedVertices.clear();
				for(j=0; j<4; j++){
					refinedVertices.push_back(polycont.at(j));
				}

				//refine corners
				cornerSubPix(grayImage, refinedVertices, Size(3,3), Size(-1,-1), TermCriteria(1, 3, 1));
				
				//rotate vertices based on upper left vertex; this gives you the most trivial homogrpahy 
				for(j=0; j<4;j++){
					roi2DPts[j] = Point2f(refinedVertices.at((4+v1-j)%4).x - pMinX, refinedVertices.at((4+v1-j)%4).y - pMinY);
				}

				//normalize the ROI (find homography and warp the ROI)
				normalizePattern(grayImage, roi2DPts, box, normROI);
					//imshow("normROI",normROI);
					//cvWaitKey(0);
					
					//IplImage nnn = (IplImage) normROI;
					//imwrite("normROI.png", normROI);

				const int retvalue = identifyPattern(normROI, library, out);

				//push-back pattern in the stack of foundPatterns and find its extrinsics
				if (retvalue>0) {
					Pattern patCand;
					patCand.id = out.index;
					patCand.orientation = out.ori;
					patCand.confidence = out.maxCor;
          // calculate/save pattern position
          Moments moment;
          moment = moments( refinedVertices, false);
          patCand.position.x = moment.m10/moment.m00; //set x position
          patCand.position.y = moment.m01/moment.m00; //set y position

          //calculate z position -> only cruel approximation
          
          Rect rec = boundingRect(refinedVertices);
          patCand.position.z = (71*markerSize)/(rec.height*scaleMarker);

					for (j=0; j<4; j++){
						patCand.vertices.push_back(refinedVertices.at((8-out.ori+v1-j)%4));
					}

					//find the transformation (from camera CS to pattern CS)
					patCand.getExtrinsics(patCand.size, cameraMatrix, distortions);
					foundPatterns.push_back(patCand);

				}
			}
		}
	}
  // return only marker with highest confidence
  unsigned int markListPos = 0;
    double markerConf_max = 0.;
    for (unsigned int i =0; i<foundPatterns.size(); i++){
      double tempConf = foundPatterns.at(i).confidence;
      if( tempConf > markerConf_max){
        i = markListPos;
        markerConf_max = tempConf;
      }
    }
    if(foundPatterns.size()>=1){
      foundPatterns.at(markListPos).draw2D( frame, cameraMatrix, distortions);
      position = foundPatterns.at(markListPos).position;
      return 1;
    }else
      return 0;

}


void markerDetector::convertAndBinarize(const Mat& src, Mat& dst1, Mat& dst2, int thresh_mode)
{

	//dst1: binary image
	//dst2: grayscale image

	if (src.channels()==3){
		cvtColor(src, dst2, CV_BGR2GRAY);
	}
	else {
		src.copyTo(dst2);
	}
	
	if (thresh_mode == 1){
		threshold(dst2, dst1, thresh1, 255, CV_THRESH_BINARY_INV);
	}
	else {
		adaptiveThreshold( dst2, dst1, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, block_size, thresh2);
	}

	dilate( dst1, dst1, Mat());
}


void markerDetector::normalizePattern(const Mat& src, const Point2f roiPoints[], Rect& rec, Mat& dst)
{
	

	//compute the homography
	Mat Homo(3,3,CV_32F);
	Homo = getPerspectiveTransform( roiPoints, norm2DPts);
	
	Mat subImg = src(cv::Range(rec.y, rec.y+rec.height), cv::Range(rec.x, rec.x+rec.width));

	//warp the input based on the homography model to get the normalized ROI
	warpPerspective( subImg, dst, Homo, Size(dst.cols, dst.rows));

}

int markerDetector::identifyPattern(const Mat& src, std::vector<cv::Mat>& loadedPatterns, patInfo& info)
{
	if (loadedPatterns.size()<1){
		cout << "No loaded pattern" << endl;
		return -1;
	}

	unsigned int j;
	int i;
	double tempsim;
	double N = (double)(normSize*normSize/4);
	double nom, den;

	
	Scalar mean_ext, std_ext, mean_int, std_int;
  // Calculates a mean and standard deviation of array elements
	meanStdDev(src, mean_ext, std_ext, patMask);
	meanStdDev(src,mean_int, std_int, patMaskInt);

	if ((mean_ext.val[0]>mean_int.val[0]))
		return -1;

	Mat inter = src(cv::Range(normSize/4,3*normSize/4), cv::Range(normSize/4,3*normSize/4));
	double normSrcSq = pow(norm(inter),2);


	//zero_mean_mode;
	int zero_mean_mode = 1;
	
	//use correlation coefficient as a robust similarity measure
	info.maxCor = -1.0;
	for (j=0; j<(loadedPatterns.size()/4); j++){
		for(i=0; i<4; i++){
			
			double const nnn = pow(norm(loadedPatterns.at(j*4+i)),2);

			if (zero_mean_mode ==1){

				double const mmm = mean(loadedPatterns.at(j*4+i)).val[0];
			
				nom = inter.dot(loadedPatterns.at(j*4+i)) - (N*mean_int.val[0]*mmm);
				den = sqrt( (normSrcSq - (N*mean_int.val[0]*mean_int.val[0]) ) * (nnn - (N*mmm*mmm) ) );
				tempsim = nom/den;
			}
			else 
			{
			tempsim = inter.dot(loadedPatterns.at(j*4+i))/(sqrt(normSrcSq*nnn));
			}

			if(tempsim>info.maxCor){
				info.maxCor = tempsim;
				info.index = j+1;
				info.ori = i;
			}
		}
	}

	if (info.maxCor>confThreshold)
		return 1;
	else
		return 0;

}

