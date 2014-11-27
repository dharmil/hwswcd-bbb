#include "markerTracking.h"

markerTracking::markerTracking(){
  iThreshold = 50;
  bFirstStripe = true;
  bFirstMarker = true;
  SliderPos = 100;
  ThresholdSliderPos = iThreshold;
  //markerPosition.x = 0.; markerPosition.y = 0.; markerPosition.z = 0.;

  memStorage = cvCreateMemStorage();
}


int markerTracking::SubPixelAccuracy( const Mat &pSrc, const Point2f &p ){
  int x = int( floorf(p.x) );
  int y = int( floorf(p.y) );

  if( x < 0 || x >= pSrc.cols - 1 || y < 0 || y >= pSrc.rows - 1){
    return 127;
  }

  int dx = int( 256 * ( p.x - floorf(p.x) ) );
  int dy = int( 256 * ( p.y - floorf(p.y) ) );

  unsigned char* i = (unsigned char*)( ( pSrc.data + y * pSrc.step ) + x );
  int m = i[0] + ( ( dx * ( i[1] - i[0] ) ) >> 8 );
  i += pSrc.step;
  int n = i[0] + ( ( dx * ( i[1] - i[0] ) ) >> 8 );

  return m + ( (dy * (n - m) ) >> 8 );
}


bool markerTracking::processMarkerTrackingColor(Mat &mOriginalImage, Point3f &markerPosition){
  bool foundMarker = false;
  Mat mThresholdImage, mGrayImage;
  //cvtColor( mOriginalImage, mGrayImage, CV_BGR2GRAY );

  cvtColor( mOriginalImage, mGrayImage, CV_RGB2HSV);
  //inRange(mGrayImage,cv::Scalar(85,150,131),cv::Scalar(119,201,255), mThresholdImage); //red
  //inRange(mGrayImage,cv::Scalar(38,37,93),cv::Scalar(100,255,255), mThresholdImage); //green
  inRange(mGrayImage,cv::Scalar(42,30,133),cv::Scalar(86,154,234), mThresholdImage); //green
  //imshow("hsv", mThresholdImage);
  //threshold( mGrayImage, mThresholdImage, SliderPos, 255, CV_THRESH_BINARY );
  //adaptiveThreshold( mGrayImage, mThresholdImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 33, 5);

  CvSeq* sContour;
  CvMat mThresholdImage_(mThresholdImage);

  cvFindContours( &mThresholdImage_, memStorage, &sContour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );

  //		double olyArea = cvContourArea(sContour, CV_WHOLE_SEQ);

  //		if( PolyArea <= 5000 ){
  //			continue;
  //		}

  for( ; sContour; sContour = sContour -> h_next ){
    CvSeq* sRectangleCandidate = cvApproxPoly( sContour, sizeof(CvContour), memStorage, CV_POLY_APPROX_DP, cvContourPerimeter(sContour) * 0.02, 0 );

    if( sRectangleCandidate -> total != 4 ){
      continue;
    }

    Mat mRectangleCandidate = cvarrToMat(sRectangleCandidate);
    Rect RectangleWinner = boundingRect(mRectangleCandidate);
    if( RectangleWinner.height < 10 || RectangleWinner.width < 10 || RectangleWinner.height > mThresholdImage.rows - 10 || RectangleWinner.width > mThresholdImage.cols - 10 ){
      continue;
    }

    const Point *Rectangle = ( const Point* ) mRectangleCandidate.data;
    int iVertex = mRectangleCandidate.rows;

    CvSeq* sPolyContour = cvApproxPoly( sContour, sizeof(CvContour), memStorage, CV_POLY_APPROX_DP, cvContourPerimeter(sContour) * 0.02, 0 );
    CvMat* mPolyContour = (CvMat*)sPolyContour;
    CvMoments Moment;
    cvMoments( mPolyContour, &Moment, 0 );
    float fM = cvGetSpatialMoment( &Moment, 0, 0 );
    float fX = cvGetSpatialMoment( &Moment, 1, 0 ) / fM;
    float fY = cvGetSpatialMoment( &Moment, 0, 1 ) / fM;
    const float fStandardDistance = 85.0;
    const float fStandardHeight = 60.0;
    const float fZ = fStandardDistance - fStandardHeight * RectangleWinner.height/fStandardDistance; 
    //cout << endl << " X = " << fX << " Y = " << fY << " Z = " << fZ << endl;

    /*
     * TODO: wird hier der X,Y Wert der Funktion berechnet?
     * -> wo wird der z Wert berechnet
     */ 
    markerPosition.x = fX; // current marker x position
    markerPosition.y = fY; // current marker y position
    markerPosition.z = fZ;
    foundMarker = true;

    polylines( mOriginalImage, &Rectangle, &iVertex, 1, TRUE, CV_RGB( 0, 0, 255), 5, CV_AA, 0 );

    float Line[16];
    Mat mLine( Size( 4, 4 ), CV_32F, Line );

    for( int i = 0; i < 4; i++ ){
      circle( mOriginalImage, Rectangle[i], 3, CV_RGB( 255, 0, 0 ), -1 );

      double dx = (double)( Rectangle[ ( i + 1 ) % 4 ].x - Rectangle[i].x ) / 7.0;
      double dy = (double)( Rectangle[ ( i + 1 ) % 4 ].y - Rectangle[i].y ) / 7.0;

      int StripeLength = (int)( 0.8 * sqrt( dx * dx + dy * dy ) );
      if( StripeLength < 5 ){
        StripeLength = 5;
      }

      StripeLength |= 1;

      int StripeEnd = StripeLength >> 1;
      int StripeStart = - StripeEnd;

      Size StripeSize;
      StripeSize.height = StripeLength;
      StripeSize.width = 3;

      Point2f StripeVectorX;
      Point2f StripeVectorY;

      StripeVectorX.x = dx / sqrt( dx * dx + dy * dy );
      StripeVectorX.y = dy / sqrt( dx * dx + dy * dy );
      StripeVectorY.x = - StripeVectorX.y;
      StripeVectorY.y = - StripeVectorX.x;

      Mat mStripe( StripeSize, CV_8UC1 );

      Point2f Point[6];

      for( int j = 1; j < 7; j++ ){
        double px = (double)Rectangle[i].x + (double)j*dx;
        double py = (double)Rectangle[i].y + (double)j*dy;

        CvPoint SectionPoint;
        SectionPoint.x = (int)px;
        SectionPoint.y = (int)py;
        circle( mThresholdImage, SectionPoint, 2, CV_RGB( 0, 255, 0 ), -1 );

        for( int u = -1; u <= 1; u++ ){
          for( int v = StripeStart; v <= StripeEnd; v++ ){
            Point2f SubPixel;
            SubPixel.x = (double)SectionPoint.x + ( (double)u * StripeVectorX.x ) + ( (double)v * StripeVectorX.x );
            SubPixel.y = (double)SectionPoint.y + ( (double)u * StripeVectorX.y ) + ( (double)v * StripeVectorY.y );

            CvPoint SubSectionPoint;
            SubSectionPoint.x = (int)SubPixel.x;
            SubSectionPoint.y = (int)SubPixel.y;

            if(bFirstStripe){
              circle( mOriginalImage, SubSectionPoint, 1, CV_RGB( 0, 255, 255 ), -1 );
            }
            else{
              circle( mOriginalImage, SubSectionPoint, 1, CV_RGB( 255, 255, 0 ), -1 );
            }

            int Pixel = SubPixelAccuracy(mGrayImage, SubPixel);

            int s = v + ( StripeLength >> 1 );
            int t = u + 1;

            mStripe.at<uchar>( s, t ) = (uchar)Pixel;
          }

        }

        vector<double> Sobel( StripeLength - 2 );

        for( int i = 1; i < ( StripeLength - 1 ); i++ ){
          unsigned char* StripePosition = &( mStripe.at<uchar>(i - 1, 0) );

          double FirstSobelCol = -StripePosition[0] - 2 * StripePosition[1] - StripePosition[2];
          StripePosition += 2 * mStripe.step;
          double ThirdSobelCol = StripePosition[0] + 2 * StripePosition[1] + StripePosition[2];

          Sobel[i - 1] = FirstSobelCol + ThirdSobelCol;
        }

        double MaxValue = -1;
        int MaxIndex = 0;

        for( int i = 0; i < StripeLength - 2; i++ ){
          if( Sobel[i] > MaxValue ){
            MaxValue = Sobel[i];
            MaxIndex = i;
          }
        }

        double y0 = ( MaxIndex <= 0 ) ? 0 : Sobel[MaxIndex - 1];
        double y1 = Sobel[MaxIndex];
        double y2 = ( MaxIndex >= StripeLength - 3 ) ? 0 : Sobel[MaxIndex + 1];

        double PointAdjustment = ( y2 - y0 ) / (4 * y1 - 2 * y0 - 2 * y2);

        int MaxIndexShift = MaxIndex - ( StripeLength >> 1 );
        Point2f EdgeMiddle;
        EdgeMiddle.x = (double)SectionPoint.x + ( ( (double)MaxIndexShift + PointAdjustment ) * StripeVectorY.x );
        EdgeMiddle.y = (double)SectionPoint.y + ( ( (double)MaxIndexShift + PointAdjustment ) * StripeVectorY.y );

        CvPoint CenterPoint;
        CenterPoint.x = (int)EdgeMiddle.x;
        CenterPoint.y = (int)EdgeMiddle.y;
        circle( mOriginalImage, CenterPoint, 1, CV_RGB( 0, 255, 0 ), -1 );

        Point[ j - 1 ].x = EdgeMiddle.x;
        Point[ j - 1 ].y = EdgeMiddle.y;

        if(bFirstStripe){
          //Mat mStripeSample;
          //resize( mStripe, mStripeSample, Size( 100, 300 ) );
          //imshow( "Stripe Video", mStripeSample );
          bFirstStripe = FALSE;
        }

      }

      Mat LineFitting( Size( 1, 6 ), CV_32FC2, Point );
      fitLine( LineFitting, mLine.row(i), CV_DIST_L2, 0, 0.01, 0.01 );

      CvPoint LinePointM;
      LinePointM.x = (int)Line[ 4 * i + 2 ] - (int)( 50.0 * Line[ 4 * i + 0 ] );
      LinePointM.y = (int)Line[ 4 * i + 3 ] - (int)( 50.0 * Line[ 4 * i + 1 ] );
      CvPoint LinePointP;
      LinePointP.x = (int)Line[ 4 * i + 2 ] + (int)( 50.0 * Line[ 4 * i + 0 ] );
      LinePointP.y = (int)Line[ 4 * i + 3 ] + (int)( 50.0 * Line[ 4 * i + 1 ] );

      line( mOriginalImage, LinePointM, LinePointP, CV_RGB( 0, 0, 255 ), 1, 8, 0 );
    }

    Point2f Corner[4];

    for( int i = 0; i < 4; i++ ){
      int j = ( i + 1 ) % 4;

      double x0 = Line[ 4 * i + 2 ];
      double x1 = Line[ 4 * j + 2 ];
      double y0 = Line[ 4 * i + 3 ];
      double y1 = Line[ 4 * j + 3 ];
      double u0 = Line[ 4 * i + 0 ];
      double u1 = Line[ 4 * j + 0 ];
      double v0 = Line[ 4 * i + 1 ];
      double v1 = Line[ 4 * j + 1 ];

      double a = x1 * u0 * v1 - y1 * u0 * u1 - x0 * u1 * v0 + y0 * u0 * u1;
      double b = - x1 * v0 * v1 + y0 * u0 * v1 + x1 * v0 * v1 - y1 * v0 * u1;
      double c = v1 * u0 - v0 * u1;

      if( fabs(c) < 0.001 ){
        continue;
      }

      a /= c;
      b /= c;

      Corner[i].x = a;
      Corner[i].y = b;
      CvPoint CornerPoint;
      CornerPoint.x = (int)Corner[i].x;
      CornerPoint.y = (int)Corner[i].y;

      circle( mOriginalImage, CornerPoint, 5, CV_RGB( 255, 0, 0 ), -1 );
    }

    Point2f TargetCorner[4];
    TargetCorner[0].x = -0.5;
    TargetCorner[0].y = -0.5;
    TargetCorner[1].x = 5.5;
    TargetCorner[1].y = -0.5;
    TargetCorner[2].x = 5.5;
    TargetCorner[2].y = 5.5;
    TargetCorner[3].x = -0.5;
    TargetCorner[3].y = 5.5;

    Mat mProjection( Size(3, 3), CV_32FC1 );
    mProjection = getPerspectiveTransform( Corner, TargetCorner );

    Mat mMarker( Size( 6, 6 ), CV_8UC1 );
    warpPerspective( mGrayImage, mMarker, mProjection, Size( 6, 6 ) );

    threshold( mMarker, mMarker, iThreshold, 255, CV_THRESH_BINARY );

    int iCodeValue = 0;
    for( int i = 0; i < 6; i++ ){
      int Pixel1 = mMarker.at<uchar>( 0, i );
      int Pixel2 = mMarker.at<uchar>( 5, i );
      int Pixel3 = mMarker.at<uchar>( i, 0 );
      int Pixel4 = mMarker.at<uchar>( i, 5 );

      if( ( Pixel1 > 0 ) || ( Pixel2 > 0 ) || ( Pixel3 > 0 ) || ( Pixel4 > 0 ) ){
        iCodeValue = -1;
        break;
      }
    }

    if( iCodeValue < 0 ){
      continue;
    }

    int MarkerPoint[4][4];
    for( int i = 0; i < 4; i++ ){
      for ( int j = 0; j < 4; j++ ){
        MarkerPoint[i][j] = mMarker.at<uchar>( i + 1, j + 1 );
        MarkerPoint[i][j] = ( MarkerPoint[i][j] == 0 ) ? 1 : 0;
      }
    }

    int Code[4];
    Code[0] = 0;
    Code[1] = 0;
    Code[2] = 0;
    Code[3] = 0;
    for( int i = 0; i < 16; i++ ){
      int CodeRow = i >> 2;
      int CodeCol = i % 4;

      Code[0] <<= 1;
      Code[0] |= MarkerPoint[CodeRow][CodeCol];
      Code[1] <<= 1;
      Code[1] |= MarkerPoint[ 3 - CodeCol ][CodeRow];
      Code[2] <<= 1;
      Code[2] |= MarkerPoint[ 3 - CodeRow ][ 3 - CodeCol ];
      Code[3] <<= 1;
      Code[3] |= MarkerPoint[CodeCol][ 3 - CodeRow ];
    }

    if( (Code[0] == 0) || (Code[0] == 0xffff) ){
      continue;
    }

    iCodeValue = Code[0];
    for( int i = 1; i < 4; i++ ){
      if( Code[i] < iCodeValue ){
        iCodeValue = Code[i];
      }
    }

  }

  bFirstStripe = true;
  bFirstMarker = true;

  cvClearMemStorage(memStorage);

  return foundMarker;
}

bool markerTracking::processMarkerTracking(Mat &mOriginalImage, Point3f &markerPosition){
  bool foundMarker = false;
  cvtColor( mOriginalImage, mGrayImage, CV_BGR2GRAY );

  //threshold( mGrayImage, mThresholdImage, SliderPos, 255, CV_THRESH_BINARY );
  adaptiveThreshold( mGrayImage, mThresholdImage, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 33, 5);
  CvSeq* sContour;
  CvMat mThresholdImage_(mThresholdImage);
  //cvtColor(mOriginalImage, mGrayImage, CV_RGB2HSV);
  //inRange(mGrayImage,Scalar(80,0,0),Scalar(170,255,255), mThresholdImage);
  cvFindContours( &mThresholdImage_, memStorage, &sContour, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );

  //		double PolyArea = cvContourArea(sContour, CV_WHOLE_SEQ);

  //		if( PolyArea <= 5000 ){
  //			continue;
  //		}

  for( ; sContour; sContour = sContour -> h_next ){
    CvSeq* sRectangleCandidate = cvApproxPoly( sContour, sizeof(CvContour), memStorage, CV_POLY_APPROX_DP, cvContourPerimeter(sContour) * 0.02, 0 );

    if( sRectangleCandidate -> total != 4 ){
      continue;
    }

    Mat mRectangleCandidate = cvarrToMat(sRectangleCandidate);
    Rect RectangleWinner = boundingRect(mRectangleCandidate);
    if( RectangleWinner.height < 10 || RectangleWinner.width < 10 || RectangleWinner.height > mThresholdImage.rows - 10 || RectangleWinner.width > mThresholdImage.cols - 10 ){
      continue;
    }

    const Point *Rectangle = ( const Point* ) mRectangleCandidate.data;
    int iVertex = mRectangleCandidate.rows;

    CvSeq* sPolyContour = cvApproxPoly( sContour, sizeof(CvContour), memStorage, CV_POLY_APPROX_DP, cvContourPerimeter(sContour) * 0.02, 0 );
    CvMat* mPolyContour = (CvMat*)sPolyContour;
    CvMoments Moment;
    cvMoments( mPolyContour, &Moment, 0 );
    float fM = cvGetSpatialMoment( &Moment, 0, 0 );
    float fX = cvGetSpatialMoment( &Moment, 1, 0 ) / fM;
    float fY = cvGetSpatialMoment( &Moment, 0, 1 ) / fM;
    const float fStandardDistance = 20.0;
    const float fStandardHeight = 80.0;
    const float fZ = fStandardDistance * fStandardHeight / RectangleWinner.height; 
    //cout << endl << " X = " << fX << " Y = " << fY << " Z = " << fZ << endl;

    /*
     * TODO: wird hier der X,Y Wert der Funktion berechnet?
     * -> wo wird der z Wert berechnet
     */ 
    markerPosition.x = fX; // current marker x position
    markerPosition.y = fY; // current marker y position
    markerPosition.z = fZ;

    polylines( mOriginalImage, &Rectangle, &iVertex, 1, TRUE, CV_RGB( 0, 0, 255), 5, CV_AA, 0 );

    float Line[16];
    Mat mLine( Size( 4, 4 ), CV_32F, Line );

    for( int i = 0; i < 4; i++ ){
      circle( mOriginalImage, Rectangle[i], 3, CV_RGB( 255, 0, 0 ), -1 );

      double dx = (double)( Rectangle[ ( i + 1 ) % 4 ].x - Rectangle[i].x ) / 7.0;
      double dy = (double)( Rectangle[ ( i + 1 ) % 4 ].y - Rectangle[i].y ) / 7.0;

      int StripeLength = (int)( 0.8 * sqrt( dx * dx + dy * dy ) );
      if( StripeLength < 5 ){
        StripeLength = 5;
      }

      StripeLength |= 1;

      int StripeEnd = StripeLength >> 1;
      int StripeStart = - StripeEnd;

      Size StripeSize;
      StripeSize.height = StripeLength;
      StripeSize.width = 3;

      Point2f StripeVectorX;
      Point2f StripeVectorY;

      StripeVectorX.x = dx / sqrt( dx * dx + dy * dy );
      StripeVectorX.y = dy / sqrt( dx * dx + dy * dy );
      StripeVectorY.x = - StripeVectorX.y;
      StripeVectorY.y = - StripeVectorX.x;

      Mat mStripe( StripeSize, CV_8UC1 );

      Point2f Point[6];

      for( int j = 1; j < 7; j++ ){
        double px = (double)Rectangle[i].x + (double)j*dx;
        double py = (double)Rectangle[i].y + (double)j*dy;

        CvPoint SectionPoint;
        SectionPoint.x = (int)px;
        SectionPoint.y = (int)py;
        circle( mThresholdImage, SectionPoint, 2, CV_RGB( 0, 255, 0 ), -1 );

        for( int u = -1; u <= 1; u++ ){
          for( int v = StripeStart; v <= StripeEnd; v++ ){
            Point2f SubPixel;
            SubPixel.x = (double)SectionPoint.x + ( (double)u * StripeVectorX.x ) + ( (double)v * StripeVectorX.x );
            SubPixel.y = (double)SectionPoint.y + ( (double)u * StripeVectorX.y ) + ( (double)v * StripeVectorY.y );

            CvPoint SubSectionPoint;
            SubSectionPoint.x = (int)SubPixel.x;
            SubSectionPoint.y = (int)SubPixel.y;

            if(bFirstStripe){
              circle( mOriginalImage, SubSectionPoint, 1, CV_RGB( 0, 255, 255 ), -1 );
            }
            else{
              circle( mOriginalImage, SubSectionPoint, 1, CV_RGB( 255, 255, 0 ), -1 );
            }

            int Pixel = SubPixelAccuracy(mGrayImage, SubPixel);

            int s = v + ( StripeLength >> 1 );
            int t = u + 1;

            mStripe.at<uchar>( s, t ) = (uchar)Pixel;
          }

        }

        vector<double> Sobel( StripeLength - 2 );

        for( int i = 1; i < ( StripeLength - 1 ); i++ ){
          unsigned char* StripePosition = &( mStripe.at<uchar>(i - 1, 0) );

          double FirstSobelCol = -StripePosition[0] - 2 * StripePosition[1] - StripePosition[2];
          StripePosition += 2 * mStripe.step;
          double ThirdSobelCol = StripePosition[0] + 2 * StripePosition[1] + StripePosition[2];

          Sobel[i - 1] = FirstSobelCol + ThirdSobelCol;
        }

        double MaxValue = -1;
        int MaxIndex = 0;

        for( int i = 0; i < StripeLength - 2; i++ ){
          if( Sobel[i] > MaxValue ){
            MaxValue = Sobel[i];
            MaxIndex = i;
          }
        }

        double y0 = ( MaxIndex <= 0 ) ? 0 : Sobel[MaxIndex - 1];
        double y1 = Sobel[MaxIndex];
        double y2 = ( MaxIndex >= StripeLength - 3 ) ? 0 : Sobel[MaxIndex + 1];

        double PointAdjustment = ( y2 - y0 ) / (4 * y1 - 2 * y0 - 2 * y2);

        int MaxIndexShift = MaxIndex - ( StripeLength >> 1 );
        Point2f EdgeMiddle;
        EdgeMiddle.x = (double)SectionPoint.x + ( ( (double)MaxIndexShift + PointAdjustment ) * StripeVectorY.x );
        EdgeMiddle.y = (double)SectionPoint.y + ( ( (double)MaxIndexShift + PointAdjustment ) * StripeVectorY.y );

        CvPoint CenterPoint;
        CenterPoint.x = (int)EdgeMiddle.x;
        CenterPoint.y = (int)EdgeMiddle.y;
        circle( mOriginalImage, CenterPoint, 1, CV_RGB( 0, 255, 0 ), -1 );

        Point[ j - 1 ].x = EdgeMiddle.x;
        Point[ j - 1 ].y = EdgeMiddle.y;

        if(bFirstStripe){
          //Mat mStripeSample;
          //resize( mStripe, mStripeSample, Size( 100, 300 ) );
          //imshow( "Stripe Video", mStripeSample );
          bFirstStripe = FALSE;
        }

      }

      Mat LineFitting( Size( 1, 6 ), CV_32FC2, Point );
      fitLine( LineFitting, mLine.row(i), CV_DIST_L2, 0, 0.01, 0.01 );

      CvPoint LinePointM;
      LinePointM.x = (int)Line[ 4 * i + 2 ] - (int)( 50.0 * Line[ 4 * i + 0 ] );
      LinePointM.y = (int)Line[ 4 * i + 3 ] - (int)( 50.0 * Line[ 4 * i + 1 ] );
      CvPoint LinePointP;
      LinePointP.x = (int)Line[ 4 * i + 2 ] + (int)( 50.0 * Line[ 4 * i + 0 ] );
      LinePointP.y = (int)Line[ 4 * i + 3 ] + (int)( 50.0 * Line[ 4 * i + 1 ] );

      line( mOriginalImage, LinePointM, LinePointP, CV_RGB( 0, 255, 255 ), 1, 8, 0 );
    }

    Point2f Corner[4];

    for( int i = 0; i < 4; i++ ){
      int j = ( i + 1 ) % 4;

      double x0 = Line[ 4 * i + 2 ];
      double x1 = Line[ 4 * j + 2 ];
      double y0 = Line[ 4 * i + 3 ];
      double y1 = Line[ 4 * j + 3 ];
      double u0 = Line[ 4 * i + 0 ];
      double u1 = Line[ 4 * j + 0 ];
      double v0 = Line[ 4 * i + 1 ];
      double v1 = Line[ 4 * j + 1 ];

      double a = x1 * u0 * v1 - y1 * u0 * u1 - x0 * u1 * v0 + y0 * u0 * u1;
      double b = - x1 * v0 * v1 + y0 * u0 * v1 + x1 * v0 * v1 - y1 * v0 * u1;
      double c = v1 * u0 - v0 * u1;

      if( fabs(c) < 0.001 ){
        continue;
      }

      a /= c;
      b /= c;

      Corner[i].x = a;
      Corner[i].y = b;
      CvPoint CornerPoint;
      CornerPoint.x = (int)Corner[i].x;
      CornerPoint.y = (int)Corner[i].y;

      circle( mOriginalImage, CornerPoint, 5, CV_RGB( 255, 0, 0 ), -1 );
    }

    Point2f TargetCorner[4];
    TargetCorner[0].x = -0.5;
    TargetCorner[0].y = -0.5;
    TargetCorner[1].x = 5.5;
    TargetCorner[1].y = -0.5;
    TargetCorner[2].x = 5.5;
    TargetCorner[2].y = 5.5;
    TargetCorner[3].x = -0.5;
    TargetCorner[3].y = 5.5;

    Mat mProjection( Size(3, 3), CV_32FC1 );
    mProjection = getPerspectiveTransform( Corner, TargetCorner );

    Mat mMarker( Size( 6, 6 ), CV_8UC1 );
    warpPerspective( mGrayImage, mMarker, mProjection, Size( 6, 6 ) );

    threshold( mMarker, mMarker, iThreshold, 255, CV_THRESH_BINARY );

    int iCodeValue = 0;
    for( int i = 0; i < 6; i++ ){
      int Pixel1 = mMarker.at<uchar>( 0, i );
      int Pixel2 = mMarker.at<uchar>( 5, i );
      int Pixel3 = mMarker.at<uchar>( i, 0 );
      int Pixel4 = mMarker.at<uchar>( i, 5 );

      if( ( Pixel1 > 0 ) || ( Pixel2 > 0 ) || ( Pixel3 > 0 ) || ( Pixel4 > 0 ) ){
        iCodeValue = -1;
        break;
      }
    }

    if( iCodeValue < 0 ){
      continue;
    }

    int MarkerPoint[4][4];
    for( int i = 0; i < 4; i++ ){
      for ( int j = 0; j < 4; j++ ){
        MarkerPoint[i][j] = mMarker.at<uchar>( i + 1, j + 1 );
        MarkerPoint[i][j] = ( MarkerPoint[i][j] == 0 ) ? 1 : 0;
      }
    }

    int Code[4];
    Code[0] = 0;
    Code[1] = 0;
    Code[2] = 0;
    Code[3] = 0;
    for( int i = 0; i < 16; i++ ){
      int CodeRow = i >> 2;
      int CodeCol = i % 4;

      Code[0] <<= 1;
      Code[0] |= MarkerPoint[CodeRow][CodeCol];
      Code[1] <<= 1;
      Code[1] |= MarkerPoint[ 3 - CodeCol ][CodeRow];
      Code[2] <<= 1;
      Code[2] |= MarkerPoint[ 3 - CodeRow ][ 3 - CodeCol ];
      Code[3] <<= 1;
      Code[3] |= MarkerPoint[CodeCol][ 3 - CodeRow ];
    }
    std::cout << "code: " << Code << std::endl;
    if( (Code[0] == 0) || (Code[0] == 0xffff) ){
      foundMarker = false;
      continue;
    }else {
      foundMarker = true;
    }
    iCodeValue = Code[0];
    for( int i = 1; i < 4; i++ ){
      if( Code[i] < iCodeValue ){
        iCodeValue = Code[i];
      }
    }

  }
  /*
   * if(bFirstMarker){
   *   foundMarker = true;
   * }
   */

  bFirstStripe = true;
  bFirstMarker = true;

  cvClearMemStorage(memStorage);

  return foundMarker;
}
