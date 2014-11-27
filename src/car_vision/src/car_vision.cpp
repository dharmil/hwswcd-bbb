// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// opencv includes
#include <opencv2/imgproc/imgproc.hpp>>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include "std_msgs/String.h"
#include <bitset>

#include "servoCommand.h"
#include "lineFinder.h"
#include "pid_controller.h"
#include "markerTracking.h"
#include "markerdetector.h"
#include "cameraparams.h"
#include "car_parameters.h"
#include "MarkerDetector.h"

/*
 * Global Parameters
 */
int frameRate = 10; // frame rate of camera
int imgSizeXhalf = 0;
bool boolLineFinder = true;

bool rc_car, cartox; // change between rc_car and CarToX car
// change modes between line detection and marker detection
// marker_version = 0 => marker detection mode II
// marker_version = 1 => marker detection mode III
int marker_version;
bool lane; 

// PID-control
float motor_ki, motor_kp, motor_kd;
float servo_ki, servo_kp, servo_kd;
float motor_speed;
bool img_stream; // camera stream
float car_distance; // minimum distance to car in front
float max_steering_angle = 15.; // maximum steering angle of the rc_car


// load marker pattern
char* filename="src/car_vision/include/num1.png";

int main(int argc, char **argv){
  /*
   * Initialization
   * ----------------------------------------------------------------
   * ----------------------------------------------------------------
   */
  std::cout << " --- Start car_alpenapollo --- " << std::endl;
  std::cout << "[main] - initializing car parameters " << std::endl;
  // get parameters of parameter file (HSCD.txt)
  bool initFile = init_var(&rc_car, &cartox, &marker_version, &lane, &motor_ki, &motor_kp, &motor_kd, &servo_ki, &servo_kp, &servo_kd, &motor_speed, &img_stream, &car_distance);
	if(!initFile)
		return -1;

  /*
   * ROS init
   * ----------------------------------------------------------------
   */
  // parameters to construct ros_messages of cv::mat
  ros::init(argc, argv, "car_vision");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Publisher image_pub_ = it_.advertise("/car_vision/output_video",1);
  cv_bridge::CvImage out_msg;
  //ros::Rate loop_rate(10);

  /*
   * Init camera parameters
   * ----------------------------------------------------------------
   */
  cv::Mat inputImg; // camera image
  // get image from camera
  cv::VideoCapture capture(0); // open video file
  int imgSizeX = 320;
  int imgSizeY = 240;
  // at the moment line detection works on Beaglbone black only
  // with a resolution of 160 x 120
  if( lane == true){
    imgSizeX = 160;
    imgSizeY = 120;
  }
  // set image size and frame rate
  capture.set(CV_CAP_PROP_FRAME_WIDTH, imgSizeX);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, imgSizeY);
  capture.set(CV_CAP_PROP_FPS, frameRate);
  // Get camera image
  if(!capture.isOpened()){ // checking if a web cam is connected
    std::cout << "[WARNING] - main: No camera" << std::endl;
    return -1;
  }else
  std::cout << " Found camera " << std::endl;
  capture.read(inputImg);
  std::cout << "Image size: " << inputImg.cols << " x " << inputImg.rows << std::endl;
  imgSizeXhalf = inputImg.rows/2; // calculate image center
  /*
   * Init other function parameters
   * ------------------------------
   */
  // init lineFinder
  // ----------------------------------------------------------------
  std::cout << "[main] - initializing line tracking " << std::endl;
  lineFinder lineDetect;
  lineDetect.setImageParameter(inputImg);
  
  // Marker tracking
  // ----------------------------------------------------------------
  // (first version)
  std::cout << "[main] - initializing marker tracking " << std::endl;
  markerTracking marker;
  // (second version)
  int switchMarker = 1;
  std::vector<cv::Mat> patternLibrary;
  std::vector<Pattern> detectedMarker;
  int patternCount = 0;

	int norm_pattern_size = 64;
	double fixed_thresh = 80; //40
	double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
	int adapt_block_size = 85;//45;//non-used with FIXED_THRESHOLD mode
	double confidenceThreshold = 0.35;
	int modeMarker = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD
  double markerSize = 80.0; // marker sizce
  // init marker class
	markerDetector markerDet( fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, modeMarker, markerSize, filename, patternCount );
  // third marker version
  MarkerDetector markerCapture;

  // PID controller
  // ----------------------------------------------------------------
  // PID parameters
  float sampleTimeSeconds = 1./(float)frameRate;
  float minOutput_a = (float)-imgSizeX/2;
  float maxOutput_a = (float)imgSizeX/2.;
  float minOutput_d = 0.; // minimum value for velocity
  float maxOutput_d = 25.;//25.; // set maximum distance to car in front = max motor speed
  float minDistance = 0.; // set minimum distance to car in front

  ePIDMode mode;
  mode = AUTOMATIC;
  ePIDDirection controllDir;
  controllDir = DIRECT;

  // PID 1: steering
  float kp_a = servo_kp; float ki_a = servo_ki; float kd_a = servo_kd;
  tPIDParam pid_angle;
  PIDInit(&pid_angle, kp_a, ki_a, kd_a, sampleTimeSeconds, minOutput_a, maxOutput_a, mode, controllDir);
  //pid_angle.setpoint = maxOutput/2.;
  pid_angle.setpoint = 0.;
  float angleOutput; // pid output
  // PID 2: velocity
  float kp_d = motor_kp; float ki_d = motor_ki; float kd_d = motor_kd;
  tPIDParam pid_distance;
  PIDInit(&pid_distance, kp_d, ki_d, kd_d, sampleTimeSeconds, minOutput_d, maxOutput_d, mode, controllDir);
  pid_distance.setpoint = minDistance;
  float distanceOutput;


  // init servo class (send commands to motors)
  // TODO: für cartox anpassen
  std::cout << "[main] - initializing motor class... " << std::endl;
  //this was not working servoCommand servo(cartox);
   servoCommand servo(cartox);
  // set range of motor
  // -> minDistance = turn motor off
  // -> maxOutput = max speed
  servo.setMinMaxDistance((int)minOutput_d+1, (int)(maxOutput_d-minDistance));
  /*
   * ---------------------
   * END - Initialization
   * ----------------------------------------------------------------
   */

  /*
   * ----------------------------------------------------------------
   * Main loop
   * ----------------------------------------------------------------
   */
  bool breakMainLoop = false; // go out of while loop
  int linePos = inputImg.cols/2; // current line position
  int imgCenter = inputImg.cols/2; // image center
  float cur_deg = 0.; // degree of turning front car wheels
  bool init = true;
  
  boolLineFinder = lane; // set 
  int countPosition = 0;
  cv::Point3f savedMarkerPosition;
  std::cout << "Start main loop" << std::endl;
  while(ros::ok() && breakMainLoop == false){
    
    // Get camera image
    if(!capture.isOpened()){ // checking if a web cam is connected
      std::cout << "[WARNING] - main: No camera" << std::endl;
    }
    capture.read(inputImg);

    /*
     * Image processing
     * --------------------------------------------------------------
     */
    // marker tracking
    // --------------------------------------
    if(!boolLineFinder){
      cv::Point3f markerPosition;
      bool foundMarker = false;
      if(marker_version == 0){
        foundMarker = markerDet.detect(inputImg, cameraMatrix, distortions, detectedMarker, markerPosition); 
        detectedMarker.clear();
      }else if( marker_version == 1){
        //foundMarker = marker.processMarkerTrackingColor(inputImg, markerPosition);
        foundMarker = markerCapture.processingFrame(inputImg, inputImg, markerPosition, true);
      }else
        continue;
   
      std::cout << "Marker position: " << markerPosition << std::endl;
      // checking
      if(markerPosition.x == 0. && markerPosition.y == 0. && markerPosition.z ==0){
       markerPosition = savedMarkerPosition;

        ++countPosition;
      }else{
        savedMarkerPosition = markerPosition;
        countPosition = 0;
      }
      /*
       * Set motor commands -> Controller
       * ------------------------------------
       */
      if(countPosition <= 5){
      // PID angle
      pid_angle.input = markerPosition.x-(float)inputImg.cols/2.;
      if(pid_angle.input < -(float)inputImg.cols/2+20.){
        pid_angle.input = -(float)inputImg.cols/2;
      }else if(pid_angle.input > (float)inputImg.cols/2-20.){
        pid_angle.input = (float)inputImg.cols/2;
      }
      PIDCompute(&pid_angle);
      angleOutput = pid_angle.output;
      cur_deg = angleOutput*(max_steering_angle/(float)imgSizeXhalf);
      
      // PID distance
      pid_distance.input = (-1)*(markerPosition.z-25.);
      PIDCompute(&pid_distance);
      distanceOutput = pid_distance.output;
     
     // set motor commands
      servo.turn(-cur_deg); // set steering angle
      servo.move(distanceOutput); // set motor velocity
      }else{
        servo.turn(0);
        servo.move(0);
      }
    }else if (boolLineFinder){
      // line Detection
      // ------------------------------------
      // TODO: At the moment, motor speed is constant -> has to be changed
      float lPos = (float)lineDetect.processLineFinder(inputImg);
      //std::cout << "linePos: " << lPos << std::endl;
      pid_angle.input = lPos-(float)inputImg.cols/2.;
      PIDCompute(&pid_angle);
      angleOutput = pid_angle.output;
      cur_deg = angleOutput*(max_steering_angle/(float)imgSizeXhalf);
      servo.turn(-cur_deg);
      servo.move(motor_speed);

    }

    /*
     * Construct "ros-image-msg" to send a image 
     */
    if(img_stream == 1){ 
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      out_msg.header.stamp = ros::Time::now();
      out_msg.header.frame_id = "image";
      out_msg.image = inputImg;

      sensor_msgs::Image img;
      out_msg.toImageMsg(img);
      image_pub_.publish(img);
      ros::spinOnce();
    }
  }
  capture.release();

}
