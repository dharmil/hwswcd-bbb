#include "servoControl.h"
#include "ros/ros.h"

// path to set pwm signals
#define PWM8_13_PATH "/sys/devices/ocp.3/pwm_test_P8_13.11/"
#define PWM9_14_PATH "/sys/devices/ocp.3/pwm_test_P9_14.12/"
/*
 * _period, _steering_zero, _motor_zero are constant variables
 * as they should not be changed during the program. These variables
 * work only for this car and have to be changed for other cars
 */
servoControl::servoControl()
  :_period(20000000), _steering_zero(1500000), _motor_zero(1200000){
    /*
     * Set global parameters
     */
    // Set global pwm values of steering angle
    _steeringLeft_max = 1000000; // furthest left steering position
    _steeringRight_max = 2000000; // furthest right steering position
    _steeringVal = _steering_zero; // set steering servo to zero position
    // Set motor max/min value
    _motor_max = 1300000; // restricted maximum motor velocity
    _motor_min = 1280000; // minimum motor velocity -> car is moving
    _motorVal = _motor_zero; // set current motor value to zero velocity

    _polarity = 0;
    _run = 1;
    
    // construct communication message
    _transferArr[0] = 'C';
    _transferArr[1] = 'A';
    _transferArr[2] = 'R';
    _transferArr[3] = 'P';
    _transferArr[4] = 0;
    _transferArr[5] = 0x1;
    _transferArr[6] = 0;
    _transferArr[7] = 0x8;
    _transferArr[8] = 0x30;
    _transferArr[9] = 0x8;
    _transferArr[10] = 0;
    _transferArr[11] = 0;
    _transferArr[20] = '\0';
  }

servoControl::~servoControl(){
  close(_sockHandle);
}
/*
 * Init function
 */
void servoControl::initCartox(){
  cout << "[servoControl]: Initializing cartox car..." << endl;
  /*
   * Socket initialization 
   */

  struct sockaddr_in sock_in, remoteSocketInfo;
  struct hostent *hPtr;
  // Create socket
  if((_sockHandle = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    puts("Socket opening Error");
    exit(1);
  }

  printf("this printf is in honour of matthias for finding a really nasty bug\n");
  
  sock_in.sin_family = AF_INET;
  sock_in.sin_addr.s_addr = htonl(IP_LP); // translate long integer to network byte order
  sock_in.sin_port = htons(PORT_LP); // set port number
  int yes=1;
  if (setsockopt(_sockHandle, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1) {
    perror("setsockopt");
        exit(1);
  }
  // bind socket to a local socket adress
  if( bind(_sockHandle, (struct sockaddr *) &sock_in, sizeof(struct sockaddr_in)) < 0)
  {
    close(_sockHandle);
    puts("Binding Error");
    exit(1);
  }
  remoteSocketInfo.sin_family = AF_INET;
  remoteSocketInfo.sin_addr.s_addr = htonl(IP_BBONE);
  remoteSocketInfo.sin_port = htons(PORT_BBONE);
  // load system information for remote socket server into data socket structure
  //memcpy((char *)&remoteSocketInfo.sin_addr, hPtr->h_addr, hPtr->h_length);
  remoteSocketInfo.sin_family = AF_INET;
  remoteSocketInfo.sin_port = htons(PORT_BBONE);      // Set port number
  // connet to remote adress

//this historic code was written 12-Nov-2014 on the day of Matthias's birthday and Rosetta probe landing on the commet
while(ros::ok())
{ 
  if(connect(_sockHandle, (struct sockaddr *)&remoteSocketInfo, sizeof(sockaddr_in)) < 0)
  {
    cout << "Connection Error Unable to connect" << endl;
    //close(_sockHandle);
    //exit(1);
  }
  else break;
}
cout<<"connection established!! go home you are drunk!!"<<endl;
//end of historic code, peace out
}
void servoControl::initRcCar(){
  cout << "[servoControl]: Initializing rc_car..." << endl;
  // write parameters to file
  std::stringstream str_period;
  str_period << PWM8_13_PATH << "period"; 
  _ofstrFile_period.open(str_period.str().c_str());
  if(_ofstrFile_period.is_open()){
    _ofstrFile_period << _period << endl;
    _ofstrFile_period.close();
  }else{
    cout << "[servoControl]: Could not open period file" << endl;
  }

  std::stringstream str_polarity;
  str_polarity << PWM8_13_PATH << "polarity"; 
  _ofstrFile_polarity.open(str_polarity.str().c_str());
  if(_ofstrFile_polarity.is_open()){
    _ofstrFile_polarity << _polarity << endl;
    _ofstrFile_polarity.close();
  }else{
    cout << "[servoControl]: Could not open polarity file" << endl;
  }

  std::stringstream str_run;
  str_run << PWM8_13_PATH << "run"; 
  _ofstrFile_run.open(str_run.str().c_str());
  if(_ofstrFile_run.is_open()){
    _ofstrFile_run << _run << endl;
    _ofstrFile_run.close();
  }else{
    cout << "[servoControl]: Could not open run file" << endl;
  } 
  //set stringstream of duty cycle
  std::stringstream str_duty;
  str_duty << PWM8_13_PATH << "duty"; 
  _ofstrFile_duty.open(str_duty.str().c_str());
  if(_ofstrFile_duty.is_open()){
    _ofstrFile_duty << _steering_zero << endl;
    _ofstrFile_duty.close();
  }else{
    cout << "[servoControl]: Could not open duty file" << endl;
  }

  /*
   * Motor -> write parameters to file
   * -------------------------------------------------------
   */
  // write parameters to file
  std::stringstream str_period_m;
  str_period_m << PWM9_14_PATH << "period"; 
  _ofstrFile_period_m.open(str_period_m.str().c_str());
  if(_ofstrFile_period_m.is_open()){
    _ofstrFile_period_m << _period << endl;
    _ofstrFile_period_m.close();
  }else{
    cout << "[servoControl]: Could not open period file" << endl;
  }

  std::stringstream str_polarity_m;
  //str_polarity.str("");
  //str_period.clear();
  str_polarity_m << PWM9_14_PATH << "polarity"; 
  _ofstrFile_polarity_m.open(str_polarity_m.str().c_str());
  if(_ofstrFile_polarity_m.is_open()){
    _ofstrFile_polarity_m << _polarity << endl;
    _ofstrFile_polarity_m.close();
  }else{
    cout << "[servoControl]: Could not open polarity file" << endl;
  }

  std::stringstream str_run_m;
  str_run_m << PWM9_14_PATH << "run"; 
  _ofstrFile_run_m.open(str_run_m.str().c_str());
  if(_ofstrFile_run_m.is_open()){
    _ofstrFile_run_m << _run << endl;
    _ofstrFile_run_m.close();
  }else{
    cout << "[servoControl]: Could not open run file" << endl;
  } 
  //set stringstream of duty cycle
  std::stringstream str_duty_m;
  str_duty_m << PWM9_14_PATH << "duty"; 
  _ofstrFile_duty_m.open(str_duty_m.str().c_str());
  if(_ofstrFile_duty_m.is_open()){
    _ofstrFile_duty_m << _motor_zero << endl;
    _ofstrFile_duty_m.close();
  }else{
    cout << "[servoControl]: Could not open duty file" << endl;
  }
}

/*
 * Set duty cycle: write current duty value to file
 */
void servoControl::setDutyServo(int duty){

  //std::cout << "duty_servo: " << duty << std::endl; 
  std::stringstream str_duty;
  str_duty << PWM8_13_PATH << "duty"; 
  _ofstrFile_duty.open(str_duty.str().c_str());
  if(_ofstrFile_duty.is_open()){
    _ofstrFile_duty << duty << endl;
    _ofstrFile_duty.close();
  }else{
    cout << "[servoControl]: Could not open duty file" << endl;
  }
}

/*
 * Set duty cycle: write current duty value to file
 */
void servoControl::setDutyMotor(int duty){
  std::stringstream str_duty_m;
  str_duty_m << PWM9_14_PATH << "duty"; 
  _ofstrFile_duty_m.open(str_duty_m.str().c_str());
  if(_ofstrFile_duty_m.is_open()){
    _ofstrFile_duty_m << duty << endl;
    _ofstrFile_duty_m.close();
  }else{
    cout << "[servoControl]: Could not open duty file" << endl;
  }
}

/*
 * Get value of duty cycle (servo)
 */
int servoControl::getDutyServo(){
  return _dutyServo;
}

/*
 * Get value of duty cycle (motor)
 */
int servoControl::getDutyMotor(){
  return _dutyMotor;
}
