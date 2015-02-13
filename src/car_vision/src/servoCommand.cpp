#include "servoCommand.h"
#define CAR_CONTROL_MESSAGE_LENGTH 20

//default constructor servo id to zero
servoCommand::servoCommand(bool car){
  // Initial servo/motor parameter and communication
  _servoDegree = _steering_zero;
  _servoSpeed = _motor_zero;
  _minDist = 0;
  _maxDist = 25.;
  _cartox = car;
  if( _cartox ){
    initCartox();
  }else{
    initRcCar();
  }
}

/*
 * Get position (degree value) of servo
 * This function gets the PWM value of duty cycle of the
 * class servoControl and returns at the end only a degree value,
 */
float servoCommand::getServoPosDeg(){
  return _servoDegree;
}
/*
 * Move forward or backward
 * - as we have no incremental sensor we can only set a speed value
 *  and the servoControl class will calculate this value in some
 *  PWM signal
 * - this means the speed depends on the distance to the obstacle
 *    in front
 */
void servoCommand::move(float distance){
  char arr[CAR_CONTROL_MESSAGE_LENGTH + 1];
  if(!_cartox){
    // checking if min and max dist is set
    if(_minDist == 0 && _maxDist == 0){
      std::cout << "[WARNING] - servoCommand: _minDist and _maxDist is not set" << std::endl;
    }

    if(distance > _maxDist){ // set max motor speed
      _servoSpeed = _motor_max;
    }else if (distance >= _minDist && distance <= _maxDist){
      _servoSpeed = calcPWMFromSpeed(distance);
    }else { // turn motor off
      _servoSpeed = _motor_zero;
    }

    setDutyMotor(_servoSpeed); // set calculated motor speed
  }else{
    /*
     * Set car velocity
     */
    if(_degree!=0){
      /*
       * TODO: Calculation has to be adapt to car model -> not tested yet
       */
      float radius = MODEL_LENGTH/(cos(90-_degree));
      float velocity_right = (CARTOX_MAX_SPEED * distance / _maxDist) * (1 + MODEL_WIDTH/(2*radius));
      float velocity_left =  (CARTOX_MAX_SPEED * distance / _maxDist) * (1 - MODEL_WIDTH/(2*radius));
        
      int velocity_right_int = ((int) velocity_right >= 200) ? 199 : (int) velocity_right;
      int velocity_left_int = ((int) velocity_left >= 200) ? 199 : (int) velocity_left;
        
     /* The max velocity according to the Control Core is 200... so we saturate the greater values to 200. In line following, there is no chance of negative values, but according to the CCarControl Message Spec, it is a legal value. */
      
      /*_transferArr[12] = 0;
      _transferArr[13] = (char)(int) velocity_left; //front left
      _transferArr[13] = 100;
      _transferArr[14] = 0;
      _transferArr[15] = (char)(int) velocity_right; //front right
      _transferArr[15] = 100;
      _transferArr[16] = 0;
      _transferArr[17] = (char)(int) velocity_left; //back left
      _transferArr[17] = 100;
      _transferArr[18] = 0;
      _transferArr[19] = (char)(int) velocity_right; //back right
      _transferArr[19] = 100;*/
        
     /* The above block of code, and _transferArr gets corrupted for some reason when it is sent, so we stop using that and manually init a CCarControlMessage in a one liner */

      char arr[CAR_CONTROL_MESSAGE_LENGTH + 1] = {'C', 'A', 'R', 'P', 0x00, 0x01, 0x00, 0x0c, '0', 0x0c, 0x00, 0x00, 0x00, velocity_left_int, 0x00, velocity_left_int, 0x00, velocity_right_int, 0x00, velocity_right_int, '\0'};
    }
    else{
      char vel = (char)CARTOX_MAX_SPEED * distance / _maxDist;
        
      /*_transferArr[12] = 0;
      _transferArr[13] = vel;
      _transferArr[13] = 100;
      _transferArr[14] = 0;
      _transferArr[15] = vel;
      _transferArr[15] = 100;
      _transferArr[16] = 0;
      _transferArr[17] = vel;
      _transferArr[17] = 100;
      _transferArr[18] = 0;
      _transferArr[19] = vel;
      _transferArr[19] = 100;*/

      vel = (vel >= 200) ? 199 : vel; //scaling down

      char arr[CAR_CONTROL_MESSAGE_LENGTH + 1] = {'C', 'A', 'R', 'P', 0x00, 0x01, 0x00, 0x0c, '0', 0x0c, 0x00, 0x00, 0x00, vel, 0x00, vel, 0x00, vel, 0x00, vel, '\0'};
    }

    //We print the exact values that we send for debugging purposes
      
    for(int h = 0; h < CAR_CONTROL_MESSAGE_LENGTH + 1; h++) {
        printf("%d ", arr[h]);
    }
      
    printf("\nRight side wheel speed = %d\n", arr[19]);
    printf("Left side wheel speed = %d\n", arr[13]);
    sleep(1); //sleep for a second; please do not bombard the cyclone board.. as it needs time to process such messages
      
    //please do not use strlen(arr) here as there are 0s in the array and thus the len will be 4
      
    if( send(_sockHandle, arr, CAR_CONTROL_MESSAGE_LENGTH, 0) < 0)
    {
        puts("Send failed\n");
    }
      
    puts("Data Send\n");
  }
}
/*
 * Turn left or right in degree
 * right = + degree
 * left = - degree
 */
void servoCommand::turn(float degree){
  if(!_cartox){
    float range = 0.5;
    int tmpDeg = 0;
    if(degree <= 0+range && degree >=0-range){ 
      _servoDegree = _steering_zero;
    } else if (degree>0+range){
      _servoDegree = calcPWMFromDegree(degree)  ;
    } else if (degree<0-range){
      _servoDegree = calcPWMFromDegree(degree);
    }

    setDutyServo(_servoDegree);
  }else{
    _degree = degree;
  }
}

/*
 * Calculate PWM value of degree value to change the
 * steering angle
 *  - max steering angle of the car is 15°
 *    -> 15°_right = 2000000
 *    -> 15°_left  = 1100000 -> 1000000
 *    -> 0° = 1500000
 *     = 1° = 500000/15
 */
int servoCommand::calcPWMFromDegree(float degree){
  int pwm = _steering_zero;
  if(degree>0){
    // turn right
    pwm = pwm+(int)(degree*(500000/15));
    // constrains pwm signal to max steering angle
    if(pwm > _steeringRight_max)
      pwm = _steeringRight_max;

  }else{
    // turn left
    pwm = pwm+(int)(degree*(500000/15));
    // constrains pwm signal to min steering angle
    if(pwm < _steeringLeft_max)
      pwm = _steeringLeft_max;

  }
  return pwm;
}

/*
 * Calculate PWM value of speed input to move the car
 * -> this is a extra function as it is only compatible
 *    whith this car
 * -> !!! ATENTION -> this function restricts that the
 *  motor can not move backwards pwm < _motor_zero = _motor_zero
 */
int servoCommand::calcPWMFromSpeed(float distance){

  int pwm = _motor_min;
  pwm = pwm + int(floor(distance*(_motor_max-_motor_min)/(_maxDist-_minDist)));

  // restrict pwm output to max and min value
  if(pwm > _motor_max){
    pwm = _motor_max;
  }else if(pwm < _motor_zero){
    pwm = _motor_zero;
  }

  return pwm;
}

/*
 * Set min and max distance to initialize motor
 */
void servoCommand::setMinMaxDistance(int min, int max){
 _minDist = min;
 _maxDist = max;
 _motorRange = _maxDist-_minDist; // set motor range
}

