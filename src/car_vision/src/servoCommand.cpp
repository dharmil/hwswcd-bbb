#include "servoCommand.h"
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
      
      _transferArr[12] = (char)velocity_left; //front left
      _transferArr[13] = (char)velocity_right; //front right
      _transferArr[14] = (char)velocity_left; //back left
      _transferArr[15] = (char)velocity_right; //back right
    }
    else{
      char vel = (char)CARTOX_MAX_SPEED * distance / _maxDist;
      _transferArr[12] = vel;
      _transferArr[13] = vel;
      _transferArr[14] = vel;
      _transferArr[15] = vel;
    }
	//std::cout << "array:" << (int)_transferArr[12] << (int)_transferArr[13] << std::endl; 
    send(_sockHandle, _transferArr, 16, 0);
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

