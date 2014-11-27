#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#define IP_BBONE 0xc0a800c8 //192.168.0.200
#define PORT_LP 23
#define IP_LP 0xc0a80096 //192.168.0.150
#define PORT_BBONE 23

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>

using namespace std;

/*
 * ServoControl class is at the lowest level. Here you
 * communicate directly with the motors (set PWM signal).
 * So if the communication of the hardware changes, this
 * class has to be adapted.
 */
class servoControl {
  private:
    // servo/motor parameter
    int _dutyServo;
    int _dutyMotor;
    const int _period;
    bool _run;
    int _polarity;

    // streams
    // servo
    ofstream _ofstrFile_period;
    ofstream _ofstrFile_duty;
    ofstream _ofstrFile_run;
    ofstream _ofstrFile_polarity;
    std::stringstream _str_duty;
 
    // motor
    ofstream _ofstrFile_period_m;
    ofstream _ofstrFile_duty_m;
    ofstream _ofstrFile_run_m;
    ofstream _ofstrFile_polarity_m;
    std::stringstream _str_duty_m;

  protected:
    /*
     * Max and min servo/motor position/speed
     */
    // Servo: all values are pwm values
    int _steeringLeft_max; // max steering angle
    int _steeringRight_max; // min steering angle
    const int _steering_zero; // zero steering position
    int _steeringVal; // current steering value
    // Motor: all values are pwm values
    int _motor_max; // max motor speed
    int _motor_min; // min motor speed
    const int _motor_zero; // motor speed = 0
    int _motorVal; // current motor value

    bool _cartox;
    
    //CartoX
    char _transferArr[16];
    int _sockHandle;

    servoControl();
    ~servoControl();
    /*
     * Init function
     */
    void initCartox();
    void initRcCar();
    /*
     * Set duty cycle: write current duty value to file
     */
    // servo
    void setDutyServo(int duty);
    // motor
    void setDutyMotor(int duty);
    /*
     * Get value of duty cycle
     */
    int getDutyServo();
    /*
     * Get value of duty cycle
     */
    int getDutyMotor();

};

#endif //SERVOCONTROL_H_

