#include <Pololu3piPlus32U4.h>
#include "Distance_pid_controller.h"
using namespace Pololu3piPlus32U4;

Distance_pid_controller::Distance_pid_controller(int kp, int ki, int kd, double minOutput, double maxOutput) {
  _kp = kp;
  _ki = ki;
	_kd = kd;
  _dt = 0;
  _CummulError = 0;
  _maxOutput = 25;
	_minOutput = -25;
	_error = 0.0;
  _lastError = 0.0;
	_output = 0.0;
	_clampOut = 0.0;
	_previousTime = 0.0;
	_currentTime = millis();
  _integral = 0;
}

double Distance_pid_controller::update(double value, double target_value){
  // calc error
  _error = value - target_value;
  // adjust cumulative error
  _CummulError += _error;

  // print for testing
  // Serial.print("Error: ");
  // Serial.println(_error);
  
  // get current time
  _currentTime = millis();
  // calculate the change in time in seconds
  _dt = (_currentTime - _previousTime) / 1000.0; 
  
  // initialize variables
  double p, i, d;
  
  // derivative = kp ratio * error
  d = _kp*_error;  

  if (_previousTime!=0.0){ // P component + D component
	  p = _kd*(_error - _lastError)/(_currentTime-_previousTime);
  }
  else{ // at the begining, we only have P component.
    p = 0.0;
  }

  // integral = cumulative errors * change in time
  _integral = _CummulError * _dt;

  // constrain the output
  _integral = constrain(_integral, _minOutput, _maxOutput);

  // i = ki ratio * integral
  i = _ki * _integral;
  
  // add p i and d values
  _output = p + i + d ;
  //constrain output
  _clampOut = constrain(_output, _minOutput, _maxOutput);
  
  //update time and error
  _previousTime = _currentTime;
  _lastError = _error;

  return _clampOut;
}
