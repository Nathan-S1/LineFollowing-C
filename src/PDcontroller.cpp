#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

PDcontroller::PDcontroller(int kp, int kd, double minOutput, double maxOutput) {
  _kp = kp;
	_kd = kd;
  _maxOutput = maxOutput;
	_minOutput = minOutput;
	_error = 0.0;
  _lastError = 0.0;
	_output = 0.0;
	_clampOut = 0.0;
	_previousTime = 0.0;
	_currentTime = millis();
}

double PDcontroller::update(double value, double target_value){

  // calculate angle error
  _error = value - target_value;

  // print out the error for checking
  Serial.print("Error: ");
  Serial.println(_error);

  // get the current time
  _currentTime = millis();
  
  // initialize values for proportional and derivative values
  double p, d;
  
  // derivative = kp ratio times error
  d = _kp*_error;  

  // proportional value = kd * change in error / change in time
  if (_previousTime!=0.0){ // P component + D component
	  p = _kd*(_error - _lastError)/(_currentTime-_previousTime);
  }
  else{ // at the begining, we only have P component.
    p = 0.0;
  }
  
  // add p and d corrections
  _output = p + d;

  //clamp output
  _clampOut = constrain(_output, _minOutput, _maxOutput);
  
  // update times and error
  _previousTime = _currentTime;
  _lastError = _error;

  return _clampOut;
}
