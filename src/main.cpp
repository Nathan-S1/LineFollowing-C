#ifndef Distance_pid_controller_h
#define Distance_pid_controller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class Distance_pid_controller{
  public:
    Distance_pid_controller(int kp, int ki, int kd, double minOutput, double maxOutput);
    double update(double value, double target_value);
    
  private:
    int _kp;
    int _ki;
	  int _kd;
    int _dt;
    double _CummulError;
    double _maxOutput;
	  double _minOutput;
	  double _lastError;
	  double _error;
	  double _output;
	  double _clampOut;
    double _integral;
   long int _previousTime;
   long int _currentTime;
};

#endif
