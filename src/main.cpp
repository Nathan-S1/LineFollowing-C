#include <Pololu3piPlus32U4.h>
//#include <Wire.h>
#include "printOLED.h"
#include "odometry.h"
#include "pid_controller.h"
#include "Distance_pid_controller.h"
using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;

#define DEAD_RECKONING false
#define diaL 3.2
#define diaR 3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define minOutput -100
#define maxOutput 100
#define kp 60  // find good values for these controller gains
#define kd 30
#define ki 15

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
pid_controller pid_controller(kp, ki, kd, minOutput, maxOutput);
// Distance_pid_controller Distance_pid_controller(kp, ki, kd, minOutput, maxOutput);

// goals in cm and rad
const float goal_x = 100;
const float goal_y = 100;
const float goal_theta = 0.785;

//odometry
int16_t deltaL = 0, deltaR = 0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;

//baseSpeed
const float baseSpeed = 100;

//Boolean to continue
bool run = true;


  void
  setup() {
  Serial.begin(9600);
}

void loop() {
  if (run) {
    // Read data from encoders
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();

    // Increment total encoder cound
    encCountsLeft += deltaL;
    encCountsRight += deltaR;

    odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);  //calculate robot's position

    // angle our robot is facing with respect to our current position
    double angle_to_goal = atan2(goal_y - y, goal_x - x);
    // angle we need the robot to face
    double actual_angle = atan2(sin(theta), cos(theta));

    double current_distance = sqrt((x*x) + (y*y));
    double desired_distance = sqrt(20000);


    // TODO call controller's update function
    // and the motors.setSpeeds(left, right) here
    // motors.setSpeeds(baseSpeed, baseSpeed);


    // update pid_controller with error for Lab 4 step 3
    // float control_signal = pid_controller.update(theta, goal_theta);

    // update pid_controller for lab 4:
    float control_signal = pid_controller.update(actual_angle, angle_to_goal);

    // update Distance PID controller for lab4 step 3
    // float distance_control_signal = Distance_pid_controller.update(desired_distance, current_distance);

    Serial.print("Theta control signal: ");
    Serial.println(control_signal);
    Serial.print("Distance control signal: ");
    // Serial.println(distance_control_signal);

    //Combined control signals:
    // float total_control_signal = control_signal + distance_control_signal;

    // ^^^ PID controller for step three was update(theta, goal_theta);

    //calculate wheel velocities for step 4.2-
    int rightWheelSpeed = -control_signal + baseSpeed;
    int leftWheelSpeed = control_signal + baseSpeed;

    //calculate wheel velocities for step 4.3-
    // int rightWheelSpeed = -control_signal + distance_control_signal + baseSpeed;
    // int leftWheelSpeed = control_signal + distance_control_signal + baseSpeed;

    // update wheel speeds
    motors.setSpeeds(leftWheelSpeed, rightWheelSpeed);

    Serial.print("X value: ");
    Serial.println(x);

    // Step 4 did not include the step below:
    // if ((abs(goal_y - y) < 1) && abs(goal_x - x) < 1) {
    //   motors.setSpeeds(0,0);
    //   run = false;
    // }

  }
}
