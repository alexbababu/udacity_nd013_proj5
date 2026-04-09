/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi,
               double output_lim_mini) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
  track_error = 0.0;
  previous_track_error = 0.0;
  integral_error = 0.0;
  derivative_error = 0.0;
  dt = 0.0;
}

void PID::UpdateError(double cte, bool first_update) {
  /**
   * TODO: Update PID errors based on cte.
   **/
  if (first_update) {
    previous_track_error = cte;
    first_update = false;
  }  else if (!first_update) {
    previous_track_error =
        track_error;  // save the previous error for derivative calculation
  }
  std::cout << "!!---- first_update: " << first_update << " ----!!" << endl;
  std::cout << "!!---- previous_track_error: " << previous_track_error << " ----!!" << endl;
  track_error = cte;
  std::cout << "!!---- track_error: " << track_error << " ----!!" << endl;
  integral_error += cte * dt;  // integrate the error over time
  if (dt != 0) {
    derivative_error = (track_error - previous_track_error) / dt;  // calculate the derivative of the error
  } else {
    derivative_error = 0;
  }
  //std::cout << "!!---- cte: " << cte << " ----!!" << endl;
  //std::cout << "!!---- track_error: " << track_error << " ----!!" << endl; 
  //std::cout << "!!---- integral_error: " << integral_error << " ----!!" << endl;
  //std::cout << "!!---- derivative_error: " << derivative_error << " ----!!" << endl;
  //std::cout << "!!---- dt: " << dt << " ----!!" << endl;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   * The code should return a value in the interval [output_lim_mini,
   * output_lim_maxi]
   */
  double control;
  control = -(Kp * track_error + Ki * integral_error +
             Kd * derivative_error);  // calculate the control output from the
                                      // PID formula
  if (control >
      output_lim_max) {  // limit the control output to the maximum value
    control = output_lim_max;
  } else if (control <
             output_lim_min) {  // limit the control output to the minimum value
    control = output_lim_min;
  }
  return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  /**
   * TODO: Update the delta time with new value
   */
  dt = new_delta_time;
  return dt;
}