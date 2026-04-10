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
  previous_track_error = 0.0;
  integral_error = 0.0;
  derivative_error = 0.0;
  dt = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   **/
  if (delta_time != =){
    derivative_error = (cte - previous_track_error) / delta_time;
  }
  else{
    derivative_error = 0;
  }
  integral_error += cte * delta_time;
  previous_track_error = cte;
  
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   * The code should return a value in the interval [output_lim_mini,
   * output_lim_maxi]
   */
  double control;
  control = -(Kp * previous_track_error) - (Ki * integral_error) - (Kd * derivative_error);  // calculate the control output from thePID formula 

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