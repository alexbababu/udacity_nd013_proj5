/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID {
public:

   /**
   * TODO: Create the PID class
   **/

    /*
    * Errors
    */
    double previous_track_error; // previous error for derivative calculation

    double integral_error; // integral of the error
    double derivative_error; // derivative of the error
    /*
    * Coefficients
    */
    double Kp; // proportional coefficient tau_p
    double Kd; // derivative coefficient tau_d
    double Ki; // integral coefficient tau_i
    /*
    * Output limits
    */
    double output_lim_max; // maximum output limit
    double output_lim_min; // minimum output limit
  
    /*
    * Delta time
    */
    double dt; // time step between two measurements

    /*
    * Constructor
    */
    PID();

    /*
    * Destructor.
    */
    virtual ~PID();

    /*
    * Initialize PID.
    */
    void Init(double Kp, double Ki, double Kd, double output_lim_max, double output_lim_min);

    /*
    * Update the PID error variables given cross track error.
    */
    void UpdateError(double cte);

    /*
    * Calculate the total PID error.
    */
    double TotalError();
  
    /*
    * Update the delta time.
    */
    double UpdateDeltaTime(double new_delta_time);
};

#endif //PID_CONTROLLER_H


