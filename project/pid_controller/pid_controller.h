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
    double cte;		// current error (last we saw)
    double int_cte;     // integrated error i.e. sum of previous errors
    /*
    * Coefficients
    */
    double Kp; // proportional coefficient
    double Ki; // integral coefficient
    double Kd; // differential coefficient
    double steering_angle; // the current calculated action
    /*
    * Output limits
    */
    double output_lim_max;
    double output_lim_min;

    /*
    * Delta time
    */
    double delta_time;
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


