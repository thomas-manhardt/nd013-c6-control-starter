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
    double cte;        // current (last) error
    double int_cte;    // integrated error
    double prev_cte;   // previous error (for derivative)
    double diff_cte;   // filtered derivative term (raw derivative low-pass)
    /*
    * Coefficients
    */
    double Kp;
    double Ki;
    double Kd;
    double steering_angle; // current (raw) control before clamp
    /*
    * Output limits
    */
    double output_lim_max;
    double output_lim_min;

    /* Integral clamp */
    double I_MAX; // absolute limit for integral term accumulation

    /* Saturation flag */
    bool output_saturated;

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



