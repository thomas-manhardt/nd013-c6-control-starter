/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
   // pid coefficients
   this->Kp = Kpi;
   this->Ki = Kii;
   this->Kd = Kdi;
   // boundary conditions
   this->output_lim_max = output_lim_maxi;
   this->output_lim_min = output_lim_mini;
   // errors
   this->cte = 0.0; // current error
   this->int_cte = 0.0; // integral error
   this->delta_time = 0.0; // time delta
   this->steering_angle = 0.0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   steer = -tau_p  *cte - tau_d*  diff_cte - tau_i * int_cte
   **/
   const double eps = 1e-9;
   if (this->delta_time < eps) return; // don't do anything if delta time is too small

   double prop_term = this->Kp * cte; // proportional term
   double diff_term = this->Kd * (cte - this->cte) / this->delta_time; // differential term
   this->int_cte += cte * this->delta_time;
   double int_term = this->Ki * this->int_cte; // integral term
   this->steering_angle = - prop_term - diff_term - int_term;
   this->cte = cte; // save cte for next computation
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = this->steering_angle;
    if (control < this->output_lim_min) control = output_lim_min;
    if (control > this->output_lim_max) control = output_lim_max;
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
   this->delta_time = new_delta_time;
}
