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
   // Initialize PID coefficients
   Kp = Kpi;
   Ki = Kii;
   Kd = Kdi;
   // limits
   output_lim_max = output_lim_maxi;
   output_lim_min = output_lim_mini;
   // state
   cte = 0.0;
   prev_cte = 0.0;
   int_cte = 0.0;
   diff_cte = 0.0; // filtered derivative
   steering_angle = 0.0;
   delta_time = 0.0;
   I_MAX = 10.0; // integral clamp limit (tunable)
   output_saturated = false;
}

void PID::UpdateError(double current_cte) {
   // Guard: need a reasonable dt to compute derivative
   const double min_dt = 1e-3;
   double dt = (delta_time < min_dt) ? min_dt : delta_time;

   // Proportional term uses current_cte directly
   double p_term = Kp * current_cte;

   // Raw derivative (risk of noise); apply simple low-pass filter
   double raw_d = (current_cte - prev_cte) / dt;
   // Exponential smoothing factor (alpha). Smaller -> more smoothing.
   const double alpha = 0.1;
   diff_cte = (1.0 - alpha) * diff_cte + alpha * raw_d;
   double d_term = Kd * diff_cte;

   // Conditional integral accumulation (anti-windup)
   // Only integrate if not saturated OR error drives output back toward range.
   bool will_reduce_saturation = (output_saturated && (
         (steering_angle >= output_lim_max && current_cte < 0) ||
         (steering_angle <= output_lim_min && current_cte > 0)));
   if (!output_saturated || will_reduce_saturation) {
       int_cte += current_cte * dt;
       // Clamp integral
       if (int_cte > I_MAX) int_cte = I_MAX;
       if (int_cte < -I_MAX) int_cte = -I_MAX;
   }
   double i_term = Ki * int_cte;

   // Combine (negative sign for standard "corrective" action)
   steering_angle = - (p_term + d_term + i_term);

   // Update previous error
   prev_cte = current_cte;
   cte = current_cte;
}

double PID::TotalError() {
    // Clamp output and mark saturation
    double control = steering_angle;
    if (control < output_lim_min) control = output_lim_min;
    if (control > output_lim_max) control = output_lim_max;
    output_saturated = (control <= output_lim_min + 1e-9) || (control >= output_lim_max - 1e-9);
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   delta_time = new_delta_time;
   return delta_time;
}

