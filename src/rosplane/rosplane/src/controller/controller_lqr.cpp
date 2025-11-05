/**
 * @file controller_lqr.cpp
 *
 * Implements the LQR controller described in Small Unmanned Aircraft by Tim McLain and Randy Beard
 *
 * @author Matheus Sanches <matheusps1995@hotmail.com>
 */

#include "controller/controller_lqr.hpp"

namespace rosplane
{

ControllerLQR::ControllerLQR()
{

  // Initialize course hold, roll hold and pitch hold errors and integrators to zero.
  chi_integrator_ = 0;
  chi_error_ = 0;
  h_integrator_ = 0;
  h_error_ = 0;
  va_integrator_ = 0;
  va_error_ = 0;

  // Declare parameters associated with this controller, controller_state_machine
  declare_parameters();
  // Set parameters according to the parameters in the launch file, otherwise use the default values
  params_.set_parameters();

  //py::initialize_interpreter();
}

/*void ControllerLQR::take_off(const Input & input, Output & output)
{
  // Run lateral and longitudinal controls.
  take_off_lateral_control(input, output);
  take_off_longitudinal_control(input, output);
}

void ControllerLQR::take_off_exit()
{
  // Put any code that should run as the airplane exits take off mode.
}

void ControllerLQR::climb(const Input & input, Output & output)
{
  // Run lateral and longitudinal controls.
  climb_lateral_control(input, output);
  climb_longitudinal_control(input, output);
}*/

void ControllerLQR::climb_exit()
{
  chi_integrator_ = 0;
  chi_error_ = 0;
  h_integrator_ = 0;
  h_error_ = 0;
  va_integrator_ = 0;
  va_error_ = 0;
}

void ControllerLQR::altitude_hold(const Input & input, Output & output)
{
  const bool lqr_true = params_.get_bool("lqr_true");

  if (lqr_true == true)
  {
    // Run lateral and longitudinal controls for LQR.
    lqr_alt_hold_lateral_control(input, output);
    alt_hold_longitudinal_control(input, output);
  }
  else
  {
    
    chi_integrator_ = 0;
    h_integrator_ = 0;
    va_integrator_ = 0;
    
    // Run lateral and longitudinal controls.
    alt_hold_lateral_control(input, output);
    alt_hold_longitudinal_control(input, output);
  }
}

void ControllerLQR::altitude_hold_exit()
{
  // Put any code that should run as the airplane exits altitude hold mode.
}

void ControllerLQR::lqr_alt_hold_lateral_control(const Input& input, Output& output)
{
  // Trims (all in SI units: rad, m/s)
  double xt_v   = params_.get_double("lqr_trim_v");
  double xt_p   = params_.get_double("lqr_trim_p");
  double xt_r   = params_.get_double("lqr_trim_r");
  double xt_phi = params_.get_double("lqr_trim_phi");
  //double xt_chi = params_.get_double("lqr_trim_chi");

  double trim_a = params_.get_double("lqr_trim_da");
  //double trim_r = params_.get_double("lqr_trim_dr");

  // LQR gains for aileron (state row) and rudder (if used)
  double ka_v   = params_.get_double("lqr_ka_v");
  double ka_p   = params_.get_double("lqr_ka_p");
  double ka_r   = params_.get_double("lqr_ka_r");
  double ka_phi = params_.get_double("lqr_ka_phi");
  double ka_chi = params_.get_double("lqr_ka_chi");

  /*double kr_v   = params_.get_double("lqr_kr_v");
  double kr_p   = params_.get_double("lqr_kr_p");
  double kr_r   = params_.get_double("lqr_kr_r");
  double kr_phi = params_.get_double("lqr_kr_phi");
  double kr_chi = params_.get_double("lqr_kr_chi");*/

  // Integral gain for course (aileron row)
  double ka_Ichi = params_.get_double("lqr_ka_Ichi");

  // Limits (radians)
  double max_a = params_.get_double("max_a");
  //double max_r = params_.get_double("max_r");

  // Controller frequency from base (or simply use in.Ts)
  double frequency = params_.get_double("controller_output_frequency");
  float Ts = 1.0 / frequency;

  // Deviation errors: e = x - x_trim
  float e_v   = input.v   - xt_v;
  float e_p   = input.p   - xt_p;
  float e_r   = input.r   - xt_r;
  float e_phi = input.phi - xt_phi;
  //const float e_chi = input.chi;

  // Integral update (H positive convention): integrate (chi - chi_c)
  double wrapped_chi_c = wrap_within_180(input.chi, input.chi_c);
  float chi_error = input.chi - wrapped_chi_c;

  float chi_integrator_prev = chi_integrator_;
  chi_integrator_ = chi_integrator_ + (Ts / 2.0) * (chi_error + chi_error_);

  float factor = params_.get_double("factor_lat");

  // Raw LQR with integral on aileron row
  float da_cmd_unsat =
      trim_a
    - 1.0 * (ka_v*e_v + ka_p*e_p + ka_r*e_r + ka_phi*e_phi + ka_chi*chi_error)
    - factor * (ka_Ichi * chi_integrator_);

  // If you still compute a rudder LQR row, you can keep it here; otherwise yaw damper:
  //const float dr_cmd_unsat =
  //    trim_r
  //  - (kr_v*e_v + kr_p*e_p + kr_r*e_r + kr_phi*e_phi + kr_chi*e_chi);

  // Saturate using your existing helper
  float da_cmd = sat(da_cmd_unsat,  max_a, -max_a);
  //const float dr_cmd = sat(dr_cmd_unsat,  max_r, -max_r);

  // Anti-windup: if aileron saturates, restore integrator
  if (fabs(da_cmd - da_cmd_unsat) > 0.0001) {
    chi_integrator_ = chi_integrator_prev;
  }

  chi_error_ = chi_error;

  // Outputs:
  output.delta_a = da_cmd;
  output.delta_r = yaw_damper(input.r); // or: dr_cmd if using the LQR rudder row
  output.phi_c   = 0.0f;
}


void ControllerLQR::lqr_alt_hold_longitudinal_control(const Input& input, Output& output)
{
  // Trims (SI units)
  double xt_u     = params_.get_double("lqr_trim_u");
  double xt_w     = params_.get_double("lqr_trim_w");
  double xt_q     = params_.get_double("lqr_trim_q");
  double xt_theta = params_.get_double("lqr_trim_theta");
  //double xt_h     = params_.get_double("lqr_trim_h");

  double trim_e = params_.get_double("lqr_trim_de");
  double trim_t = params_.get_double("lqr_trim_dt");

  // LQR gains (state rows)
  double ke_u     = params_.get_double("lqr_ke_u");
  double ke_w     = params_.get_double("lqr_ke_w");
  double ke_q     = params_.get_double("lqr_ke_q");
  double ke_theta = params_.get_double("lqr_ke_theta");
  double ke_h     = params_.get_double("lqr_ke_h");

  double kt_u     = params_.get_double("lqr_kt_u");
  double kt_w     = params_.get_double("lqr_kt_w");
  double kt_q     = params_.get_double("lqr_kt_q");
  double kt_theta = params_.get_double("lqr_kt_theta");
  double kt_h     = params_.get_double("lqr_kt_h");

  // Integral gains (from augmented LQR design)
  double ke_Ih    = params_.get_double("lqr_ke_Ih");   // elevator on ∫(h - h_c)
  double ke_IVa   = params_.get_double("lqr_ke_IVa");  // elevator on ∫(va - va_c)
  double kt_Ih    = params_.get_double("lqr_kt_Ih");   // throttle on ∫(h - h_c)
  double kt_IVa   = params_.get_double("lqr_kt_IVa");  // throttle on ∫(va - va_c)

  // Limits
  double max_e = params_.get_double("max_e");  // elevator magnitude limit (rad)
  double max_t = params_.get_double("max_t");  // throttle upper limit [0..1]

  // Controller frequency from base (or simply use in.Ts)
  double frequency = params_.get_double("controller_output_frequency");
  float Ts = 1.0 / frequency;

  // Deviation states e = x - x_trim  (state order [u, w, q, theta, h])
  float e_u     = input.u     - xt_u;
  float e_w     = input.w     - xt_w;
  float e_q     = input.q     - xt_q;
  float e_theta = input.theta - xt_theta;
  //const float e_h     = input.h     - xt_h;

  double alt_hz = params_.get_double("alt_hz");
  // Saturate the altitude command.
  double adjusted_hc = adjust_h_c(input.h_c, input.h, alt_hz);

  // Integral updates (H positive convention): integrate (measured - commanded)
  float h_err  = input.h - adjusted_hc;
  float va_err = input.va - input.va_c;

  float h_integrator_prev  = h_integrator_;
  float va_integrator_prev = va_integrator_;

  if (-alt_hz + .01 < h_err && h_err < alt_hz - .01) 
  {
    h_integrator_ = h_integrator_ + (Ts / 2.0) * (h_err + h_error_);
  } else {
    h_integrator_ = 0.0;
  }

  va_integrator_ = va_integrator_ + (Ts / 2.0) * (va_err + va_error_);

  double factor = params_.get_double("factor_lon");

  // Raw LQR + integral: u = u* - Kx*e - Ki*[I_h; I_Va]
  float de_cmd_unsat =
      trim_e
    - factor * (ke_u*e_u + ke_w*e_w + ke_q*e_q + ke_theta*e_theta + ke_h*h_err)
    - factor * (ke_Ih*h_integrator_ + ke_IVa*va_integrator_);

  float dt_cmd_unsat =
      trim_t
    - factor * (kt_u*e_u + kt_w*e_w + kt_q*e_q + kt_theta*e_theta + kt_h*h_err)
    - factor * (kt_Ih*h_integrator_ + kt_IVa*va_integrator_);

  // Throttle clamped to [0, max_t]
  float de_cmd = sat(de_cmd_unsat,  max_e, -max_e);
  float dt_cmd = sat(dt_cmd_unsat,  max_t,  0);

  // Anti-windup: if any command saturates, restore integrators (freeze)
  if (fabs(de_cmd - de_cmd_unsat) > 0.0001 || fabs(dt_cmd - dt_cmd_unsat) > 0.0001) {
    h_integrator_  = h_integrator_prev;
    va_integrator_ = va_integrator_prev;
  }

  h_error_ = h_err;
  va_error_ = va_err;

  // Outputs
  output.delta_e = de_cmd;
  output.delta_t = dt_cmd;
  output.theta_c = 0.0f;
}

/*void ControllerLQR::climb_lateral_control(const Input & input, Output & output)
{

}

void ControllerLQR::climb_longitudinal_control(const Input & input, Output & output)
{

}

void ControllerLQR::take_off_lateral_control(const Input & input, Output & output)
{

}

void ControllerLQR::take_off_longitudinal_control(const Input & input, Output & output)
{

}*/



void ControllerLQR::declare_parameters()
{
  // Actuators Trim
  params_.declare_double("lqr_trim_de",  0.0993);
  params_.declare_double("lqr_trim_dt",  0.5060);
  params_.declare_double("lqr_trim_da",  0.0040);
  params_.declare_double("lqr_trim_dr", -0.0006);

  // Longitudinal Trim States LQR (u, w, q, theta, h)
  params_.declare_double("lqr_trim_u",  14.9987); // m/s
  params_.declare_double("lqr_trim_w",  0.2009); // m/s
  params_.declare_double("lqr_trim_q",  0.0000); // rad/s
  params_.declare_double("lqr_trim_theta",  0.0134); // rad
  params_.declare_double("lqr_trim_h",  0.0000); // m

  // Lateral Trim States LQR (v, p, r, phi, chi)
  params_.declare_double("lqr_trim_v",  0.0000); // m/s
  params_.declare_double("lqr_trim_p", -0.0000); // rad/s
  params_.declare_double("lqr_trim_r", -0.0000); // rad/s
  params_.declare_double("lqr_trim_phi", -0.0003); // rad
  params_.declare_double("lqr_trim_chi",  0.0000); // rad

  // Longitudinal K gains for elevator de and throtle dt
  params_.declare_double("lqr_ke_u", -0.0051);
  params_.declare_double("lqr_ke_w",  0.1784);
  params_.declare_double("lqr_ke_q", -0.3064);
  params_.declare_double("lqr_ke_theta", -5.5531);
  params_.declare_double("lqr_ke_h",  0.5787);
  params_.declare_double("lqr_ke_Ih",  0.4518);
  params_.declare_double("lqr_ke_IVa",  0.1222);

  params_.declare_double("lqr_kt_u",  0.2569);
  params_.declare_double("lqr_kt_w", -0.0105);
  params_.declare_double("lqr_kt_q",  0.0004);
  params_.declare_double("lqr_kt_theta",  0.1869);
  params_.declare_double("lqr_kt_h", -0.1194);
  params_.declare_double("lqr_kt_Ih", -0.0999);
  params_.declare_double("lqr_kt_IVa",  0.4938);

  // Lateral K gains for aileron ka and rudder kr
  params_.declare_double("lqr_ka_v",  0.0360);
  params_.declare_double("lqr_ka_p",  0.0548);
  params_.declare_double("lqr_ka_r",  0.0154);
  params_.declare_double("lqr_ka_phi",  0.7091);
  params_.declare_double("lqr_ka_chi",  0.6005);
  params_.declare_double("lqr_ka_Ichi",  0.1002);

  params_.declare_bool("lqr_true",  false);
  params_.declare_double("factor_lat", 0.0);
  params_.declare_double("factor_lon", 0.0);

  params_.declare_double("lqr_max_a", .15);
  params_.declare_double("lqr_max_e", .15);
  params_.declare_double("lqr_max_r", 1.0);
  params_.declare_double("lqr_max_t", 1.0);

}

} // namespace rosplane