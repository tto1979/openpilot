#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_569863517644810549);
void live_err_fun(double *nom_x, double *delta_x, double *out_634035935564065381);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_5298129847379186960);
void live_H_mod_fun(double *state, double *out_8851719371895393500);
void live_f_fun(double *state, double dt, double *out_5169776530747108505);
void live_F_fun(double *state, double dt, double *out_4239757431990277444);
void live_h_4(double *state, double *unused, double *out_9014189468717077473);
void live_H_4(double *state, double *unused, double *out_161398558458061163);
void live_h_9(double *state, double *unused, double *out_5595239530888596262);
void live_H_9(double *state, double *unused, double *out_7448617493722508633);
void live_h_10(double *state, double *unused, double *out_7192518251958024066);
void live_H_10(double *state, double *unused, double *out_8312414767245700271);
void live_h_12(double *state, double *unused, double *out_1722393456610716087);
void live_H_12(double *state, double *unused, double *out_6219859818584671833);
void live_h_35(double *state, double *unused, double *out_1141252328464121412);
void live_H_35(double *state, double *unused, double *out_3528060615830668539);
void live_h_32(double *state, double *unused, double *out_6139954395664833010);
void live_H_32(double *state, double *unused, double *out_6233085128732418476);
void live_h_13(double *state, double *unused, double *out_5207513955360467013);
void live_H_13(double *state, double *unused, double *out_1782774541402133929);
void live_h_14(double *state, double *unused, double *out_5595239530888596262);
void live_H_14(double *state, double *unused, double *out_7448617493722508633);
void live_h_33(double *state, double *unused, double *out_5378958054232174770);
void live_H_33(double *state, double *unused, double *out_4722097164605168648);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}