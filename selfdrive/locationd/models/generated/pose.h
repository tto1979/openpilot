#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8400786688062439867);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5956079812122370151);
void pose_H_mod_fun(double *state, double *out_8542454711932240757);
void pose_f_fun(double *state, double dt, double *out_6490866571001637660);
void pose_F_fun(double *state, double dt, double *out_4543420401477853170);
void pose_h_4(double *state, double *unused, double *out_6014386894159692855);
void pose_H_4(double *state, double *unused, double *out_6012289415225888079);
void pose_h_10(double *state, double *unused, double *out_2247240218230926467);
void pose_H_10(double *state, double *unused, double *out_2995791716314335637);
void pose_h_13(double *state, double *unused, double *out_2231888146994886805);
void pose_H_13(double *state, double *unused, double *out_1598341793090812850);
void pose_h_14(double *state, double *unused, double *out_2947005334086789936);
void pose_H_14(double *state, double *unused, double *out_2049048558886403550);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}