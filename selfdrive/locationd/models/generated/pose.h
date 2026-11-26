#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6300647231984494843);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4334582594945192021);
void pose_H_mod_fun(double *state, double *out_1393911559862424916);
void pose_f_fun(double *state, double dt, double *out_7637055358137252657);
void pose_F_fun(double *state, double dt, double *out_682184522508178564);
void pose_h_4(double *state, double *unused, double *out_8188971748029855794);
void pose_H_4(double *state, double *unused, double *out_2591859041745323020);
void pose_h_10(double *state, double *unused, double *out_8240345718422408180);
void pose_H_10(double *state, double *unused, double *out_3189726223905577658);
void pose_h_13(double *state, double *unused, double *out_8973495704883649188);
void pose_H_13(double *state, double *unused, double *out_5804132867077655821);
void pose_h_14(double *state, double *unused, double *out_6947000857610459247);
void pose_H_14(double *state, double *unused, double *out_490929390550049276);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}