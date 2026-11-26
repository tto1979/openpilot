#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_8849506868854592089);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5410938565941557577);
void car_H_mod_fun(double *state, double *out_827426236933840625);
void car_f_fun(double *state, double dt, double *out_6706657950808162679);
void car_F_fun(double *state, double dt, double *out_5591835881445432406);
void car_h_25(double *state, double *unused, double *out_1395753995457823274);
void car_H_25(double *state, double *unused, double *out_8210940910453809530);
void car_h_24(double *state, double *unused, double *out_4854234005854810911);
void car_H_24(double *state, double *unused, double *out_8607586214807669025);
void car_h_30(double *state, double *unused, double *out_8166589221808174210);
void car_H_30(double *state, double *unused, double *out_1294250568962192775);
void car_h_26(double *state, double *unused, double *out_6998845420732032473);
void car_H_26(double *state, double *unused, double *out_6494299844381685862);
void car_h_27(double *state, double *unused, double *out_3086201830193504234);
void car_H_27(double *state, double *unused, double *out_3469013880762617686);
void car_h_29(double *state, double *unused, double *out_7467602748631524023);
void car_H_29(double *state, double *unused, double *out_5182376607632168719);
void car_h_28(double *state, double *unused, double *out_189645982553952355);
void car_H_28(double *state, double *unused, double *out_8181968449007852323);
void car_h_31(double *state, double *unused, double *out_6367511261362849167);
void car_H_31(double *state, double *unused, double *out_8180294948576849102);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}