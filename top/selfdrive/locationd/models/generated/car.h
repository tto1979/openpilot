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
void car_err_fun(double *nom_x, double *delta_x, double *out_3231158882167630390);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2255683443976328672);
void car_H_mod_fun(double *state, double *out_1538918716375221260);
void car_f_fun(double *state, double dt, double *out_5181322862125731861);
void car_F_fun(double *state, double dt, double *out_5354287983213807462);
void car_h_25(double *state, double *unused, double *out_3994723713418228284);
void car_H_25(double *state, double *unused, double *out_4557989650231153356);
void car_h_24(double *state, double *unused, double *out_2483498575061758787);
void car_H_24(double *state, double *unused, double *out_1000064051428830220);
void car_h_30(double *state, double *unused, double *out_2776111512932122652);
void car_H_30(double *state, double *unused, double *out_2039656691723904729);
void car_h_26(double *state, double *unused, double *out_1832601660030460217);
void car_H_26(double *state, double *unused, double *out_8299492969105209580);
void car_h_27(double *state, double *unused, double *out_8996396981079073929);
void car_H_27(double *state, double *unused, double *out_183937379460038488);
void car_h_29(double *state, double *unused, double *out_5812897564376455337);
void car_H_29(double *state, double *unused, double *out_1529425347409512545);
void car_h_28(double *state, double *unused, double *out_4828161063174369373);
void car_H_28(double *state, double *unused, double *out_6611824364479043119);
void car_h_31(double *state, double *unused, double *out_6688755103470650891);
void car_H_31(double *state, double *unused, double *out_4527343688354192928);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}