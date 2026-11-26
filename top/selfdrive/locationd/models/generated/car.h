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
void car_err_fun(double *nom_x, double *delta_x, double *out_2414765271784937644);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3704150315909089492);
void car_H_mod_fun(double *state, double *out_814895361450768104);
void car_f_fun(double *state, double dt, double *out_1583533894033430000);
void car_F_fun(double *state, double dt, double *out_5522342914271432502);
void car_h_25(double *state, double *unused, double *out_142737944331781953);
void car_H_25(double *state, double *unused, double *out_3979935481477101005);
void car_h_24(double *state, double *unused, double *out_9041743034093230167);
void car_H_24(double *state, double *unused, double *out_6152585080482600571);
void car_h_30(double *state, double *unused, double *out_458680213281581601);
void car_H_30(double *state, double *unused, double *out_1461602522969852378);
void car_h_26(double *state, double *unused, double *out_5460353480942427246);
void car_H_26(double *state, double *unused, double *out_7721438800351157229);
void car_h_27(double *state, double *unused, double *out_5144411211992627598);
void car_H_27(double *state, double *unused, double *out_7764348950304417502);
void car_h_29(double *state, double *unused, double *out_9006094688421129250);
void car_H_29(double *state, double *unused, double *out_7997400467290317019);
void car_h_28(double *state, double *unused, double *out_8076425645881599071);
void car_H_28(double *state, double *unused, double *out_6033770195724990768);
void car_h_31(double *state, double *unused, double *out_7906003201152454394);
void car_H_31(double *state, double *unused, double *out_8347646902584508705);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}