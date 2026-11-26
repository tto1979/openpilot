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
void car_err_fun(double *nom_x, double *delta_x, double *out_304922656389978708);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8736675264401473111);
void car_H_mod_fun(double *state, double *out_1958795193290297982);
void car_f_fun(double *state, double dt, double *out_5561233027092623321);
void car_F_fun(double *state, double dt, double *out_8639493895554282576);
void car_h_25(double *state, double *unused, double *out_277981070052228054);
void car_H_25(double *state, double *unused, double *out_7294760654507016397);
void car_h_24(double *state, double *unused, double *out_2169134300801136205);
void car_H_24(double *state, double *unused, double *out_8926275635223666657);
void car_h_30(double *state, double *unused, double *out_8917858546770769938);
void car_H_30(double *state, double *unused, double *out_4235293077710918464);
void car_h_26(double *state, double *unused, double *out_5881072495326437253);
void car_H_26(double *state, double *unused, double *out_3553257335632960173);
void car_h_27(double *state, double *unused, double *out_2098103142851759932);
void car_H_27(double *state, double *unused, double *out_6410056389511343375);
void car_h_29(double *state, double *unused, double *out_8585375674037119243);
void car_H_29(double *state, double *unused, double *out_3725061733396526280);
void car_h_28(double *state, double *unused, double *out_1307418907959547575);
void car_H_28(double *state, double *unused, double *out_5240925940259126634);
void car_h_31(double *state, double *unused, double *out_360782353736282748);
void car_H_31(double *state, double *unused, double *out_7325406616383976825);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}