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
void live_H(double *in_vec, double *out_4043676768277520287);
void live_err_fun(double *nom_x, double *delta_x, double *out_7737101252641419964);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_6169519276112948062);
void live_H_mod_fun(double *state, double *out_7356630437633909026);
void live_f_fun(double *state, double dt, double *out_5010714662003537791);
void live_F_fun(double *state, double dt, double *out_7219900460637439194);
void live_h_4(double *state, double *unused, double *out_5749275878221916624);
void live_H_4(double *state, double *unused, double *out_916170304046667809);
void live_h_9(double *state, double *unused, double *out_2449977666238941803);
void live_H_9(double *state, double *unused, double *out_1157359950676258454);
void live_h_10(double *state, double *unused, double *out_3276681402667739127);
void live_H_10(double *state, double *unused, double *out_7307260979416931515);
void live_h_12(double *state, double *unused, double *out_5269246623230359412);
void live_H_12(double *state, double *unused, double *out_1537269329094261476);
void live_h_35(double *state, double *unused, double *out_7766364239290227446);
void live_H_35(double *state, double *unused, double *out_4282832361419275185);
void live_h_32(double *state, double *unused, double *out_1048729722811172389);
void live_H_32(double *state, double *unused, double *out_7830676701406433914);
void live_h_13(double *state, double *unused, double *out_4675788559484252165);
void live_H_13(double *state, double *unused, double *out_8809998074231566326);
void live_h_14(double *state, double *unused, double *out_2449977666238941803);
void live_H_14(double *state, double *unused, double *out_1157359950676258454);
void live_h_33(double *state, double *unused, double *out_5088122228283621623);
void live_H_33(double *state, double *unused, double *out_7433389366058132789);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}