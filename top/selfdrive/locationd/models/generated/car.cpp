#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3231158882167630390) {
   out_3231158882167630390[0] = delta_x[0] + nom_x[0];
   out_3231158882167630390[1] = delta_x[1] + nom_x[1];
   out_3231158882167630390[2] = delta_x[2] + nom_x[2];
   out_3231158882167630390[3] = delta_x[3] + nom_x[3];
   out_3231158882167630390[4] = delta_x[4] + nom_x[4];
   out_3231158882167630390[5] = delta_x[5] + nom_x[5];
   out_3231158882167630390[6] = delta_x[6] + nom_x[6];
   out_3231158882167630390[7] = delta_x[7] + nom_x[7];
   out_3231158882167630390[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2255683443976328672) {
   out_2255683443976328672[0] = -nom_x[0] + true_x[0];
   out_2255683443976328672[1] = -nom_x[1] + true_x[1];
   out_2255683443976328672[2] = -nom_x[2] + true_x[2];
   out_2255683443976328672[3] = -nom_x[3] + true_x[3];
   out_2255683443976328672[4] = -nom_x[4] + true_x[4];
   out_2255683443976328672[5] = -nom_x[5] + true_x[5];
   out_2255683443976328672[6] = -nom_x[6] + true_x[6];
   out_2255683443976328672[7] = -nom_x[7] + true_x[7];
   out_2255683443976328672[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1538918716375221260) {
   out_1538918716375221260[0] = 1.0;
   out_1538918716375221260[1] = 0.0;
   out_1538918716375221260[2] = 0.0;
   out_1538918716375221260[3] = 0.0;
   out_1538918716375221260[4] = 0.0;
   out_1538918716375221260[5] = 0.0;
   out_1538918716375221260[6] = 0.0;
   out_1538918716375221260[7] = 0.0;
   out_1538918716375221260[8] = 0.0;
   out_1538918716375221260[9] = 0.0;
   out_1538918716375221260[10] = 1.0;
   out_1538918716375221260[11] = 0.0;
   out_1538918716375221260[12] = 0.0;
   out_1538918716375221260[13] = 0.0;
   out_1538918716375221260[14] = 0.0;
   out_1538918716375221260[15] = 0.0;
   out_1538918716375221260[16] = 0.0;
   out_1538918716375221260[17] = 0.0;
   out_1538918716375221260[18] = 0.0;
   out_1538918716375221260[19] = 0.0;
   out_1538918716375221260[20] = 1.0;
   out_1538918716375221260[21] = 0.0;
   out_1538918716375221260[22] = 0.0;
   out_1538918716375221260[23] = 0.0;
   out_1538918716375221260[24] = 0.0;
   out_1538918716375221260[25] = 0.0;
   out_1538918716375221260[26] = 0.0;
   out_1538918716375221260[27] = 0.0;
   out_1538918716375221260[28] = 0.0;
   out_1538918716375221260[29] = 0.0;
   out_1538918716375221260[30] = 1.0;
   out_1538918716375221260[31] = 0.0;
   out_1538918716375221260[32] = 0.0;
   out_1538918716375221260[33] = 0.0;
   out_1538918716375221260[34] = 0.0;
   out_1538918716375221260[35] = 0.0;
   out_1538918716375221260[36] = 0.0;
   out_1538918716375221260[37] = 0.0;
   out_1538918716375221260[38] = 0.0;
   out_1538918716375221260[39] = 0.0;
   out_1538918716375221260[40] = 1.0;
   out_1538918716375221260[41] = 0.0;
   out_1538918716375221260[42] = 0.0;
   out_1538918716375221260[43] = 0.0;
   out_1538918716375221260[44] = 0.0;
   out_1538918716375221260[45] = 0.0;
   out_1538918716375221260[46] = 0.0;
   out_1538918716375221260[47] = 0.0;
   out_1538918716375221260[48] = 0.0;
   out_1538918716375221260[49] = 0.0;
   out_1538918716375221260[50] = 1.0;
   out_1538918716375221260[51] = 0.0;
   out_1538918716375221260[52] = 0.0;
   out_1538918716375221260[53] = 0.0;
   out_1538918716375221260[54] = 0.0;
   out_1538918716375221260[55] = 0.0;
   out_1538918716375221260[56] = 0.0;
   out_1538918716375221260[57] = 0.0;
   out_1538918716375221260[58] = 0.0;
   out_1538918716375221260[59] = 0.0;
   out_1538918716375221260[60] = 1.0;
   out_1538918716375221260[61] = 0.0;
   out_1538918716375221260[62] = 0.0;
   out_1538918716375221260[63] = 0.0;
   out_1538918716375221260[64] = 0.0;
   out_1538918716375221260[65] = 0.0;
   out_1538918716375221260[66] = 0.0;
   out_1538918716375221260[67] = 0.0;
   out_1538918716375221260[68] = 0.0;
   out_1538918716375221260[69] = 0.0;
   out_1538918716375221260[70] = 1.0;
   out_1538918716375221260[71] = 0.0;
   out_1538918716375221260[72] = 0.0;
   out_1538918716375221260[73] = 0.0;
   out_1538918716375221260[74] = 0.0;
   out_1538918716375221260[75] = 0.0;
   out_1538918716375221260[76] = 0.0;
   out_1538918716375221260[77] = 0.0;
   out_1538918716375221260[78] = 0.0;
   out_1538918716375221260[79] = 0.0;
   out_1538918716375221260[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_5181322862125731861) {
   out_5181322862125731861[0] = state[0];
   out_5181322862125731861[1] = state[1];
   out_5181322862125731861[2] = state[2];
   out_5181322862125731861[3] = state[3];
   out_5181322862125731861[4] = state[4];
   out_5181322862125731861[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_5181322862125731861[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_5181322862125731861[7] = state[7];
   out_5181322862125731861[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5354287983213807462) {
   out_5354287983213807462[0] = 1;
   out_5354287983213807462[1] = 0;
   out_5354287983213807462[2] = 0;
   out_5354287983213807462[3] = 0;
   out_5354287983213807462[4] = 0;
   out_5354287983213807462[5] = 0;
   out_5354287983213807462[6] = 0;
   out_5354287983213807462[7] = 0;
   out_5354287983213807462[8] = 0;
   out_5354287983213807462[9] = 0;
   out_5354287983213807462[10] = 1;
   out_5354287983213807462[11] = 0;
   out_5354287983213807462[12] = 0;
   out_5354287983213807462[13] = 0;
   out_5354287983213807462[14] = 0;
   out_5354287983213807462[15] = 0;
   out_5354287983213807462[16] = 0;
   out_5354287983213807462[17] = 0;
   out_5354287983213807462[18] = 0;
   out_5354287983213807462[19] = 0;
   out_5354287983213807462[20] = 1;
   out_5354287983213807462[21] = 0;
   out_5354287983213807462[22] = 0;
   out_5354287983213807462[23] = 0;
   out_5354287983213807462[24] = 0;
   out_5354287983213807462[25] = 0;
   out_5354287983213807462[26] = 0;
   out_5354287983213807462[27] = 0;
   out_5354287983213807462[28] = 0;
   out_5354287983213807462[29] = 0;
   out_5354287983213807462[30] = 1;
   out_5354287983213807462[31] = 0;
   out_5354287983213807462[32] = 0;
   out_5354287983213807462[33] = 0;
   out_5354287983213807462[34] = 0;
   out_5354287983213807462[35] = 0;
   out_5354287983213807462[36] = 0;
   out_5354287983213807462[37] = 0;
   out_5354287983213807462[38] = 0;
   out_5354287983213807462[39] = 0;
   out_5354287983213807462[40] = 1;
   out_5354287983213807462[41] = 0;
   out_5354287983213807462[42] = 0;
   out_5354287983213807462[43] = 0;
   out_5354287983213807462[44] = 0;
   out_5354287983213807462[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5354287983213807462[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5354287983213807462[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5354287983213807462[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5354287983213807462[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5354287983213807462[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5354287983213807462[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5354287983213807462[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5354287983213807462[53] = -9.8100000000000005*dt;
   out_5354287983213807462[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5354287983213807462[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5354287983213807462[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5354287983213807462[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5354287983213807462[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5354287983213807462[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5354287983213807462[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5354287983213807462[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5354287983213807462[62] = 0;
   out_5354287983213807462[63] = 0;
   out_5354287983213807462[64] = 0;
   out_5354287983213807462[65] = 0;
   out_5354287983213807462[66] = 0;
   out_5354287983213807462[67] = 0;
   out_5354287983213807462[68] = 0;
   out_5354287983213807462[69] = 0;
   out_5354287983213807462[70] = 1;
   out_5354287983213807462[71] = 0;
   out_5354287983213807462[72] = 0;
   out_5354287983213807462[73] = 0;
   out_5354287983213807462[74] = 0;
   out_5354287983213807462[75] = 0;
   out_5354287983213807462[76] = 0;
   out_5354287983213807462[77] = 0;
   out_5354287983213807462[78] = 0;
   out_5354287983213807462[79] = 0;
   out_5354287983213807462[80] = 1;
}
void h_25(double *state, double *unused, double *out_3994723713418228284) {
   out_3994723713418228284[0] = state[6];
}
void H_25(double *state, double *unused, double *out_4557989650231153356) {
   out_4557989650231153356[0] = 0;
   out_4557989650231153356[1] = 0;
   out_4557989650231153356[2] = 0;
   out_4557989650231153356[3] = 0;
   out_4557989650231153356[4] = 0;
   out_4557989650231153356[5] = 0;
   out_4557989650231153356[6] = 1;
   out_4557989650231153356[7] = 0;
   out_4557989650231153356[8] = 0;
}
void h_24(double *state, double *unused, double *out_2483498575061758787) {
   out_2483498575061758787[0] = state[4];
   out_2483498575061758787[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1000064051428830220) {
   out_1000064051428830220[0] = 0;
   out_1000064051428830220[1] = 0;
   out_1000064051428830220[2] = 0;
   out_1000064051428830220[3] = 0;
   out_1000064051428830220[4] = 1;
   out_1000064051428830220[5] = 0;
   out_1000064051428830220[6] = 0;
   out_1000064051428830220[7] = 0;
   out_1000064051428830220[8] = 0;
   out_1000064051428830220[9] = 0;
   out_1000064051428830220[10] = 0;
   out_1000064051428830220[11] = 0;
   out_1000064051428830220[12] = 0;
   out_1000064051428830220[13] = 0;
   out_1000064051428830220[14] = 1;
   out_1000064051428830220[15] = 0;
   out_1000064051428830220[16] = 0;
   out_1000064051428830220[17] = 0;
}
void h_30(double *state, double *unused, double *out_2776111512932122652) {
   out_2776111512932122652[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2039656691723904729) {
   out_2039656691723904729[0] = 0;
   out_2039656691723904729[1] = 0;
   out_2039656691723904729[2] = 0;
   out_2039656691723904729[3] = 0;
   out_2039656691723904729[4] = 1;
   out_2039656691723904729[5] = 0;
   out_2039656691723904729[6] = 0;
   out_2039656691723904729[7] = 0;
   out_2039656691723904729[8] = 0;
}
void h_26(double *state, double *unused, double *out_1832601660030460217) {
   out_1832601660030460217[0] = state[7];
}
void H_26(double *state, double *unused, double *out_8299492969105209580) {
   out_8299492969105209580[0] = 0;
   out_8299492969105209580[1] = 0;
   out_8299492969105209580[2] = 0;
   out_8299492969105209580[3] = 0;
   out_8299492969105209580[4] = 0;
   out_8299492969105209580[5] = 0;
   out_8299492969105209580[6] = 0;
   out_8299492969105209580[7] = 1;
   out_8299492969105209580[8] = 0;
}
void h_27(double *state, double *unused, double *out_8996396981079073929) {
   out_8996396981079073929[0] = state[3];
}
void H_27(double *state, double *unused, double *out_183937379460038488) {
   out_183937379460038488[0] = 0;
   out_183937379460038488[1] = 0;
   out_183937379460038488[2] = 0;
   out_183937379460038488[3] = 1;
   out_183937379460038488[4] = 0;
   out_183937379460038488[5] = 0;
   out_183937379460038488[6] = 0;
   out_183937379460038488[7] = 0;
   out_183937379460038488[8] = 0;
}
void h_29(double *state, double *unused, double *out_5812897564376455337) {
   out_5812897564376455337[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1529425347409512545) {
   out_1529425347409512545[0] = 0;
   out_1529425347409512545[1] = 1;
   out_1529425347409512545[2] = 0;
   out_1529425347409512545[3] = 0;
   out_1529425347409512545[4] = 0;
   out_1529425347409512545[5] = 0;
   out_1529425347409512545[6] = 0;
   out_1529425347409512545[7] = 0;
   out_1529425347409512545[8] = 0;
}
void h_28(double *state, double *unused, double *out_4828161063174369373) {
   out_4828161063174369373[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6611824364479043119) {
   out_6611824364479043119[0] = 1;
   out_6611824364479043119[1] = 0;
   out_6611824364479043119[2] = 0;
   out_6611824364479043119[3] = 0;
   out_6611824364479043119[4] = 0;
   out_6611824364479043119[5] = 0;
   out_6611824364479043119[6] = 0;
   out_6611824364479043119[7] = 0;
   out_6611824364479043119[8] = 0;
}
void h_31(double *state, double *unused, double *out_6688755103470650891) {
   out_6688755103470650891[0] = state[8];
}
void H_31(double *state, double *unused, double *out_4527343688354192928) {
   out_4527343688354192928[0] = 0;
   out_4527343688354192928[1] = 0;
   out_4527343688354192928[2] = 0;
   out_4527343688354192928[3] = 0;
   out_4527343688354192928[4] = 0;
   out_4527343688354192928[5] = 0;
   out_4527343688354192928[6] = 0;
   out_4527343688354192928[7] = 0;
   out_4527343688354192928[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3231158882167630390) {
  err_fun(nom_x, delta_x, out_3231158882167630390);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2255683443976328672) {
  inv_err_fun(nom_x, true_x, out_2255683443976328672);
}
void car_H_mod_fun(double *state, double *out_1538918716375221260) {
  H_mod_fun(state, out_1538918716375221260);
}
void car_f_fun(double *state, double dt, double *out_5181322862125731861) {
  f_fun(state,  dt, out_5181322862125731861);
}
void car_F_fun(double *state, double dt, double *out_5354287983213807462) {
  F_fun(state,  dt, out_5354287983213807462);
}
void car_h_25(double *state, double *unused, double *out_3994723713418228284) {
  h_25(state, unused, out_3994723713418228284);
}
void car_H_25(double *state, double *unused, double *out_4557989650231153356) {
  H_25(state, unused, out_4557989650231153356);
}
void car_h_24(double *state, double *unused, double *out_2483498575061758787) {
  h_24(state, unused, out_2483498575061758787);
}
void car_H_24(double *state, double *unused, double *out_1000064051428830220) {
  H_24(state, unused, out_1000064051428830220);
}
void car_h_30(double *state, double *unused, double *out_2776111512932122652) {
  h_30(state, unused, out_2776111512932122652);
}
void car_H_30(double *state, double *unused, double *out_2039656691723904729) {
  H_30(state, unused, out_2039656691723904729);
}
void car_h_26(double *state, double *unused, double *out_1832601660030460217) {
  h_26(state, unused, out_1832601660030460217);
}
void car_H_26(double *state, double *unused, double *out_8299492969105209580) {
  H_26(state, unused, out_8299492969105209580);
}
void car_h_27(double *state, double *unused, double *out_8996396981079073929) {
  h_27(state, unused, out_8996396981079073929);
}
void car_H_27(double *state, double *unused, double *out_183937379460038488) {
  H_27(state, unused, out_183937379460038488);
}
void car_h_29(double *state, double *unused, double *out_5812897564376455337) {
  h_29(state, unused, out_5812897564376455337);
}
void car_H_29(double *state, double *unused, double *out_1529425347409512545) {
  H_29(state, unused, out_1529425347409512545);
}
void car_h_28(double *state, double *unused, double *out_4828161063174369373) {
  h_28(state, unused, out_4828161063174369373);
}
void car_H_28(double *state, double *unused, double *out_6611824364479043119) {
  H_28(state, unused, out_6611824364479043119);
}
void car_h_31(double *state, double *unused, double *out_6688755103470650891) {
  h_31(state, unused, out_6688755103470650891);
}
void car_H_31(double *state, double *unused, double *out_4527343688354192928) {
  H_31(state, unused, out_4527343688354192928);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
