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
void err_fun(double *nom_x, double *delta_x, double *out_2414765271784937644) {
   out_2414765271784937644[0] = delta_x[0] + nom_x[0];
   out_2414765271784937644[1] = delta_x[1] + nom_x[1];
   out_2414765271784937644[2] = delta_x[2] + nom_x[2];
   out_2414765271784937644[3] = delta_x[3] + nom_x[3];
   out_2414765271784937644[4] = delta_x[4] + nom_x[4];
   out_2414765271784937644[5] = delta_x[5] + nom_x[5];
   out_2414765271784937644[6] = delta_x[6] + nom_x[6];
   out_2414765271784937644[7] = delta_x[7] + nom_x[7];
   out_2414765271784937644[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3704150315909089492) {
   out_3704150315909089492[0] = -nom_x[0] + true_x[0];
   out_3704150315909089492[1] = -nom_x[1] + true_x[1];
   out_3704150315909089492[2] = -nom_x[2] + true_x[2];
   out_3704150315909089492[3] = -nom_x[3] + true_x[3];
   out_3704150315909089492[4] = -nom_x[4] + true_x[4];
   out_3704150315909089492[5] = -nom_x[5] + true_x[5];
   out_3704150315909089492[6] = -nom_x[6] + true_x[6];
   out_3704150315909089492[7] = -nom_x[7] + true_x[7];
   out_3704150315909089492[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_814895361450768104) {
   out_814895361450768104[0] = 1.0;
   out_814895361450768104[1] = 0.0;
   out_814895361450768104[2] = 0.0;
   out_814895361450768104[3] = 0.0;
   out_814895361450768104[4] = 0.0;
   out_814895361450768104[5] = 0.0;
   out_814895361450768104[6] = 0.0;
   out_814895361450768104[7] = 0.0;
   out_814895361450768104[8] = 0.0;
   out_814895361450768104[9] = 0.0;
   out_814895361450768104[10] = 1.0;
   out_814895361450768104[11] = 0.0;
   out_814895361450768104[12] = 0.0;
   out_814895361450768104[13] = 0.0;
   out_814895361450768104[14] = 0.0;
   out_814895361450768104[15] = 0.0;
   out_814895361450768104[16] = 0.0;
   out_814895361450768104[17] = 0.0;
   out_814895361450768104[18] = 0.0;
   out_814895361450768104[19] = 0.0;
   out_814895361450768104[20] = 1.0;
   out_814895361450768104[21] = 0.0;
   out_814895361450768104[22] = 0.0;
   out_814895361450768104[23] = 0.0;
   out_814895361450768104[24] = 0.0;
   out_814895361450768104[25] = 0.0;
   out_814895361450768104[26] = 0.0;
   out_814895361450768104[27] = 0.0;
   out_814895361450768104[28] = 0.0;
   out_814895361450768104[29] = 0.0;
   out_814895361450768104[30] = 1.0;
   out_814895361450768104[31] = 0.0;
   out_814895361450768104[32] = 0.0;
   out_814895361450768104[33] = 0.0;
   out_814895361450768104[34] = 0.0;
   out_814895361450768104[35] = 0.0;
   out_814895361450768104[36] = 0.0;
   out_814895361450768104[37] = 0.0;
   out_814895361450768104[38] = 0.0;
   out_814895361450768104[39] = 0.0;
   out_814895361450768104[40] = 1.0;
   out_814895361450768104[41] = 0.0;
   out_814895361450768104[42] = 0.0;
   out_814895361450768104[43] = 0.0;
   out_814895361450768104[44] = 0.0;
   out_814895361450768104[45] = 0.0;
   out_814895361450768104[46] = 0.0;
   out_814895361450768104[47] = 0.0;
   out_814895361450768104[48] = 0.0;
   out_814895361450768104[49] = 0.0;
   out_814895361450768104[50] = 1.0;
   out_814895361450768104[51] = 0.0;
   out_814895361450768104[52] = 0.0;
   out_814895361450768104[53] = 0.0;
   out_814895361450768104[54] = 0.0;
   out_814895361450768104[55] = 0.0;
   out_814895361450768104[56] = 0.0;
   out_814895361450768104[57] = 0.0;
   out_814895361450768104[58] = 0.0;
   out_814895361450768104[59] = 0.0;
   out_814895361450768104[60] = 1.0;
   out_814895361450768104[61] = 0.0;
   out_814895361450768104[62] = 0.0;
   out_814895361450768104[63] = 0.0;
   out_814895361450768104[64] = 0.0;
   out_814895361450768104[65] = 0.0;
   out_814895361450768104[66] = 0.0;
   out_814895361450768104[67] = 0.0;
   out_814895361450768104[68] = 0.0;
   out_814895361450768104[69] = 0.0;
   out_814895361450768104[70] = 1.0;
   out_814895361450768104[71] = 0.0;
   out_814895361450768104[72] = 0.0;
   out_814895361450768104[73] = 0.0;
   out_814895361450768104[74] = 0.0;
   out_814895361450768104[75] = 0.0;
   out_814895361450768104[76] = 0.0;
   out_814895361450768104[77] = 0.0;
   out_814895361450768104[78] = 0.0;
   out_814895361450768104[79] = 0.0;
   out_814895361450768104[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1583533894033430000) {
   out_1583533894033430000[0] = state[0];
   out_1583533894033430000[1] = state[1];
   out_1583533894033430000[2] = state[2];
   out_1583533894033430000[3] = state[3];
   out_1583533894033430000[4] = state[4];
   out_1583533894033430000[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1583533894033430000[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1583533894033430000[7] = state[7];
   out_1583533894033430000[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5522342914271432502) {
   out_5522342914271432502[0] = 1;
   out_5522342914271432502[1] = 0;
   out_5522342914271432502[2] = 0;
   out_5522342914271432502[3] = 0;
   out_5522342914271432502[4] = 0;
   out_5522342914271432502[5] = 0;
   out_5522342914271432502[6] = 0;
   out_5522342914271432502[7] = 0;
   out_5522342914271432502[8] = 0;
   out_5522342914271432502[9] = 0;
   out_5522342914271432502[10] = 1;
   out_5522342914271432502[11] = 0;
   out_5522342914271432502[12] = 0;
   out_5522342914271432502[13] = 0;
   out_5522342914271432502[14] = 0;
   out_5522342914271432502[15] = 0;
   out_5522342914271432502[16] = 0;
   out_5522342914271432502[17] = 0;
   out_5522342914271432502[18] = 0;
   out_5522342914271432502[19] = 0;
   out_5522342914271432502[20] = 1;
   out_5522342914271432502[21] = 0;
   out_5522342914271432502[22] = 0;
   out_5522342914271432502[23] = 0;
   out_5522342914271432502[24] = 0;
   out_5522342914271432502[25] = 0;
   out_5522342914271432502[26] = 0;
   out_5522342914271432502[27] = 0;
   out_5522342914271432502[28] = 0;
   out_5522342914271432502[29] = 0;
   out_5522342914271432502[30] = 1;
   out_5522342914271432502[31] = 0;
   out_5522342914271432502[32] = 0;
   out_5522342914271432502[33] = 0;
   out_5522342914271432502[34] = 0;
   out_5522342914271432502[35] = 0;
   out_5522342914271432502[36] = 0;
   out_5522342914271432502[37] = 0;
   out_5522342914271432502[38] = 0;
   out_5522342914271432502[39] = 0;
   out_5522342914271432502[40] = 1;
   out_5522342914271432502[41] = 0;
   out_5522342914271432502[42] = 0;
   out_5522342914271432502[43] = 0;
   out_5522342914271432502[44] = 0;
   out_5522342914271432502[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5522342914271432502[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5522342914271432502[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5522342914271432502[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5522342914271432502[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5522342914271432502[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5522342914271432502[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5522342914271432502[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5522342914271432502[53] = -9.8100000000000005*dt;
   out_5522342914271432502[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5522342914271432502[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5522342914271432502[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5522342914271432502[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5522342914271432502[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5522342914271432502[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5522342914271432502[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5522342914271432502[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5522342914271432502[62] = 0;
   out_5522342914271432502[63] = 0;
   out_5522342914271432502[64] = 0;
   out_5522342914271432502[65] = 0;
   out_5522342914271432502[66] = 0;
   out_5522342914271432502[67] = 0;
   out_5522342914271432502[68] = 0;
   out_5522342914271432502[69] = 0;
   out_5522342914271432502[70] = 1;
   out_5522342914271432502[71] = 0;
   out_5522342914271432502[72] = 0;
   out_5522342914271432502[73] = 0;
   out_5522342914271432502[74] = 0;
   out_5522342914271432502[75] = 0;
   out_5522342914271432502[76] = 0;
   out_5522342914271432502[77] = 0;
   out_5522342914271432502[78] = 0;
   out_5522342914271432502[79] = 0;
   out_5522342914271432502[80] = 1;
}
void h_25(double *state, double *unused, double *out_142737944331781953) {
   out_142737944331781953[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3979935481477101005) {
   out_3979935481477101005[0] = 0;
   out_3979935481477101005[1] = 0;
   out_3979935481477101005[2] = 0;
   out_3979935481477101005[3] = 0;
   out_3979935481477101005[4] = 0;
   out_3979935481477101005[5] = 0;
   out_3979935481477101005[6] = 1;
   out_3979935481477101005[7] = 0;
   out_3979935481477101005[8] = 0;
}
void h_24(double *state, double *unused, double *out_9041743034093230167) {
   out_9041743034093230167[0] = state[4];
   out_9041743034093230167[1] = state[5];
}
void H_24(double *state, double *unused, double *out_6152585080482600571) {
   out_6152585080482600571[0] = 0;
   out_6152585080482600571[1] = 0;
   out_6152585080482600571[2] = 0;
   out_6152585080482600571[3] = 0;
   out_6152585080482600571[4] = 1;
   out_6152585080482600571[5] = 0;
   out_6152585080482600571[6] = 0;
   out_6152585080482600571[7] = 0;
   out_6152585080482600571[8] = 0;
   out_6152585080482600571[9] = 0;
   out_6152585080482600571[10] = 0;
   out_6152585080482600571[11] = 0;
   out_6152585080482600571[12] = 0;
   out_6152585080482600571[13] = 0;
   out_6152585080482600571[14] = 1;
   out_6152585080482600571[15] = 0;
   out_6152585080482600571[16] = 0;
   out_6152585080482600571[17] = 0;
}
void h_30(double *state, double *unused, double *out_458680213281581601) {
   out_458680213281581601[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1461602522969852378) {
   out_1461602522969852378[0] = 0;
   out_1461602522969852378[1] = 0;
   out_1461602522969852378[2] = 0;
   out_1461602522969852378[3] = 0;
   out_1461602522969852378[4] = 1;
   out_1461602522969852378[5] = 0;
   out_1461602522969852378[6] = 0;
   out_1461602522969852378[7] = 0;
   out_1461602522969852378[8] = 0;
}
void h_26(double *state, double *unused, double *out_5460353480942427246) {
   out_5460353480942427246[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7721438800351157229) {
   out_7721438800351157229[0] = 0;
   out_7721438800351157229[1] = 0;
   out_7721438800351157229[2] = 0;
   out_7721438800351157229[3] = 0;
   out_7721438800351157229[4] = 0;
   out_7721438800351157229[5] = 0;
   out_7721438800351157229[6] = 0;
   out_7721438800351157229[7] = 1;
   out_7721438800351157229[8] = 0;
}
void h_27(double *state, double *unused, double *out_5144411211992627598) {
   out_5144411211992627598[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7764348950304417502) {
   out_7764348950304417502[0] = 0;
   out_7764348950304417502[1] = 0;
   out_7764348950304417502[2] = 0;
   out_7764348950304417502[3] = 1;
   out_7764348950304417502[4] = 0;
   out_7764348950304417502[5] = 0;
   out_7764348950304417502[6] = 0;
   out_7764348950304417502[7] = 0;
   out_7764348950304417502[8] = 0;
}
void h_29(double *state, double *unused, double *out_9006094688421129250) {
   out_9006094688421129250[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7997400467290317019) {
   out_7997400467290317019[0] = 0;
   out_7997400467290317019[1] = 1;
   out_7997400467290317019[2] = 0;
   out_7997400467290317019[3] = 0;
   out_7997400467290317019[4] = 0;
   out_7997400467290317019[5] = 0;
   out_7997400467290317019[6] = 0;
   out_7997400467290317019[7] = 0;
   out_7997400467290317019[8] = 0;
}
void h_28(double *state, double *unused, double *out_8076425645881599071) {
   out_8076425645881599071[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6033770195724990768) {
   out_6033770195724990768[0] = 1;
   out_6033770195724990768[1] = 0;
   out_6033770195724990768[2] = 0;
   out_6033770195724990768[3] = 0;
   out_6033770195724990768[4] = 0;
   out_6033770195724990768[5] = 0;
   out_6033770195724990768[6] = 0;
   out_6033770195724990768[7] = 0;
   out_6033770195724990768[8] = 0;
}
void h_31(double *state, double *unused, double *out_7906003201152454394) {
   out_7906003201152454394[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8347646902584508705) {
   out_8347646902584508705[0] = 0;
   out_8347646902584508705[1] = 0;
   out_8347646902584508705[2] = 0;
   out_8347646902584508705[3] = 0;
   out_8347646902584508705[4] = 0;
   out_8347646902584508705[5] = 0;
   out_8347646902584508705[6] = 0;
   out_8347646902584508705[7] = 0;
   out_8347646902584508705[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2414765271784937644) {
  err_fun(nom_x, delta_x, out_2414765271784937644);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3704150315909089492) {
  inv_err_fun(nom_x, true_x, out_3704150315909089492);
}
void car_H_mod_fun(double *state, double *out_814895361450768104) {
  H_mod_fun(state, out_814895361450768104);
}
void car_f_fun(double *state, double dt, double *out_1583533894033430000) {
  f_fun(state,  dt, out_1583533894033430000);
}
void car_F_fun(double *state, double dt, double *out_5522342914271432502) {
  F_fun(state,  dt, out_5522342914271432502);
}
void car_h_25(double *state, double *unused, double *out_142737944331781953) {
  h_25(state, unused, out_142737944331781953);
}
void car_H_25(double *state, double *unused, double *out_3979935481477101005) {
  H_25(state, unused, out_3979935481477101005);
}
void car_h_24(double *state, double *unused, double *out_9041743034093230167) {
  h_24(state, unused, out_9041743034093230167);
}
void car_H_24(double *state, double *unused, double *out_6152585080482600571) {
  H_24(state, unused, out_6152585080482600571);
}
void car_h_30(double *state, double *unused, double *out_458680213281581601) {
  h_30(state, unused, out_458680213281581601);
}
void car_H_30(double *state, double *unused, double *out_1461602522969852378) {
  H_30(state, unused, out_1461602522969852378);
}
void car_h_26(double *state, double *unused, double *out_5460353480942427246) {
  h_26(state, unused, out_5460353480942427246);
}
void car_H_26(double *state, double *unused, double *out_7721438800351157229) {
  H_26(state, unused, out_7721438800351157229);
}
void car_h_27(double *state, double *unused, double *out_5144411211992627598) {
  h_27(state, unused, out_5144411211992627598);
}
void car_H_27(double *state, double *unused, double *out_7764348950304417502) {
  H_27(state, unused, out_7764348950304417502);
}
void car_h_29(double *state, double *unused, double *out_9006094688421129250) {
  h_29(state, unused, out_9006094688421129250);
}
void car_H_29(double *state, double *unused, double *out_7997400467290317019) {
  H_29(state, unused, out_7997400467290317019);
}
void car_h_28(double *state, double *unused, double *out_8076425645881599071) {
  h_28(state, unused, out_8076425645881599071);
}
void car_H_28(double *state, double *unused, double *out_6033770195724990768) {
  H_28(state, unused, out_6033770195724990768);
}
void car_h_31(double *state, double *unused, double *out_7906003201152454394) {
  h_31(state, unused, out_7906003201152454394);
}
void car_H_31(double *state, double *unused, double *out_8347646902584508705) {
  H_31(state, unused, out_8347646902584508705);
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
