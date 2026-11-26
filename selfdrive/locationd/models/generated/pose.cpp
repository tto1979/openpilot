#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8400786688062439867) {
   out_8400786688062439867[0] = delta_x[0] + nom_x[0];
   out_8400786688062439867[1] = delta_x[1] + nom_x[1];
   out_8400786688062439867[2] = delta_x[2] + nom_x[2];
   out_8400786688062439867[3] = delta_x[3] + nom_x[3];
   out_8400786688062439867[4] = delta_x[4] + nom_x[4];
   out_8400786688062439867[5] = delta_x[5] + nom_x[5];
   out_8400786688062439867[6] = delta_x[6] + nom_x[6];
   out_8400786688062439867[7] = delta_x[7] + nom_x[7];
   out_8400786688062439867[8] = delta_x[8] + nom_x[8];
   out_8400786688062439867[9] = delta_x[9] + nom_x[9];
   out_8400786688062439867[10] = delta_x[10] + nom_x[10];
   out_8400786688062439867[11] = delta_x[11] + nom_x[11];
   out_8400786688062439867[12] = delta_x[12] + nom_x[12];
   out_8400786688062439867[13] = delta_x[13] + nom_x[13];
   out_8400786688062439867[14] = delta_x[14] + nom_x[14];
   out_8400786688062439867[15] = delta_x[15] + nom_x[15];
   out_8400786688062439867[16] = delta_x[16] + nom_x[16];
   out_8400786688062439867[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5956079812122370151) {
   out_5956079812122370151[0] = -nom_x[0] + true_x[0];
   out_5956079812122370151[1] = -nom_x[1] + true_x[1];
   out_5956079812122370151[2] = -nom_x[2] + true_x[2];
   out_5956079812122370151[3] = -nom_x[3] + true_x[3];
   out_5956079812122370151[4] = -nom_x[4] + true_x[4];
   out_5956079812122370151[5] = -nom_x[5] + true_x[5];
   out_5956079812122370151[6] = -nom_x[6] + true_x[6];
   out_5956079812122370151[7] = -nom_x[7] + true_x[7];
   out_5956079812122370151[8] = -nom_x[8] + true_x[8];
   out_5956079812122370151[9] = -nom_x[9] + true_x[9];
   out_5956079812122370151[10] = -nom_x[10] + true_x[10];
   out_5956079812122370151[11] = -nom_x[11] + true_x[11];
   out_5956079812122370151[12] = -nom_x[12] + true_x[12];
   out_5956079812122370151[13] = -nom_x[13] + true_x[13];
   out_5956079812122370151[14] = -nom_x[14] + true_x[14];
   out_5956079812122370151[15] = -nom_x[15] + true_x[15];
   out_5956079812122370151[16] = -nom_x[16] + true_x[16];
   out_5956079812122370151[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8542454711932240757) {
   out_8542454711932240757[0] = 1.0;
   out_8542454711932240757[1] = 0.0;
   out_8542454711932240757[2] = 0.0;
   out_8542454711932240757[3] = 0.0;
   out_8542454711932240757[4] = 0.0;
   out_8542454711932240757[5] = 0.0;
   out_8542454711932240757[6] = 0.0;
   out_8542454711932240757[7] = 0.0;
   out_8542454711932240757[8] = 0.0;
   out_8542454711932240757[9] = 0.0;
   out_8542454711932240757[10] = 0.0;
   out_8542454711932240757[11] = 0.0;
   out_8542454711932240757[12] = 0.0;
   out_8542454711932240757[13] = 0.0;
   out_8542454711932240757[14] = 0.0;
   out_8542454711932240757[15] = 0.0;
   out_8542454711932240757[16] = 0.0;
   out_8542454711932240757[17] = 0.0;
   out_8542454711932240757[18] = 0.0;
   out_8542454711932240757[19] = 1.0;
   out_8542454711932240757[20] = 0.0;
   out_8542454711932240757[21] = 0.0;
   out_8542454711932240757[22] = 0.0;
   out_8542454711932240757[23] = 0.0;
   out_8542454711932240757[24] = 0.0;
   out_8542454711932240757[25] = 0.0;
   out_8542454711932240757[26] = 0.0;
   out_8542454711932240757[27] = 0.0;
   out_8542454711932240757[28] = 0.0;
   out_8542454711932240757[29] = 0.0;
   out_8542454711932240757[30] = 0.0;
   out_8542454711932240757[31] = 0.0;
   out_8542454711932240757[32] = 0.0;
   out_8542454711932240757[33] = 0.0;
   out_8542454711932240757[34] = 0.0;
   out_8542454711932240757[35] = 0.0;
   out_8542454711932240757[36] = 0.0;
   out_8542454711932240757[37] = 0.0;
   out_8542454711932240757[38] = 1.0;
   out_8542454711932240757[39] = 0.0;
   out_8542454711932240757[40] = 0.0;
   out_8542454711932240757[41] = 0.0;
   out_8542454711932240757[42] = 0.0;
   out_8542454711932240757[43] = 0.0;
   out_8542454711932240757[44] = 0.0;
   out_8542454711932240757[45] = 0.0;
   out_8542454711932240757[46] = 0.0;
   out_8542454711932240757[47] = 0.0;
   out_8542454711932240757[48] = 0.0;
   out_8542454711932240757[49] = 0.0;
   out_8542454711932240757[50] = 0.0;
   out_8542454711932240757[51] = 0.0;
   out_8542454711932240757[52] = 0.0;
   out_8542454711932240757[53] = 0.0;
   out_8542454711932240757[54] = 0.0;
   out_8542454711932240757[55] = 0.0;
   out_8542454711932240757[56] = 0.0;
   out_8542454711932240757[57] = 1.0;
   out_8542454711932240757[58] = 0.0;
   out_8542454711932240757[59] = 0.0;
   out_8542454711932240757[60] = 0.0;
   out_8542454711932240757[61] = 0.0;
   out_8542454711932240757[62] = 0.0;
   out_8542454711932240757[63] = 0.0;
   out_8542454711932240757[64] = 0.0;
   out_8542454711932240757[65] = 0.0;
   out_8542454711932240757[66] = 0.0;
   out_8542454711932240757[67] = 0.0;
   out_8542454711932240757[68] = 0.0;
   out_8542454711932240757[69] = 0.0;
   out_8542454711932240757[70] = 0.0;
   out_8542454711932240757[71] = 0.0;
   out_8542454711932240757[72] = 0.0;
   out_8542454711932240757[73] = 0.0;
   out_8542454711932240757[74] = 0.0;
   out_8542454711932240757[75] = 0.0;
   out_8542454711932240757[76] = 1.0;
   out_8542454711932240757[77] = 0.0;
   out_8542454711932240757[78] = 0.0;
   out_8542454711932240757[79] = 0.0;
   out_8542454711932240757[80] = 0.0;
   out_8542454711932240757[81] = 0.0;
   out_8542454711932240757[82] = 0.0;
   out_8542454711932240757[83] = 0.0;
   out_8542454711932240757[84] = 0.0;
   out_8542454711932240757[85] = 0.0;
   out_8542454711932240757[86] = 0.0;
   out_8542454711932240757[87] = 0.0;
   out_8542454711932240757[88] = 0.0;
   out_8542454711932240757[89] = 0.0;
   out_8542454711932240757[90] = 0.0;
   out_8542454711932240757[91] = 0.0;
   out_8542454711932240757[92] = 0.0;
   out_8542454711932240757[93] = 0.0;
   out_8542454711932240757[94] = 0.0;
   out_8542454711932240757[95] = 1.0;
   out_8542454711932240757[96] = 0.0;
   out_8542454711932240757[97] = 0.0;
   out_8542454711932240757[98] = 0.0;
   out_8542454711932240757[99] = 0.0;
   out_8542454711932240757[100] = 0.0;
   out_8542454711932240757[101] = 0.0;
   out_8542454711932240757[102] = 0.0;
   out_8542454711932240757[103] = 0.0;
   out_8542454711932240757[104] = 0.0;
   out_8542454711932240757[105] = 0.0;
   out_8542454711932240757[106] = 0.0;
   out_8542454711932240757[107] = 0.0;
   out_8542454711932240757[108] = 0.0;
   out_8542454711932240757[109] = 0.0;
   out_8542454711932240757[110] = 0.0;
   out_8542454711932240757[111] = 0.0;
   out_8542454711932240757[112] = 0.0;
   out_8542454711932240757[113] = 0.0;
   out_8542454711932240757[114] = 1.0;
   out_8542454711932240757[115] = 0.0;
   out_8542454711932240757[116] = 0.0;
   out_8542454711932240757[117] = 0.0;
   out_8542454711932240757[118] = 0.0;
   out_8542454711932240757[119] = 0.0;
   out_8542454711932240757[120] = 0.0;
   out_8542454711932240757[121] = 0.0;
   out_8542454711932240757[122] = 0.0;
   out_8542454711932240757[123] = 0.0;
   out_8542454711932240757[124] = 0.0;
   out_8542454711932240757[125] = 0.0;
   out_8542454711932240757[126] = 0.0;
   out_8542454711932240757[127] = 0.0;
   out_8542454711932240757[128] = 0.0;
   out_8542454711932240757[129] = 0.0;
   out_8542454711932240757[130] = 0.0;
   out_8542454711932240757[131] = 0.0;
   out_8542454711932240757[132] = 0.0;
   out_8542454711932240757[133] = 1.0;
   out_8542454711932240757[134] = 0.0;
   out_8542454711932240757[135] = 0.0;
   out_8542454711932240757[136] = 0.0;
   out_8542454711932240757[137] = 0.0;
   out_8542454711932240757[138] = 0.0;
   out_8542454711932240757[139] = 0.0;
   out_8542454711932240757[140] = 0.0;
   out_8542454711932240757[141] = 0.0;
   out_8542454711932240757[142] = 0.0;
   out_8542454711932240757[143] = 0.0;
   out_8542454711932240757[144] = 0.0;
   out_8542454711932240757[145] = 0.0;
   out_8542454711932240757[146] = 0.0;
   out_8542454711932240757[147] = 0.0;
   out_8542454711932240757[148] = 0.0;
   out_8542454711932240757[149] = 0.0;
   out_8542454711932240757[150] = 0.0;
   out_8542454711932240757[151] = 0.0;
   out_8542454711932240757[152] = 1.0;
   out_8542454711932240757[153] = 0.0;
   out_8542454711932240757[154] = 0.0;
   out_8542454711932240757[155] = 0.0;
   out_8542454711932240757[156] = 0.0;
   out_8542454711932240757[157] = 0.0;
   out_8542454711932240757[158] = 0.0;
   out_8542454711932240757[159] = 0.0;
   out_8542454711932240757[160] = 0.0;
   out_8542454711932240757[161] = 0.0;
   out_8542454711932240757[162] = 0.0;
   out_8542454711932240757[163] = 0.0;
   out_8542454711932240757[164] = 0.0;
   out_8542454711932240757[165] = 0.0;
   out_8542454711932240757[166] = 0.0;
   out_8542454711932240757[167] = 0.0;
   out_8542454711932240757[168] = 0.0;
   out_8542454711932240757[169] = 0.0;
   out_8542454711932240757[170] = 0.0;
   out_8542454711932240757[171] = 1.0;
   out_8542454711932240757[172] = 0.0;
   out_8542454711932240757[173] = 0.0;
   out_8542454711932240757[174] = 0.0;
   out_8542454711932240757[175] = 0.0;
   out_8542454711932240757[176] = 0.0;
   out_8542454711932240757[177] = 0.0;
   out_8542454711932240757[178] = 0.0;
   out_8542454711932240757[179] = 0.0;
   out_8542454711932240757[180] = 0.0;
   out_8542454711932240757[181] = 0.0;
   out_8542454711932240757[182] = 0.0;
   out_8542454711932240757[183] = 0.0;
   out_8542454711932240757[184] = 0.0;
   out_8542454711932240757[185] = 0.0;
   out_8542454711932240757[186] = 0.0;
   out_8542454711932240757[187] = 0.0;
   out_8542454711932240757[188] = 0.0;
   out_8542454711932240757[189] = 0.0;
   out_8542454711932240757[190] = 1.0;
   out_8542454711932240757[191] = 0.0;
   out_8542454711932240757[192] = 0.0;
   out_8542454711932240757[193] = 0.0;
   out_8542454711932240757[194] = 0.0;
   out_8542454711932240757[195] = 0.0;
   out_8542454711932240757[196] = 0.0;
   out_8542454711932240757[197] = 0.0;
   out_8542454711932240757[198] = 0.0;
   out_8542454711932240757[199] = 0.0;
   out_8542454711932240757[200] = 0.0;
   out_8542454711932240757[201] = 0.0;
   out_8542454711932240757[202] = 0.0;
   out_8542454711932240757[203] = 0.0;
   out_8542454711932240757[204] = 0.0;
   out_8542454711932240757[205] = 0.0;
   out_8542454711932240757[206] = 0.0;
   out_8542454711932240757[207] = 0.0;
   out_8542454711932240757[208] = 0.0;
   out_8542454711932240757[209] = 1.0;
   out_8542454711932240757[210] = 0.0;
   out_8542454711932240757[211] = 0.0;
   out_8542454711932240757[212] = 0.0;
   out_8542454711932240757[213] = 0.0;
   out_8542454711932240757[214] = 0.0;
   out_8542454711932240757[215] = 0.0;
   out_8542454711932240757[216] = 0.0;
   out_8542454711932240757[217] = 0.0;
   out_8542454711932240757[218] = 0.0;
   out_8542454711932240757[219] = 0.0;
   out_8542454711932240757[220] = 0.0;
   out_8542454711932240757[221] = 0.0;
   out_8542454711932240757[222] = 0.0;
   out_8542454711932240757[223] = 0.0;
   out_8542454711932240757[224] = 0.0;
   out_8542454711932240757[225] = 0.0;
   out_8542454711932240757[226] = 0.0;
   out_8542454711932240757[227] = 0.0;
   out_8542454711932240757[228] = 1.0;
   out_8542454711932240757[229] = 0.0;
   out_8542454711932240757[230] = 0.0;
   out_8542454711932240757[231] = 0.0;
   out_8542454711932240757[232] = 0.0;
   out_8542454711932240757[233] = 0.0;
   out_8542454711932240757[234] = 0.0;
   out_8542454711932240757[235] = 0.0;
   out_8542454711932240757[236] = 0.0;
   out_8542454711932240757[237] = 0.0;
   out_8542454711932240757[238] = 0.0;
   out_8542454711932240757[239] = 0.0;
   out_8542454711932240757[240] = 0.0;
   out_8542454711932240757[241] = 0.0;
   out_8542454711932240757[242] = 0.0;
   out_8542454711932240757[243] = 0.0;
   out_8542454711932240757[244] = 0.0;
   out_8542454711932240757[245] = 0.0;
   out_8542454711932240757[246] = 0.0;
   out_8542454711932240757[247] = 1.0;
   out_8542454711932240757[248] = 0.0;
   out_8542454711932240757[249] = 0.0;
   out_8542454711932240757[250] = 0.0;
   out_8542454711932240757[251] = 0.0;
   out_8542454711932240757[252] = 0.0;
   out_8542454711932240757[253] = 0.0;
   out_8542454711932240757[254] = 0.0;
   out_8542454711932240757[255] = 0.0;
   out_8542454711932240757[256] = 0.0;
   out_8542454711932240757[257] = 0.0;
   out_8542454711932240757[258] = 0.0;
   out_8542454711932240757[259] = 0.0;
   out_8542454711932240757[260] = 0.0;
   out_8542454711932240757[261] = 0.0;
   out_8542454711932240757[262] = 0.0;
   out_8542454711932240757[263] = 0.0;
   out_8542454711932240757[264] = 0.0;
   out_8542454711932240757[265] = 0.0;
   out_8542454711932240757[266] = 1.0;
   out_8542454711932240757[267] = 0.0;
   out_8542454711932240757[268] = 0.0;
   out_8542454711932240757[269] = 0.0;
   out_8542454711932240757[270] = 0.0;
   out_8542454711932240757[271] = 0.0;
   out_8542454711932240757[272] = 0.0;
   out_8542454711932240757[273] = 0.0;
   out_8542454711932240757[274] = 0.0;
   out_8542454711932240757[275] = 0.0;
   out_8542454711932240757[276] = 0.0;
   out_8542454711932240757[277] = 0.0;
   out_8542454711932240757[278] = 0.0;
   out_8542454711932240757[279] = 0.0;
   out_8542454711932240757[280] = 0.0;
   out_8542454711932240757[281] = 0.0;
   out_8542454711932240757[282] = 0.0;
   out_8542454711932240757[283] = 0.0;
   out_8542454711932240757[284] = 0.0;
   out_8542454711932240757[285] = 1.0;
   out_8542454711932240757[286] = 0.0;
   out_8542454711932240757[287] = 0.0;
   out_8542454711932240757[288] = 0.0;
   out_8542454711932240757[289] = 0.0;
   out_8542454711932240757[290] = 0.0;
   out_8542454711932240757[291] = 0.0;
   out_8542454711932240757[292] = 0.0;
   out_8542454711932240757[293] = 0.0;
   out_8542454711932240757[294] = 0.0;
   out_8542454711932240757[295] = 0.0;
   out_8542454711932240757[296] = 0.0;
   out_8542454711932240757[297] = 0.0;
   out_8542454711932240757[298] = 0.0;
   out_8542454711932240757[299] = 0.0;
   out_8542454711932240757[300] = 0.0;
   out_8542454711932240757[301] = 0.0;
   out_8542454711932240757[302] = 0.0;
   out_8542454711932240757[303] = 0.0;
   out_8542454711932240757[304] = 1.0;
   out_8542454711932240757[305] = 0.0;
   out_8542454711932240757[306] = 0.0;
   out_8542454711932240757[307] = 0.0;
   out_8542454711932240757[308] = 0.0;
   out_8542454711932240757[309] = 0.0;
   out_8542454711932240757[310] = 0.0;
   out_8542454711932240757[311] = 0.0;
   out_8542454711932240757[312] = 0.0;
   out_8542454711932240757[313] = 0.0;
   out_8542454711932240757[314] = 0.0;
   out_8542454711932240757[315] = 0.0;
   out_8542454711932240757[316] = 0.0;
   out_8542454711932240757[317] = 0.0;
   out_8542454711932240757[318] = 0.0;
   out_8542454711932240757[319] = 0.0;
   out_8542454711932240757[320] = 0.0;
   out_8542454711932240757[321] = 0.0;
   out_8542454711932240757[322] = 0.0;
   out_8542454711932240757[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6490866571001637660) {
   out_6490866571001637660[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6490866571001637660[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6490866571001637660[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6490866571001637660[3] = dt*state[12] + state[3];
   out_6490866571001637660[4] = dt*state[13] + state[4];
   out_6490866571001637660[5] = dt*state[14] + state[5];
   out_6490866571001637660[6] = state[6];
   out_6490866571001637660[7] = state[7];
   out_6490866571001637660[8] = state[8];
   out_6490866571001637660[9] = state[9];
   out_6490866571001637660[10] = state[10];
   out_6490866571001637660[11] = state[11];
   out_6490866571001637660[12] = state[12];
   out_6490866571001637660[13] = state[13];
   out_6490866571001637660[14] = state[14];
   out_6490866571001637660[15] = state[15];
   out_6490866571001637660[16] = state[16];
   out_6490866571001637660[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4543420401477853170) {
   out_4543420401477853170[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4543420401477853170[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4543420401477853170[2] = 0;
   out_4543420401477853170[3] = 0;
   out_4543420401477853170[4] = 0;
   out_4543420401477853170[5] = 0;
   out_4543420401477853170[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4543420401477853170[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4543420401477853170[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4543420401477853170[9] = 0;
   out_4543420401477853170[10] = 0;
   out_4543420401477853170[11] = 0;
   out_4543420401477853170[12] = 0;
   out_4543420401477853170[13] = 0;
   out_4543420401477853170[14] = 0;
   out_4543420401477853170[15] = 0;
   out_4543420401477853170[16] = 0;
   out_4543420401477853170[17] = 0;
   out_4543420401477853170[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4543420401477853170[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4543420401477853170[20] = 0;
   out_4543420401477853170[21] = 0;
   out_4543420401477853170[22] = 0;
   out_4543420401477853170[23] = 0;
   out_4543420401477853170[24] = 0;
   out_4543420401477853170[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4543420401477853170[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4543420401477853170[27] = 0;
   out_4543420401477853170[28] = 0;
   out_4543420401477853170[29] = 0;
   out_4543420401477853170[30] = 0;
   out_4543420401477853170[31] = 0;
   out_4543420401477853170[32] = 0;
   out_4543420401477853170[33] = 0;
   out_4543420401477853170[34] = 0;
   out_4543420401477853170[35] = 0;
   out_4543420401477853170[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4543420401477853170[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4543420401477853170[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4543420401477853170[39] = 0;
   out_4543420401477853170[40] = 0;
   out_4543420401477853170[41] = 0;
   out_4543420401477853170[42] = 0;
   out_4543420401477853170[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4543420401477853170[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4543420401477853170[45] = 0;
   out_4543420401477853170[46] = 0;
   out_4543420401477853170[47] = 0;
   out_4543420401477853170[48] = 0;
   out_4543420401477853170[49] = 0;
   out_4543420401477853170[50] = 0;
   out_4543420401477853170[51] = 0;
   out_4543420401477853170[52] = 0;
   out_4543420401477853170[53] = 0;
   out_4543420401477853170[54] = 0;
   out_4543420401477853170[55] = 0;
   out_4543420401477853170[56] = 0;
   out_4543420401477853170[57] = 1;
   out_4543420401477853170[58] = 0;
   out_4543420401477853170[59] = 0;
   out_4543420401477853170[60] = 0;
   out_4543420401477853170[61] = 0;
   out_4543420401477853170[62] = 0;
   out_4543420401477853170[63] = 0;
   out_4543420401477853170[64] = 0;
   out_4543420401477853170[65] = 0;
   out_4543420401477853170[66] = dt;
   out_4543420401477853170[67] = 0;
   out_4543420401477853170[68] = 0;
   out_4543420401477853170[69] = 0;
   out_4543420401477853170[70] = 0;
   out_4543420401477853170[71] = 0;
   out_4543420401477853170[72] = 0;
   out_4543420401477853170[73] = 0;
   out_4543420401477853170[74] = 0;
   out_4543420401477853170[75] = 0;
   out_4543420401477853170[76] = 1;
   out_4543420401477853170[77] = 0;
   out_4543420401477853170[78] = 0;
   out_4543420401477853170[79] = 0;
   out_4543420401477853170[80] = 0;
   out_4543420401477853170[81] = 0;
   out_4543420401477853170[82] = 0;
   out_4543420401477853170[83] = 0;
   out_4543420401477853170[84] = 0;
   out_4543420401477853170[85] = dt;
   out_4543420401477853170[86] = 0;
   out_4543420401477853170[87] = 0;
   out_4543420401477853170[88] = 0;
   out_4543420401477853170[89] = 0;
   out_4543420401477853170[90] = 0;
   out_4543420401477853170[91] = 0;
   out_4543420401477853170[92] = 0;
   out_4543420401477853170[93] = 0;
   out_4543420401477853170[94] = 0;
   out_4543420401477853170[95] = 1;
   out_4543420401477853170[96] = 0;
   out_4543420401477853170[97] = 0;
   out_4543420401477853170[98] = 0;
   out_4543420401477853170[99] = 0;
   out_4543420401477853170[100] = 0;
   out_4543420401477853170[101] = 0;
   out_4543420401477853170[102] = 0;
   out_4543420401477853170[103] = 0;
   out_4543420401477853170[104] = dt;
   out_4543420401477853170[105] = 0;
   out_4543420401477853170[106] = 0;
   out_4543420401477853170[107] = 0;
   out_4543420401477853170[108] = 0;
   out_4543420401477853170[109] = 0;
   out_4543420401477853170[110] = 0;
   out_4543420401477853170[111] = 0;
   out_4543420401477853170[112] = 0;
   out_4543420401477853170[113] = 0;
   out_4543420401477853170[114] = 1;
   out_4543420401477853170[115] = 0;
   out_4543420401477853170[116] = 0;
   out_4543420401477853170[117] = 0;
   out_4543420401477853170[118] = 0;
   out_4543420401477853170[119] = 0;
   out_4543420401477853170[120] = 0;
   out_4543420401477853170[121] = 0;
   out_4543420401477853170[122] = 0;
   out_4543420401477853170[123] = 0;
   out_4543420401477853170[124] = 0;
   out_4543420401477853170[125] = 0;
   out_4543420401477853170[126] = 0;
   out_4543420401477853170[127] = 0;
   out_4543420401477853170[128] = 0;
   out_4543420401477853170[129] = 0;
   out_4543420401477853170[130] = 0;
   out_4543420401477853170[131] = 0;
   out_4543420401477853170[132] = 0;
   out_4543420401477853170[133] = 1;
   out_4543420401477853170[134] = 0;
   out_4543420401477853170[135] = 0;
   out_4543420401477853170[136] = 0;
   out_4543420401477853170[137] = 0;
   out_4543420401477853170[138] = 0;
   out_4543420401477853170[139] = 0;
   out_4543420401477853170[140] = 0;
   out_4543420401477853170[141] = 0;
   out_4543420401477853170[142] = 0;
   out_4543420401477853170[143] = 0;
   out_4543420401477853170[144] = 0;
   out_4543420401477853170[145] = 0;
   out_4543420401477853170[146] = 0;
   out_4543420401477853170[147] = 0;
   out_4543420401477853170[148] = 0;
   out_4543420401477853170[149] = 0;
   out_4543420401477853170[150] = 0;
   out_4543420401477853170[151] = 0;
   out_4543420401477853170[152] = 1;
   out_4543420401477853170[153] = 0;
   out_4543420401477853170[154] = 0;
   out_4543420401477853170[155] = 0;
   out_4543420401477853170[156] = 0;
   out_4543420401477853170[157] = 0;
   out_4543420401477853170[158] = 0;
   out_4543420401477853170[159] = 0;
   out_4543420401477853170[160] = 0;
   out_4543420401477853170[161] = 0;
   out_4543420401477853170[162] = 0;
   out_4543420401477853170[163] = 0;
   out_4543420401477853170[164] = 0;
   out_4543420401477853170[165] = 0;
   out_4543420401477853170[166] = 0;
   out_4543420401477853170[167] = 0;
   out_4543420401477853170[168] = 0;
   out_4543420401477853170[169] = 0;
   out_4543420401477853170[170] = 0;
   out_4543420401477853170[171] = 1;
   out_4543420401477853170[172] = 0;
   out_4543420401477853170[173] = 0;
   out_4543420401477853170[174] = 0;
   out_4543420401477853170[175] = 0;
   out_4543420401477853170[176] = 0;
   out_4543420401477853170[177] = 0;
   out_4543420401477853170[178] = 0;
   out_4543420401477853170[179] = 0;
   out_4543420401477853170[180] = 0;
   out_4543420401477853170[181] = 0;
   out_4543420401477853170[182] = 0;
   out_4543420401477853170[183] = 0;
   out_4543420401477853170[184] = 0;
   out_4543420401477853170[185] = 0;
   out_4543420401477853170[186] = 0;
   out_4543420401477853170[187] = 0;
   out_4543420401477853170[188] = 0;
   out_4543420401477853170[189] = 0;
   out_4543420401477853170[190] = 1;
   out_4543420401477853170[191] = 0;
   out_4543420401477853170[192] = 0;
   out_4543420401477853170[193] = 0;
   out_4543420401477853170[194] = 0;
   out_4543420401477853170[195] = 0;
   out_4543420401477853170[196] = 0;
   out_4543420401477853170[197] = 0;
   out_4543420401477853170[198] = 0;
   out_4543420401477853170[199] = 0;
   out_4543420401477853170[200] = 0;
   out_4543420401477853170[201] = 0;
   out_4543420401477853170[202] = 0;
   out_4543420401477853170[203] = 0;
   out_4543420401477853170[204] = 0;
   out_4543420401477853170[205] = 0;
   out_4543420401477853170[206] = 0;
   out_4543420401477853170[207] = 0;
   out_4543420401477853170[208] = 0;
   out_4543420401477853170[209] = 1;
   out_4543420401477853170[210] = 0;
   out_4543420401477853170[211] = 0;
   out_4543420401477853170[212] = 0;
   out_4543420401477853170[213] = 0;
   out_4543420401477853170[214] = 0;
   out_4543420401477853170[215] = 0;
   out_4543420401477853170[216] = 0;
   out_4543420401477853170[217] = 0;
   out_4543420401477853170[218] = 0;
   out_4543420401477853170[219] = 0;
   out_4543420401477853170[220] = 0;
   out_4543420401477853170[221] = 0;
   out_4543420401477853170[222] = 0;
   out_4543420401477853170[223] = 0;
   out_4543420401477853170[224] = 0;
   out_4543420401477853170[225] = 0;
   out_4543420401477853170[226] = 0;
   out_4543420401477853170[227] = 0;
   out_4543420401477853170[228] = 1;
   out_4543420401477853170[229] = 0;
   out_4543420401477853170[230] = 0;
   out_4543420401477853170[231] = 0;
   out_4543420401477853170[232] = 0;
   out_4543420401477853170[233] = 0;
   out_4543420401477853170[234] = 0;
   out_4543420401477853170[235] = 0;
   out_4543420401477853170[236] = 0;
   out_4543420401477853170[237] = 0;
   out_4543420401477853170[238] = 0;
   out_4543420401477853170[239] = 0;
   out_4543420401477853170[240] = 0;
   out_4543420401477853170[241] = 0;
   out_4543420401477853170[242] = 0;
   out_4543420401477853170[243] = 0;
   out_4543420401477853170[244] = 0;
   out_4543420401477853170[245] = 0;
   out_4543420401477853170[246] = 0;
   out_4543420401477853170[247] = 1;
   out_4543420401477853170[248] = 0;
   out_4543420401477853170[249] = 0;
   out_4543420401477853170[250] = 0;
   out_4543420401477853170[251] = 0;
   out_4543420401477853170[252] = 0;
   out_4543420401477853170[253] = 0;
   out_4543420401477853170[254] = 0;
   out_4543420401477853170[255] = 0;
   out_4543420401477853170[256] = 0;
   out_4543420401477853170[257] = 0;
   out_4543420401477853170[258] = 0;
   out_4543420401477853170[259] = 0;
   out_4543420401477853170[260] = 0;
   out_4543420401477853170[261] = 0;
   out_4543420401477853170[262] = 0;
   out_4543420401477853170[263] = 0;
   out_4543420401477853170[264] = 0;
   out_4543420401477853170[265] = 0;
   out_4543420401477853170[266] = 1;
   out_4543420401477853170[267] = 0;
   out_4543420401477853170[268] = 0;
   out_4543420401477853170[269] = 0;
   out_4543420401477853170[270] = 0;
   out_4543420401477853170[271] = 0;
   out_4543420401477853170[272] = 0;
   out_4543420401477853170[273] = 0;
   out_4543420401477853170[274] = 0;
   out_4543420401477853170[275] = 0;
   out_4543420401477853170[276] = 0;
   out_4543420401477853170[277] = 0;
   out_4543420401477853170[278] = 0;
   out_4543420401477853170[279] = 0;
   out_4543420401477853170[280] = 0;
   out_4543420401477853170[281] = 0;
   out_4543420401477853170[282] = 0;
   out_4543420401477853170[283] = 0;
   out_4543420401477853170[284] = 0;
   out_4543420401477853170[285] = 1;
   out_4543420401477853170[286] = 0;
   out_4543420401477853170[287] = 0;
   out_4543420401477853170[288] = 0;
   out_4543420401477853170[289] = 0;
   out_4543420401477853170[290] = 0;
   out_4543420401477853170[291] = 0;
   out_4543420401477853170[292] = 0;
   out_4543420401477853170[293] = 0;
   out_4543420401477853170[294] = 0;
   out_4543420401477853170[295] = 0;
   out_4543420401477853170[296] = 0;
   out_4543420401477853170[297] = 0;
   out_4543420401477853170[298] = 0;
   out_4543420401477853170[299] = 0;
   out_4543420401477853170[300] = 0;
   out_4543420401477853170[301] = 0;
   out_4543420401477853170[302] = 0;
   out_4543420401477853170[303] = 0;
   out_4543420401477853170[304] = 1;
   out_4543420401477853170[305] = 0;
   out_4543420401477853170[306] = 0;
   out_4543420401477853170[307] = 0;
   out_4543420401477853170[308] = 0;
   out_4543420401477853170[309] = 0;
   out_4543420401477853170[310] = 0;
   out_4543420401477853170[311] = 0;
   out_4543420401477853170[312] = 0;
   out_4543420401477853170[313] = 0;
   out_4543420401477853170[314] = 0;
   out_4543420401477853170[315] = 0;
   out_4543420401477853170[316] = 0;
   out_4543420401477853170[317] = 0;
   out_4543420401477853170[318] = 0;
   out_4543420401477853170[319] = 0;
   out_4543420401477853170[320] = 0;
   out_4543420401477853170[321] = 0;
   out_4543420401477853170[322] = 0;
   out_4543420401477853170[323] = 1;
}
void h_4(double *state, double *unused, double *out_6014386894159692855) {
   out_6014386894159692855[0] = state[6] + state[9];
   out_6014386894159692855[1] = state[7] + state[10];
   out_6014386894159692855[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6012289415225888079) {
   out_6012289415225888079[0] = 0;
   out_6012289415225888079[1] = 0;
   out_6012289415225888079[2] = 0;
   out_6012289415225888079[3] = 0;
   out_6012289415225888079[4] = 0;
   out_6012289415225888079[5] = 0;
   out_6012289415225888079[6] = 1;
   out_6012289415225888079[7] = 0;
   out_6012289415225888079[8] = 0;
   out_6012289415225888079[9] = 1;
   out_6012289415225888079[10] = 0;
   out_6012289415225888079[11] = 0;
   out_6012289415225888079[12] = 0;
   out_6012289415225888079[13] = 0;
   out_6012289415225888079[14] = 0;
   out_6012289415225888079[15] = 0;
   out_6012289415225888079[16] = 0;
   out_6012289415225888079[17] = 0;
   out_6012289415225888079[18] = 0;
   out_6012289415225888079[19] = 0;
   out_6012289415225888079[20] = 0;
   out_6012289415225888079[21] = 0;
   out_6012289415225888079[22] = 0;
   out_6012289415225888079[23] = 0;
   out_6012289415225888079[24] = 0;
   out_6012289415225888079[25] = 1;
   out_6012289415225888079[26] = 0;
   out_6012289415225888079[27] = 0;
   out_6012289415225888079[28] = 1;
   out_6012289415225888079[29] = 0;
   out_6012289415225888079[30] = 0;
   out_6012289415225888079[31] = 0;
   out_6012289415225888079[32] = 0;
   out_6012289415225888079[33] = 0;
   out_6012289415225888079[34] = 0;
   out_6012289415225888079[35] = 0;
   out_6012289415225888079[36] = 0;
   out_6012289415225888079[37] = 0;
   out_6012289415225888079[38] = 0;
   out_6012289415225888079[39] = 0;
   out_6012289415225888079[40] = 0;
   out_6012289415225888079[41] = 0;
   out_6012289415225888079[42] = 0;
   out_6012289415225888079[43] = 0;
   out_6012289415225888079[44] = 1;
   out_6012289415225888079[45] = 0;
   out_6012289415225888079[46] = 0;
   out_6012289415225888079[47] = 1;
   out_6012289415225888079[48] = 0;
   out_6012289415225888079[49] = 0;
   out_6012289415225888079[50] = 0;
   out_6012289415225888079[51] = 0;
   out_6012289415225888079[52] = 0;
   out_6012289415225888079[53] = 0;
}
void h_10(double *state, double *unused, double *out_2247240218230926467) {
   out_2247240218230926467[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2247240218230926467[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2247240218230926467[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2995791716314335637) {
   out_2995791716314335637[0] = 0;
   out_2995791716314335637[1] = 9.8100000000000005*cos(state[1]);
   out_2995791716314335637[2] = 0;
   out_2995791716314335637[3] = 0;
   out_2995791716314335637[4] = -state[8];
   out_2995791716314335637[5] = state[7];
   out_2995791716314335637[6] = 0;
   out_2995791716314335637[7] = state[5];
   out_2995791716314335637[8] = -state[4];
   out_2995791716314335637[9] = 0;
   out_2995791716314335637[10] = 0;
   out_2995791716314335637[11] = 0;
   out_2995791716314335637[12] = 1;
   out_2995791716314335637[13] = 0;
   out_2995791716314335637[14] = 0;
   out_2995791716314335637[15] = 1;
   out_2995791716314335637[16] = 0;
   out_2995791716314335637[17] = 0;
   out_2995791716314335637[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2995791716314335637[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2995791716314335637[20] = 0;
   out_2995791716314335637[21] = state[8];
   out_2995791716314335637[22] = 0;
   out_2995791716314335637[23] = -state[6];
   out_2995791716314335637[24] = -state[5];
   out_2995791716314335637[25] = 0;
   out_2995791716314335637[26] = state[3];
   out_2995791716314335637[27] = 0;
   out_2995791716314335637[28] = 0;
   out_2995791716314335637[29] = 0;
   out_2995791716314335637[30] = 0;
   out_2995791716314335637[31] = 1;
   out_2995791716314335637[32] = 0;
   out_2995791716314335637[33] = 0;
   out_2995791716314335637[34] = 1;
   out_2995791716314335637[35] = 0;
   out_2995791716314335637[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2995791716314335637[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2995791716314335637[38] = 0;
   out_2995791716314335637[39] = -state[7];
   out_2995791716314335637[40] = state[6];
   out_2995791716314335637[41] = 0;
   out_2995791716314335637[42] = state[4];
   out_2995791716314335637[43] = -state[3];
   out_2995791716314335637[44] = 0;
   out_2995791716314335637[45] = 0;
   out_2995791716314335637[46] = 0;
   out_2995791716314335637[47] = 0;
   out_2995791716314335637[48] = 0;
   out_2995791716314335637[49] = 0;
   out_2995791716314335637[50] = 1;
   out_2995791716314335637[51] = 0;
   out_2995791716314335637[52] = 0;
   out_2995791716314335637[53] = 1;
}
void h_13(double *state, double *unused, double *out_2231888146994886805) {
   out_2231888146994886805[0] = state[3];
   out_2231888146994886805[1] = state[4];
   out_2231888146994886805[2] = state[5];
}
void H_13(double *state, double *unused, double *out_1598341793090812850) {
   out_1598341793090812850[0] = 0;
   out_1598341793090812850[1] = 0;
   out_1598341793090812850[2] = 0;
   out_1598341793090812850[3] = 1;
   out_1598341793090812850[4] = 0;
   out_1598341793090812850[5] = 0;
   out_1598341793090812850[6] = 0;
   out_1598341793090812850[7] = 0;
   out_1598341793090812850[8] = 0;
   out_1598341793090812850[9] = 0;
   out_1598341793090812850[10] = 0;
   out_1598341793090812850[11] = 0;
   out_1598341793090812850[12] = 0;
   out_1598341793090812850[13] = 0;
   out_1598341793090812850[14] = 0;
   out_1598341793090812850[15] = 0;
   out_1598341793090812850[16] = 0;
   out_1598341793090812850[17] = 0;
   out_1598341793090812850[18] = 0;
   out_1598341793090812850[19] = 0;
   out_1598341793090812850[20] = 0;
   out_1598341793090812850[21] = 0;
   out_1598341793090812850[22] = 1;
   out_1598341793090812850[23] = 0;
   out_1598341793090812850[24] = 0;
   out_1598341793090812850[25] = 0;
   out_1598341793090812850[26] = 0;
   out_1598341793090812850[27] = 0;
   out_1598341793090812850[28] = 0;
   out_1598341793090812850[29] = 0;
   out_1598341793090812850[30] = 0;
   out_1598341793090812850[31] = 0;
   out_1598341793090812850[32] = 0;
   out_1598341793090812850[33] = 0;
   out_1598341793090812850[34] = 0;
   out_1598341793090812850[35] = 0;
   out_1598341793090812850[36] = 0;
   out_1598341793090812850[37] = 0;
   out_1598341793090812850[38] = 0;
   out_1598341793090812850[39] = 0;
   out_1598341793090812850[40] = 0;
   out_1598341793090812850[41] = 1;
   out_1598341793090812850[42] = 0;
   out_1598341793090812850[43] = 0;
   out_1598341793090812850[44] = 0;
   out_1598341793090812850[45] = 0;
   out_1598341793090812850[46] = 0;
   out_1598341793090812850[47] = 0;
   out_1598341793090812850[48] = 0;
   out_1598341793090812850[49] = 0;
   out_1598341793090812850[50] = 0;
   out_1598341793090812850[51] = 0;
   out_1598341793090812850[52] = 0;
   out_1598341793090812850[53] = 0;
}
void h_14(double *state, double *unused, double *out_2947005334086789936) {
   out_2947005334086789936[0] = state[6];
   out_2947005334086789936[1] = state[7];
   out_2947005334086789936[2] = state[8];
}
void H_14(double *state, double *unused, double *out_2049048558886403550) {
   out_2049048558886403550[0] = 0;
   out_2049048558886403550[1] = 0;
   out_2049048558886403550[2] = 0;
   out_2049048558886403550[3] = 0;
   out_2049048558886403550[4] = 0;
   out_2049048558886403550[5] = 0;
   out_2049048558886403550[6] = 1;
   out_2049048558886403550[7] = 0;
   out_2049048558886403550[8] = 0;
   out_2049048558886403550[9] = 0;
   out_2049048558886403550[10] = 0;
   out_2049048558886403550[11] = 0;
   out_2049048558886403550[12] = 0;
   out_2049048558886403550[13] = 0;
   out_2049048558886403550[14] = 0;
   out_2049048558886403550[15] = 0;
   out_2049048558886403550[16] = 0;
   out_2049048558886403550[17] = 0;
   out_2049048558886403550[18] = 0;
   out_2049048558886403550[19] = 0;
   out_2049048558886403550[20] = 0;
   out_2049048558886403550[21] = 0;
   out_2049048558886403550[22] = 0;
   out_2049048558886403550[23] = 0;
   out_2049048558886403550[24] = 0;
   out_2049048558886403550[25] = 1;
   out_2049048558886403550[26] = 0;
   out_2049048558886403550[27] = 0;
   out_2049048558886403550[28] = 0;
   out_2049048558886403550[29] = 0;
   out_2049048558886403550[30] = 0;
   out_2049048558886403550[31] = 0;
   out_2049048558886403550[32] = 0;
   out_2049048558886403550[33] = 0;
   out_2049048558886403550[34] = 0;
   out_2049048558886403550[35] = 0;
   out_2049048558886403550[36] = 0;
   out_2049048558886403550[37] = 0;
   out_2049048558886403550[38] = 0;
   out_2049048558886403550[39] = 0;
   out_2049048558886403550[40] = 0;
   out_2049048558886403550[41] = 0;
   out_2049048558886403550[42] = 0;
   out_2049048558886403550[43] = 0;
   out_2049048558886403550[44] = 1;
   out_2049048558886403550[45] = 0;
   out_2049048558886403550[46] = 0;
   out_2049048558886403550[47] = 0;
   out_2049048558886403550[48] = 0;
   out_2049048558886403550[49] = 0;
   out_2049048558886403550[50] = 0;
   out_2049048558886403550[51] = 0;
   out_2049048558886403550[52] = 0;
   out_2049048558886403550[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_8400786688062439867) {
  err_fun(nom_x, delta_x, out_8400786688062439867);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5956079812122370151) {
  inv_err_fun(nom_x, true_x, out_5956079812122370151);
}
void pose_H_mod_fun(double *state, double *out_8542454711932240757) {
  H_mod_fun(state, out_8542454711932240757);
}
void pose_f_fun(double *state, double dt, double *out_6490866571001637660) {
  f_fun(state,  dt, out_6490866571001637660);
}
void pose_F_fun(double *state, double dt, double *out_4543420401477853170) {
  F_fun(state,  dt, out_4543420401477853170);
}
void pose_h_4(double *state, double *unused, double *out_6014386894159692855) {
  h_4(state, unused, out_6014386894159692855);
}
void pose_H_4(double *state, double *unused, double *out_6012289415225888079) {
  H_4(state, unused, out_6012289415225888079);
}
void pose_h_10(double *state, double *unused, double *out_2247240218230926467) {
  h_10(state, unused, out_2247240218230926467);
}
void pose_H_10(double *state, double *unused, double *out_2995791716314335637) {
  H_10(state, unused, out_2995791716314335637);
}
void pose_h_13(double *state, double *unused, double *out_2231888146994886805) {
  h_13(state, unused, out_2231888146994886805);
}
void pose_H_13(double *state, double *unused, double *out_1598341793090812850) {
  H_13(state, unused, out_1598341793090812850);
}
void pose_h_14(double *state, double *unused, double *out_2947005334086789936) {
  h_14(state, unused, out_2947005334086789936);
}
void pose_H_14(double *state, double *unused, double *out_2049048558886403550) {
  H_14(state, unused, out_2049048558886403550);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
