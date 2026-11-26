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
void err_fun(double *nom_x, double *delta_x, double *out_6300647231984494843) {
   out_6300647231984494843[0] = delta_x[0] + nom_x[0];
   out_6300647231984494843[1] = delta_x[1] + nom_x[1];
   out_6300647231984494843[2] = delta_x[2] + nom_x[2];
   out_6300647231984494843[3] = delta_x[3] + nom_x[3];
   out_6300647231984494843[4] = delta_x[4] + nom_x[4];
   out_6300647231984494843[5] = delta_x[5] + nom_x[5];
   out_6300647231984494843[6] = delta_x[6] + nom_x[6];
   out_6300647231984494843[7] = delta_x[7] + nom_x[7];
   out_6300647231984494843[8] = delta_x[8] + nom_x[8];
   out_6300647231984494843[9] = delta_x[9] + nom_x[9];
   out_6300647231984494843[10] = delta_x[10] + nom_x[10];
   out_6300647231984494843[11] = delta_x[11] + nom_x[11];
   out_6300647231984494843[12] = delta_x[12] + nom_x[12];
   out_6300647231984494843[13] = delta_x[13] + nom_x[13];
   out_6300647231984494843[14] = delta_x[14] + nom_x[14];
   out_6300647231984494843[15] = delta_x[15] + nom_x[15];
   out_6300647231984494843[16] = delta_x[16] + nom_x[16];
   out_6300647231984494843[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4334582594945192021) {
   out_4334582594945192021[0] = -nom_x[0] + true_x[0];
   out_4334582594945192021[1] = -nom_x[1] + true_x[1];
   out_4334582594945192021[2] = -nom_x[2] + true_x[2];
   out_4334582594945192021[3] = -nom_x[3] + true_x[3];
   out_4334582594945192021[4] = -nom_x[4] + true_x[4];
   out_4334582594945192021[5] = -nom_x[5] + true_x[5];
   out_4334582594945192021[6] = -nom_x[6] + true_x[6];
   out_4334582594945192021[7] = -nom_x[7] + true_x[7];
   out_4334582594945192021[8] = -nom_x[8] + true_x[8];
   out_4334582594945192021[9] = -nom_x[9] + true_x[9];
   out_4334582594945192021[10] = -nom_x[10] + true_x[10];
   out_4334582594945192021[11] = -nom_x[11] + true_x[11];
   out_4334582594945192021[12] = -nom_x[12] + true_x[12];
   out_4334582594945192021[13] = -nom_x[13] + true_x[13];
   out_4334582594945192021[14] = -nom_x[14] + true_x[14];
   out_4334582594945192021[15] = -nom_x[15] + true_x[15];
   out_4334582594945192021[16] = -nom_x[16] + true_x[16];
   out_4334582594945192021[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1393911559862424916) {
   out_1393911559862424916[0] = 1.0;
   out_1393911559862424916[1] = 0.0;
   out_1393911559862424916[2] = 0.0;
   out_1393911559862424916[3] = 0.0;
   out_1393911559862424916[4] = 0.0;
   out_1393911559862424916[5] = 0.0;
   out_1393911559862424916[6] = 0.0;
   out_1393911559862424916[7] = 0.0;
   out_1393911559862424916[8] = 0.0;
   out_1393911559862424916[9] = 0.0;
   out_1393911559862424916[10] = 0.0;
   out_1393911559862424916[11] = 0.0;
   out_1393911559862424916[12] = 0.0;
   out_1393911559862424916[13] = 0.0;
   out_1393911559862424916[14] = 0.0;
   out_1393911559862424916[15] = 0.0;
   out_1393911559862424916[16] = 0.0;
   out_1393911559862424916[17] = 0.0;
   out_1393911559862424916[18] = 0.0;
   out_1393911559862424916[19] = 1.0;
   out_1393911559862424916[20] = 0.0;
   out_1393911559862424916[21] = 0.0;
   out_1393911559862424916[22] = 0.0;
   out_1393911559862424916[23] = 0.0;
   out_1393911559862424916[24] = 0.0;
   out_1393911559862424916[25] = 0.0;
   out_1393911559862424916[26] = 0.0;
   out_1393911559862424916[27] = 0.0;
   out_1393911559862424916[28] = 0.0;
   out_1393911559862424916[29] = 0.0;
   out_1393911559862424916[30] = 0.0;
   out_1393911559862424916[31] = 0.0;
   out_1393911559862424916[32] = 0.0;
   out_1393911559862424916[33] = 0.0;
   out_1393911559862424916[34] = 0.0;
   out_1393911559862424916[35] = 0.0;
   out_1393911559862424916[36] = 0.0;
   out_1393911559862424916[37] = 0.0;
   out_1393911559862424916[38] = 1.0;
   out_1393911559862424916[39] = 0.0;
   out_1393911559862424916[40] = 0.0;
   out_1393911559862424916[41] = 0.0;
   out_1393911559862424916[42] = 0.0;
   out_1393911559862424916[43] = 0.0;
   out_1393911559862424916[44] = 0.0;
   out_1393911559862424916[45] = 0.0;
   out_1393911559862424916[46] = 0.0;
   out_1393911559862424916[47] = 0.0;
   out_1393911559862424916[48] = 0.0;
   out_1393911559862424916[49] = 0.0;
   out_1393911559862424916[50] = 0.0;
   out_1393911559862424916[51] = 0.0;
   out_1393911559862424916[52] = 0.0;
   out_1393911559862424916[53] = 0.0;
   out_1393911559862424916[54] = 0.0;
   out_1393911559862424916[55] = 0.0;
   out_1393911559862424916[56] = 0.0;
   out_1393911559862424916[57] = 1.0;
   out_1393911559862424916[58] = 0.0;
   out_1393911559862424916[59] = 0.0;
   out_1393911559862424916[60] = 0.0;
   out_1393911559862424916[61] = 0.0;
   out_1393911559862424916[62] = 0.0;
   out_1393911559862424916[63] = 0.0;
   out_1393911559862424916[64] = 0.0;
   out_1393911559862424916[65] = 0.0;
   out_1393911559862424916[66] = 0.0;
   out_1393911559862424916[67] = 0.0;
   out_1393911559862424916[68] = 0.0;
   out_1393911559862424916[69] = 0.0;
   out_1393911559862424916[70] = 0.0;
   out_1393911559862424916[71] = 0.0;
   out_1393911559862424916[72] = 0.0;
   out_1393911559862424916[73] = 0.0;
   out_1393911559862424916[74] = 0.0;
   out_1393911559862424916[75] = 0.0;
   out_1393911559862424916[76] = 1.0;
   out_1393911559862424916[77] = 0.0;
   out_1393911559862424916[78] = 0.0;
   out_1393911559862424916[79] = 0.0;
   out_1393911559862424916[80] = 0.0;
   out_1393911559862424916[81] = 0.0;
   out_1393911559862424916[82] = 0.0;
   out_1393911559862424916[83] = 0.0;
   out_1393911559862424916[84] = 0.0;
   out_1393911559862424916[85] = 0.0;
   out_1393911559862424916[86] = 0.0;
   out_1393911559862424916[87] = 0.0;
   out_1393911559862424916[88] = 0.0;
   out_1393911559862424916[89] = 0.0;
   out_1393911559862424916[90] = 0.0;
   out_1393911559862424916[91] = 0.0;
   out_1393911559862424916[92] = 0.0;
   out_1393911559862424916[93] = 0.0;
   out_1393911559862424916[94] = 0.0;
   out_1393911559862424916[95] = 1.0;
   out_1393911559862424916[96] = 0.0;
   out_1393911559862424916[97] = 0.0;
   out_1393911559862424916[98] = 0.0;
   out_1393911559862424916[99] = 0.0;
   out_1393911559862424916[100] = 0.0;
   out_1393911559862424916[101] = 0.0;
   out_1393911559862424916[102] = 0.0;
   out_1393911559862424916[103] = 0.0;
   out_1393911559862424916[104] = 0.0;
   out_1393911559862424916[105] = 0.0;
   out_1393911559862424916[106] = 0.0;
   out_1393911559862424916[107] = 0.0;
   out_1393911559862424916[108] = 0.0;
   out_1393911559862424916[109] = 0.0;
   out_1393911559862424916[110] = 0.0;
   out_1393911559862424916[111] = 0.0;
   out_1393911559862424916[112] = 0.0;
   out_1393911559862424916[113] = 0.0;
   out_1393911559862424916[114] = 1.0;
   out_1393911559862424916[115] = 0.0;
   out_1393911559862424916[116] = 0.0;
   out_1393911559862424916[117] = 0.0;
   out_1393911559862424916[118] = 0.0;
   out_1393911559862424916[119] = 0.0;
   out_1393911559862424916[120] = 0.0;
   out_1393911559862424916[121] = 0.0;
   out_1393911559862424916[122] = 0.0;
   out_1393911559862424916[123] = 0.0;
   out_1393911559862424916[124] = 0.0;
   out_1393911559862424916[125] = 0.0;
   out_1393911559862424916[126] = 0.0;
   out_1393911559862424916[127] = 0.0;
   out_1393911559862424916[128] = 0.0;
   out_1393911559862424916[129] = 0.0;
   out_1393911559862424916[130] = 0.0;
   out_1393911559862424916[131] = 0.0;
   out_1393911559862424916[132] = 0.0;
   out_1393911559862424916[133] = 1.0;
   out_1393911559862424916[134] = 0.0;
   out_1393911559862424916[135] = 0.0;
   out_1393911559862424916[136] = 0.0;
   out_1393911559862424916[137] = 0.0;
   out_1393911559862424916[138] = 0.0;
   out_1393911559862424916[139] = 0.0;
   out_1393911559862424916[140] = 0.0;
   out_1393911559862424916[141] = 0.0;
   out_1393911559862424916[142] = 0.0;
   out_1393911559862424916[143] = 0.0;
   out_1393911559862424916[144] = 0.0;
   out_1393911559862424916[145] = 0.0;
   out_1393911559862424916[146] = 0.0;
   out_1393911559862424916[147] = 0.0;
   out_1393911559862424916[148] = 0.0;
   out_1393911559862424916[149] = 0.0;
   out_1393911559862424916[150] = 0.0;
   out_1393911559862424916[151] = 0.0;
   out_1393911559862424916[152] = 1.0;
   out_1393911559862424916[153] = 0.0;
   out_1393911559862424916[154] = 0.0;
   out_1393911559862424916[155] = 0.0;
   out_1393911559862424916[156] = 0.0;
   out_1393911559862424916[157] = 0.0;
   out_1393911559862424916[158] = 0.0;
   out_1393911559862424916[159] = 0.0;
   out_1393911559862424916[160] = 0.0;
   out_1393911559862424916[161] = 0.0;
   out_1393911559862424916[162] = 0.0;
   out_1393911559862424916[163] = 0.0;
   out_1393911559862424916[164] = 0.0;
   out_1393911559862424916[165] = 0.0;
   out_1393911559862424916[166] = 0.0;
   out_1393911559862424916[167] = 0.0;
   out_1393911559862424916[168] = 0.0;
   out_1393911559862424916[169] = 0.0;
   out_1393911559862424916[170] = 0.0;
   out_1393911559862424916[171] = 1.0;
   out_1393911559862424916[172] = 0.0;
   out_1393911559862424916[173] = 0.0;
   out_1393911559862424916[174] = 0.0;
   out_1393911559862424916[175] = 0.0;
   out_1393911559862424916[176] = 0.0;
   out_1393911559862424916[177] = 0.0;
   out_1393911559862424916[178] = 0.0;
   out_1393911559862424916[179] = 0.0;
   out_1393911559862424916[180] = 0.0;
   out_1393911559862424916[181] = 0.0;
   out_1393911559862424916[182] = 0.0;
   out_1393911559862424916[183] = 0.0;
   out_1393911559862424916[184] = 0.0;
   out_1393911559862424916[185] = 0.0;
   out_1393911559862424916[186] = 0.0;
   out_1393911559862424916[187] = 0.0;
   out_1393911559862424916[188] = 0.0;
   out_1393911559862424916[189] = 0.0;
   out_1393911559862424916[190] = 1.0;
   out_1393911559862424916[191] = 0.0;
   out_1393911559862424916[192] = 0.0;
   out_1393911559862424916[193] = 0.0;
   out_1393911559862424916[194] = 0.0;
   out_1393911559862424916[195] = 0.0;
   out_1393911559862424916[196] = 0.0;
   out_1393911559862424916[197] = 0.0;
   out_1393911559862424916[198] = 0.0;
   out_1393911559862424916[199] = 0.0;
   out_1393911559862424916[200] = 0.0;
   out_1393911559862424916[201] = 0.0;
   out_1393911559862424916[202] = 0.0;
   out_1393911559862424916[203] = 0.0;
   out_1393911559862424916[204] = 0.0;
   out_1393911559862424916[205] = 0.0;
   out_1393911559862424916[206] = 0.0;
   out_1393911559862424916[207] = 0.0;
   out_1393911559862424916[208] = 0.0;
   out_1393911559862424916[209] = 1.0;
   out_1393911559862424916[210] = 0.0;
   out_1393911559862424916[211] = 0.0;
   out_1393911559862424916[212] = 0.0;
   out_1393911559862424916[213] = 0.0;
   out_1393911559862424916[214] = 0.0;
   out_1393911559862424916[215] = 0.0;
   out_1393911559862424916[216] = 0.0;
   out_1393911559862424916[217] = 0.0;
   out_1393911559862424916[218] = 0.0;
   out_1393911559862424916[219] = 0.0;
   out_1393911559862424916[220] = 0.0;
   out_1393911559862424916[221] = 0.0;
   out_1393911559862424916[222] = 0.0;
   out_1393911559862424916[223] = 0.0;
   out_1393911559862424916[224] = 0.0;
   out_1393911559862424916[225] = 0.0;
   out_1393911559862424916[226] = 0.0;
   out_1393911559862424916[227] = 0.0;
   out_1393911559862424916[228] = 1.0;
   out_1393911559862424916[229] = 0.0;
   out_1393911559862424916[230] = 0.0;
   out_1393911559862424916[231] = 0.0;
   out_1393911559862424916[232] = 0.0;
   out_1393911559862424916[233] = 0.0;
   out_1393911559862424916[234] = 0.0;
   out_1393911559862424916[235] = 0.0;
   out_1393911559862424916[236] = 0.0;
   out_1393911559862424916[237] = 0.0;
   out_1393911559862424916[238] = 0.0;
   out_1393911559862424916[239] = 0.0;
   out_1393911559862424916[240] = 0.0;
   out_1393911559862424916[241] = 0.0;
   out_1393911559862424916[242] = 0.0;
   out_1393911559862424916[243] = 0.0;
   out_1393911559862424916[244] = 0.0;
   out_1393911559862424916[245] = 0.0;
   out_1393911559862424916[246] = 0.0;
   out_1393911559862424916[247] = 1.0;
   out_1393911559862424916[248] = 0.0;
   out_1393911559862424916[249] = 0.0;
   out_1393911559862424916[250] = 0.0;
   out_1393911559862424916[251] = 0.0;
   out_1393911559862424916[252] = 0.0;
   out_1393911559862424916[253] = 0.0;
   out_1393911559862424916[254] = 0.0;
   out_1393911559862424916[255] = 0.0;
   out_1393911559862424916[256] = 0.0;
   out_1393911559862424916[257] = 0.0;
   out_1393911559862424916[258] = 0.0;
   out_1393911559862424916[259] = 0.0;
   out_1393911559862424916[260] = 0.0;
   out_1393911559862424916[261] = 0.0;
   out_1393911559862424916[262] = 0.0;
   out_1393911559862424916[263] = 0.0;
   out_1393911559862424916[264] = 0.0;
   out_1393911559862424916[265] = 0.0;
   out_1393911559862424916[266] = 1.0;
   out_1393911559862424916[267] = 0.0;
   out_1393911559862424916[268] = 0.0;
   out_1393911559862424916[269] = 0.0;
   out_1393911559862424916[270] = 0.0;
   out_1393911559862424916[271] = 0.0;
   out_1393911559862424916[272] = 0.0;
   out_1393911559862424916[273] = 0.0;
   out_1393911559862424916[274] = 0.0;
   out_1393911559862424916[275] = 0.0;
   out_1393911559862424916[276] = 0.0;
   out_1393911559862424916[277] = 0.0;
   out_1393911559862424916[278] = 0.0;
   out_1393911559862424916[279] = 0.0;
   out_1393911559862424916[280] = 0.0;
   out_1393911559862424916[281] = 0.0;
   out_1393911559862424916[282] = 0.0;
   out_1393911559862424916[283] = 0.0;
   out_1393911559862424916[284] = 0.0;
   out_1393911559862424916[285] = 1.0;
   out_1393911559862424916[286] = 0.0;
   out_1393911559862424916[287] = 0.0;
   out_1393911559862424916[288] = 0.0;
   out_1393911559862424916[289] = 0.0;
   out_1393911559862424916[290] = 0.0;
   out_1393911559862424916[291] = 0.0;
   out_1393911559862424916[292] = 0.0;
   out_1393911559862424916[293] = 0.0;
   out_1393911559862424916[294] = 0.0;
   out_1393911559862424916[295] = 0.0;
   out_1393911559862424916[296] = 0.0;
   out_1393911559862424916[297] = 0.0;
   out_1393911559862424916[298] = 0.0;
   out_1393911559862424916[299] = 0.0;
   out_1393911559862424916[300] = 0.0;
   out_1393911559862424916[301] = 0.0;
   out_1393911559862424916[302] = 0.0;
   out_1393911559862424916[303] = 0.0;
   out_1393911559862424916[304] = 1.0;
   out_1393911559862424916[305] = 0.0;
   out_1393911559862424916[306] = 0.0;
   out_1393911559862424916[307] = 0.0;
   out_1393911559862424916[308] = 0.0;
   out_1393911559862424916[309] = 0.0;
   out_1393911559862424916[310] = 0.0;
   out_1393911559862424916[311] = 0.0;
   out_1393911559862424916[312] = 0.0;
   out_1393911559862424916[313] = 0.0;
   out_1393911559862424916[314] = 0.0;
   out_1393911559862424916[315] = 0.0;
   out_1393911559862424916[316] = 0.0;
   out_1393911559862424916[317] = 0.0;
   out_1393911559862424916[318] = 0.0;
   out_1393911559862424916[319] = 0.0;
   out_1393911559862424916[320] = 0.0;
   out_1393911559862424916[321] = 0.0;
   out_1393911559862424916[322] = 0.0;
   out_1393911559862424916[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_7637055358137252657) {
   out_7637055358137252657[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_7637055358137252657[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_7637055358137252657[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_7637055358137252657[3] = dt*state[12] + state[3];
   out_7637055358137252657[4] = dt*state[13] + state[4];
   out_7637055358137252657[5] = dt*state[14] + state[5];
   out_7637055358137252657[6] = state[6];
   out_7637055358137252657[7] = state[7];
   out_7637055358137252657[8] = state[8];
   out_7637055358137252657[9] = state[9];
   out_7637055358137252657[10] = state[10];
   out_7637055358137252657[11] = state[11];
   out_7637055358137252657[12] = state[12];
   out_7637055358137252657[13] = state[13];
   out_7637055358137252657[14] = state[14];
   out_7637055358137252657[15] = state[15];
   out_7637055358137252657[16] = state[16];
   out_7637055358137252657[17] = state[17];
}
void F_fun(double *state, double dt, double *out_682184522508178564) {
   out_682184522508178564[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_682184522508178564[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_682184522508178564[2] = 0;
   out_682184522508178564[3] = 0;
   out_682184522508178564[4] = 0;
   out_682184522508178564[5] = 0;
   out_682184522508178564[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_682184522508178564[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_682184522508178564[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_682184522508178564[9] = 0;
   out_682184522508178564[10] = 0;
   out_682184522508178564[11] = 0;
   out_682184522508178564[12] = 0;
   out_682184522508178564[13] = 0;
   out_682184522508178564[14] = 0;
   out_682184522508178564[15] = 0;
   out_682184522508178564[16] = 0;
   out_682184522508178564[17] = 0;
   out_682184522508178564[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_682184522508178564[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_682184522508178564[20] = 0;
   out_682184522508178564[21] = 0;
   out_682184522508178564[22] = 0;
   out_682184522508178564[23] = 0;
   out_682184522508178564[24] = 0;
   out_682184522508178564[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_682184522508178564[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_682184522508178564[27] = 0;
   out_682184522508178564[28] = 0;
   out_682184522508178564[29] = 0;
   out_682184522508178564[30] = 0;
   out_682184522508178564[31] = 0;
   out_682184522508178564[32] = 0;
   out_682184522508178564[33] = 0;
   out_682184522508178564[34] = 0;
   out_682184522508178564[35] = 0;
   out_682184522508178564[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_682184522508178564[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_682184522508178564[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_682184522508178564[39] = 0;
   out_682184522508178564[40] = 0;
   out_682184522508178564[41] = 0;
   out_682184522508178564[42] = 0;
   out_682184522508178564[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_682184522508178564[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_682184522508178564[45] = 0;
   out_682184522508178564[46] = 0;
   out_682184522508178564[47] = 0;
   out_682184522508178564[48] = 0;
   out_682184522508178564[49] = 0;
   out_682184522508178564[50] = 0;
   out_682184522508178564[51] = 0;
   out_682184522508178564[52] = 0;
   out_682184522508178564[53] = 0;
   out_682184522508178564[54] = 0;
   out_682184522508178564[55] = 0;
   out_682184522508178564[56] = 0;
   out_682184522508178564[57] = 1;
   out_682184522508178564[58] = 0;
   out_682184522508178564[59] = 0;
   out_682184522508178564[60] = 0;
   out_682184522508178564[61] = 0;
   out_682184522508178564[62] = 0;
   out_682184522508178564[63] = 0;
   out_682184522508178564[64] = 0;
   out_682184522508178564[65] = 0;
   out_682184522508178564[66] = dt;
   out_682184522508178564[67] = 0;
   out_682184522508178564[68] = 0;
   out_682184522508178564[69] = 0;
   out_682184522508178564[70] = 0;
   out_682184522508178564[71] = 0;
   out_682184522508178564[72] = 0;
   out_682184522508178564[73] = 0;
   out_682184522508178564[74] = 0;
   out_682184522508178564[75] = 0;
   out_682184522508178564[76] = 1;
   out_682184522508178564[77] = 0;
   out_682184522508178564[78] = 0;
   out_682184522508178564[79] = 0;
   out_682184522508178564[80] = 0;
   out_682184522508178564[81] = 0;
   out_682184522508178564[82] = 0;
   out_682184522508178564[83] = 0;
   out_682184522508178564[84] = 0;
   out_682184522508178564[85] = dt;
   out_682184522508178564[86] = 0;
   out_682184522508178564[87] = 0;
   out_682184522508178564[88] = 0;
   out_682184522508178564[89] = 0;
   out_682184522508178564[90] = 0;
   out_682184522508178564[91] = 0;
   out_682184522508178564[92] = 0;
   out_682184522508178564[93] = 0;
   out_682184522508178564[94] = 0;
   out_682184522508178564[95] = 1;
   out_682184522508178564[96] = 0;
   out_682184522508178564[97] = 0;
   out_682184522508178564[98] = 0;
   out_682184522508178564[99] = 0;
   out_682184522508178564[100] = 0;
   out_682184522508178564[101] = 0;
   out_682184522508178564[102] = 0;
   out_682184522508178564[103] = 0;
   out_682184522508178564[104] = dt;
   out_682184522508178564[105] = 0;
   out_682184522508178564[106] = 0;
   out_682184522508178564[107] = 0;
   out_682184522508178564[108] = 0;
   out_682184522508178564[109] = 0;
   out_682184522508178564[110] = 0;
   out_682184522508178564[111] = 0;
   out_682184522508178564[112] = 0;
   out_682184522508178564[113] = 0;
   out_682184522508178564[114] = 1;
   out_682184522508178564[115] = 0;
   out_682184522508178564[116] = 0;
   out_682184522508178564[117] = 0;
   out_682184522508178564[118] = 0;
   out_682184522508178564[119] = 0;
   out_682184522508178564[120] = 0;
   out_682184522508178564[121] = 0;
   out_682184522508178564[122] = 0;
   out_682184522508178564[123] = 0;
   out_682184522508178564[124] = 0;
   out_682184522508178564[125] = 0;
   out_682184522508178564[126] = 0;
   out_682184522508178564[127] = 0;
   out_682184522508178564[128] = 0;
   out_682184522508178564[129] = 0;
   out_682184522508178564[130] = 0;
   out_682184522508178564[131] = 0;
   out_682184522508178564[132] = 0;
   out_682184522508178564[133] = 1;
   out_682184522508178564[134] = 0;
   out_682184522508178564[135] = 0;
   out_682184522508178564[136] = 0;
   out_682184522508178564[137] = 0;
   out_682184522508178564[138] = 0;
   out_682184522508178564[139] = 0;
   out_682184522508178564[140] = 0;
   out_682184522508178564[141] = 0;
   out_682184522508178564[142] = 0;
   out_682184522508178564[143] = 0;
   out_682184522508178564[144] = 0;
   out_682184522508178564[145] = 0;
   out_682184522508178564[146] = 0;
   out_682184522508178564[147] = 0;
   out_682184522508178564[148] = 0;
   out_682184522508178564[149] = 0;
   out_682184522508178564[150] = 0;
   out_682184522508178564[151] = 0;
   out_682184522508178564[152] = 1;
   out_682184522508178564[153] = 0;
   out_682184522508178564[154] = 0;
   out_682184522508178564[155] = 0;
   out_682184522508178564[156] = 0;
   out_682184522508178564[157] = 0;
   out_682184522508178564[158] = 0;
   out_682184522508178564[159] = 0;
   out_682184522508178564[160] = 0;
   out_682184522508178564[161] = 0;
   out_682184522508178564[162] = 0;
   out_682184522508178564[163] = 0;
   out_682184522508178564[164] = 0;
   out_682184522508178564[165] = 0;
   out_682184522508178564[166] = 0;
   out_682184522508178564[167] = 0;
   out_682184522508178564[168] = 0;
   out_682184522508178564[169] = 0;
   out_682184522508178564[170] = 0;
   out_682184522508178564[171] = 1;
   out_682184522508178564[172] = 0;
   out_682184522508178564[173] = 0;
   out_682184522508178564[174] = 0;
   out_682184522508178564[175] = 0;
   out_682184522508178564[176] = 0;
   out_682184522508178564[177] = 0;
   out_682184522508178564[178] = 0;
   out_682184522508178564[179] = 0;
   out_682184522508178564[180] = 0;
   out_682184522508178564[181] = 0;
   out_682184522508178564[182] = 0;
   out_682184522508178564[183] = 0;
   out_682184522508178564[184] = 0;
   out_682184522508178564[185] = 0;
   out_682184522508178564[186] = 0;
   out_682184522508178564[187] = 0;
   out_682184522508178564[188] = 0;
   out_682184522508178564[189] = 0;
   out_682184522508178564[190] = 1;
   out_682184522508178564[191] = 0;
   out_682184522508178564[192] = 0;
   out_682184522508178564[193] = 0;
   out_682184522508178564[194] = 0;
   out_682184522508178564[195] = 0;
   out_682184522508178564[196] = 0;
   out_682184522508178564[197] = 0;
   out_682184522508178564[198] = 0;
   out_682184522508178564[199] = 0;
   out_682184522508178564[200] = 0;
   out_682184522508178564[201] = 0;
   out_682184522508178564[202] = 0;
   out_682184522508178564[203] = 0;
   out_682184522508178564[204] = 0;
   out_682184522508178564[205] = 0;
   out_682184522508178564[206] = 0;
   out_682184522508178564[207] = 0;
   out_682184522508178564[208] = 0;
   out_682184522508178564[209] = 1;
   out_682184522508178564[210] = 0;
   out_682184522508178564[211] = 0;
   out_682184522508178564[212] = 0;
   out_682184522508178564[213] = 0;
   out_682184522508178564[214] = 0;
   out_682184522508178564[215] = 0;
   out_682184522508178564[216] = 0;
   out_682184522508178564[217] = 0;
   out_682184522508178564[218] = 0;
   out_682184522508178564[219] = 0;
   out_682184522508178564[220] = 0;
   out_682184522508178564[221] = 0;
   out_682184522508178564[222] = 0;
   out_682184522508178564[223] = 0;
   out_682184522508178564[224] = 0;
   out_682184522508178564[225] = 0;
   out_682184522508178564[226] = 0;
   out_682184522508178564[227] = 0;
   out_682184522508178564[228] = 1;
   out_682184522508178564[229] = 0;
   out_682184522508178564[230] = 0;
   out_682184522508178564[231] = 0;
   out_682184522508178564[232] = 0;
   out_682184522508178564[233] = 0;
   out_682184522508178564[234] = 0;
   out_682184522508178564[235] = 0;
   out_682184522508178564[236] = 0;
   out_682184522508178564[237] = 0;
   out_682184522508178564[238] = 0;
   out_682184522508178564[239] = 0;
   out_682184522508178564[240] = 0;
   out_682184522508178564[241] = 0;
   out_682184522508178564[242] = 0;
   out_682184522508178564[243] = 0;
   out_682184522508178564[244] = 0;
   out_682184522508178564[245] = 0;
   out_682184522508178564[246] = 0;
   out_682184522508178564[247] = 1;
   out_682184522508178564[248] = 0;
   out_682184522508178564[249] = 0;
   out_682184522508178564[250] = 0;
   out_682184522508178564[251] = 0;
   out_682184522508178564[252] = 0;
   out_682184522508178564[253] = 0;
   out_682184522508178564[254] = 0;
   out_682184522508178564[255] = 0;
   out_682184522508178564[256] = 0;
   out_682184522508178564[257] = 0;
   out_682184522508178564[258] = 0;
   out_682184522508178564[259] = 0;
   out_682184522508178564[260] = 0;
   out_682184522508178564[261] = 0;
   out_682184522508178564[262] = 0;
   out_682184522508178564[263] = 0;
   out_682184522508178564[264] = 0;
   out_682184522508178564[265] = 0;
   out_682184522508178564[266] = 1;
   out_682184522508178564[267] = 0;
   out_682184522508178564[268] = 0;
   out_682184522508178564[269] = 0;
   out_682184522508178564[270] = 0;
   out_682184522508178564[271] = 0;
   out_682184522508178564[272] = 0;
   out_682184522508178564[273] = 0;
   out_682184522508178564[274] = 0;
   out_682184522508178564[275] = 0;
   out_682184522508178564[276] = 0;
   out_682184522508178564[277] = 0;
   out_682184522508178564[278] = 0;
   out_682184522508178564[279] = 0;
   out_682184522508178564[280] = 0;
   out_682184522508178564[281] = 0;
   out_682184522508178564[282] = 0;
   out_682184522508178564[283] = 0;
   out_682184522508178564[284] = 0;
   out_682184522508178564[285] = 1;
   out_682184522508178564[286] = 0;
   out_682184522508178564[287] = 0;
   out_682184522508178564[288] = 0;
   out_682184522508178564[289] = 0;
   out_682184522508178564[290] = 0;
   out_682184522508178564[291] = 0;
   out_682184522508178564[292] = 0;
   out_682184522508178564[293] = 0;
   out_682184522508178564[294] = 0;
   out_682184522508178564[295] = 0;
   out_682184522508178564[296] = 0;
   out_682184522508178564[297] = 0;
   out_682184522508178564[298] = 0;
   out_682184522508178564[299] = 0;
   out_682184522508178564[300] = 0;
   out_682184522508178564[301] = 0;
   out_682184522508178564[302] = 0;
   out_682184522508178564[303] = 0;
   out_682184522508178564[304] = 1;
   out_682184522508178564[305] = 0;
   out_682184522508178564[306] = 0;
   out_682184522508178564[307] = 0;
   out_682184522508178564[308] = 0;
   out_682184522508178564[309] = 0;
   out_682184522508178564[310] = 0;
   out_682184522508178564[311] = 0;
   out_682184522508178564[312] = 0;
   out_682184522508178564[313] = 0;
   out_682184522508178564[314] = 0;
   out_682184522508178564[315] = 0;
   out_682184522508178564[316] = 0;
   out_682184522508178564[317] = 0;
   out_682184522508178564[318] = 0;
   out_682184522508178564[319] = 0;
   out_682184522508178564[320] = 0;
   out_682184522508178564[321] = 0;
   out_682184522508178564[322] = 0;
   out_682184522508178564[323] = 1;
}
void h_4(double *state, double *unused, double *out_8188971748029855794) {
   out_8188971748029855794[0] = state[6] + state[9];
   out_8188971748029855794[1] = state[7] + state[10];
   out_8188971748029855794[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2591859041745323020) {
   out_2591859041745323020[0] = 0;
   out_2591859041745323020[1] = 0;
   out_2591859041745323020[2] = 0;
   out_2591859041745323020[3] = 0;
   out_2591859041745323020[4] = 0;
   out_2591859041745323020[5] = 0;
   out_2591859041745323020[6] = 1;
   out_2591859041745323020[7] = 0;
   out_2591859041745323020[8] = 0;
   out_2591859041745323020[9] = 1;
   out_2591859041745323020[10] = 0;
   out_2591859041745323020[11] = 0;
   out_2591859041745323020[12] = 0;
   out_2591859041745323020[13] = 0;
   out_2591859041745323020[14] = 0;
   out_2591859041745323020[15] = 0;
   out_2591859041745323020[16] = 0;
   out_2591859041745323020[17] = 0;
   out_2591859041745323020[18] = 0;
   out_2591859041745323020[19] = 0;
   out_2591859041745323020[20] = 0;
   out_2591859041745323020[21] = 0;
   out_2591859041745323020[22] = 0;
   out_2591859041745323020[23] = 0;
   out_2591859041745323020[24] = 0;
   out_2591859041745323020[25] = 1;
   out_2591859041745323020[26] = 0;
   out_2591859041745323020[27] = 0;
   out_2591859041745323020[28] = 1;
   out_2591859041745323020[29] = 0;
   out_2591859041745323020[30] = 0;
   out_2591859041745323020[31] = 0;
   out_2591859041745323020[32] = 0;
   out_2591859041745323020[33] = 0;
   out_2591859041745323020[34] = 0;
   out_2591859041745323020[35] = 0;
   out_2591859041745323020[36] = 0;
   out_2591859041745323020[37] = 0;
   out_2591859041745323020[38] = 0;
   out_2591859041745323020[39] = 0;
   out_2591859041745323020[40] = 0;
   out_2591859041745323020[41] = 0;
   out_2591859041745323020[42] = 0;
   out_2591859041745323020[43] = 0;
   out_2591859041745323020[44] = 1;
   out_2591859041745323020[45] = 0;
   out_2591859041745323020[46] = 0;
   out_2591859041745323020[47] = 1;
   out_2591859041745323020[48] = 0;
   out_2591859041745323020[49] = 0;
   out_2591859041745323020[50] = 0;
   out_2591859041745323020[51] = 0;
   out_2591859041745323020[52] = 0;
   out_2591859041745323020[53] = 0;
}
void h_10(double *state, double *unused, double *out_8240345718422408180) {
   out_8240345718422408180[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8240345718422408180[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8240345718422408180[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3189726223905577658) {
   out_3189726223905577658[0] = 0;
   out_3189726223905577658[1] = 9.8100000000000005*cos(state[1]);
   out_3189726223905577658[2] = 0;
   out_3189726223905577658[3] = 0;
   out_3189726223905577658[4] = -state[8];
   out_3189726223905577658[5] = state[7];
   out_3189726223905577658[6] = 0;
   out_3189726223905577658[7] = state[5];
   out_3189726223905577658[8] = -state[4];
   out_3189726223905577658[9] = 0;
   out_3189726223905577658[10] = 0;
   out_3189726223905577658[11] = 0;
   out_3189726223905577658[12] = 1;
   out_3189726223905577658[13] = 0;
   out_3189726223905577658[14] = 0;
   out_3189726223905577658[15] = 1;
   out_3189726223905577658[16] = 0;
   out_3189726223905577658[17] = 0;
   out_3189726223905577658[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3189726223905577658[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3189726223905577658[20] = 0;
   out_3189726223905577658[21] = state[8];
   out_3189726223905577658[22] = 0;
   out_3189726223905577658[23] = -state[6];
   out_3189726223905577658[24] = -state[5];
   out_3189726223905577658[25] = 0;
   out_3189726223905577658[26] = state[3];
   out_3189726223905577658[27] = 0;
   out_3189726223905577658[28] = 0;
   out_3189726223905577658[29] = 0;
   out_3189726223905577658[30] = 0;
   out_3189726223905577658[31] = 1;
   out_3189726223905577658[32] = 0;
   out_3189726223905577658[33] = 0;
   out_3189726223905577658[34] = 1;
   out_3189726223905577658[35] = 0;
   out_3189726223905577658[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3189726223905577658[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3189726223905577658[38] = 0;
   out_3189726223905577658[39] = -state[7];
   out_3189726223905577658[40] = state[6];
   out_3189726223905577658[41] = 0;
   out_3189726223905577658[42] = state[4];
   out_3189726223905577658[43] = -state[3];
   out_3189726223905577658[44] = 0;
   out_3189726223905577658[45] = 0;
   out_3189726223905577658[46] = 0;
   out_3189726223905577658[47] = 0;
   out_3189726223905577658[48] = 0;
   out_3189726223905577658[49] = 0;
   out_3189726223905577658[50] = 1;
   out_3189726223905577658[51] = 0;
   out_3189726223905577658[52] = 0;
   out_3189726223905577658[53] = 1;
}
void h_13(double *state, double *unused, double *out_8973495704883649188) {
   out_8973495704883649188[0] = state[3];
   out_8973495704883649188[1] = state[4];
   out_8973495704883649188[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5804132867077655821) {
   out_5804132867077655821[0] = 0;
   out_5804132867077655821[1] = 0;
   out_5804132867077655821[2] = 0;
   out_5804132867077655821[3] = 1;
   out_5804132867077655821[4] = 0;
   out_5804132867077655821[5] = 0;
   out_5804132867077655821[6] = 0;
   out_5804132867077655821[7] = 0;
   out_5804132867077655821[8] = 0;
   out_5804132867077655821[9] = 0;
   out_5804132867077655821[10] = 0;
   out_5804132867077655821[11] = 0;
   out_5804132867077655821[12] = 0;
   out_5804132867077655821[13] = 0;
   out_5804132867077655821[14] = 0;
   out_5804132867077655821[15] = 0;
   out_5804132867077655821[16] = 0;
   out_5804132867077655821[17] = 0;
   out_5804132867077655821[18] = 0;
   out_5804132867077655821[19] = 0;
   out_5804132867077655821[20] = 0;
   out_5804132867077655821[21] = 0;
   out_5804132867077655821[22] = 1;
   out_5804132867077655821[23] = 0;
   out_5804132867077655821[24] = 0;
   out_5804132867077655821[25] = 0;
   out_5804132867077655821[26] = 0;
   out_5804132867077655821[27] = 0;
   out_5804132867077655821[28] = 0;
   out_5804132867077655821[29] = 0;
   out_5804132867077655821[30] = 0;
   out_5804132867077655821[31] = 0;
   out_5804132867077655821[32] = 0;
   out_5804132867077655821[33] = 0;
   out_5804132867077655821[34] = 0;
   out_5804132867077655821[35] = 0;
   out_5804132867077655821[36] = 0;
   out_5804132867077655821[37] = 0;
   out_5804132867077655821[38] = 0;
   out_5804132867077655821[39] = 0;
   out_5804132867077655821[40] = 0;
   out_5804132867077655821[41] = 1;
   out_5804132867077655821[42] = 0;
   out_5804132867077655821[43] = 0;
   out_5804132867077655821[44] = 0;
   out_5804132867077655821[45] = 0;
   out_5804132867077655821[46] = 0;
   out_5804132867077655821[47] = 0;
   out_5804132867077655821[48] = 0;
   out_5804132867077655821[49] = 0;
   out_5804132867077655821[50] = 0;
   out_5804132867077655821[51] = 0;
   out_5804132867077655821[52] = 0;
   out_5804132867077655821[53] = 0;
}
void h_14(double *state, double *unused, double *out_6947000857610459247) {
   out_6947000857610459247[0] = state[6];
   out_6947000857610459247[1] = state[7];
   out_6947000857610459247[2] = state[8];
}
void H_14(double *state, double *unused, double *out_490929390550049276) {
   out_490929390550049276[0] = 0;
   out_490929390550049276[1] = 0;
   out_490929390550049276[2] = 0;
   out_490929390550049276[3] = 0;
   out_490929390550049276[4] = 0;
   out_490929390550049276[5] = 0;
   out_490929390550049276[6] = 1;
   out_490929390550049276[7] = 0;
   out_490929390550049276[8] = 0;
   out_490929390550049276[9] = 0;
   out_490929390550049276[10] = 0;
   out_490929390550049276[11] = 0;
   out_490929390550049276[12] = 0;
   out_490929390550049276[13] = 0;
   out_490929390550049276[14] = 0;
   out_490929390550049276[15] = 0;
   out_490929390550049276[16] = 0;
   out_490929390550049276[17] = 0;
   out_490929390550049276[18] = 0;
   out_490929390550049276[19] = 0;
   out_490929390550049276[20] = 0;
   out_490929390550049276[21] = 0;
   out_490929390550049276[22] = 0;
   out_490929390550049276[23] = 0;
   out_490929390550049276[24] = 0;
   out_490929390550049276[25] = 1;
   out_490929390550049276[26] = 0;
   out_490929390550049276[27] = 0;
   out_490929390550049276[28] = 0;
   out_490929390550049276[29] = 0;
   out_490929390550049276[30] = 0;
   out_490929390550049276[31] = 0;
   out_490929390550049276[32] = 0;
   out_490929390550049276[33] = 0;
   out_490929390550049276[34] = 0;
   out_490929390550049276[35] = 0;
   out_490929390550049276[36] = 0;
   out_490929390550049276[37] = 0;
   out_490929390550049276[38] = 0;
   out_490929390550049276[39] = 0;
   out_490929390550049276[40] = 0;
   out_490929390550049276[41] = 0;
   out_490929390550049276[42] = 0;
   out_490929390550049276[43] = 0;
   out_490929390550049276[44] = 1;
   out_490929390550049276[45] = 0;
   out_490929390550049276[46] = 0;
   out_490929390550049276[47] = 0;
   out_490929390550049276[48] = 0;
   out_490929390550049276[49] = 0;
   out_490929390550049276[50] = 0;
   out_490929390550049276[51] = 0;
   out_490929390550049276[52] = 0;
   out_490929390550049276[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_6300647231984494843) {
  err_fun(nom_x, delta_x, out_6300647231984494843);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_4334582594945192021) {
  inv_err_fun(nom_x, true_x, out_4334582594945192021);
}
void pose_H_mod_fun(double *state, double *out_1393911559862424916) {
  H_mod_fun(state, out_1393911559862424916);
}
void pose_f_fun(double *state, double dt, double *out_7637055358137252657) {
  f_fun(state,  dt, out_7637055358137252657);
}
void pose_F_fun(double *state, double dt, double *out_682184522508178564) {
  F_fun(state,  dt, out_682184522508178564);
}
void pose_h_4(double *state, double *unused, double *out_8188971748029855794) {
  h_4(state, unused, out_8188971748029855794);
}
void pose_H_4(double *state, double *unused, double *out_2591859041745323020) {
  H_4(state, unused, out_2591859041745323020);
}
void pose_h_10(double *state, double *unused, double *out_8240345718422408180) {
  h_10(state, unused, out_8240345718422408180);
}
void pose_H_10(double *state, double *unused, double *out_3189726223905577658) {
  H_10(state, unused, out_3189726223905577658);
}
void pose_h_13(double *state, double *unused, double *out_8973495704883649188) {
  h_13(state, unused, out_8973495704883649188);
}
void pose_H_13(double *state, double *unused, double *out_5804132867077655821) {
  H_13(state, unused, out_5804132867077655821);
}
void pose_h_14(double *state, double *unused, double *out_6947000857610459247) {
  h_14(state, unused, out_6947000857610459247);
}
void pose_H_14(double *state, double *unused, double *out_490929390550049276) {
  H_14(state, unused, out_490929390550049276);
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
