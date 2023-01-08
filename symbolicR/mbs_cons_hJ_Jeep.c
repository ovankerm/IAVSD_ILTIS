//
//	MBsysTran - Release 8.1
//
//	Copyright 
//	Universite catholique de Louvain (UCLouvain) 
//	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
//	2, Place du Levant
//	1348 Louvain-la-Neuve 
//	Belgium 
//
//	http://www.robotran.be 
//
//	==> Generation Date: Sun Jan  8 23:29:46 2023
//
//	==> Project name: Jeep
//
//	==> Number of joints: 41
//
//	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
//
//	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
//

#include <math.h> 

#include "mbs_data.h"

void mbs_cons_hJ(double *h, double **Jac,
MbsData *s, double tsim)
{
#include "mbs_cons_hJ_Jeep.h"

double *q, *qd;
double **dpt, *lrod;

q = s->q;
qd = s->qd;

dpt = s->dpt;
lrod = s->lrod;
 
// Trigonometric functions

S7 = sin(q[7]);
C7 = cos(q[7]);
S8 = sin(q[8]);
C8 = cos(q[8]);
S9 = sin(q[9]);
C9 = cos(q[9]);
S14 = sin(q[14]);
C14 = cos(q[14]);
S17 = sin(q[17]);
C17 = cos(q[17]);
S18 = sin(q[18]);
C18 = cos(q[18]);
S31 = sin(q[31]);
C31 = cos(q[31]);
S32 = sin(q[32]);
C32 = cos(q[32]);
S33 = sin(q[33]);
C33 = cos(q[33]);
S22 = sin(q[22]);
C22 = cos(q[22]);
S25 = sin(q[25]);
C25 = cos(q[25]);
S26 = sin(q[26]);
C26 = cos(q[26]);
S10 = sin(q[10]);
C10 = cos(q[10]);
S19 = sin(q[19]);
C19 = cos(q[19]);
S34 = sin(q[34]);
C34 = cos(q[34]);
S27 = sin(q[27]);
C27 = cos(q[27]);
 
// Augmented Joint Position Vectors

Dz413 = q[41]+dpt[3][17];
 
// Augmented Joint Position Vectors

 
// Constraints and Constraints Jacobian

ROlp2_52 = C7*C8-S7*S8;
ROlp2_62 = C7*S8+S7*C8;
ROlp2_82 = -C7*S8-S7*C8;
ROlp2_92 = C7*C8-S7*S8;
ROlp2_53 = ROlp2_52*C9+ROlp2_82*S9;
ROlp2_63 = ROlp2_62*C9+ROlp2_92*S9;
RLlp2_22 = dpt[2][18]*C7;
RLlp2_32 = dpt[2][18]*S7;
POlp2_22 = RLlp2_22+dpt[2][2];
POlp2_32 = RLlp2_32+dpt[3][2];
RLlp2_23 = ROlp2_82*dpt[3][19];
RLlp2_33 = ROlp2_92*dpt[3][19];
POlp2_23 = POlp2_22+RLlp2_23;
POlp2_33 = POlp2_32+RLlp2_33;
JTlp2_23_1 = -RLlp2_32-RLlp2_33;
JTlp2_33_1 = RLlp2_22+RLlp2_23;
RLlp2_24 = ROlp2_53*dpt[2][21];
RLlp2_34 = ROlp2_63*dpt[2][21];
POlp2_24 = POlp2_23+RLlp2_24;
POlp2_34 = POlp2_33+RLlp2_34;
JTlp2_24_1 = JTlp2_23_1-RLlp2_34;
JTlp2_34_1 = JTlp2_33_1+RLlp2_24;
JTlp2_24_2 = -RLlp2_33-RLlp2_34;
JTlp2_34_2 = RLlp2_23+RLlp2_24;
h_2 = -POlp2_24+dpt[2][1];
h_3 = -POlp2_34+dpt[3][1];
ROlp4_54 = C14*C17-S14*S17;
ROlp4_64 = C14*S17+S14*C17;
ROlp4_84 = -C14*S17-S14*C17;
ROlp4_94 = C14*C17-S14*S17;
ROlp4_55 = ROlp4_54*C18+ROlp4_84*S18;
ROlp4_65 = ROlp4_64*C18+ROlp4_94*S18;
RLlp4_22 = q[15]*C14;
RLlp4_32 = q[15]*S14;
POlp4_22 = RLlp4_22+dpt[2][4];
POlp4_32 = RLlp4_32+dpt[3][4];
RLlp4_23 = -q[16]*S14;
RLlp4_33 = q[16]*C14;
POlp4_23 = POlp4_22+RLlp4_23;
POlp4_33 = POlp4_32+RLlp4_33;
JTlp4_23_1 = -RLlp4_32-RLlp4_33;
JTlp4_33_1 = RLlp4_22+RLlp4_23;
RLlp4_24 = dpt[2][29]*C14;
RLlp4_34 = dpt[2][29]*S14;
POlp4_24 = POlp4_23+RLlp4_24;
POlp4_34 = POlp4_33+RLlp4_34;
JTlp4_24_1 = JTlp4_23_1-RLlp4_34;
JTlp4_34_1 = JTlp4_33_1+RLlp4_24;
RLlp4_25 = ROlp4_84*dpt[3][30];
RLlp4_35 = ROlp4_94*dpt[3][30];
POlp4_25 = POlp4_24+RLlp4_25;
POlp4_35 = POlp4_34+RLlp4_35;
JTlp4_25_1 = JTlp4_24_1-RLlp4_35;
JTlp4_35_1 = JTlp4_34_1+RLlp4_25;
RLlp4_26 = ROlp4_55*dpt[2][32];
RLlp4_36 = ROlp4_65*dpt[2][32];
POlp4_26 = POlp4_25+RLlp4_26;
POlp4_36 = POlp4_35+RLlp4_36;
JTlp4_26_1 = JTlp4_25_1-RLlp4_36;
JTlp4_36_1 = JTlp4_35_1+RLlp4_26;
JTlp4_26_4 = -RLlp4_35-RLlp4_36;
JTlp4_36_4 = RLlp4_25+RLlp4_26;
h_5 = -POlp4_26+dpt[2][5];
h_6 = -POlp4_36+dpt[3][5];
ROlp6_52 = C31*C32-S31*S32;
ROlp6_62 = C31*S32+S31*C32;
ROlp6_82 = -C31*S32-S31*C32;
ROlp6_92 = C31*C32-S31*S32;
ROlp6_53 = ROlp6_52*C33+ROlp6_82*S33;
ROlp6_63 = ROlp6_62*C33+ROlp6_92*S33;
RLlp6_22 = dpt[2][49]*C31;
RLlp6_32 = dpt[2][49]*S31;
POlp6_22 = RLlp6_22+dpt[2][10];
POlp6_32 = RLlp6_32+dpt[3][10];
RLlp6_23 = ROlp6_82*dpt[3][50];
RLlp6_33 = ROlp6_92*dpt[3][50];
POlp6_23 = POlp6_22+RLlp6_23;
POlp6_33 = POlp6_32+RLlp6_33;
JTlp6_23_1 = -RLlp6_32-RLlp6_33;
JTlp6_33_1 = RLlp6_22+RLlp6_23;
RLlp6_24 = ROlp6_53*dpt[2][52];
RLlp6_34 = ROlp6_63*dpt[2][52];
POlp6_24 = POlp6_23+RLlp6_24;
POlp6_34 = POlp6_33+RLlp6_34;
JTlp6_24_1 = JTlp6_23_1-RLlp6_34;
JTlp6_34_1 = JTlp6_33_1+RLlp6_24;
JTlp6_24_2 = -RLlp6_33-RLlp6_34;
JTlp6_34_2 = RLlp6_23+RLlp6_24;
h_8 = -POlp6_24+dpt[2][9];
h_9 = -POlp6_34+dpt[3][9];
ROlp8_54 = C22*C25-S22*S25;
ROlp8_64 = C22*S25+S22*C25;
ROlp8_84 = -C22*S25-S22*C25;
ROlp8_94 = C22*C25-S22*S25;
ROlp8_55 = ROlp8_54*C26+ROlp8_84*S26;
ROlp8_65 = ROlp8_64*C26+ROlp8_94*S26;
RLlp8_22 = q[23]*C22;
RLlp8_32 = q[23]*S22;
POlp8_22 = RLlp8_22+dpt[2][7];
POlp8_32 = RLlp8_32+dpt[3][7];
RLlp8_23 = -q[24]*S22;
RLlp8_33 = q[24]*C22;
POlp8_23 = POlp8_22+RLlp8_23;
POlp8_33 = POlp8_32+RLlp8_33;
JTlp8_23_1 = -RLlp8_32-RLlp8_33;
JTlp8_33_1 = RLlp8_22+RLlp8_23;
RLlp8_24 = dpt[2][38]*C22;
RLlp8_34 = dpt[2][38]*S22;
POlp8_24 = POlp8_23+RLlp8_24;
POlp8_34 = POlp8_33+RLlp8_34;
JTlp8_24_1 = JTlp8_23_1-RLlp8_34;
JTlp8_34_1 = JTlp8_33_1+RLlp8_24;
RLlp8_25 = ROlp8_84*dpt[3][39];
RLlp8_35 = ROlp8_94*dpt[3][39];
POlp8_25 = POlp8_24+RLlp8_25;
POlp8_35 = POlp8_34+RLlp8_35;
JTlp8_25_1 = JTlp8_24_1-RLlp8_35;
JTlp8_35_1 = JTlp8_34_1+RLlp8_25;
RLlp8_26 = ROlp8_55*dpt[2][41];
RLlp8_36 = ROlp8_65*dpt[2][41];
POlp8_26 = POlp8_25+RLlp8_26;
POlp8_36 = POlp8_35+RLlp8_36;
JTlp8_26_1 = JTlp8_25_1-RLlp8_36;
JTlp8_36_1 = JTlp8_35_1+RLlp8_26;
JTlp8_26_4 = -RLlp8_35-RLlp8_36;
JTlp8_36_4 = RLlp8_25+RLlp8_26;
h_11 = -POlp8_26+dpt[2][6];
h_12 = -POlp8_36+dpt[3][6];
POlp9_22 = q[13]+dpt[2][27];
ROlp10_52 = C7*C8-S7*S8;
ROlp10_62 = C7*S8+S7*C8;
ROlp10_82 = -C7*S8-S7*C8;
ROlp10_92 = C7*C8-S7*S8;
ROlp10_23 = ROlp10_52*S10;
ROlp10_33 = ROlp10_62*S10;
ROlp10_53 = ROlp10_52*C10;
ROlp10_63 = ROlp10_62*C10;
RLlp10_22 = dpt[2][18]*C7;
RLlp10_32 = dpt[2][18]*S7;
POlp10_22 = RLlp10_22+dpt[2][2];
POlp10_32 = RLlp10_32+dpt[3][2];
RLlp10_23 = ROlp10_82*dpt[3][20];
RLlp10_33 = ROlp10_92*dpt[3][20];
POlp10_23 = POlp10_22+RLlp10_23;
POlp10_33 = POlp10_32+RLlp10_33;
JTlp10_23_1 = -RLlp10_32-RLlp10_33;
JTlp10_33_1 = RLlp10_22+RLlp10_23;
RLlp10_14 = dpt[1][24]*C10-dpt[2][24]*S10;
RLlp10_24 = ROlp10_23*dpt[1][24]+ROlp10_53*dpt[2][24]+ROlp10_82*dpt[3][24];
RLlp10_34 = ROlp10_33*dpt[1][24]+ROlp10_63*dpt[2][24]+ROlp10_92*dpt[3][24];
POlp10_14 = RLlp10_14+dpt[1][2];
POlp10_24 = POlp10_23+RLlp10_24;
POlp10_34 = POlp10_33+RLlp10_34;
JTlp10_24_1 = JTlp10_23_1-RLlp10_34;
JTlp10_34_1 = JTlp10_33_1+RLlp10_24;
JTlp10_24_2 = -RLlp10_33-RLlp10_34;
JTlp10_34_2 = RLlp10_23+RLlp10_24;
JTlp10_14_3 = -RLlp10_24*ROlp10_92+RLlp10_34*ROlp10_82;
JTlp10_24_3 = RLlp10_14*ROlp10_92;
JTlp10_34_3 = -RLlp10_14*ROlp10_82;
Plp11 = -POlp10_14+dpt[1][3];
Plp21 = -POlp10_24+POlp9_22;
Plp31 = -POlp10_34+dpt[3][3];
P2lp1 = Plp11*Plp11+Plp21*Plp21+Plp31*Plp31;
l2rod1 = lrod[1]*lrod[1];
h_13 = (0.50)*(P2lp1-l2rod1);
Jac_13_7 = -JTlp10_24_1*Plp21-JTlp10_34_1*Plp31;
Jac_13_8 = -JTlp10_24_2*Plp21-JTlp10_34_2*Plp31;
Jac_13_10 = -JTlp10_14_3*Plp11-JTlp10_24_3*Plp21-JTlp10_34_3*Plp31;
POlp11_22 = q[13]+dpt[2][28];
ROlp12_54 = C14*C17-S14*S17;
ROlp12_64 = C14*S17+S14*C17;
ROlp12_84 = -C14*S17-S14*C17;
ROlp12_94 = C14*C17-S14*S17;
ROlp12_25 = ROlp12_54*S19;
ROlp12_35 = ROlp12_64*S19;
ROlp12_55 = ROlp12_54*C19;
ROlp12_65 = ROlp12_64*C19;
RLlp12_22 = q[15]*C14;
RLlp12_32 = q[15]*S14;
POlp12_22 = RLlp12_22+dpt[2][4];
POlp12_32 = RLlp12_32+dpt[3][4];
RLlp12_23 = -q[16]*S14;
RLlp12_33 = q[16]*C14;
POlp12_23 = POlp12_22+RLlp12_23;
POlp12_33 = POlp12_32+RLlp12_33;
JTlp12_23_1 = -RLlp12_32-RLlp12_33;
JTlp12_33_1 = RLlp12_22+RLlp12_23;
RLlp12_24 = dpt[2][29]*C14;
RLlp12_34 = dpt[2][29]*S14;
POlp12_24 = POlp12_23+RLlp12_24;
POlp12_34 = POlp12_33+RLlp12_34;
JTlp12_24_1 = JTlp12_23_1-RLlp12_34;
JTlp12_34_1 = JTlp12_33_1+RLlp12_24;
RLlp12_25 = ROlp12_84*dpt[3][31];
RLlp12_35 = ROlp12_94*dpt[3][31];
POlp12_25 = POlp12_24+RLlp12_25;
POlp12_35 = POlp12_34+RLlp12_35;
JTlp12_25_1 = JTlp12_24_1-RLlp12_35;
JTlp12_35_1 = JTlp12_34_1+RLlp12_25;
RLlp12_16 = dpt[1][35]*C19-dpt[2][35]*S19;
RLlp12_26 = ROlp12_25*dpt[1][35]+ROlp12_55*dpt[2][35]+ROlp12_84*dpt[3][35];
RLlp12_36 = ROlp12_35*dpt[1][35]+ROlp12_65*dpt[2][35]+ROlp12_94*dpt[3][35];
POlp12_16 = RLlp12_16+dpt[1][4];
POlp12_26 = POlp12_25+RLlp12_26;
POlp12_36 = POlp12_35+RLlp12_36;
JTlp12_26_1 = JTlp12_25_1-RLlp12_36;
JTlp12_36_1 = JTlp12_35_1+RLlp12_26;
JTlp12_26_4 = -RLlp12_35-RLlp12_36;
JTlp12_36_4 = RLlp12_25+RLlp12_26;
JTlp12_16_5 = -RLlp12_26*ROlp12_94+RLlp12_36*ROlp12_84;
JTlp12_26_5 = RLlp12_16*ROlp12_94;
JTlp12_36_5 = -RLlp12_16*ROlp12_84;
Plp12 = -POlp12_16+dpt[1][3];
Plp22 = POlp11_22-POlp12_26;
Plp32 = -POlp12_36+dpt[3][3];
P2lp2 = Plp12*Plp12+Plp22*Plp22+Plp32*Plp32;
l2rod2 = lrod[2]*lrod[2];
h_14 = (0.50)*(P2lp2-l2rod2);
Jac_14_14 = -JTlp12_26_1*Plp22-JTlp12_36_1*Plp32;
Jac_14_15 = -Plp22*C14-Plp32*S14;
Jac_14_16 = Plp22*S14-Plp32*C14;
Jac_14_17 = -JTlp12_26_4*Plp22-JTlp12_36_4*Plp32;
Jac_14_19 = -JTlp12_16_5*Plp12-JTlp12_26_5*Plp22-JTlp12_36_5*Plp32;
POlp13_22 = q[30]+dpt[2][47];
ROlp14_52 = C31*C32-S31*S32;
ROlp14_62 = C31*S32+S31*C32;
ROlp14_82 = -C31*S32-S31*C32;
ROlp14_92 = C31*C32-S31*S32;
ROlp14_23 = ROlp14_52*S34;
ROlp14_33 = ROlp14_62*S34;
ROlp14_53 = ROlp14_52*C34;
ROlp14_63 = ROlp14_62*C34;
RLlp14_22 = dpt[2][49]*C31;
RLlp14_32 = dpt[2][49]*S31;
POlp14_22 = RLlp14_22+dpt[2][10];
POlp14_32 = RLlp14_32+dpt[3][10];
RLlp14_23 = ROlp14_82*dpt[3][51];
RLlp14_33 = ROlp14_92*dpt[3][51];
POlp14_23 = POlp14_22+RLlp14_23;
POlp14_33 = POlp14_32+RLlp14_33;
JTlp14_23_1 = -RLlp14_32-RLlp14_33;
JTlp14_33_1 = RLlp14_22+RLlp14_23;
RLlp14_14 = dpt[1][55]*C34-dpt[2][55]*S34;
RLlp14_24 = ROlp14_23*dpt[1][55]+ROlp14_53*dpt[2][55]+ROlp14_82*dpt[3][55];
RLlp14_34 = ROlp14_33*dpt[1][55]+ROlp14_63*dpt[2][55]+ROlp14_92*dpt[3][55];
POlp14_14 = RLlp14_14+dpt[1][10];
POlp14_24 = POlp14_23+RLlp14_24;
POlp14_34 = POlp14_33+RLlp14_34;
JTlp14_24_1 = JTlp14_23_1-RLlp14_34;
JTlp14_34_1 = JTlp14_33_1+RLlp14_24;
JTlp14_24_2 = -RLlp14_33-RLlp14_34;
JTlp14_34_2 = RLlp14_23+RLlp14_24;
JTlp14_14_3 = -RLlp14_24*ROlp14_92+RLlp14_34*ROlp14_82;
JTlp14_24_3 = RLlp14_14*ROlp14_92;
JTlp14_34_3 = -RLlp14_14*ROlp14_82;
Plp13 = -POlp14_14+dpt[1][8];
Plp23 = POlp13_22-POlp14_24;
Plp33 = -POlp14_34+dpt[3][8];
P2lp3 = Plp13*Plp13+Plp23*Plp23+Plp33*Plp33;
l2rod3 = lrod[3]*lrod[3];
h_15 = (0.50)*(P2lp3-l2rod3);
Jac_15_31 = -JTlp14_24_1*Plp23-JTlp14_34_1*Plp33;
Jac_15_32 = -JTlp14_24_2*Plp23-JTlp14_34_2*Plp33;
Jac_15_34 = -JTlp14_14_3*Plp13-JTlp14_24_3*Plp23-JTlp14_34_3*Plp33;
POlp15_22 = q[30]+dpt[2][48];
ROlp16_54 = C22*C25-S22*S25;
ROlp16_64 = C22*S25+S22*C25;
ROlp16_84 = -C22*S25-S22*C25;
ROlp16_94 = C22*C25-S22*S25;
ROlp16_25 = ROlp16_54*S27;
ROlp16_35 = ROlp16_64*S27;
ROlp16_55 = ROlp16_54*C27;
ROlp16_65 = ROlp16_64*C27;
RLlp16_22 = q[23]*C22;
RLlp16_32 = q[23]*S22;
POlp16_22 = RLlp16_22+dpt[2][7];
POlp16_32 = RLlp16_32+dpt[3][7];
RLlp16_23 = -q[24]*S22;
RLlp16_33 = q[24]*C22;
POlp16_23 = POlp16_22+RLlp16_23;
POlp16_33 = POlp16_32+RLlp16_33;
JTlp16_23_1 = -RLlp16_32-RLlp16_33;
JTlp16_33_1 = RLlp16_22+RLlp16_23;
RLlp16_24 = dpt[2][38]*C22;
RLlp16_34 = dpt[2][38]*S22;
POlp16_24 = POlp16_23+RLlp16_24;
POlp16_34 = POlp16_33+RLlp16_34;
JTlp16_24_1 = JTlp16_23_1-RLlp16_34;
JTlp16_34_1 = JTlp16_33_1+RLlp16_24;
RLlp16_25 = ROlp16_84*dpt[3][40];
RLlp16_35 = ROlp16_94*dpt[3][40];
POlp16_25 = POlp16_24+RLlp16_25;
POlp16_35 = POlp16_34+RLlp16_35;
JTlp16_25_1 = JTlp16_24_1-RLlp16_35;
JTlp16_35_1 = JTlp16_34_1+RLlp16_25;
RLlp16_16 = dpt[1][44]*C27-dpt[2][44]*S27;
RLlp16_26 = ROlp16_25*dpt[1][44]+ROlp16_55*dpt[2][44]+ROlp16_84*dpt[3][44];
RLlp16_36 = ROlp16_35*dpt[1][44]+ROlp16_65*dpt[2][44]+ROlp16_94*dpt[3][44];
POlp16_16 = RLlp16_16+dpt[1][7];
POlp16_26 = POlp16_25+RLlp16_26;
POlp16_36 = POlp16_35+RLlp16_36;
JTlp16_26_1 = JTlp16_25_1-RLlp16_36;
JTlp16_36_1 = JTlp16_35_1+RLlp16_26;
JTlp16_26_4 = -RLlp16_35-RLlp16_36;
JTlp16_36_4 = RLlp16_25+RLlp16_26;
JTlp16_16_5 = -RLlp16_26*ROlp16_94+RLlp16_36*ROlp16_84;
JTlp16_26_5 = RLlp16_16*ROlp16_94;
JTlp16_36_5 = -RLlp16_16*ROlp16_84;
Plp14 = -POlp16_16+dpt[1][8];
Plp24 = POlp15_22-POlp16_26;
Plp34 = -POlp16_36+dpt[3][8];
P2lp4 = Plp14*Plp14+Plp24*Plp24+Plp34*Plp34;
l2rod4 = lrod[4]*lrod[4];
h_16 = (0.50)*(P2lp4-l2rod4);
Jac_16_22 = -JTlp16_26_1*Plp24-JTlp16_36_1*Plp34;
Jac_16_23 = -Plp24*C22-Plp34*S22;
Jac_16_24 = Plp24*S22-Plp34*C22;
Jac_16_25 = -JTlp16_26_4*Plp24-JTlp16_36_4*Plp34;
Jac_16_27 = -JTlp16_16_5*Plp14-JTlp16_26_5*Plp24-JTlp16_36_5*Plp34;
h[1] = h_2;
h[2] = h_3;
h[3] = h_5;
h[4] = h_6;
h[5] = h_8;
h[6] = h_9;
h[7] = h_11;
h[8] = h_12;
h[9] = h_13;
h[10] = h_14;
h[11] = h_15;
h[12] = h_16;
Jac[1][1] = 0;
Jac[1][2] = 0;
Jac[1][3] = 0;
Jac[1][4] = 0;
Jac[1][5] = 0;
Jac[1][6] = 0;
Jac[1][7] = -JTlp2_24_1;
Jac[1][8] = -JTlp2_24_2;
Jac[1][9] = RLlp2_34;
Jac[1][10] = 0;
Jac[1][11] = 0;
Jac[1][12] = 0;
Jac[1][13] = 0;
Jac[1][14] = 0;
Jac[1][15] = 0;
Jac[1][16] = 0;
Jac[1][17] = 0;
Jac[1][18] = 0;
Jac[1][19] = 0;
Jac[1][20] = 0;
Jac[1][21] = 0;
Jac[1][22] = 0;
Jac[1][23] = 0;
Jac[1][24] = 0;
Jac[1][25] = 0;
Jac[1][26] = 0;
Jac[1][27] = 0;
Jac[1][28] = 0;
Jac[1][29] = 0;
Jac[1][30] = 0;
Jac[1][31] = 0;
Jac[1][32] = 0;
Jac[1][33] = 0;
Jac[1][34] = 0;
Jac[1][35] = 0;
Jac[1][36] = 0;
Jac[1][37] = 0;
Jac[1][38] = 0;
Jac[1][39] = 0;
Jac[1][40] = 0;
Jac[1][41] = 0;
Jac[2][1] = 0;
Jac[2][2] = 0;
Jac[2][3] = 0;
Jac[2][4] = 0;
Jac[2][5] = 0;
Jac[2][6] = 0;
Jac[2][7] = -JTlp2_34_1;
Jac[2][8] = -JTlp2_34_2;
Jac[2][9] = -RLlp2_24;
Jac[2][10] = 0;
Jac[2][11] = 0;
Jac[2][12] = 0;
Jac[2][13] = 0;
Jac[2][14] = 0;
Jac[2][15] = 0;
Jac[2][16] = 0;
Jac[2][17] = 0;
Jac[2][18] = 0;
Jac[2][19] = 0;
Jac[2][20] = 0;
Jac[2][21] = 0;
Jac[2][22] = 0;
Jac[2][23] = 0;
Jac[2][24] = 0;
Jac[2][25] = 0;
Jac[2][26] = 0;
Jac[2][27] = 0;
Jac[2][28] = 0;
Jac[2][29] = 0;
Jac[2][30] = 0;
Jac[2][31] = 0;
Jac[2][32] = 0;
Jac[2][33] = 0;
Jac[2][34] = 0;
Jac[2][35] = 0;
Jac[2][36] = 0;
Jac[2][37] = 0;
Jac[2][38] = 0;
Jac[2][39] = 0;
Jac[2][40] = 0;
Jac[2][41] = 0;
Jac[3][1] = 0;
Jac[3][2] = 0;
Jac[3][3] = 0;
Jac[3][4] = 0;
Jac[3][5] = 0;
Jac[3][6] = 0;
Jac[3][7] = 0;
Jac[3][8] = 0;
Jac[3][9] = 0;
Jac[3][10] = 0;
Jac[3][11] = 0;
Jac[3][12] = 0;
Jac[3][13] = 0;
Jac[3][14] = -JTlp4_26_1;
Jac[3][15] = -C14;
Jac[3][16] = S14;
Jac[3][17] = -JTlp4_26_4;
Jac[3][18] = RLlp4_36;
Jac[3][19] = 0;
Jac[3][20] = 0;
Jac[3][21] = 0;
Jac[3][22] = 0;
Jac[3][23] = 0;
Jac[3][24] = 0;
Jac[3][25] = 0;
Jac[3][26] = 0;
Jac[3][27] = 0;
Jac[3][28] = 0;
Jac[3][29] = 0;
Jac[3][30] = 0;
Jac[3][31] = 0;
Jac[3][32] = 0;
Jac[3][33] = 0;
Jac[3][34] = 0;
Jac[3][35] = 0;
Jac[3][36] = 0;
Jac[3][37] = 0;
Jac[3][38] = 0;
Jac[3][39] = 0;
Jac[3][40] = 0;
Jac[3][41] = 0;
Jac[4][1] = 0;
Jac[4][2] = 0;
Jac[4][3] = 0;
Jac[4][4] = 0;
Jac[4][5] = 0;
Jac[4][6] = 0;
Jac[4][7] = 0;
Jac[4][8] = 0;
Jac[4][9] = 0;
Jac[4][10] = 0;
Jac[4][11] = 0;
Jac[4][12] = 0;
Jac[4][13] = 0;
Jac[4][14] = -JTlp4_36_1;
Jac[4][15] = -S14;
Jac[4][16] = -C14;
Jac[4][17] = -JTlp4_36_4;
Jac[4][18] = -RLlp4_26;
Jac[4][19] = 0;
Jac[4][20] = 0;
Jac[4][21] = 0;
Jac[4][22] = 0;
Jac[4][23] = 0;
Jac[4][24] = 0;
Jac[4][25] = 0;
Jac[4][26] = 0;
Jac[4][27] = 0;
Jac[4][28] = 0;
Jac[4][29] = 0;
Jac[4][30] = 0;
Jac[4][31] = 0;
Jac[4][32] = 0;
Jac[4][33] = 0;
Jac[4][34] = 0;
Jac[4][35] = 0;
Jac[4][36] = 0;
Jac[4][37] = 0;
Jac[4][38] = 0;
Jac[4][39] = 0;
Jac[4][40] = 0;
Jac[4][41] = 0;
Jac[5][1] = 0;
Jac[5][2] = 0;
Jac[5][3] = 0;
Jac[5][4] = 0;
Jac[5][5] = 0;
Jac[5][6] = 0;
Jac[5][7] = 0;
Jac[5][8] = 0;
Jac[5][9] = 0;
Jac[5][10] = 0;
Jac[5][11] = 0;
Jac[5][12] = 0;
Jac[5][13] = 0;
Jac[5][14] = 0;
Jac[5][15] = 0;
Jac[5][16] = 0;
Jac[5][17] = 0;
Jac[5][18] = 0;
Jac[5][19] = 0;
Jac[5][20] = 0;
Jac[5][21] = 0;
Jac[5][22] = 0;
Jac[5][23] = 0;
Jac[5][24] = 0;
Jac[5][25] = 0;
Jac[5][26] = 0;
Jac[5][27] = 0;
Jac[5][28] = 0;
Jac[5][29] = 0;
Jac[5][30] = 0;
Jac[5][31] = -JTlp6_24_1;
Jac[5][32] = -JTlp6_24_2;
Jac[5][33] = RLlp6_34;
Jac[5][34] = 0;
Jac[5][35] = 0;
Jac[5][36] = 0;
Jac[5][37] = 0;
Jac[5][38] = 0;
Jac[5][39] = 0;
Jac[5][40] = 0;
Jac[5][41] = 0;
Jac[6][1] = 0;
Jac[6][2] = 0;
Jac[6][3] = 0;
Jac[6][4] = 0;
Jac[6][5] = 0;
Jac[6][6] = 0;
Jac[6][7] = 0;
Jac[6][8] = 0;
Jac[6][9] = 0;
Jac[6][10] = 0;
Jac[6][11] = 0;
Jac[6][12] = 0;
Jac[6][13] = 0;
Jac[6][14] = 0;
Jac[6][15] = 0;
Jac[6][16] = 0;
Jac[6][17] = 0;
Jac[6][18] = 0;
Jac[6][19] = 0;
Jac[6][20] = 0;
Jac[6][21] = 0;
Jac[6][22] = 0;
Jac[6][23] = 0;
Jac[6][24] = 0;
Jac[6][25] = 0;
Jac[6][26] = 0;
Jac[6][27] = 0;
Jac[6][28] = 0;
Jac[6][29] = 0;
Jac[6][30] = 0;
Jac[6][31] = -JTlp6_34_1;
Jac[6][32] = -JTlp6_34_2;
Jac[6][33] = -RLlp6_24;
Jac[6][34] = 0;
Jac[6][35] = 0;
Jac[6][36] = 0;
Jac[6][37] = 0;
Jac[6][38] = 0;
Jac[6][39] = 0;
Jac[6][40] = 0;
Jac[6][41] = 0;
Jac[7][1] = 0;
Jac[7][2] = 0;
Jac[7][3] = 0;
Jac[7][4] = 0;
Jac[7][5] = 0;
Jac[7][6] = 0;
Jac[7][7] = 0;
Jac[7][8] = 0;
Jac[7][9] = 0;
Jac[7][10] = 0;
Jac[7][11] = 0;
Jac[7][12] = 0;
Jac[7][13] = 0;
Jac[7][14] = 0;
Jac[7][15] = 0;
Jac[7][16] = 0;
Jac[7][17] = 0;
Jac[7][18] = 0;
Jac[7][19] = 0;
Jac[7][20] = 0;
Jac[7][21] = 0;
Jac[7][22] = -JTlp8_26_1;
Jac[7][23] = -C22;
Jac[7][24] = S22;
Jac[7][25] = -JTlp8_26_4;
Jac[7][26] = RLlp8_36;
Jac[7][27] = 0;
Jac[7][28] = 0;
Jac[7][29] = 0;
Jac[7][30] = 0;
Jac[7][31] = 0;
Jac[7][32] = 0;
Jac[7][33] = 0;
Jac[7][34] = 0;
Jac[7][35] = 0;
Jac[7][36] = 0;
Jac[7][37] = 0;
Jac[7][38] = 0;
Jac[7][39] = 0;
Jac[7][40] = 0;
Jac[7][41] = 0;
Jac[8][1] = 0;
Jac[8][2] = 0;
Jac[8][3] = 0;
Jac[8][4] = 0;
Jac[8][5] = 0;
Jac[8][6] = 0;
Jac[8][7] = 0;
Jac[8][8] = 0;
Jac[8][9] = 0;
Jac[8][10] = 0;
Jac[8][11] = 0;
Jac[8][12] = 0;
Jac[8][13] = 0;
Jac[8][14] = 0;
Jac[8][15] = 0;
Jac[8][16] = 0;
Jac[8][17] = 0;
Jac[8][18] = 0;
Jac[8][19] = 0;
Jac[8][20] = 0;
Jac[8][21] = 0;
Jac[8][22] = -JTlp8_36_1;
Jac[8][23] = -S22;
Jac[8][24] = -C22;
Jac[8][25] = -JTlp8_36_4;
Jac[8][26] = -RLlp8_26;
Jac[8][27] = 0;
Jac[8][28] = 0;
Jac[8][29] = 0;
Jac[8][30] = 0;
Jac[8][31] = 0;
Jac[8][32] = 0;
Jac[8][33] = 0;
Jac[8][34] = 0;
Jac[8][35] = 0;
Jac[8][36] = 0;
Jac[8][37] = 0;
Jac[8][38] = 0;
Jac[8][39] = 0;
Jac[8][40] = 0;
Jac[8][41] = 0;
Jac[9][1] = 0;
Jac[9][2] = 0;
Jac[9][3] = 0;
Jac[9][4] = 0;
Jac[9][5] = 0;
Jac[9][6] = 0;
Jac[9][7] = Jac_13_7;
Jac[9][8] = Jac_13_8;
Jac[9][9] = 0;
Jac[9][10] = Jac_13_10;
Jac[9][11] = 0;
Jac[9][12] = 0;
Jac[9][13] = Plp21;
Jac[9][14] = 0;
Jac[9][15] = 0;
Jac[9][16] = 0;
Jac[9][17] = 0;
Jac[9][18] = 0;
Jac[9][19] = 0;
Jac[9][20] = 0;
Jac[9][21] = 0;
Jac[9][22] = 0;
Jac[9][23] = 0;
Jac[9][24] = 0;
Jac[9][25] = 0;
Jac[9][26] = 0;
Jac[9][27] = 0;
Jac[9][28] = 0;
Jac[9][29] = 0;
Jac[9][30] = 0;
Jac[9][31] = 0;
Jac[9][32] = 0;
Jac[9][33] = 0;
Jac[9][34] = 0;
Jac[9][35] = 0;
Jac[9][36] = 0;
Jac[9][37] = 0;
Jac[9][38] = 0;
Jac[9][39] = 0;
Jac[9][40] = 0;
Jac[9][41] = 0;
Jac[10][1] = 0;
Jac[10][2] = 0;
Jac[10][3] = 0;
Jac[10][4] = 0;
Jac[10][5] = 0;
Jac[10][6] = 0;
Jac[10][7] = 0;
Jac[10][8] = 0;
Jac[10][9] = 0;
Jac[10][10] = 0;
Jac[10][11] = 0;
Jac[10][12] = 0;
Jac[10][13] = Plp22;
Jac[10][14] = Jac_14_14;
Jac[10][15] = Jac_14_15;
Jac[10][16] = Jac_14_16;
Jac[10][17] = Jac_14_17;
Jac[10][18] = 0;
Jac[10][19] = Jac_14_19;
Jac[10][20] = 0;
Jac[10][21] = 0;
Jac[10][22] = 0;
Jac[10][23] = 0;
Jac[10][24] = 0;
Jac[10][25] = 0;
Jac[10][26] = 0;
Jac[10][27] = 0;
Jac[10][28] = 0;
Jac[10][29] = 0;
Jac[10][30] = 0;
Jac[10][31] = 0;
Jac[10][32] = 0;
Jac[10][33] = 0;
Jac[10][34] = 0;
Jac[10][35] = 0;
Jac[10][36] = 0;
Jac[10][37] = 0;
Jac[10][38] = 0;
Jac[10][39] = 0;
Jac[10][40] = 0;
Jac[10][41] = 0;
Jac[11][1] = 0;
Jac[11][2] = 0;
Jac[11][3] = 0;
Jac[11][4] = 0;
Jac[11][5] = 0;
Jac[11][6] = 0;
Jac[11][7] = 0;
Jac[11][8] = 0;
Jac[11][9] = 0;
Jac[11][10] = 0;
Jac[11][11] = 0;
Jac[11][12] = 0;
Jac[11][13] = 0;
Jac[11][14] = 0;
Jac[11][15] = 0;
Jac[11][16] = 0;
Jac[11][17] = 0;
Jac[11][18] = 0;
Jac[11][19] = 0;
Jac[11][20] = 0;
Jac[11][21] = 0;
Jac[11][22] = 0;
Jac[11][23] = 0;
Jac[11][24] = 0;
Jac[11][25] = 0;
Jac[11][26] = 0;
Jac[11][27] = 0;
Jac[11][28] = 0;
Jac[11][29] = 0;
Jac[11][30] = Plp23;
Jac[11][31] = Jac_15_31;
Jac[11][32] = Jac_15_32;
Jac[11][33] = 0;
Jac[11][34] = Jac_15_34;
Jac[11][35] = 0;
Jac[11][36] = 0;
Jac[11][37] = 0;
Jac[11][38] = 0;
Jac[11][39] = 0;
Jac[11][40] = 0;
Jac[11][41] = 0;
Jac[12][1] = 0;
Jac[12][2] = 0;
Jac[12][3] = 0;
Jac[12][4] = 0;
Jac[12][5] = 0;
Jac[12][6] = 0;
Jac[12][7] = 0;
Jac[12][8] = 0;
Jac[12][9] = 0;
Jac[12][10] = 0;
Jac[12][11] = 0;
Jac[12][12] = 0;
Jac[12][13] = 0;
Jac[12][14] = 0;
Jac[12][15] = 0;
Jac[12][16] = 0;
Jac[12][17] = 0;
Jac[12][18] = 0;
Jac[12][19] = 0;
Jac[12][20] = 0;
Jac[12][21] = 0;
Jac[12][22] = Jac_16_22;
Jac[12][23] = Jac_16_23;
Jac[12][24] = Jac_16_24;
Jac[12][25] = Jac_16_25;
Jac[12][26] = 0;
Jac[12][27] = Jac_16_27;
Jac[12][28] = 0;
Jac[12][29] = 0;
Jac[12][30] = Plp24;
Jac[12][31] = 0;
Jac[12][32] = 0;
Jac[12][33] = 0;
Jac[12][34] = 0;
Jac[12][35] = 0;
Jac[12][36] = 0;
Jac[12][37] = 0;
Jac[12][38] = 0;
Jac[12][39] = 0;
Jac[12][40] = 0;
Jac[12][41] = 0;

// Number of continuation lines = 0

}
