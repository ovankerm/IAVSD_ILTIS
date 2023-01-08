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
//	==> Generation Date: Sun Jan  8 23:07:25 2023
//
//	==> Project name: Jeep
//
//	==> Number of joints: 41
//
//	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
//
//	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
//

#include <math.h> 

#include "mbs_data.h"

void mbs_cons_jdqd(double *Jdqd,
MbsData *s, double tsim)
{
#include "mbs_cons_jdqd_Jeep.h"

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

 
// Constraints Quadratic Terms

ROjdqd2_52 = C7*C8-S7*S8;
ROjdqd2_62 = C7*S8+S7*C8;
ROjdqd2_82 = -C7*S8-S7*C8;
ROjdqd2_92 = C7*C8-S7*S8;
ROjdqd2_53 = ROjdqd2_52*C9+ROjdqd2_82*S9;
ROjdqd2_63 = ROjdqd2_62*C9+ROjdqd2_92*S9;
RLjdqd2_22 = dpt[2][18]*C7;
RLjdqd2_32 = dpt[2][18]*S7;
OMjdqd2_12 = qd[7]+qd[8];
ORjdqd2_22 = -RLjdqd2_32*qd[7];
ORjdqd2_32 = RLjdqd2_22*qd[7];
Apqpjdqd2_22 = -ORjdqd2_32*qd[7];
Apqpjdqd2_32 = ORjdqd2_22*qd[7];
RLjdqd2_23 = ROjdqd2_82*dpt[3][19];
RLjdqd2_33 = ROjdqd2_92*dpt[3][19];
OMjdqd2_13 = OMjdqd2_12+qd[9];
ORjdqd2_23 = -OMjdqd2_12*RLjdqd2_33;
ORjdqd2_33 = OMjdqd2_12*RLjdqd2_23;
Apqpjdqd2_23 = Apqpjdqd2_22-OMjdqd2_12*ORjdqd2_33;
Apqpjdqd2_33 = Apqpjdqd2_32+OMjdqd2_12*ORjdqd2_23;
RLjdqd2_24 = ROjdqd2_53*dpt[2][21];
RLjdqd2_34 = ROjdqd2_63*dpt[2][21];
ORjdqd2_24 = -OMjdqd2_13*RLjdqd2_34;
ORjdqd2_34 = OMjdqd2_13*RLjdqd2_24;
Apqpjdqd2_24 = Apqpjdqd2_23-OMjdqd2_13*ORjdqd2_34;
Apqpjdqd2_34 = Apqpjdqd2_33+OMjdqd2_13*ORjdqd2_24;
ROjdqd4_54 = C14*C17-S14*S17;
ROjdqd4_64 = C14*S17+S14*C17;
ROjdqd4_84 = -C14*S17-S14*C17;
ROjdqd4_94 = C14*C17-S14*S17;
ROjdqd4_55 = ROjdqd4_54*C18+ROjdqd4_84*S18;
ROjdqd4_65 = ROjdqd4_64*C18+ROjdqd4_94*S18;
RLjdqd4_22 = q[15]*C14;
RLjdqd4_32 = q[15]*S14;
ORjdqd4_22 = -RLjdqd4_32*qd[14];
ORjdqd4_32 = RLjdqd4_22*qd[14];
Apqpjdqd4_22 = -ORjdqd4_32*qd[14]-(2.0)*qd[14]*qd[15]*S14;
Apqpjdqd4_32 = ORjdqd4_22*qd[14]+(2.0)*qd[14]*qd[15]*C14;
RLjdqd4_23 = -q[16]*S14;
RLjdqd4_33 = q[16]*C14;
ORjdqd4_23 = -RLjdqd4_33*qd[14];
ORjdqd4_33 = RLjdqd4_23*qd[14];
Apqpjdqd4_23 = Apqpjdqd4_22-ORjdqd4_33*qd[14]-(2.0)*qd[14]*qd[16]*C14;
Apqpjdqd4_33 = Apqpjdqd4_32+ORjdqd4_23*qd[14]-(2.0)*qd[14]*qd[16]*S14;
RLjdqd4_24 = dpt[2][29]*C14;
RLjdqd4_34 = dpt[2][29]*S14;
OMjdqd4_14 = qd[14]+qd[17];
ORjdqd4_24 = -RLjdqd4_34*qd[14];
ORjdqd4_34 = RLjdqd4_24*qd[14];
Apqpjdqd4_24 = Apqpjdqd4_23-ORjdqd4_34*qd[14];
Apqpjdqd4_34 = Apqpjdqd4_33+ORjdqd4_24*qd[14];
RLjdqd4_25 = ROjdqd4_84*dpt[3][30];
RLjdqd4_35 = ROjdqd4_94*dpt[3][30];
OMjdqd4_15 = OMjdqd4_14+qd[18];
ORjdqd4_25 = -OMjdqd4_14*RLjdqd4_35;
ORjdqd4_35 = OMjdqd4_14*RLjdqd4_25;
Apqpjdqd4_25 = Apqpjdqd4_24-OMjdqd4_14*ORjdqd4_35;
Apqpjdqd4_35 = Apqpjdqd4_34+OMjdqd4_14*ORjdqd4_25;
RLjdqd4_26 = ROjdqd4_55*dpt[2][32];
RLjdqd4_36 = ROjdqd4_65*dpt[2][32];
ORjdqd4_26 = -OMjdqd4_15*RLjdqd4_36;
ORjdqd4_36 = OMjdqd4_15*RLjdqd4_26;
Apqpjdqd4_26 = Apqpjdqd4_25-OMjdqd4_15*ORjdqd4_36;
Apqpjdqd4_36 = Apqpjdqd4_35+OMjdqd4_15*ORjdqd4_26;
ROjdqd6_52 = C31*C32-S31*S32;
ROjdqd6_62 = C31*S32+S31*C32;
ROjdqd6_82 = -C31*S32-S31*C32;
ROjdqd6_92 = C31*C32-S31*S32;
ROjdqd6_53 = ROjdqd6_52*C33+ROjdqd6_82*S33;
ROjdqd6_63 = ROjdqd6_62*C33+ROjdqd6_92*S33;
RLjdqd6_22 = dpt[2][49]*C31;
RLjdqd6_32 = dpt[2][49]*S31;
OMjdqd6_12 = qd[31]+qd[32];
ORjdqd6_22 = -RLjdqd6_32*qd[31];
ORjdqd6_32 = RLjdqd6_22*qd[31];
Apqpjdqd6_22 = -ORjdqd6_32*qd[31];
Apqpjdqd6_32 = ORjdqd6_22*qd[31];
RLjdqd6_23 = ROjdqd6_82*dpt[3][50];
RLjdqd6_33 = ROjdqd6_92*dpt[3][50];
OMjdqd6_13 = OMjdqd6_12+qd[33];
ORjdqd6_23 = -OMjdqd6_12*RLjdqd6_33;
ORjdqd6_33 = OMjdqd6_12*RLjdqd6_23;
Apqpjdqd6_23 = Apqpjdqd6_22-OMjdqd6_12*ORjdqd6_33;
Apqpjdqd6_33 = Apqpjdqd6_32+OMjdqd6_12*ORjdqd6_23;
RLjdqd6_24 = ROjdqd6_53*dpt[2][52];
RLjdqd6_34 = ROjdqd6_63*dpt[2][52];
ORjdqd6_24 = -OMjdqd6_13*RLjdqd6_34;
ORjdqd6_34 = OMjdqd6_13*RLjdqd6_24;
Apqpjdqd6_24 = Apqpjdqd6_23-OMjdqd6_13*ORjdqd6_34;
Apqpjdqd6_34 = Apqpjdqd6_33+OMjdqd6_13*ORjdqd6_24;
ROjdqd8_54 = C22*C25-S22*S25;
ROjdqd8_64 = C22*S25+S22*C25;
ROjdqd8_84 = -C22*S25-S22*C25;
ROjdqd8_94 = C22*C25-S22*S25;
ROjdqd8_55 = ROjdqd8_54*C26+ROjdqd8_84*S26;
ROjdqd8_65 = ROjdqd8_64*C26+ROjdqd8_94*S26;
RLjdqd8_22 = q[23]*C22;
RLjdqd8_32 = q[23]*S22;
ORjdqd8_22 = -RLjdqd8_32*qd[22];
ORjdqd8_32 = RLjdqd8_22*qd[22];
Apqpjdqd8_22 = -ORjdqd8_32*qd[22]-(2.0)*qd[22]*qd[23]*S22;
Apqpjdqd8_32 = ORjdqd8_22*qd[22]+(2.0)*qd[22]*qd[23]*C22;
RLjdqd8_23 = -q[24]*S22;
RLjdqd8_33 = q[24]*C22;
ORjdqd8_23 = -RLjdqd8_33*qd[22];
ORjdqd8_33 = RLjdqd8_23*qd[22];
Apqpjdqd8_23 = Apqpjdqd8_22-ORjdqd8_33*qd[22]-(2.0)*qd[22]*qd[24]*C22;
Apqpjdqd8_33 = Apqpjdqd8_32+ORjdqd8_23*qd[22]-(2.0)*qd[22]*qd[24]*S22;
RLjdqd8_24 = dpt[2][38]*C22;
RLjdqd8_34 = dpt[2][38]*S22;
OMjdqd8_14 = qd[22]+qd[25];
ORjdqd8_24 = -RLjdqd8_34*qd[22];
ORjdqd8_34 = RLjdqd8_24*qd[22];
Apqpjdqd8_24 = Apqpjdqd8_23-ORjdqd8_34*qd[22];
Apqpjdqd8_34 = Apqpjdqd8_33+ORjdqd8_24*qd[22];
RLjdqd8_25 = ROjdqd8_84*dpt[3][39];
RLjdqd8_35 = ROjdqd8_94*dpt[3][39];
OMjdqd8_15 = OMjdqd8_14+qd[26];
ORjdqd8_25 = -OMjdqd8_14*RLjdqd8_35;
ORjdqd8_35 = OMjdqd8_14*RLjdqd8_25;
Apqpjdqd8_25 = Apqpjdqd8_24-OMjdqd8_14*ORjdqd8_35;
Apqpjdqd8_35 = Apqpjdqd8_34+OMjdqd8_14*ORjdqd8_25;
RLjdqd8_26 = ROjdqd8_55*dpt[2][41];
RLjdqd8_36 = ROjdqd8_65*dpt[2][41];
ORjdqd8_26 = -OMjdqd8_15*RLjdqd8_36;
ORjdqd8_36 = OMjdqd8_15*RLjdqd8_26;
Apqpjdqd8_26 = Apqpjdqd8_25-OMjdqd8_15*ORjdqd8_36;
Apqpjdqd8_36 = Apqpjdqd8_35+OMjdqd8_15*ORjdqd8_26;
POjdqd9_22 = q[13]+dpt[2][27];
ROjdqd10_52 = C7*C8-S7*S8;
ROjdqd10_62 = C7*S8+S7*C8;
ROjdqd10_82 = -C7*S8-S7*C8;
ROjdqd10_92 = C7*C8-S7*S8;
ROjdqd10_23 = ROjdqd10_52*S10;
ROjdqd10_33 = ROjdqd10_62*S10;
ROjdqd10_53 = ROjdqd10_52*C10;
ROjdqd10_63 = ROjdqd10_62*C10;
RLjdqd10_22 = dpt[2][18]*C7;
RLjdqd10_32 = dpt[2][18]*S7;
POjdqd10_22 = RLjdqd10_22+dpt[2][2];
POjdqd10_32 = RLjdqd10_32+dpt[3][2];
OMjdqd10_12 = qd[7]+qd[8];
ORjdqd10_22 = -RLjdqd10_32*qd[7];
ORjdqd10_32 = RLjdqd10_22*qd[7];
Apqpjdqd10_22 = -ORjdqd10_32*qd[7];
Apqpjdqd10_32 = ORjdqd10_22*qd[7];
RLjdqd10_23 = ROjdqd10_82*dpt[3][20];
RLjdqd10_33 = ROjdqd10_92*dpt[3][20];
POjdqd10_23 = POjdqd10_22+RLjdqd10_23;
POjdqd10_33 = POjdqd10_32+RLjdqd10_33;
OMjdqd10_23 = ROjdqd10_82*qd[10];
OMjdqd10_33 = ROjdqd10_92*qd[10];
ORjdqd10_23 = -OMjdqd10_12*RLjdqd10_33;
ORjdqd10_33 = OMjdqd10_12*RLjdqd10_23;
VIjdqd10_23 = ORjdqd10_22+ORjdqd10_23;
VIjdqd10_33 = ORjdqd10_32+ORjdqd10_33;
Ompqpjdqd10_23 = -OMjdqd10_12*ROjdqd10_92*qd[10];
Ompqpjdqd10_33 = OMjdqd10_12*ROjdqd10_82*qd[10];
Apqpjdqd10_23 = Apqpjdqd10_22-OMjdqd10_12*ORjdqd10_33;
Apqpjdqd10_33 = Apqpjdqd10_32+OMjdqd10_12*ORjdqd10_23;
RLjdqd10_14 = dpt[1][24]*C10-dpt[2][24]*S10;
RLjdqd10_24 = ROjdqd10_23*dpt[1][24]+ROjdqd10_53*dpt[2][24]+ROjdqd10_82*dpt[3][24];
RLjdqd10_34 = ROjdqd10_33*dpt[1][24]+ROjdqd10_63*dpt[2][24]+ROjdqd10_92*dpt[3][24];
POjdqd10_14 = RLjdqd10_14+dpt[1][2];
POjdqd10_24 = POjdqd10_23+RLjdqd10_24;
POjdqd10_34 = POjdqd10_33+RLjdqd10_34;
ORjdqd10_14 = OMjdqd10_23*RLjdqd10_34-OMjdqd10_33*RLjdqd10_24;
ORjdqd10_24 = -OMjdqd10_12*RLjdqd10_34+OMjdqd10_33*RLjdqd10_14;
ORjdqd10_34 = OMjdqd10_12*RLjdqd10_24-OMjdqd10_23*RLjdqd10_14;
VIjdqd10_24 = ORjdqd10_24+VIjdqd10_23;
VIjdqd10_34 = ORjdqd10_34+VIjdqd10_33;
Apqpjdqd10_14 = OMjdqd10_23*ORjdqd10_34-OMjdqd10_33*ORjdqd10_24+Ompqpjdqd10_23*RLjdqd10_34-Ompqpjdqd10_33*RLjdqd10_24;
Apqpjdqd10_24 = Apqpjdqd10_23-OMjdqd10_12*ORjdqd10_34+OMjdqd10_33*ORjdqd10_14+Ompqpjdqd10_33*RLjdqd10_14;
Apqpjdqd10_34 = Apqpjdqd10_33+OMjdqd10_12*ORjdqd10_24-OMjdqd10_23*ORjdqd10_14-Ompqpjdqd10_23*RLjdqd10_14;
jdqd13 = -Apqpjdqd10_14*(-POjdqd10_14+dpt[1][3])-Apqpjdqd10_24*(-POjdqd10_24+POjdqd9_22)-Apqpjdqd10_34*(-POjdqd10_34+
 dpt[3][3])+ORjdqd10_14*ORjdqd10_14+VIjdqd10_34*VIjdqd10_34+(-VIjdqd10_24+qd[13])*(-VIjdqd10_24+qd[13]);
POjdqd11_22 = q[13]+dpt[2][28];
ROjdqd12_54 = C14*C17-S14*S17;
ROjdqd12_64 = C14*S17+S14*C17;
ROjdqd12_84 = -C14*S17-S14*C17;
ROjdqd12_94 = C14*C17-S14*S17;
ROjdqd12_25 = ROjdqd12_54*S19;
ROjdqd12_35 = ROjdqd12_64*S19;
ROjdqd12_55 = ROjdqd12_54*C19;
ROjdqd12_65 = ROjdqd12_64*C19;
RLjdqd12_22 = q[15]*C14;
RLjdqd12_32 = q[15]*S14;
POjdqd12_22 = RLjdqd12_22+dpt[2][4];
POjdqd12_32 = RLjdqd12_32+dpt[3][4];
ORjdqd12_22 = -RLjdqd12_32*qd[14];
ORjdqd12_32 = RLjdqd12_22*qd[14];
VIjdqd12_22 = ORjdqd12_22+qd[15]*C14;
VIjdqd12_32 = ORjdqd12_32+qd[15]*S14;
Apqpjdqd12_22 = -ORjdqd12_32*qd[14]-(2.0)*qd[14]*qd[15]*S14;
Apqpjdqd12_32 = ORjdqd12_22*qd[14]+(2.0)*qd[14]*qd[15]*C14;
RLjdqd12_23 = -q[16]*S14;
RLjdqd12_33 = q[16]*C14;
POjdqd12_23 = POjdqd12_22+RLjdqd12_23;
POjdqd12_33 = POjdqd12_32+RLjdqd12_33;
ORjdqd12_23 = -RLjdqd12_33*qd[14];
ORjdqd12_33 = RLjdqd12_23*qd[14];
VIjdqd12_23 = ORjdqd12_23+VIjdqd12_22-qd[16]*S14;
VIjdqd12_33 = ORjdqd12_33+VIjdqd12_32+qd[16]*C14;
Apqpjdqd12_23 = Apqpjdqd12_22-ORjdqd12_33*qd[14]-(2.0)*qd[14]*qd[16]*C14;
Apqpjdqd12_33 = Apqpjdqd12_32+ORjdqd12_23*qd[14]-(2.0)*qd[14]*qd[16]*S14;
RLjdqd12_24 = dpt[2][29]*C14;
RLjdqd12_34 = dpt[2][29]*S14;
POjdqd12_24 = POjdqd12_23+RLjdqd12_24;
POjdqd12_34 = POjdqd12_33+RLjdqd12_34;
OMjdqd12_14 = qd[14]+qd[17];
ORjdqd12_24 = -RLjdqd12_34*qd[14];
ORjdqd12_34 = RLjdqd12_24*qd[14];
VIjdqd12_24 = ORjdqd12_24+VIjdqd12_23;
VIjdqd12_34 = ORjdqd12_34+VIjdqd12_33;
Apqpjdqd12_24 = Apqpjdqd12_23-ORjdqd12_34*qd[14];
Apqpjdqd12_34 = Apqpjdqd12_33+ORjdqd12_24*qd[14];
RLjdqd12_25 = ROjdqd12_84*dpt[3][31];
RLjdqd12_35 = ROjdqd12_94*dpt[3][31];
POjdqd12_25 = POjdqd12_24+RLjdqd12_25;
POjdqd12_35 = POjdqd12_34+RLjdqd12_35;
OMjdqd12_25 = ROjdqd12_84*qd[19];
OMjdqd12_35 = ROjdqd12_94*qd[19];
ORjdqd12_25 = -OMjdqd12_14*RLjdqd12_35;
ORjdqd12_35 = OMjdqd12_14*RLjdqd12_25;
VIjdqd12_25 = ORjdqd12_25+VIjdqd12_24;
VIjdqd12_35 = ORjdqd12_35+VIjdqd12_34;
Ompqpjdqd12_25 = -OMjdqd12_14*ROjdqd12_94*qd[19];
Ompqpjdqd12_35 = OMjdqd12_14*ROjdqd12_84*qd[19];
Apqpjdqd12_25 = Apqpjdqd12_24-OMjdqd12_14*ORjdqd12_35;
Apqpjdqd12_35 = Apqpjdqd12_34+OMjdqd12_14*ORjdqd12_25;
RLjdqd12_16 = dpt[1][35]*C19-dpt[2][35]*S19;
RLjdqd12_26 = ROjdqd12_25*dpt[1][35]+ROjdqd12_55*dpt[2][35]+ROjdqd12_84*dpt[3][35];
RLjdqd12_36 = ROjdqd12_35*dpt[1][35]+ROjdqd12_65*dpt[2][35]+ROjdqd12_94*dpt[3][35];
POjdqd12_16 = RLjdqd12_16+dpt[1][4];
POjdqd12_26 = POjdqd12_25+RLjdqd12_26;
POjdqd12_36 = POjdqd12_35+RLjdqd12_36;
ORjdqd12_16 = OMjdqd12_25*RLjdqd12_36-OMjdqd12_35*RLjdqd12_26;
ORjdqd12_26 = -OMjdqd12_14*RLjdqd12_36+OMjdqd12_35*RLjdqd12_16;
ORjdqd12_36 = OMjdqd12_14*RLjdqd12_26-OMjdqd12_25*RLjdqd12_16;
VIjdqd12_26 = ORjdqd12_26+VIjdqd12_25;
VIjdqd12_36 = ORjdqd12_36+VIjdqd12_35;
Apqpjdqd12_16 = OMjdqd12_25*ORjdqd12_36-OMjdqd12_35*ORjdqd12_26+Ompqpjdqd12_25*RLjdqd12_36-Ompqpjdqd12_35*RLjdqd12_26;
Apqpjdqd12_26 = Apqpjdqd12_25-OMjdqd12_14*ORjdqd12_36+OMjdqd12_35*ORjdqd12_16+Ompqpjdqd12_35*RLjdqd12_16;
Apqpjdqd12_36 = Apqpjdqd12_35+OMjdqd12_14*ORjdqd12_26-OMjdqd12_25*ORjdqd12_16-Ompqpjdqd12_25*RLjdqd12_16;
jdqd14 = -Apqpjdqd12_16*(-POjdqd12_16+dpt[1][3])-Apqpjdqd12_26*(POjdqd11_22-POjdqd12_26)-Apqpjdqd12_36*(-POjdqd12_36+
 dpt[3][3])+ORjdqd12_16*ORjdqd12_16+VIjdqd12_36*VIjdqd12_36+(-VIjdqd12_26+qd[13])*(-VIjdqd12_26+qd[13]);
POjdqd13_22 = q[30]+dpt[2][47];
ROjdqd14_52 = C31*C32-S31*S32;
ROjdqd14_62 = C31*S32+S31*C32;
ROjdqd14_82 = -C31*S32-S31*C32;
ROjdqd14_92 = C31*C32-S31*S32;
ROjdqd14_23 = ROjdqd14_52*S34;
ROjdqd14_33 = ROjdqd14_62*S34;
ROjdqd14_53 = ROjdqd14_52*C34;
ROjdqd14_63 = ROjdqd14_62*C34;
RLjdqd14_22 = dpt[2][49]*C31;
RLjdqd14_32 = dpt[2][49]*S31;
POjdqd14_22 = RLjdqd14_22+dpt[2][10];
POjdqd14_32 = RLjdqd14_32+dpt[3][10];
OMjdqd14_12 = qd[31]+qd[32];
ORjdqd14_22 = -RLjdqd14_32*qd[31];
ORjdqd14_32 = RLjdqd14_22*qd[31];
Apqpjdqd14_22 = -ORjdqd14_32*qd[31];
Apqpjdqd14_32 = ORjdqd14_22*qd[31];
RLjdqd14_23 = ROjdqd14_82*dpt[3][51];
RLjdqd14_33 = ROjdqd14_92*dpt[3][51];
POjdqd14_23 = POjdqd14_22+RLjdqd14_23;
POjdqd14_33 = POjdqd14_32+RLjdqd14_33;
OMjdqd14_23 = ROjdqd14_82*qd[34];
OMjdqd14_33 = ROjdqd14_92*qd[34];
ORjdqd14_23 = -OMjdqd14_12*RLjdqd14_33;
ORjdqd14_33 = OMjdqd14_12*RLjdqd14_23;
VIjdqd14_23 = ORjdqd14_22+ORjdqd14_23;
VIjdqd14_33 = ORjdqd14_32+ORjdqd14_33;
Ompqpjdqd14_23 = -OMjdqd14_12*ROjdqd14_92*qd[34];
Ompqpjdqd14_33 = OMjdqd14_12*ROjdqd14_82*qd[34];
Apqpjdqd14_23 = Apqpjdqd14_22-OMjdqd14_12*ORjdqd14_33;
Apqpjdqd14_33 = Apqpjdqd14_32+OMjdqd14_12*ORjdqd14_23;
RLjdqd14_14 = dpt[1][55]*C34-dpt[2][55]*S34;
RLjdqd14_24 = ROjdqd14_23*dpt[1][55]+ROjdqd14_53*dpt[2][55]+ROjdqd14_82*dpt[3][55];
RLjdqd14_34 = ROjdqd14_33*dpt[1][55]+ROjdqd14_63*dpt[2][55]+ROjdqd14_92*dpt[3][55];
POjdqd14_14 = RLjdqd14_14+dpt[1][10];
POjdqd14_24 = POjdqd14_23+RLjdqd14_24;
POjdqd14_34 = POjdqd14_33+RLjdqd14_34;
ORjdqd14_14 = OMjdqd14_23*RLjdqd14_34-OMjdqd14_33*RLjdqd14_24;
ORjdqd14_24 = -OMjdqd14_12*RLjdqd14_34+OMjdqd14_33*RLjdqd14_14;
ORjdqd14_34 = OMjdqd14_12*RLjdqd14_24-OMjdqd14_23*RLjdqd14_14;
VIjdqd14_24 = ORjdqd14_24+VIjdqd14_23;
VIjdqd14_34 = ORjdqd14_34+VIjdqd14_33;
Apqpjdqd14_14 = OMjdqd14_23*ORjdqd14_34-OMjdqd14_33*ORjdqd14_24+Ompqpjdqd14_23*RLjdqd14_34-Ompqpjdqd14_33*RLjdqd14_24;
Apqpjdqd14_24 = Apqpjdqd14_23-OMjdqd14_12*ORjdqd14_34+OMjdqd14_33*ORjdqd14_14+Ompqpjdqd14_33*RLjdqd14_14;
Apqpjdqd14_34 = Apqpjdqd14_33+OMjdqd14_12*ORjdqd14_24-OMjdqd14_23*ORjdqd14_14-Ompqpjdqd14_23*RLjdqd14_14;
jdqd15 = -Apqpjdqd14_14*(-POjdqd14_14+dpt[1][8])-Apqpjdqd14_24*(POjdqd13_22-POjdqd14_24)-Apqpjdqd14_34*(-POjdqd14_34+
 dpt[3][8])+ORjdqd14_14*ORjdqd14_14+VIjdqd14_34*VIjdqd14_34+(-VIjdqd14_24+qd[30])*(-VIjdqd14_24+qd[30]);
POjdqd15_22 = q[30]+dpt[2][48];
ROjdqd16_54 = C22*C25-S22*S25;
ROjdqd16_64 = C22*S25+S22*C25;
ROjdqd16_84 = -C22*S25-S22*C25;
ROjdqd16_94 = C22*C25-S22*S25;
ROjdqd16_25 = ROjdqd16_54*S27;
ROjdqd16_35 = ROjdqd16_64*S27;
ROjdqd16_55 = ROjdqd16_54*C27;
ROjdqd16_65 = ROjdqd16_64*C27;
RLjdqd16_22 = q[23]*C22;
RLjdqd16_32 = q[23]*S22;
POjdqd16_22 = RLjdqd16_22+dpt[2][7];
POjdqd16_32 = RLjdqd16_32+dpt[3][7];
ORjdqd16_22 = -RLjdqd16_32*qd[22];
ORjdqd16_32 = RLjdqd16_22*qd[22];
VIjdqd16_22 = ORjdqd16_22+qd[23]*C22;
VIjdqd16_32 = ORjdqd16_32+qd[23]*S22;
Apqpjdqd16_22 = -ORjdqd16_32*qd[22]-(2.0)*qd[22]*qd[23]*S22;
Apqpjdqd16_32 = ORjdqd16_22*qd[22]+(2.0)*qd[22]*qd[23]*C22;
RLjdqd16_23 = -q[24]*S22;
RLjdqd16_33 = q[24]*C22;
POjdqd16_23 = POjdqd16_22+RLjdqd16_23;
POjdqd16_33 = POjdqd16_32+RLjdqd16_33;
ORjdqd16_23 = -RLjdqd16_33*qd[22];
ORjdqd16_33 = RLjdqd16_23*qd[22];
VIjdqd16_23 = ORjdqd16_23+VIjdqd16_22-qd[24]*S22;
VIjdqd16_33 = ORjdqd16_33+VIjdqd16_32+qd[24]*C22;
Apqpjdqd16_23 = Apqpjdqd16_22-ORjdqd16_33*qd[22]-(2.0)*qd[22]*qd[24]*C22;
Apqpjdqd16_33 = Apqpjdqd16_32+ORjdqd16_23*qd[22]-(2.0)*qd[22]*qd[24]*S22;
RLjdqd16_24 = dpt[2][38]*C22;
RLjdqd16_34 = dpt[2][38]*S22;
POjdqd16_24 = POjdqd16_23+RLjdqd16_24;
POjdqd16_34 = POjdqd16_33+RLjdqd16_34;
OMjdqd16_14 = qd[22]+qd[25];
ORjdqd16_24 = -RLjdqd16_34*qd[22];
ORjdqd16_34 = RLjdqd16_24*qd[22];
VIjdqd16_24 = ORjdqd16_24+VIjdqd16_23;
VIjdqd16_34 = ORjdqd16_34+VIjdqd16_33;
Apqpjdqd16_24 = Apqpjdqd16_23-ORjdqd16_34*qd[22];
Apqpjdqd16_34 = Apqpjdqd16_33+ORjdqd16_24*qd[22];
RLjdqd16_25 = ROjdqd16_84*dpt[3][40];
RLjdqd16_35 = ROjdqd16_94*dpt[3][40];
POjdqd16_25 = POjdqd16_24+RLjdqd16_25;
POjdqd16_35 = POjdqd16_34+RLjdqd16_35;
OMjdqd16_25 = ROjdqd16_84*qd[27];
OMjdqd16_35 = ROjdqd16_94*qd[27];
ORjdqd16_25 = -OMjdqd16_14*RLjdqd16_35;
ORjdqd16_35 = OMjdqd16_14*RLjdqd16_25;
VIjdqd16_25 = ORjdqd16_25+VIjdqd16_24;
VIjdqd16_35 = ORjdqd16_35+VIjdqd16_34;
Ompqpjdqd16_25 = -OMjdqd16_14*ROjdqd16_94*qd[27];
Ompqpjdqd16_35 = OMjdqd16_14*ROjdqd16_84*qd[27];
Apqpjdqd16_25 = Apqpjdqd16_24-OMjdqd16_14*ORjdqd16_35;
Apqpjdqd16_35 = Apqpjdqd16_34+OMjdqd16_14*ORjdqd16_25;
RLjdqd16_16 = dpt[1][44]*C27-dpt[2][44]*S27;
RLjdqd16_26 = ROjdqd16_25*dpt[1][44]+ROjdqd16_55*dpt[2][44]+ROjdqd16_84*dpt[3][44];
RLjdqd16_36 = ROjdqd16_35*dpt[1][44]+ROjdqd16_65*dpt[2][44]+ROjdqd16_94*dpt[3][44];
POjdqd16_16 = RLjdqd16_16+dpt[1][7];
POjdqd16_26 = POjdqd16_25+RLjdqd16_26;
POjdqd16_36 = POjdqd16_35+RLjdqd16_36;
ORjdqd16_16 = OMjdqd16_25*RLjdqd16_36-OMjdqd16_35*RLjdqd16_26;
ORjdqd16_26 = -OMjdqd16_14*RLjdqd16_36+OMjdqd16_35*RLjdqd16_16;
ORjdqd16_36 = OMjdqd16_14*RLjdqd16_26-OMjdqd16_25*RLjdqd16_16;
VIjdqd16_26 = ORjdqd16_26+VIjdqd16_25;
VIjdqd16_36 = ORjdqd16_36+VIjdqd16_35;
Apqpjdqd16_16 = OMjdqd16_25*ORjdqd16_36-OMjdqd16_35*ORjdqd16_26+Ompqpjdqd16_25*RLjdqd16_36-Ompqpjdqd16_35*RLjdqd16_26;
Apqpjdqd16_26 = Apqpjdqd16_25-OMjdqd16_14*ORjdqd16_36+OMjdqd16_35*ORjdqd16_16+Ompqpjdqd16_35*RLjdqd16_16;
Apqpjdqd16_36 = Apqpjdqd16_35+OMjdqd16_14*ORjdqd16_26-OMjdqd16_25*ORjdqd16_16-Ompqpjdqd16_25*RLjdqd16_16;
jdqd16 = -Apqpjdqd16_16*(-POjdqd16_16+dpt[1][8])-Apqpjdqd16_26*(POjdqd15_22-POjdqd16_26)-Apqpjdqd16_36*(-POjdqd16_36+
 dpt[3][8])+ORjdqd16_16*ORjdqd16_16+VIjdqd16_36*VIjdqd16_36+(-VIjdqd16_26+qd[30])*(-VIjdqd16_26+qd[30]);
Jdqd[1] = -Apqpjdqd2_24;
Jdqd[2] = -Apqpjdqd2_34;
Jdqd[3] = -Apqpjdqd4_26;
Jdqd[4] = -Apqpjdqd4_36;
Jdqd[5] = -Apqpjdqd6_24;
Jdqd[6] = -Apqpjdqd6_34;
Jdqd[7] = -Apqpjdqd8_26;
Jdqd[8] = -Apqpjdqd8_36;
Jdqd[9] = jdqd13;
Jdqd[10] = jdqd14;
Jdqd[11] = jdqd15;
Jdqd[12] = jdqd16;

// Number of continuation lines = 1

}
