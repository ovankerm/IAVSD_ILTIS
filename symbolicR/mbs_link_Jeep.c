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
<<<<<<< Updated upstream
//	==> Generation Date: Thu Jan  5 23:49:40 2023
=======
//	==> Generation Date: Thu Jan  5 23:06:39 2023
>>>>>>> Stashed changes
//
//	==> Project name: Jeep
//
//	==> Number of joints: 40
//
//	==> Function: F7 - Link Forces (1D)
//
//	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
//

#include <math.h> 

#include "mbs_data.h"

#include "mbs_project_interface.h"

// #include "mbs_link_xml_Jeep.h"  // future development
// #include "mbs_link_hard_Jeep.h" // future development
 
void mbs_link(double **frc, double **trq, double *Flink, double *Z, double *Zd,
MbsData *s, double tsim)
{
#include "mbs_link_Jeep.h"

double *q, *qd;
double **l, **dpt;

frc = s->frc;
trq = s->trq;
Z = s->Z;
Zd = s->Zd;

q = s->q;
qd = s->qd;

dpt = s->dpt;
l  = s->l;
 
// Trigonometric functions

S7 = sin(q[7]);
C7 = cos(q[7]);
S8 = sin(q[8]);
C8 = cos(q[8]);
S9 = sin(q[9]);
C9 = cos(q[9]);
S31 = sin(q[31]);
C31 = cos(q[31]);
S32 = sin(q[32]);
C32 = cos(q[32]);
S33 = sin(q[33]);
C33 = cos(q[33]);
S14 = sin(q[14]);
C14 = cos(q[14]);
S17 = sin(q[17]);
C17 = cos(q[17]);
S18 = sin(q[18]);
C18 = cos(q[18]);
S22 = sin(q[22]);
C22 = cos(q[22]);
S25 = sin(q[25]);
C25 = cos(q[25]);
S26 = sin(q[26]);
C26 = cos(q[26]);
 
// Augmented Joint Position Vectors

 
// Link anchor points Kinematics

ROlnk2_58 = C7*C8-S7*S8;
ROlnk2_68 = C7*S8+S7*C8;
ROlnk2_88 = -C7*S8-S7*C8;
ROlnk2_98 = C7*C8-S7*S8;
ROlnk2_59 = ROlnk2_58*C9+ROlnk2_88*S9;
ROlnk2_69 = ROlnk2_68*C9+ROlnk2_98*S9;
ROlnk2_89 = -ROlnk2_58*S9+ROlnk2_88*C9;
ROlnk2_99 = -ROlnk2_68*S9+ROlnk2_98*C9;
RLlnk2_22 = dpt[2][18]*C7;
RLlnk2_32 = dpt[2][18]*S7;
POlnk2_22 = RLlnk2_22+dpt[2][2];
POlnk2_32 = RLlnk2_32+dpt[3][2];
OMlnk2_12 = qd[7]+qd[8];
ORlnk2_22 = -qd[7]*RLlnk2_32;
ORlnk2_32 = qd[7]*RLlnk2_22;
RLlnk2_23 = ROlnk2_88*dpt[3][19];
RLlnk2_33 = ROlnk2_98*dpt[3][19];
POlnk2_23 = POlnk2_22+RLlnk2_23;
POlnk2_33 = POlnk2_32+RLlnk2_33;
OMlnk2_13 = qd[9]+OMlnk2_12;
ORlnk2_23 = -OMlnk2_12*RLlnk2_33;
ORlnk2_33 = OMlnk2_12*RLlnk2_23;
VIlnk2_23 = ORlnk2_22+ORlnk2_23;
VIlnk2_33 = ORlnk2_32+ORlnk2_33;
RLlnk2_24 = ROlnk2_59*dpt[2][22]+ROlnk2_89*dpt[3][22];
RLlnk2_34 = ROlnk2_69*dpt[2][22]+ROlnk2_99*dpt[3][22];
POlnk2_14 = dpt[1][22]+dpt[1][2];
POlnk2_24 = POlnk2_23+RLlnk2_24;
POlnk2_34 = POlnk2_33+RLlnk2_34;
ORlnk2_24 = -OMlnk2_13*RLlnk2_34;
ORlnk2_34 = OMlnk2_13*RLlnk2_24;
VIlnk2_24 = ORlnk2_24+VIlnk2_23;
VIlnk2_34 = ORlnk2_34+VIlnk2_33;
Plnk11 = POlnk2_14-dpt[1][11];
Plnk21 = POlnk2_24-dpt[2][11];
Plnk31 = POlnk2_34-dpt[3][11];
PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31;
Z1 = sqrt(PPlnk1);
e11 = Plnk11/Z1;
e21 = Plnk21/Z1;
e31 = Plnk31/Z1;
Zd1 = VIlnk2_24*e21+VIlnk2_34*e31;
ROlnk4_532 = C31*C32-S31*S32;
ROlnk4_632 = C31*S32+S31*C32;
ROlnk4_832 = -C31*S32-S31*C32;
ROlnk4_932 = C31*C32-S31*S32;
ROlnk4_533 = ROlnk4_532*C33+ROlnk4_832*S33;
ROlnk4_633 = ROlnk4_632*C33+ROlnk4_932*S33;
ROlnk4_833 = -ROlnk4_532*S33+ROlnk4_832*C33;
ROlnk4_933 = -ROlnk4_632*S33+ROlnk4_932*C33;
RLlnk4_22 = dpt[2][49]*C31;
RLlnk4_32 = dpt[2][49]*S31;
POlnk4_22 = RLlnk4_22+dpt[2][10];
POlnk4_32 = RLlnk4_32+dpt[3][10];
OMlnk4_12 = qd[31]+qd[32];
ORlnk4_22 = -qd[31]*RLlnk4_32;
ORlnk4_32 = qd[31]*RLlnk4_22;
RLlnk4_23 = ROlnk4_832*dpt[3][50];
RLlnk4_33 = ROlnk4_932*dpt[3][50];
POlnk4_23 = POlnk4_22+RLlnk4_23;
POlnk4_33 = POlnk4_32+RLlnk4_33;
OMlnk4_13 = qd[33]+OMlnk4_12;
ORlnk4_23 = -OMlnk4_12*RLlnk4_33;
ORlnk4_33 = OMlnk4_12*RLlnk4_23;
VIlnk4_23 = ORlnk4_22+ORlnk4_23;
VIlnk4_33 = ORlnk4_32+ORlnk4_33;
RLlnk4_24 = ROlnk4_533*dpt[2][53]+ROlnk4_833*dpt[3][53];
RLlnk4_34 = ROlnk4_633*dpt[2][53]+ROlnk4_933*dpt[3][53];
POlnk4_14 = dpt[1][10]+dpt[1][53];
POlnk4_24 = POlnk4_23+RLlnk4_24;
POlnk4_34 = POlnk4_33+RLlnk4_34;
ORlnk4_24 = -OMlnk4_13*RLlnk4_34;
ORlnk4_34 = OMlnk4_13*RLlnk4_24;
VIlnk4_24 = ORlnk4_24+VIlnk4_23;
VIlnk4_34 = ORlnk4_34+VIlnk4_33;
Plnk12 = POlnk4_14-dpt[1][13];
Plnk22 = POlnk4_24-dpt[2][13];
Plnk32 = POlnk4_34-dpt[3][13];
PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32;
Z2 = sqrt(PPlnk2);
e12 = Plnk12/Z2;
e22 = Plnk22/Z2;
e32 = Plnk32/Z2;
Zd2 = VIlnk4_24*e22+VIlnk4_34*e32;
ROlnk6_517 = C14*C17-S14*S17;
ROlnk6_617 = C14*S17+S14*C17;
ROlnk6_817 = -C14*S17-S14*C17;
ROlnk6_917 = C14*C17-S14*S17;
ROlnk6_518 = ROlnk6_517*C18+ROlnk6_817*S18;
ROlnk6_618 = ROlnk6_617*C18+ROlnk6_917*S18;
ROlnk6_818 = -ROlnk6_517*S18+ROlnk6_817*C18;
ROlnk6_918 = -ROlnk6_617*S18+ROlnk6_917*C18;
RLlnk6_22 = q[15]*C14;
RLlnk6_32 = q[15]*S14;
POlnk6_22 = RLlnk6_22+dpt[2][4];
POlnk6_32 = RLlnk6_32+dpt[3][4];
ORlnk6_22 = -qd[14]*RLlnk6_32;
ORlnk6_32 = qd[14]*RLlnk6_22;
VIlnk6_22 = ORlnk6_22+qd[15]*C14;
VIlnk6_32 = ORlnk6_32+qd[15]*S14;
RLlnk6_23 = -q[16]*S14;
RLlnk6_33 = q[16]*C14;
POlnk6_23 = POlnk6_22+RLlnk6_23;
POlnk6_33 = POlnk6_32+RLlnk6_33;
ORlnk6_23 = -qd[14]*RLlnk6_33;
ORlnk6_33 = qd[14]*RLlnk6_23;
VIlnk6_23 = ORlnk6_23+VIlnk6_22-qd[16]*S14;
VIlnk6_33 = ORlnk6_33+VIlnk6_32+qd[16]*C14;
RLlnk6_24 = dpt[2][29]*C14;
RLlnk6_34 = dpt[2][29]*S14;
POlnk6_24 = POlnk6_23+RLlnk6_24;
POlnk6_34 = POlnk6_33+RLlnk6_34;
OMlnk6_14 = qd[14]+qd[17];
ORlnk6_24 = -qd[14]*RLlnk6_34;
ORlnk6_34 = qd[14]*RLlnk6_24;
VIlnk6_24 = ORlnk6_24+VIlnk6_23;
VIlnk6_34 = ORlnk6_34+VIlnk6_33;
RLlnk6_25 = ROlnk6_817*dpt[3][30];
RLlnk6_35 = ROlnk6_917*dpt[3][30];
POlnk6_25 = POlnk6_24+RLlnk6_25;
POlnk6_35 = POlnk6_34+RLlnk6_35;
OMlnk6_15 = qd[18]+OMlnk6_14;
ORlnk6_25 = -OMlnk6_14*RLlnk6_35;
ORlnk6_35 = OMlnk6_14*RLlnk6_25;
VIlnk6_25 = ORlnk6_25+VIlnk6_24;
VIlnk6_35 = ORlnk6_35+VIlnk6_34;
RLlnk6_26 = ROlnk6_518*dpt[2][33]+ROlnk6_818*dpt[3][33];
RLlnk6_36 = ROlnk6_618*dpt[2][33]+ROlnk6_918*dpt[3][33];
POlnk6_16 = dpt[1][33]+dpt[1][4];
POlnk6_26 = POlnk6_25+RLlnk6_26;
POlnk6_36 = POlnk6_35+RLlnk6_36;
ORlnk6_26 = -OMlnk6_15*RLlnk6_36;
ORlnk6_36 = OMlnk6_15*RLlnk6_26;
VIlnk6_26 = ORlnk6_26+VIlnk6_25;
VIlnk6_36 = ORlnk6_36+VIlnk6_35;
Plnk13 = POlnk6_16-dpt[1][12];
Plnk23 = POlnk6_26-dpt[2][12];
Plnk33 = POlnk6_36-dpt[3][12];
PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33;
Z3 = sqrt(PPlnk3);
e13 = Plnk13/Z3;
e23 = Plnk23/Z3;
e33 = Plnk33/Z3;
Zd3 = VIlnk6_26*e23+VIlnk6_36*e33;
ROlnk7_525 = C22*C25-S22*S25;
ROlnk7_625 = C22*S25+S22*C25;
ROlnk7_825 = -C22*S25-S22*C25;
ROlnk7_925 = C22*C25-S22*S25;
ROlnk7_526 = ROlnk7_525*C26+ROlnk7_825*S26;
ROlnk7_626 = ROlnk7_625*C26+ROlnk7_925*S26;
ROlnk7_826 = -ROlnk7_525*S26+ROlnk7_825*C26;
ROlnk7_926 = -ROlnk7_625*S26+ROlnk7_925*C26;
RLlnk7_22 = q[23]*C22;
RLlnk7_32 = q[23]*S22;
POlnk7_22 = RLlnk7_22+dpt[2][7];
POlnk7_32 = RLlnk7_32+dpt[3][7];
ORlnk7_22 = -qd[22]*RLlnk7_32;
ORlnk7_32 = qd[22]*RLlnk7_22;
VIlnk7_22 = ORlnk7_22+qd[23]*C22;
VIlnk7_32 = ORlnk7_32+qd[23]*S22;
RLlnk7_23 = -q[24]*S22;
RLlnk7_33 = q[24]*C22;
POlnk7_23 = POlnk7_22+RLlnk7_23;
POlnk7_33 = POlnk7_32+RLlnk7_33;
ORlnk7_23 = -qd[22]*RLlnk7_33;
ORlnk7_33 = qd[22]*RLlnk7_23;
VIlnk7_23 = ORlnk7_23+VIlnk7_22-qd[24]*S22;
VIlnk7_33 = ORlnk7_33+VIlnk7_32+qd[24]*C22;
RLlnk7_24 = dpt[2][38]*C22;
RLlnk7_34 = dpt[2][38]*S22;
POlnk7_24 = POlnk7_23+RLlnk7_24;
POlnk7_34 = POlnk7_33+RLlnk7_34;
OMlnk7_14 = qd[22]+qd[25];
ORlnk7_24 = -qd[22]*RLlnk7_34;
ORlnk7_34 = qd[22]*RLlnk7_24;
VIlnk7_24 = ORlnk7_24+VIlnk7_23;
VIlnk7_34 = ORlnk7_34+VIlnk7_33;
RLlnk7_25 = ROlnk7_825*dpt[3][39];
RLlnk7_35 = ROlnk7_925*dpt[3][39];
POlnk7_25 = POlnk7_24+RLlnk7_25;
POlnk7_35 = POlnk7_34+RLlnk7_35;
OMlnk7_15 = qd[26]+OMlnk7_14;
ORlnk7_25 = -OMlnk7_14*RLlnk7_35;
ORlnk7_35 = OMlnk7_14*RLlnk7_25;
VIlnk7_25 = ORlnk7_25+VIlnk7_24;
VIlnk7_35 = ORlnk7_35+VIlnk7_34;
RLlnk7_26 = ROlnk7_526*dpt[2][42]+ROlnk7_826*dpt[3][42];
RLlnk7_36 = ROlnk7_626*dpt[2][42]+ROlnk7_926*dpt[3][42];
POlnk7_16 = dpt[1][42]+dpt[1][7];
POlnk7_26 = POlnk7_25+RLlnk7_26;
POlnk7_36 = POlnk7_35+RLlnk7_36;
ORlnk7_26 = -OMlnk7_15*RLlnk7_36;
ORlnk7_36 = OMlnk7_15*RLlnk7_26;
VIlnk7_26 = ORlnk7_26+VIlnk7_25;
VIlnk7_36 = ORlnk7_36+VIlnk7_35;
Plnk14 = -POlnk7_16+dpt[1][14];
Plnk24 = -POlnk7_26+dpt[2][14];
Plnk34 = -POlnk7_36+dpt[3][14];
PPlnk4 = Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34;
Z4 = sqrt(PPlnk4);
e14 = Plnk14/Z4;
e24 = Plnk24/Z4;
e34 = Plnk34/Z4;
Zd4 = -VIlnk7_26*e24-VIlnk7_36*e34;
RLlnk10_22 = dpt[2][18]*C7;
RLlnk10_32 = dpt[2][18]*S7;
POlnk10_22 = RLlnk10_22+dpt[2][2];
POlnk10_32 = RLlnk10_32+dpt[3][2];
ORlnk10_22 = -qd[7]*RLlnk10_32;
ORlnk10_32 = qd[7]*RLlnk10_22;
Plnk15 = -dpt[1][15]+dpt[1][2];
Plnk25 = -q[37]+POlnk10_22;
Plnk35 = POlnk10_32-dpt[3][15];
PPlnk5 = Plnk15*Plnk15+Plnk25*Plnk25+Plnk35*Plnk35;
Z5 = sqrt(PPlnk5);
e15 = Plnk15/Z5;
e25 = Plnk25/Z5;
e35 = Plnk35/Z5;
Zd5 = ORlnk10_32*e35+e25*(-qd[37]+ORlnk10_22);
RLlnk12_22 = q[15]*C14;
RLlnk12_32 = q[15]*S14;
POlnk12_22 = RLlnk12_22+dpt[2][4];
POlnk12_32 = RLlnk12_32+dpt[3][4];
ORlnk12_22 = -qd[14]*RLlnk12_32;
ORlnk12_32 = qd[14]*RLlnk12_22;
VIlnk12_22 = ORlnk12_22+qd[15]*C14;
VIlnk12_32 = ORlnk12_32+qd[15]*S14;
RLlnk12_23 = -q[16]*S14;
RLlnk12_33 = q[16]*C14;
POlnk12_23 = POlnk12_22+RLlnk12_23;
POlnk12_33 = POlnk12_32+RLlnk12_33;
ORlnk12_23 = -qd[14]*RLlnk12_33;
ORlnk12_33 = qd[14]*RLlnk12_23;
VIlnk12_23 = ORlnk12_23+VIlnk12_22-qd[16]*S14;
VIlnk12_33 = ORlnk12_33+VIlnk12_32+qd[16]*C14;
RLlnk12_24 = dpt[2][29]*C14;
RLlnk12_34 = dpt[2][29]*S14;
POlnk12_24 = POlnk12_23+RLlnk12_24;
POlnk12_34 = POlnk12_33+RLlnk12_34;
ORlnk12_24 = -qd[14]*RLlnk12_34;
ORlnk12_34 = qd[14]*RLlnk12_24;
VIlnk12_24 = ORlnk12_24+VIlnk12_23;
VIlnk12_34 = ORlnk12_34+VIlnk12_33;
Plnk16 = -dpt[1][15]+dpt[1][4];
Plnk26 = -q[38]+POlnk12_24;
Plnk36 = POlnk12_34-dpt[3][15];
PPlnk6 = Plnk16*Plnk16+Plnk26*Plnk26+Plnk36*Plnk36;
Z6 = sqrt(PPlnk6);
e16 = Plnk16/Z6;
e26 = Plnk26/Z6;
e36 = Plnk36/Z6;
Zd6 = VIlnk12_34*e36+e26*(-qd[38]+VIlnk12_24);
RLlnk14_22 = dpt[2][49]*C31;
RLlnk14_32 = dpt[2][49]*S31;
POlnk14_22 = RLlnk14_22+dpt[2][10];
POlnk14_32 = RLlnk14_32+dpt[3][10];
ORlnk14_22 = -qd[31]*RLlnk14_32;
ORlnk14_32 = qd[31]*RLlnk14_22;
Plnk17 = dpt[1][10]-dpt[1][16];
Plnk27 = -q[39]+POlnk14_22;
Plnk37 = POlnk14_32-dpt[3][16];
PPlnk7 = Plnk17*Plnk17+Plnk27*Plnk27+Plnk37*Plnk37;
Z7 = sqrt(PPlnk7);
e17 = Plnk17/Z7;
e27 = Plnk27/Z7;
e37 = Plnk37/Z7;
Zd7 = ORlnk14_32*e37+e27*(-qd[39]+ORlnk14_22);
RLlnk16_22 = q[23]*C22;
RLlnk16_32 = q[23]*S22;
POlnk16_22 = RLlnk16_22+dpt[2][7];
POlnk16_32 = RLlnk16_32+dpt[3][7];
ORlnk16_22 = -qd[22]*RLlnk16_32;
ORlnk16_32 = qd[22]*RLlnk16_22;
VIlnk16_22 = ORlnk16_22+qd[23]*C22;
VIlnk16_32 = ORlnk16_32+qd[23]*S22;
RLlnk16_23 = -q[24]*S22;
RLlnk16_33 = q[24]*C22;
POlnk16_23 = POlnk16_22+RLlnk16_23;
POlnk16_33 = POlnk16_32+RLlnk16_33;
ORlnk16_23 = -qd[22]*RLlnk16_33;
ORlnk16_33 = qd[22]*RLlnk16_23;
VIlnk16_23 = ORlnk16_23+VIlnk16_22-qd[24]*S22;
VIlnk16_33 = ORlnk16_33+VIlnk16_32+qd[24]*C22;
RLlnk16_24 = dpt[2][38]*C22;
RLlnk16_34 = dpt[2][38]*S22;
POlnk16_24 = POlnk16_23+RLlnk16_24;
POlnk16_34 = POlnk16_33+RLlnk16_34;
ORlnk16_24 = -qd[22]*RLlnk16_34;
ORlnk16_34 = qd[22]*RLlnk16_24;
VIlnk16_24 = ORlnk16_24+VIlnk16_23;
VIlnk16_34 = ORlnk16_34+VIlnk16_33;
Plnk18 = -dpt[1][16]+dpt[1][7];
Plnk28 = -q[40]+POlnk16_24;
Plnk38 = POlnk16_34-dpt[3][16];
PPlnk8 = Plnk18*Plnk18+Plnk28*Plnk28+Plnk38*Plnk38;
Z8 = sqrt(PPlnk8);
e18 = Plnk18/Z8;
e28 = Plnk28/Z8;
e38 = Plnk38/Z8;
Zd8 = VIlnk16_34*e38+e28*(-qd[40]+VIlnk16_24);

// Link Forces 

Flink1 = user_LinkForces(Z1,Zd1,s,tsim,1);
Flink2 = user_LinkForces(Z2,Zd2,s,tsim,2);
Flink3 = user_LinkForces(Z3,Zd3,s,tsim,3);
Flink4 = user_LinkForces(Z4,Zd4,s,tsim,4);
Flink5 = user_LinkForces(Z5,Zd5,s,tsim,5);
Flink6 = user_LinkForces(Z6,Zd6,s,tsim,6);
Flink7 = user_LinkForces(Z7,Zd7,s,tsim,7);
Flink8 = user_LinkForces(Z8,Zd8,s,tsim,8);
 
// Link Dynamics: forces projection on body-fixed frames

fPlnk11 = Flink1*e11;
fPlnk21 = Flink1*e21;
fPlnk31 = Flink1*e31;
trqlnk6_1_1 = -fPlnk21*dpt[3][11]+fPlnk31*dpt[2][11];
trqlnk6_1_2 = fPlnk11*dpt[3][11]-fPlnk31*dpt[1][11];
trqlnk6_1_3 = -fPlnk11*dpt[2][11]+fPlnk21*dpt[1][11];
fSlnk11 = Flink1*e11;
fSlnk21 = Flink1*(ROlnk2_59*e21+ROlnk2_69*e31);
fSlnk31 = Flink1*(ROlnk2_89*e21+ROlnk2_99*e31);
trqlnk9_1_1 = fSlnk21*dpt[3][22]-fSlnk31*(dpt[2][22]-l[2][9]);
trqlnk9_1_2 = -fSlnk11*dpt[3][22]+fSlnk31*dpt[1][22];
trqlnk9_1_3 = fSlnk11*(dpt[2][22]-l[2][9])-fSlnk21*dpt[1][22];
fPlnk12 = Flink2*e12;
fPlnk22 = Flink2*e22;
fPlnk32 = Flink2*e32;
frclnk6_2_1 = fPlnk11+fPlnk12;
frclnk6_2_2 = fPlnk21+fPlnk22;
frclnk6_2_3 = fPlnk31+fPlnk32;
trqlnk6_2_1 = trqlnk6_1_1-fPlnk22*dpt[3][13]+fPlnk32*dpt[2][13];
trqlnk6_2_2 = trqlnk6_1_2+fPlnk12*dpt[3][13]-fPlnk32*dpt[1][13];
trqlnk6_2_3 = trqlnk6_1_3-fPlnk12*dpt[2][13]+fPlnk22*dpt[1][13];
fSlnk12 = Flink2*e12;
fSlnk22 = Flink2*(ROlnk4_533*e22+ROlnk4_633*e32);
fSlnk32 = Flink2*(ROlnk4_833*e22+ROlnk4_933*e32);
trqlnk33_2_1 = fSlnk22*dpt[3][53]-fSlnk32*(dpt[2][53]-l[2][33]);
trqlnk33_2_2 = -fSlnk12*dpt[3][53]+fSlnk32*dpt[1][53];
trqlnk33_2_3 = fSlnk12*(dpt[2][53]-l[2][33])-fSlnk22*dpt[1][53];
fPlnk13 = Flink3*e13;
fPlnk23 = Flink3*e23;
fPlnk33 = Flink3*e33;
frclnk6_3_1 = fPlnk13+frclnk6_2_1;
frclnk6_3_2 = fPlnk23+frclnk6_2_2;
frclnk6_3_3 = fPlnk33+frclnk6_2_3;
trqlnk6_3_1 = trqlnk6_2_1-fPlnk23*dpt[3][12]+fPlnk33*dpt[2][12];
trqlnk6_3_2 = trqlnk6_2_2+fPlnk13*dpt[3][12]-fPlnk33*dpt[1][12];
trqlnk6_3_3 = trqlnk6_2_3-fPlnk13*dpt[2][12]+fPlnk23*dpt[1][12];
fSlnk13 = Flink3*e13;
fSlnk23 = Flink3*(ROlnk6_518*e23+ROlnk6_618*e33);
fSlnk33 = Flink3*(ROlnk6_818*e23+ROlnk6_918*e33);
trqlnk18_3_1 = fSlnk23*dpt[3][33]-fSlnk33*(dpt[2][33]-l[2][18]);
trqlnk18_3_2 = -fSlnk13*dpt[3][33]+fSlnk33*dpt[1][33];
trqlnk18_3_3 = fSlnk13*(dpt[2][33]-l[2][18])-fSlnk23*dpt[1][33];
fPlnk14 = Flink4*e14;
fPlnk24 = Flink4*(ROlnk7_526*e24+ROlnk7_626*e34);
fPlnk34 = Flink4*(ROlnk7_826*e24+ROlnk7_926*e34);
trqlnk26_4_1 = -fPlnk24*dpt[3][42]+fPlnk34*(dpt[2][42]-l[2][26]);
trqlnk26_4_2 = fPlnk14*dpt[3][42]-fPlnk34*dpt[1][42];
trqlnk26_4_3 = -fPlnk14*(dpt[2][42]-l[2][26])+fPlnk24*dpt[1][42];
fSlnk14 = Flink4*e14;
fSlnk24 = Flink4*e24;
fSlnk34 = Flink4*e34;
frclnk6_4_1 = -fSlnk14+frclnk6_3_1;
frclnk6_4_2 = -fSlnk24+frclnk6_3_2;
frclnk6_4_3 = -fSlnk34+frclnk6_3_3;
trqlnk6_4_1 = trqlnk6_3_1+fSlnk24*dpt[3][14]-fSlnk34*dpt[2][14];
trqlnk6_4_2 = trqlnk6_3_2-fSlnk14*dpt[3][14]+fSlnk34*dpt[1][14];
trqlnk6_4_3 = trqlnk6_3_3+fSlnk14*dpt[2][14]-fSlnk24*dpt[1][14];
fPlnk15 = Flink5*e15;
fPlnk25 = Flink5*e25;
fPlnk35 = Flink5*e35;
fSlnk15 = Flink5*e15;
fSlnk25 = Flink5*(e25*C7+e35*S7);
fSlnk35 = Flink5*(-e25*S7+e35*C7);
trqlnk7_5_1 = -fSlnk35*dpt[2][18];
trqlnk7_5_3 = fSlnk15*dpt[2][18];
fPlnk16 = Flink6*e16;
fPlnk26 = Flink6*e26;
fPlnk36 = Flink6*e36;
fSlnk16 = Flink6*e16;
fSlnk26 = Flink6*(e26*C14+e36*S14);
fSlnk36 = Flink6*(-e26*S14+e36*C14);
trqlnk16_6_1 = -fSlnk36*dpt[2][29];
trqlnk16_6_3 = fSlnk16*dpt[2][29];
fPlnk17 = Flink7*e17;
fPlnk27 = Flink7*e27;
fPlnk37 = Flink7*e37;
fSlnk17 = Flink7*e17;
fSlnk27 = Flink7*(e27*C31+e37*S31);
fSlnk37 = Flink7*(-e27*S31+e37*C31);
trqlnk31_7_1 = -fSlnk37*dpt[2][49];
trqlnk31_7_3 = fSlnk17*dpt[2][49];
fPlnk18 = Flink8*e18;
fPlnk28 = Flink8*e28;
fPlnk38 = Flink8*e38;
fSlnk18 = Flink8*e18;
fSlnk28 = Flink8*(e28*C22+e38*S22);
fSlnk38 = Flink8*(-e28*S22+e38*C22);
trqlnk24_8_1 = -fSlnk38*dpt[2][38];
trqlnk24_8_3 = fSlnk18*dpt[2][38];
 
// Symbolic model output

frc[1][6] = frc[1][6]+frclnk6_4_1;
frc[2][6] = frc[2][6]+frclnk6_4_2;
frc[3][6] = frc[3][6]+frclnk6_4_3;
trq[1][6] = trq[1][6]+trqlnk6_4_1;
trq[2][6] = trq[2][6]+trqlnk6_4_2;
trq[3][6] = trq[3][6]+trqlnk6_4_3;
frc[1][7] = frc[1][7]-fSlnk15;
frc[2][7] = frc[2][7]-fSlnk25;
frc[3][7] = frc[3][7]-fSlnk35;
trq[1][7] = trq[1][7]+trqlnk7_5_1;
trq[3][7] = trq[3][7]+trqlnk7_5_3;
frc[1][9] = frc[1][9]-fSlnk11;
frc[2][9] = frc[2][9]-fSlnk21;
frc[3][9] = frc[3][9]-fSlnk31;
trq[1][9] = trq[1][9]+trqlnk9_1_1;
trq[2][9] = trq[2][9]+trqlnk9_1_2;
trq[3][9] = trq[3][9]+trqlnk9_1_3;
frc[1][16] = frc[1][16]-fSlnk16;
frc[2][16] = frc[2][16]-fSlnk26;
frc[3][16] = frc[3][16]-fSlnk36;
trq[1][16] = trq[1][16]+trqlnk16_6_1;
trq[3][16] = trq[3][16]+trqlnk16_6_3;
frc[1][18] = frc[1][18]-fSlnk13;
frc[2][18] = frc[2][18]-fSlnk23;
frc[3][18] = frc[3][18]-fSlnk33;
trq[1][18] = trq[1][18]+trqlnk18_3_1;
trq[2][18] = trq[2][18]+trqlnk18_3_2;
trq[3][18] = trq[3][18]+trqlnk18_3_3;
frc[1][24] = frc[1][24]-fSlnk18;
frc[2][24] = frc[2][24]-fSlnk28;
frc[3][24] = frc[3][24]-fSlnk38;
trq[1][24] = trq[1][24]+trqlnk24_8_1;
trq[3][24] = trq[3][24]+trqlnk24_8_3;
frc[1][26] = frc[1][26]+fPlnk14;
frc[2][26] = frc[2][26]+fPlnk24;
frc[3][26] = frc[3][26]+fPlnk34;
trq[1][26] = trq[1][26]+trqlnk26_4_1;
trq[2][26] = trq[2][26]+trqlnk26_4_2;
trq[3][26] = trq[3][26]+trqlnk26_4_3;
frc[1][31] = frc[1][31]-fSlnk17;
frc[2][31] = frc[2][31]-fSlnk27;
frc[3][31] = frc[3][31]-fSlnk37;
trq[1][31] = trq[1][31]+trqlnk31_7_1;
trq[3][31] = trq[3][31]+trqlnk31_7_3;
frc[1][33] = frc[1][33]-fSlnk12;
frc[2][33] = frc[2][33]-fSlnk22;
frc[3][33] = frc[3][33]-fSlnk32;
trq[1][33] = trq[1][33]+trqlnk33_2_1;
trq[2][33] = trq[2][33]+trqlnk33_2_2;
trq[3][33] = trq[3][33]+trqlnk33_2_3;
frc[1][37] = frc[1][37]+fPlnk15;
frc[2][37] = frc[2][37]+fPlnk25;
frc[3][37] = frc[3][37]+fPlnk35;
frc[1][38] = frc[1][38]+fPlnk16;
frc[2][38] = frc[2][38]+fPlnk26;
frc[3][38] = frc[3][38]+fPlnk36;
frc[1][39] = frc[1][39]+fPlnk17;
frc[2][39] = frc[2][39]+fPlnk27;
frc[3][39] = frc[3][39]+fPlnk37;
frc[1][40] = frc[1][40]+fPlnk18;
frc[2][40] = frc[2][40]+fPlnk28;
frc[3][40] = frc[3][40]+fPlnk38;
 
// Symbolic model output

Z[1] = Z1;
Zd[1] = Zd1;
Flink[1] = Flink1;
Z[2] = Z2;
Zd[2] = Zd2;
Flink[2] = Flink2;
Z[3] = Z3;
Zd[3] = Zd3;
Flink[3] = Flink3;
Z[4] = Z4;
Zd[4] = Zd4;
Flink[4] = Flink4;
Z[5] = Z5;
Zd[5] = Zd5;
Flink[5] = Flink5;
Z[6] = Z6;
Zd[6] = Zd6;
Flink[6] = Flink6;
Z[7] = Z7;
Zd[7] = Zd7;
Flink[7] = Flink7;
Z[8] = Z8;
Zd[8] = Zd8;
Flink[8] = Flink8;

// Number of continuation lines = 0

}
