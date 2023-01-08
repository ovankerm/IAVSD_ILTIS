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
<<<<<<< HEAD
//	==> Generation Date: Sun Jan  8 21:56:23 2023
=======
//	==> Generation Date: Sun Jan  8 13:26:38 2023
>>>>>>> 4e3c68237ce49cc63248a517797502ff720fc3d5
=======
//	==> Generation Date: Sun Jan  8 22:23:52 2023
>>>>>>> Stashed changes
//
//	==> Project name: Jeep
//
//	==> Number of joints: 41
//
//	==> Function: F6 - Sensors Kinematics
//
//	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
//

#include <math.h> 

#include "mbs_data.h"
#include "mbs_sensor.h"

void mbs_sensor(MbsSensor *sens,
MbsData *s, int isens)
{
#include "mbs_sensor_Jeep.h"

double *q, *qd, *qdd;
double **dpt;

q = s->q;
qd = s->qd;
qdd = s->qdd;

dpt = s->dpt;
 
// Trigonometric functions

S4 = sin(q[4]);
C4 = cos(q[4]);
S5 = sin(q[5]);
C5 = cos(q[5]);
S6 = sin(q[6]);
C6 = cos(q[6]);
S7 = sin(q[7]);
C7 = cos(q[7]);
S8 = sin(q[8]);
C8 = cos(q[8]);
S10 = sin(q[10]);
C10 = cos(q[10]);
S11 = sin(q[11]);
C11 = cos(q[11]);
S12 = sin(q[12]);
C12 = cos(q[12]);
S14 = sin(q[14]);
C14 = cos(q[14]);
S17 = sin(q[17]);
C17 = cos(q[17]);
S19 = sin(q[19]);
C19 = cos(q[19]);
S20 = sin(q[20]);
C20 = cos(q[20]);
S21 = sin(q[21]);
C21 = cos(q[21]);
S22 = sin(q[22]);
C22 = cos(q[22]);
S25 = sin(q[25]);
C25 = cos(q[25]);
S27 = sin(q[27]);
C27 = cos(q[27]);
S28 = sin(q[28]);
C28 = cos(q[28]);
S29 = sin(q[29]);
C29 = cos(q[29]);
S31 = sin(q[31]);
C31 = cos(q[31]);
S32 = sin(q[32]);
C32 = cos(q[32]);
S34 = sin(q[34]);
C34 = cos(q[34]);
S35 = sin(q[35]);
C35 = cos(q[35]);
S36 = sin(q[36]);
C36 = cos(q[36]);
 
// Augmented Joint Position Vectors

Dz413 = q[41]+dpt[3][17];
 
// Augmented Joint Position Vectors

 
// Sensor Kinematics


switch(isens)
{
case 1:

ROcp1_45 = S4*S5;
ROcp1_65 = C4*S5;
ROcp1_75 = S4*C5;
ROcp1_95 = C4*C5;
ROcp1_16 = ROcp1_45*S6+C4*C6;
ROcp1_26 = C5*S6;
ROcp1_36 = ROcp1_65*S6-S4*C6;
ROcp1_46 = ROcp1_45*C6-C4*S6;
ROcp1_56 = C5*C6;
ROcp1_66 = ROcp1_65*C6+S4*S6;
OMcp1_15 = qd[5]*C4;
OMcp1_35 = -qd[5]*S4;
OPcp1_15 = qdd[5]*C4-qd[4]*qd[5]*S4;
OPcp1_35 = -qdd[5]*S4-qd[4]*qd[5]*C4;
OMcp1_16 = OMcp1_15+ROcp1_75*qd[6];
OMcp1_26 = qd[4]-qd[6]*S5;
OMcp1_36 = OMcp1_35+ROcp1_95*qd[6];
OPcp1_16 = OPcp1_15+ROcp1_75*qdd[6]+qd[6]*(OMcp1_35*S5+ROcp1_95*qd[4]);
OPcp1_26 = qdd[4]-qdd[6]*S5+qd[6]*(-OMcp1_15*ROcp1_95+OMcp1_35*ROcp1_75);
OPcp1_36 = OPcp1_35+ROcp1_95*qdd[6]+qd[6]*(-OMcp1_15*S5-ROcp1_75*qd[4]);
RLcp1_17 = ROcp1_46*dpt[2][17]+ROcp1_75*Dz413;
RLcp1_27 = ROcp1_56*dpt[2][17]-Dz413*S5;
RLcp1_37 = ROcp1_66*dpt[2][17]+ROcp1_95*Dz413;
POcp1_17 = RLcp1_17+q[1];
POcp1_27 = RLcp1_27+q[2];
POcp1_37 = RLcp1_37+q[3];
JTcp1_17_5 = RLcp1_27*S4;
JTcp1_27_5 = -RLcp1_17*S4-RLcp1_37*C4;
JTcp1_37_5 = RLcp1_27*C4;
JTcp1_17_6 = -RLcp1_27*ROcp1_95-RLcp1_37*S5;
JTcp1_27_6 = RLcp1_17*ROcp1_95-RLcp1_37*ROcp1_75;
JTcp1_37_6 = RLcp1_17*S5+RLcp1_27*ROcp1_75;
ORcp1_17 = OMcp1_26*RLcp1_37-OMcp1_36*RLcp1_27;
ORcp1_27 = -OMcp1_16*RLcp1_37+OMcp1_36*RLcp1_17;
ORcp1_37 = OMcp1_16*RLcp1_27-OMcp1_26*RLcp1_17;
VIcp1_17 = ORcp1_17+qd[1]+ROcp1_75*qd[41];
VIcp1_27 = ORcp1_27+qd[2]-qd[41]*S5;
VIcp1_37 = ORcp1_37+qd[3]+ROcp1_95*qd[41];
ACcp1_17 = qdd[1]+OMcp1_26*ORcp1_37-OMcp1_36*ORcp1_27+OPcp1_26*RLcp1_37-OPcp1_36*RLcp1_27+ROcp1_75*qdd[41]+(2.0)*qd[41]*(
 OMcp1_26*ROcp1_95+OMcp1_36*S5);
ACcp1_27 = qdd[2]-OMcp1_16*ORcp1_37+OMcp1_36*ORcp1_17-OPcp1_16*RLcp1_37+OPcp1_36*RLcp1_17-qdd[41]*S5+(2.0)*qd[41]*(-
 OMcp1_16*ROcp1_95+OMcp1_36*ROcp1_75);
ACcp1_37 = qdd[3]+OMcp1_16*ORcp1_27-OMcp1_26*ORcp1_17+OPcp1_16*RLcp1_27-OPcp1_26*RLcp1_17+ROcp1_95*qdd[41]+(2.0)*qd[41]*(-
 OMcp1_16*S5-OMcp1_26*ROcp1_75);
sens->P[1] = POcp1_17;
sens->P[2] = POcp1_27;
sens->P[3] = POcp1_37;
sens->R[1][1] = ROcp1_16;
sens->R[1][2] = ROcp1_26;
sens->R[1][3] = ROcp1_36;
sens->R[2][1] = ROcp1_46;
sens->R[2][2] = ROcp1_56;
sens->R[2][3] = ROcp1_66;
sens->R[3][1] = ROcp1_75;
sens->R[3][2] = -S5;
sens->R[3][3] = ROcp1_95;
sens->V[1] = VIcp1_17;
sens->V[2] = VIcp1_27;
sens->V[3] = VIcp1_37;
sens->OM[1] = OMcp1_16;
sens->OM[2] = OMcp1_26;
sens->OM[3] = OMcp1_36;
sens->J[1][1] = (1.0);
sens->J[1][4] = RLcp1_37;
sens->J[1][5] = JTcp1_17_5;
sens->J[1][6] = JTcp1_17_6;
sens->J[1][41] = ROcp1_75;
sens->J[2][2] = (1.0);
sens->J[2][5] = JTcp1_27_5;
sens->J[2][6] = JTcp1_27_6;
sens->J[2][41] = -S5;
sens->J[3][3] = (1.0);
sens->J[3][4] = -RLcp1_17;
sens->J[3][5] = JTcp1_37_5;
sens->J[3][6] = JTcp1_37_6;
sens->J[3][41] = ROcp1_95;
sens->J[4][5] = C4;
sens->J[4][6] = ROcp1_75;
sens->J[5][4] = (1.0);
sens->J[5][6] = -S5;
sens->J[6][5] = -S4;
sens->J[6][6] = ROcp1_95;
sens->A[1] = ACcp1_17;
sens->A[2] = ACcp1_27;
sens->A[3] = ACcp1_37;
sens->OMP[1] = OPcp1_16;
sens->OMP[2] = OPcp1_26;
sens->OMP[3] = OPcp1_36;

break;

case 2:

ROcp2_45 = S4*S5;
ROcp2_65 = C4*S5;
ROcp2_75 = S4*C5;
ROcp2_95 = C4*C5;
ROcp2_16 = ROcp2_45*S6+C4*C6;
ROcp2_26 = C5*S6;
ROcp2_36 = ROcp2_65*S6-S4*C6;
ROcp2_46 = ROcp2_45*C6-C4*S6;
ROcp2_56 = C5*C6;
ROcp2_66 = ROcp2_65*C6+S4*S6;
ROcp2_47 = ROcp2_46*C7+ROcp2_75*S7;
ROcp2_57 = ROcp2_56*C7-S5*S7;
ROcp2_67 = ROcp2_66*C7+ROcp2_95*S7;
ROcp2_77 = -ROcp2_46*S7+ROcp2_75*C7;
ROcp2_87 = -ROcp2_56*S7-S5*C7;
ROcp2_97 = -ROcp2_66*S7+ROcp2_95*C7;
ROcp2_48 = ROcp2_47*C8+ROcp2_77*S8;
ROcp2_58 = ROcp2_57*C8+ROcp2_87*S8;
ROcp2_68 = ROcp2_67*C8+ROcp2_97*S8;
ROcp2_78 = -ROcp2_47*S8+ROcp2_77*C8;
ROcp2_88 = -ROcp2_57*S8+ROcp2_87*C8;
ROcp2_98 = -ROcp2_67*S8+ROcp2_97*C8;
ROcp2_110 = ROcp2_16*C10+ROcp2_48*S10;
ROcp2_210 = ROcp2_26*C10+ROcp2_58*S10;
ROcp2_310 = ROcp2_36*C10+ROcp2_68*S10;
ROcp2_410 = -ROcp2_16*S10+ROcp2_48*C10;
ROcp2_510 = -ROcp2_26*S10+ROcp2_58*C10;
ROcp2_610 = -ROcp2_36*S10+ROcp2_68*C10;
ROcp2_411 = ROcp2_410*C11+ROcp2_78*S11;
ROcp2_511 = ROcp2_510*C11+ROcp2_88*S11;
ROcp2_611 = ROcp2_610*C11+ROcp2_98*S11;
ROcp2_711 = -ROcp2_410*S11+ROcp2_78*C11;
ROcp2_811 = -ROcp2_510*S11+ROcp2_88*C11;
ROcp2_911 = -ROcp2_610*S11+ROcp2_98*C11;
ROcp2_112 = ROcp2_110*C12-ROcp2_711*S12;
ROcp2_212 = ROcp2_210*C12-ROcp2_811*S12;
ROcp2_312 = ROcp2_310*C12-ROcp2_911*S12;
ROcp2_712 = ROcp2_110*S12+ROcp2_711*C12;
ROcp2_812 = ROcp2_210*S12+ROcp2_811*C12;
ROcp2_912 = ROcp2_310*S12+ROcp2_911*C12;
OMcp2_15 = qd[5]*C4;
OMcp2_35 = -qd[5]*S4;
OPcp2_15 = qdd[5]*C4-qd[4]*qd[5]*S4;
OPcp2_35 = -qdd[5]*S4-qd[4]*qd[5]*C4;
OMcp2_16 = OMcp2_15+ROcp2_75*qd[6];
OMcp2_26 = qd[4]-qd[6]*S5;
OMcp2_36 = OMcp2_35+ROcp2_95*qd[6];
OPcp2_16 = OPcp2_15+ROcp2_75*qdd[6]+qd[6]*(OMcp2_35*S5+ROcp2_95*qd[4]);
OPcp2_26 = qdd[4]-qdd[6]*S5+qd[6]*(-OMcp2_15*ROcp2_95+OMcp2_35*ROcp2_75);
OPcp2_36 = OPcp2_35+ROcp2_95*qdd[6]+qd[6]*(-OMcp2_15*S5-ROcp2_75*qd[4]);
RLcp2_17 = ROcp2_16*dpt[1][2]+ROcp2_46*dpt[2][2]+ROcp2_75*dpt[3][2];
RLcp2_27 = ROcp2_26*dpt[1][2]+ROcp2_56*dpt[2][2]-dpt[3][2]*S5;
RLcp2_37 = ROcp2_36*dpt[1][2]+ROcp2_66*dpt[2][2]+ROcp2_95*dpt[3][2];
POcp2_17 = RLcp2_17+q[1];
POcp2_27 = RLcp2_27+q[2];
POcp2_37 = RLcp2_37+q[3];
OMcp2_17 = OMcp2_16+ROcp2_16*qd[7];
OMcp2_27 = OMcp2_26+ROcp2_26*qd[7];
OMcp2_37 = OMcp2_36+ROcp2_36*qd[7];
ORcp2_17 = OMcp2_26*RLcp2_37-OMcp2_36*RLcp2_27;
ORcp2_27 = -OMcp2_16*RLcp2_37+OMcp2_36*RLcp2_17;
ORcp2_37 = OMcp2_16*RLcp2_27-OMcp2_26*RLcp2_17;
VIcp2_17 = ORcp2_17+qd[1];
VIcp2_27 = ORcp2_27+qd[2];
VIcp2_37 = ORcp2_37+qd[3];
OPcp2_17 = OPcp2_16+ROcp2_16*qdd[7]+qd[7]*(OMcp2_26*ROcp2_36-OMcp2_36*ROcp2_26);
OPcp2_27 = OPcp2_26+ROcp2_26*qdd[7]+qd[7]*(-OMcp2_16*ROcp2_36+OMcp2_36*ROcp2_16);
OPcp2_37 = OPcp2_36+ROcp2_36*qdd[7]+qd[7]*(OMcp2_16*ROcp2_26-OMcp2_26*ROcp2_16);
ACcp2_17 = qdd[1]+OMcp2_26*ORcp2_37-OMcp2_36*ORcp2_27+OPcp2_26*RLcp2_37-OPcp2_36*RLcp2_27;
ACcp2_27 = qdd[2]-OMcp2_16*ORcp2_37+OMcp2_36*ORcp2_17-OPcp2_16*RLcp2_37+OPcp2_36*RLcp2_17;
ACcp2_37 = qdd[3]+OMcp2_16*ORcp2_27-OMcp2_26*ORcp2_17+OPcp2_16*RLcp2_27-OPcp2_26*RLcp2_17;
RLcp2_18 = ROcp2_47*dpt[2][18];
RLcp2_28 = ROcp2_57*dpt[2][18];
RLcp2_38 = ROcp2_67*dpt[2][18];
POcp2_18 = POcp2_17+RLcp2_18;
POcp2_28 = POcp2_27+RLcp2_28;
POcp2_38 = POcp2_37+RLcp2_38;
OMcp2_18 = OMcp2_17+ROcp2_16*qd[8];
OMcp2_28 = OMcp2_27+ROcp2_26*qd[8];
OMcp2_38 = OMcp2_37+ROcp2_36*qd[8];
ORcp2_18 = OMcp2_27*RLcp2_38-OMcp2_37*RLcp2_28;
ORcp2_28 = -OMcp2_17*RLcp2_38+OMcp2_37*RLcp2_18;
ORcp2_38 = OMcp2_17*RLcp2_28-OMcp2_27*RLcp2_18;
VIcp2_18 = ORcp2_18+VIcp2_17;
VIcp2_28 = ORcp2_28+VIcp2_27;
VIcp2_38 = ORcp2_38+VIcp2_37;
OPcp2_18 = OPcp2_17+ROcp2_16*qdd[8]+qd[8]*(OMcp2_27*ROcp2_36-OMcp2_37*ROcp2_26);
OPcp2_28 = OPcp2_27+ROcp2_26*qdd[8]+qd[8]*(-OMcp2_17*ROcp2_36+OMcp2_37*ROcp2_16);
OPcp2_38 = OPcp2_37+ROcp2_36*qdd[8]+qd[8]*(OMcp2_17*ROcp2_26-OMcp2_27*ROcp2_16);
ACcp2_18 = ACcp2_17+OMcp2_27*ORcp2_38-OMcp2_37*ORcp2_28+OPcp2_27*RLcp2_38-OPcp2_37*RLcp2_28;
ACcp2_28 = ACcp2_27-OMcp2_17*ORcp2_38+OMcp2_37*ORcp2_18-OPcp2_17*RLcp2_38+OPcp2_37*RLcp2_18;
ACcp2_38 = ACcp2_37+OMcp2_17*ORcp2_28-OMcp2_27*ORcp2_18+OPcp2_17*RLcp2_28-OPcp2_27*RLcp2_18;
RLcp2_19 = ROcp2_78*dpt[3][20];
RLcp2_29 = ROcp2_88*dpt[3][20];
RLcp2_39 = ROcp2_98*dpt[3][20];
POcp2_19 = POcp2_18+RLcp2_19;
POcp2_29 = POcp2_28+RLcp2_29;
POcp2_39 = POcp2_38+RLcp2_39;
OMcp2_19 = OMcp2_18+ROcp2_78*qd[10];
OMcp2_29 = OMcp2_28+ROcp2_88*qd[10];
OMcp2_39 = OMcp2_38+ROcp2_98*qd[10];
ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29;
ORcp2_29 = -OMcp2_18*RLcp2_39+OMcp2_38*RLcp2_19;
ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19;
VIcp2_19 = ORcp2_19+VIcp2_18;
VIcp2_29 = ORcp2_29+VIcp2_28;
VIcp2_39 = ORcp2_39+VIcp2_38;
OPcp2_19 = OPcp2_18+ROcp2_78*qdd[10]+qd[10]*(OMcp2_28*ROcp2_98-OMcp2_38*ROcp2_88);
OPcp2_29 = OPcp2_28+ROcp2_88*qdd[10]+qd[10]*(-OMcp2_18*ROcp2_98+OMcp2_38*ROcp2_78);
OPcp2_39 = OPcp2_38+ROcp2_98*qdd[10]+qd[10]*(OMcp2_18*ROcp2_88-OMcp2_28*ROcp2_78);
ACcp2_19 = ACcp2_18+OMcp2_28*ORcp2_39-OMcp2_38*ORcp2_29+OPcp2_28*RLcp2_39-OPcp2_38*RLcp2_29;
ACcp2_29 = ACcp2_28-OMcp2_18*ORcp2_39+OMcp2_38*ORcp2_19-OPcp2_18*RLcp2_39+OPcp2_38*RLcp2_19;
ACcp2_39 = ACcp2_38+OMcp2_18*ORcp2_29-OMcp2_28*ORcp2_19+OPcp2_18*RLcp2_29-OPcp2_28*RLcp2_19;
RLcp2_110 = ROcp2_78*dpt[3][23];
RLcp2_210 = ROcp2_88*dpt[3][23];
RLcp2_310 = ROcp2_98*dpt[3][23];
POcp2_110 = POcp2_19+RLcp2_110;
POcp2_210 = POcp2_29+RLcp2_210;
POcp2_310 = POcp2_39+RLcp2_310;
OMcp2_110 = OMcp2_19+ROcp2_110*qd[11];
OMcp2_210 = OMcp2_29+ROcp2_210*qd[11];
OMcp2_310 = OMcp2_39+ROcp2_310*qd[11];
ORcp2_110 = OMcp2_29*RLcp2_310-OMcp2_39*RLcp2_210;
ORcp2_210 = -OMcp2_19*RLcp2_310+OMcp2_39*RLcp2_110;
ORcp2_310 = OMcp2_19*RLcp2_210-OMcp2_29*RLcp2_110;
VIcp2_110 = ORcp2_110+VIcp2_19;
VIcp2_210 = ORcp2_210+VIcp2_29;
VIcp2_310 = ORcp2_310+VIcp2_39;
OPcp2_110 = OPcp2_19+ROcp2_110*qdd[11]+qd[11]*(OMcp2_29*ROcp2_310-OMcp2_39*ROcp2_210);
OPcp2_210 = OPcp2_29+ROcp2_210*qdd[11]+qd[11]*(-OMcp2_19*ROcp2_310+OMcp2_39*ROcp2_110);
OPcp2_310 = OPcp2_39+ROcp2_310*qdd[11]+qd[11]*(OMcp2_19*ROcp2_210-OMcp2_29*ROcp2_110);
ACcp2_110 = ACcp2_19+OMcp2_29*ORcp2_310-OMcp2_39*ORcp2_210+OPcp2_29*RLcp2_310-OPcp2_39*RLcp2_210;
ACcp2_210 = ACcp2_29-OMcp2_19*ORcp2_310+OMcp2_39*ORcp2_110-OPcp2_19*RLcp2_310+OPcp2_39*RLcp2_110;
ACcp2_310 = ACcp2_39+OMcp2_19*ORcp2_210-OMcp2_29*ORcp2_110+OPcp2_19*RLcp2_210-OPcp2_29*RLcp2_110;
RLcp2_111 = ROcp2_411*dpt[2][25]+ROcp2_711*dpt[3][25];
RLcp2_211 = ROcp2_511*dpt[2][25]+ROcp2_811*dpt[3][25];
RLcp2_311 = ROcp2_611*dpt[2][25]+ROcp2_911*dpt[3][25];
POcp2_111 = POcp2_110+RLcp2_111;
POcp2_211 = POcp2_210+RLcp2_211;
POcp2_311 = POcp2_310+RLcp2_311;
OMcp2_111 = OMcp2_110+ROcp2_411*qd[12];
OMcp2_211 = OMcp2_210+ROcp2_511*qd[12];
OMcp2_311 = OMcp2_310+ROcp2_611*qd[12];
ORcp2_111 = OMcp2_210*RLcp2_311-OMcp2_310*RLcp2_211;
ORcp2_211 = -OMcp2_110*RLcp2_311+OMcp2_310*RLcp2_111;
ORcp2_311 = OMcp2_110*RLcp2_211-OMcp2_210*RLcp2_111;
VIcp2_111 = ORcp2_111+VIcp2_110;
VIcp2_211 = ORcp2_211+VIcp2_210;
VIcp2_311 = ORcp2_311+VIcp2_310;
OPcp2_111 = OPcp2_110+ROcp2_411*qdd[12]+qd[12]*(OMcp2_210*ROcp2_611-OMcp2_310*ROcp2_511);
OPcp2_211 = OPcp2_210+ROcp2_511*qdd[12]+qd[12]*(-OMcp2_110*ROcp2_611+OMcp2_310*ROcp2_411);
OPcp2_311 = OPcp2_310+ROcp2_611*qdd[12]+qd[12]*(OMcp2_110*ROcp2_511-OMcp2_210*ROcp2_411);
ACcp2_111 = ACcp2_110+OMcp2_210*ORcp2_311-OMcp2_310*ORcp2_211+OPcp2_210*RLcp2_311-OPcp2_310*RLcp2_211;
ACcp2_211 = ACcp2_210-OMcp2_110*ORcp2_311+OMcp2_310*ORcp2_111-OPcp2_110*RLcp2_311+OPcp2_310*RLcp2_111;
ACcp2_311 = ACcp2_310+OMcp2_110*ORcp2_211-OMcp2_210*ORcp2_111+OPcp2_110*RLcp2_211-OPcp2_210*RLcp2_111;
sens->P[1] = POcp2_111;
sens->P[2] = POcp2_211;
sens->P[3] = POcp2_311;
sens->R[1][1] = ROcp2_112;
sens->R[1][2] = ROcp2_212;
sens->R[1][3] = ROcp2_312;
sens->R[2][1] = ROcp2_411;
sens->R[2][2] = ROcp2_511;
sens->R[2][3] = ROcp2_611;
sens->R[3][1] = ROcp2_712;
sens->R[3][2] = ROcp2_812;
sens->R[3][3] = ROcp2_912;
sens->V[1] = VIcp2_111;
sens->V[2] = VIcp2_211;
sens->V[3] = VIcp2_311;
sens->OM[1] = OMcp2_111;
sens->OM[2] = OMcp2_211;
sens->OM[3] = OMcp2_311;
sens->A[1] = ACcp2_111;
sens->A[2] = ACcp2_211;
sens->A[3] = ACcp2_311;
sens->OMP[1] = OPcp2_111;
sens->OMP[2] = OPcp2_211;
sens->OMP[3] = OPcp2_311;

break;

case 3:

ROcp3_45 = S4*S5;
ROcp3_65 = C4*S5;
ROcp3_75 = S4*C5;
ROcp3_95 = C4*C5;
ROcp3_16 = ROcp3_45*S6+C4*C6;
ROcp3_26 = C5*S6;
ROcp3_36 = ROcp3_65*S6-S4*C6;
ROcp3_46 = ROcp3_45*C6-C4*S6;
ROcp3_56 = C5*C6;
ROcp3_66 = ROcp3_65*C6+S4*S6;
ROcp3_414 = ROcp3_46*C14+ROcp3_75*S14;
ROcp3_514 = ROcp3_56*C14-S14*S5;
ROcp3_614 = ROcp3_66*C14+ROcp3_95*S14;
ROcp3_714 = -ROcp3_46*S14+ROcp3_75*C14;
ROcp3_814 = -ROcp3_56*S14-C14*S5;
ROcp3_914 = -ROcp3_66*S14+ROcp3_95*C14;
ROcp3_417 = ROcp3_414*C17+ROcp3_714*S17;
ROcp3_517 = ROcp3_514*C17+ROcp3_814*S17;
ROcp3_617 = ROcp3_614*C17+ROcp3_914*S17;
ROcp3_717 = -ROcp3_414*S17+ROcp3_714*C17;
ROcp3_817 = -ROcp3_514*S17+ROcp3_814*C17;
ROcp3_917 = -ROcp3_614*S17+ROcp3_914*C17;
ROcp3_119 = ROcp3_16*C19+ROcp3_417*S19;
ROcp3_219 = ROcp3_26*C19+ROcp3_517*S19;
ROcp3_319 = ROcp3_36*C19+ROcp3_617*S19;
ROcp3_419 = -ROcp3_16*S19+ROcp3_417*C19;
ROcp3_519 = -ROcp3_26*S19+ROcp3_517*C19;
ROcp3_619 = -ROcp3_36*S19+ROcp3_617*C19;
ROcp3_420 = ROcp3_419*C20+ROcp3_717*S20;
ROcp3_520 = ROcp3_519*C20+ROcp3_817*S20;
ROcp3_620 = ROcp3_619*C20+ROcp3_917*S20;
ROcp3_720 = -ROcp3_419*S20+ROcp3_717*C20;
ROcp3_820 = -ROcp3_519*S20+ROcp3_817*C20;
ROcp3_920 = -ROcp3_619*S20+ROcp3_917*C20;
ROcp3_121 = ROcp3_119*C21-ROcp3_720*S21;
ROcp3_221 = ROcp3_219*C21-ROcp3_820*S21;
ROcp3_321 = ROcp3_319*C21-ROcp3_920*S21;
ROcp3_721 = ROcp3_119*S21+ROcp3_720*C21;
ROcp3_821 = ROcp3_219*S21+ROcp3_820*C21;
ROcp3_921 = ROcp3_319*S21+ROcp3_920*C21;
OMcp3_15 = qd[5]*C4;
OMcp3_35 = -qd[5]*S4;
OPcp3_15 = qdd[5]*C4-qd[4]*qd[5]*S4;
OPcp3_35 = -qdd[5]*S4-qd[4]*qd[5]*C4;
OMcp3_16 = OMcp3_15+ROcp3_75*qd[6];
OMcp3_26 = qd[4]-qd[6]*S5;
OMcp3_36 = OMcp3_35+ROcp3_95*qd[6];
OPcp3_16 = OPcp3_15+ROcp3_75*qdd[6]+qd[6]*(OMcp3_35*S5+ROcp3_95*qd[4]);
OPcp3_26 = qdd[4]-qdd[6]*S5+qd[6]*(-OMcp3_15*ROcp3_95+OMcp3_35*ROcp3_75);
OPcp3_36 = OPcp3_35+ROcp3_95*qdd[6]+qd[6]*(-OMcp3_15*S5-ROcp3_75*qd[4]);
RLcp3_17 = ROcp3_16*dpt[1][4]+ROcp3_46*dpt[2][4]+ROcp3_75*dpt[3][4];
RLcp3_27 = ROcp3_26*dpt[1][4]+ROcp3_56*dpt[2][4]-dpt[3][4]*S5;
RLcp3_37 = ROcp3_36*dpt[1][4]+ROcp3_66*dpt[2][4]+ROcp3_95*dpt[3][4];
POcp3_17 = RLcp3_17+q[1];
POcp3_27 = RLcp3_27+q[2];
POcp3_37 = RLcp3_37+q[3];
OMcp3_17 = OMcp3_16+ROcp3_16*qd[14];
OMcp3_27 = OMcp3_26+ROcp3_26*qd[14];
OMcp3_37 = OMcp3_36+ROcp3_36*qd[14];
ORcp3_17 = OMcp3_26*RLcp3_37-OMcp3_36*RLcp3_27;
ORcp3_27 = -OMcp3_16*RLcp3_37+OMcp3_36*RLcp3_17;
ORcp3_37 = OMcp3_16*RLcp3_27-OMcp3_26*RLcp3_17;
VIcp3_17 = ORcp3_17+qd[1];
VIcp3_27 = ORcp3_27+qd[2];
VIcp3_37 = ORcp3_37+qd[3];
OPcp3_17 = OPcp3_16+ROcp3_16*qdd[14]+qd[14]*(OMcp3_26*ROcp3_36-OMcp3_36*ROcp3_26);
OPcp3_27 = OPcp3_26+ROcp3_26*qdd[14]+qd[14]*(-OMcp3_16*ROcp3_36+OMcp3_36*ROcp3_16);
OPcp3_37 = OPcp3_36+ROcp3_36*qdd[14]+qd[14]*(OMcp3_16*ROcp3_26-OMcp3_26*ROcp3_16);
ACcp3_17 = qdd[1]+OMcp3_26*ORcp3_37-OMcp3_36*ORcp3_27+OPcp3_26*RLcp3_37-OPcp3_36*RLcp3_27;
ACcp3_27 = qdd[2]-OMcp3_16*ORcp3_37+OMcp3_36*ORcp3_17-OPcp3_16*RLcp3_37+OPcp3_36*RLcp3_17;
ACcp3_37 = qdd[3]+OMcp3_16*ORcp3_27-OMcp3_26*ORcp3_17+OPcp3_16*RLcp3_27-OPcp3_26*RLcp3_17;
RLcp3_18 = ROcp3_414*q[15];
RLcp3_28 = ROcp3_514*q[15];
RLcp3_38 = ROcp3_614*q[15];
POcp3_18 = POcp3_17+RLcp3_18;
POcp3_28 = POcp3_27+RLcp3_28;
POcp3_38 = POcp3_37+RLcp3_38;
ORcp3_18 = OMcp3_27*RLcp3_38-OMcp3_37*RLcp3_28;
ORcp3_28 = -OMcp3_17*RLcp3_38+OMcp3_37*RLcp3_18;
ORcp3_38 = OMcp3_17*RLcp3_28-OMcp3_27*RLcp3_18;
VIcp3_18 = ORcp3_18+VIcp3_17+ROcp3_414*qd[15];
VIcp3_28 = ORcp3_28+VIcp3_27+ROcp3_514*qd[15];
VIcp3_38 = ORcp3_38+VIcp3_37+ROcp3_614*qd[15];
ACcp3_18 = ACcp3_17+OMcp3_27*ORcp3_38-OMcp3_37*ORcp3_28+OPcp3_27*RLcp3_38-OPcp3_37*RLcp3_28+ROcp3_414*qdd[15]+(2.0)*qd[15]*
 (OMcp3_27*ROcp3_614-OMcp3_37*ROcp3_514);
ACcp3_28 = ACcp3_27-OMcp3_17*ORcp3_38+OMcp3_37*ORcp3_18-OPcp3_17*RLcp3_38+OPcp3_37*RLcp3_18+ROcp3_514*qdd[15]+(2.0)*qd[15]*
 (-OMcp3_17*ROcp3_614+OMcp3_37*ROcp3_414);
ACcp3_38 = ACcp3_37+OMcp3_17*ORcp3_28-OMcp3_27*ORcp3_18+OPcp3_17*RLcp3_28-OPcp3_27*RLcp3_18+ROcp3_614*qdd[15]+(2.0)*qd[15]*
 (OMcp3_17*ROcp3_514-OMcp3_27*ROcp3_414);
RLcp3_19 = ROcp3_714*q[16];
RLcp3_29 = ROcp3_814*q[16];
RLcp3_39 = ROcp3_914*q[16];
POcp3_19 = POcp3_18+RLcp3_19;
POcp3_29 = POcp3_28+RLcp3_29;
POcp3_39 = POcp3_38+RLcp3_39;
ORcp3_19 = OMcp3_27*RLcp3_39-OMcp3_37*RLcp3_29;
ORcp3_29 = -OMcp3_17*RLcp3_39+OMcp3_37*RLcp3_19;
ORcp3_39 = OMcp3_17*RLcp3_29-OMcp3_27*RLcp3_19;
VIcp3_19 = ORcp3_19+VIcp3_18+ROcp3_714*qd[16];
VIcp3_29 = ORcp3_29+VIcp3_28+ROcp3_814*qd[16];
VIcp3_39 = ORcp3_39+VIcp3_38+ROcp3_914*qd[16];
ACcp3_19 = ACcp3_18+OMcp3_27*ORcp3_39-OMcp3_37*ORcp3_29+OPcp3_27*RLcp3_39-OPcp3_37*RLcp3_29+ROcp3_714*qdd[16]+(2.0)*qd[16]*
 (OMcp3_27*ROcp3_914-OMcp3_37*ROcp3_814);
ACcp3_29 = ACcp3_28-OMcp3_17*ORcp3_39+OMcp3_37*ORcp3_19-OPcp3_17*RLcp3_39+OPcp3_37*RLcp3_19+ROcp3_814*qdd[16]+(2.0)*qd[16]*
 (-OMcp3_17*ROcp3_914+OMcp3_37*ROcp3_714);
ACcp3_39 = ACcp3_38+OMcp3_17*ORcp3_29-OMcp3_27*ORcp3_19+OPcp3_17*RLcp3_29-OPcp3_27*RLcp3_19+ROcp3_914*qdd[16]+(2.0)*qd[16]*
 (OMcp3_17*ROcp3_814-OMcp3_27*ROcp3_714);
RLcp3_110 = ROcp3_414*dpt[2][29];
RLcp3_210 = ROcp3_514*dpt[2][29];
RLcp3_310 = ROcp3_614*dpt[2][29];
POcp3_110 = POcp3_19+RLcp3_110;
POcp3_210 = POcp3_29+RLcp3_210;
POcp3_310 = POcp3_39+RLcp3_310;
OMcp3_110 = OMcp3_17+ROcp3_16*qd[17];
OMcp3_210 = OMcp3_27+ROcp3_26*qd[17];
OMcp3_310 = OMcp3_37+ROcp3_36*qd[17];
ORcp3_110 = OMcp3_27*RLcp3_310-OMcp3_37*RLcp3_210;
ORcp3_210 = -OMcp3_17*RLcp3_310+OMcp3_37*RLcp3_110;
ORcp3_310 = OMcp3_17*RLcp3_210-OMcp3_27*RLcp3_110;
VIcp3_110 = ORcp3_110+VIcp3_19;
VIcp3_210 = ORcp3_210+VIcp3_29;
VIcp3_310 = ORcp3_310+VIcp3_39;
OPcp3_110 = OPcp3_17+ROcp3_16*qdd[17]+qd[17]*(OMcp3_27*ROcp3_36-OMcp3_37*ROcp3_26);
OPcp3_210 = OPcp3_27+ROcp3_26*qdd[17]+qd[17]*(-OMcp3_17*ROcp3_36+OMcp3_37*ROcp3_16);
OPcp3_310 = OPcp3_37+ROcp3_36*qdd[17]+qd[17]*(OMcp3_17*ROcp3_26-OMcp3_27*ROcp3_16);
ACcp3_110 = ACcp3_19+OMcp3_27*ORcp3_310-OMcp3_37*ORcp3_210+OPcp3_27*RLcp3_310-OPcp3_37*RLcp3_210;
ACcp3_210 = ACcp3_29-OMcp3_17*ORcp3_310+OMcp3_37*ORcp3_110-OPcp3_17*RLcp3_310+OPcp3_37*RLcp3_110;
ACcp3_310 = ACcp3_39+OMcp3_17*ORcp3_210-OMcp3_27*ORcp3_110+OPcp3_17*RLcp3_210-OPcp3_27*RLcp3_110;
RLcp3_111 = ROcp3_717*dpt[3][31];
RLcp3_211 = ROcp3_817*dpt[3][31];
RLcp3_311 = ROcp3_917*dpt[3][31];
POcp3_111 = POcp3_110+RLcp3_111;
POcp3_211 = POcp3_210+RLcp3_211;
POcp3_311 = POcp3_310+RLcp3_311;
OMcp3_111 = OMcp3_110+ROcp3_717*qd[19];
OMcp3_211 = OMcp3_210+ROcp3_817*qd[19];
OMcp3_311 = OMcp3_310+ROcp3_917*qd[19];
ORcp3_111 = OMcp3_210*RLcp3_311-OMcp3_310*RLcp3_211;
ORcp3_211 = -OMcp3_110*RLcp3_311+OMcp3_310*RLcp3_111;
ORcp3_311 = OMcp3_110*RLcp3_211-OMcp3_210*RLcp3_111;
VIcp3_111 = ORcp3_111+VIcp3_110;
VIcp3_211 = ORcp3_211+VIcp3_210;
VIcp3_311 = ORcp3_311+VIcp3_310;
OPcp3_111 = OPcp3_110+ROcp3_717*qdd[19]+qd[19]*(OMcp3_210*ROcp3_917-OMcp3_310*ROcp3_817);
OPcp3_211 = OPcp3_210+ROcp3_817*qdd[19]+qd[19]*(-OMcp3_110*ROcp3_917+OMcp3_310*ROcp3_717);
OPcp3_311 = OPcp3_310+ROcp3_917*qdd[19]+qd[19]*(OMcp3_110*ROcp3_817-OMcp3_210*ROcp3_717);
ACcp3_111 = ACcp3_110+OMcp3_210*ORcp3_311-OMcp3_310*ORcp3_211+OPcp3_210*RLcp3_311-OPcp3_310*RLcp3_211;
ACcp3_211 = ACcp3_210-OMcp3_110*ORcp3_311+OMcp3_310*ORcp3_111-OPcp3_110*RLcp3_311+OPcp3_310*RLcp3_111;
ACcp3_311 = ACcp3_310+OMcp3_110*ORcp3_211-OMcp3_210*ORcp3_111+OPcp3_110*RLcp3_211-OPcp3_210*RLcp3_111;
RLcp3_112 = ROcp3_717*dpt[3][34];
RLcp3_212 = ROcp3_817*dpt[3][34];
RLcp3_312 = ROcp3_917*dpt[3][34];
POcp3_112 = POcp3_111+RLcp3_112;
POcp3_212 = POcp3_211+RLcp3_212;
POcp3_312 = POcp3_311+RLcp3_312;
OMcp3_112 = OMcp3_111+ROcp3_119*qd[20];
OMcp3_212 = OMcp3_211+ROcp3_219*qd[20];
OMcp3_312 = OMcp3_311+ROcp3_319*qd[20];
ORcp3_112 = OMcp3_211*RLcp3_312-OMcp3_311*RLcp3_212;
ORcp3_212 = -OMcp3_111*RLcp3_312+OMcp3_311*RLcp3_112;
ORcp3_312 = OMcp3_111*RLcp3_212-OMcp3_211*RLcp3_112;
VIcp3_112 = ORcp3_112+VIcp3_111;
VIcp3_212 = ORcp3_212+VIcp3_211;
VIcp3_312 = ORcp3_312+VIcp3_311;
OPcp3_112 = OPcp3_111+ROcp3_119*qdd[20]+qd[20]*(OMcp3_211*ROcp3_319-OMcp3_311*ROcp3_219);
OPcp3_212 = OPcp3_211+ROcp3_219*qdd[20]+qd[20]*(-OMcp3_111*ROcp3_319+OMcp3_311*ROcp3_119);
OPcp3_312 = OPcp3_311+ROcp3_319*qdd[20]+qd[20]*(OMcp3_111*ROcp3_219-OMcp3_211*ROcp3_119);
ACcp3_112 = ACcp3_111+OMcp3_211*ORcp3_312-OMcp3_311*ORcp3_212+OPcp3_211*RLcp3_312-OPcp3_311*RLcp3_212;
ACcp3_212 = ACcp3_211-OMcp3_111*ORcp3_312+OMcp3_311*ORcp3_112-OPcp3_111*RLcp3_312+OPcp3_311*RLcp3_112;
ACcp3_312 = ACcp3_311+OMcp3_111*ORcp3_212-OMcp3_211*ORcp3_112+OPcp3_111*RLcp3_212-OPcp3_211*RLcp3_112;
RLcp3_113 = ROcp3_420*dpt[2][36]+ROcp3_720*dpt[3][36];
RLcp3_213 = ROcp3_520*dpt[2][36]+ROcp3_820*dpt[3][36];
RLcp3_313 = ROcp3_620*dpt[2][36]+ROcp3_920*dpt[3][36];
POcp3_113 = POcp3_112+RLcp3_113;
POcp3_213 = POcp3_212+RLcp3_213;
POcp3_313 = POcp3_312+RLcp3_313;
OMcp3_113 = OMcp3_112+ROcp3_420*qd[21];
OMcp3_213 = OMcp3_212+ROcp3_520*qd[21];
OMcp3_313 = OMcp3_312+ROcp3_620*qd[21];
ORcp3_113 = OMcp3_212*RLcp3_313-OMcp3_312*RLcp3_213;
ORcp3_213 = -OMcp3_112*RLcp3_313+OMcp3_312*RLcp3_113;
ORcp3_313 = OMcp3_112*RLcp3_213-OMcp3_212*RLcp3_113;
VIcp3_113 = ORcp3_113+VIcp3_112;
VIcp3_213 = ORcp3_213+VIcp3_212;
VIcp3_313 = ORcp3_313+VIcp3_312;
OPcp3_113 = OPcp3_112+ROcp3_420*qdd[21]+qd[21]*(OMcp3_212*ROcp3_620-OMcp3_312*ROcp3_520);
OPcp3_213 = OPcp3_212+ROcp3_520*qdd[21]+qd[21]*(-OMcp3_112*ROcp3_620+OMcp3_312*ROcp3_420);
OPcp3_313 = OPcp3_312+ROcp3_620*qdd[21]+qd[21]*(OMcp3_112*ROcp3_520-OMcp3_212*ROcp3_420);
ACcp3_113 = ACcp3_112+OMcp3_212*ORcp3_313-OMcp3_312*ORcp3_213+OPcp3_212*RLcp3_313-OPcp3_312*RLcp3_213;
ACcp3_213 = ACcp3_212-OMcp3_112*ORcp3_313+OMcp3_312*ORcp3_113-OPcp3_112*RLcp3_313+OPcp3_312*RLcp3_113;
ACcp3_313 = ACcp3_312+OMcp3_112*ORcp3_213-OMcp3_212*ORcp3_113+OPcp3_112*RLcp3_213-OPcp3_212*RLcp3_113;
sens->P[1] = POcp3_113;
sens->P[2] = POcp3_213;
sens->P[3] = POcp3_313;
sens->R[1][1] = ROcp3_121;
sens->R[1][2] = ROcp3_221;
sens->R[1][3] = ROcp3_321;
sens->R[2][1] = ROcp3_420;
sens->R[2][2] = ROcp3_520;
sens->R[2][3] = ROcp3_620;
sens->R[3][1] = ROcp3_721;
sens->R[3][2] = ROcp3_821;
sens->R[3][3] = ROcp3_921;
sens->V[1] = VIcp3_113;
sens->V[2] = VIcp3_213;
sens->V[3] = VIcp3_313;
sens->OM[1] = OMcp3_113;
sens->OM[2] = OMcp3_213;
sens->OM[3] = OMcp3_313;
sens->A[1] = ACcp3_113;
sens->A[2] = ACcp3_213;
sens->A[3] = ACcp3_313;
sens->OMP[1] = OPcp3_113;
sens->OMP[2] = OPcp3_213;
sens->OMP[3] = OPcp3_313;

break;

case 4:

ROcp4_45 = S4*S5;
ROcp4_65 = C4*S5;
ROcp4_75 = S4*C5;
ROcp4_95 = C4*C5;
ROcp4_16 = ROcp4_45*S6+C4*C6;
ROcp4_26 = C5*S6;
ROcp4_36 = ROcp4_65*S6-S4*C6;
ROcp4_46 = ROcp4_45*C6-C4*S6;
ROcp4_56 = C5*C6;
ROcp4_66 = ROcp4_65*C6+S4*S6;
ROcp4_422 = ROcp4_46*C22+ROcp4_75*S22;
ROcp4_522 = ROcp4_56*C22-S22*S5;
ROcp4_622 = ROcp4_66*C22+ROcp4_95*S22;
ROcp4_722 = -ROcp4_46*S22+ROcp4_75*C22;
ROcp4_822 = -ROcp4_56*S22-C22*S5;
ROcp4_922 = -ROcp4_66*S22+ROcp4_95*C22;
ROcp4_425 = ROcp4_422*C25+ROcp4_722*S25;
ROcp4_525 = ROcp4_522*C25+ROcp4_822*S25;
ROcp4_625 = ROcp4_622*C25+ROcp4_922*S25;
ROcp4_725 = -ROcp4_422*S25+ROcp4_722*C25;
ROcp4_825 = -ROcp4_522*S25+ROcp4_822*C25;
ROcp4_925 = -ROcp4_622*S25+ROcp4_922*C25;
ROcp4_127 = ROcp4_16*C27+ROcp4_425*S27;
ROcp4_227 = ROcp4_26*C27+ROcp4_525*S27;
ROcp4_327 = ROcp4_36*C27+ROcp4_625*S27;
ROcp4_427 = -ROcp4_16*S27+ROcp4_425*C27;
ROcp4_527 = -ROcp4_26*S27+ROcp4_525*C27;
ROcp4_627 = -ROcp4_36*S27+ROcp4_625*C27;
ROcp4_428 = ROcp4_427*C28+ROcp4_725*S28;
ROcp4_528 = ROcp4_527*C28+ROcp4_825*S28;
ROcp4_628 = ROcp4_627*C28+ROcp4_925*S28;
ROcp4_728 = -ROcp4_427*S28+ROcp4_725*C28;
ROcp4_828 = -ROcp4_527*S28+ROcp4_825*C28;
ROcp4_928 = -ROcp4_627*S28+ROcp4_925*C28;
ROcp4_129 = ROcp4_127*C29-ROcp4_728*S29;
ROcp4_229 = ROcp4_227*C29-ROcp4_828*S29;
ROcp4_329 = ROcp4_327*C29-ROcp4_928*S29;
ROcp4_729 = ROcp4_127*S29+ROcp4_728*C29;
ROcp4_829 = ROcp4_227*S29+ROcp4_828*C29;
ROcp4_929 = ROcp4_327*S29+ROcp4_928*C29;
OMcp4_15 = qd[5]*C4;
OMcp4_35 = -qd[5]*S4;
OPcp4_15 = qdd[5]*C4-qd[4]*qd[5]*S4;
OPcp4_35 = -qdd[5]*S4-qd[4]*qd[5]*C4;
OMcp4_16 = OMcp4_15+ROcp4_75*qd[6];
OMcp4_26 = qd[4]-qd[6]*S5;
OMcp4_36 = OMcp4_35+ROcp4_95*qd[6];
OPcp4_16 = OPcp4_15+ROcp4_75*qdd[6]+qd[6]*(OMcp4_35*S5+ROcp4_95*qd[4]);
OPcp4_26 = qdd[4]-qdd[6]*S5+qd[6]*(-OMcp4_15*ROcp4_95+OMcp4_35*ROcp4_75);
OPcp4_36 = OPcp4_35+ROcp4_95*qdd[6]+qd[6]*(-OMcp4_15*S5-ROcp4_75*qd[4]);
RLcp4_17 = ROcp4_16*dpt[1][7]+ROcp4_46*dpt[2][7]+ROcp4_75*dpt[3][7];
RLcp4_27 = ROcp4_26*dpt[1][7]+ROcp4_56*dpt[2][7]-dpt[3][7]*S5;
RLcp4_37 = ROcp4_36*dpt[1][7]+ROcp4_66*dpt[2][7]+ROcp4_95*dpt[3][7];
POcp4_17 = RLcp4_17+q[1];
POcp4_27 = RLcp4_27+q[2];
POcp4_37 = RLcp4_37+q[3];
OMcp4_17 = OMcp4_16+ROcp4_16*qd[22];
OMcp4_27 = OMcp4_26+ROcp4_26*qd[22];
OMcp4_37 = OMcp4_36+ROcp4_36*qd[22];
ORcp4_17 = OMcp4_26*RLcp4_37-OMcp4_36*RLcp4_27;
ORcp4_27 = -OMcp4_16*RLcp4_37+OMcp4_36*RLcp4_17;
ORcp4_37 = OMcp4_16*RLcp4_27-OMcp4_26*RLcp4_17;
VIcp4_17 = ORcp4_17+qd[1];
VIcp4_27 = ORcp4_27+qd[2];
VIcp4_37 = ORcp4_37+qd[3];
OPcp4_17 = OPcp4_16+ROcp4_16*qdd[22]+qd[22]*(OMcp4_26*ROcp4_36-OMcp4_36*ROcp4_26);
OPcp4_27 = OPcp4_26+ROcp4_26*qdd[22]+qd[22]*(-OMcp4_16*ROcp4_36+OMcp4_36*ROcp4_16);
OPcp4_37 = OPcp4_36+ROcp4_36*qdd[22]+qd[22]*(OMcp4_16*ROcp4_26-OMcp4_26*ROcp4_16);
ACcp4_17 = qdd[1]+OMcp4_26*ORcp4_37-OMcp4_36*ORcp4_27+OPcp4_26*RLcp4_37-OPcp4_36*RLcp4_27;
ACcp4_27 = qdd[2]-OMcp4_16*ORcp4_37+OMcp4_36*ORcp4_17-OPcp4_16*RLcp4_37+OPcp4_36*RLcp4_17;
ACcp4_37 = qdd[3]+OMcp4_16*ORcp4_27-OMcp4_26*ORcp4_17+OPcp4_16*RLcp4_27-OPcp4_26*RLcp4_17;
RLcp4_18 = ROcp4_422*q[23];
RLcp4_28 = ROcp4_522*q[23];
RLcp4_38 = ROcp4_622*q[23];
POcp4_18 = POcp4_17+RLcp4_18;
POcp4_28 = POcp4_27+RLcp4_28;
POcp4_38 = POcp4_37+RLcp4_38;
ORcp4_18 = OMcp4_27*RLcp4_38-OMcp4_37*RLcp4_28;
ORcp4_28 = -OMcp4_17*RLcp4_38+OMcp4_37*RLcp4_18;
ORcp4_38 = OMcp4_17*RLcp4_28-OMcp4_27*RLcp4_18;
VIcp4_18 = ORcp4_18+VIcp4_17+ROcp4_422*qd[23];
VIcp4_28 = ORcp4_28+VIcp4_27+ROcp4_522*qd[23];
VIcp4_38 = ORcp4_38+VIcp4_37+ROcp4_622*qd[23];
ACcp4_18 = ACcp4_17+OMcp4_27*ORcp4_38-OMcp4_37*ORcp4_28+OPcp4_27*RLcp4_38-OPcp4_37*RLcp4_28+ROcp4_422*qdd[23]+(2.0)*qd[23]*
 (OMcp4_27*ROcp4_622-OMcp4_37*ROcp4_522);
ACcp4_28 = ACcp4_27-OMcp4_17*ORcp4_38+OMcp4_37*ORcp4_18-OPcp4_17*RLcp4_38+OPcp4_37*RLcp4_18+ROcp4_522*qdd[23]+(2.0)*qd[23]*
 (-OMcp4_17*ROcp4_622+OMcp4_37*ROcp4_422);
ACcp4_38 = ACcp4_37+OMcp4_17*ORcp4_28-OMcp4_27*ORcp4_18+OPcp4_17*RLcp4_28-OPcp4_27*RLcp4_18+ROcp4_622*qdd[23]+(2.0)*qd[23]*
 (OMcp4_17*ROcp4_522-OMcp4_27*ROcp4_422);
RLcp4_19 = ROcp4_722*q[24];
RLcp4_29 = ROcp4_822*q[24];
RLcp4_39 = ROcp4_922*q[24];
POcp4_19 = POcp4_18+RLcp4_19;
POcp4_29 = POcp4_28+RLcp4_29;
POcp4_39 = POcp4_38+RLcp4_39;
ORcp4_19 = OMcp4_27*RLcp4_39-OMcp4_37*RLcp4_29;
ORcp4_29 = -OMcp4_17*RLcp4_39+OMcp4_37*RLcp4_19;
ORcp4_39 = OMcp4_17*RLcp4_29-OMcp4_27*RLcp4_19;
VIcp4_19 = ORcp4_19+VIcp4_18+ROcp4_722*qd[24];
VIcp4_29 = ORcp4_29+VIcp4_28+ROcp4_822*qd[24];
VIcp4_39 = ORcp4_39+VIcp4_38+ROcp4_922*qd[24];
ACcp4_19 = ACcp4_18+OMcp4_27*ORcp4_39-OMcp4_37*ORcp4_29+OPcp4_27*RLcp4_39-OPcp4_37*RLcp4_29+ROcp4_722*qdd[24]+(2.0)*qd[24]*
 (OMcp4_27*ROcp4_922-OMcp4_37*ROcp4_822);
ACcp4_29 = ACcp4_28-OMcp4_17*ORcp4_39+OMcp4_37*ORcp4_19-OPcp4_17*RLcp4_39+OPcp4_37*RLcp4_19+ROcp4_822*qdd[24]+(2.0)*qd[24]*
 (-OMcp4_17*ROcp4_922+OMcp4_37*ROcp4_722);
ACcp4_39 = ACcp4_38+OMcp4_17*ORcp4_29-OMcp4_27*ORcp4_19+OPcp4_17*RLcp4_29-OPcp4_27*RLcp4_19+ROcp4_922*qdd[24]+(2.0)*qd[24]*
 (OMcp4_17*ROcp4_822-OMcp4_27*ROcp4_722);
RLcp4_110 = ROcp4_422*dpt[2][38];
RLcp4_210 = ROcp4_522*dpt[2][38];
RLcp4_310 = ROcp4_622*dpt[2][38];
POcp4_110 = POcp4_19+RLcp4_110;
POcp4_210 = POcp4_29+RLcp4_210;
POcp4_310 = POcp4_39+RLcp4_310;
OMcp4_110 = OMcp4_17+ROcp4_16*qd[25];
OMcp4_210 = OMcp4_27+ROcp4_26*qd[25];
OMcp4_310 = OMcp4_37+ROcp4_36*qd[25];
ORcp4_110 = OMcp4_27*RLcp4_310-OMcp4_37*RLcp4_210;
ORcp4_210 = -OMcp4_17*RLcp4_310+OMcp4_37*RLcp4_110;
ORcp4_310 = OMcp4_17*RLcp4_210-OMcp4_27*RLcp4_110;
VIcp4_110 = ORcp4_110+VIcp4_19;
VIcp4_210 = ORcp4_210+VIcp4_29;
VIcp4_310 = ORcp4_310+VIcp4_39;
OPcp4_110 = OPcp4_17+ROcp4_16*qdd[25]+qd[25]*(OMcp4_27*ROcp4_36-OMcp4_37*ROcp4_26);
OPcp4_210 = OPcp4_27+ROcp4_26*qdd[25]+qd[25]*(-OMcp4_17*ROcp4_36+OMcp4_37*ROcp4_16);
OPcp4_310 = OPcp4_37+ROcp4_36*qdd[25]+qd[25]*(OMcp4_17*ROcp4_26-OMcp4_27*ROcp4_16);
ACcp4_110 = ACcp4_19+OMcp4_27*ORcp4_310-OMcp4_37*ORcp4_210+OPcp4_27*RLcp4_310-OPcp4_37*RLcp4_210;
ACcp4_210 = ACcp4_29-OMcp4_17*ORcp4_310+OMcp4_37*ORcp4_110-OPcp4_17*RLcp4_310+OPcp4_37*RLcp4_110;
ACcp4_310 = ACcp4_39+OMcp4_17*ORcp4_210-OMcp4_27*ORcp4_110+OPcp4_17*RLcp4_210-OPcp4_27*RLcp4_110;
RLcp4_111 = ROcp4_725*dpt[3][40];
RLcp4_211 = ROcp4_825*dpt[3][40];
RLcp4_311 = ROcp4_925*dpt[3][40];
POcp4_111 = POcp4_110+RLcp4_111;
POcp4_211 = POcp4_210+RLcp4_211;
POcp4_311 = POcp4_310+RLcp4_311;
OMcp4_111 = OMcp4_110+ROcp4_725*qd[27];
OMcp4_211 = OMcp4_210+ROcp4_825*qd[27];
OMcp4_311 = OMcp4_310+ROcp4_925*qd[27];
ORcp4_111 = OMcp4_210*RLcp4_311-OMcp4_310*RLcp4_211;
ORcp4_211 = -OMcp4_110*RLcp4_311+OMcp4_310*RLcp4_111;
ORcp4_311 = OMcp4_110*RLcp4_211-OMcp4_210*RLcp4_111;
VIcp4_111 = ORcp4_111+VIcp4_110;
VIcp4_211 = ORcp4_211+VIcp4_210;
VIcp4_311 = ORcp4_311+VIcp4_310;
OPcp4_111 = OPcp4_110+ROcp4_725*qdd[27]+qd[27]*(OMcp4_210*ROcp4_925-OMcp4_310*ROcp4_825);
OPcp4_211 = OPcp4_210+ROcp4_825*qdd[27]+qd[27]*(-OMcp4_110*ROcp4_925+OMcp4_310*ROcp4_725);
OPcp4_311 = OPcp4_310+ROcp4_925*qdd[27]+qd[27]*(OMcp4_110*ROcp4_825-OMcp4_210*ROcp4_725);
ACcp4_111 = ACcp4_110+OMcp4_210*ORcp4_311-OMcp4_310*ORcp4_211+OPcp4_210*RLcp4_311-OPcp4_310*RLcp4_211;
ACcp4_211 = ACcp4_210-OMcp4_110*ORcp4_311+OMcp4_310*ORcp4_111-OPcp4_110*RLcp4_311+OPcp4_310*RLcp4_111;
ACcp4_311 = ACcp4_310+OMcp4_110*ORcp4_211-OMcp4_210*ORcp4_111+OPcp4_110*RLcp4_211-OPcp4_210*RLcp4_111;
RLcp4_112 = ROcp4_725*dpt[3][43];
RLcp4_212 = ROcp4_825*dpt[3][43];
RLcp4_312 = ROcp4_925*dpt[3][43];
POcp4_112 = POcp4_111+RLcp4_112;
POcp4_212 = POcp4_211+RLcp4_212;
POcp4_312 = POcp4_311+RLcp4_312;
OMcp4_112 = OMcp4_111+ROcp4_127*qd[28];
OMcp4_212 = OMcp4_211+ROcp4_227*qd[28];
OMcp4_312 = OMcp4_311+ROcp4_327*qd[28];
ORcp4_112 = OMcp4_211*RLcp4_312-OMcp4_311*RLcp4_212;
ORcp4_212 = -OMcp4_111*RLcp4_312+OMcp4_311*RLcp4_112;
ORcp4_312 = OMcp4_111*RLcp4_212-OMcp4_211*RLcp4_112;
VIcp4_112 = ORcp4_112+VIcp4_111;
VIcp4_212 = ORcp4_212+VIcp4_211;
VIcp4_312 = ORcp4_312+VIcp4_311;
OPcp4_112 = OPcp4_111+ROcp4_127*qdd[28]+qd[28]*(OMcp4_211*ROcp4_327-OMcp4_311*ROcp4_227);
OPcp4_212 = OPcp4_211+ROcp4_227*qdd[28]+qd[28]*(-OMcp4_111*ROcp4_327+OMcp4_311*ROcp4_127);
OPcp4_312 = OPcp4_311+ROcp4_327*qdd[28]+qd[28]*(OMcp4_111*ROcp4_227-OMcp4_211*ROcp4_127);
ACcp4_112 = ACcp4_111+OMcp4_211*ORcp4_312-OMcp4_311*ORcp4_212+OPcp4_211*RLcp4_312-OPcp4_311*RLcp4_212;
ACcp4_212 = ACcp4_211-OMcp4_111*ORcp4_312+OMcp4_311*ORcp4_112-OPcp4_111*RLcp4_312+OPcp4_311*RLcp4_112;
ACcp4_312 = ACcp4_311+OMcp4_111*ORcp4_212-OMcp4_211*ORcp4_112+OPcp4_111*RLcp4_212-OPcp4_211*RLcp4_112;
RLcp4_113 = ROcp4_428*dpt[2][45]+ROcp4_728*dpt[3][45];
RLcp4_213 = ROcp4_528*dpt[2][45]+ROcp4_828*dpt[3][45];
RLcp4_313 = ROcp4_628*dpt[2][45]+ROcp4_928*dpt[3][45];
POcp4_113 = POcp4_112+RLcp4_113;
POcp4_213 = POcp4_212+RLcp4_213;
POcp4_313 = POcp4_312+RLcp4_313;
OMcp4_113 = OMcp4_112+ROcp4_428*qd[29];
OMcp4_213 = OMcp4_212+ROcp4_528*qd[29];
OMcp4_313 = OMcp4_312+ROcp4_628*qd[29];
ORcp4_113 = OMcp4_212*RLcp4_313-OMcp4_312*RLcp4_213;
ORcp4_213 = -OMcp4_112*RLcp4_313+OMcp4_312*RLcp4_113;
ORcp4_313 = OMcp4_112*RLcp4_213-OMcp4_212*RLcp4_113;
VIcp4_113 = ORcp4_113+VIcp4_112;
VIcp4_213 = ORcp4_213+VIcp4_212;
VIcp4_313 = ORcp4_313+VIcp4_312;
OPcp4_113 = OPcp4_112+ROcp4_428*qdd[29]+qd[29]*(OMcp4_212*ROcp4_628-OMcp4_312*ROcp4_528);
OPcp4_213 = OPcp4_212+ROcp4_528*qdd[29]+qd[29]*(-OMcp4_112*ROcp4_628+OMcp4_312*ROcp4_428);
OPcp4_313 = OPcp4_312+ROcp4_628*qdd[29]+qd[29]*(OMcp4_112*ROcp4_528-OMcp4_212*ROcp4_428);
ACcp4_113 = ACcp4_112+OMcp4_212*ORcp4_313-OMcp4_312*ORcp4_213+OPcp4_212*RLcp4_313-OPcp4_312*RLcp4_213;
ACcp4_213 = ACcp4_212-OMcp4_112*ORcp4_313+OMcp4_312*ORcp4_113-OPcp4_112*RLcp4_313+OPcp4_312*RLcp4_113;
ACcp4_313 = ACcp4_312+OMcp4_112*ORcp4_213-OMcp4_212*ORcp4_113+OPcp4_112*RLcp4_213-OPcp4_212*RLcp4_113;
sens->P[1] = POcp4_113;
sens->P[2] = POcp4_213;
sens->P[3] = POcp4_313;
sens->R[1][1] = ROcp4_129;
sens->R[1][2] = ROcp4_229;
sens->R[1][3] = ROcp4_329;
sens->R[2][1] = ROcp4_428;
sens->R[2][2] = ROcp4_528;
sens->R[2][3] = ROcp4_628;
sens->R[3][1] = ROcp4_729;
sens->R[3][2] = ROcp4_829;
sens->R[3][3] = ROcp4_929;
sens->V[1] = VIcp4_113;
sens->V[2] = VIcp4_213;
sens->V[3] = VIcp4_313;
sens->OM[1] = OMcp4_113;
sens->OM[2] = OMcp4_213;
sens->OM[3] = OMcp4_313;
sens->A[1] = ACcp4_113;
sens->A[2] = ACcp4_213;
sens->A[3] = ACcp4_313;
sens->OMP[1] = OPcp4_113;
sens->OMP[2] = OPcp4_213;
sens->OMP[3] = OPcp4_313;

break;

case 5:

ROcp5_45 = S4*S5;
ROcp5_65 = C4*S5;
ROcp5_75 = S4*C5;
ROcp5_95 = C4*C5;
ROcp5_16 = ROcp5_45*S6+C4*C6;
ROcp5_26 = C5*S6;
ROcp5_36 = ROcp5_65*S6-S4*C6;
ROcp5_46 = ROcp5_45*C6-C4*S6;
ROcp5_56 = C5*C6;
ROcp5_66 = ROcp5_65*C6+S4*S6;
ROcp5_431 = ROcp5_46*C31+ROcp5_75*S31;
ROcp5_531 = ROcp5_56*C31-S31*S5;
ROcp5_631 = ROcp5_66*C31+ROcp5_95*S31;
ROcp5_731 = -ROcp5_46*S31+ROcp5_75*C31;
ROcp5_831 = -ROcp5_56*S31-C31*S5;
ROcp5_931 = -ROcp5_66*S31+ROcp5_95*C31;
ROcp5_432 = ROcp5_431*C32+ROcp5_731*S32;
ROcp5_532 = ROcp5_531*C32+ROcp5_831*S32;
ROcp5_632 = ROcp5_631*C32+ROcp5_931*S32;
ROcp5_732 = -ROcp5_431*S32+ROcp5_731*C32;
ROcp5_832 = -ROcp5_531*S32+ROcp5_831*C32;
ROcp5_932 = -ROcp5_631*S32+ROcp5_931*C32;
ROcp5_134 = ROcp5_16*C34+ROcp5_432*S34;
ROcp5_234 = ROcp5_26*C34+ROcp5_532*S34;
ROcp5_334 = ROcp5_36*C34+ROcp5_632*S34;
ROcp5_434 = -ROcp5_16*S34+ROcp5_432*C34;
ROcp5_534 = -ROcp5_26*S34+ROcp5_532*C34;
ROcp5_634 = -ROcp5_36*S34+ROcp5_632*C34;
ROcp5_435 = ROcp5_434*C35+ROcp5_732*S35;
ROcp5_535 = ROcp5_534*C35+ROcp5_832*S35;
ROcp5_635 = ROcp5_634*C35+ROcp5_932*S35;
ROcp5_735 = -ROcp5_434*S35+ROcp5_732*C35;
ROcp5_835 = -ROcp5_534*S35+ROcp5_832*C35;
ROcp5_935 = -ROcp5_634*S35+ROcp5_932*C35;
ROcp5_136 = ROcp5_134*C36-ROcp5_735*S36;
ROcp5_236 = ROcp5_234*C36-ROcp5_835*S36;
ROcp5_336 = ROcp5_334*C36-ROcp5_935*S36;
ROcp5_736 = ROcp5_134*S36+ROcp5_735*C36;
ROcp5_836 = ROcp5_234*S36+ROcp5_835*C36;
ROcp5_936 = ROcp5_334*S36+ROcp5_935*C36;
OMcp5_15 = qd[5]*C4;
OMcp5_35 = -qd[5]*S4;
OPcp5_15 = qdd[5]*C4-qd[4]*qd[5]*S4;
OPcp5_35 = -qdd[5]*S4-qd[4]*qd[5]*C4;
OMcp5_16 = OMcp5_15+ROcp5_75*qd[6];
OMcp5_26 = qd[4]-qd[6]*S5;
OMcp5_36 = OMcp5_35+ROcp5_95*qd[6];
OPcp5_16 = OPcp5_15+ROcp5_75*qdd[6]+qd[6]*(OMcp5_35*S5+ROcp5_95*qd[4]);
OPcp5_26 = qdd[4]-qdd[6]*S5+qd[6]*(-OMcp5_15*ROcp5_95+OMcp5_35*ROcp5_75);
OPcp5_36 = OPcp5_35+ROcp5_95*qdd[6]+qd[6]*(-OMcp5_15*S5-ROcp5_75*qd[4]);
RLcp5_17 = ROcp5_16*dpt[1][10]+ROcp5_46*dpt[2][10]+ROcp5_75*dpt[3][10];
RLcp5_27 = ROcp5_26*dpt[1][10]+ROcp5_56*dpt[2][10]-dpt[3][10]*S5;
RLcp5_37 = ROcp5_36*dpt[1][10]+ROcp5_66*dpt[2][10]+ROcp5_95*dpt[3][10];
POcp5_17 = RLcp5_17+q[1];
POcp5_27 = RLcp5_27+q[2];
POcp5_37 = RLcp5_37+q[3];
OMcp5_17 = OMcp5_16+ROcp5_16*qd[31];
OMcp5_27 = OMcp5_26+ROcp5_26*qd[31];
OMcp5_37 = OMcp5_36+ROcp5_36*qd[31];
ORcp5_17 = OMcp5_26*RLcp5_37-OMcp5_36*RLcp5_27;
ORcp5_27 = -OMcp5_16*RLcp5_37+OMcp5_36*RLcp5_17;
ORcp5_37 = OMcp5_16*RLcp5_27-OMcp5_26*RLcp5_17;
VIcp5_17 = ORcp5_17+qd[1];
VIcp5_27 = ORcp5_27+qd[2];
VIcp5_37 = ORcp5_37+qd[3];
OPcp5_17 = OPcp5_16+ROcp5_16*qdd[31]+qd[31]*(OMcp5_26*ROcp5_36-OMcp5_36*ROcp5_26);
OPcp5_27 = OPcp5_26+ROcp5_26*qdd[31]+qd[31]*(-OMcp5_16*ROcp5_36+OMcp5_36*ROcp5_16);
OPcp5_37 = OPcp5_36+ROcp5_36*qdd[31]+qd[31]*(OMcp5_16*ROcp5_26-OMcp5_26*ROcp5_16);
ACcp5_17 = qdd[1]+OMcp5_26*ORcp5_37-OMcp5_36*ORcp5_27+OPcp5_26*RLcp5_37-OPcp5_36*RLcp5_27;
ACcp5_27 = qdd[2]-OMcp5_16*ORcp5_37+OMcp5_36*ORcp5_17-OPcp5_16*RLcp5_37+OPcp5_36*RLcp5_17;
ACcp5_37 = qdd[3]+OMcp5_16*ORcp5_27-OMcp5_26*ORcp5_17+OPcp5_16*RLcp5_27-OPcp5_26*RLcp5_17;
RLcp5_18 = ROcp5_431*dpt[2][49];
RLcp5_28 = ROcp5_531*dpt[2][49];
RLcp5_38 = ROcp5_631*dpt[2][49];
POcp5_18 = POcp5_17+RLcp5_18;
POcp5_28 = POcp5_27+RLcp5_28;
POcp5_38 = POcp5_37+RLcp5_38;
OMcp5_18 = OMcp5_17+ROcp5_16*qd[32];
OMcp5_28 = OMcp5_27+ROcp5_26*qd[32];
OMcp5_38 = OMcp5_37+ROcp5_36*qd[32];
ORcp5_18 = OMcp5_27*RLcp5_38-OMcp5_37*RLcp5_28;
ORcp5_28 = -OMcp5_17*RLcp5_38+OMcp5_37*RLcp5_18;
ORcp5_38 = OMcp5_17*RLcp5_28-OMcp5_27*RLcp5_18;
VIcp5_18 = ORcp5_18+VIcp5_17;
VIcp5_28 = ORcp5_28+VIcp5_27;
VIcp5_38 = ORcp5_38+VIcp5_37;
OPcp5_18 = OPcp5_17+ROcp5_16*qdd[32]+qd[32]*(OMcp5_27*ROcp5_36-OMcp5_37*ROcp5_26);
OPcp5_28 = OPcp5_27+ROcp5_26*qdd[32]+qd[32]*(-OMcp5_17*ROcp5_36+OMcp5_37*ROcp5_16);
OPcp5_38 = OPcp5_37+ROcp5_36*qdd[32]+qd[32]*(OMcp5_17*ROcp5_26-OMcp5_27*ROcp5_16);
ACcp5_18 = ACcp5_17+OMcp5_27*ORcp5_38-OMcp5_37*ORcp5_28+OPcp5_27*RLcp5_38-OPcp5_37*RLcp5_28;
ACcp5_28 = ACcp5_27-OMcp5_17*ORcp5_38+OMcp5_37*ORcp5_18-OPcp5_17*RLcp5_38+OPcp5_37*RLcp5_18;
ACcp5_38 = ACcp5_37+OMcp5_17*ORcp5_28-OMcp5_27*ORcp5_18+OPcp5_17*RLcp5_28-OPcp5_27*RLcp5_18;
RLcp5_19 = ROcp5_732*dpt[3][51];
RLcp5_29 = ROcp5_832*dpt[3][51];
RLcp5_39 = ROcp5_932*dpt[3][51];
POcp5_19 = POcp5_18+RLcp5_19;
POcp5_29 = POcp5_28+RLcp5_29;
POcp5_39 = POcp5_38+RLcp5_39;
OMcp5_19 = OMcp5_18+ROcp5_732*qd[34];
OMcp5_29 = OMcp5_28+ROcp5_832*qd[34];
OMcp5_39 = OMcp5_38+ROcp5_932*qd[34];
ORcp5_19 = OMcp5_28*RLcp5_39-OMcp5_38*RLcp5_29;
ORcp5_29 = -OMcp5_18*RLcp5_39+OMcp5_38*RLcp5_19;
ORcp5_39 = OMcp5_18*RLcp5_29-OMcp5_28*RLcp5_19;
VIcp5_19 = ORcp5_19+VIcp5_18;
VIcp5_29 = ORcp5_29+VIcp5_28;
VIcp5_39 = ORcp5_39+VIcp5_38;
OPcp5_19 = OPcp5_18+ROcp5_732*qdd[34]+qd[34]*(OMcp5_28*ROcp5_932-OMcp5_38*ROcp5_832);
OPcp5_29 = OPcp5_28+ROcp5_832*qdd[34]+qd[34]*(-OMcp5_18*ROcp5_932+OMcp5_38*ROcp5_732);
OPcp5_39 = OPcp5_38+ROcp5_932*qdd[34]+qd[34]*(OMcp5_18*ROcp5_832-OMcp5_28*ROcp5_732);
ACcp5_19 = ACcp5_18+OMcp5_28*ORcp5_39-OMcp5_38*ORcp5_29+OPcp5_28*RLcp5_39-OPcp5_38*RLcp5_29;
ACcp5_29 = ACcp5_28-OMcp5_18*ORcp5_39+OMcp5_38*ORcp5_19-OPcp5_18*RLcp5_39+OPcp5_38*RLcp5_19;
ACcp5_39 = ACcp5_38+OMcp5_18*ORcp5_29-OMcp5_28*ORcp5_19+OPcp5_18*RLcp5_29-OPcp5_28*RLcp5_19;
RLcp5_110 = ROcp5_732*dpt[3][54];
RLcp5_210 = ROcp5_832*dpt[3][54];
RLcp5_310 = ROcp5_932*dpt[3][54];
POcp5_110 = POcp5_19+RLcp5_110;
POcp5_210 = POcp5_29+RLcp5_210;
POcp5_310 = POcp5_39+RLcp5_310;
OMcp5_110 = OMcp5_19+ROcp5_134*qd[35];
OMcp5_210 = OMcp5_29+ROcp5_234*qd[35];
OMcp5_310 = OMcp5_39+ROcp5_334*qd[35];
ORcp5_110 = OMcp5_29*RLcp5_310-OMcp5_39*RLcp5_210;
ORcp5_210 = -OMcp5_19*RLcp5_310+OMcp5_39*RLcp5_110;
ORcp5_310 = OMcp5_19*RLcp5_210-OMcp5_29*RLcp5_110;
VIcp5_110 = ORcp5_110+VIcp5_19;
VIcp5_210 = ORcp5_210+VIcp5_29;
VIcp5_310 = ORcp5_310+VIcp5_39;
OPcp5_110 = OPcp5_19+ROcp5_134*qdd[35]+qd[35]*(OMcp5_29*ROcp5_334-OMcp5_39*ROcp5_234);
OPcp5_210 = OPcp5_29+ROcp5_234*qdd[35]+qd[35]*(-OMcp5_19*ROcp5_334+OMcp5_39*ROcp5_134);
OPcp5_310 = OPcp5_39+ROcp5_334*qdd[35]+qd[35]*(OMcp5_19*ROcp5_234-OMcp5_29*ROcp5_134);
ACcp5_110 = ACcp5_19+OMcp5_29*ORcp5_310-OMcp5_39*ORcp5_210+OPcp5_29*RLcp5_310-OPcp5_39*RLcp5_210;
ACcp5_210 = ACcp5_29-OMcp5_19*ORcp5_310+OMcp5_39*ORcp5_110-OPcp5_19*RLcp5_310+OPcp5_39*RLcp5_110;
ACcp5_310 = ACcp5_39+OMcp5_19*ORcp5_210-OMcp5_29*ORcp5_110+OPcp5_19*RLcp5_210-OPcp5_29*RLcp5_110;
RLcp5_111 = ROcp5_435*dpt[2][56]+ROcp5_735*dpt[3][56];
RLcp5_211 = ROcp5_535*dpt[2][56]+ROcp5_835*dpt[3][56];
RLcp5_311 = ROcp5_635*dpt[2][56]+ROcp5_935*dpt[3][56];
POcp5_111 = POcp5_110+RLcp5_111;
POcp5_211 = POcp5_210+RLcp5_211;
POcp5_311 = POcp5_310+RLcp5_311;
OMcp5_111 = OMcp5_110+ROcp5_435*qd[36];
OMcp5_211 = OMcp5_210+ROcp5_535*qd[36];
OMcp5_311 = OMcp5_310+ROcp5_635*qd[36];
ORcp5_111 = OMcp5_210*RLcp5_311-OMcp5_310*RLcp5_211;
ORcp5_211 = -OMcp5_110*RLcp5_311+OMcp5_310*RLcp5_111;
ORcp5_311 = OMcp5_110*RLcp5_211-OMcp5_210*RLcp5_111;
VIcp5_111 = ORcp5_111+VIcp5_110;
VIcp5_211 = ORcp5_211+VIcp5_210;
VIcp5_311 = ORcp5_311+VIcp5_310;
OPcp5_111 = OPcp5_110+ROcp5_435*qdd[36]+qd[36]*(OMcp5_210*ROcp5_635-OMcp5_310*ROcp5_535);
OPcp5_211 = OPcp5_210+ROcp5_535*qdd[36]+qd[36]*(-OMcp5_110*ROcp5_635+OMcp5_310*ROcp5_435);
OPcp5_311 = OPcp5_310+ROcp5_635*qdd[36]+qd[36]*(OMcp5_110*ROcp5_535-OMcp5_210*ROcp5_435);
ACcp5_111 = ACcp5_110+OMcp5_210*ORcp5_311-OMcp5_310*ORcp5_211+OPcp5_210*RLcp5_311-OPcp5_310*RLcp5_211;
ACcp5_211 = ACcp5_210-OMcp5_110*ORcp5_311+OMcp5_310*ORcp5_111-OPcp5_110*RLcp5_311+OPcp5_310*RLcp5_111;
ACcp5_311 = ACcp5_310+OMcp5_110*ORcp5_211-OMcp5_210*ORcp5_111+OPcp5_110*RLcp5_211-OPcp5_210*RLcp5_111;
sens->P[1] = POcp5_111;
sens->P[2] = POcp5_211;
sens->P[3] = POcp5_311;
sens->R[1][1] = ROcp5_136;
sens->R[1][2] = ROcp5_236;
sens->R[1][3] = ROcp5_336;
sens->R[2][1] = ROcp5_435;
sens->R[2][2] = ROcp5_535;
sens->R[2][3] = ROcp5_635;
sens->R[3][1] = ROcp5_736;
sens->R[3][2] = ROcp5_836;
sens->R[3][3] = ROcp5_936;
sens->V[1] = VIcp5_111;
sens->V[2] = VIcp5_211;
sens->V[3] = VIcp5_311;
sens->OM[1] = OMcp5_111;
sens->OM[2] = OMcp5_211;
sens->OM[3] = OMcp5_311;
sens->A[1] = ACcp5_111;
sens->A[2] = ACcp5_211;
sens->A[3] = ACcp5_311;
sens->OMP[1] = OPcp5_111;
sens->OMP[2] = OPcp5_211;
sens->OMP[3] = OPcp5_311;

break;

default:

break;

}


// Number of continuation lines = 1

}
