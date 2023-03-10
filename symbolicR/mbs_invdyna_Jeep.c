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
//	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
//
//	==> Git hash: 8f46effdc05c898000a15b4c4dfc8f70efce4fc0
//

#include <math.h> 

#include "mbs_data.h"

 
void mbs_invdyna(double *Qq,
MbsData *s, double tsim)
{
#include "mbs_invdyna_Jeep.h"

double *q, *qd, *qdd;
double *g, *m;
double **l, **In, **dpt, **frc, **trq;

q = s->q;
qd = s->qd;
qdd = s->qdd;

dpt = s->dpt;
l   = s->l;

m = s->m;
In  = s->In;

frc = s->frc;
trq = s->trq;
g = s->g;
 
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
S9 = sin(q[9]);
C9 = cos(q[9]);
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
S18 = sin(q[18]);
C18 = cos(q[18]);
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
S26 = sin(q[26]);
C26 = cos(q[26]);
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
S33 = sin(q[33]);
C33 = cos(q[33]);
S34 = sin(q[34]);
C34 = cos(q[34]);
S35 = sin(q[35]);
C35 = cos(q[35]);
S36 = sin(q[36]);
C36 = cos(q[36]);
 
// Augmented Joint Position Vectors

Dz413 = q[41]+dpt[3][17];
 
// Augmented Joint Position Vectors

 
// Forward Kinematics

ALPHA33 = qdd[3]-g[3];
ALPHA14 = qdd[1]*C4-ALPHA33*S4;
ALPHA34 = qdd[1]*S4+ALPHA33*C4;
OM25 = qd[4]*C5;
OM35 = -qd[4]*S5;
OMp25 = -qd[4]*qd[5]*S5+qdd[4]*C5;
OMp35 = -qd[4]*qd[5]*C5-qdd[4]*S5;
ALPHA25 = qdd[2]*C5+ALPHA34*S5;
ALPHA35 = -qdd[2]*S5+ALPHA34*C5;
OM16 = qd[5]*C6+OM25*S6;
OM26 = -qd[5]*S6+OM25*C6;
OM36 = qd[6]+OM35;
OMp16 = C6*(qdd[5]+qd[6]*OM25)+S6*(OMp25-qd[5]*qd[6]);
OMp26 = C6*(OMp25-qd[5]*qd[6])-S6*(qdd[5]+qd[6]*OM25);
OMp36 = qdd[6]+OMp35;
BS16 = -OM26*OM26-OM36*OM36;
BS26 = OM16*OM26;
BS36 = OM16*OM36;
BS56 = -OM16*OM16-OM36*OM36;
BS66 = OM26*OM36;
BS96 = -OM16*OM16-OM26*OM26;
BETA26 = BS26-OMp36;
BETA36 = BS36+OMp26;
BETA46 = BS26+OMp36;
BETA66 = BS66-OMp16;
BETA76 = BS36-OMp26;
BETA86 = BS66+OMp16;
ALPHA16 = ALPHA14*C6+ALPHA25*S6;
ALPHA26 = -ALPHA14*S6+ALPHA25*C6;
OM17 = qd[7]+OM16;
OM27 = OM26*C7+OM36*S7;
OM37 = -OM26*S7+OM36*C7;
OMp17 = qdd[7]+OMp16;
OMp27 = C7*(OMp26+qd[7]*OM36)+S7*(OMp36-qd[7]*OM26);
OMp37 = C7*(OMp36-qd[7]*OM26)-S7*(OMp26+qd[7]*OM36);
BS27 = OM17*OM27;
BS57 = -OM17*OM17-OM37*OM37;
BS67 = OM27*OM37;
BETA27 = BS27-OMp37;
BETA87 = BS67+OMp17;
ALPHA17 = ALPHA16+BETA26*dpt[2][2]+BETA36*dpt[3][2]+BS16*dpt[1][2];
ALPHA27 = C7*(ALPHA26+BETA46*dpt[1][2]+BETA66*dpt[3][2]+BS56*dpt[2][2])+S7*(ALPHA35+BETA76*dpt[1][2]+BETA86*dpt[2][2]+
 BS96*dpt[3][2]);
ALPHA37 = C7*(ALPHA35+BETA76*dpt[1][2]+BETA86*dpt[2][2]+BS96*dpt[3][2])-S7*(ALPHA26+BETA46*dpt[1][2]+BETA66*dpt[3][2]+
 BS56*dpt[2][2]);
OM18 = qd[8]+OM17;
OM28 = OM27*C8+OM37*S8;
OM38 = -OM27*S8+OM37*C8;
OMp18 = qdd[8]+OMp17;
OMp28 = C8*(OMp27+qd[8]*OM37)+S8*(OMp37-qd[8]*OM27);
OMp38 = C8*(OMp37-qd[8]*OM27)-S8*(OMp27+qd[8]*OM37);
BS38 = OM18*OM38;
BS68 = OM28*OM38;
BS98 = -OM18*OM18-OM28*OM28;
BETA38 = BS38+OMp28;
BETA68 = BS68-OMp18;
ALPHA18 = ALPHA17+BETA27*dpt[2][18];
ALPHA28 = C8*(ALPHA27+BS57*dpt[2][18])+S8*(ALPHA37+BETA87*dpt[2][18]);
ALPHA38 = C8*(ALPHA37+BETA87*dpt[2][18])-S8*(ALPHA27+BS57*dpt[2][18]);
OM19 = qd[9]+OM18;
OM29 = OM28*C9+OM38*S9;
OM39 = -OM28*S9+OM38*C9;
OMp19 = qdd[9]+OMp18;
OMp29 = C9*(OMp28+qd[9]*OM38)+S9*(OMp38-qd[9]*OM28);
OMp39 = C9*(OMp38-qd[9]*OM28)-S9*(OMp28+qd[9]*OM38);
BS29 = OM19*OM29;
BS59 = -OM19*OM19-OM39*OM39;
BS69 = OM29*OM39;
BETA29 = BS29-OMp39;
BETA89 = BS69+OMp19;
ALPHA19 = ALPHA18+BETA38*dpt[3][19];
ALPHA29 = C9*(ALPHA28+BETA68*dpt[3][19])+S9*(ALPHA38+BS98*dpt[3][19]);
ALPHA39 = C9*(ALPHA38+BS98*dpt[3][19])-S9*(ALPHA28+BETA68*dpt[3][19]);
OM110 = OM18*C10+OM28*S10;
OM210 = -OM18*S10+OM28*C10;
OM310 = qd[10]+OM38;
OMp110 = C10*(OMp18+qd[10]*OM28)+S10*(OMp28-qd[10]*OM18);
OMp210 = C10*(OMp28-qd[10]*OM18)-S10*(OMp18+qd[10]*OM28);
OMp310 = qdd[10]+OMp38;
BS310 = OM110*OM310;
BS610 = OM210*OM310;
BS910 = -OM110*OM110-OM210*OM210;
BETA310 = BS310+OMp210;
BETA610 = BS610-OMp110;
ALPHA110 = C10*(ALPHA18+BETA38*dpt[3][20])+S10*(ALPHA28+BETA68*dpt[3][20]);
ALPHA210 = C10*(ALPHA28+BETA68*dpt[3][20])-S10*(ALPHA18+BETA38*dpt[3][20]);
ALPHA310 = ALPHA38+BS98*dpt[3][20];
OM111 = qd[11]+OM110;
OM211 = OM210*C11+OM310*S11;
OM311 = -OM210*S11+OM310*C11;
OMp111 = qdd[11]+OMp110;
OMp211 = C11*(OMp210+qd[11]*OM310)+S11*(OMp310-qd[11]*OM210);
OMp311 = C11*(OMp310-qd[11]*OM210)-S11*(OMp210+qd[11]*OM310);
BS211 = OM111*OM211;
BS311 = OM111*OM311;
BS511 = -OM111*OM111-OM311*OM311;
BS611 = OM211*OM311;
BS911 = -OM111*OM111-OM211*OM211;
BETA211 = BS211-OMp311;
BETA311 = BS311+OMp211;
BETA611 = BS611-OMp111;
BETA811 = BS611+OMp111;
ALPHA111 = ALPHA110+BETA310*dpt[3][23];
ALPHA211 = C11*(ALPHA210+BETA610*dpt[3][23])+S11*(ALPHA310+BS910*dpt[3][23]);
ALPHA311 = C11*(ALPHA310+BS910*dpt[3][23])-S11*(ALPHA210+BETA610*dpt[3][23]);
OM112 = OM111*C12-OM311*S12;
OM212 = qd[12]+OM211;
OM312 = OM111*S12+OM311*C12;
OMp112 = C12*(OMp111-qd[12]*OM311)-S12*(OMp311+qd[12]*OM111);
OMp212 = qdd[12]+OMp211;
OMp312 = C12*(OMp311+qd[12]*OM111)+S12*(OMp111-qd[12]*OM311);
ALPHA112 = C12*(ALPHA111+BETA211*dpt[2][25]+BETA311*dpt[3][25])-S12*(ALPHA311+BETA811*dpt[2][25]+BS911*dpt[3][25]);
ALPHA212 = ALPHA211+BETA611*dpt[3][25]+BS511*dpt[2][25];
ALPHA312 = C12*(ALPHA311+BETA811*dpt[2][25]+BS911*dpt[3][25])+S12*(ALPHA111+BETA211*dpt[2][25]+BETA311*dpt[3][25]);
OM114 = qd[14]+OM16;
OM214 = OM26*C14+OM36*S14;
OM314 = -OM26*S14+OM36*C14;
OMp114 = qdd[14]+OMp16;
OMp214 = C14*(OMp26+qd[14]*OM36)+S14*(OMp36-qd[14]*OM26);
OMp314 = C14*(OMp36-qd[14]*OM26)-S14*(OMp26+qd[14]*OM36);
BS214 = OM114*OM214;
BS514 = -OM114*OM114-OM314*OM314;
BS614 = OM214*OM314;
BETA214 = BS214-OMp314;
BETA814 = BS614+OMp114;
ALPHA114 = ALPHA16+BETA26*dpt[2][4]+BETA36*dpt[3][4]+BS16*dpt[1][4];
ALPHA214 = C14*(ALPHA26+BETA46*dpt[1][4]+BETA66*dpt[3][4]+BS56*dpt[2][4])+S14*(ALPHA35+BETA76*dpt[1][4]+BETA86*
 dpt[2][4]+BS96*dpt[3][4]);
ALPHA314 = C14*(ALPHA35+BETA76*dpt[1][4]+BETA86*dpt[2][4]+BS96*dpt[3][4])-S14*(ALPHA26+BETA46*dpt[1][4]+BETA66*
 dpt[3][4]+BS56*dpt[2][4]);
BS315 = OM114*OM314;
BS615 = OM214*OM314;
BS915 = -OM114*OM114-OM214*OM214;
BETA315 = BS315+OMp214;
BETA615 = BS615-OMp114;
ALPHA115 = ALPHA114+q[15]*BETA214-(2.0)*qd[15]*OM314;
ALPHA215 = qdd[15]+ALPHA214+q[15]*BS514;
ALPHA315 = ALPHA314+q[15]*BETA814+(2.0)*qd[15]*OM114;
BS216 = OM114*OM214;
BS516 = -OM114*OM114-OM314*OM314;
BS616 = OM214*OM314;
BETA216 = BS216-OMp314;
BETA816 = BS616+OMp114;
ALPHA116 = ALPHA115+q[16]*BETA315+(2.0)*qd[16]*OM214;
ALPHA216 = ALPHA215+q[16]*BETA615-(2.0)*qd[16]*OM114;
ALPHA316 = qdd[16]+ALPHA315+q[16]*BS915;
OM117 = qd[17]+OM114;
OM217 = OM214*C17+OM314*S17;
OM317 = -OM214*S17+OM314*C17;
OMp117 = qdd[17]+OMp114;
OMp217 = C17*(OMp214+qd[17]*OM314)+S17*(OMp314-qd[17]*OM214);
OMp317 = C17*(OMp314-qd[17]*OM214)-S17*(OMp214+qd[17]*OM314);
BS317 = OM117*OM317;
BS617 = OM217*OM317;
BS917 = -OM117*OM117-OM217*OM217;
BETA317 = BS317+OMp217;
BETA617 = BS617-OMp117;
ALPHA117 = ALPHA116+BETA216*dpt[2][29];
ALPHA217 = C17*(ALPHA216+BS516*dpt[2][29])+S17*(ALPHA316+BETA816*dpt[2][29]);
ALPHA317 = C17*(ALPHA316+BETA816*dpt[2][29])-S17*(ALPHA216+BS516*dpt[2][29]);
OM118 = qd[18]+OM117;
OM218 = OM217*C18+OM317*S18;
OM318 = -OM217*S18+OM317*C18;
OMp118 = qdd[18]+OMp117;
OMp218 = C18*(OMp217+qd[18]*OM317)+S18*(OMp317-qd[18]*OM217);
OMp318 = C18*(OMp317-qd[18]*OM217)-S18*(OMp217+qd[18]*OM317);
BS218 = OM118*OM218;
BS518 = -OM118*OM118-OM318*OM318;
BS618 = OM218*OM318;
BETA218 = BS218-OMp318;
BETA818 = BS618+OMp118;
ALPHA118 = ALPHA117+BETA317*dpt[3][30];
ALPHA218 = C18*(ALPHA217+BETA617*dpt[3][30])+S18*(ALPHA317+BS917*dpt[3][30]);
ALPHA318 = C18*(ALPHA317+BS917*dpt[3][30])-S18*(ALPHA217+BETA617*dpt[3][30]);
OM119 = OM117*C19+OM217*S19;
OM219 = -OM117*S19+OM217*C19;
OM319 = qd[19]+OM317;
OMp119 = C19*(OMp117+qd[19]*OM217)+S19*(OMp217-qd[19]*OM117);
OMp219 = C19*(OMp217-qd[19]*OM117)-S19*(OMp117+qd[19]*OM217);
OMp319 = qdd[19]+OMp317;
BS319 = OM119*OM319;
BS619 = OM219*OM319;
BS919 = -OM119*OM119-OM219*OM219;
BETA319 = BS319+OMp219;
BETA619 = BS619-OMp119;
ALPHA119 = C19*(ALPHA117+BETA317*dpt[3][31])+S19*(ALPHA217+BETA617*dpt[3][31]);
ALPHA219 = C19*(ALPHA217+BETA617*dpt[3][31])-S19*(ALPHA117+BETA317*dpt[3][31]);
ALPHA319 = ALPHA317+BS917*dpt[3][31];
OM120 = qd[20]+OM119;
OM220 = OM219*C20+OM319*S20;
OM320 = -OM219*S20+OM319*C20;
OMp120 = qdd[20]+OMp119;
OMp220 = C20*(OMp219+qd[20]*OM319)+S20*(OMp319-qd[20]*OM219);
OMp320 = C20*(OMp319-qd[20]*OM219)-S20*(OMp219+qd[20]*OM319);
BS220 = OM120*OM220;
BS320 = OM120*OM320;
BS520 = -OM120*OM120-OM320*OM320;
BS620 = OM220*OM320;
BS920 = -OM120*OM120-OM220*OM220;
BETA220 = BS220-OMp320;
BETA320 = BS320+OMp220;
BETA620 = BS620-OMp120;
BETA820 = BS620+OMp120;
ALPHA120 = ALPHA119+BETA319*dpt[3][34];
ALPHA220 = C20*(ALPHA219+BETA619*dpt[3][34])+S20*(ALPHA319+BS919*dpt[3][34]);
ALPHA320 = C20*(ALPHA319+BS919*dpt[3][34])-S20*(ALPHA219+BETA619*dpt[3][34]);
OM121 = OM120*C21-OM320*S21;
OM221 = qd[21]+OM220;
OM321 = OM120*S21+OM320*C21;
OMp121 = C21*(OMp120-qd[21]*OM320)-S21*(OMp320+qd[21]*OM120);
OMp221 = qdd[21]+OMp220;
OMp321 = C21*(OMp320+qd[21]*OM120)+S21*(OMp120-qd[21]*OM320);
ALPHA121 = C21*(ALPHA120+BETA220*dpt[2][36]+BETA320*dpt[3][36])-S21*(ALPHA320+BETA820*dpt[2][36]+BS920*dpt[3][36]);
ALPHA221 = ALPHA220+BETA620*dpt[3][36]+BS520*dpt[2][36];
ALPHA321 = C21*(ALPHA320+BETA820*dpt[2][36]+BS920*dpt[3][36])+S21*(ALPHA120+BETA220*dpt[2][36]+BETA320*dpt[3][36]);
OM122 = qd[22]+OM16;
OM222 = OM26*C22+OM36*S22;
OM322 = -OM26*S22+OM36*C22;
OMp122 = qdd[22]+OMp16;
OMp222 = C22*(OMp26+qd[22]*OM36)+S22*(OMp36-qd[22]*OM26);
OMp322 = C22*(OMp36-qd[22]*OM26)-S22*(OMp26+qd[22]*OM36);
BS222 = OM122*OM222;
BS522 = -OM122*OM122-OM322*OM322;
BS622 = OM222*OM322;
BETA222 = BS222-OMp322;
BETA822 = BS622+OMp122;
ALPHA122 = ALPHA16+BETA26*dpt[2][7]+BETA36*dpt[3][7]+BS16*dpt[1][7];
ALPHA222 = C22*(ALPHA26+BETA46*dpt[1][7]+BETA66*dpt[3][7]+BS56*dpt[2][7])+S22*(ALPHA35+BETA76*dpt[1][7]+BETA86*
 dpt[2][7]+BS96*dpt[3][7]);
ALPHA322 = C22*(ALPHA35+BETA76*dpt[1][7]+BETA86*dpt[2][7]+BS96*dpt[3][7])-S22*(ALPHA26+BETA46*dpt[1][7]+BETA66*
 dpt[3][7]+BS56*dpt[2][7]);
BS323 = OM122*OM322;
BS623 = OM222*OM322;
BS923 = -OM122*OM122-OM222*OM222;
BETA323 = BS323+OMp222;
BETA623 = BS623-OMp122;
ALPHA123 = ALPHA122+q[23]*BETA222-(2.0)*qd[23]*OM322;
ALPHA223 = qdd[23]+ALPHA222+q[23]*BS522;
ALPHA323 = ALPHA322+q[23]*BETA822+(2.0)*qd[23]*OM122;
BS224 = OM122*OM222;
BS524 = -OM122*OM122-OM322*OM322;
BS624 = OM222*OM322;
BETA224 = BS224-OMp322;
BETA824 = BS624+OMp122;
ALPHA124 = ALPHA123+q[24]*BETA323+(2.0)*qd[24]*OM222;
ALPHA224 = ALPHA223+q[24]*BETA623-(2.0)*qd[24]*OM122;
ALPHA324 = qdd[24]+ALPHA323+q[24]*BS923;
OM125 = qd[25]+OM122;
OM225 = OM222*C25+OM322*S25;
OM325 = -OM222*S25+OM322*C25;
OMp125 = qdd[25]+OMp122;
OMp225 = C25*(OMp222+qd[25]*OM322)+S25*(OMp322-qd[25]*OM222);
OMp325 = C25*(OMp322-qd[25]*OM222)-S25*(OMp222+qd[25]*OM322);
BS325 = OM125*OM325;
BS625 = OM225*OM325;
BS925 = -OM125*OM125-OM225*OM225;
BETA325 = BS325+OMp225;
BETA625 = BS625-OMp125;
ALPHA125 = ALPHA124+BETA224*dpt[2][38];
ALPHA225 = C25*(ALPHA224+BS524*dpt[2][38])+S25*(ALPHA324+BETA824*dpt[2][38]);
ALPHA325 = C25*(ALPHA324+BETA824*dpt[2][38])-S25*(ALPHA224+BS524*dpt[2][38]);
OM126 = qd[26]+OM125;
OM226 = OM225*C26+OM325*S26;
OM326 = -OM225*S26+OM325*C26;
OMp126 = qdd[26]+OMp125;
OMp226 = C26*(OMp225+qd[26]*OM325)+S26*(OMp325-qd[26]*OM225);
OMp326 = C26*(OMp325-qd[26]*OM225)-S26*(OMp225+qd[26]*OM325);
BS226 = OM126*OM226;
BS526 = -OM126*OM126-OM326*OM326;
BS626 = OM226*OM326;
BETA226 = BS226-OMp326;
BETA826 = BS626+OMp126;
ALPHA126 = ALPHA125+BETA325*dpt[3][39];
ALPHA226 = C26*(ALPHA225+BETA625*dpt[3][39])+S26*(ALPHA325+BS925*dpt[3][39]);
ALPHA326 = C26*(ALPHA325+BS925*dpt[3][39])-S26*(ALPHA225+BETA625*dpt[3][39]);
OM127 = OM125*C27+OM225*S27;
OM227 = -OM125*S27+OM225*C27;
OM327 = qd[27]+OM325;
OMp127 = C27*(OMp125+qd[27]*OM225)+S27*(OMp225-qd[27]*OM125);
OMp227 = C27*(OMp225-qd[27]*OM125)-S27*(OMp125+qd[27]*OM225);
OMp327 = qdd[27]+OMp325;
BS327 = OM127*OM327;
BS627 = OM227*OM327;
BS927 = -OM127*OM127-OM227*OM227;
BETA327 = BS327+OMp227;
BETA627 = BS627-OMp127;
ALPHA127 = C27*(ALPHA125+BETA325*dpt[3][40])+S27*(ALPHA225+BETA625*dpt[3][40]);
ALPHA227 = C27*(ALPHA225+BETA625*dpt[3][40])-S27*(ALPHA125+BETA325*dpt[3][40]);
ALPHA327 = ALPHA325+BS925*dpt[3][40];
OM128 = qd[28]+OM127;
OM228 = OM227*C28+OM327*S28;
OM328 = -OM227*S28+OM327*C28;
OMp128 = qdd[28]+OMp127;
OMp228 = C28*(OMp227+qd[28]*OM327)+S28*(OMp327-qd[28]*OM227);
OMp328 = C28*(OMp327-qd[28]*OM227)-S28*(OMp227+qd[28]*OM327);
BS228 = OM128*OM228;
BS328 = OM128*OM328;
BS528 = -OM128*OM128-OM328*OM328;
BS628 = OM228*OM328;
BS928 = -OM128*OM128-OM228*OM228;
BETA228 = BS228-OMp328;
BETA328 = BS328+OMp228;
BETA628 = BS628-OMp128;
BETA828 = BS628+OMp128;
ALPHA128 = ALPHA127+BETA327*dpt[3][43];
ALPHA228 = C28*(ALPHA227+BETA627*dpt[3][43])+S28*(ALPHA327+BS927*dpt[3][43]);
ALPHA328 = C28*(ALPHA327+BS927*dpt[3][43])-S28*(ALPHA227+BETA627*dpt[3][43]);
OM129 = OM128*C29-OM328*S29;
OM229 = qd[29]+OM228;
OM329 = OM128*S29+OM328*C29;
OMp129 = C29*(OMp128-qd[29]*OM328)-S29*(OMp328+qd[29]*OM128);
OMp229 = qdd[29]+OMp228;
OMp329 = C29*(OMp328+qd[29]*OM128)+S29*(OMp128-qd[29]*OM328);
ALPHA129 = C29*(ALPHA128+BETA228*dpt[2][45]+BETA328*dpt[3][45])-S29*(ALPHA328+BETA828*dpt[2][45]+BS928*dpt[3][45]);
ALPHA229 = ALPHA228+BETA628*dpt[3][45]+BS528*dpt[2][45];
ALPHA329 = C29*(ALPHA328+BETA828*dpt[2][45]+BS928*dpt[3][45])+S29*(ALPHA128+BETA228*dpt[2][45]+BETA328*dpt[3][45]);
OM131 = qd[31]+OM16;
OM231 = OM26*C31+OM36*S31;
OM331 = -OM26*S31+OM36*C31;
OMp131 = qdd[31]+OMp16;
OMp231 = C31*(OMp26+qd[31]*OM36)+S31*(OMp36-qd[31]*OM26);
OMp331 = C31*(OMp36-qd[31]*OM26)-S31*(OMp26+qd[31]*OM36);
BS231 = OM131*OM231;
BS531 = -OM131*OM131-OM331*OM331;
BS631 = OM231*OM331;
BETA231 = BS231-OMp331;
BETA831 = BS631+OMp131;
ALPHA131 = ALPHA16+BETA26*dpt[2][10]+BETA36*dpt[3][10]+BS16*dpt[1][10];
ALPHA231 = C31*(ALPHA26+BETA46*dpt[1][10]+BETA66*dpt[3][10]+BS56*dpt[2][10])+S31*(ALPHA35+BETA76*dpt[1][10]+BETA86*
 dpt[2][10]+BS96*dpt[3][10]);
ALPHA331 = C31*(ALPHA35+BETA76*dpt[1][10]+BETA86*dpt[2][10]+BS96*dpt[3][10])-S31*(ALPHA26+BETA46*dpt[1][10]+BETA66*
 dpt[3][10]+BS56*dpt[2][10]);
OM132 = qd[32]+OM131;
OM232 = OM231*C32+OM331*S32;
OM332 = -OM231*S32+OM331*C32;
OMp132 = qdd[32]+OMp131;
OMp232 = C32*(OMp231+qd[32]*OM331)+S32*(OMp331-qd[32]*OM231);
OMp332 = C32*(OMp331-qd[32]*OM231)-S32*(OMp231+qd[32]*OM331);
BS332 = OM132*OM332;
BS632 = OM232*OM332;
BS932 = -OM132*OM132-OM232*OM232;
BETA332 = BS332+OMp232;
BETA632 = BS632-OMp132;
ALPHA132 = ALPHA131+BETA231*dpt[2][49];
ALPHA232 = C32*(ALPHA231+BS531*dpt[2][49])+S32*(ALPHA331+BETA831*dpt[2][49]);
ALPHA332 = C32*(ALPHA331+BETA831*dpt[2][49])-S32*(ALPHA231+BS531*dpt[2][49]);
OM133 = qd[33]+OM132;
OM233 = OM232*C33+OM332*S33;
OM333 = -OM232*S33+OM332*C33;
OMp133 = qdd[33]+OMp132;
OMp233 = C33*(OMp232+qd[33]*OM332)+S33*(OMp332-qd[33]*OM232);
OMp333 = C33*(OMp332-qd[33]*OM232)-S33*(OMp232+qd[33]*OM332);
BS233 = OM133*OM233;
BS533 = -OM133*OM133-OM333*OM333;
BS633 = OM233*OM333;
BETA233 = BS233-OMp333;
BETA833 = BS633+OMp133;
ALPHA133 = ALPHA132+BETA332*dpt[3][50];
ALPHA233 = C33*(ALPHA232+BETA632*dpt[3][50])+S33*(ALPHA332+BS932*dpt[3][50]);
ALPHA333 = C33*(ALPHA332+BS932*dpt[3][50])-S33*(ALPHA232+BETA632*dpt[3][50]);
OM134 = OM132*C34+OM232*S34;
OM234 = -OM132*S34+OM232*C34;
OM334 = qd[34]+OM332;
OMp134 = C34*(OMp132+qd[34]*OM232)+S34*(OMp232-qd[34]*OM132);
OMp234 = C34*(OMp232-qd[34]*OM132)-S34*(OMp132+qd[34]*OM232);
OMp334 = qdd[34]+OMp332;
BS334 = OM134*OM334;
BS634 = OM234*OM334;
BS934 = -OM134*OM134-OM234*OM234;
BETA334 = BS334+OMp234;
BETA634 = BS634-OMp134;
ALPHA134 = C34*(ALPHA132+BETA332*dpt[3][51])+S34*(ALPHA232+BETA632*dpt[3][51]);
ALPHA234 = C34*(ALPHA232+BETA632*dpt[3][51])-S34*(ALPHA132+BETA332*dpt[3][51]);
ALPHA334 = ALPHA332+BS932*dpt[3][51];
OM135 = qd[35]+OM134;
OM235 = OM234*C35+OM334*S35;
OM335 = -OM234*S35+OM334*C35;
OMp135 = qdd[35]+OMp134;
OMp235 = C35*(OMp234+qd[35]*OM334)+S35*(OMp334-qd[35]*OM234);
OMp335 = C35*(OMp334-qd[35]*OM234)-S35*(OMp234+qd[35]*OM334);
BS235 = OM135*OM235;
BS335 = OM135*OM335;
BS535 = -OM135*OM135-OM335*OM335;
BS635 = OM235*OM335;
BS935 = -OM135*OM135-OM235*OM235;
BETA235 = BS235-OMp335;
BETA335 = BS335+OMp235;
BETA635 = BS635-OMp135;
BETA835 = BS635+OMp135;
ALPHA135 = ALPHA134+BETA334*dpt[3][54];
ALPHA235 = C35*(ALPHA234+BETA634*dpt[3][54])+S35*(ALPHA334+BS934*dpt[3][54]);
ALPHA335 = C35*(ALPHA334+BS934*dpt[3][54])-S35*(ALPHA234+BETA634*dpt[3][54]);
OM136 = OM135*C36-OM335*S36;
OM236 = qd[36]+OM235;
OM336 = OM135*S36+OM335*C36;
OMp136 = C36*(OMp135-qd[36]*OM335)-S36*(OMp335+qd[36]*OM135);
OMp236 = qdd[36]+OMp235;
OMp336 = C36*(OMp335+qd[36]*OM135)+S36*(OMp135-qd[36]*OM335);
ALPHA136 = C36*(ALPHA135+BETA235*dpt[2][56]+BETA335*dpt[3][56])-S36*(ALPHA335+BETA835*dpt[2][56]+BS935*dpt[3][56]);
ALPHA236 = ALPHA235+BETA635*dpt[3][56]+BS535*dpt[2][56];
ALPHA336 = C36*(ALPHA335+BETA835*dpt[2][56]+BS935*dpt[3][56])+S36*(ALPHA135+BETA235*dpt[2][56]+BETA335*dpt[3][56]);
ALPHA141 = ALPHA16+(2.0)*qd[41]*OM26+BETA26*dpt[2][17]+BETA36*Dz413;
ALPHA241 = ALPHA26-(2.0)*qd[41]*OM16+BETA66*Dz413+BS56*dpt[2][17];
ALPHA341 = qdd[41]+ALPHA35+BETA86*dpt[2][17]+BS96*Dz413;
 
// Backward Dynamics

Fs141 = -frc[1][41]+m[41]*ALPHA141;
Fs241 = -frc[2][41]+m[41]*ALPHA241;
Fs341 = -frc[3][41]+m[41]*ALPHA341;
Cq141 = -trq[1][41]+In[1][41]*OMp16-In[5][41]*OM26*OM36+In[9][41]*OM26*OM36;
Cq241 = -trq[2][41]+In[1][41]*OM16*OM36+In[5][41]*OMp26-In[9][41]*OM16*OM36;
Cq341 = -trq[3][41]-In[1][41]*OM16*OM26+In[5][41]*OM16*OM26+In[9][41]*OMp36;
Fs136 = -frc[1][36]+m[36]*ALPHA136;
Fs236 = -frc[2][36]+m[36]*ALPHA236;
Fs336 = -frc[3][36]+m[36]*ALPHA336;
Cq136 = -trq[1][36]+In[1][36]*OMp136-In[5][36]*OM236*OM336+In[9][36]*OM236*OM336;
Cq236 = -trq[2][36]+In[1][36]*OM136*OM336+In[5][36]*OMp236-In[9][36]*OM136*OM336;
Cq336 = -trq[3][36]-In[1][36]*OM136*OM236+In[5][36]*OM136*OM236+In[9][36]*OMp336;
Fq135 = -frc[1][35]+Fs136*C36+Fs336*S36;
Fq235 = -frc[2][35]+Fs236;
Fq335 = -frc[3][35]-Fs136*S36+Fs336*C36;
Cq135 = -trq[1][35]+Cq136*C36+Cq336*S36-Fs236*dpt[3][56]+dpt[2][56]*(-Fs136*S36+Fs336*C36);
Cq235 = -trq[2][35]+Cq236+dpt[3][56]*(Fs136*C36+Fs336*S36);
Cq335 = -trq[3][35]-Cq136*S36+Cq336*C36-dpt[2][56]*(Fs136*C36+Fs336*S36);
Fq134 = -frc[1][34]+Fq135;
Fq234 = -frc[2][34]+Fq235*C35-Fq335*S35;
Fq334 = -frc[3][34]+Fq235*S35+Fq335*C35;
Cq134 = -trq[1][34]+Cq135-dpt[3][54]*(Fq235*C35-Fq335*S35);
Cq234 = -trq[2][34]+Cq235*C35-Cq335*S35+Fq135*dpt[3][54];
Cq334 = -trq[3][34]+Cq235*S35+Cq335*C35;
Fs133 = -frc[1][33]+m[33]*(ALPHA133+BETA233*l[2][33]);
Fs233 = -frc[2][33]+m[33]*(ALPHA233+BS533*l[2][33]);
Fs333 = -frc[3][33]+m[33]*(ALPHA333+BETA833*l[2][33]);
Cq133 = -trq[1][33]+In[1][33]*OMp133-In[5][33]*OM233*OM333+In[9][33]*OM233*OM333+Fs333*l[2][33];
Cq233 = -trq[2][33]+In[1][33]*OM133*OM333+In[5][33]*OMp233-In[9][33]*OM133*OM333;
Cq333 = -trq[3][33]-In[1][33]*OM133*OM233+In[5][33]*OM133*OM233+In[9][33]*OMp333-Fs133*l[2][33];
Fq132 = -frc[1][32]+Fs133+Fq134*C34-Fq234*S34;
Fq232 = -frc[2][32]+Fq134*S34+Fq234*C34+Fs233*C33-Fs333*S33;
Fq332 = -frc[3][32]+Fq334+Fs233*S33+Fs333*C33;
Cq132 = -trq[1][32]+Cq133+Cq134*C34-Cq234*S34-dpt[3][50]*(Fs233*C33-Fs333*S33)-dpt[3][51]*(Fq134*S34+Fq234*C34);
Cq232 = -trq[2][32]+Cq134*S34+Cq233*C33+Cq234*C34-Cq333*S33+Fs133*dpt[3][50]+dpt[3][51]*(Fq134*C34-Fq234*S34);
Cq332 = -trq[3][32]+Cq334+Cq233*S33+Cq333*C33;
Fq131 = -frc[1][31]+Fq132;
Fq231 = -frc[2][31]+Fq232*C32-Fq332*S32;
Fq331 = -frc[3][31]+Fq232*S32+Fq332*C32;
Cq131 = -trq[1][31]+Cq132+dpt[2][49]*(Fq232*S32+Fq332*C32);
Cq231 = -trq[2][31]+Cq232*C32-Cq332*S32;
Cq331 = -trq[3][31]+Cq232*S32+Cq332*C32-Fq132*dpt[2][49];
Fs129 = -frc[1][29]+m[29]*ALPHA129;
Fs229 = -frc[2][29]+m[29]*ALPHA229;
Fs329 = -frc[3][29]+m[29]*ALPHA329;
Cq129 = -trq[1][29]+In[1][29]*OMp129-In[5][29]*OM229*OM329+In[9][29]*OM229*OM329;
Cq229 = -trq[2][29]+In[1][29]*OM129*OM329+In[5][29]*OMp229-In[9][29]*OM129*OM329;
Cq329 = -trq[3][29]-In[1][29]*OM129*OM229+In[5][29]*OM129*OM229+In[9][29]*OMp329;
Fq128 = -frc[1][28]+Fs129*C29+Fs329*S29;
Fq228 = -frc[2][28]+Fs229;
Fq328 = -frc[3][28]-Fs129*S29+Fs329*C29;
Cq128 = -trq[1][28]+Cq129*C29+Cq329*S29-Fs229*dpt[3][45]+dpt[2][45]*(-Fs129*S29+Fs329*C29);
Cq228 = -trq[2][28]+Cq229+dpt[3][45]*(Fs129*C29+Fs329*S29);
Cq328 = -trq[3][28]-Cq129*S29+Cq329*C29-dpt[2][45]*(Fs129*C29+Fs329*S29);
Fq127 = -frc[1][27]+Fq128;
Fq227 = -frc[2][27]+Fq228*C28-Fq328*S28;
Fq327 = -frc[3][27]+Fq228*S28+Fq328*C28;
Cq127 = -trq[1][27]+Cq128-dpt[3][43]*(Fq228*C28-Fq328*S28);
Cq227 = -trq[2][27]+Cq228*C28-Cq328*S28+Fq128*dpt[3][43];
Cq327 = -trq[3][27]+Cq228*S28+Cq328*C28;
Fs126 = -frc[1][26]+m[26]*(ALPHA126+BETA226*l[2][26]);
Fs226 = -frc[2][26]+m[26]*(ALPHA226+BS526*l[2][26]);
Fs326 = -frc[3][26]+m[26]*(ALPHA326+BETA826*l[2][26]);
Cq126 = -trq[1][26]+In[1][26]*OMp126-In[5][26]*OM226*OM326+In[9][26]*OM226*OM326+Fs326*l[2][26];
Cq226 = -trq[2][26]+In[1][26]*OM126*OM326+In[5][26]*OMp226-In[9][26]*OM126*OM326;
Cq326 = -trq[3][26]-In[1][26]*OM126*OM226+In[5][26]*OM126*OM226+In[9][26]*OMp326-Fs126*l[2][26];
Fq125 = -frc[1][25]+Fs126+Fq127*C27-Fq227*S27;
Fq225 = -frc[2][25]+Fq127*S27+Fq227*C27+Fs226*C26-Fs326*S26;
Fq325 = -frc[3][25]+Fq327+Fs226*S26+Fs326*C26;
Cq125 = -trq[1][25]+Cq126+Cq127*C27-Cq227*S27-dpt[3][39]*(Fs226*C26-Fs326*S26)-dpt[3][40]*(Fq127*S27+Fq227*C27);
Cq225 = -trq[2][25]+Cq127*S27+Cq226*C26+Cq227*C27-Cq326*S26+Fs126*dpt[3][39]+dpt[3][40]*(Fq127*C27-Fq227*S27);
Cq325 = -trq[3][25]+Cq327+Cq226*S26+Cq326*C26;
Fq124 = -frc[1][24]+Fq125;
Fq224 = -frc[2][24]+Fq225*C25-Fq325*S25;
Fq324 = -frc[3][24]+Fq225*S25+Fq325*C25;
Cq124 = -trq[1][24]+Cq125+dpt[2][38]*(Fq225*S25+Fq325*C25);
Cq224 = -trq[2][24]+Cq225*C25-Cq325*S25;
Cq324 = -trq[3][24]+Cq225*S25+Cq325*C25-Fq125*dpt[2][38];
Cq123 = Cq124-q[24]*Fq224;
Cq223 = Cq224+q[24]*Fq124;
Cq122 = Cq123+q[23]*Fq324;
Cq322 = Cq324-q[23]*Fq124;
Fs121 = -frc[1][21]+m[21]*ALPHA121;
Fs221 = -frc[2][21]+m[21]*ALPHA221;
Fs321 = -frc[3][21]+m[21]*ALPHA321;
Cq121 = -trq[1][21]+In[1][21]*OMp121-In[5][21]*OM221*OM321+In[9][21]*OM221*OM321;
Cq221 = -trq[2][21]+In[1][21]*OM121*OM321+In[5][21]*OMp221-In[9][21]*OM121*OM321;
Cq321 = -trq[3][21]-In[1][21]*OM121*OM221+In[5][21]*OM121*OM221+In[9][21]*OMp321;
Fq120 = -frc[1][20]+Fs121*C21+Fs321*S21;
Fq220 = -frc[2][20]+Fs221;
Fq320 = -frc[3][20]-Fs121*S21+Fs321*C21;
Cq120 = -trq[1][20]+Cq121*C21+Cq321*S21-Fs221*dpt[3][36]+dpt[2][36]*(-Fs121*S21+Fs321*C21);
Cq220 = -trq[2][20]+Cq221+dpt[3][36]*(Fs121*C21+Fs321*S21);
Cq320 = -trq[3][20]-Cq121*S21+Cq321*C21-dpt[2][36]*(Fs121*C21+Fs321*S21);
Fq119 = -frc[1][19]+Fq120;
Fq219 = -frc[2][19]+Fq220*C20-Fq320*S20;
Fq319 = -frc[3][19]+Fq220*S20+Fq320*C20;
Cq119 = -trq[1][19]+Cq120-dpt[3][34]*(Fq220*C20-Fq320*S20);
Cq219 = -trq[2][19]+Cq220*C20-Cq320*S20+Fq120*dpt[3][34];
Cq319 = -trq[3][19]+Cq220*S20+Cq320*C20;
Fs118 = -frc[1][18]+m[18]*(ALPHA118+BETA218*l[2][18]);
Fs218 = -frc[2][18]+m[18]*(ALPHA218+BS518*l[2][18]);
Fs318 = -frc[3][18]+m[18]*(ALPHA318+BETA818*l[2][18]);
Cq118 = -trq[1][18]+In[1][18]*OMp118-In[5][18]*OM218*OM318+In[9][18]*OM218*OM318+Fs318*l[2][18];
Cq218 = -trq[2][18]+In[1][18]*OM118*OM318+In[5][18]*OMp218-In[9][18]*OM118*OM318;
Cq318 = -trq[3][18]-In[1][18]*OM118*OM218+In[5][18]*OM118*OM218+In[9][18]*OMp318-Fs118*l[2][18];
Fq117 = -frc[1][17]+Fs118+Fq119*C19-Fq219*S19;
Fq217 = -frc[2][17]+Fq119*S19+Fq219*C19+Fs218*C18-Fs318*S18;
Fq317 = -frc[3][17]+Fq319+Fs218*S18+Fs318*C18;
Cq117 = -trq[1][17]+Cq118+Cq119*C19-Cq219*S19-dpt[3][30]*(Fs218*C18-Fs318*S18)-dpt[3][31]*(Fq119*S19+Fq219*C19);
Cq217 = -trq[2][17]+Cq119*S19+Cq218*C18+Cq219*C19-Cq318*S18+Fs118*dpt[3][30]+dpt[3][31]*(Fq119*C19-Fq219*S19);
Cq317 = -trq[3][17]+Cq319+Cq218*S18+Cq318*C18;
Fq116 = -frc[1][16]+Fq117;
Fq216 = -frc[2][16]+Fq217*C17-Fq317*S17;
Fq316 = -frc[3][16]+Fq217*S17+Fq317*C17;
Cq116 = -trq[1][16]+Cq117+dpt[2][29]*(Fq217*S17+Fq317*C17);
Cq216 = -trq[2][16]+Cq217*C17-Cq317*S17;
Cq316 = -trq[3][16]+Cq217*S17+Cq317*C17-Fq117*dpt[2][29];
Cq115 = Cq116-q[16]*Fq216;
Cq215 = Cq216+q[16]*Fq116;
Cq114 = Cq115+q[15]*Fq316;
Cq314 = Cq316-q[15]*Fq116;
Fs112 = -frc[1][12]+m[12]*ALPHA112;
Fs212 = -frc[2][12]+m[12]*ALPHA212;
Fs312 = -frc[3][12]+m[12]*ALPHA312;
Cq112 = -trq[1][12]+In[1][12]*OMp112-In[5][12]*OM212*OM312+In[9][12]*OM212*OM312;
Cq212 = -trq[2][12]+In[1][12]*OM112*OM312+In[5][12]*OMp212-In[9][12]*OM112*OM312;
Cq312 = -trq[3][12]-In[1][12]*OM112*OM212+In[5][12]*OM112*OM212+In[9][12]*OMp312;
Fq111 = -frc[1][11]+Fs112*C12+Fs312*S12;
Fq211 = -frc[2][11]+Fs212;
Fq311 = -frc[3][11]-Fs112*S12+Fs312*C12;
Cq111 = -trq[1][11]+Cq112*C12+Cq312*S12-Fs212*dpt[3][25]+dpt[2][25]*(-Fs112*S12+Fs312*C12);
Cq211 = -trq[2][11]+Cq212+dpt[3][25]*(Fs112*C12+Fs312*S12);
Cq311 = -trq[3][11]-Cq112*S12+Cq312*C12-dpt[2][25]*(Fs112*C12+Fs312*S12);
Fq110 = -frc[1][10]+Fq111;
Fq210 = -frc[2][10]+Fq211*C11-Fq311*S11;
Fq310 = -frc[3][10]+Fq211*S11+Fq311*C11;
Cq110 = -trq[1][10]+Cq111-dpt[3][23]*(Fq211*C11-Fq311*S11);
Cq210 = -trq[2][10]+Cq211*C11-Cq311*S11+Fq111*dpt[3][23];
Cq310 = -trq[3][10]+Cq211*S11+Cq311*C11;
Fs19 = -frc[1][9]+m[9]*(ALPHA19+BETA29*l[2][9]);
Fs29 = -frc[2][9]+m[9]*(ALPHA29+BS59*l[2][9]);
Fs39 = -frc[3][9]+m[9]*(ALPHA39+BETA89*l[2][9]);
Cq19 = -trq[1][9]+In[1][9]*OMp19-In[5][9]*OM29*OM39+In[9][9]*OM29*OM39+Fs39*l[2][9];
Cq29 = -trq[2][9]+In[1][9]*OM19*OM39+In[5][9]*OMp29-In[9][9]*OM19*OM39;
Cq39 = -trq[3][9]-In[1][9]*OM19*OM29+In[5][9]*OM19*OM29+In[9][9]*OMp39-Fs19*l[2][9];
Fq18 = -frc[1][8]+Fs19+Fq110*C10-Fq210*S10;
Fq28 = -frc[2][8]+Fq110*S10+Fq210*C10+Fs29*C9-Fs39*S9;
Fq38 = -frc[3][8]+Fq310+Fs29*S9+Fs39*C9;
Cq18 = -trq[1][8]+Cq19+Cq110*C10-Cq210*S10-dpt[3][19]*(Fs29*C9-Fs39*S9)-dpt[3][20]*(Fq110*S10+Fq210*C10);
Cq28 = -trq[2][8]+Cq110*S10+Cq210*C10+Cq29*C9-Cq39*S9+Fs19*dpt[3][19]+dpt[3][20]*(Fq110*C10-Fq210*S10);
Cq38 = -trq[3][8]+Cq310+Cq29*S9+Cq39*C9;
Fq17 = -frc[1][7]+Fq18;
Fq27 = -frc[2][7]+Fq28*C8-Fq38*S8;
Fq37 = -frc[3][7]+Fq28*S8+Fq38*C8;
Cq17 = -trq[1][7]+Cq18+dpt[2][18]*(Fq28*S8+Fq38*C8);
Cq27 = -trq[2][7]+Cq28*C8-Cq38*S8;
Cq37 = -trq[3][7]+Cq28*S8+Cq38*C8-Fq18*dpt[2][18];
Fs16 = -frc[1][6]+m[6]*ALPHA16;
Fs26 = -frc[2][6]+m[6]*ALPHA26;
Fs36 = -frc[3][6]+m[6]*ALPHA35;
Fq16 = -frc[1][13]-frc[1][30]-frc[1][37]-frc[1][38]-frc[1][39]-frc[1][40]+Fq116+Fq124+Fq131+Fq17+Fs141+Fs16;
Fq26 = -frc[2][13]-frc[2][30]-frc[2][37]-frc[2][38]-frc[2][39]-frc[2][40]+Fs241+Fs26+Fq216*C14+Fq224*C22+Fq231*C31+
 Fq27*C7-Fq316*S14-Fq324*S22-Fq331*S31-Fq37*S7;
Fq36 = -frc[3][13]-frc[3][30]-frc[3][37]-frc[3][38]-frc[3][39]-frc[3][40]+Fs341+Fs36+Fq216*S14+Fq224*S22+Fq231*S31+
 Fq27*S7+Fq316*C14+Fq324*C22+Fq331*C31+Fq37*C7;
Cq16 = -trq[1][13]-trq[1][30]-trq[1][37]-trq[1][38]-trq[1][39]-trq[1][40]-trq[1][6]+Cq114+Cq122+Cq131+Cq141+Cq17-q[13]
 *frc[3][13]-q[30]*frc[3][30]-q[37]*frc[3][37]-q[38]*frc[3][38]-q[39]*frc[3][39]-q[40]*frc[3][40]+In[1][6]*OMp16-In[5][6]*
 OM26*OM36+In[9][6]*OM26*OM36+frc[2][13]*dpt[3][3]+frc[2][30]*dpt[3][8]+frc[2][37]*dpt[3][15]+frc[2][38]*dpt[3][15]+
 frc[2][39]*dpt[3][16]+frc[2][40]*dpt[3][16]-Fs241*Dz413+Fs341*dpt[2][17]+dpt[2][10]*(Fq231*S31+Fq331*C31)+dpt[2][2]*(Fq27*S7
 +Fq37*C7)+dpt[2][4]*(Fq216*S14+Fq316*C14)+dpt[2][7]*(Fq224*S22+Fq324*C22)-dpt[3][10]*(Fq231*C31-Fq331*S31)-dpt[3][2]*(Fq27*
 C7-Fq37*S7)-dpt[3][4]*(Fq216*C14-Fq316*S14)-dpt[3][7]*(Fq224*C22-Fq324*S22);
Cq26 = -trq[2][13]-trq[2][30]-trq[2][37]-trq[2][38]-trq[2][39]-trq[2][40]-trq[2][6]+Cq241+In[1][6]*OM16*OM36+In[5][6]*
 OMp26-In[9][6]*OM16*OM36-frc[1][13]*dpt[3][3]-frc[1][30]*dpt[3][8]-frc[1][37]*dpt[3][15]-frc[1][38]*dpt[3][15]-frc[1][39]*
 dpt[3][16]-frc[1][40]*dpt[3][16]+frc[3][13]*dpt[1][3]+frc[3][30]*dpt[1][8]+frc[3][37]*dpt[1][15]+frc[3][38]*dpt[1][15]+
 frc[3][39]*dpt[1][16]+frc[3][40]*dpt[1][16]+Cq215*C14+Cq223*C22+Cq231*C31+Cq27*C7-Cq314*S14-Cq322*S22-Cq331*S31-Cq37*S7+
 Fq116*dpt[3][4]+Fq124*dpt[3][7]+Fq131*dpt[3][10]+Fq17*dpt[3][2]+Fs141*Dz413-dpt[1][10]*(Fq231*S31+Fq331*C31)-dpt[1][2]*(Fq27
 *S7+Fq37*C7)-dpt[1][4]*(Fq216*S14+Fq316*C14)-dpt[1][7]*(Fq224*S22+Fq324*C22);
Cq36 = -trq[3][13]-trq[3][30]-trq[3][37]-trq[3][38]-trq[3][39]-trq[3][40]-trq[3][6]+Cq341+q[13]*frc[1][13]+q[30]*
 frc[1][30]+q[37]*frc[1][37]+q[38]*frc[1][38]+q[39]*frc[1][39]+q[40]*frc[1][40]-In[1][6]*OM16*OM26+In[5][6]*OM16*OM26+
 In[9][6]*OMp36-frc[2][13]*dpt[1][3]-frc[2][30]*dpt[1][8]-frc[2][37]*dpt[1][15]-frc[2][38]*dpt[1][15]-frc[2][39]*dpt[1][16]-
 frc[2][40]*dpt[1][16]+Cq215*S14+Cq223*S22+Cq231*S31+Cq27*S7+Cq314*C14+Cq322*C22+Cq331*C31+Cq37*C7-Fq116*dpt[2][4]-Fq124*
 dpt[2][7]-Fq131*dpt[2][10]-Fq17*dpt[2][2]-Fs141*dpt[2][17]+dpt[1][10]*(Fq231*C31-Fq331*S31)+dpt[1][2]*(Fq27*C7-Fq37*S7)+
 dpt[1][4]*(Fq216*C14-Fq316*S14)+dpt[1][7]*(Fq224*C22-Fq324*S22);
Fq15 = Fq16*C6-Fq26*S6;
Fq25 = Fq16*S6+Fq26*C6;
Cq15 = Cq16*C6-Cq26*S6;
Cq25 = Cq16*S6+Cq26*C6;
Fq24 = Fq25*C5-Fq36*S5;
Fq34 = Fq25*S5+Fq36*C5;
Cq24 = Cq25*C5-Cq36*S5;
Fq13 = Fq15*C4+Fq34*S4;
Fq33 = -Fq15*S4+Fq34*C4;
 
// Symbolic model output

Qq[1] = Fq13;
Qq[2] = Fq24;
Qq[3] = Fq33;
Qq[4] = Cq24;
Qq[5] = Cq15;
Qq[6] = Cq36;
Qq[7] = Cq17;
Qq[8] = Cq18;
Qq[9] = Cq19;
Qq[10] = Cq310;
Qq[11] = Cq111;
Qq[12] = Cq212;
Qq[13] = -frc[2][13];
Qq[14] = Cq114;
Qq[15] = Fq216;
Qq[16] = Fq316;
Qq[17] = Cq117;
Qq[18] = Cq118;
Qq[19] = Cq319;
Qq[20] = Cq120;
Qq[21] = Cq221;
Qq[22] = Cq122;
Qq[23] = Fq224;
Qq[24] = Fq324;
Qq[25] = Cq125;
Qq[26] = Cq126;
Qq[27] = Cq327;
Qq[28] = Cq128;
Qq[29] = Cq229;
Qq[30] = -frc[2][30];
Qq[31] = Cq131;
Qq[32] = Cq132;
Qq[33] = Cq133;
Qq[34] = Cq334;
Qq[35] = Cq135;
Qq[36] = Cq236;
Qq[37] = -frc[2][37];
Qq[38] = -frc[2][38];
Qq[39] = -frc[2][39];
Qq[40] = -frc[2][40];
Qq[41] = Fs341;

// Number of continuation lines = 5

}
