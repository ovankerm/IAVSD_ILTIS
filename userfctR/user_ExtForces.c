//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//
//
//---------------------------

#include "math.h"
#include "mbs_data.h"
#include "mbs_matrix.h"
#include "user_model.h"
#include "set_output.h"
#include "user_all_id.h"
#include "GroundContact.h"
#include "useful_functions.h"
#include "mbs_project_interface.h"

double* user_ExtForces(double PxF[4], double RxF[4][4],
                       double VxF[4], double OMxF[4],
                       double AxF[4], double OMPxF[4],
                       MbsData *mbs_data, double tsim,int ixF)
{
    double Fx=0.0, Fy=0.0, Fz=0.0;
    double Mx=0.0, My=0.0, Mz=0.0;
    double dxF[4] ={0.0, 0.0, 0.0, 0.0};
    double *SWr = mbs_data->SWr[ixF];
    int idpt = 0;
    idpt = mbs_data->xfidpt[ixF];
    dxF[1] = mbs_data->dpt[1][idpt];
    dxF[2] = mbs_data->dpt[2][idpt];
    dxF[3] = mbs_data->dpt[3][idpt];

    UserModel *um = mbs_data->user_model;
    double R = 0.0;
    double K = um->Wheels.Stiffness;
    int left = 0;

    switch(ixF){
        case FL_Ground_id:
            left=1;
        case FR_Ground_id:
            R = um->Wheels.F_Rad;
            break;
        case RL_Ground_id:
            left=1;
        case RR_Ground_id:
            R = um->Wheels.R_Rad;
            break;
    }
    switch (mbs_data->process){
        case 0:
            // Nominal condition
            Fz = PxF[3]-R>0.0 ? 0.0 : (R-PxF[3])*K;
            if (ixF == FL_Ground_id){
                um->Initial_State.FL_Wheel_F = Fz;
                um->Initial_State.FL_Wheel_Z = PxF[3];
            }
            else if (ixF == RL_Ground_id){
                um->Initial_State.RL_Wheel_F = Fz;
                um->Initial_State.RL_Wheel_Z = PxF[3];
            }
        case 2:
            // Static equilibrium
            Fz = (R-PxF[3])*K;
            if (ixF == FR_Ground_id){
                um->Equilibrium_State.FR_Tyre = Fz;
            }
            else if (ixF == RR_Ground_id){
                um->Equilibrium_State.RR_Tyre = Fz;
            }
            break;
        case 3:
            // direct dynamic
            set_output_value(PxF[3], ixF, "Wheel_height");
            double Angcamb,Anglis;
            double *ng = get_dvec_1(3); // > R3: Vector normal to ground profile (normalized)
            double *P  = get_dvec_1(3);
            double Fvert = 0;
            if (um->Status.Bump){
                if(um->Status.Simple_contact){
                    Fvert = ComputeSimpleRadialForce(PxF[1], PxF[3], K, R, P, ng, left*um->Status.AntiPhase); // > Radial force in the Ground Frame ([Rsol])
                } else {
                    Fvert = ComputeRadialForce(PxF[1], PxF[3], K, R, P, ng, left*um->Status.AntiPhase); // > Radial force in the Ground Frame ([Rsol])
                }
            }
            else if (um->Status.Belgian_road) {
                //Fvert = ComputeRadialForce(PxF[1], PxF[3], K, R, P, ng, left*um->Status.AntiPhase);
                Fvert = ComputeRadialForce_Belgian_road(PxF[1], PxF[3], K, R, P, ng, left);
            }
            else{
                ng[1] = 0; ng[2] = 0; ng[3] = 1;
                P[1] = PxF[1]; P[2] = PxF[2]; P[3] = 0;
                Fvert = K*(R-PxF[3]);
                if (Fvert<0) Fvert = 0;
            }

            if (Fvert !=0){
                double Flat = 0.0; // > Lateral force in the Ground Frame ([Rsol])
                double Myaw = 0.0; // > Aligning torque in the Ground Frame ([Rsol])
                double Gliss= 1.0; // > Longitudinal slip, imposed by benchmark.
                double *ey = get_dvec_1(3); // > Y2=T2: Wheel axis in Inertial frame (normalized)
                ey[1] = RxF[2][1];
                ey[2] = RxF[2][2];
                ey[3] = RxF[2][3];
                double *ex =get_dvec_1(3); // > T1=R1: Vector orthogonal to ground normal and wheel axis (normalized): wheel tangeant vector
                cross_product(ey,ng,ex);
                normalize(ex, ex);
                double *eyPG = get_dvec_1(3); // >R2: Projection of wheel axis in ground tangeant plane (normalized)
                cross_product(ng,ex,eyPG);
                normalize(eyPG, eyPG);

                double Rsol[4][4];  // > Ground frame, x wheel tangeant, y is projection of wheel axis in ground tangeant plane, z is normal to ground profile
                double Rtsol[4][4]; // > Transpose of Rsol
                // [Rsol] = Rsol*[I]: [R1; R2; R3]
                Rsol[1][1] = ex[1];
                Rsol[1][2] = ex[2];
                Rsol[1][3] = ex[3];

                Rsol[2][1] = eyPG[1];
                Rsol[2][2] = eyPG[2];
                Rsol[2][3] = eyPG[3];

                Rsol[3][1] = ng[1];
                Rsol[3][2] = ng[2];
                Rsol[3][3] = ng[3];

                transpose(Rsol,Rtsol);

                double *ez =get_dvec_1(3); // > T3: Wheel spoke vector: tangeant to wheel direction on ground and axis(T1) and wheel axis (T2) (normalized)
                cross_product(ex,ey,ez);
                normalize(ez, ez);
                double rw = (PxF[3]-P[3])/ez[3];
                if (rw<0) printf("mert\n");
                double Rtg[4][4];  // > Tangeant wheel frame, x wheel tangeant belonging to ground, y is wheel axis, z is a wheel spoke from ground to center
                double Rttg[4][4]; // > Transpose of Rtg
                // [Rtg] = Rtg*[I]: [T1; T2; T3]
                Rtg[1][1] = ex[1];
                Rtg[1][2] = ex[2];
                Rtg[1][3] = ex[3];

                Rtg[2][1] = ey[1];
                Rtg[2][2] = ey[2];
                Rtg[2][3] = ey[3];

                Rtg[3][1] = ez[1];
                Rtg[3][2] = ez[2];
                Rtg[3][3] = ez[3];

                transpose(Rtg,Rttg);

                Angcamb = atan( scalar_product(ng, ey)/scalar_product(ng, ez) );

                if (norm(VxF)>0.1 && scalar_product(VxF, ex)>1e-9){
                    Anglis  = atan( scalar_product(VxF, eyPG)/scalar_product(VxF, ex));
                    // Assumption :
                    //  1: d(camber_angle)/dt small
                    //  2: d(theta)/dt small (~angular velocity of the wheel around its axis)
                }
                else{
                    Anglis=0;
                }

                Calspan(&Flat, &Myaw, Fvert, Anglis, Angcamb, Gliss, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.);
                // Putting all parameters at 0 force uses of values from benchmark (hardcoded in function Calspan)

                // Express forces and torques in inertial frame
                double Temp_IN[4];
                double Temp_OUT[4];
                // Radial Force
                Temp_IN[0]=3; Temp_IN[1]=0; Temp_IN[2]=0; Temp_IN[3]=Fvert;
                matrix_product(Rtsol, Temp_IN, Temp_OUT);
                Fx += Temp_OUT[1];
                Fy += Temp_OUT[2];
                Fz += Temp_OUT[3];

                // Lateral Force
                Temp_IN[1]=0; Temp_IN[2]=Flat; Temp_IN[3]=0.0;
                matrix_product(Rtsol, Temp_IN, Temp_OUT);
                Fx += Temp_OUT[1];
                Fy += Temp_OUT[2];
                Fz += Temp_OUT[3];

                // Aligning Torque
                Temp_IN[1]=0; Temp_IN[2]=0.0; Temp_IN[3]=Myaw;
                matrix_product(Rtsol, Temp_IN, Temp_OUT);
                Mx += Temp_OUT[1];
                My += Temp_OUT[2];
                Mz += Temp_OUT[3];

                // Contact point in Wheel Frame
                Temp_IN[1]=0.0; Temp_IN[2]=0.0; Temp_IN[3]=-rw;
                matrix_product(Rttg, Temp_IN, Temp_OUT);// in I
                matrix_product(RxF, Temp_OUT, dxF);//in wheel


                free_dvec_1(ez);
                free_dvec_1(eyPG);
                free_dvec_1(ex);
                free_dvec_1(ey);
            }

            free_dvec_1(P);
            free_dvec_1(ng);
            break;
        case 4:
            if (um->Status.Linear_Modal==1){
                double Angcamb,Anglis;
                double ng[4] = {3, 0.0, 0.0, 1.0};
                double P[4]  = {3, PxF[1], PxF[2], 0.0};
                Fz = (R-PxF[3])*K;

                double *ey = get_dvec_1(3); // > Y2=T2: Wheel axis in Inertial frame (normalized)
                ey[1] = RxF[2][1];
                ey[2] = RxF[2][2];
                ey[3] = RxF[2][3];
                double *ex =get_dvec_1(3); // > T1=R1: Vector orthogonal to ground normal and wheel axis (normalized): wheel tangeant vector
                cross_product(ey,ng,ex);
                normalize(ex, ex);
                double *eyPG = get_dvec_1(3); // >R2: Projection of wheel axis in ground tangeant plane (normalized)
                cross_product(ng,ex,eyPG);
                normalize(eyPG, eyPG);

                double Rsol[4][4];  // > Ground frame, x wheel tangeant, y is projection of wheel axis in ground tangeant plane, z is normal to ground profile
                double Rtsol[4][4]; // > Transpose of Rsol
                // [Rsol] = Rsol*[I]: [R1; R2; R3]
                Rsol[1][1] = ex[1];
                Rsol[1][2] = ex[2];
                Rsol[1][3] = ex[3];

                Rsol[2][1] = eyPG[1];
                Rsol[2][2] = eyPG[2];
                Rsol[2][3] = eyPG[3];

                Rsol[3][1] = ng[1];
                Rsol[3][2] = ng[2];
                Rsol[3][3] = ng[3];

                transpose(Rsol,Rtsol);

                double *ez =get_dvec_1(3); // > T3: Wheel spoke vector: tangeant to wheel direction on ground and axis(T1) and wheel axis (T2) (normalized)
                cross_product(ex,ey,ez);
                normalize(ez, ez);
                double rw = (PxF[3]-P[3])/ez[3];
                if (rw<0){printf("mert\n");}
                double Rtg[4][4];  // > Tangeant wheel frame, x wheel tangeant belonging to ground, y is wheel axis, z is a wheel spoke from ground to center
                double Rttg[4][4]; // > Transpose of Rtg
                // [Rtg] = Rtg*[I]: [T1; T2; T3]
                Rtg[1][1] = ex[1];
                Rtg[1][2] = ex[2];
                Rtg[1][3] = ex[3];

                Rtg[2][1] = ey[1];
                Rtg[2][2] = ey[2];
                Rtg[2][3] = ey[3];

                Rtg[3][1] = ez[1];
                Rtg[3][2] = ez[2];
                Rtg[3][3] = ez[3];

                transpose(Rtg,Rttg);

                Angcamb = atan( scalar_product(ng, ey)/scalar_product(ng, ez) );

                if (norm(VxF)>0.1 && scalar_product(VxF, ex)>1e-9){
                    Anglis  = atan( scalar_product(VxF, eyPG)/scalar_product(VxF, ex));
                    // Assumption :
                    //  1: d(camber_angle)/dt small
                    //  2: d(theta)/dt small (~angular velocity of the wheel around its axis)
                }
                else{
                    Anglis=0;
                }
                double Myaw, Mroll, Flat;
                double Kcorn, Kcamb, Trail, Dlat;
                if (ixF == FL_Ground_id || ixF == FR_Ground_id){
                    Kcorn = 41641;
                    Kcamb = 6925;
                    Trail = 0.06553;
                    Dlat  = 2082.05;
                }
                else{
                    Kcorn = 40162;
                    Kcamb = 6600;
                    Trail = 0.06142;
                    Dlat  = 2008.1;
                }
                Myaw  = Kcorn*Anglis;
                Mroll = Kcamb*Angcamb;
                Flat  = -scalar_product(VxF, eyPG)*Dlat;

                // Express forces and torques in inertial frame
                // Fz already include Fvert/rad
                double Temp_IN[4];
                double Temp_OUT[4];

                // Lateral Force expressed in Rsol [R]
                Temp_IN[1]=0.0; Temp_IN[2]=Flat; Temp_IN[3]=0.0;
                matrix_product(Rtsol, Temp_IN, Temp_OUT);
                Fx += Temp_OUT[1];
                Fy += Temp_OUT[2];
                Fz += Temp_OUT[3];

                // Camber Torque expressed in Rsol [R]
                Temp_IN[1]=Mroll; Temp_IN[2]=0.0; Temp_IN[3]=0.0;
                matrix_product(Rtsol, Temp_IN, Temp_OUT);
                Mx += Temp_OUT[1];
                My += Temp_OUT[2];
                Mz += Temp_OUT[3];

                // Aligning Torque expressed in Rsol [R]
                Temp_IN[1]=0.0; Temp_IN[2]=0.0; Temp_IN[3]=Myaw;
                matrix_product(Rtsol, Temp_IN, Temp_OUT);
                Mx += Temp_OUT[1];
                My += Temp_OUT[2];
                Mz += Temp_OUT[3];

                // Coordinate of contact point in Wheel Frame (actually in Rtg [T])
                Temp_IN[1]=-Trail; Temp_IN[2]=0.0; Temp_IN[3]=-rw;
                matrix_product(Rttg, Temp_IN, Temp_OUT);// in I
                matrix_product(RxF, Temp_OUT, dxF);//in wheel


                free_dvec_1(ez);
                free_dvec_1(eyPG);
                free_dvec_1(ex);
                free_dvec_1(ey);


            }
            break;
    }
    set_output_value(Fz, ixF, "Ground_Forces");

    SWr[1]=Fx;
    SWr[2]=Fy;
    SWr[3]=Fz;
    SWr[4]=Mx;
    SWr[5]=My;
    SWr[6]=Mz;
    SWr[7]=dxF[1];
    SWr[8]=dxF[2];
    SWr[9]=dxF[3];

    return SWr;
}


