//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"
#include "user_IO.h"
#include "mbs_data.h"
#include "set_output.h"
#include "user_model.h"
#include "user_all_id.h"

double user_LinkForces(double Z, double Zd, MbsData *mbs_data, double tsim, int ilnk)
{

    double Flink = 0;

    switch (ilnk) {
    case FL_Bumper_id:
    case FR_Bumper_id:
        if (mbs_data->process==4 && mbs_data->user_model->Status.Linear_Modal){
            double F_spring = 24529*Z;
            double F_damp   = 9945.627*Zd;
            Flink = F_spring+F_damp;
            break;
        }
    case RL_Bumper_id:
    case RR_Bumper_id:
        if (mbs_data->process==4 && mbs_data->user_model->Status.Linear_Modal){
            double F_spring = 36975*Z;
            double F_damp   = 9945.627*Zd;
            Flink = F_spring+F_damp;
        }
        else{
            Flink = 0.0;
            double F_spring = -4.0092e6 +2.8397e7*Z -6.7061e7*Z*Z +5.2796e7*Z*Z*Z;
            double F_damp   = 0.0;
            if (Zd < -0.2){
                F_damp = -416.42 + 1844.3*Zd;
            }
            else if (Zd > 0.21){
                F_damp = 1919.1638 +1634.727*Zd;
            }
            else{
                F_damp = 9945.627*Zd +33955.72*Zd*Zd -59832.25*Zd*Zd*Zd -395651.0*Zd*Zd*Zd*Zd;
            }
            Flink = (F_damp+F_spring);

            if (mbs_data->process==0 && ilnk == FL_Bumper_id){
                mbs_data->user_model->Initial_State.FL_Spring_F = Flink;
                mbs_data->user_model->Initial_State.FL_Spring_Z = Z;
            }
            else if (mbs_data->process==2 && ilnk == FR_Bumper_id){
                mbs_data->user_model->Equilibrium_State.FR_Spring = Flink;
            }
            else if (mbs_data->process==2 && ilnk == RR_Bumper_id){
                mbs_data->user_model->Equilibrium_State.RR_Spring = Flink;
            }
        }
        break;
    case FL_Leaf_id:
    case FR_Leaf_id:
    case RL_Leaf_id:
    case RR_Leaf_id:
        Flink = 0.0;
        double L = Z - 0.1;
        if (L<0.006){
            Flink = 35906*(0.006-0.152)+1e7*(L-0.006);//1e7*(L-0.152);//
        }
        else {
            Flink = 35906*(L-0.152);
        }

        if (mbs_data->process==0 && ilnk == FL_Leaf_id){
            mbs_data->user_model->Initial_State.FL_Leaf_F = Flink;
            mbs_data->user_model->Initial_State.FL_Leaf_Z = L;
        }
        else if (mbs_data->process==2 && ilnk == FR_Leaf_id){
            mbs_data->user_model->Equilibrium_State.FR_Leaf_F = Flink;
        }
        else if (mbs_data->process==2 && ilnk == RR_Leaf_id){
            mbs_data->user_model->Equilibrium_State.RR_Leaf_F = Flink;
        }
        set_output_value(Flink, ilnk-4, "Leaf_Forces");
        break;
    default:
        break;
    }
    return  Flink;
}
