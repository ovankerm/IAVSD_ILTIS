/* --------------------------------------------------------
 * This code was generated automatically by MBsysC modules.
 * MBsysC modules are distributed as part of the ROBOTRAN 
 * software. They provides functionalities for dealing with
 * symbolic equations generated by ROBOTRAN. 
 *
 * More info on www.robotran.be 
 *
 * Universite catholique de Louvain, Belgium 
 *
 * Last update : Thu Jan  5 23:49:46 2023
 * --------------------------------------------------------
 *
 */
#include "mbs_path.h"
#include "user_model.h"
#include "mbs_xml_binder_public.h"
#include "mbs_load_xml.h"
#include "useful_functions.h"
#include "mbs_message.h"
#include "math.h"

// ============================================================ //


UserModel* mbs_new_user_model() 
{
    UserModel* um;
    um = (UserModel*)malloc(sizeof(UserModel));
    um->Wheels.F_Rad = 0.0;
    um->Wheels.R_Rad = 0.0;
    um->Wheels.Stiffness = 0.0;
 
    um->LeafSpring.k = 0.0;
    um->LeafSpring.l0 = 0.0;
    um->LeafSpring.k_stop = 0.0;
 
    um->Initial_State.FL_Wheel_Z = 0.0;
    um->Initial_State.RL_Wheel_Z = 0.0;
    um->Initial_State.FL_Wheel_F = 0.0;
    um->Initial_State.RL_Wheel_F = 0.0;
    um->Initial_State.FL_Leaf_F = 0.0;
    um->Initial_State.FL_Leaf_Z = 0.0;
    um->Initial_State.FL_Spring_F = 0.0;
    um->Initial_State.FL_Spring_Z = 0.0;
 
    um->Equilibrium_State.FR_Tyre = 0.0;
    um->Equilibrium_State.RR_Tyre = 0.0;
    um->Equilibrium_State.FR_Spring = 0.0;
    um->Equilibrium_State.RR_Spring = 0.0;
    um->Equilibrium_State.FR_Leaf_F = 0.0;
    um->Equilibrium_State.FR_Leaf_Rod_F = 0.0;
    um->Equilibrium_State.RR_Leaf_F = 0.0;
    um->Equilibrium_State.RR_Leaf_Rod_F = 0.0;
 
    um->Status.AntiPhase = 0;
    um->Status.Bump = 0;
    um->Status.Steering = 0;
    um->Status.Steering_sinus = 0;
    um->Status.Linear_Modal = 0;
    um->Status.PID = 0;
    um->Status.Simple_contact = 0;
 
    um->PID.Kp = 0.0;
    um->PID.Kd = 0.0;
    um->PID.Ki = 0.0;
    um->PID.e_sum = 0.0;
    um->PID.e_prev = 0.0;
 
    return um;
}

void mbs_delete_user_model(UserModel* um) 
{
    free(um);
}

 void mbs_load_user_model_xml(MbsInfos* mbs_infos, UserModel* um) 
{
    int ind_state_value = 1;

    um->Wheels.F_Rad = mbs_infos->user_models->user_model_list[0]->parameter_list[0]->value_list[1];
    um->Wheels.R_Rad = mbs_infos->user_models->user_model_list[0]->parameter_list[1]->value_list[1];
    um->Wheels.Stiffness = mbs_infos->user_models->user_model_list[0]->parameter_list[2]->value_list[1];
 
    um->LeafSpring.k = mbs_infos->user_models->user_model_list[1]->parameter_list[0]->value_list[1];
    um->LeafSpring.l0 = mbs_infos->user_models->user_model_list[1]->parameter_list[1]->value_list[1];
    um->LeafSpring.k_stop = mbs_infos->user_models->user_model_list[1]->parameter_list[2]->value_list[1];
 
    um->Initial_State.FL_Wheel_Z = mbs_infos->user_models->user_model_list[2]->parameter_list[0]->value_list[1];
    um->Initial_State.RL_Wheel_Z = mbs_infos->user_models->user_model_list[2]->parameter_list[1]->value_list[1];
    um->Initial_State.FL_Wheel_F = mbs_infos->user_models->user_model_list[2]->parameter_list[2]->value_list[1];
    um->Initial_State.RL_Wheel_F = mbs_infos->user_models->user_model_list[2]->parameter_list[3]->value_list[1];
    um->Initial_State.FL_Leaf_F = mbs_infos->user_models->user_model_list[2]->parameter_list[4]->value_list[1];
    um->Initial_State.FL_Leaf_Z = mbs_infos->user_models->user_model_list[2]->parameter_list[5]->value_list[1];
    um->Initial_State.FL_Spring_F = mbs_infos->user_models->user_model_list[2]->parameter_list[6]->value_list[1];
    um->Initial_State.FL_Spring_Z = mbs_infos->user_models->user_model_list[2]->parameter_list[7]->value_list[1];
 
    um->Equilibrium_State.FR_Tyre = mbs_infos->user_models->user_model_list[3]->parameter_list[0]->value_list[1];
    um->Equilibrium_State.RR_Tyre = mbs_infos->user_models->user_model_list[3]->parameter_list[1]->value_list[1];
    um->Equilibrium_State.FR_Spring = mbs_infos->user_models->user_model_list[3]->parameter_list[2]->value_list[1];
    um->Equilibrium_State.RR_Spring = mbs_infos->user_models->user_model_list[3]->parameter_list[3]->value_list[1];
    um->Equilibrium_State.FR_Leaf_F = mbs_infos->user_models->user_model_list[3]->parameter_list[4]->value_list[1];
    um->Equilibrium_State.FR_Leaf_Rod_F = mbs_infos->user_models->user_model_list[3]->parameter_list[5]->value_list[1];
    um->Equilibrium_State.RR_Leaf_F = mbs_infos->user_models->user_model_list[3]->parameter_list[6]->value_list[1];
    um->Equilibrium_State.RR_Leaf_Rod_F = mbs_infos->user_models->user_model_list[3]->parameter_list[7]->value_list[1];
 
    um->Status.AntiPhase = (int)lround(mbs_infos->user_models->user_model_list[4]->parameter_list[0]->value_list[1]);
    um->Status.Bump = (int)lround(mbs_infos->user_models->user_model_list[4]->parameter_list[1]->value_list[1]);
    um->Status.Steering = (int)lround(mbs_infos->user_models->user_model_list[4]->parameter_list[2]->value_list[1]);
    um->Status.Steering_sinus = (int)lround(mbs_infos->user_models->user_model_list[4]->parameter_list[3]->value_list[1]);
    um->Status.Linear_Modal = (int)lround(mbs_infos->user_models->user_model_list[4]->parameter_list[4]->value_list[1]);
    um->Status.PID = (int)lround(mbs_infos->user_models->user_model_list[4]->parameter_list[5]->value_list[1]);
    um->Status.Simple_contact = (int)lround(mbs_infos->user_models->user_model_list[4]->parameter_list[6]->value_list[1]);
 
    um->PID.Kp = mbs_infos->user_models->user_model_list[5]->parameter_list[0]->value_list[1];
    um->PID.Kd = mbs_infos->user_models->user_model_list[5]->parameter_list[1]->value_list[1];
    um->PID.Ki = mbs_infos->user_models->user_model_list[5]->parameter_list[2]->value_list[1];
    um->PID.e_sum = mbs_infos->user_models->user_model_list[5]->parameter_list[3]->value_list[1];
    um->PID.e_prev = mbs_infos->user_models->user_model_list[5]->parameter_list[4]->value_list[1];
 
}

 void mbs_bind_user_model(MbsInfos* mbs_infos, UserModel* um) 
{
    mbs_infos->user_models->user_model_list[0]->parameter_list[0]->val_ptr = &um->Wheels.F_Rad;
    mbs_infos->user_models->user_model_list[0]->parameter_list[1]->val_ptr = &um->Wheels.R_Rad;
    mbs_infos->user_models->user_model_list[0]->parameter_list[2]->val_ptr = &um->Wheels.Stiffness;
 
    mbs_infos->user_models->user_model_list[1]->parameter_list[0]->val_ptr = &um->LeafSpring.k;
    mbs_infos->user_models->user_model_list[1]->parameter_list[1]->val_ptr = &um->LeafSpring.l0;
    mbs_infos->user_models->user_model_list[1]->parameter_list[2]->val_ptr = &um->LeafSpring.k_stop;
 
    mbs_infos->user_models->user_model_list[2]->parameter_list[0]->val_ptr = &um->Initial_State.FL_Wheel_Z;
    mbs_infos->user_models->user_model_list[2]->parameter_list[1]->val_ptr = &um->Initial_State.RL_Wheel_Z;
    mbs_infos->user_models->user_model_list[2]->parameter_list[2]->val_ptr = &um->Initial_State.FL_Wheel_F;
    mbs_infos->user_models->user_model_list[2]->parameter_list[3]->val_ptr = &um->Initial_State.RL_Wheel_F;
    mbs_infos->user_models->user_model_list[2]->parameter_list[4]->val_ptr = &um->Initial_State.FL_Leaf_F;
    mbs_infos->user_models->user_model_list[2]->parameter_list[5]->val_ptr = &um->Initial_State.FL_Leaf_Z;
    mbs_infos->user_models->user_model_list[2]->parameter_list[6]->val_ptr = &um->Initial_State.FL_Spring_F;
    mbs_infos->user_models->user_model_list[2]->parameter_list[7]->val_ptr = &um->Initial_State.FL_Spring_Z;
 
    mbs_infos->user_models->user_model_list[3]->parameter_list[0]->val_ptr = &um->Equilibrium_State.FR_Tyre;
    mbs_infos->user_models->user_model_list[3]->parameter_list[1]->val_ptr = &um->Equilibrium_State.RR_Tyre;
    mbs_infos->user_models->user_model_list[3]->parameter_list[2]->val_ptr = &um->Equilibrium_State.FR_Spring;
    mbs_infos->user_models->user_model_list[3]->parameter_list[3]->val_ptr = &um->Equilibrium_State.RR_Spring;
    mbs_infos->user_models->user_model_list[3]->parameter_list[4]->val_ptr = &um->Equilibrium_State.FR_Leaf_F;
    mbs_infos->user_models->user_model_list[3]->parameter_list[5]->val_ptr = &um->Equilibrium_State.FR_Leaf_Rod_F;
    mbs_infos->user_models->user_model_list[3]->parameter_list[6]->val_ptr = &um->Equilibrium_State.RR_Leaf_F;
    mbs_infos->user_models->user_model_list[3]->parameter_list[7]->val_ptr = &um->Equilibrium_State.RR_Leaf_Rod_F;
 
    mbs_infos->user_models->user_model_list[4]->parameter_list[0]->val_ptr = &um->Status.AntiPhase;
    mbs_infos->user_models->user_model_list[4]->parameter_list[1]->val_ptr = &um->Status.Bump;
    mbs_infos->user_models->user_model_list[4]->parameter_list[2]->val_ptr = &um->Status.Steering;
    mbs_infos->user_models->user_model_list[4]->parameter_list[3]->val_ptr = &um->Status.Steering_sinus;
    mbs_infos->user_models->user_model_list[4]->parameter_list[4]->val_ptr = &um->Status.Linear_Modal;
    mbs_infos->user_models->user_model_list[4]->parameter_list[5]->val_ptr = &um->Status.PID;
    mbs_infos->user_models->user_model_list[4]->parameter_list[6]->val_ptr = &um->Status.Simple_contact;
 
    mbs_infos->user_models->user_model_list[5]->parameter_list[0]->val_ptr = &um->PID.Kp;
    mbs_infos->user_models->user_model_list[5]->parameter_list[1]->val_ptr = &um->PID.Kd;
    mbs_infos->user_models->user_model_list[5]->parameter_list[2]->val_ptr = &um->PID.Ki;
    mbs_infos->user_models->user_model_list[5]->parameter_list[3]->val_ptr = &um->PID.e_sum;
    mbs_infos->user_models->user_model_list[5]->parameter_list[4]->val_ptr = &um->PID.e_prev;
 
}
 
 void mbs_print_user_model(UserModel* um) 
{

    printf("user_model->Wheels.F_Rad=%f\n", um->Wheels.F_Rad);
    printf("user_model->Wheels.R_Rad=%f\n", um->Wheels.R_Rad);
    printf("user_model->Wheels.Stiffness=%f\n", um->Wheels.Stiffness);
 
    printf("user_model->LeafSpring.k=%f\n", um->LeafSpring.k);
    printf("user_model->LeafSpring.l0=%f\n", um->LeafSpring.l0);
    printf("user_model->LeafSpring.k_stop=%f\n", um->LeafSpring.k_stop);
 
    printf("user_model->Initial_State.FL_Wheel_Z=%f\n", um->Initial_State.FL_Wheel_Z);
    printf("user_model->Initial_State.RL_Wheel_Z=%f\n", um->Initial_State.RL_Wheel_Z);
    printf("user_model->Initial_State.FL_Wheel_F=%f\n", um->Initial_State.FL_Wheel_F);
    printf("user_model->Initial_State.RL_Wheel_F=%f\n", um->Initial_State.RL_Wheel_F);
    printf("user_model->Initial_State.FL_Leaf_F=%f\n", um->Initial_State.FL_Leaf_F);
    printf("user_model->Initial_State.FL_Leaf_Z=%f\n", um->Initial_State.FL_Leaf_Z);
    printf("user_model->Initial_State.FL_Spring_F=%f\n", um->Initial_State.FL_Spring_F);
    printf("user_model->Initial_State.FL_Spring_Z=%f\n", um->Initial_State.FL_Spring_Z);
 
    printf("user_model->Equilibrium_State.FR_Tyre=%f\n", um->Equilibrium_State.FR_Tyre);
    printf("user_model->Equilibrium_State.RR_Tyre=%f\n", um->Equilibrium_State.RR_Tyre);
    printf("user_model->Equilibrium_State.FR_Spring=%f\n", um->Equilibrium_State.FR_Spring);
    printf("user_model->Equilibrium_State.RR_Spring=%f\n", um->Equilibrium_State.RR_Spring);
    printf("user_model->Equilibrium_State.FR_Leaf_F=%f\n", um->Equilibrium_State.FR_Leaf_F);
    printf("user_model->Equilibrium_State.FR_Leaf_Rod_F=%f\n", um->Equilibrium_State.FR_Leaf_Rod_F);
    printf("user_model->Equilibrium_State.RR_Leaf_F=%f\n", um->Equilibrium_State.RR_Leaf_F);
    printf("user_model->Equilibrium_State.RR_Leaf_Rod_F=%f\n", um->Equilibrium_State.RR_Leaf_Rod_F);
 
    printf("user_model->Status.AntiPhase=%d\n", um->Status.AntiPhase);
    printf("user_model->Status.Bump=%d\n", um->Status.Bump);
    printf("user_model->Status.Steering=%d\n", um->Status.Steering);
    printf("user_model->Status.Steering_sinus=%d\n", um->Status.Steering_sinus);
    printf("user_model->Status.Linear_Modal=%d\n", um->Status.Linear_Modal);
    printf("user_model->Status.PID=%d\n", um->Status.PID);
    printf("user_model->Status.Simple_contact=%d\n", um->Status.Simple_contact);
 
    printf("user_model->PID.Kp=%f\n", um->PID.Kp);
    printf("user_model->PID.Kd=%f\n", um->PID.Kd);
    printf("user_model->PID.Ki=%f\n", um->PID.Ki);
    printf("user_model->PID.e_sum=%f\n", um->PID.e_sum);
    printf("user_model->PID.e_prev=%f\n", um->PID.e_prev);
 
}
 
void mbs_get_user_model_size(int *n_user_model) 
{
    *n_user_model  = 6; 
}
 
void mbs_get_user_model_list(int *user_model_list) 
{
    user_model_list[1]  = 3; 
    user_model_list[2]  = 3; 
    user_model_list[3]  = 8; 
    user_model_list[4]  = 8; 
    user_model_list[5]  = 7; 
    user_model_list[6]  = 5; 

}

// ============================================================ //
 
