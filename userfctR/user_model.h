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
 * Last update : Mon Nov 14 14:37:06 2022
 * --------------------------------------------------------
 *
 */
#ifndef USERMODEL_h
#define USERMODEL_h



#include "mbs_user_interface.h"
// ============================================================ //


struct UserModel 
{
    struct Wheels{
        double F_Rad;
        double R_Rad;
        double Stiffness;
    } Wheels;
 
    struct LeafSpring{
        double k;
        double l0;
        double k_stop;
    } LeafSpring;
 
    struct Initial_State{
        double FL_Wheel_Z;
        double RL_Wheel_Z;
        double FL_Wheel_F;
        double RL_Wheel_F;
        double FL_Leaf_F;
        double FL_Leaf_Z;
        double FL_Spring_F;
        double FL_Spring_Z;
    } Initial_State;
 
    struct Equilibrium_State{
        double FR_Tyre;
        double RR_Tyre;
        double FR_Spring;
        double RR_Spring;
        double FR_Leaf_F;
        double FR_Leaf_Rod_F;
        double RR_Leaf_F;
        double RR_Leaf_Rod_F;
    } Equilibrium_State;
 
    struct Status{
        int AntiPhase;
        int Bump;
        int Steering;
        int Steering_sinus;
        int Linear_Modal;
    } Status;
 
};

// ============================================================ //
 
# endif
