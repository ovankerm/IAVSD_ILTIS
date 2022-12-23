//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"
#include "mbs_data.h"
#include "user_all_id.h"

void user_cons_hJ(double *h, double **Jac, MbsData *mbs_data, double tsim)
{
    h[1] = 0.1585 +0.33664707*cos(mbs_data->q[R1_FL_id]) +mbs_data->q[T2_FL_Stop_id];
    h[2] =-0.1585 -0.33664707*cos(mbs_data->q[R1_FR_id]) +mbs_data->q[T2_FR_Stop_id];
    h[3] = 0.1585 +0.33664707*cos(mbs_data->q[R1_RL_id]) +mbs_data->q[T2_RL_Stop_id];
    h[4] =-0.1585 -0.33664707*cos(mbs_data->q[R1_RR_id]) +mbs_data->q[T2_RR_Stop_id];
    Jac[1][R1_FL_id] = -0.33664707*sin(mbs_data->q[R1_FL_id]);
    Jac[2][R1_FR_id] =  0.33664707*sin(mbs_data->q[R1_FR_id]);
    Jac[3][R1_RL_id] = -0.33664707*sin(mbs_data->q[R1_RL_id]);
    Jac[4][R1_RR_id] =  0.33664707*sin(mbs_data->q[R1_RR_id]);
    Jac[1][T2_FL_Stop_id] = 1;
    Jac[2][T2_FR_Stop_id] = 1;
    Jac[3][T2_RL_Stop_id] = 1;
    Jac[4][T2_RR_Stop_id] = 1;
}
