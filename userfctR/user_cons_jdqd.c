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


void user_cons_jdqd(double *jdqd, MbsData *mbs_data, double tsim)
{
    // Can be ignored if speed is required, in this case it has no impact on the dynamics.
    jdqd[1] = -0.33664707*cos(mbs_data->q[R1_FL_id])*mbs_data->qd[R1_FL_id]*mbs_data->qd[R1_FL_id];
    jdqd[2] =  0.33664707*cos(mbs_data->q[R1_FR_id])*mbs_data->qd[R1_FR_id]*mbs_data->qd[R1_FR_id];
    jdqd[3] = -0.33664707*cos(mbs_data->q[R1_RL_id])*mbs_data->qd[R1_RL_id]*mbs_data->qd[R1_RL_id];
    jdqd[4] =  0.33664707*cos(mbs_data->q[R1_RR_id])*mbs_data->qd[R1_RR_id]*mbs_data->qd[R1_RR_id];
}