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
#include "user_model.h"
#include "user_all_id.h"


void user_DrivenJoints(MbsData *mbs_data,double tsim)
{
    UserModel *um = mbs_data->user_model;
    if (um->Status.Steering && !um->Status.Steering_sinus){
        if (tsim<0.5){
            // Id of the steering rack: T2_Steering_Rack_id
            mbs_data->q[T2_Steering_Rack_id]  = 4.0*tsim/1000.;
            mbs_data->qd[T2_Steering_Rack_id] = 4.0/1000.;
        }
        else{
            mbs_data->q[T2_Steering_Rack_id]  = 2.0/1000.;
            mbs_data->qd[T2_Steering_Rack_id] = 0;
        }
    }
    else if (um->Status.Steering && um->Status.Steering_sinus){
        mbs_data->q[T2_Steering_Rack_id] = 0.001*um->Status.Steering_sinus*sin(2*M_PI*tsim);
        mbs_data->qd[T2_Steering_Rack_id] = 0.001*um->Status.Steering_sinus*cos(2*M_PI*tsim)*2*M_PI;
        mbs_data->qdd[T2_Steering_Rack_id] = -0.001*um->Status.Steering_sinus*sin(2*M_PI*tsim)*2*M_PI*2*M_PI;
    }
}

 
