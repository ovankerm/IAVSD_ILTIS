   /**
    *
    *   Universite catholique de Louvain
    *   CEREM : Centre for research in mechatronics
    *   http://www.robotran.be
    *   Contact : info@robotran.be
    *
    *
    *   MBsysC main script template for simple model:
    *   -----------------------------------------------
    *    This template loads the data file *.mbs and execute:
    *      - the coordinate partitioning module
    *      - the direct dynamic module (time integration of
    *        equations of motion).
    *    It may be adapted and completed by the user.
    *
    *    (c) Universite catholique de Louvain
    *
    * To turn this file as a C++ file, just change its extension to .cc (or .cpp).
    * If you plan to use some C++ files, it is usually better that the main is compiled as a C++ function.
    * Currently, most compilers do not require this, but it is a safer approach to port your code to other computers.
    */

#include <stdio.h>
#include "mbs_set.h"
#include "mbs_data.h"
#include "mbs_part.h"
#include "realtime.h"
#include "mbs_equil.h"
#include "mbs_modal.h"
#include "mbs_dirdyn.h"
#include "user_model.h"
#include "user_all_id.h"
#include "mbs_load_xml.h"
#include "cmake_config.h"
#include "useful_functions.h"
#include "mbs_project_interface.h"
#include "math.h"

int main(int argc, char const *argv[])

{
    MbsData *mbs_data;
    MbsPart *mbs_part;
    MbsEquil *mbs_equil;
    MbsDirdyn *mbs_dirdyn;

    printf("Starting Jeep MBS project!\n");


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                     LOADING                               *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    printf("Loading the Jeep data file !\n");
    mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Jeep.mbs", BUILD_PATH);
    printf("*.mbs file loaded!\n");

    /* Setting user constraints */
    mbs_set_nb_userc(mbs_data, 4);
    // 1 constraint (lateral position) for each Leaf Spring stop

    /* Put exact (lenght, angle) value (avoid rounding from book) */
    double AarmLength = pow(pow(0.302-0.229,2.0)+pow(0.259-0.572,2.0),0.5);
    mbs_data->dpt[2][FL_AArm_AArm_length_id]  =  AarmLength;
    mbs_data->dpt[2][RRL_AArm_AArm_length_id] =  AarmLength;
    mbs_data->dpt[2][FR_AArm_AArm_length_id]  = -AarmLength;
    mbs_data->dpt[2][RRR_AArm_AArm_length_id] = -AarmLength;
    double Carrier1Length = pow(pow(0.531-0.229,2.0)+pow(0.488-0.572,2.0),0.5);
    mbs_data->dpt[3][FL_Carrier1_AArm_attachement_carrier1_id]  = -Carrier1Length;
    mbs_data->dpt[3][RRL_Carrier1_AArm_attachement_carrier1_id] = -Carrier1Length;
    mbs_data->dpt[3][FR_Carrier1_AArm_attachement_carrier1_id]  = -Carrier1Length;
    mbs_data->dpt[3][RRR_Carrier1_AArm_attachement_carrier1_id] = -Carrier1Length;
    double Carrier2toCarrier3 = Carrier1Length*(0.356-0.229)/(0.531-0.229);
    mbs_data->dpt[3][FL_Carrier2_Carrier2_Carrier3_id]  = -Carrier2toCarrier3;
    mbs_data->dpt[3][RRL_Carrier2_Carrier2_Carrier3_id] = -Carrier2toCarrier3;
    mbs_data->dpt[3][FR_Carrier2_Carrier2_Carrier3_id]  = -Carrier2toCarrier3;
    mbs_data->dpt[3][RRR_Carrier2_Carrier2_Carrier3_id] = -Carrier2toCarrier3;
    double Carrier1toR3 = Carrier1Length-Carrier2toCarrier3;
    mbs_data->dpt[3][FL_Carrier1_Wheel_carrier2_attachement_id]  = -Carrier1toR3;
    mbs_data->dpt[3][RRL_Carrier1_Wheel_carrier2_attachement_id] = -Carrier1toR3;
    mbs_data->dpt[3][FR_Carrier1_Wheel_carrier2_attachement_id]  = -Carrier1toR3;
    mbs_data->dpt[3][RRR_Carrier1_Wheel_carrier2_attachement_id] = -Carrier1toR3;
    double HubCarrierLength = pow(pow(0.488-0.1585,2.0)+pow(0.531-0.6,2.0),0.5);
    mbs_data->dpt[2][FL_Leaf_spring_LeafSpringLength_id]  = -HubCarrierLength;
    mbs_data->dpt[2][RRL_Leaf_spring_LeafSpringLength_id] = -HubCarrierLength;
    mbs_data->dpt[2][FR_Leaf_spring_LeafSpringLength_id]  =  HubCarrierLength;
    mbs_data->dpt[2][RRR_Leaf_spring_LeafSpringLength_id] =  HubCarrierLength;
    double Carrier3Angle = atan((0.572-0.488)/(0.531-0.229));
    mbs_data->q[R1_FL_Carrier_id] =  Carrier3Angle;
    mbs_data->q[R1_RL_Carrier_id] =  Carrier3Angle;
    mbs_data->q[R1_FR_Carrier_id] = -Carrier3Angle;
    mbs_data->q[R1_RR_Carrier_id] = -Carrier3Angle;
    double initial_angle = atan((0.6-0.531)/(0.488-0.1585));
    mbs_data->q[R1_FL_id] =  initial_angle;
    mbs_data->q[R1_RL_id] =  initial_angle;
    mbs_data->q[R1_FR_id] = -initial_angle;
    mbs_data->q[R1_RR_id] = -initial_angle;


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*              COORDINATE PARTITIONING                      *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_data->process = 1;

    mbs_part = mbs_new_part(mbs_data);
    mbs_part->options->rowperm=1;
    mbs_part->options->verbose = 1;
    mbs_run_part(mbs_part, mbs_data);

    mbs_delete_part(mbs_part);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                   COMPUTING NOMINAL STATE                 *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_data->process = 0;
    mbs_data->NRerr=1e-14; //fine precision
    MbsAux* mbs_aux;
    mbs_aux = initMbsAux(mbs_data);
    // Setting driven joints
    user_DrivenJoints(mbs_data, 0.0);
    // Closing constraints
    mbs_close_geo(mbs_data, mbs_aux);
    //computing forces
    mbs_calc_force(mbs_data);

    // Create Nominal report
    FILE *report;
    report = fopen(PROJECT_SOURCE_DIR"/../resultsR/1_Nominal_Report.txt", "w+");
    fprintf(report, " _____________________________________________________________ \n");
    fprintf(report, "|Element studied                     | Robotran   | Benchmark |\n");
    fprintf(report, "|____________________________________|____________|___________|\n");
    fprintf(report, "|Leaf spring to wheel carrier force  | % 8.2f N | -2728.9 N |\n", mbs_data->user_model->Initial_State.FL_Leaf_F);
    fprintf(report, "|____________________________________|____________|___________|\n");
    fprintf(report, "|Damper spring force                 | % 8.2f N |  -128.0 N |\n", mbs_data->user_model->Initial_State.FL_Spring_F);
    fprintf(report, "|____________________________________|____________|___________|\n");
    fprintf(report, "|Front tyre load                     | % 8.2f N |  3829.6 N |\n", mbs_data->user_model->Initial_State.FL_Wheel_F);
    fprintf(report, "|____________________________________|____________|___________|\n");
    fprintf(report, "|Rear tyre load                      | % 8.2f N |  3593.6 N |\n", mbs_data->user_model->Initial_State.RL_Wheel_F);
    fprintf(report, "|____________________________________|____________|___________|\n");
    fprintf(report, "\n\n\n Other data's\n\n");
    fprintf(report, "Front tyre height :    %.8f m\n", mbs_data->user_model->Initial_State.FL_Wheel_Z);
    fprintf(report, "Rear tyre height :     %.8f m\n", mbs_data->user_model->Initial_State.RL_Wheel_Z);
    fprintf(report, "\n");
    fprintf(report, "Leaf spring length :   %.8f m\n", mbs_data->user_model->Initial_State.FL_Leaf_Z);
    fprintf(report, "Damper spring length : %.8f m\n", mbs_data->user_model->Initial_State.FL_Spring_Z);
    fclose(report);

    mbs_data->NRerr=1e-9; //normal precision


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                   EQUILIBRIUM                             *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_data->process = 2;

    mbs_equil = mbs_new_equil(mbs_data);
    mbs_run_equil(mbs_equil, mbs_data);
    mbs_delete_equil(mbs_equil, mbs_data);

    // Computing Force in driven joint
    dirdynared(mbs_aux, mbs_data);
    mbs_data->user_model->Equilibrium_State.FR_Leaf_Rod_F =-mbs_data->Qc[T2_FR_Leaf_rod_id]; //Minus because compressive is negative (opposite from frame orientation)
    mbs_data->user_model->Equilibrium_State.RR_Leaf_Rod_F =-mbs_data->Qc[T2_RR_Leaf_rod_id]; //Minus because compressive is negative (opposite from frame orientation)

    // Create Equilibrium report
    report = fopen(PROJECT_SOURCE_DIR"/../resultsR/2_Equilibrium_Report.txt", "w+");
    fprintf(report, " ______________________________________________________ \n");
    fprintf(report, "| Interaction (right side)           | Dim. | Robotran |\n");
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Cabin: vert. displacement          |  mm  | % 8.3f |\n"  , (mbs_data->q[Z_Chassis_id]-mbs_data->q0[Z_Chassis_id])*1000);
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Cabin: pitch angle                 |   °  | % 8.4f |\n"  , mbs_data->q[Pitch_Chassis_id]*180/M_PI);
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Front tyre vertical force          |   N  | % 8.2f |\n"  , mbs_data->user_model->Equilibrium_State.FR_Tyre);
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Rear tyre vertical force           |   N  | % 8.2f |\n"  , mbs_data->user_model->Equilibrium_State.RR_Tyre);
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Front shock absorber               |   N  | % 8.2f |\n"  , mbs_data->user_model->Equilibrium_State.FR_Spring);
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Rear shock absorber                |   N  | % 8.2f |\n"  , mbs_data->user_model->Equilibrium_State.RR_Spring);
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Front leaf spring (force element)  |   N  | % 8.2f |\n"  , mbs_data->user_model->Equilibrium_State.FR_Leaf_F);
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Front rod leaf spring              |   N  | % 8.2f |\n"  , mbs_data->user_model->Equilibrium_State.FR_Leaf_Rod_F);
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Rear leaf spring (force element)   |   N  | % 8.2f |\n"  , mbs_data->user_model->Equilibrium_State.RR_Leaf_F);
    fprintf(report, "|____________________________________|______|__________|\n");
    fprintf(report, "| Rear rod leaf spring               |   N  | % 8.2f |\n"  , mbs_data->user_model->Equilibrium_State.RR_Leaf_Rod_F);
    fprintf(report, "|____________________________________|______|__________|\n");
    fclose(report);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                   SAVING EQUIL STATE                      *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    copy_dvec_1(mbs_data->q, mbs_data->q0);
    copy_dvec_1(mbs_data->qd, mbs_data->qd0);
    copy_dvec_1(mbs_data->qdd, mbs_data->qdd0);
    zeros_dvec_1(mbs_data->Qq);
    zeros_dvec_1(mbs_data->Qc);

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                   MODAL ANALYSIS                          *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    MbsModal *mbs_modal;
    mbs_data->process = 4;
    mbs_data->user_model->Status.Linear_Modal=1;

    mbs_set_qu(mbs_data,1);
    mbs_set_qu(mbs_data,2);
    mbs_set_qu(mbs_data,6);
    mbs_data->qd[1] = 20;
    mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;

    mbs_modal = mbs_new_modal(mbs_data);

    // modal options (see documentations for additional options)
    mbs_modal->options->save_result = 1;
    mbs_modal->options->save_anim = 1;
    mbs_modal->options->mode_ampl=0.2;
    mbs_modal->options->verbose = 1;

    mbs_run_modal(mbs_modal, mbs_data);
    mbs_delete_modal(mbs_modal, mbs_data);


    /***********************
        Test PID
    ************************/

    mbs_reset_data(mbs_data);
    mbs_data->process = 3;
    mbs_set_qu(mbs_data,1); // Setting joint 1 to independant
    mbs_set_qu(mbs_data,2); // Setting joint 2 to independant
    mbs_set_qu(mbs_data,6); // Setting joint 6 to independant
    float speed = 10;
    float angle = 1;
    mbs_data->qd[1] = speed * cos(angle * M_PI/180); // Speed in the X direction of the vehicle
    mbs_data->qd[2] = speed * sin(angle * M_PI/180); // Speed in the Y direction of the vehicle
    mbs_data->q[6] = angle * M_PI/180; // Yaw of the vehicle
    mbs_data->q[Y_Chassis_id] = 0; // Y position of the chassis
    mbs_data->qd[J_FR_Wheel_id] = speed/mbs_data->user_model->Wheels.F_Rad; // Rotation speed of the front right wheel
    mbs_data->qd[J_FL_Wheel_id] = speed/mbs_data->user_model->Wheels.F_Rad; // Rotation speed of the front left wheel
    mbs_data->qd[J_RR_Wheel_id] = speed/mbs_data->user_model->Wheels.R_Rad; // Rotation speed of the rear right wheel
    mbs_data->qd[J_RL_Wheel_id] = speed/mbs_data->user_model->Wheels.R_Rad; // Rotation speed of the rear left wheel
    mbs_data->user_model->Status.Bump = 0; // =1 if we have a bump
    mbs_data->user_model->Status.AntiPhase = 0; // =1 if we have a anti-phase bump
    mbs_data->user_model->Status.Steering = 0; // =1 if we are steering
    mbs_data->user_model->Status.Steering_sinus = 0; // =1 if we are steering in a sine wave
    mbs_data->user_model->Status.PID = 1; // =1 if PID engaged 
    mbs_data->user_model->Status.Simple_contact = 0; // =1 if we want the simple contact

    // Setting PID coefficients
    mbs_data->user_model->PID.Kp = 0.55;
    mbs_data->user_model->PID.Kd = 0.3;
    mbs_data->user_model->PID.Ki = 0.01;
    mbs_data->user_model->PID.e_prev = mbs_data->q[Y_Chassis_id];
    mbs_data->user_model->PID.e_sum = 0.0;

    mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // dirdyn options: about output data
    mbs_dirdyn->options->resfilename = "Test_PID";
    mbs_dirdyn->options->show_failed_closure = 1;
    mbs_dirdyn->options->save2file = 1;
    mbs_dirdyn->options->verbose = 1;
    // dirdyn options: about integration time
    mbs_dirdyn->options->tf  = 5.0;
    mbs_dirdyn->options->dt0 = 1e-3;
    // dirdyn options: about integrator
    mbs_dirdyn->options->integrator = Dopri5;
    mbs_dirdyn->options->dt_max = 1e-3;

    mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /* IN-PHASE COSINE BUMP AT 5 m/s normal contact              *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_reset_data(mbs_data);
    mbs_data->process = 3;
    mbs_set_qu(mbs_data,1);
    mbs_set_qu(mbs_data,2);
    mbs_set_qu(mbs_data,6);
    mbs_data->qd[1] = 5;
    mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    mbs_data->user_model->Status.Bump = 1;
    mbs_data->user_model->Status.Simple_contact = 0;

    mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // dirdyn options: about output data
    mbs_dirdyn->options->resfilename = "InPhaseCosine_5ms_normal_contact";
    mbs_dirdyn->options->show_failed_closure = 1;
    mbs_dirdyn->options->save2file = 1;
    mbs_dirdyn->options->verbose = 0;
    // dirdyn options: about integration time
    mbs_dirdyn->options->tf  = 2.5;
    mbs_dirdyn->options->dt0 = 1e-3;
    // dirdyn options: about integrator
    mbs_dirdyn->options->integrator = Dopri5;
    mbs_dirdyn->options->dt_max = 5e-2;

    mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /* IN-PHASE COSINE BUMP AT 5 m/s normal contact              *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    mbs_reset_data(mbs_data);
    mbs_data->process = 3;
    mbs_set_qu(mbs_data,1);
    mbs_set_qu(mbs_data,2);
    mbs_set_qu(mbs_data,6);
    mbs_data->qd[1] = 5;
    mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    mbs_data->user_model->Status.Bump = 1;
    mbs_data->user_model->Status.Simple_contact = 1;

    mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // dirdyn options: about output data
    mbs_dirdyn->options->resfilename = "InPhaseCosine_5ms_simple_contact";
    mbs_dirdyn->options->show_failed_closure = 1;
    mbs_dirdyn->options->save2file = 1;
    mbs_dirdyn->options->verbose = 0;
    // dirdyn options: about integration time
    mbs_dirdyn->options->tf  = 2.5;
    mbs_dirdyn->options->dt0 = 1e-3;
    // dirdyn options: about integrator
    mbs_dirdyn->options->integrator = Dopri5;
    mbs_dirdyn->options->dt_max = 5e-2;

    mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // /* IN-PHASE COSINE BUMP AT 10 m/s                            *
    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // mbs_reset_data(mbs_data);
    // mbs_data->process = 3;
    // mbs_set_qu(mbs_data,1);
    // mbs_set_qu(mbs_data,2);
    // mbs_set_qu(mbs_data,6);
    // mbs_data->qd[1] = 10.0;
    // mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->user_model->Status.Bump = 1;

    // mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // // dirdyn options: about output data
    // mbs_dirdyn->options->resfilename = "InPhaseCosine_10ms";
    // mbs_dirdyn->options->show_failed_closure = 1;
    // mbs_dirdyn->options->save2file = 1;
    // mbs_dirdyn->options->verbose = 1;
    // // dirdyn options: about integration time
    // mbs_dirdyn->options->tf  = 2.5;
    // mbs_dirdyn->options->dt0 = 1e-3;
    // // dirdyn options: about integrator
    // mbs_dirdyn->options->integrator = Dopri5;
    // mbs_dirdyn->options->dt_max = 1e-3;

    // mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    // mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // /* ANTI-PHASE COSINE BUMP AT 5 m/s                           *
    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // mbs_reset_data(mbs_data);
    // mbs_data->process = 3;
    // mbs_set_qu(mbs_data,1);
    // mbs_set_qu(mbs_data,2);
    // mbs_set_qu(mbs_data,6);
    // mbs_data->qd[1] = 5.0;
    // mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->user_model->Status.Bump = 1;
    // mbs_data->user_model->Status.AntiPhase = 1;

    // mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // // dirdyn options: about output data
    // mbs_dirdyn->options->resfilename = "AntiPhaseCosine_5ms";
    // mbs_dirdyn->options->show_failed_closure = 1;
    // mbs_dirdyn->options->save2file = 1;
    // mbs_dirdyn->options->verbose = 1;
    // // dirdyn options: about integration time
    // mbs_dirdyn->options->tf  = 3.0;
    // mbs_dirdyn->options->dt0 = 1e-3;
    // // dirdyn options: about integrator
    // mbs_dirdyn->options->integrator = Dopri5;
    // mbs_dirdyn->options->dt_max = 1e-3;

    // mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    // mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // /* ANTI-PHASE COSINE BUMP AT 10 m/s                          *
    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // mbs_reset_data(mbs_data);
    // mbs_data->process = 3;
    // mbs_set_qu(mbs_data,1);
    // mbs_set_qu(mbs_data,2);
    // mbs_set_qu(mbs_data,6);
    // mbs_data->qd[1] = 10.0;
    // mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->user_model->Status.Bump = 1;
    // mbs_data->user_model->Status.AntiPhase = 1;

    // mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // // dirdyn options: about output data
    // mbs_dirdyn->options->resfilename = "AntiPhaseCosine_10ms";
    // mbs_dirdyn->options->show_failed_closure = 1;
    // mbs_dirdyn->options->save2file = 1;
    // mbs_dirdyn->options->verbose = 1;
    // // dirdyn options: about integration time
    // mbs_dirdyn->options->tf  = 2.0;
    // mbs_dirdyn->options->dt0 = 1e-3;
    // // dirdyn options: about integrator
    // mbs_dirdyn->options->integrator = Dopri5;
    // mbs_dirdyn->options->dt_max = 1e-3;

    // mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    // mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // /* HANDLING TEST: RAMP TO STEER AT 10 m/s                    *
    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // mbs_reset_data(mbs_data);
    // mbs_data->process = 3;
    // mbs_set_qu(mbs_data,1);
    // mbs_set_qu(mbs_data,2);
    // mbs_set_qu(mbs_data,6);
    // mbs_data->qd[1] = 10.0;
    // mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->user_model->Status.Bump = 0;
    // mbs_data->user_model->Status.AntiPhase = 0;
    // mbs_data->user_model->Status.Steering = 1;
    // mbs_data->user_model->Status.Steering_sinus = 0;

    // mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // // dirdyn options: about output data
    // mbs_dirdyn->options->resfilename = "RampToSteer_10ms";
    // mbs_dirdyn->options->show_failed_closure = 1;
    // mbs_dirdyn->options->save2file = 1;
    // mbs_dirdyn->options->verbose = 1;
    // // dirdyn options: about integration time
    // mbs_dirdyn->options->tf  = 5.0;
    // mbs_dirdyn->options->dt0 = 1e-3;
    // // dirdyn options: about integrator
    // mbs_dirdyn->options->integrator = Dopri5;
    // mbs_dirdyn->options->dt_max = 1e-3;

    // mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    // mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // /* HANDLING TEST: RAMP TO STEER AT 20 m/s                    *
    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // mbs_reset_data(mbs_data);
    // mbs_data->process = 3;
    // mbs_set_qu(mbs_data,1);
    // mbs_set_qu(mbs_data,2);
    // mbs_set_qu(mbs_data,6);
    // mbs_data->qd[1] = 20.0;
    // mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->user_model->Status.Bump = 0;
    // mbs_data->user_model->Status.AntiPhase = 0;
    // mbs_data->user_model->Status.Steering = 1;
    // mbs_data->user_model->Status.Steering_sinus = 0;

    // mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // // dirdyn options: about output data
    // mbs_dirdyn->options->resfilename = "RampToSteer_20ms";
    // mbs_dirdyn->options->show_failed_closure = 1;
    // mbs_dirdyn->options->save2file = 1;
    // mbs_dirdyn->options->verbose = 1;
    // // dirdyn options: about integration time
    // mbs_dirdyn->options->tf  = 5.0;
    // mbs_dirdyn->options->dt0 = 1e-3;
    // // dirdyn options: about integrator
    // mbs_dirdyn->options->integrator = Dopri5;
    // mbs_dirdyn->options->dt_max = 1e-3;

    // mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    // mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // /* HANDLING TEST: RAMP TO STEER AT 30 m/s                    *
    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // mbs_reset_data(mbs_data);
    // mbs_data->process = 3;
    // mbs_set_qu(mbs_data,1);
    // mbs_set_qu(mbs_data,2);
    // mbs_set_qu(mbs_data,6);
    // mbs_data->qd[1] = 30.0;
    // mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->user_model->Status.Bump = 0;
    // mbs_data->user_model->Status.AntiPhase = 0;
    // mbs_data->user_model->Status.Steering = 1;
    // mbs_data->user_model->Status.Steering_sinus = 0;

    // mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // // dirdyn options: about output data
    // mbs_dirdyn->options->resfilename = "RampToSteer_30ms";
    // mbs_dirdyn->options->show_failed_closure = 1;
    // mbs_dirdyn->options->save2file = 1;
    // mbs_dirdyn->options->verbose = 1;
    // // dirdyn options: about integration time
    // mbs_dirdyn->options->tf  = 5.0;
    // mbs_dirdyn->options->dt0 = 1e-3;
    // // dirdyn options: about integrator
    // mbs_dirdyn->options->integrator = Dopri5;
    // mbs_dirdyn->options->dt_max = 1e-3;

    // mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    // mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // /* HANDLING TEST: SINUSOIDAL STERRING OF 1 mm                *
    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // mbs_reset_data(mbs_data);
    // mbs_data->process = 3;
    // mbs_set_qu(mbs_data,1);
    // mbs_set_qu(mbs_data,2);
    // mbs_set_qu(mbs_data,6);
    // mbs_data->qd[1] = 20.0;
    // mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->user_model->Status.Bump = 0;
    // mbs_data->user_model->Status.AntiPhase = 0;
    // mbs_data->user_model->Status.Steering = 1;
    // mbs_data->user_model->Status.Steering_sinus = 1;

    // mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // // dirdyn options: about output data
    // mbs_dirdyn->options->resfilename = "SinusoidalSteer_1mm";
    // mbs_dirdyn->options->show_failed_closure = 1;
    // mbs_dirdyn->options->save2file = 1;
    // mbs_dirdyn->options->verbose = 1;
    // // dirdyn options: about integration time
    // mbs_dirdyn->options->tf  = 3.0;
    // mbs_dirdyn->options->dt0 = 1e-3;
    // // dirdyn options: about integrator
    // mbs_dirdyn->options->integrator = Dopri5;
    // mbs_dirdyn->options->dt_max = 1e-3;

    // mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    // mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // /* HANDLING TEST: SINUSOIDAL STERRING OF 2 mm                *
    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // mbs_reset_data(mbs_data);
    // mbs_data->process = 3;
    // mbs_set_qu(mbs_data,1);
    // mbs_set_qu(mbs_data,2);
    // mbs_set_qu(mbs_data,6);
    // mbs_data->qd[1] = 20.0;
    // mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->user_model->Status.Bump = 0;
    // mbs_data->user_model->Status.AntiPhase = 0;
    // mbs_data->user_model->Status.Steering = 1;
    // mbs_data->user_model->Status.Steering_sinus = 2;

    // mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // // dirdyn options: about output data
    // mbs_dirdyn->options->resfilename = "SinusoidalSteer_2mm";
    // mbs_dirdyn->options->show_failed_closure = 1;
    // mbs_dirdyn->options->save2file = 1;
    // mbs_dirdyn->options->verbose = 1;
    // // dirdyn options: about integration time
    // mbs_dirdyn->options->tf  = 3.0;
    // mbs_dirdyn->options->dt0 = 1e-3;
    // // dirdyn options: about integrator
    // mbs_dirdyn->options->integrator = Dopri5;
    // mbs_dirdyn->options->dt_max = 1e-3;

    // mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    // mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    // /* HANDLING TEST: SINUSOIDAL STERRING OF 4 mm                *
    // /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    // mbs_reset_data(mbs_data);
    // mbs_data->process = 3;
    // mbs_set_qu(mbs_data,1);
    // mbs_set_qu(mbs_data,2);
    // mbs_set_qu(mbs_data,6);
    // mbs_data->qd[1] = 20.0;
    // mbs_data->qd[J_FR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_FL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.F_Rad;
    // mbs_data->qd[J_RR_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->qd[J_RL_Wheel_id] = mbs_data->qd[1]/mbs_data->user_model->Wheels.R_Rad;
    // mbs_data->user_model->Status.Bump = 0;
    // mbs_data->user_model->Status.AntiPhase = 0;
    // mbs_data->user_model->Status.Steering = 1;
    // mbs_data->user_model->Status.Steering_sinus = 4;

    // mbs_dirdyn = mbs_new_dirdyn(mbs_data);

    // // dirdyn options: about output data
    // mbs_dirdyn->options->resfilename = "SinusoidalSteer_4mm";
    // mbs_dirdyn->options->show_failed_closure = 1;
    // mbs_dirdyn->options->save2file = 1;
    // mbs_dirdyn->options->verbose = 1;
    // // dirdyn options: about integration time
    // mbs_dirdyn->options->tf  = 3.0;
    // mbs_dirdyn->options->dt0 = 1e-3;
    // // dirdyn options: about integrator
    // mbs_dirdyn->options->integrator = Dopri5;
    // mbs_dirdyn->options->dt_max = 1e-3;

    // mbs_run_dirdyn(mbs_dirdyn, mbs_data);

    // mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    /*                   CLOSING OPERATIONS                      *
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    freeMbsAux(mbs_aux, mbs_data);
    mbs_delete_data(mbs_data);

    return 0;
}

