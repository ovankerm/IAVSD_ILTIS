/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for direct dynamics module
 * 
 * This files enable the user to call custom at
 * specific places in the time simulation. It is a template
 * file that can be edited by the user.
 * 
 * (c) Universite catholique de Louvain
 *     
 */

#include "math.h"
#include "mbs_data.h"
#include "set_output.h"
#include "mbs_dirdyn_struct.h"
#include "user_all_id.h"
#include "mbs_sensor_Jeep.h"


/*! \brief user own initialization functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_init(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{
    define_output_vector("Ground_Forces", 4);
    define_output_vector("Wheel_height", 4);
    define_output_vector("Leaf_Forces", 4);
    define_output_vector("Spring_Forces", 4);
}

/*! \brief user own loop functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_loop(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{
    if (mbs_data->process == 3) {

        int id = Sensor_Conducteur_id;
        
        // retrieve the pointer to the sensor structure defined in mbs_aux
        MbsSensor *PtrSensor = mbs_dd->mbs_aux->psens;
        
        // compute the sensor (position, velocity...)
        mbs_comp_S_sensor(PtrSensor, mbs_data, id);

        // save the acceleration 
        set_output(PtrSensor->A[1], "Horizontal_Acc");
        set_output(PtrSensor->A[2], "Lateral_Acc");
        set_output(PtrSensor->A[3], "Vertical_Acc");
    }
        


    
}

/*! \brief user own finishing functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_finish(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

}
