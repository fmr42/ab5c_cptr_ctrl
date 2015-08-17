#include <iostream>
#include <armadillo>

#include "../ctypes/ctype_cptr_ctrlaction.h"
#include "../ctypes/ctype_cptr_ctrl_params.h"
#include "../ctypes/ctype_cptr_ctrl_state.h"
#include "../ctypes/ctype_cptr_model.h"
#include "../ctypes/ctype_cptr_sp.h"
#include "../ctypes/ctype_cptr_state.h"

#include "../cblx/cptr_ekf_predict/cptr_ekf_predict.hpp"

using namespace arma;
using namespace std;

int main (){

  struct cptr_ctrlaction_t   ctrl_action ;
  /*
  // Set ctrl params
  struct cptr_ctrl_params_t  ctrl_params ;
  ctrl_params.l1 ;
  ctrl_params.l2 ;
  ctrl_params.k1 ;
  ctrl_params.k2 ;
  ctrl_params.kp ;
  ctrl_params.kd ;
  ctrl_params.max_thrust     ;
  ctrl_params.max_torque_x   ;
  ctrl_params.max_torque_y   ;
  ctrl_params.max_torque_z   ;
  ctrl_params.hysteresis_thr ;

  // Init ctrl state
  struct cptr_ctrl_state_t   ctrl_state  ;
  ctrl_state.hysteresis = 0 ;
  ctrl_state.Qc0_old[0] = 0 ;
  ctrl_state.Qc0_old[1] = 0 ;
  ctrl_state.Qc0_old[2] = 0 ;
  ctrl_state.Qc0_old[3] = 0 ;
  ctrl_state.Qc1_old[0] = 0 ;
  ctrl_state.Qc1_old[1] = 0 ;
  ctrl_state.Qc1_old[2] = 0 ;
  ctrl_state.Qc1_old[3] = 0 ;


  struct cptr_sp_t {

 cptr_ctrl ( &ctrl_action ,
 struct cptr_sp_t                    *sp          ,
 struct cptr_state_t                 *state       ,
 struct cptr_ctrl_state_t            *ctrl_state  ,
 struct cptr_model_t                 *cptr_model  ,
 struct cptr_ctrl_params_t           *params      );

*/


/* ==================================     INIT STATE    =========================*/
struct cptr_state_t        state  ;

vec arma_linpos = vec ( state.linpos.mem , 3 , false , true ); arma_linpos.zeros() ;
vec arma_linvel = vec ( state.linvel.mem , 3 , false , true ); arma_linvel.zeros() ;
vec arma_angpos = vec ( state.angpos.mem , 3 , false , true ); arma_angpos.zeros() ; arma_angpos(0) = 1 ;
vec arma_angvel = vec ( state.angvel.mem , 3 , false , true ); arma_angvel.zeros() ;

mat arma_linpos_linpos_crosscov = mat (state.linpos_linpos_crosscov.mem , 3 , 3 , false , true ); 
mat arma_linpos_linvel_crosscov = mat (state.linpos_linvel_crosscov.mem , 3 , 3 , false , true );
mat arma_linpos_angpos_crosscov = mat (state.linpos_angpos_crosscov.mem , 3 , 4 , false , true );
mat arma_linpos_angvel_crosscov = mat (state.linpos_angvel_crosscov.mem , 3 , 4 , false , true );

mat arma_linvel_linvel_crosscov = mat (state.linvel_linvel_crosscov.mem , 3 , 3 , false , true );
mat arma_linvel_angpos_crosscov = mat (state.linvel_angpos_crosscov.mem , 3 , 4 , false , true );
mat arma_linvel_angvel_crosscov = mat (state.linvel_angvel_crosscov.mem , 3 , 4 , false , true );

mat arma_angpos_angpos_crosscov = mat (state.angpos_angpos_crosscov.mem , 4 , 4 , false , true );
mat arma_angpos_angvel_crosscov = mat (state.angpos_angvel_crosscov.mem , 4 , 4 , false , true );

mat arma_angvel_angvel_crosscov = mat (state.angvel_angvel_crosscov.mem , 4 , 4 , false , true );

arma_linpos_linpos_crosscov.eye();
arma_linpos_linvel_crosscov.eye();
arma_linpos_angpos_crosscov.eye();
arma_linpos_angvel_crosscov.eye();
arma_linvel_linvel_crosscov.eye();
arma_linvel_angpos_crosscov.eye();
arma_linvel_angvel_crosscov.eye();
arma_angpos_angpos_crosscov.eye();
arma_angpos_angvel_crosscov.eye();
arma_angvel_angvel_crosscov.eye();
/* ====================================    INIT CTRL ACTION   ======================== */

struct cptr_ctrlaction_t   action ;

action.thrust.data = 10 ;
action.torque.data.x = 0 ;
action.torque.data.y = 0 ;
action.torque.data.z = 0 ;

/* ===================================    INIT TIME AND MODEL    ======================== */


struct cptr_scalar_t       dt     ; dt.data=0.001 ;
struct cptr_model_t        model  ;

model.mass = 1 ;
mat J = mat ( model.inertia , 3 , 3 , false, true) ;
J.eye();

model.proc_noise_linpos = 0 ;
model.proc_noise_linvel = 0 ;
model.proc_noise_angpos = 0 ;
model.proc_noise_angvel = 0 ;

model.gravity    = 9.81 ;
model.max_thrust = 50 ;
model.max_torque[0] = 10 ;
model.max_torque[1] = 10 ;
model.max_torque[2] = 10 ;







/* ========================    START MAIN LOOP    ========================   */


int counter = 0 ;
while (true) {
  cptr_ekf_predict ( &state   , &action , &state , &dt , &model ) ;

  if (counter >=10 ){
    counter = 0;
    arma_linpos.print() ;
  } else {
    counter++;
  }
}

return 0 ;

}
