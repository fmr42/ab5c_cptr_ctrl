#include <iostream>
#include <armadillo>

#include "../../ctypes/ctype_cptr_ctrlaction.h"
#include "../../ctypes/ctype_cptr_ctrl_params.h"
#include "../../ctypes/ctype_cptr_ctrl_state.h"
#include "../../ctypes/ctype_cptr_model.h"
#include "../../ctypes/ctype_cptr_sp.h"
#include "../../ctypes/ctype_cptr_state.h"


#include "../../cblx/cptr_ekf_predict/cptr_ekf_predict.hpp"
#include "../../cblx/cptr_ctrl/cptr_ctrl.hpp"
using namespace arma;
using namespace std;

int main (){


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



/* =================================== INIT CTRL STATE =============================*/

struct cptr_ctrl_state_t ctrl_state;
ctrl_state.hysteresis = 1 ;

ctrl_state.Qc0_old[0] = 0 ;
ctrl_state.Qc0_old[1] = 0 ;
ctrl_state.Qc0_old[2] = 0 ;
ctrl_state.Qc0_old[3] = 0 ;


ctrl_state.Qc1_old[0] = 0 ;
ctrl_state.Qc1_old[1] = 0 ;
ctrl_state.Qc1_old[2] = 0 ;
ctrl_state.Qc1_old[3] = 0 ;

/* =================================== INIT CTRL PARAMS =============================*/


struct cptr_ctrl_params_t ctrl_params ;

ctrl_params.l1 = 5 ;
ctrl_params.l2 = 10 ;
ctrl_params.k1 = 0.7 ;
ctrl_params.k2 = 10 ;
ctrl_params.kp = 40 ;
ctrl_params.kd = 10 ;

ctrl_params.max_thrust     = 20 ;
ctrl_params.max_torque_x   = 3 ;
ctrl_params.max_torque_y   = 3 ;
ctrl_params.max_torque_z   = 3 ;
ctrl_params.hysteresis_thr = 0.01 ;

/* ===================================    INIT TIME AND MODEL    ======================== */


struct cptr_scalar_t       dt     ; dt.data=0.001 ;
struct cptr_model_t        model  ;

model.mass = 1 ;
mat J = mat ( model.inertia , 3 , 3 , false, true) ;
J = eye<mat>(3,3) * 0.01;

model.proc_noise_linpos = 0 ;
model.proc_noise_linvel = 0 ;
model.proc_noise_angpos = 0 ;
model.proc_noise_angvel = 0 ;

model.gravity    = 9.81 ;
model.max_thrust = 50 ;
model.max_torque[0] = 10 ;
model.max_torque[1] = 10 ;
model.max_torque[2] = 10 ;



/* ==========================   INIT SP   ==============================*/

struct cptr_sp_t sp ;
vec linpos_sp = vec ( sp.linpos_sp.mem , 3 , false , true );
vec linvel_sp = vec ( sp.linvel_sp.mem , 3 , false , true );
vec linacc_sp = vec ( sp.linacc_sp.mem , 3 , false , true );
vec angpos_sp = vec ( sp.angpos_sp.mem , 4 , false , true );
vec angvel_sp = vec ( sp.angvel_sp.mem , 4 , false , true );
vec angacc_sp = vec ( sp.angacc_sp.mem , 4 , false , true );


linpos_sp.ones() ;
linvel_sp.zeros() ;
linacc_sp.zeros() ;

angpos_sp.zeros() ; angpos_sp(0)=-1 ;
angvel_sp.zeros() ;
angacc_sp.zeros() ;

/* ========================    START MAIN LOOP    ========================   */


  int counter = 0 ;
  while (true) {
    cptr_ctrl (&action , &sp , &state , &ctrl_state , &model  , &ctrl_params      );

  cptr_ekf_predict ( &state   , &action , &state , &dt , &model ) ;

  if (counter >=100 ){
    counter = 0;
    arma_linpos.print() ;
    cout << "------------------------- " <<  endl;
  } else {
    counter++;
  }
}

return 0 ;

}
