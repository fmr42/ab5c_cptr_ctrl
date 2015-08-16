#include "cptr_ekf_update.hpp"

int cptr_ekf_predict (
  struct cptr_state_t                 *state_old  ,
  struct cptr_state_t                 *state_new  ,
  struct cptr_sensdata_t              *sensdata   ){


  // Old and new data point to the same memory trunk, so copy the old value before overwrite it!!!
  vec P0_old = vec ( state_old->cptr_state_lin_pos.vec.mem , 3 , true  , true ) ;
  vec P1_old = vec ( state_old->cptr_state_lin_pos.vec.mem , 3 , true  , true ) ;
  vec Q0_old = vec ( state_old->cptr_state_ang_pos.vec.mem , 4 , true  , true ) ;
  vec Q1_old = vec ( state_old->cptr_state_ang_vel.vec.mem , 4 , true  , true ) ;

  // Do _NOT_ copy the data or new values will not be written out!!!
  vec P0_new = vec ( state_new->cptr_state_lin_pos.vec.mem , 3 , false , true) ;
  vec P1_new = vec ( state_new->cptr_state_lin_vel.vec.mem , 3 , false , true) ;
  vec Q0_new = vec ( state_new->cptr_state_ang_pos.vec.mem , 4 , false , true) ;
  vec Q1_new = vec ( state_new->cptr_state_ang_vel.vec.mem , 4 , false , true) ;

  // Do _NOT_ copy the data or new values will not be written out!!!
  vec P0_sd = vec ( sensdata->cptr_state_lin_pos.vec.mem , 3 , false  , true) ;
  vec P1_sd = vec ( sensdata->cptr_state_lin_vel.vec.mem , 3 , false  , true) ;
  vec Q0_sd = vec ( sensdata->cptr_state_ang_pos.vec.mem , 4 , false  , true) ;
  vec Q1_sd = vec ( sensdata->cptr_state_ang_vel.vec.mem , 4 , false  , true) ;



  mat COV_old = zeros (14,14) ;
  COV_old.submat(0,0,2,2)     = mat ( state_old->cptr_state_lin_pos.cov.mem , 3 , 3 , true , true ) ;
  COV_old.submat(3,3,5,5)     = mat ( state_old->cptr_state_lin_vel.cov.mem , 3 , 3 , true , true ) ;
  COV_old.submat(6,6,9,9)     = mat ( state_old->cptr_state_ang_pos.cov.mem , 4 , 4 , true , true ) ;
  COV_old.submat(10,10,13,13) = mat ( state_old->cptr_state_ang_vel.cov.mem , 4 , 4 , true , true ) ;
  // state transition matrix
  mat A = eye<mat>( 14,14 ) ;
  A.submat(0,3,2,5)   = eye(3,3) * dt ;
  A.submat(6,12,9,15) = eye(4,4) * dt ;

  mat COV_new = zeros (14,14) ;
  COV_new.submat(0,0,2,2)     = mat ( state_new->cptr_state_lin_pos.cov.mem , 3 , 3 , false , true ) ;
  COV_new.submat(3,3,5,5)     = mat ( state_new->cptr_state_lin_vel.cov.mem , 3 , 3 , false , true ) ;
  COV_new.submat(6,6,9,9)     = mat ( state_new->cptr_state_ang_pos.cov.mem , 4 , 4 , false , true ) ;
  COV_new.submat(10,10,13,13) = mat ( state_new->cptr_state_ang_vel.cov.mem , 4 , 4 , false , true ) ;
 

  mat Q = zeros<mat>(14,14);
  Q.submat(0,0,2,2)     =  eye (3,3) * cptr_model->proc_noise_lin_pos ;
  Q.submat(3,3,5,5)     =  eye (3,3) * cptr_model->proc_noise_lin_vel ;
  Q.submat(6,6,9,9)     =  eye (4,4) * cptr_model->proc_noise_ang_pos ;
  Q.submat(10,10,13,13) =  eye (4,4) * cptr_model->proc_noise_ang_vel ;

  COV_new  = A * COV_old * A.i() + Q ;

  return 0;

}


