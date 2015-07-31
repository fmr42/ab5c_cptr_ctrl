#include "cptr_ekf_predict.hpp"

int cptr_ekf_predict (
  struct cptr_state_t                 *state_old   ,
  struct cptr_vectored_thrust_input_t *cptr_input  ,
  struct cptr_state_t                 *state_new   ,
  struct copter_model_t               *cptr_model  )
  {

  // Map/declare all entities
  vec e3 = arma::zeros<vec>(3) ;
  e3(2) = 1 ;
  
  vec    Ut     = vec (cptr_input->torque , 3, false, true) ;

  vec Q0_old    = vec (state_old->cptr_state_ang_pos.data_vec , 4 , true , true ) ;
  vec Q1_old    = vec (state_old->cptr_state_ang_vel.data_vec , 4 , true , true ) ;

  // Old and new data point to the same memory trunk, so copy the old value before overwrite it!!!
  vec P0_old = vec ( state_old->cptr_state_lin_pos.data_vec , 3 , true , true) ;
  vec P1_old = vec ( state_old->cptr_state_lin_pos.data_vec , 3 , true , true) ;

  // Do _NOT_ copy the data or new values will not be written out!!!
  vec P0_new = vec (state_new->cptr_state_lin_pos.data_vec , 3 , false, true) ;
  vec P1_new = vec (state_new->cptr_state_lin_vel.data_vec , 3 , false, true) ;
  vec Q0_new = vec (state_new->cptr_state_ang_pos.data_vec , 4 , false, true) ;
  vec Q1_new = vec (state_new->cptr_state_ang_vel.data_vec , 4 , false, true) ;

  // These will be computed later...
  vec P2_old = zeros<vec>(3) ;
  vec Q2_old = zeros<vec>(4) ;
  vec W0_old = zeros<vec>(4) ;
  vec W1_old = zeros<vec>(4) ;
  
  vec W0_new = zeros<vec>(4) ;
  vec W1_new = zeros<vec>(4) ;
  mat J = mat ( cptr_model->inertia , 3 , 3 , false, true) ;

  // Do computation!!
  (W0_old).subvec(1,3) = arma_quat_d_to_vel ( Q0_old, Q1_old ) ;
  (W1_old).subvec(1,3) = J.i() * ( cross ( J * ( W0_old ).subvec(1,3),( W0_old).subvec(1,3)) + Ut ) ;
  W0_new = W0_old + W1_old * cptr_model->sampling_time ;

  P2_old = - cptr_input->thrust * ( arma_rodriguez( Q0_old ) * e3 ) / cptr_model->mass + cptr_model->gravity * e3 ; 
  Q2_old = 0.5 * arma_quat_hamilton ( Q1_old , W0_old ) + 0.5 * arma_quat_hamilton ( Q0_old , W1_old )  ;

  P0_new = P0_old + P1_old * cptr_model->sampling_time + 0.5 * P2_old * cptr_model->sampling_time * cptr_model->sampling_time;
  P1_new = P1_old + P2_old * cptr_model->sampling_time	;
  
  Q0_new = normalise( Q0_old + Q1_old * cptr_model->sampling_time + 0.5 * Q2_old * cptr_model->sampling_time * cptr_model->sampling_time ) ;
  Q1_new = Q1_old + Q2_old * cptr_model->sampling_time ;



  mat COV_old = zeros (14,14) ;
  COV_old.submat(0,0,2,2)     = mat ( state_old->cptr_state_lin_pos.covariance , 3 , 3 , true , true ) ;
  COV_old.submat(3,3,5,5)     = mat ( state_old->cptr_state_lin_vel.covariance , 3 , 3 , true , true ) ;
  COV_old.submat(6,6,9,9)     = mat ( state_old->cptr_state_ang_pos.covariance , 4 , 4 , true , true ) ;
  COV_old.submat(10,10,13,13) = mat ( state_old->cptr_state_ang_vel.covariance , 4 , 4 , true , true ) ;
  // state transition matrix
  mat A = eye<mat>( 14,14 ) ;
  A.submat(0,3,2,5)   = eye(3,3) * cptr_model->sampling_time  ;
  A.submat(6,12,9,15) = eye(4,4) * cptr_model->sampling_time ;

  mat COV_new = zeros (14,14) ;
  COV_new.submat(0,0,2,2)     = mat ( state_new->cptr_state_lin_pos.covariance , 3 , 3 , false , true ) ;
  COV_new.submat(3,3,5,5)     = mat ( state_new->cptr_state_lin_vel.covariance , 3 , 3 , false , true ) ;
  COV_new.submat(6,6,9,9)     = mat ( state_new->cptr_state_ang_pos.covariance , 4 , 4 , false , true ) ;
  COV_new.submat(10,10,13,13) = mat ( state_new->cptr_state_ang_vel.covariance , 4 , 4 , false , true ) ;
 

  mat Q = zeros<mat>(14,14);
  Q.submat(0,0,2,2)     =  eye (3,3) * cptr_model->proc_noise_lin_pos ;
  Q.submat(3,3,5,5)     =  eye (3,3) * cptr_model->proc_noise_lin_vel ;
  Q.submat(6,6,9,9)     =  eye (4,4) * cptr_model->proc_noise_ang_pos ;
  Q.submat(10,10,13,13) =  eye (4,4) * cptr_model->proc_noise_ang_vel ;

  COV_new  = A * COV_old * A.i() + Q ;

  return 0;

}


