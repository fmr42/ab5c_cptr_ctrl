#include "cptr_ekf_predict.hpp"

int cptr_ekf_predict (
  struct cptr_state_t                 *state_old     ,
  struct cptr_ctrlaction_t *cptr_input    ,
  struct cptr_state_t                 *state_new     ,
  struct cptr_scalar_t                *sampling_time ,
  struct cptr_model_t                 *cptr_model    ){

  double dt = sampling_time->data ;
  // Measurement unit conversion
  // may be needed in the future,
  // so I copy the value to dt (that is in seconds)

  // Map or declare all necessary variables
  vec e3 = arma::zeros<vec>(3) ;
  e3(2) = 1 ;
  
  vec    Ut     = vec (cptr_input->torque.mem , 3, false, true) ;

  vec Q0_old    = vec (state_old->angpos.mem , 4 , true , true ) ;
  vec Q1_old    = vec (state_old->angvel.mem , 4 , true , true ) ;

  // Old and new data point to the same memory trunk, so copy the old value before overwrite it!!!
  vec P0_old = vec ( state_old->linpos.mem , 3 , true , true) ;
  vec P1_old = vec ( state_old->linvel.mem , 3 , true , true) ;

  // Do _NOT_ copy the data or new values will not be written out!!!
  vec P0_new = vec (state_new->linpos.mem , 3 , false, true) ;
  vec P1_new = vec (state_new->linvel.mem , 3 , false, true) ;
  vec Q0_new = vec (state_new->angpos.mem , 4 , false, true) ;
  vec Q1_new = vec (state_new->angvel.mem , 4 , false, true) ;

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
  W0_new = W0_old + W1_old * dt ;

  P2_old = - cptr_input->thrust.data * ( arma_rodriguez( Q0_old ) * e3 ) / cptr_model->mass + cptr_model->gravity * e3 ; 
  Q2_old = 0.5 * arma_quat_hamilton ( Q1_old , W0_old ) + 0.5 * arma_quat_hamilton ( Q0_old , W1_old )  ;

  P0_new = P0_old + P1_old * dt + 0.5 * P2_old * dt * dt ;
  P1_new = P1_old + P2_old * dt ;
  
  Q0_new = normalise( Q0_old + Q1_old * dt + 0.5 * Q2_old * dt ) ;
  Q1_new = Q1_old + Q2_old * dt ;

  // Generate process noise covariance mat
  //
  mat Q_p0 =  eye (3,3) * cptr_model->proc_noise_linpos ;
  mat Q_p1 =  eye (3,3) * cptr_model->proc_noise_linvel ;
  mat Q_q0 =  eye (4,4) * cptr_model->proc_noise_angpos ;
  mat Q_q1 =  eye (4,4) * cptr_model->proc_noise_angvel ;


  mat covnew_p0p0 = mat ( state_new->linpos_linpos_crosscov.mem , 3 , 3 , false , true ) ;
  mat covnew_p0p1 = mat ( state_new->linpos_linvel_crosscov.mem , 3 , 3 , false , true ) ;
  mat covnew_p0q0 = mat ( state_new->linpos_angpos_crosscov.mem , 3 , 4 , false , true ) ;
  mat covnew_p0q1 = mat ( state_new->linpos_angvel_crosscov.mem , 3 , 4 , false , true ) ;

  mat covnew_p1p1 = mat ( state_new->linvel_linvel_crosscov.mem , 3 , 3 , false , true ) ;
  mat covnew_p1q0 = mat ( state_new->linvel_angpos_crosscov.mem , 3 , 4 , false , true ) ;
  mat covnew_p1q1 = mat ( state_new->linvel_angvel_crosscov.mem , 3 , 4 , false , true ) ;

  mat covnew_q0q0 = mat ( state_new->angpos_angpos_crosscov.mem , 4 , 4 , false , true ) ;
  mat covnew_q0q1 = mat ( state_new->angpos_angvel_crosscov.mem , 4 , 4 , false , true ) ;

  mat covnew_q1q1 = mat ( state_new->angvel_angvel_crosscov.mem , 4 , 4 , false , true ) ;


  mat covold_p0p0 = mat ( state_old->linpos_linpos_crosscov.mem , 3 , 3 , true , true ) ;
  mat covold_p0p1 = mat ( state_old->linpos_linvel_crosscov.mem , 3 , 3 , true , true ) ;
  mat covold_p0q0 = mat ( state_old->linpos_angpos_crosscov.mem , 3 , 4 , true , true ) ;
  mat covold_p0q1 = mat ( state_old->linpos_angvel_crosscov.mem , 3 , 4 , true , true ) ;

  mat covold_p1p1 = mat ( state_old->linvel_linvel_crosscov.mem , 3 , 3 , true , true ) ;
  mat covold_p1q0 = mat ( state_old->linvel_angpos_crosscov.mem , 3 , 4 , true , true ) ;
  mat covold_p1q1 = mat ( state_old->linvel_angvel_crosscov.mem , 3 , 4 , true , true ) ;

  mat covold_q0q0 = mat ( state_old->angpos_angpos_crosscov.mem , 4 , 4 , true , true ) ;
  mat covold_q0q1 = mat ( state_old->angpos_angvel_crosscov.mem , 4 , 4 , true , true ) ;

  mat covold_q1q1 = mat ( state_old->angvel_angvel_crosscov.mem , 4 , 4 , true , true ) ;



  // TODO something wrong with this... transpose should be used somewhere...
  //          | p0p0 p0p1 p0q0 p0q1 |   
  // covnew = |      p1p1 p1q0 p1q1 | = F * covold * F_t + Q    =
  //          |           q0q0 q0q1 |
  //          |                q1q1 |
  //
  //
  //        = 
  //
  //
  //

  covnew_p0p0 = covold_p0p0 + covold_p0p1 * dt + covold_p0p1.t() * dt + covold_p1p1 * dt * dt    + Q_p0  ;
  covnew_p0p1 = covold_p0p1 + covold_p1p1 * dt                                                           ;
  covnew_p0q0 = covold_p0q0 + covold_p0q1 * dt + covold_p1q0     * dt + covold_p1q1 * dt                 ;
  covnew_p0q1 = covold_p0q1 + covold_p1q1 * dt                                                           ;

  covnew_p1p1 = covold_p1p1                                                                       + Q_p1 ;
  covnew_p1q0 = covold_p1q0 + covold_p1q1 * dt                                                           ;
  covnew_p1q1 = covold_p1q1                                                                              ;

  covnew_q0q0 = covold_q0q0 + covold_q0q1 * dt + covold_q0q1.t() * dt + covold_q1q1 * dt * dt   + Q_q0   ;
  covnew_q0q1 = covold_q0q1 + covold_q1q1 * dt                                                           ;

  covnew_q1q1 = covold_q1q1                                                                       + Q_q1 ;

  return 0;

}


