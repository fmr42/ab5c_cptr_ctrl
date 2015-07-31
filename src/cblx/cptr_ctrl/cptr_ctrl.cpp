#include "cptr_ctrl.hpp"


int cptr_ctrl (
  struct cptr_vectored_thrust_input_t *action      ,
  struct cptr_sp_t                    *sp          ,
  struct cptr_state_t                 *state       ,
  struct cptr_ctrl_state_t            *ctrl_state  ,
  struct copter_model_t               *cptr_model  ,
  struct cptr_ctrl_params_t           *params      ) {
  
  vec e3 = zeros<vec>(3); e3(2) = 1;
  mat J  = mat ( cptr_model->inertia , 3 , 3 , false , true );
  
  vec Pr0 = vec ( sp->cptr_sp_lin_pos.data_vec , 3 , false , true );
  vec Pr1 = vec ( sp->cptr_sp_lin_vel.data_vec , 3 , false , true );
  vec Pr2 = vec ( sp->cptr_sp_lin_acc.data_vec , 3 , false , true );
  vec Qr0 = vec ( sp->cptr_sp_ang_pos.data_vec , 4 , false , true );
  vec Qr1 = vec ( sp->cptr_sp_ang_vel.data_vec , 4 , false , true );
  vec Qr2 = vec ( sp->cptr_sp_ang_acc.data_vec , 4 , false , true );

  vec P0 = vec( state->cptr_state_lin_pos.data_vec , 3 , false , true );
  vec P1 = vec( state->cptr_state_lin_vel.data_vec , 3 , false , true );
  vec Q0 = vec( state->cptr_state_ang_pos.data_vec , 4 , false , true );
  vec Q1 = vec( state->cptr_state_ang_vel.data_vec , 4 , false , true );

  vec Qc0_old = vec ( ctrl_state->Qc0_old , 4 , false , true );
  vec Qc1_old = vec ( ctrl_state->Qc0_old , 4 , false , true );

  vec Ut = vec ( action->torque , 3 , false , true );
  
  vec W0 = zeros<vec>(4) ;
  vec W1 = zeros<vec>(4) ;
  vec Wc0 = zeros<vec>(4) ;
  vec Wc1 = zeros<vec>(4) ;

  // Computer thrust reference (4)
  // TODO Vcr0 should be saturated
  vec Vcr0 = cptr_model->mass * ( cptr_model->gravity * e3 - Pr2 );
  vec k0 = params->l2 * sat ( ( params->k2 / params->l2 ) * ( ( P1 - Pr1 ) + params->l1 * sat ( (params->k1 / params->l1) * ( P0 - Pr0 ) ) ) ) ;

  // Compute control thrust
  vec Vc0 = Vcr0 + k0 ;
  // Compute thrust
  action->thrust = norm ( Vc0 , 2 ) ;
  
  // Compute control orientation 			
  mat Rc0 = zeros<mat>(3,3);
  mat Rr0 = arma_rodriguez(Qr0);

  Rc0.col(2) = normalise ( Vc0 );
  Rc0.col(0) = normalise ( cross ( Rr0.col(1) , Rc0.col(2)  ) ) ;
  Rc0.col(1) = normalise ( cross ( Rc0.col(2) , Rc0.col(0)  ) ) ;

  mat Qc0 = arma_mat_to_q(Rc0);
  mat Qc1 = Qc0 - Qc0_old ;
  mat Qc2 = Qc1 - Qc1_old ;
	
  Wc0.subvec(1,3) = arma_quat_d_to_vel( Qc0 , Qc1 );
  //Wc1.subvec(1,3) = arma_quat_d_to_acc( Qc0 , Qc1 , Qc2 );
  // Compute error coordinates
  
  vec Qe0  = arma_quat_hamilton( arma_quat_inv(Qc0) , Q0 );
  
  W0.subvec(1,3) = arma_quat_d_to_vel (Q0 , Q1 );
  mat Re = arma_rodriguez(Qe0);
  vec Wce0 = zeros<vec>(3)         ;
  (Wce0).subvec(1,3) = Re.t() * (Wc0).subvec(1,3);
  vec We0  = W0 - Wce0 ;

  // Update hysteresis
  if ( ctrl_state->hysteresis * Qe0(0) <= params->hysteresis_thr ) {
    ctrl_state->hysteresis = copysign ( 1 , Qe0(0) ) ; // TODO h=-h should be the same and more efficent
  }

  // Compute control action
  vec UtFF = J * ( Re.t() * Wc1.subvec(1,3) ) - cross( J * Wce0.subvec(1,3) , Wce0.subvec(1,3) ) ;
  vec UtFB = - params->kp * ctrl_state->hysteresis * Qe0.subvec(1,3) - params->kd * We0.subvec(1,3) ;
  Ut = UtFB + UtFF ;

  if ( action->thrust >= 0 ) {
    action->thrust = std::min ( +cptr_model->max_thrust , action->thrust ) ;
  }else{
    action->thrust = std::max ( -cptr_model->max_thrust , action->thrust ) ;
  }
  
  for (int i = 0 ; i<3 ; i++ ){
    if ( Ut(i) >= 0 ) {
      Ut(i) = std::min ( +cptr_model->max_torque[i] , Ut(i) ) ;
    }else{
      Ut(i) = std::max ( -cptr_model->max_torque[i] , Ut(i) ) ;
    }
  }

  // Update ctrl state for next iteration
  Qc0_old = Qc0 ;
  Qc1_old = Qc1 ;
 
  return 0;

}









