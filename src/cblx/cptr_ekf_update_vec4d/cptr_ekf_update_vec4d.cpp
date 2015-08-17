#include "cptr_ekf_update_vec4d.hpp"


int cptr_ekf_update_vec4d (
 struct cptr_state_t                      *state_old     ,
 struct cptr_state_t                      *state_new     ,
 struct cptr_obsmodel_state_to_vec4d_t    *obsmodel      ,
 struct cptr_sensdata_4d_t                *sensdata      ){


  // Old and new data point to the same memory trunk, so copy the old value before overwrite it!!!
  vec P0_old = vec ( state_old->linpos.mem , 3 , true  , true ) ;
  vec P1_old = vec ( state_old->linvel.mem , 3 , true  , true ) ;
  vec Q0_old = vec ( state_old->angpos.mem , 4 , true  , true ) ;
  vec Q1_old = vec ( state_old->angvel.mem , 4 , true  , true ) ;

  // Do _NOT_ copy the data or new values will not be written out!!!
  vec P0_new = vec ( state_new->linpos.mem , 3 , false , true) ;
  vec P1_new = vec ( state_new->linvel.mem , 3 , false , true) ;
  vec Q0_new = vec ( state_new->angpos.mem , 4 , false , true) ;
  vec Q1_new = vec ( state_new->angvel.mem , 4 , false , true) ;


  vec z = vec ( sensdata->vec.mem , 4 , false , true );

  // import observation model
/*  mat H_linpos_to_z = mat ( obsmodel->linpos_to_vec4d.mem , 4 , 3 , false , true ) ;
  mat H_linvel_to_z = mat ( obsmodel->linvel_to_vec4d.mem , 4 , 3 , false , true ) ;
  mat H_angpos_to_z = mat ( obsmodel->angpos_to_vec4d.mem , 4 , 4 , false , true ) ;
  mat H_angvel_to_z = mat ( obsmodel->angvel_to_vec4d.mem , 4 , 4 , false , true ) ;
*/
// With number is easier..
//
  // H = [ H1 H2 H3 H4 ]
  mat H1 = mat ( obsmodel->linpos_to_vec4d.mem , 4 , 3 , false , true ) ;
  mat H2 = mat ( obsmodel->linvel_to_vec4d.mem , 4 , 3 , false , true ) ;
  mat H3 = mat ( obsmodel->angpos_to_vec4d.mem , 4 , 4 , false , true ) ;
  mat H4 = mat ( obsmodel->angvel_to_vec4d.mem , 4 , 4 , false , true ) ;


  // Compute innovation
  vec y = z - ( H1 * P0_old + H2 * P1_old + H3 * Q0_old + H4 * Q1_old ) ;

  // Import covariance mat old
  // TODO move this to the top
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



  // Map covariance matrix new

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



  // Import sensor data covariance
  mat R = mat ( sensdata->cov.mem , 4 , 4 , false , true );

  // Compute innovation covariance

  mat S = H1 * ( covold_p0p0     * H1.t() + covold_p0p1     * H2.t() + covold_p0q0     * H3.t() + covold_p0q1 * H4.t() ) +
          H2 * ( covold_p0p1.t() * H1.t() + covold_p1p1     * H2.t() + covold_p1q0     * H3.t() + covold_p1q1 * H4.t() ) +
          H3 * ( covold_p0q0.t() * H1.t() + covold_p1q0.t() * H2.t() + covold_q0q0     * H3.t() + covold_q0q1 * H4.t() ) +
          H4 * ( covold_p0q1.t() * H1.t() + covold_p1q1.t() * H2.t() + covold_q0q1.t() * H3.t() + covold_q1q1 * H4.t() ) ;

  // Compute Kalman gain
  //
  // N.B.    H_t = transpose(H)
  //
  //     | K1 |                     -1     | p0p0   p0p1   p0q0   p0q1 |   ( | H1_t |    -1 )
  // K = | K2 | = covold * ( H_t * S   ) = | p0p1_t p1p1   p1q0   p1q1 | * ( | H2_t | * s   )
  //     | K3 |                            | p0q0_t p1q0_t q0q0   q0q1 |   ( | H3_t |       )
  //     | K4 |                            | p0q1_t p1q1_t q0q1_t q1q1 |   ( | H4_t |       )
  //
  //                                                                       \______/\/\______/
  //
  //                                                                            | tM1 |
  //                                                                            | tM2 |
  //                                                                            | tM3 |
  //                                                                            | tM4 |
  //
  //

  mat S_i = S.i(); // Do not recompute inverse multiple time..
  
  mat tM1 = H1.t() * S_i ; // tmp mat 1
  mat tM2 = H2.t() * S_i ; // tmp mat 2
  mat tM3 = H3.t() * S_i ; // tmp mat 3
  mat tM4 = H4.t() * S_i ; // tmp mat 4
  
  mat K1 = covold_p0p0     * tM1 + covold_p0p1     * tM2 + covold_p0q0     * tM3 + covold_p0q1 * tM4 ;
  mat K2 = covold_p0p1.t() * tM1 + covold_p1p1     * tM2 + covold_p1q0     * tM3 + covold_p1q1 * tM4 ;
  mat K3 = covold_p0q0.t() * tM1 + covold_p1q0.t() * tM2 + covold_q0q0     * tM3 + covold_q0q1 * tM4 ;
  mat K4 = covold_p0q1.t() * tM1 + covold_p1q1.t() * tM2 + covold_q0q1.t() * tM3 + covold_q1q1 * tM4 ;

  // Update state
  
  P0_new = P0_old + K1 * y ;
  P1_new = P1_old + K2 * y ;
  Q0_new = Q0_old + K3 * y ;
  Q1_new = Q1_old + K4 * y ;

  // Update covariance
  //
  //
  //
  // covnew = (  I - K*H ) * covold
  //          \___/\/\___/
  //              IKH
  //
  // tM1 and tM2 are tmp support matrix used to compute the value of the diagonal block of IKH (ie IKH_11 , IKH_22 , ecc)

  tM1 = K1*H1 ;
  tM2.copy_size ( tM1 ) ; tM2.eye() ;
  mat IKH_11 = tM2 - tM1 ;

  tM1 = K2*H2 ;
  tM2.copy_size ( tM1 ) ; tM2.eye() ;
  mat IKH_22 = tM2 - tM1 ;

  tM1 = K3*H3 ;
  tM2.copy_size ( tM1 ) ; tM2.eye() ;
  mat IKH_33 = tM2 - tM1 ;

  tM1 = K4*H4 ;
  tM2.copy_size ( tM1 ) ; tM2.eye() ;
  mat IKH_44 = tM2 - tM1 ;
  
  mat IKH_12 = - K1 * H2 ;
  mat IKH_13 = - K1 * H3 ;
  mat IKH_14 = - K1 * H4 ;

  mat IKH_21 = - K2 * H1 ;
  mat IKH_23 = - K2 * H3 ;
  mat IKH_24 = - K2 * H4 ;

  mat IKH_31 = - K3 * H1 ;
  mat IKH_32 = - K3 * H2 ;
  mat IKH_34 = - K3 * H4 ;

  mat IKH_41 = - K4 * H1 ;
  mat IKH_42 = - K4 * H2 ;
  mat IKH_43 = - K4 * H3 ;

  covnew_p0p0 = IKH_11 * covold_p0p0 + IKH_12 * covold_p0p1.t() + IKH_13 * covold_p0q0.t() + IKH_14 * covold_p0q1.t() ;
  covnew_p0p1 = IKH_11 * covold_p0p1 + IKH_12 * covold_p1p1     + IKH_13 * covold_p1q0.t() + IKH_14 * covold_p1q1.t() ;
  covnew_p0q0 = IKH_11 * covold_p0q0 + IKH_12 * covold_p1q0     + IKH_13 * covold_q0q0     + IKH_14 * covold_q0q1.t() ;
  covnew_p0q1 = IKH_11 * covold_p0q1 + IKH_12 * covold_p1q1     + IKH_13 * covold_q0q1     + IKH_14 * covold_q1q1     ;

  covnew_p1p1 = IKH_21 * covold_p0p1 + IKH_22 * covold_p1p1     + IKH_23 * covold_p1q0.t() + IKH_24 * covold_p1q1.t() ;
  covnew_p1q0 = IKH_21 * covold_p0q0 + IKH_22 * covold_p1q0     + IKH_23 * covold_q0q0     + IKH_24 * covold_q0q1.t() ;
  covnew_p1q1 = IKH_21 * covold_p0q1 + IKH_22 * covold_p1q1     + IKH_23 * covold_q0q1     + IKH_24 * covold_q1q1     ;

  covnew_p1q0 = IKH_31 * covold_p0q0 + IKH_32 * covold_p1q0     + IKH_33 * covold_q0q0     + IKH_34 * covold_q0q1.t() ;
  covnew_p1q1 = IKH_31 * covold_p0q1 + IKH_32 * covold_p1q1     + IKH_33 * covold_q0q1     + IKH_34 * covold_q1q1     ;

  covnew_q1q1 = IKH_41 * covold_p0q1 + IKH_42 * covold_p1q1     + IKH_43 * covold_q0q1     + IKH_44 * covold_q1q1     ;

  return 0;

}


