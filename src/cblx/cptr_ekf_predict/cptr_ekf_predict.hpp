#ifndef _CPTR_EKF_PREDICT_H_
#define _CPTR_EKF_PREDICT_H_

#include <iostream>
#include <ctime>
#include <armadillo>
#include "../../ctypes/ctype_cptr_state.h"
#include "../../ctypes/ctype_cptr_vectored_thrust_input.h"
#include "../../ctypes/ctype_cptr_model.h"
#include "../../ctypes/ctype_cptr_time.h"
#include "../../lib/arma_quaternion_lib.cpp"

#define UBX_NO_DEBUG
//#define ARMA_NO_DEBUG

#ifdef __cplusplus
 extern "C" {
#endif


int cptr_ekf_predict (
 struct cptr_state_t                 *state_old      ,
 struct cptr_vectored_thrust_input_t *cptr_input     ,
 struct cptr_state_t                 *state_new      ,
 struct cptr_time_t                  *sampling_time  ,
 struct copter_model_t               *cptr_model    );

#ifdef __cplusplus
 }
#endif


#endif

