#ifndef _CPTR_EKF_PREDICT_H_
#define _CPTR_EKF_PREDICT_H_

#include <iostream>
#include <ctime>
#include <armadillo>
#include "../../ctypes/cptr_state-ctype.h"
#include "../../ctypes/cptr_vectores_thrust_input-ctype.h"
#include "../../ctypes/cptr_model-ctype.h"
#include "../../lib/arma_quaternion_lib.cpp"

#define UBX_NO_DEBUG
//#define ARMA_NO_DEBUG

#ifdef __cplusplus
 extern "C" {
#endif


int cptr_ekf_predict (
 struct cptr_state_t                 *state_old   ,
 struct cptr_state_t                 *state_new   ,
 struct copter_model_t               *cptr_model  );

#ifdef __cplusplus
 }
#endif


#endif

