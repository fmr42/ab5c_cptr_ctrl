#ifndef _CPTR_EKF_PREDICT_H_
#define _CPTR_EKF_PREDICT_H_

#include <iostream>
#include <ctime>
#include <armadillo>
#include "../../ctypes/cptr_state-ctype.h"
#include "../../ctypes/cptr_vectores_thrust_input-ctype.h"
#include "../../ctypes/cptr_model-ctype.h"
#include "../../ctypes/cptr_sp-ctype.h"
#include "../../ctypes/cptr_ctrl_state-ctype.h"
#include "../../ctypes/cptr_ctrl_params-ctype.h"
#include "../../lib/arma_quaternion_lib.cpp"
#include "ctrl_functions.cpp"


#define UBX_NO_DEBUG
//#define ARMA_NO_DEBUG

#ifdef __cplusplus
 extern "C" {
#endif


int cptr_ctrl (
 struct cptr_vectored_thrust_input_t *action      ,
 struct cptr_sp_t                    *sp          ,
 struct cptr_state_t                 *state       ,
 struct cptr_ctrl_state_t            *ctrl_state  ,
 struct copter_model_t               *cptr_model  ,
 struct cptr_ctrl_params_t           *params      );

#ifdef __cplusplus
 }
#endif


#endif

