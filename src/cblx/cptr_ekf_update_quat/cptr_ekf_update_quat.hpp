#ifndef _CPTR_EKF_UPDATE_H_
#define _CPTR_EKF_UPDATE_H_

#include <iostream>
#include <ctime>
#include <armadillo>
#include "../../ctypes/ctype_cptr_state.h"
#include "../../ctypes/ctype_cptr_sensdata.h"
#include "../../ctypes/ctype_cptr_obsmodel.h"
#include "../../lib/arma_quaternion_lib.cpp"

#define UBX_NO_DEBUG
//#define ARMA_NO_DEBUG

#ifdef __cplusplus
 extern "C" {
#endif


int cptr_ekf_update_vec4d (
 struct state_t                      *state_old      ,
 struct state_t                      *state_new      ,
 struct obsmodel_state_to_vec4d_t    *obsmodel       ,
 struct sensdata_t                   *sensdata       );

#ifdef __cplusplus
 }
#endif


#endif

