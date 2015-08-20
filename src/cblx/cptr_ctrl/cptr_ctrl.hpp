#ifndef _CPTR_CTRL_H_
#define _CPTR_CTRL_H_

#include <iostream>
#include <armadillo>
#include "../../ctypes/ctype_cptr_state.h"
#include "../../ctypes/ctype_cptr_ctrlaction.h"
#include "../../ctypes/ctype_cptr_model.h"
#include "../../ctypes/ctype_cptr_sp.h"
#include "../../ctypes/ctype_cptr_ctrl_state.h"
#include "../../ctypes/ctype_cptr_ctrl_params.h"

#include "../../lib/arma_quaternion_lib.hpp"
#include "../../lib/cptr_ctrl_aux_fun.hpp"

#define UBX_NO_DEBUG
//#define ARMA_NO_DEBUG

#ifdef __cplusplus
 extern "C" {
#endif


int cptr_ctrl (
 struct cptr_ctrlaction_t            *action      ,
 struct cptr_sp_t                    *sp          ,
 struct cptr_state_t                 *state       ,
 struct cptr_ctrl_state_t            *ctrl_state  ,
 struct cptr_model_t                 *cptr_model  ,
 struct cptr_ctrl_params_t           *params      );


#ifdef __cplusplus
 }
#endif

#endif

