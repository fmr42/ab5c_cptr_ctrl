#ifndef _CTYPE_CPTR_STATE_
#define _CTYPE_CPTR_STATE_

#include "ctype_cptr_primitives.h"

struct cptr_state_t {
  struct cptr_state_lin_pos_t     cptr_state_lin_pos       ;
  struct cptr_state_lin_vel_t     cptr_state_lin_vel       ;
  struct cptr_state_ang_pos_t     cptr_state_ang_pos       ;
  struct cptr_state_ang_vel_t     cptr_state_ang_vel       ;
  const char*  type;
  
};

#endif

