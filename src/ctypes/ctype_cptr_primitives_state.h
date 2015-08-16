#ifndef _CTYPE_CPTR_PRIMITIVES_STATE_
#define _CTYPE_CPTR_PRIMITIVES_STATE_

#include "ctype_cptr_primitives_vecs.h"
#include "ctype_cptr_primitives_mats.h"

struct cptr_state_lin_pos_t {
  struct cptr_vec3d_t     vec             ;
  struct cptr_vec3d_vec3d_crosscov_t  cov ;
  char *type                   ;
};

struct cptr_state_lin_vel_t {
  struct cptr_vec3d_t     vec        ;
  struct cptr_vec3d_vec3d_crosscov_t  cov ;
  char *type             ;
};

struct cptr_state_ang_pos_t {
  struct cptr_vec4d_t      vec ;
  struct cptr_vec4d_vec4d_crosscov_t   cov ;
  char *type      ;
};

struct cptr_state_ang_vel_t {
  struct cptr_vec4d_t      vec ;
  struct cptr_vec4d_vec4d_crosscov_t   cov ;
  char *type      ;
};

#endif

