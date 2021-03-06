#ifndef _CTYPE_CPTR_STATE_
#define _CTYPE_CPTR_STATE_

#include "ctype_cptr_primitives_vecs.h"
#include "ctype_cptr_primitives_mats.h"

struct cptr_state_t {
  struct cptr_vec3d_t linpos ;
  struct cptr_vec3d_t linvel ;
  struct cptr_vec3d_t angpos ;
  struct cptr_vec3d_t angvel ;
  
  struct cptr_vec3d_vec3d_crosscov_t linpos_linpos_crosscov  ;
  struct cptr_vec3d_vec3d_crosscov_t linpos_linvel_crosscov  ;
  struct cptr_vec3d_vec4d_crosscov_t linpos_angpos_crosscov  ;
  struct cptr_vec3d_vec4d_crosscov_t linpos_angvel_crosscov  ;
  
  struct cptr_vec3d_vec3d_crosscov_t linvel_linvel_crosscov  ;
  struct cptr_vec3d_vec4d_crosscov_t linvel_angpos_crosscov  ;
  struct cptr_vec3d_vec4d_crosscov_t linvel_angvel_crosscov  ;

  struct cptr_vec4d_vec4d_crosscov_t angpos_angpos_crosscov  ;
  struct cptr_vec4d_vec4d_crosscov_t angpos_angvel_crosscov  ;

  struct cptr_vec4d_vec4d_crosscov_t angvel_angvel_crosscov  ;

  char*  type;
  
};

#endif

