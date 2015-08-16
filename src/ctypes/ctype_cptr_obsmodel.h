#ifndef _CTYPE_CPTR_OBSMODEL_
#define _CTYPE_CPTR_OBSMODEL_


#include "ctype_cptr_primitives_mats.h"


struct cptr_obsmodel_state_to_vec4d_t {
  struct cptr_obsmodel_vec3d_to_vec4d_t linpos_to_vec4d ;
  struct cptr_obsmodel_vec3d_to_vec4d_t linvel_to_vec4d ;
  struct cptr_obsmodel_vec4d_to_vec4d_t angpos_to_vec4d ;
  struct cptr_obsmodel_vec4d_to_vec4d_t angvel_to_vec4d ;
  char   *type ;
}

struct cptr_obsmodel_state_to_vec3d_t {
  struct cptr_obsmodel_vec3d_to_vec3d_t linpos_to_vec3d ;
  struct cptr_obsmodel_vec3d_to_vec3d_t linvel_to_vec3d ;
  struct cptr_obsmodel_vec4d_to_vec3d_t angpos_to_vec3d ;
  struct cptr_obsmodel_vec4d_to_vec3d_t angvel_to_vec3d ;
  char   *type ;
}


#endif

